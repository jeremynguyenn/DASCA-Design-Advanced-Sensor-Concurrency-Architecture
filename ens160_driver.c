#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/tasklet.h>
#include <linux/dma-mapping.h>
#include <asm/barrier.h>
#include <linux/kernel.h> // Bổ sung
#include <linux/delay.h> // Bổ sung
#include <linux/err.h> // Bổ sung
#include <linux/printk.h> // Bổ sung

#define DEV_NAME "ens160"
#define ENS160_PART_ID 0x0E
#define ENS160_PART_ID_VAL 0x0160
#define AHT21_ADDR 0x38
#define SHARED_SIZE PAGE_SIZE

union data_union {
    u8 bytes[16];
    struct {
        u8 aqi;
        u16 tvoc;
        u16 eco2;
        float temp;
        float hum;
    } values;
};

static dev_t dev_num;
static struct cdev cdev;
static struct class *cl;
static struct i2c_client *client;
static struct i2c_client *aht_client;
static struct regmap *regmap;
static struct workqueue_struct *wq;
static struct delayed_work sample_work;
static struct tasklet_struct sample_tasklet;
static spinlock_t lock;
static u8 *data_buffer;
static void *shared_buffer;
static dma_addr_t dma_handle;
static wait_queue_head_t data_wait;
static atomic_t data_ready = ATOMIC_INIT(0);
static struct completion sample_complete;
static struct dentry *debug_dir;
static u32 debug_metric = 0;

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    return scnprintf(buf, PAGE_SIZE, "Data ready: %d\n", atomic_read(&data_ready));
}
static DEVICE_ATTR_RO(status);

static void sample_tasklet_func(unsigned long data) {
    complete(&sample_complete);
}

static void sample_data(struct work_struct *work) {
    unsigned long flags;
    int ret;
    u8 status;
    u8 cmd[3] = {0xAC, 0x33, 0x00};
    u8 aht_data[7];
    u8 ens_data[8];
    u8 crc = 0;

    pm_runtime_get_sync(&client->dev);

    spin_lock_irqsave(&lock, flags);
    ret = regmap_write(regmap, 0x10, 0x02); // Standard mode
    if (ret < 0) goto out;
    msleep(50); // Wait
    ret = regmap_read(regmap, 0x20, &status);
    if (ret < 0 || !(status & 0x01)) goto out; // New data?

    ret = regmap_bulk_read(regmap, 0x21, ens_data, 8); // AQI(1), TVOC(2), eCO2(2), etc.
    if (ret < 0) goto out;
    memcpy(data_buffer, ens_data, 8);

    // AHT21
    ret = i2c_master_send(aht_client, cmd, 3);
    if (ret < 3) goto out;
    msleep(80);
    ret = i2c_master_recv(aht_client, aht_data, 7);
    if (ret < 7) goto out;
    if (aht_data[0] & 0x80) goto out; // Busy

    // Bổ sung: CRC check cho AHT21 (datasheet formula: CRC = 0xFF - sum(bytes0-5) + 1)
    for (int i = 0; i < 6; i++) crc += aht_data[i];
    crc = 0xFF - crc + 1;
    if (crc != aht_data[6]) {
        ret = -EIO;
        goto out;
    }

    u32 hum_raw = (aht_data[1] << 12) | (aht_data[2] << 4) | (aht_data[3] >> 4);
    ((union data_union *)data_buffer)->values.hum = (float)hum_raw * 100.0 / 1048576.0;

    u32 temp_raw = ((aht_data[3] & 0x0F) << 16) | (aht_data[4] << 8) | aht_data[5];
    ((union data_union *)data_buffer)->values.temp = (float)temp_raw * 200.0 / 1048576.0 - 50.0;

    __sync_synchronize();

out:
    spin_unlock_irqrestore(&lock, flags);
    if (ret >= 0) {
        atomic_set(&data_ready, 1);
        wake_up_interruptible(&data_wait);
    } else {
        dev_err(&client->dev, "Sample error: %d\n", ret);
    }
    schedule_delayed_work(&sample_work, msecs_to_jiffies(500));
    tasklet_schedule(&sample_tasklet);
    pm_runtime_put(&client->dev);
}

static unsigned int dev_poll(struct file *filp, poll_table *wait) {
    unsigned int mask = 0;
    poll_wait(filp, &data_wait, wait);
    if (atomic_read(&data_ready)) mask |= POLLIN | POLLRDNORM;
    return mask;
}

static int dev_mmap(struct file *filp, struct vm_area_struct *vma) {
    unsigned long start = vma->vm_start;
    unsigned long size = vma->vm_end - vma->vm_start;
    if (size > SHARED_SIZE) return -EINVAL;
    if (remap_pfn_range(vma, start, virt_to_phys(shared_buffer) >> PAGE_SHIFT, size, vma->vm_page_prot)) return -EAGAIN;
    return 0;
}

static ssize_t dev_read(struct file *filp, char __user *buf, size_t count, loff_t *off) {
    unsigned long flags;
    int ret;

    // Bổ sung: Buffer checks (V.22) - Giới hạn count để tránh overflow
    if (count > 16) {
        count = 16; // Max size của union data_union
        pr_warn("Read count capped to 16 bytes to prevent overflow\n");
    }

    wait_for_completion_interruptible(&sample_complete);
    wait_event_interruptible(data_wait, atomic_read(&data_ready));
    atomic_set(&data_ready, 0);

    spin_lock_irqsave(&lock, flags);
    if (copy_to_user(buf, data_buffer, count)) {
        ret = -EFAULT;
    } else {
        ret = count;
    }
    spin_unlock_irqrestore(&lock, flags);
    return ret;
}

static ssize_t dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *off) {
    return -EPERM; // Not implemented
}

static long dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    return -ENOTTY; // Not implemented
}

static int dev_open(struct inode *inode, struct file *filp) {
    return 0;
}

static int dev_release(struct inode *inode, struct file *filp) {
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = dev_read,
    .write = dev_write,
    .unlocked_ioctl = dev_ioctl,
    .open = dev_open,
    .release = dev_release,
    .poll = dev_poll,
    .mmap = dev_mmap,
};

static int probe(struct i2c_client *clnt, const struct i2c_device_id *id) {
    struct regmap_config regmap_config = {
        .reg_bits = 8,
        .val_bits = 8,
    };
    struct i2c_board_info aht_info = {
        I2C_BOARD_INFO("aht21", AHT21_ADDR),
    };
    int ret;
    u16 part_id;

    client = clnt;
    regmap = regmap_init_i2c(client, &regmap_config);
    if (IS_ERR(regmap)) {
        ret = PTR_ERR(regmap);
        goto exit;
    }

    data_buffer = kmalloc(32, GFP_KERNEL);
    if (!data_buffer) {
        ret = -ENOMEM;
        goto err_regmap;
    }

    shared_buffer = dma_alloc_coherent(&clnt->dev, SHARED_SIZE, &dma_handle, GFP_KERNEL);
    if (!shared_buffer) {
        ret = -ENOMEM;
        goto free_buf;
    }

    ret = regmap_read(regmap, ENS160_PART_ID, &part_id);
    if (ret < 0 || part_id != ENS160_PART_ID_VAL) {
        ret = -ENODEV;
        goto free_shared;
    }

    ret = regmap_write(regmap, 0x10, 0x00); // Idle
    if (ret < 0) goto free_shared;

    aht_client = i2c_new_device(clnt->adapter, &aht_info);
    if (!aht_client) {
        ret = -ENODEV;
        goto free_shared;
    }

    init_completion(&sample_complete);
    init_waitqueue_head(&data_wait);
    spin_lock_init(&lock);

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0) goto unreg_aht;

    cdev_init(&cdev, &fops);
    ret = cdev_add(&cdev, dev_num, 1);
    if (ret < 0) goto unreg_chrdev;

    cl = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(cl)) {
        ret = PTR_ERR(cl);
        goto del_cdev;
    }

    ret = device_create(cl, NULL, dev_num, NULL, DEV_NAME);
    if (ret < 0) goto destroy_class;

    debug_dir = debugfs_create_dir(DEV_NAME, NULL);
    if (IS_ERR(debug_dir)) {
        ret = PTR_ERR(debug_dir);
        goto destroy_dev;
    }
    debugfs_create_u32("metric", 0644, debug_dir, &debug_metric);

    ret = device_create_file(&clnt->dev, &dev_attr_status);
    if (ret < 0) goto remove_debug;

    wq = create_singlethread_workqueue("ens_wq");
    if (!wq) {
        ret = -ENOMEM;
        goto remove_sysfs;
    }

    INIT_DELAYED_WORK(&sample_work, sample_data);
    tasklet_init(&sample_tasklet, sample_tasklet_func, 0);
    schedule_delayed_work(&sample_work, 0);

    pm_runtime_enable(&clnt->dev);
    dev_info(&clnt->dev, "ENS160 probed successfully\n");
    return 0;

remove_sysfs:
    device_remove_file(&clnt->dev, &dev_attr_status);
remove_debug:
    debugfs_remove_recursive(debug_dir);
destroy_dev:
    device_destroy(cl, dev_num);
destroy_class:
    class_destroy(cl);
del_cdev:
    cdev_del(&cdev);
unreg_chrdev:
    unregister_chrdev_region(dev_num, 1);
unreg_aht:
    i2c_unregister_device(aht_client);
free_shared:
    dma_free_coherent(&clnt->dev, SHARED_SIZE, shared_buffer, dma_handle);
free_buf:
    kfree(data_buffer);
err_regmap:
    regmap_exit(regmap);
exit:
    return ret;
}

static void remove(struct i2c_client *clnt) {
    pm_runtime_disable(&clnt->dev);
    cancel_delayed_work_sync(&sample_work);
    tasklet_kill(&sample_tasklet);
    destroy_workqueue(wq);
    device_remove_file(&clnt->dev, &dev_attr_status);
    debugfs_remove_recursive(debug_dir);
    device_destroy(cl, dev_num);
    class_destroy(cl);
    cdev_del(&cdev);
    unregister_chrdev_region(dev_num, 1);
    i2c_unregister_device(aht_client);
    dma_free_coherent(&clnt->dev, SHARED_SIZE, shared_buffer, dma_handle);
    kfree(data_buffer);
    regmap_exit(regmap);
    dev_info(&clnt->dev, "ENS160 removed\n");
}

static const struct of_device_id dt_ids[] = {
    { .compatible = "scio-sense,ens160" },
    { }
};

static const struct i2c_device_id i2c_ids[] = {
    { "ens160", 0 },
    { }
};

static struct i2c_driver driver = {
    .driver = {
        .name = DEV_NAME,
        .of_match_table = dt_ids,
    },
    .probe = probe,
    .remove = remove,
    .id_table = i2c_ids,
};

module_i2c_driver(driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ENS160 I2C Driver");
// Testing: libfuzzer for concurrency, integration for fusion
// Optimization: eBPF for bottlenecks, cache coherence in multi-core
// Bổ sung: Có thể thêm RCU cho lock-free reads nếu dùng liburcu (V.15), Kalman filter cho data fusion (V.18)

// Bổ sung tính năng nâng cao: Stub cho eBPF trace (V.21)
#if defined(CONFIG_BPF_SYSCALL)
static int bpf_prog_id = 0; // Placeholder
#endif

// Bổ sung: RCU placeholder (V.15)
#define rcu_read_lock() preempt_disable()
#define rcu_read_unlock() preempt_enable()
// Có thể dùng trong dev_read thay spin_lock nếu optimize
