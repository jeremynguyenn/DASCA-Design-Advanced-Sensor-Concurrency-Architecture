#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
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
#include <linux/kernel.h> // Bổ sung: Cho dev_err/dev_info
#include <linux/delay.h> // Bổ sung: Cho msleep
#include <linux/err.h> // Bổ sung: Cho IS_ERR/PTR_ERR

#define DEV_NAME "mpu9250"
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_WHO_AM_I_VAL 0x71
#define AK8963_ADDR 0x0C
#define AK8963_WHO_AM_I 0x00
#define AK8963_WHO_AM_I_VAL 0x48
#define SHARED_SIZE PAGE_SIZE

enum sensor_type {
    ACCEL, GYRO, MAG
};

union data_union {
    u8 bytes[20];
    struct {
        s16 accel[3];
        s16 temp;
        s16 gyro[3];
        s16 mag[3];
    } values;
};

static dev_t dev_num;
static struct cdev cdev;
static struct class *cl;
static struct i2c_client *client;
static struct i2c_client *mag_client;
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
static int irq = -1;
static struct dentry *debug_dir;
static u32 debug_metric = 0; // Example metric for debugfs

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    return scnprintf(buf, PAGE_SIZE, "Data ready: %d\n", atomic_read(&data_ready));
}
static DEVICE_ATTR_RO(status);

// Comment: Tasklet func cho bottom half, completion cho kernel synchronization (roadmap V.16)
static void sample_tasklet_func(unsigned long data) {
    complete(&sample_complete);
}

// Comment: Sample data với workqueue/timer-based sampling (roadmap V.18), spinlock cho critical section (V.16), memory barrier (I.1)
static void sample_data(struct work_struct *work) {
    unsigned long flags;
    int ret;
    u8 st2;
    u8 mag_data[7];

    pm_runtime_get_sync(&client->dev);

    spin_lock_irqsave(&lock, flags);
    ret = regmap_bulk_read(regmap, 0x3B, data_buffer, 14);
    if (ret < 0) goto out;

    ret = i2c_smbus_write_byte_data(mag_client, 0x0A, 0x16);
    if (ret < 0) goto out;
    msleep(10); // Wait for measurement
    ret = i2c_smbus_read_i2c_block_data(mag_client, 0x03, 7, mag_data);
    if (ret < 7) goto out;
    st2 = mag_data[6];
    if (st2 & 0x08) { // Overflow check
        ret = -EOVERFLOW;
        goto out;
    }
    memcpy(data_buffer + 14, mag_data, 6);
    __sync_synchronize();

out:
    spin_unlock_irqrestore(&lock, flags);
    if (ret == 0) {
        atomic_set(&data_ready, 1);
        wake_up_interruptible(&data_wait);
    } else {
        // Bổ sung: Error handling đầy đủ
        dev_err(&client->dev, "Sample error: %d\n", ret);
    }
    schedule_delayed_work(&sample_work, msecs_to_jiffies(100));
    tasklet_schedule(&sample_tasklet);
    pm_runtime_put(&client->dev);
}

// Comment: IRQ handler cho interrupt handling/deferred work (V.16)
static irqreturn_t mpu_irq_handler(int irq, void *dev_id) {
    queue_delayed_work(wq, &sample_work, 0);
    return IRQ_HANDLED;
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
    if (remap_pfn_range(vma, start, dma_handle >> PAGE_SHIFT, size, vma->vm_page_prot)) return -EAGAIN;
    return 0;
}

static ssize_t dev_read(struct file *filp, char __user *buf, size_t count, loff_t *off) {
    unsigned long flags;
    int ret;

    // Bổ sung: Buffer checks (V.22) - Giới hạn count để tránh overflow
    if (count > 20) {
        count = 20; // Max size của union data_union
        pr_warn("Read count capped to 20 bytes to prevent overflow\n");
    }

    if (wait_event_interruptible(data_wait, atomic_read(&data_ready))) return -ERESTARTSYS;
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
    struct i2c_board_info mag_info = {
        I2C_BOARD_INFO("ak8963", AK8963_ADDR),
    };
    int ret;
    u8 who;
    u8 id;

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

    ret = regmap_read(regmap, MPU9250_WHO_AM_I, &id);
    if (ret < 0 || id != MPU9250_WHO_AM_I_VAL) {
        ret = -ENODEV;
        goto free_shared;
    }

    ret = regmap_write(regmap, 0x6B, 0x80); // Reset
    if (ret < 0) goto free_shared;
    msleep(10);
    ret = regmap_write(regmap, 0x6B, 0x01); // Clock source
    if (ret < 0) goto free_shared;
    ret = regmap_write(regmap, 0x1A, 0x03); // DLPF
    if (ret < 0) goto free_shared;
    ret = regmap_write(regmap, 0x1B, 0x18); // Gyro FS 2000 dps
    if (ret < 0) goto free_shared;
    ret = regmap_write(regmap, 0x1C, 0x18); // Accel FS 16g
    if (ret < 0) goto free_shared;
    ret = regmap_write(regmap, 0x37, 0x02); // Bypass mode for mag
    if (ret < 0) goto free_shared;

    mag_client = i2c_new_device(clnt->adapter, &mag_info);
    if (!mag_client) {
        ret = -ENODEV;
        goto free_shared;
    }

    who = i2c_smbus_read_byte_data(mag_client, AK8963_WHO_AM_I);
    if (who != AK8963_WHO_AM_I_VAL) {
        ret = -ENODEV;
        i2c_unregister_device(mag_client);
        goto free_shared;
    }

    init_completion(&sample_complete);
    init_waitqueue_head(&data_wait);
    spin_lock_init(&lock);

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0) {
        i2c_unregister_device(mag_client);
        goto free_shared;
    }

    cdev_init(&cdev, &fops);
    ret = cdev_add(&cdev, dev_num, 1);
    if (ret < 0) goto unreg_chrdev;

    cl = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(cl)) {
        ret = PTR_ERR(cl);
        goto del_cdev;
    }

    ret = device_create(cl, NULL, dev_num, NULL, DEV_NAME);
    if (ret < 0) {
        goto destroy_class;
    }

    debug_dir = debugfs_create_dir(DEV_NAME, NULL);
    if (IS_ERR(debug_dir)) {
        ret = PTR_ERR(debug_dir);
        goto destroy_dev;
    }
    debugfs_create_u32("metric", 0644, debug_dir, &debug_metric);

    ret = device_create_file(&clnt->dev, &dev_attr_status);
    if (ret < 0) goto remove_debug;

    wq = create_singlethread_workqueue("mpu_wq");
    if (!wq) {
        ret = -ENOMEM;
        goto remove_sysfs;
    }

    INIT_DELAYED_WORK(&sample_work, sample_data);
    tasklet_init(&sample_tasklet, sample_tasklet_func, 0);

    if (clnt->irq) {
        irq = clnt->irq;
        ret = request_irq(irq, mpu_irq_handler, IRQF_TRIGGER_RISING, DEV_NAME, clnt);
        if (ret < 0) goto destroy_wq;
    } else {
        schedule_delayed_work(&sample_work, 0);
    }

    pm_runtime_enable(&clnt->dev);
    // Bổ sung: Logging thành công
    dev_info(&clnt->dev, "MPU9250 probed successfully\n");
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
    if (irq > 0) free_irq(irq, clnt);
    cancel_delayed_work_sync(&sample_work);
    tasklet_kill(&sample_tasklet);
    destroy_workqueue(wq);
    device_remove_file(&clnt->dev, &dev_attr_status);
    debugfs_remove_recursive(debug_dir);
    device_destroy(cl, dev_num);
    class_destroy(cl);
    cdev_del(&cdev);
    unregister_chrdev_region(dev_num, 1);
    i2c_unregister_device(mag_client);
    dma_free_coherent(&clnt->dev, SHARED_SIZE, shared_buffer, dma_handle);
    kfree(data_buffer);
    regmap_exit(regmap);
    // Bổ sung: Logging
    dev_info(&clnt->dev, "MPU9250 removed\n");
}

static const struct of_device_id dt_ids[] = {
    { .compatible = "invensense,mpu9250" },
    { }
};

static const struct i2c_device_id i2c_ids[] = {
    { "mpu9250", 0 },
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
MODULE_DESCRIPTION("MPU9250 I2C Driver");
// For testing: Use Google Test for unit tests, stress-ng for races, perf for profiling
// Security: ASLR enabled, buffer checks in read
// Deployment: Yocto for image, Rust alternative for safer concurrency
// Bổ sung: Buffer checks in read (V.22), eBPF cho profiling (V.21)

// Bổ sung tính năng nâng cao: Stub cho eBPF (V.21)
#if defined(CONFIG_BPF_SYSCALL)
static int bpf_prog_id = 0; // Placeholder cho attach BPF đến sample_data
#endif

// Bổ sung: RCU placeholder (V.15)
#define rcu_read_lock() preempt_disable()
#define rcu_read_unlock() preempt_enable()
