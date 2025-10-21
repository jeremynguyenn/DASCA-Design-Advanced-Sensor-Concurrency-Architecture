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
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <asm/barrier.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/printk.h>

#define DEV_NAME "bme280"
#define BME280_ID_REG 0xD0
#define BME280_ID_VAL 0x60
#define SHARED_SIZE PAGE_SIZE

struct bme280_calib {
    u16 dig_T1;
    s16 dig_T2, dig_T3;
    u16 dig_P1;
    s16 dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    u8 dig_H1;
    s16 dig_H2;
    u8 dig_H3;
    s16 dig_H4, dig_H5;
    s8 dig_H6;
};

union data_union {
    u8 bytes[24]; // Raw + compensated T/P/H (4 bytes each float approx, but use s32)
    struct {
        s32 temp;
        u32 press;
        u32 hum;
    } compensated;
};

static dev_t dev_num;
static struct cdev cdev;
static struct class *cl;
static struct i2c_client *client;
static struct regmap *regmap;
static struct workqueue_struct *wq;
static struct delayed_work sample_work;
static spinlock_t lock;
static u8 *data_buffer;
static void *shared_buffer;
static dma_addr_t dma_handle;
static struct bme280_calib calib;
static s32 t_fine;
static wait_queue_head_t data_wait;
static atomic_t data_ready = ATOMIC_INIT(0);
static struct dentry *debug_dir;
static u32 debug_metric = 0;

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf) {
    return scnprintf(buf, PAGE_SIZE, "Data ready: %d\n", atomic_read(&data_ready));
}
static DEVICE_ATTR_RO(status);

static s32 compensate_temp(s32 adc_T) {
    s32 var1 = (((adc_T >> 3) - ((s32)calib.dig_T1 << 1)) * ((s32)calib.dig_T2)) >> 11;
    s32 var2 = (((((adc_T >> 4) - ((s32)calib.dig_T1)) * ((adc_T >> 4) - ((s32)calib.dig_T1))) >> 12) * ((s32)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

static u32 compensate_press(s32 adc_P) {
    s64 var1 = ((s64)t_fine) - 128000;
    s64 var2 = var1 * var1 * (s64)calib.dig_P6;
    var2 = var2 + ((var1 * (s64)calib.dig_P5) << 17);
    var2 = var2 + (((s64)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (s64)calib.dig_P3) >> 8) + ((var1 * (s64)calib.dig_P2) << 12);
    var1 = (((((s64)1) << 47) + var1)) * ((s64)calib.dig_P1) >> 33;
    if (var1 == 0) return 0;
    s64 p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((s64)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((s64)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((s64)calib.dig_P7) << 4);
    return (u32)p;
}

static u32 compensate_hum(s32 adc_H) {
    s32 v_x1_u32r = (t_fine - ((s32)76800));
    v_x1_u32r = (((((adc_H << 14) - (((s32)calib.dig_H4) << 20) - (((s32)calib.dig_H5) * v_x1_u32r)) + ((s32)16384)) >> 15) * (((((((v_x1_u32r * ((s32)calib.dig_H6)) >> 10) * (((v_x1_u32r * ((s32)calib.dig_H3)) >> 11) + ((s32)32768))) >> 10) + ((s32)2097152)) * ((s32)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((s32)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    return (u32)(v_x1_u32r >> 12);
}

static void sample_data(struct work_struct *work) {
    unsigned long flags;
    int ret;
    u8 calib_data1[24];
    u8 calib_data2[7];
    u8 data[8];
    s32 adc_T, adc_P, adc_H;

    pm_runtime_get_sync(&client->dev);

    spin_lock_irqsave(&lock, flags);

    // Read calib nếu cần (e.g., lazy load calib nếu chưa có)
    if (!calib.dig_T1) {
        ret = regmap_bulk_read(regmap, 0x88, calib_data1, 24);
        if (ret < 0) goto out;
        ret = regmap_bulk_read(regmap, 0xE1, calib_data2, 7);
        if (ret < 0) goto out;

        calib.dig_T1 = (u16)((calib_data1[1] << 8) | calib_data1[0]);
        calib.dig_T2 = (s16)((calib_data1[3] << 8) | calib_data1[2]);
        calib.dig_T3 = (s16)((calib_data1[5] << 8) | calib_data1[4]);
        calib.dig_P1 = (u16)((calib_data1[7] << 8) | calib_data1[6]);
        calib.dig_P2 = (s16)((calib_data1[9] << 8) | calib_data1[8]);
        calib.dig_P3 = (s16)((calib_data1[11] << 8) | calib_data1[10]);
        calib.dig_P4 = (s16)((calib_data1[13] << 8) | calib_data1[12]);
        calib.dig_P5 = (s16)((calib_data1[15] << 8) | calib_data1[14]);
        calib.dig_P6 = (s16)((calib_data1[17] << 8) | calib_data1[16]);
        calib.dig_P7 = (s16)((calib_data1[19] << 8) | calib_data1[18]);
        calib.dig_P8 = (s16)((calib_data1[21] << 8) | calib_data1[20]);
        calib.dig_P9 = (s16)((calib_data1[23] << 8) | calib_data1[22]);
        calib.dig_H1 = calib_data2[0];
        calib.dig_H2 = (s16)((calib_data2[2] << 8) | calib_data2[1]);
        calib.dig_H3 = calib_data2[3];
        calib.dig_H4 = (s16)((calib_data2[4] << 4) | (calib_data2[5] & 0x0F));
        calib.dig_H5 = (s16)((calib_data2[6] << 4) | ((calib_data2[5] >> 4) & 0x0F));
        calib.dig_H6 = (s8)calib_data2[7];
    }

    ret = regmap_write(regmap, 0xF2, 0x05); // hum x16
    if (ret < 0) goto out;
    ret = regmap_write(regmap, 0xF4, 0xBF); // temp/press x16, normal mode
    if (ret < 0) goto out;
    msleep(100);

    ret = regmap_bulk_read(regmap, 0xF7, data, 8);
    if (ret < 0) goto out;

    adc_P = (s32)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    adc_T = (s32)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
    adc_H = (s32)((data[6] << 8) | data[7]);

    ((union data_union *)data_buffer)->compensated.temp = compensate_temp(adc_T);
    ((union data_union *)data_buffer)->compensated.press = compensate_press(adc_P);
    ((union data_union *)data_buffer)->compensated.hum = compensate_hum(adc_H);

    smp_wmb();  // Memory barrier trước set data_ready

out:
    spin_unlock_irqrestore(&lock, flags);
    if (ret >= 0) {
        atomic_set(&data_ready, 1);
        wake_up_interruptible(&data_wait);
    } else {
        dev_err(&client->dev, "Sample error: %d\n", ret);
    }
    schedule_delayed_work(&sample_work, msecs_to_jiffies(500));
    pm_runtime_put(&client->dev);
}

static unsigned int dev_poll(struct file *filp, poll_table *wait) {
    unsigned int mask = 0;
    poll_wait(filp, &data_wait, wait);
    if (atomic_read(&data_ready)) mask |= POLLIN | POLLRDNORM;
    return mask;
}

static int dev_mmap(struct file *filp, struct vm_area_struct *vma) {
    unsigned long size = vma->vm_end - vma->vm_start;
    if (size != SHARED_SIZE || !(vma->vm_flags & VM_SHARED)) return -EINVAL;
    return dma_mmap_coherent(&client->dev, vma, shared_buffer, dma_handle, size);
}

static ssize_t dev_read(struct file *filp, char __user *buf, size_t count, loff_t *off) {
    unsigned long flags;
    int ret;

    if (count > 12) {
        count = 12; // Max size compensated (3 * 4 bytes)
        pr_warn("Read count capped to 12 bytes to prevent overflow\n");
    }

    if (wait_event_interruptible(data_wait, atomic_read(&data_ready))) return -ERESTARTSYS;
    atomic_set(&data_ready, 0);

    spin_lock_irqsave(&lock, flags);
    ret = copy_to_user(buf, data_buffer, count) ? -EFAULT : count;
    spin_unlock_irqrestore(&lock, flags);
    return ret;
}

static const struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = dev_read,
    .poll = dev_poll,
    .mmap = dev_mmap,
};

static int probe(struct i2c_client *clnt, const struct i2c_device_id *id) {
    struct regmap_config regmap_config = {
        .reg_bits = 8,
        .val_bits = 8,
    };
    int ret;
    unsigned int chip_id;

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

    ret = regmap_read(regmap, BME280_ID_REG, &chip_id);
    if (ret < 0 || chip_id != BME280_ID_VAL) {
        ret = -ENODEV;
        goto free_shared;
    }

    // Reset
    ret = regmap_write(regmap, 0xE0, 0xB6);
    if (ret < 0) goto free_shared;
    msleep(10);

    init_waitqueue_head(&data_wait);
    spin_lock_init(&lock);

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
    if (ret < 0) goto free_shared;

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

    wq = create_singlethread_workqueue("bme_wq");
    if (!wq) {
        ret = -ENOMEM;
        goto remove_sysfs;
    }

    INIT_DELAYED_WORK(&sample_work, sample_data);
    schedule_delayed_work(&sample_work, 0);

    pm_runtime_enable(&clnt->dev);
    dev_info(&clnt->dev, "BME280 probed successfully\n");
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
    cancel_delayed_work_sync(&sample_work);
    destroy_workqueue(wq);
    device_remove_file(&clnt->dev, &dev_attr_status);
    debugfs_remove_recursive(debug_dir);
    device_destroy(cl, dev_num);
    class_destroy(cl);
    cdev_del(&cdev);
    unregister_chrdev_region(dev_num, 1);
    dma_free_coherent(&clnt->dev, SHARED_SIZE, shared_buffer, dma_handle);
    kfree(data_buffer);
    if (!IS_ERR(regmap)) regmap_exit(regmap);
    dev_info(&clnt->dev, "BME280 removed\n");
}

static const struct of_device_id dt_ids[] = {
    { .compatible = "bosch,bme280" },
    { }
};

static const struct i2c_device_id i2c_ids[] = {
    { "bme280", 0 },
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
MODULE_DESCRIPTION("BME280 I2C Driver");
// Testing: Google Test for compensation funcs, stress-ng for concurrency
// Security: AppArmor for IPC, secure boot for RPi
// Future: Rust kernel module for safer concurrency vs FreeRTOS
// Buffer checks in read (V.22), eBPF cho bottlenecks (V.21)