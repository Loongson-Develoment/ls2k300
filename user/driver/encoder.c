#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
 
#define PROC_NAME "encoder_count"

static int irq_a;
static int irq_b;
static int gpio_a = 62;
static int gpio_b = 63;
static int last_state;

static atomic64_t pulse_count = ATOMIC64_INIT(0);
static struct proc_dir_entry *encoder_proc;
static DEFINE_SPINLOCK(encoder_lock);

module_param(gpio_a, int, 0444);
MODULE_PARM_DESC(gpio_a, "encoder A phase GPIO number");
module_param(gpio_b, int, 0444);
MODULE_PARM_DESC(gpio_b, "encoder B phase GPIO number");
module_param_named(gpio, gpio_a, int, 0444);
MODULE_PARM_DESC(gpio, "compat alias for gpio_a");

static int encoder_read_state(void) {
    int a = gpio_get_value(gpio_a) ? 1 : 0;
    int b = gpio_get_value(gpio_b) ? 1 : 0;

    return (a << 1) | b;
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id) {
    static const s8 quad_table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0,
    };
    unsigned long flags;
    int current_state;
    int index;
    s8 delta;

    spin_lock_irqsave(&encoder_lock, flags);
    current_state = encoder_read_state();
    index = (last_state << 2) | current_state;
    delta = quad_table[index];
    if (delta)
        atomic64_add(delta, &pulse_count);
    last_state = current_state;
    spin_unlock_irqrestore(&encoder_lock, flags);

    return IRQ_HANDLED;
}

static int encoder_count_show(struct seq_file *m, void *v) {
    seq_printf(m, "%lld\n", (long long)atomic64_read(&pulse_count));
    return 0;
}

static int encoder_count_open(struct inode *inode, struct file *file) {
    return single_open(file, encoder_count_show, NULL);
}

static ssize_t encoder_count_write(struct file *file, const char __user *buf,
                                   size_t count, loff_t *ppos) {
    atomic64_set(&pulse_count, 0);
    return count;
}

static const struct file_operations encoder_count_fops = {
    .owner = THIS_MODULE,
    .open = encoder_count_open,
    .read = seq_read,
    .write = encoder_count_write,
    .llseek = seq_lseek,
    .release = single_release,
};

static int __init encoder_driver_init(void) {
    int ret;

    if (gpio_a == gpio_b) {
        printk(KERN_ERR "Encoder GPIO A and B must be different\n");
        return -EINVAL;
    }

    ret = gpio_request(gpio_a, "encoder_a");
    if (ret) {
        printk(KERN_ERR "Failed to request GPIO A %d\n", gpio_a);
        return ret;
    }

    ret = gpio_request(gpio_b, "encoder_b");
    if (ret) {
        printk(KERN_ERR "Failed to request GPIO B %d\n", gpio_b);
        gpio_free(gpio_a);
        return ret;
    }

    ret = gpio_direction_input(gpio_a);
    if (ret) {
        printk(KERN_ERR "Failed to set GPIO A %d as input\n", gpio_a);
        goto err_free_gpio_b;
    }

    ret = gpio_direction_input(gpio_b);
    if (ret) {
        printk(KERN_ERR "Failed to set GPIO B %d as input\n", gpio_b);
        goto err_free_gpio_b;
    }

    last_state = encoder_read_state();

    irq_a = gpio_to_irq(gpio_a);
    if (irq_a < 0) {
        printk(KERN_ERR "Failed to get IRQ number for GPIO A %d\n", gpio_a);
        ret = irq_a;
        goto err_free_gpio_b;
    }

    irq_b = gpio_to_irq(gpio_b);
    if (irq_b < 0) {
        printk(KERN_ERR "Failed to get IRQ number for GPIO B %d\n", gpio_b);
        ret = irq_b;
        goto err_free_gpio_b;
    }

    ret = request_irq(irq_a, gpio_irq_handler,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                      "encoder_a_irq", &irq_a);
    if (ret) {
        printk(KERN_ERR "Failed to request IRQ A %d\n", irq_a);
        goto err_free_gpio_b;
    }

    ret = request_irq(irq_b, gpio_irq_handler,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                      "encoder_b_irq", &irq_b);
    if (ret) {
        printk(KERN_ERR "Failed to request IRQ B %d\n", irq_b);
        goto err_free_irq_a;
    }

    encoder_proc = proc_create(PROC_NAME, 0666, NULL, &encoder_count_fops);
    if (!encoder_proc) {
        printk(KERN_ERR "Failed to create /proc/%s\n", PROC_NAME);
        ret = -ENOMEM;
        goto err_free_irq_b;
    }

    last_state = encoder_read_state();
    atomic64_set(&pulse_count, 0);

    printk(KERN_INFO "Encoder driver loaded: gpio_a=%d irq_a=%d gpio_b=%d irq_b=%d proc=/proc/%s\n",
           gpio_a, irq_a, gpio_b, irq_b, PROC_NAME);
    return 0;

err_free_irq_b:
    free_irq(irq_b, &irq_b);
err_free_irq_a:
    free_irq(irq_a, &irq_a);
err_free_gpio_b:
    gpio_free(gpio_b);
    gpio_free(gpio_a);
    return ret;
}
 
static void __exit encoder_driver_exit(void) {
    remove_proc_entry(PROC_NAME, NULL);
    free_irq(irq_b, &irq_b);
    free_irq(irq_a, &irq_a);
    gpio_free(gpio_b);
    gpio_free(gpio_a);
    printk(KERN_INFO "Encoder driver unloaded\n");
}
 
module_init(encoder_driver_init);
module_exit(encoder_driver_exit);
MODULE_LICENSE("GPL");
