#Device Driver Development for ethernet functions 
#Goal: Write a character or block device driver with a#Advanced features like RDMA, control functions with host/accelerators.
 #Scope: Implement async I/O handling,  Add mmap support for zero-copy access. 
# Outcomes: Driver entry points, DMA, kernel/user memory mapping.


// src/ethdev.c
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/fasync.h>

#define DEVICE_NAME "ethdev_stub"
#define CLASS_NAME  "ethdev"
#define DMA_BUF_SIZE (PAGE_SIZE * 64) // 256KB

/* Simple IOCTLs for control */
#define ETHDEV_IOC_MAGIC 'E'
#define ETHDEV_IOC_GET_DMA_HANDLE _IOR(ETHDEV_IOC_MAGIC, 1, unsigned long)
#define ETHDEV_IOC_START_TX       _IO(ETHDEV_IOC_MAGIC, 2)
#define ETHDEV_IOC_STOP_TX        _IO(ETHDEV_IOC_MAGIC, 3)

struct ethdev_priv {
    struct device *dev;
    void *dma_virt;         // kernel virtual address for DMA buffer
    dma_addr_t dma_handle;  // DMA address for device
    size_t buf_size;

    /* Simple ring queue for packets (offsets into dma_virt) */
    spinlock_t lock;
    wait_queue_head_t readq;
    size_t data_len;
    bool open;
    struct fasync_struct *fasync;
    struct miscdevice misc;
};

static struct ethdev_priv *gpriv;

/* --- File ops --- */
static int ethdev_open(struct inode *inode, struct file *filp) {
    struct ethdev_priv *p = container_of(filp->private_data = gpriv, struct ethdev_priv, misc);
    if (!p) return -ENODEV;
    p->open = true;
    return 0;
}

static int ethdev_release(struct inode *inode, struct file *filp) {
    struct ethdev_priv *p = filp->private_data;
    p->open = false;
    /* remove async */
    fasync_helper(-1, filp, 0, &p->fasync);
    return 0;
}

/* basic write: user writes packet into driver -> driver copies into DMA buffer (for demo)
   In a real NIC you'd queue the buffer to hardware and kick TX. */
static ssize_t ethdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos) {
    struct ethdev_priv *p = filp->private_data;
    unsigned long flags;
    size_t copylen;

    if (count == 0) return 0;
    if (count > p->buf_size) return -EINVAL;

    if ((filp->f_flags & O_NONBLOCK)) {
        /* non-blocking: if buffer full, return EAGAIN */
        if (p->data_len != 0)
            return -EAGAIN;
    } else {
        /* block until buffer free */
        wait_event_interruptible(p->readq, p->data_len == 0);
    }

    spin_lock_irqsave(&p->lock, flags);
    copylen = min(count, p->buf_size);
    if (copy_from_user(p->dma_virt, buf, copylen)) {
        spin_unlock_irqrestore(&p->lock, flags);
        return -EFAULT;
    }
    p->data_len = copylen;
    spin_unlock_irqrestore(&p->lock, flags);

    /* wake up readers and users polling */
    wake_up_interruptible(&p->readq);
    if (p->fasync)
        kill_fasync(&p->fasync, SIGIO, POLL_IN);

    /* For a real device, you'd pass the DMA address and length to the NIC */
    dev_info(p->dev, "ethdev: wrote %zu bytes -> dma_handle 0x%llx\n", copylen, (unsigned long long)p->dma_handle);
    return copylen;
}

/* basic read: copy from DMA buffer to userland (simulates received packet) */
static ssize_t ethdev_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos) {
    struct ethdev_priv *p = filp->private_data;
    unsigned long flags;
    size_t tocopy;

    if (count == 0) return 0;

    if ((filp->f_flags & O_NONBLOCK)) {
        if (p->data_len == 0)
            return -EAGAIN;
    } else {
        if (wait_event_interruptible(p->readq, p->data_len > 0))
            return -ERESTARTSYS;
    }

    spin_lock_irqsave(&p->lock, flags);
    if (p->data_len == 0) {
        spin_unlock_irqrestore(&p->lock, flags);
        return 0;
    }
    tocopy = min(count, p->data_len);
    if (copy_to_user(buf, p->dma_virt, tocopy)) {
        spin_unlock_irqrestore(&p->lock, flags);
        return -EFAULT;
    }
    p->data_len = 0; // consumed
    spin_unlock_irqrestore(&p->lock, flags);

    wake_up_interruptible(&p->readq); // wake writers waiting for empty space
    return tocopy;
}

/* poll/select support */
static unsigned int ethdev_poll(struct file *filp, poll_table *wait) {
    struct ethdev_priv *p = filp->private_data;
    unsigned int mask = 0;

    poll_wait(filp, &p->readq, wait);
    if (p->data_len > 0)
        mask |= POLLIN | POLLRDNORM;
    else
        mask |= POLLOUT | POLLWRNORM; /* we have space for writing */

    return mask;
}

/* mmap: expose the DMA-coherent buffer to user space
   We map the physical pages that back the coherent allocation.
   NOTE: behavior varies by arch; coherent memory is physically contiguous on many systems.
*/
static int ethdev_mmap(struct file *filp, struct vm_area_struct *vma) {
    struct ethdev_priv *p = filp->private_data;
    unsigned long vsize = vma->vm_end - vma->vm_start;
    unsigned long pfn;
    void *kaddr = p->dma_virt;
    phys_addr_t phys;

    if (vsize > p->buf_size) return -EINVAL;

    /* convert kernel virtual to physical frame number */
    phys = virt_to_phys(kaddr);
    pfn = (unsigned long)(phys >> PAGE_SHIFT);

    /* Remap physical pages into user */
    if (remap_pfn_range(vma, vma->vm_start, pfn, vsize, vma->vm_page_prot))
        return -EAGAIN;

    return 0;
}

/* ioctl: return DMA handle so userland (or RDMA library) can know the device DMA address */
static long ethdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct ethdev_priv *p = filp->private_data;
    switch (cmd) {
    case ETHDEV_IOC_GET_DMA_HANDLE:
        if (copy_to_user((unsigned long __user *)arg, &p->dma_handle, sizeof(p->dma_handle)))
            return -EFAULT;
        return 0;
    case ETHDEV_IOC_START_TX:
        dev_info(p->dev, "ethdev: START_TX (stub)\n");
        return 0;
    case ETHDEV_IOC_STOP_TX:
        dev_info(p->dev, "ethdev: STOP_TX (stub)\n");
        return 0;
    default:
        return -ENOTTY;
    }
}

/* fasync for SIGIO */
static int ethdev_fasync(int fd, struct file *filp, int mode) {
    struct ethdev_priv *p = filp->private_data;
    return fasync_helper(fd, filp, mode, &p->fasync);
}

static const struct file_operations ethdev_fops = {
    .owner = THIS_MODULE,
    .open = ethdev_open,
    .release = ethdev_release,
    .read = ethdev_read,
    .write = ethdev_write,
    .poll = ethdev_poll,
    .mmap = ethdev_mmap,
    .unlocked_ioctl = ethdev_ioctl,
    .fasync = ethdev_fasync,
};

/* Module init/exit - allocate DMA buffer and register device */
static int __init ethdev_init(void) {
    int ret;
    struct ethdev_priv *p = kzalloc(sizeof(*p), GFP_KERNEL);
    if (!p) return -ENOMEM;

    p->buf_size = DMA_BUF_SIZE;
    spin_lock_init(&p->lock);
    init_waitqueue_head(&p->readq);

    
    /* allocate coherent DMA memory */
    /* Use DMA-capable device pointer normally (from PCI probe) - for this demo use NULL device and hope coherent works */
    p->dma_virt = dma_alloc_coherent(NULL, p->buf_size, &p->dma_handle, GFP_KERNEL);
    if (!p->dma_virt) {
        dev_err(NULL, "ethdev: dma_alloc_coherent failed\n");
        ret = -ENOMEM;
        goto err_alloc;
    }

    p->misc.minor = MISC_DYNAMIC_MINOR;
    p->misc.name = DEVICE_NAME;
    p->misc.fops = &ethdev_fops;
    p->misc.parent = NULL;

    ret = misc_register(&p->misc);
    if (ret) {
        dev_err(NULL, "ethdev: misc_register failed\n");
        goto err_misc;
    }

    p->dev = p->misc.this_device;
    gpriv = p;

    pr_info("ethdev: module loaded. Device /dev/%s, dma_handle=0x%llx, vaddr=%p\n",
            DEVICE_NAME, (unsigned long long)p->dma_handle, p->dma_virt);

    return 0;

err_misc:
    dma_free_coherent(NULL, p->buf_size, p->dma_virt, p->dma_handle);
err_alloc:
    kfree(p);
    return ret;
}

static void __exit ethdev_exit(void) {
    struct ethdev_priv *p = gpriv;
    if (!p) return;

    misc_deregister(&p->misc);
    dma_free_coherent(NULL, p->buf_size, p->dma_virt, p->dma_handle);
    kfree(p);
    gpriv = NULL;
    pr_info("ethdev: module unloaded\n");
}

module_init(ethdev_init);
module_exit(ethdev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Example");
MODULE_DESCRIPTION("Example ethernet-like device: char dev, DMA, mmap, async I/O");
