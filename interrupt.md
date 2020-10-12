## Definition

### Traditional (physical machine) interrupt

The interrupt is sent from a device and sent to IOAPIC. IOAPIC checks the PRT table to find the corresponding entry PTE, and knows the target LAPIC. So the interrupt message is formatted and sent to LAPIC, and the remote irr is set to 1 (level) in the notification.

After LAPIC receives the interrupt message, it sets the IRR according to the vector number, and then selects the interrupt. After obtaining the interrupt with the highest priority, clears the IRR, sets the ISR, and submits it to the CPU for interrupt processing. After the CPU processes the interrupt, writes the LAPIC EOI. Notify IOAPIC to clear remote irr (level and deassert).

### QEMU+KVM (Virtual Machine) Interrupt

#### Interrupt exit

When the virtual machine is interrupted, take the initiative to make the guest VMEXIT, so that interrupt injection can be performed before VMENTRY.

#### Interrupt injection

By writing the interrupt to the VM-Entry interruption-infomation field of VMCS, the interrupt can be injected into the guest.

#### VMCS
Defined in VM-EXECUTION CONTROL FIELDS of SDM 3 24.6:

* Pin-Based VM-Execution Controls
    Responsible for controlling whether VMExit occurs during External-interrupt / NMI / Virtual NMIs, and return to KVM.

    For example, if External-interrupt exiting is set to 1, all external interrupts will generate VMExit, otherwise the VM will handle it by itself

* Secondary Processor-Based VM-Execution Controls - virtual-interrupt delivery
    Set to 1, when VM entry/TPR virtualization/EOI virtualization/self-IPI virtualization/posted-interrupt processing will trigger evaluate pending interrupt

Since the VMCS-Secondary Processor-Based VM-Execution Controls-virtualize APIC accesses bit is set, by setting the specific VM-execution controls bit, VMEXIT is generated when the VM accesses the page corresponding to the APIC.

When the interrupt is recognized and the following four conditions are met, the delivery of the virtual interrupt is triggered:

1. RFLAGS.IF = 1
2. No blocking due to STI
3. No blocking caused by MOV SS or POP SS
4. Primary Processor-Based VM-Execution Controls中的 interrupt-window exiting bit 为0。

The virtual interrupt delivery will update the RVI and SVI in the guest interrupt status, and generate an interrupt event in the non-root environment


#### Interrupt chip

Both QEMU and KVM have realized the simulation of the interrupt chip, which is caused by historical reasons. Before the birth of KVM, QEMU provided a complete set of device simulations, including interrupt chips. After the birth of KVM, in order to further improve the interrupt performance, a set of interrupt chips was implemented in KVM. We can decide whose interrupt chip (irq chip) to use through the startup parameter kernel-irqchip of QEMU.

* on: KVM simulates all
* Split: QEMU simulates IOAPIC and PIC, and KVM simulates LAPIC
* off: QEMU simulates all


#### GPIO(General-purpose input/output)
By definition, GPIO is a universal PIN, which can be controlled as input or output at runtime. Input is readable and output is readable and writable.

In QEMU, GPIO is used extensively to represent the PIN of the device and interrupt controller. Like the hardware implementation, it is divided into IN and OUT. Described in the following data structure:

```c
struct NamedGPIOList {
    char *name;
    qemu_irq *in;
    int num_in;
    int num_out;
    QLIST_ENTRY(NamedGPIOList) node;
};
```

The unit of PIN is qemu_irq. Therefore, the structure maintains the first address of the input qemu_irq array, and all input qemu_irq can be accessed. At the same time maintain the number of input and output.

Each device maintains one or more NamedGPIOList, organized in the form of a linked list (member node is used to string together), pointing to DeviceState.gpios. For example, 8259 has a NamedGPIOList named NULL with 1 out and 8 in.

qemu_irq is a pointer to IRQState, defined as follows:

```c
struct IRQState {
    Object parent_obj;

    qemu_irq_handler handler;
    void *opaque;
    int n;
};
```

Generally speaking, n is the PIN number, opaque points to the owning device, and handler is the callback function of the PIN.

When there is a signal to be sent to a certain PIN of the device, the handler corresponding to input qemu_irq is called to indicate that the device has received the signal, so the handler sets some attributes of the device to indicate that its state has changed. If output is needed, call a handler of output qemu_irq, which means to send the signal.

The output PIN is often set to the qemu_irq pointer during initialization. When the upstream device (receiving the output of the device) is initialized, the pointer will point to the input qemu_irq of the upstream device. Therefore, when the device outputs, it calls the handler corresponding to the input qemu_irq of the upstream device, which simulates the process of transmitting a signal from one device to another.

According to the qom-tree of q35, we found that the devices with GPIO are mainly PIC and IOAPIC:

```
/machine (pc-q35-2.8-machine)
    /device[9] (hpet)
      /unnamed-gpio-in[0] (irq)
      /unnamed-gpio-in[1] (irq)
    /device[7] (isa-i8259)
      /unnamed-gpio-in[3] (irq)
      /unnamed-gpio-in[4] (irq)
      /unnamed-gpio-in[5] (irq)
      /unnamed-gpio-in[6] (irq)
      /unnamed-gpio-in[2] (irq)
      /unnamed-gpio-in[7] (irq)
      /unnamed-gpio-in[0] (irq)
      /unnamed-gpio-in[1] (irq)
    /device[8] (isa-i8259)
      /unnamed-gpio-in[3] (irq)
      /unnamed-gpio-in[4] (irq)
      /unnamed-gpio-in[5] (irq)
      /unnamed-gpio-in[6] (irq)
      /unnamed-gpio-in[2] (irq)
      /unnamed-gpio-in[7] (irq)
      /unnamed-gpio-in[0] (irq)
      /unnamed-gpio-in[1] (irq)
 /q35 (q35-pcihost)
    /pcie.0 (PCIE)
    /ioapic (ioapic)
      /unnamed-gpio-in[17] (irq)
      /unnamed-gpio-in[9] (irq)
      /unnamed-gpio-in[20] (irq)
      /unnamed-gpio-in[19] (irq)
      /unnamed-gpio-in[22] (irq)
      /unnamed-gpio-in[0] (irq)
      /unnamed-gpio-in[10] (irq)
      /unnamed-gpio-in[2] (irq)
      /unnamed-gpio-in[12] (irq)
      /unnamed-gpio-in[4] (irq)
      /unnamed-gpio-in[14] (irq)
      /unnamed-gpio-in[6] (irq)
      /unnamed-gpio-in[16] (irq)
      /unnamed-gpio-in[8] (irq)
      /unnamed-gpio-in[18] (irq)
      /unnamed-gpio-in[21] (irq)
      /unnamed-gpio-in[23] (irq)
      /unnamed-gpio-in[1] (irq)
      /unnamed-gpio-in[11] (irq)
      /unnamed-gpio-in[3] (irq)
      /unnamed-gpio-in[13] (irq)
      /unnamed-gpio-in[5] (irq)
      /unnamed-gpio-in[15] (irq)
      /unnamed-gpio-in[7] (irq)
```



#### GSI(Global System Interrupt)

The ACPI (Advanced Configuration and Power Interface) specification defines a unified configuration interface for x86 machines, and interrupts are no exception. Ten years ago, people felt that the computer architecture was and will be at the stage of mixing PIC and APIC for a long time, such as QEMU's classic architecture q35, so GSI was defined.

GSI assigns a unique interrupt number to the input pin on each interrupt controller in the system:

* For 8259A, GSI maps directly to ISA IRQ. For example, GSI 0 maps IRQ 0.
* For IOAPIC, each IOAPIC will be assigned a GSI base by the BIOS. When mapping, it is base + pin. For example, if the GSI base of IOAPIC0 is 0 and there are 24 pins, their corresponding GSI is 0-23. The GSI base of IOAPIC1 is 24, with 16 pins, the range is 24-39, and so on.


QEMU uses GSIState to describe GSI:

```c
typedef struct GSIState {
    qemu_irq i8259_irq[ISA_NUM_IRQS];
    qemu_irq ioapic_irq[IOAPIC_NUM_PINS];
} GSIState;
```

For Q35, the machine initialization function pc_q35_init is responsible for initializing GSI and filling GPIO at the same time.





## Interrupt simulation

### KVM analog chip

#### PIC
KVM uses kvm_pic to simulate the 8259A chip. Its pointer is stored in kvm.arch.vpic.

```c
struct kvm_pic {
    spinlock_t lock;
    bool wakeup_needed;
    unsigned pending_acks;
    struct sqm * sqm;
    struct kvm_kpic_state pics[2]; /* 0 is master pic, 1 is slave pic */ // Maintain all registers and states on the interrupt chip
    int output;     /* intr from master PIC */
    struct kvm_io_device dev_master; // master device
    struct kvm_io_device dev_slave; // slave device
    struct kvm_io_device dev_eclr; // Register to control interrupt trigger mode
    void (*ack_notifier)(void *opaque, int irq);
    unsigned long irq_states[PIC_NUM_PINS];
};

struct kvm_kpic_state {
    u8 last_irr;    /* edge detection */
    u8 irr;     /* interrupt request register */
    u8 imr;     /* interrupt mask register */
    u8 isr;     /* interrupt service register */
    u8 priority_add;    /* highest irq priority */
    u8 irq_base;
    u8 read_reg_select;
    u8 poll;
    u8 special_mask;
    u8 init_state;
    u8 auto_eoi;
    u8 rotate_on_auto_eoi;
    u8 special_fully_nested_mode;
    u8 init4;       /* true if 4 byte init */
    u8 elcr;        /* PIIX edge/trigger selection */
    u8 elcr_mask;
    u8 isr_ack; /* interrupt ack detection */
    struct kvm_pic *pics_state;
};
```

dev_master, dev_slave, dev_eclr ​​define the operation functions corresponding to the device, and at the same time register them to the PIO bus (KVM_PIO_BUS) through kvm_io_bus_register_dev.

When you need to read and write to the device, the following functions are called:

```c
static const struct kvm_io_device_ops picdev_master_ops = {
    .read     = picdev_master_read,
    .write    = picdev_master_write,
};

static const struct kvm_io_device_ops picdev_slave_ops = {
    .read     = picdev_slave_read,
    .write    = picdev_slave_write,
};

static const struct kvm_io_device_ops picdev_eclr_ops = {
    .read     = picdev_eclr_read,
    .write    = picdev_eclr_write,
};
```

#### IOAPIC

KVM only simulates one type of IOAPIC, named kvm_ioapic. Its pointer is stored in kvm.kvm_arch.vioapic.

```c
struct kvm_ioapic {
    u64 base_address;
    u32 ioregsel;
    u32 id;
    u32 irr; // IRR register
    u32 pad;
    union kvm_ioapic_redirect_entry redirtbl[IOAPIC_NUM_PINS]; // PRT, each entry represents a pin
    unsigned long irq_states[IOAPIC_NUM_PINS];
    struct kvm_io_device dev;
    struct sqm * sqm;
    void (*ack_notifier)(void *opaque, int irq);
    spinlock_t lock;
    struct rtc_status rtc_status;
    struct delayed_work eoi_inject;
    u32 irq_eoi [IOAPIC_NUM_PINS];
    u32 irr_delivered;
};

// RTE
union kvm_ioapic_redirect_entry {
    u64 bits;
    struct {
        u8 vector; // Interrupt vector (ISRV) number, which specifies the vector corresponding to the interrupt. Priority = vector / 16, the bigger the higher
        u8 delivery_mode:3; // Delivery mode, specify how the interrupt is sent to the destination LAPIC, including Fixed, Lowest Priority, SMI, NMI, INIT, ExtINT
        u8 dest_mode:1; // Destination mode, 0 is Physical Mode, 1 is Logical Mode
        u8 delivery_status:1; // Delivery status, 0 is IDEL (no interrupt), 1 is Send Pending (the interrupt has been received but has not been sent for some reason)
        u8 polarity:1; //Pin polarity, specify whether the effective level of the pin is high or low, 0 is high, 1 is low
        u8 remote_irr:1; //Remote IRR, (interrupt level trigger) when LAPIC receives the interrupt, it is set to 1, and LAPIC is cleared to 0 when writing EOI
        u8 trig_mode:1; // Trig mode, 1 is horizontal, 2 is edge
        u8 mask:1; //Interrupt mask bit, mask the interrupt when 1
        u8 reserve:7; // not used
        u8 reserved[4]; // not used
        u8 dest_id; // Target, indicates the ID of the target LAPIC in Physical Mode, and indicates a group of CPUs in Logical Mode?
    } fields;
};
```

After receiving KVM_CREATE_IRQCHIP from QEMU, call kvm_ioapic_init to initialize: call kvm_iodevice_init to bind operation:

```c
static const struct kvm_io_device_ops ioapic_mmio_ops = {
    .read     = ioapic_mmio_read,
    .write    = ioapic_mmio_write,
};
```

And register dev to the MMIO bus (KVM_MMIO_BUS) through kvm_io_bus_register_dev.

#### LAPIC

In KVM, each vCPU has its own LAPIC called kvm_lapic. Its pointer is stored in vcpu.arch.apic.

```c
struct kvm_lapic {
    unsigned long base_address; // Base address (GPA)
    struct kvm_io_device dev; // Save the operation corresponding to LAPIC
    struct kvm_timer lapic_timer;
    u32 divide_count;
    struct kvm_vcpu *vcpu;
    bool sw_enabled;
    bool irr_pending;
    bool lvt0_in_nmi_mode;
    /* Number of bits set in ISR. */
    s16 isr_count;
    /* The highest vector set in ISR; if -1 - invalid, must scan ISR. */
    int highest_isr_cache;
    /**
     * APIC register page.  The layout matches the register layout seen by
     * the guest 1:1, because it is accessed by the vmx microcode.
     * Note: Only one register, the TPR, is used by the microcode.
     */
    void *regs; // Point to a page of host, save all virtual registers used by LAPIC, such as IRR, ISR, LVT, etc.
    gpa_t vapic_addr;
    struct gfn_to_hva_cache vapic_cache;
    unsigned long pending_events;
    unsigned int sipi_vector;
};
```

It is created and initialized in vmx_create_vcpu => kvm_vcpu_init => kvm_arch_vcpu_init => kvm_create_lapic.

When you need to read and write MMIO to the device, the following functions are called:

```c
static const struct kvm_io_device_ops apic_mmio_ops = {
    .read     = apic_mmio_read,
    .write    = apic_mmio_write,
};
```

When reading/writing a certain register of LAPIC, because the VMCS-Secondary Processor-Based VM-Execution Controls-virtualize APIC accesses bit is set to 1, VMEXIT is generated, and back to KVM, the register address is subtracted from base_address to get the offset, and then through kvm_lapic_reg_read / kvm_lapic_reg_write operates the LAPIC struct.





#### Create a process

QEMU in kvm_init, if it is on or split, it means that KVM is needed to simulate the interrupt chip, so for initialization, the calling process of on is as follows:

```
kvm_irqchip_create => kvm_arch_irqchip_create => kvm_vm_enable_cap(s, KVM_CAP_SPLIT_IRQCHIP, 0, 24) => kvm_vm_ioctl(s, KVM_ENABLE_CAP, &cap)
                   => kvm_vm_ioctl(s, KVM_CREATE_IRQCHIP)
```

So the key here is to create the chip through KVM_CREATE_IRQCHIP.

In KVM, the call chain is as follows:

```
kvm_arch_vm_ioctl => kvm_create_pic Create PIC chip
                  => kvm_ioapic_init creates and initializes the IOAPIC chip
                  => kvm_setup_default_irq_routing => kvm_set_irq_routing set interrupt routing
```

Under on, KVM will directly use default_routing as the interrupt route. That is, KVM initializes the interrupt routing table kvm->irq_routing.


```c
static const struct kvm_irq_routing_entry default_routing[] = {
  ROUTING_ENTRY2(0), ROUTING_ENTRY2(1),
  ROUTING_ENTRY2(2), ROUTING_ENTRY2(3),
  ROUTING_ENTRY2(4), ROUTING_ENTRY2(5),
  ROUTING_ENTRY2(6), ROUTING_ENTRY2(7),
  ROUTING_ENTRY2(8), ROUTING_ENTRY2(9),
  ROUTING_ENTRY2(10), ROUTING_ENTRY2(11),
  ROUTING_ENTRY2(12), ROUTING_ENTRY2(13),
  ROUTING_ENTRY2(14), ROUTING_ENTRY2(15),
  ROUTING_ENTRY1(16), ROUTING_ENTRY1(17),
  ROUTING_ENTRY1(18), ROUTING_ENTRY1(19),
  ROUTING_ENTRY1(20), ROUTING_ENTRY1(21),
  ROUTING_ENTRY1(22), ROUTING_ENTRY1(23),
};

#define ROUTING_ENTRY2(irq) \
  IOAPIC_ROUTING_ENTRY(irq), PIC_ROUTING_ENTRY(irq)

#define ROUTING_ENTRY1(irq) IOAPIC_ROUTING_ENTRY(irq)

#define PIC_ROUTING_ENTRY(irq) \
  {.gsi = race, .type = KVM_IRQ_ROUTING_IRQCHIP, \
    .u.irqchip = {.irqchip = SELECT_PIC (race), .pin = (race)% 8}}

#define IOAPIC_ROUTING_ENTRY(irq) \
  {.gsi = race, .type = KVM_IRQ_ROUTING_IRQCHIP, \
    .u.irqchip = {.irqchip = KVM_IRQCHIP_IOAPIC, .pin = (race)}}
```

It can be seen that the first 16 numbers (0-15) of GSI have both PIC and IOAPIC. And 16-23 is only IOAPIC.

##### kvm_set_irq_routing

```
kvm_set_irq_routing => setup_routing_entry => kvm_set_routing_entry
                    => rcu_assign_pointer(kvm->irq_routing, new)
```

It will create a new kvm_irq_routing_table, then traverse the newly passed entries array, call setup_routing_entry for each entry one by one, construct kvm_irq_routing_entry and set it to the new table. Finally, point kvm->irq_routing to the new table

The type of interrupt routing table kvm->irq_routing is kvm_irq_routing_table. The entries are stored in the map, here is a list of kvm_irq_routing_entry:

```c
struct kvm_irq_routing_table {
    int chip[KVM_NR_IRQCHIPS][KVM_IRQCHIP_NUM_PINS]; // The primary index refers to the corresponding interrupt chip, the secondary index corresponds to the pin, and the GSI number corresponding to the pin is stored. Currently deprecated
    u32 nr_rt_entries;
    /*
     * Array indexed by gsi. Each entry contains list of irq chips
     * the gsi is connected to.
     */
    struct hlist_head map[0]; // Point to the hash table of list, key is gsi, value is kvm_kernel_irq_routing_entry list
};

struct kvm_kernel_irq_routing_entry {
    u32 gsi; // GSI number of the pin
    u32 type;
    int (*set)(struct kvm_kernel_irq_routing_entry *e, // set interrupt function
           struct kvm *kvm, int irq_source_id, int level, // kvm, interrupt resource ID, high and low level
           bool line_status);
    union {
        struct {
            unsigned race;
            unsigned pin;
        } irqchip;
        struct {
            u32 address_lo;
            u32 address_hi;
            u32 data;
            u32 flags;
            u32 devid;
        } msi;
        struct kvm_s390_adapter_int adapter;
        struct kvm_hv_sint hv_sint;
    };
    struct hlist_node link;
};
```

Specific interrupt chips (such as PIC, IOAPIC) realize the corresponding behavior during interrupt injection by implementing the set function of kvm_irq_routing_entry.

##### setup_routing_entry

The function setup_routing_entry is responsible for setting up the interrupt routing of a gsi:

```c
static int setup_routing_entry(struct kvm *kvm,
                   struct kvm_irq_routing_table *rt,
                   struct kvm_kernel_irq_routing_entry *e,
                   const struct kvm_irq_routing_entry *ue)
{
    int r = -EINVAL;
    struct kvm_kernel_irq_routing_entry *ei;

    /*
     * Do not allow GSI to be mapped to the same irqchip more than once.
     * Allow only one to one mapping between GSI and non-irqchip routing.
     */
    hlist_for_each_entry(ei, &rt->map[ue->gsi], link)
        if (ei->type != KVM_IRQ_ROUTING_IRQCHIP ||
            ue->type != KVM_IRQ_ROUTING_IRQCHIP ||
            ue-> u.irqchip.irqchip == ei-> irqchip.irqchip)
            return r;

    e->gsi = ue->gsi;
    e->type = ue->type;
    r = kvm_set_routing_entry(kvm, e, ue);
    if (r)
        goto out;
    if (e->type == KVM_IRQ_ROUTING_IRQCHIP)
        rt-> chip [e-> irqchip.irqchip] [e-> irqchip.pin] = e-> gsi;

    hlist_add_head(&e->link, &rt->map[e->gsi]);
    r = 0;
out:
    return r;
}
```

Here traverse the list corresponding to gsi in kvm_irq_routing_table->map, if it is found that there is kvm_kernel_irq_routing_entry of the target irqchip, it means that it has been set, then return directly. Because the GSI corresponding to all pins in an interrupt controller should be different.

Otherwise, set the entry and further set it through kvm_set_routing_entry after filling gsi and type.

```c

int kvm_set_routing_entry(struct kvm *kvm,
              struct kvm_kernel_irq_routing_entry *e,
              const struct kvm_irq_routing_entry *ue)
{
    int r = -EINVAL;
    int delta;
    unsigned max_pin;

    switch (ue->type) {
    case KVM_IRQ_ROUTING_IRQCHIP:
        delta = 0;
        switch (ue-> u.irqchip.irqchip) {
        case KVM_IRQCHIP_PIC_MASTER:
            e->set = kvm_set_pic_irq;
            max_pin = PIC_NUM_PINS;
            break;
        case KVM_IRQCHIP_PIC_SLAVE:
            e->set = kvm_set_pic_irq;
            max_pin = PIC_NUM_PINS;
            delta = 8;
            break;
        case KVM_IRQCHIP_IOAPIC:
            max_pin = KVM_IOAPIC_NUM_PINS;
            e->set = kvm_set_ioapic_irq;
            break;
        default:
            goto out;
        }
        e-> irqchip.irqchip = ue-> u.irqchip.irqchip;
        e-> irqchip.pin = ue-> u.irqchip.pin + delta;
        if (e->irqchip.pin >= max_pin)
            goto out;
        break;
    case KVM_IRQ_ROUTING_MSI:
        e->set = kvm_set_msi;
        e->msi.address_lo = ue->u.msi.address_lo;
        e->msi.address_hi = ue->u.msi.address_hi;
        e->msi.data = ue->u.msi.data;

        if (kvm_msi_route_invalid(kvm, e))
            goto out;
        break;
    case KVM_IRQ_ROUTING_HV_SINT:
        e->set = kvm_hv_set_sint;
        e-> hv_sint.vcpu = ue-> u.hv_sint.vcpu;
        e-> hv_sint.sint = ue-> u.hv_sint.sint;
        break;
    default:
        goto out;
    }

    r = 0;
out:
    return r;
}
```

Here is the set function of the kvm_kernel_irq_routing_entry structure described above. For ordinary interrupts (KVM_IRQ_ROUTING_IRQCHIP), different set functions will be set according to different interrupt controllers (irqchip):

* For PIC, since type is KVM_IRQ_ROUTING_IRQCHIP, set is kvm_set_pic_irq
* For IOAPIC, since the type is KVM_IRQCHIP_PIC_MASTER / KVM_IRQCHIP_PIC_SLAVE, the set is kvm_set_ioapic_irq

So far, the interrupt routing in KVM is initialized.





### Interrupt injection
If the device is simulated in QEMU, interrupt injection is required when an interrupt is generated.

In the function kvm_init that QEMU initializes the KVM accelerator, there are:

```c
    s->irq_set_ioctl = KVM_IRQ_LINE;
    if (kvm_check_extension(s, KVM_CAP_IRQ_INJECT_STATUS)) {
        s->irq_set_ioctl = KVM_IRQ_LINE_STATUS;
    }

#define KVM_IRQ_LINE              _IOW(KVMIO,  0x61, struct kvm_irq_level)
#define KVM_IRQ_LINE_STATUS       _IOWR(KVMIO, 0x67, struct kvm_irq_level)
```

If KVM supports returning the injected result, set s->irq_set_ioctl = KVM_IRQ_LINE_STATUS, otherwise it is KVM_IRQ_LINE

So QEMU will inject interrupts to KVM through ioctl in kvm_set_irq:

```c
int kvm_set_irq(KVMState *s, int irq, int level)
{
    struct kvm_irq_level event;
    int ret;

    assert(kvm_async_interrupts_enabled());

    event.level = level;
    event.irq = irq;
    ret = kvm_vm_ioctl(s, s->irq_set_ioctl, &event);
    if (ret < 0) {
        perror("kvm_set_irq");
        abort();
    }

    return (s->irq_set_ioctl == KVM_IRQ_LINE) ? 1 : event.status;
}
```

The call chain in KVM is kvm_vm_ioctl => kvm_vm_ioctl_irq_line => kvm_set_irq(kvm, KVM_USERSPACE_IRQ_SOURCE_ID, irq_event->irq, irq_event->level, line_status)


```c
/*
 * Return value:
 *  < 0   Interrupt was ignored (masked or not delivered for other reasons)
 *  = 0   Interrupt was coalesced (previous irq is still pending)
 *  > 0   Number of CPUs interrupt was delivered to
 */
int kvm_set_irq(struct kvm *kvm, int irq_source_id, u32 irq, int level,
        bool line_status)
{
    struct kvm_kernel_irq_routing_entry irq_set [KVM_NR_IRQCHIPS];
    int ret = -1, i, idx;

    trace_kvm_set_irq(irq, level, irq_source_id);

    /* Not possible to detect if the guest uses the PIC or the
     * IOAPIC.  So set the bit in both. The guest will ignore
     * writes to the unused one.
     */
    idx = srcu_read_lock(&kvm->irq_srcu);
    // Query kvm->irq_routing, take out the kvm_kernel_irq_routing_entry corresponding to the interrupt number (pin?) one by one and set it to irq_set return
    i = kvm_irq_map_gsi(kvm, irq_set, irq);
    srcu_read_unlock(&kvm->irq_srcu, idx);

    // Call the set function of kvm_kernel_irq_routing_entry to set the interrupt, if the chip is not implemented, the set is empty
    while (i--) {
        int r;
        r = irq_set[i].set(&irq_set[i], kvm, irq_source_id, level,
                   line_status);
        if (r < 0)
            continue;

        ret = r + ((ret <0)? 0: ret);
    }

    return ret;
}
```

Among them, irq_source_id is the id of the interrupt source device, irq is the original interrupt request number (not converted to gsi), and level represents the high and low levels of the interrupt.

Here, find the corresponding entry from the interrupt routing table, and call the set function set when the interrupt routing is initialized. As mentioned earlier:

* For PIC, set is kvm_set_pic_irq
* For IOAPIC, set is kvm_set_ioapic_irq

#### kvm_set_pic_irq

```
kvm_set_pic_irq => pic_irqchip find the corresponding interrupt chip
                => kvm_pic_set_irq => pic_set_irq1 Set the pin corresponding to irq, set irr (interrupt request register)
                                   => pic_update_irq => pic_irq_request Send interrupt request
```

among them:

```c
static void pic_irq_request(struct kvm *kvm, int level)
{
    struct kvm_pic *s = pic_irqchip(kvm);

    if (!s->output)
        s->wakeup_needed = true;
    s->output = level;
}
```

Responsible for setting the output in the interrupt chip kvm_pic to the corresponding level. At the same time, if the original output is 0, set wakeup_needed to true, so in pic_unlock, `kvm_make_request(KVM_REQ_EVENT, found)` will be called to set the request, and then the target vCPU will exit through kvm_vcpu_kick to process the request.


#### kvm_set_ioapic_irq

kvm_set_ioapic_irq => kvm_ioapic_set_irq => ioapic_set_irq => ioapic_service

```
ioapic_service
=> Create and initialize the interrupt message kvm_lapic_irq
=> kvm_irq_delivery_to_apic sends interrupt message to LAPIC
```

kvm_lapic_irq is the interrupt message formatted by IOAPIC, defined as follows:

```c
struct kvm_lapic_irq {
    u32 vector;
    u16 delivery_mode;
    u16 dest_mode;
    bool level;
    u16 trig_mode;
    u32 shorthand;
    u32 dest_id;
    bool msi_redir_hint;
};
```

Call kvm_irq_delivery_to_apic with the message kvm_lapic_irq as a parameter

```c
int kvm_irq_delivery_to_apic(struct kvm *kvm, struct kvm_lapic *src,
        struct kvm_lapic_irq * irq, struct dest_map * dest_map)
{
    int i, r = -1;
    struct kvm_vcpu *vcpu, *lowest = NULL;
    unsigned long dest_vcpu_bitmap[BITS_TO_LONGS(KVM_MAX_VCPUS)];
    unsigned int dest_vcpus = 0;

    if (irq->dest_mode == 0 && irq->dest_id == 0xff &&
            kvm_lowest_prio_delivery(irq)) {
        printk(KERN_INFO "kvm: apic: phys broadcast and lowest prio\n");
        irq->delivery_mode = APIC_DM_FIXED;
    }

    if (kvm_irq_delivery_to_apic_fast(kvm, src, irq, &r, dest_map))
        return r;

    memset(dest_vcpu_bitmap, 0, sizeof(dest_vcpu_bitmap));

    kvm_for_each_vcpu(i, vcpu, kvm) {
        if (!kvm_apic_present(vcpu))
            continue;

        if (!kvm_apic_match_dest(vcpu, src, irq->shorthand,
                    irq-> dest_id, irq-> dest_mode))
            continue;

        if (!kvm_lowest_prio_delivery(irq)) {
            if (r < 0)
                r = 0;
            r += kvm_apic_set_irq(vcpu, irq, dest_map);
        } else if (kvm_lapic_enabled(vcpu)) {
            if (!kvm_vector_hashing_enabled()) {
                if (!lowest)
                    lowest = vcpu;
                else if (kvm_apic_compare_prio(vcpu, lowest) < 0)
                    lowest = vcpu;
            } else {
                __set_bit(i, dest_vcpu_bitmap);
                dest_vcpus ++;
            }
        }
    }

    if (dest_vcpus != 0) {
        int idx = kvm_vector_to_index(irq->vector, dest_vcpus,
                    dest_vcpu_bitmap, KVM_MAX_VCPUS);

        lowest = kvm_get_vcpu(kvm, idx);
    }

    if (lowest)
        r = kvm_apic_set_irq(lowest, irq, dest_map);

    return r;
}
```

In addition to handling external interrupts (ioapic => lapic), this function can also handle IPI (lapic => lapic, see apic_send_ipi).

It first tries to find the target LAPIC from kvm.arch.apic_map. kvm.arch.apic_map is defined as follows:

```c
struct kvm_apic_map {
    struct rcu_head rcu;
    u8 mode;
    u32 max_apic_id;
    union {
        struct kvm_lapic *xapic_flat_map[8];
        struct kvm_lapic *xapic_cluster_map[16][4];
    };
    struct kvm_lapic *phys_map[]; // Maintain the mapping from LAPIC ID to kvm_lapic pointer
};
```

So kvm_irq_delivery_to_apic_fast => kvm_apic_map_get_dest_lapic, for interrupts that are not broadcast and the lowest priority, you can directly retrieve the corresponding kvm_lapic from phys_map according to irq->dest_id. Then kvm_apic_set_irq directly sets an interrupt to the target vCPU. Otherwise, it is necessary to traverse all vCPUs and match the irq->dest_id of RTE one by one. Call kvm_apic_set_irq on the matching vcpu.


kvm_apic_set_irq is implemented as the lapic setting interrupt of the vcpu:

```
=> __apic_accept_irq
    => Set corresponding settings according to delivery_mode, for example, APIC_DM_FIXED is kvm_lapic_set_vector + kvm_lapic_set_irr
    => kvm_make_request(event, vcpu) ，event 可取 KVM_REQ_EVENT / KVM_REQ_SMI / KVM_REQ_NMI
    => kvm_vcpu_kick(vcpu) Let the target vCPU exit to process the request
```

Kvm_make_request essentially sets the bit corresponding to the request in vcpu->requests, and the request will be processed next time vcpu_enter_guest.



##### kvm_vcpu_kick

kvm_vcpu_kick => smp_send_reschedule (native_smp_send_reschedule) => apic->send_IPI(cpu, RESCHEDULE_VECTOR) (x2apic_send_IPI)

Generate an interrupt to the target vcpu and let it be rescheduled. Because an external interrupt is set in VMCS, VMExit will occur, so it returns to KVM, so that it can inject interrupts before re-VMENTRY (vcpu_enter_guest)

So kvm_x86_ops->run (vmx_vcpu_run) returns to vcpu_enter_guest and then to vcpu_run to enter the next cycle, so vcpu_enter_guest is called again:

```
vcpu_enter_guest => inject_pending_event run before checking the request, if kvm_check_request(KVM_REQ_EVENT, vcpu), interrupt injection before running vcpu
                 => kvm_x86_ops->run            VMLAUNCH/VMRESUME
                 => vmx->idt_vectoring_info = vmcs_read32(IDT_VECTORING_INFO_FIELD)
                 => vmx_complete_interrupts => __vmx_complete_interrupts Update the vcpu according to the interrupt information, the enlisted team
```

The specific process is:

```c
    if (kvm_check_request(KVM_REQ_EVENT, vcpu) || req_int_win) {
        kvm_apic_accept_events(vcpu);
        if (vcpu->arch.mp_state == KVM_MP_STATE_INIT_RECEIVED) {
            r = 1;
            goto out;
        }
        // interrupt injection
        if (inject_pending_event(vcpu, req_int_win) != 0)
            req_immediate_exit = true;
        else {
            /* Enable NMI/IRQ window open exits if needed.
             *
             * SMIs have two cases: 1) they can be nested, and
             * then there is nothing to do here because RSM will
             * cause a vmexit anyway; 2) or the SMI can be pending
             * because inject_pending_event has completed the
             * injection of an IRQ or NMI from the previous vmexit,
             * and then we request an immediate exit to inject the SMI.
             */
            if (vcpu->arch.smi_pending && !is_smm(vcpu))
                req_immediate_exit = true;
            if (vcpu->arch.nmi_pending)
                kvm_x86_ops->enable_nmi_window(vcpu);
            if (kvm_cpu_has_injectable_intr(vcpu) || req_int_win)
                kvm_x86_ops->enable_irq_window(vcpu);
        }

        if (kvm_lapic_enabled(vcpu)) {
            update_cr8_intercept(vcpu);
            kvm_lapic_sync_to_vapic(vcpu);
        }
    }
```

KVM_REQ_EVENT is found, so call inject_pending_event

```
=> If there is pending exception, call kvm_x86_ops->queue_exception (vmx_queue_exception) to queue again
=> If nmi_injected, call kvm_x86_ops->set_nmi (vmx_inject_nmi)
=> If there is a pending interrupt, call kvm_x86_ops->set_irq (vmx_inject_irq)
=> If there is pending non-maskable interrupt, call kvm_x86_ops->set_nmi (vmx_inject_nmi)
=> kvm_cpu_has_injectable_intr if vCPU has injectable interrupt
=> kvm_queue_interrupt(vcpu, kvm_cpu_get_interrupt(vcpu), false) Set the highest priority interrupt to vcpu->arch.interrupt
=> kvm_x86_ops->set_irq (vmx_inject_irq) Write interrupt information to VMCS
    => vmcs_write32(VM_ENTRY_INSTRUCTION_LEN, vmx->vcpu.arch.event_exit_inst_len) For soft interrupt, you need to write instruction length
    => vmcs_write32(VM_ENTRY_INTR_INFO_FIELD, intr) Update interrupt information area
```

##### kvm_cpu_has_injectable_intr
Used to judge whether there is an interrupt that can be injected.

=> lapic_in_kernel If LAPIC is not in KVM, it means that QEMU is responsible for the simulation, so vcpu.arch.interrupt has already been set, return interrupt.pending
=> kvm_cpu_has_extint If there is pending external (non-APIC) interrupt, return true
=> kvm_vcpu_apicv_active If virtual interrupt delivery is enabled, the APIC interrupt will be handled by hardware without software intervention, return false
=> kvm_apic_has_interrupt If LAPIC is in KVM, find the interrupt number with the highest priority, if it is greater than PPR, return true
    => apic_update_ppr update PPR
    => apic_find_highest_irr => apic_search_irr => find_highest_vector Find the highest priority interrupt number from IRR
    => If the interrupt number is less than or equal to PPR, return -1


#### Summary

Before re-run, judge whether there is an interrupt request. If so, check the interrupt queue of LAPIC to find the interrupt with the highest priority. If the interrupt vector number is greater than the PPR (Processor Priority Register), injection is required.

So set vcpu->arch.interrupt (kvm_queued_interrupt), where pending is set to true

```c
struct kvm_queued_interrupt {
    bool pending;
    bool soft; // Whether to soft interrupt
    u8 nr; // interrupt vector number
} interrupt;
```


In VMEXIT, if the injection is successful, the pending will be set to false in vmx_vcpu_run => vmx_complete_interrupts => __vmx_complete_interrupts => kvm_clear_interrupt_queue.

If the injection fails, requeue will be called in __vmx_complete_interrupts to re-inject.









### QEMU analog chip
We know that all devices in QEMU are defined by TypeInfo and then registered with TypeImpl. When creating a device, call class_init to initialize the class object, then call instance_init to initialize the class instance object, and finally complete the construction of the device by realizing.

#### PIC

```c
static const TypeInfo i8259_info = {
    .name       = TYPE_I8259,
    .instance_size = sizeof(PICCommonState),
    .parent     = TYPE_PIC_COMMON,
    .class_init = i8259_class_init,
    .class_size = sizeof(PICClass),
    .interfaces = (InterfaceInfo[]) {
        { TYPE_INTERRUPT_STATS_PROVIDER },
        { }
    },
};
```

In pc_q35_init, there is the following piece of code:

```c
i8259 = i8259_init(isa_bus, pc_allocate_cpu_irq());
for (i = 0; i < ISA_NUM_IRQS; i++) {
    gsi_state->i8259_irq[i] = i8259[i];
}
```

Here first allocate an interrupt (parent_irq) for the PIC device, then call pc_allocate_cpu_irq => qemu_allocate_irq(pic_irq_request, NULL, 0) to create an interrupt with a sequence number of 0 and a handler for pic_irq_request. As an upstream interrupt.

Then initialize PIC:

```c
qemu_irq *i8259_init(ISABus *bus, qemu_irq parent_irq)
{
    qemu_irq *irq_set;
    DeviceState *dev;
    ISADevice * isadev;
    int i;

    irq_set = g_new0(qemu_irq, ISA_NUM_IRQS);

    // Create PIC master device and hang on isa_bus
    isadev = i8259_init_chip(TYPE_I8259, bus, true);
    dev = DEVICE(isadev);

    qdev_connect_gpio_out(dev, 0, parent_irq);
    for (i = 0 ; i < 8; i++) {
        irq_set[i] = qdev_get_gpio_in(dev, i);
    }

    isa_pic = dev;
    // Create PIC slave device and hang on isa_bus
    isadev = i8259_init_chip(TYPE_I8259, bus, false);
    dev = DEVICE(isadev);

    qdev_connect_gpio_out(dev, 0, irq_set[2]);
    for (i = 0 ; i < 8; i++) {
        irq_set[i + 8] = qdev_get_gpio_in(dev, i);
    }

    slave_pic = PIC_COMMON(dev);

    return irq_set;
}
```

It is responsible for creating two class instance objects of the 8259 interrupt chip. i8259_init_chip => qdev_init_nofail => ... => pic_realize

```c
static void pic_realize(DeviceState *dev, Error **errp)
{
    PICCommonState *s = PIC_COMMON(dev);
    PICClass *pc = PIC_GET_CLASS(dev);

    memory_region_init_io(&s->base_io, OBJECT(s), &pic_base_ioport_ops, s,
                          "pic", 2);
    memory_region_init_io(&s->elcr_io, OBJECT(s), &pic_elcr_ioport_ops, s,
                          "elcr", 1);

    qdev_init_gpio_out(dev, s->int_out, ARRAY_SIZE(s->int_out));
    qdev_init_gpio_in(dev, pic_set_irq, 8);

    pc->parent_realize(dev, errp);
}
```

##### qdev_init_gpio_out

```
=> qdev_init_gpio_out_named(dev, pins, NULL, n)
    => qdev_get_named_gpio_list Take out gpio from the device instance (DeviceState), traverse the linked list to find the NamedGPIOList of the corresponding name, if not found, create one, and insert it to the top
    => If no name is passed in, set name to "unnamed-gpio-out"
    => object_property_add_link uses the name "name[i]" as the link attribute of dev according to the qemu_irq array and length passed in.
    => NamedGPIOList.num_out +=n
```

##### qdev_init_gpio_in

```
=> qdev_init_gpio_in_named(dev, handler, NULL, n)
    => qdev_get_named_gpio_list Take out gpio from the device instance (DeviceState), traverse the linked list to find the NamedGPIOList of the corresponding name, if not found, create one, and insert it to the top
    => NamedGPIOList.in = qemu_extend_irqs(gpio_list->in, gpio_list->num_in, handler, dev, n) Create n qemu_irq on the original basis, the handler is the incoming function
    => If no name is passed in, set name to "unnamed-gpio-in"
    => According to the number passed in, each qemu_irq is named "name[i]" as the child attribute of dev
    => NamedGPIOList.num_in increases by n
```

So each 8259 will have 1 out and 8 in GPIOs, stored in the DeviceState.gpios linked list named NULL

The value of "unnamed-gpio-out[0]" corresponding to out is a qemu_irq *pointer*, pointing to s->int_out, which stores the address of the member, which has not been set.
The value of "unnamed-gpio-in[i]" corresponding to in is a qemu_irq. Its handler is pic_set_irq, opaque is dev


#### PIC connection

After creating the class instance object of the 8259 interrupt chip, i8259_init calls qdev_connect_gpio_out to the master to connect:

qdev_connect_gpio_out(dev, 0, parent_irq) => qdev_connect_gpio_out_named(dev, NULL, n, pin)  *<-pin就是parent_irq*

##### qdev_connect_gpio_out_named

```
=> object_property_add_child sets the parent interrupt (parent_irq) as the child property of the "/machine/unattached" container under the name "non-qdev-gpio[*]".
=> object_property_set_link sets the link attribute value of the dev name "name[i]" to parent_irq. This attribute is the "name[i]" created in the previous qdev_init_gpio_out, and the value points to s->int_out, so s->int_out is set to parent_irq, and the pit of out is filled.
```

If the parent interrupt has a path, `child->parent != NULL`, object_property_add_child returns. Otherwise, add it as a child of the "/machine/unattached" container. There is a detail here that in fact their property name is not "non-qdev-gpio[*]", because in object_property_add_child => object_property_add, it will try to replace `*` from 0. For example, the attribute name of parent_irq assigned here is "non-qdev-gpio[24]", so the full path is "/machine/unattached/non-qdev-gpio[24]", which is also reflected in qom-tree :

```
/machine (pc-q35-2.8-machine)
  /unattached (container)
    /non-qdev-gpio[24] (irq)
```

The reason for setting the upper-level interrupt (such as parent_irq) to the child property of "/machine/unattached" container is because the upper-level interrupt needs to have its own path in the next object_property_set_link:

```c
void object_property_set_link(Object *obj, Object *value,
                              const char *name, Error **errp)
{
    if (value) {
        // Take out the path of the superior interrupt
        gchar *path = object_get_canonical_path(value);
        object_property_set_str(obj, path, name, errp);
        g_free(path);
    } else {
        object_property_set_str(obj, "", name, errp);
    }
}
```

##### qdev_get_gpio_in => qdev_get_gpio_in_named

Get the 8 qemu_irqs created in qdev_init_gpio_in and stored in NamedGPIOList.in, and store them in positions 0-7 of the irq_set array

The slave will also be initialized through a similar process, but its upstream port is not parent_irq, but the third qemu_irq of the master, that is, irq_set[2], which simulates the hardware out of the slave to the master. Circuit of irq2 pin. Finally, get the 8 qemu_irqs created in qdev_init_gpio_in and stored in NamedGPIOList.in, and store them in positions 8-15 of the irq_set array.

After PIC initialization is complete, irq_set is returned and stored in gsi_state->i8259_irq.



#### LAPIC

pc_new_cpu ==> apic_init(env,env->cpuid_apic_id) ==> qdev_create(NULL, "kvm-apic");

According to x86_cpu_realizefn => x86_cpu_apic_create => apic_get_class, LAPIC is not placed in KVM at this time, and QEMU is responsible for simulating it, so apic_type = "apic":

```c
static const TypeInfo apic_common_type = {
    .name = TYPE_APIC_COMMON,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(APICCommonState),
    .instance_init = apic_common_initfn,
    .class_size = sizeof(APICCommonClass),
    .class_init = apic_common_class_init,
    .abstract = true,
};
```

apic_common_realize => apic_realize

```c
static void apic_realize(DeviceState *dev, Error **errp)
{
    APICCommonState *s = APIC(dev);

    if (s->id >= MAX_APICS) {
        error_setg(errp, "%s initialization failed. APIC ID %d is invalid",
                   object_get_typename(OBJECT(dev)), s->id);
        return;
    }

    memory_region_init_io(&s->io_memory, OBJECT(s), &apic_io_ops, s, "apic-msi",
                          APIC_SPACE_SIZE);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, apic_timer, s);
    local_apics[s->id] = s;

    msi_nonbroken = true;
}
```

You can see that it has registered the corresponding MemoryRegion for MSI. When operating on the MemoryRegion, perform the following operations:

```c
static const MemoryRegionOps apic_io_ops = {
    .old_mmio = {
        .read = { apic_mem_readb, apic_mem_readw, apic_mem_readl, },
        .write = { apic_mem_writeb, apic_mem_writew, apic_mem_writel, },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};
```


#### IOAPIC

It is defined as follows:

```c
static const TypeInfo ioapic_info = {
    .name          = "ioapic",
    .parent        = TYPE_IOAPIC_COMMON,
    .instance_size = sizeof(IOAPICCommonState),
    .class_init    = ioapic_class_init,
};
```

If PIC is enabled, pc_q35_init will call ʻioapic_init_gsi(gsi_state, "q35");` to initialize IOAPIC:

```c
void ioapic_init_gsi(GSIState *gsi_state, const char *parent_name)
{
    DeviceState *dev;
    SysBusDevice *d;
    unsigned int i;

    if (kvm_ioapic_in_kernel()) {
        dev = qdev_create(NULL, "kvm-ioapic");
    } else {
        dev = qdev_create(NULL, "ioapic");
    }
    if (parent_name) {
        object_property_add_child(object_resolve_path(parent_name, NULL),
                                  "ioapic", OBJECT(dev), NULL);
    }
    qdev_init_nofail(dev);
    d = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(d, 0, IO_APIC_DEFAULT_ADDRESS);

    for (i = 0; i < IOAPIC_NUM_PINS; i++) {
        gsi_state->ioapic_irq[i] = qdev_get_gpio_in(dev, i);
    }
}
```

At this time, IOAPIC is not placed in KVM, and QEMU is responsible for its simulation. So qdev_init_nofail => ... => ioapic_realize

```c
static void ioapic_realize(DeviceState *dev, Error **errp)
{
    IOAPICCommonState *s = IOAPIC_COMMON(dev);

    if (s->version != 0x11 && s->version != 0x20) {
        error_report("IOAPIC only supports version 0x11 or 0x20 "
                     "(default: 0x11).");
        exit(1);
    }

    memory_region_init_io(&s->io_memory, OBJECT(s), &ioapic_io_ops, s,
                          "ioapic", 0x1000);

    qdev_init_gpio_in(dev, ioapic_set_irq, IOAPIC_NUM_PINS);

    ioapics[ioapic_no] = s;
    s->machine_done.notify = ioapic_machine_done_notify;
    qemu_add_machine_init_done_notifier(&s->machine_done);
}
```

The same is the qemu_irq of in created by qdev_init_gpio_in, a total of 24 are created, and the handler is ioapic_set_irq

IOAPIC is stored in the global array ioapics, and the index is also maintained by the global variable ioapic_no.

So IOAPIC will have 24 in GPIO. Different from the qemu_irq that PIC will create and return it to gsi_state by pc_q35_init, ioapic_init_gsi directly passes in the gsi_state pointer, and gsi_state->ioapic_irq is set in the function.

#### GSI

So far, i8259_irq and ioapic_irq in gsi_state have been filled. In fact, before initializing PIC and IOAPIC, pc_q35_init will create GSI qemu_irq:

```c
    // If ioapic is in the kernel (KVM), that is, when kvm is turned on, specify the parameter kernel-irqchip=on, then
    if (kvm_ioapic_in_kernel()) {
        kvm_pc_setup_irq_routing(pcmc->pci_enabled); // Create an interrupt route and set it to KVM
        pcms->gsi = qemu_allocate_irqs(kvm_pc_gsi_handler, gsi_state,
                                       GSI_NUM_PINS);
    }
    // Otherwise (ioapic is in QEMU when off/split)
    else {
        pcms->gsi = qemu_allocate_irqs(gsi_handler, gsi_state, GSI_NUM_PINS);
    }
```

GSI_NUM_PINS(24) qemu_irqs will be created, numbered from 0-23, opaque points to gsi_state, handler is kvm_pc_gsi_handler (when IOAPIC is simulated by KVM) / gsi_handler (when IOAPIC is simulated by QEMU), and saved to PCMachineState.gsi.

Then pc_q35_init will create and initialize ICH9-LPC through pci_create_simple_multifunction and call its realize function:

```
ich9_lpc_realize => isa_bus = isa_bus_new(...)                                              创建 ISABus
                 => lpc->isa_bus = isa_bus sets ISABus as a member of ICH9LPCState
                 => qdev_init_gpio_out_named(dev, lpc->gsi, ICH9_GPIO_GSI, GSI_NUM_PINS) Create 24 out GPIO
                 => isa_bus_irqs(isa_bus, lpc->gsi) Set ISABus.irqs to ICH9LPCState.gsi
```

Created 24 out GPIO (qemu_irq) through qdev_init_gpio_out_named, stored in the gpios linked list of the device parent DeviceState, named "gsi". Each qemu_irq *pointer* takes the name "name[i]" as the link attribute of dev and points to the address of the ICH9LPCState.gsi array member.

The following isa_bus_irqs sets ISABus.irqs to ICH9LPCState.gsi. In other words, ISABus.irqs points to the value pointed to by ICH9-LPC out GPIO(qemu_irq).

Next, pc_q35_init will connect the out GPIO just created by ICH9LPCState to pcms->gsi one by one:

```c
    for (i = 0; i < GSI_NUM_PINS; i++) {
        qdev_connect_gpio_out_named(lpc_dev, ICH9_GPIO_GSI, i, pcms->gsi[i]);
    }
```

So ISABus.irqs is equivalent to PCMachineState.gsi, that is, ISABus.irqs[i] == PCMachineState.gsi[i]. as the picture shows:

```
PCMachineState.gsi (gsi_handler)
| | | | | | ... |
0 1 2 3 4 5     23
| | | | | | ... |  out
------------------
|    ICH9-LPC    |
------------------
   (ISABus.irqs)
```


You can use GDB to verify:

```
(gdb) p *isa_bus->irqs
$15 = (qemu_irq) 0x55555694f8c0
(gdb) p *PC_MACHINE(qdev_get_machine())->gsi
$16 = (qemu_irq) 0x55555694f8c0
```


#### Interrupt injection

Different from the interrupt chip (IOAPIC) simulated by KVM, the interrupt is transmitted by finding the target LAPIC and then directly setting its variables. QEMU uses GPIO.

Generally speaking, the irq member of the device that generates the interrupt will be set to PCMachineState.gsi. Taking the serial port (isa-serial) as an example, in its realize function serial_isa_realizefn, ʻisa_init_irq(isadev, &s->irq, isa->isairq)` is called to set the irq member of the device object (SerialState), where isa-> isairq is obtained through isa_serial_irq[isa->index], index is a static variable in serial_isa_realizefn, which is incremented by one for each call.

According to the definition of isa_serial_irq, there are 4 serial devices in total, and the corresponding isairqs are 4, 3, 4, 3 respectively. For the serial device with index 0, its isairq is 4, so:

```c
void isa_init_irq(ISADevice *dev, qemu_irq *p, int isairq)
{
    assert(dev->nirqs < ARRAY_SIZE(dev->isairq));
    dev->isairq[dev->nirqs] = isairq;
    *p = isa_get_irq(dev, isairq);
    dev->nirqs++;
}
```

Call isa_get_irq to fetch the corresponding qemu_irq (isabus->irqs[4]) from isabus->irqs, and set it to the class instance object of the serial device, namely SerialState.irq.

As mentioned earlier, ISABus.irqs is equivalent to PCMachineState.gsi. So the irq of the serial device actually points to the GSI qemu_irq. This is equivalent to each device corresponding to the GSI. The handler of GSI qemu_irq is gsi_handler, and n specifies its serial number in the GSIState array.

```
PCMachineState.gsi (gsi_handler)
          |
          4
          | irq
-----------------------
|  isa-serial device  |
-----------------------
```


Continue to interrupt the injection analysis. The device will call the following two functions to set the level when sending an interrupt:

* qemu_irq_lower => qemu_set_irq(irq, 0) is set to low level
* qemu_irq_raise => qemu_set_irq(irq, 1) is set to high level


##### qemu_set_irq

=> irq->handler(irq->opaque, irq->n, level) is responsible for taking out the handler in qemu_irq to call.

Because it belongs to GSIState, gsi_handler is called:

```c
void gsi_handler(void *opaque, int n, int level)
{
    GSIState *s = opaque;

    DPRINTF("pc: %s GSI %d\n", level ? "raising" : "lowering", n);
    if (n < ISA_NUM_IRQS) {
        qemu_set_irq(s->i8259_irq[n], level);
    }
    qemu_set_irq(s->ioapic_irq[n], level);
}
```

According to the sequence number (qemu_irq.n), it takes out the corresponding qemu_irq handler of the corresponding chip to call.

#### PIC

```
parent_irq (pic_irq_request)
    | out
-----------------
|  8259 master  |
-----------------
| | | | | | | |
0 1 | 3 4 5 6 7     (pic_set_irq)
    |
    | out
-----------------
|  8259 slave   |
-----------------
| | | | | | | |
0 1 2 3 4 5 6 7     (pic_set_irq)
```

For PIC, the handler is pic_set_irq, which belongs to in of PIC. So pic_set_irq sets the irr register (variable) of the PIC chip (PICCommonState), and then calls pic_update_irq:

```c
static void pic_update_irq(PICCommonState *s)
{
    int irq;

    irq = pic_get_irq(s);
    if (irq >= 0) {
        DPRINTF("pic%d: imr=%x irr=%x padd=%d\n",
                s->master ? 0 : 1, s->imr, s->irr, s->priority_add);
        qemu_irq_raise(s->int_out[0]);
    } else {
        qemu_irq_lower(s->int_out[0]);
    }
}

```

Get the interrupt with the highest priority in irr that is not shielded by imr through pic_get_irq. If there is, set out (s->int_out[0]) to high level, otherwise set to low level. then

```
qemu_set_irq => pic_irq_request => If the CPU has LAPIC, call apic_deliver_pic_intr to set it to LAPIC
                                => otherwise call cpu_interrupt / cpu_reset_interrupt according to the level
```

Here is an interesting point: In SMP, which CPU should the PIC interrupt be sent to? The implementation of QEMU is simple and rude. According to pic_irq_request, it selects the first CPU (first_cpu).

#### IOAPIC

```
------------------
|    IOAPIC      |
------------------
| | | | | | ... |
0 1 | 3 4 5     23  in (ioapic_set_irq)
```

For IOAPIC, the handler is ioapic_set_irq, which belongs to the in of IOAPIC.

```c
static void ioapic_set_irq(void *opaque, int vector, int level)
{
    IOAPICCommonState *s = opaque;

    /* ISA IRQs map to GSI 1-1 except for IRQ0 which maps
     * to GSI 2.  GSI maps to ioapic 1-1.  This is not
     * the cleanest way of doing it but it should work. */

    DPRINTF("%s: %s vec %x\n", __func__, level ? "raise" : "lower", vector);
    if (vector == 0) {
        vector = 2;
    }
    if (vector >= 0 && vector < IOAPIC_NUM_PINS) {
        uint32_t mask = 1 << vector;
        uint64_t entry = s->ioredtbl[vector];

        if (((entry >> IOAPIC_LVT_TRIGGER_MODE_SHIFT) & 1) ==
            IOAPIC_TRIGGER_LEVEL) {
            /* level triggered */
            if (level) {
                s->irr |= mask;
                if (!(entry & IOAPIC_LVT_REMOTE_IRR)) {
                    ioapic_service(s);
                }
            } else {
                s->irr &= ~mask;
            }
        } else {
            /* According to the 82093AA manual, we must ignore edge requests
             * if the input pin is masked. */
            if (level && !(entry & IOAPIC_LVT_MASKED)) {
                s->irr |= mask;
                ioapic_service(s);
            }
        }
    }
}
```

First, it finds the entry register corresponding to the interrupt vector number from the I/O REDIRECTION TABLE of IOAPIC. Including Interrupt Mask, Trigger Mode, Remote IRR and other bits. If the Trigger Mode bit is 1, it means horizontal trigger, and 0 means edge trigger.

For horizontal trigger, after setting the corresponding bit in irr, you need to judge the Remote IRR bit. If it is 1, it means that LAPIC has received the interrupt from IOAPIC and is being processed; if it is 0, it means that LAPIC has finished processing the interrupt. Send an EOI message to IOAPIC, indicating that it can continue to receive interrupts. So if it is 0, ioapic_service can be called to send an interrupt message.

For edge triggering, the Interrupt Mask bit needs to be judged. If it is 1, it means that the interrupt is masked and there is no need to set irr; if it is 0, it means that the interrupt can be sent, so after setting the corresponding bit in irr, call ioapic_service to send the interrupt message.

ioapic_service will traverse all pins on IOAPIC. If irr is 1 in the corresponding bit, an interrupt needs to be sent:

If LAPIC is in KVM (kernel-irq=split), set it to KVM through kvm_set_irq, otherwise it will be converted to *MSI*. According to the definition, the device can directly construct the MSI message, which indicates the interrupt target address, and then the device directly sends the interrupt to LAPIC, bypassing IOAPIC.

Since we are discussing the situation where LAPIC is simulated by QEMU, we first query I/O REDIRECTION TABLE (IOAPICCommonState.ioredtbl) with the pin number to get the entry, and then get the relevant information (ioapic_entry_info) through ioapic_entry_parse, and finally pass `stl_le_phys(ioapic_as, info.addr) , info.data)` Modify IOAPIC AddressSpace.


If IR is turned on, IOAPIC AddressSpace is the address space of a virtual machine `vtd_host_dma_iommu(bus, s, Q35_PSEUDO_DEVFN_IOAPIC)`. Otherwise, it is address_space_memory. When writing to the AddressSpace, similar to MMIO, the apic_io_ops bound to the MemoryRegion is finally called. As mentioned above, they are bound to the apic-msi MemoryRegion of LAPIC during apic_realize.

then

```
stl_le_phys => address_space_stl_le => address_space_stl_internal => memory_region_dispatch_write => access_with_adjusted_size => memory_region_oldmmio_write_accessor => mr->ops->old_mmio.write[ctz32(size)] (apic_mem_writel)
```

apic_mem_writel obtains the LAPIC (APICCommonState) of the current CPU through cpu_get_current_apic, and then writes data to its corresponding location according to addr.

Therefore, IOAPIC has no out, and it sends the interrupt to LAPIC via MSI.










#### QEMU simulates PIC and IOAPIC chips, and KVM simulates LAPIC

Compared with the interrupt delivery process described above, the interrupt will be sent to the KVM in the ioapic_service in spilt mode. KVM performs interrupt routing according to its own kvm->irq_routing.


#### Interrupt chip initialization

QEMU initializes the interrupt chip for KVM in kvm_init:

```
kvm_irqchip_create => kvm_arch_irqchip_create => kvm_vm_enable_cap(s, KVM_CAP_SPLIT_IRQCHIP, 0, 24) => kvm_vm_ioctl(s, KVM_ENABLE_CAP, &cap)
                   => kvm_init_irq_routing
```

For split, only need to create LAPIC in KVM without kvm_vm_ioctl(s, KVM_CREATE_IRQCHIP). It tries to open the split capability through KVM_ENABLE_CAP, and then calls kvm_init_irq_routing to initialize the interrupt routing of all pins of IOAPIC.


##### kvm_init_irq_routing

```
=> kvm_check_extension(s, KVM_CAP_IRQ_ROUTING) Get the total number of gsi supported by KVM
=> Create used_gsi_bitmap and allocate irq_routes array
=> kvm_arch_init_irq_routing => kvm_irqchip_add_msi_route => kvm_add_routing_entry add entry to KVMState.entries array
                                                          => kvm_irqchip_commit_routes => kvm_vm_ioctl(KVM_SET_GSI_ROUTING) Set entries to KVM
```

kvm_irqchip_add_msi_route will be called 24 times, and the KVMState.entries when nr (the length of the entries array) is 1 to 24 are set to QEMU as kvm_irq_routing, and kvm_irq_routing is defined as follows:

```c
struct kvm_irq_routing {
  __u32 no;
  __u32 flags;
  struct kvm_irq_routing_entry entries[0];
};

struct kvm_irq_routing_entry {
  __u32 gsi;
  __u32 type;
  __u32 flags;
  __u32 pad;
  union {
    struct kvm_irq_routing_irqchip irqchip;
    struct kvm_irq_routing_msi msi;
    struct kvm_irq_routing_s390_adapter adapter;
    struct kvm_irq_routing_hv_sint hv_sint;
    __u32 pad[8];
  } u;
};
```

At this time, since the device has not been initialized, the attributes in the routing table entry kvm_irq_routing_entry are all 0.

After that, until after the virtual machine is started, when BIOS/OS updates the interrupt routing table, it triggers VMExit and returns to QEMU for update (because IOAPIC is simulated in QEMU):

```
address_space_rw => address_space_write => address_space_write_continue => memory_region_dispatch_write => access_with_adjusted_size => memory_region_write_accessor => ioapic_mem_write => ioapic_update_kvm_routes
=> ioapic_entry_parse(s->ioredtbl[i], &info)
=> msg.address = info.addr
=> msg.data = info.data
=> kvm_irqchip_update_msi_route(kvm_state, i, msg, NULL) => kvm_update_routing_entry 用entry更新 KVMState.entries 数组
=> kvm_irqchip_commit_routes => kvm_vm_ioctl(s, KVM_SET_GSI_ROUTING, s->irq_routes) Update the new KVMState.entries array to KVM
```

For example, the entry content of gsi 22 corresponding to e1000 is as follows:

```
(gdb) p kvm_state->irq_routes->entries[22]
$29 = {
  gsi = 22,
  type = 2,
  flags = 0,
  pad = 0,
  u = {
    irqchip = {
      irqchip = 4276092928,
      pin = 0
    },
    msi = {
      address_lo = 4276092928,
      address_hi = 0,
      data = 32865,
      {
        pad = 0,
        devid = 0
      }
    },
    adapter = {
      ind_addr = 4276092928,
      summary_addr = 32865,
      ind_offset = 0,
      summary_offset = 0,
      adapter_id = 0
    },
    hv_sint = {
      vcpu = 4276092928,
      saint = 0
    },
    pad = {[0] = 4276092928, [1] = 0, [2] = 32865, [3] = 0, [4] = 0, [5] = 0, [6] = 0, [7] = 0}
  }
}
```


#### KVM

In KVM, the call chain of kvm_vm_ioctl(s, KVM_ENABLE_CAP, &cap) is as follows:

```
kvm_vm_ioctl_enable_cap => kvm_setup_empty_irq_routing => kvm_set_irq_routing(kvm, empty_routing, 0, 0)
                        => kvm->arch.irqchip_split = true;
```

Unlike IOAPIC, which is simulated by KVM, the routing is initialized to default_routing through kvm_set_irq_routing. In split mode, routing needs to wait for QEMU to be set, so it is set to empty, that is, empty_routing.

At the same time, set kvm->arch.irqchip_split = true, and then this variable is checked by the function irqchip_split in KVM to determine whether it is in split mode.


The call chain of kvm_vm_ioctl(s, KVM_SET_GSI_ROUTING, s->irq_routes) is as follows:

```
kvm_vm_ioctl => kvm_set_irq_routing => setup_routing_entry => kvm_set_routing_entry
                                    => rcu_assign_pointer(kvm->irq_routing, new)
```

It will create a new kvm_irq_routing_table, then traverse the newly passed entries array, call setup_routing_entry for each entry one by one, construct kvm_irq_routing_entry and set it to the new table. Finally, point kvm->irq_routing to the new table.

Since the type of the incoming entry is KVM_IRQ_ROUTING_MSI(2), set the set to kvm_set_msi in kvm_set_routing_entry




#### Interrupt injection

When the topic returns to spilt mode, kvm_set_irq => kvm_vm_ioctl(s, s->irq_set_ioctl, &event) will be called in ioapic_service to inject interrupts into KVM. s->irq_set_ioctl may be KVM_IRQ_LINE or KVM_IRQ_LINE_STATUS according to KVM capabilities, the difference is that the latter will return status.

So enter KVM, kvm_vm_ioctl => kvm_vm_ioctl_irq_line

```
=> kvm_irq_map_gsi query kvm->irq_routing and take out the kvm_kernel_irq_routing_entry corresponding to the gsi one by one
=> kvm_set_irq(kvm, KVM_USERSPACE_IRQ_SOURCE_ID, irq_event->irq, irq_event->level, line_status) => irq_set[i].set (kvm_set_msi)
```

##### kvm_set_msi

```
kvm_set_msi => kvm_set_msi_irq
            => kvm_irq_delivery_to_apic
```

Responsible for parsing the irq message, constructing kvm_lapic_irq, and then setting it to the LAPIC of the corresponding vCPU.

kvm_irq_delivery_to_apic => kvm_apic_set_irq => __apic_accept_irq realizes setting interrupt for target LAPIC:

```
=> Set corresponding settings according to delivery_mode, for example, APIC_DM_FIXED is kvm_lapic_set_vector + kvm_lapic_set_irr
=> kvm_make_request(KVM_REQ_EVENT, vcpu)
=> kvm_vcpu_kick(vcpu) Let the target vCPU exit to process the request
```

Next, in vcpu_run => vcpu_enter_guest, since LAPIC is in KVM, first update the interrupt with the highest priority in irr through kvm_x86_ops->hwapic_irr_update (vmx_hwapic_irr_update)?

After detecting a KVM_REQ_EVENT request in KVM, call inject_pending_event for interrupt injection:

```c
static int inject_pending_event(struct kvm_vcpu *vcpu, bool req_int_win)
{
  ...
  if (vcpu->arch.interrupt.pending) {
      kvm_x86_ops->set_irq(vcpu);
      return 0;
  }
  ...
}
```

Finally, vmx_inject_irq writes the interrupt into VMCS.






















### Interrupted complete injection process

Take the interrupt after e1000 receives the packet as an example

The scenarios where interrupts need to be set are as follows:

* MMIO
    address_space_rw => address_space_read => address_space_read_full => address_space_read_continue => memory_region_dispatch_read => memory_region_dispatch_read1 => access_with_adjusted_size => memory_region_read_accessor => e1000_mmio_read => mac_icr_read / ... => set_interrupt_cause => pci_set_irq

* QEMU received e1000 package

    main_loop => main_loop_wait => qemu_clock_run_all_timers => qemu_clock_run_timers => timerlist_run_timers => ra_timer_handler => ndp_send_ra ip6_output => if_output => if_start => if_encap => slirp_output => qemu_send_packet => qemu_sendv_packet_async => qemu_net_queue_send_iov => qemu_net_queue_deliver_iov => qemu_deliver_packet_iov => e1000_receive_iov => set_ics => set_interrupt_cause => pci_set_irq

* Mitigation timer timeout (triggered by the main thread)

    main_loop => main_loop_wait => qemu_clock_run_all_timers => qemu_clock_run_timers => timerlist_run_timers => e1000_mit_timer => set_interrupt_cause => pci_set_irq

Finally, pci_set_irq is called to set the interrupt.

```
pci_set_irq => pci_intx Get PCI_INTERRUPT_PIN of PCI configuration space
            => pci_irq_handler => pci_set_irq_state Set the irq_state of the device
                               => pci_update_irq_status is PCI_STATUS of configuration space plus PCI_STATUS_INTERRUPT bit
                               => pci_irq_disabled If interrupt is disabled, return directly
                               => pci_change_irq_level otherwise the transmission is interrupted
```

##### pci_change_irq_level

```c
static void pci_change_irq_level(PCIDevice *pci_dev, int irq_num, int change)
{
    PCIBus *bus;
    for (;;) {
        bus = pci_dev->bus;
        irq_num = bus->map_irq(pci_dev, irq_num);
        if (bus->set_irq)
            break;
        pci_dev = bus->parent_dev;
    }
    bus->irq_count[irq_num] += change;
    bus->set_irq(bus->irq_opaque, irq_num, bus->irq_count[irq_num] != 0);
}
```

Get the bus where the PCI device is located and call bus->map_irq to find the pirq (Programmable Interrupt Router) number corresponding to the device. For e1000, its bus is pcie.0 and map_irq is ich9_lpc_map_irq:

```c
int ich9_lpc_map_irq(PCIDevice *pci_dev, int intx)
{
    BusState *bus = qdev_get_parent_bus(&pci_dev->qdev);
    PCIBus *pci_bus = PCI_BUS(bus);
    PCIDevice *lpc_pdev =
            pci_bus->devices[PCI_DEVFN(ICH9_LPC_DEV, ICH9_LPC_FUNC)];
    ICH9LPCState *lpc = ICH9_LPC_DEVICE(lpc_pdev);

    return lpc->irr[PCI_SLOT(pci_dev->devfn)][intx];
}
```

It first obtains the bus object (pcie.0) to which the device belongs through qdev_get_parent_bus, then finds the ICH9 LPC PCI to ISA bridge from the array of devices connected to the bus, and finds the corresponding pirq number of the e1000 in its irr, which is 6.

If the current level of bus defines the set_irq function, the for loop is interrupted and the call is sent to interrupt; otherwise, it is set as the parent device of the bus and enters the next round of searching. That is, starting from the device that sent the interrupt, searching upwards step by step until it finds a bus that can handle the interrupt.

Here set_irq is ich9_lpc_set_irq. So add change to the value corresponding to the current interrupt in the interrupt count array to indicate how many interrupts of this type are waiting to be processed. Then call ich9_lpc_set_irq.

```c
void ich9_lpc_set_irq(void *opaque, int pirq, int level)
{
    ICH9LPCState *lpc = opaque;
    int pic_irq, pic_dis;

    assert(0 <= pirq);
    assert(pirq < ICH9_LPC_NB_PIRQS);

    ich9_lpc_update_apic(lpc, ich9_pirq_to_gsi(pirq));
    ich9_lpc_pic_irq(lpc, pirq, &pic_irq, &pic_dis);
    ich9_lpc_update_pic(lpc, pic_irq);
}
```

Use ich9_pirq_to_gsi to convert pirq into GSI number, which is actually pirq + 16, and e1000 is 22. Then call ich9_lpc_update_apic, if the value corresponding to the current interrupt in the interrupt count array is not 0, the level is 1.

So according to the GSI number, take out the corresponding qemu_irq from ICH9LPCState.gsi, call qemu_set_irq to set its value to level.

#### Consider the case where e1000, IOAPIC, and LAPIC are all simulated by QEMU (off)

The handler of gsi qemu_irq is gsi_handler, so:

qemu_set_irq => irq->handler (gsi_handler) => qemu_set_irq => ioapic_set_irq 设置 IOAPICCommonState 的 irr 。

But at this time, the Remote IRR bit may be 1, so ioapic_service will not be called after setting irr.

Until a certain time after LAPIC is processed, EOI is sent to make the Remote IRR bit of IOAPIC become 0, and then ioapic_service will be called in ioapic_set_irq.

Since irr may accumulate multiple interrupts at this time, ioapic_service will traverse all pins on IOAPIC. If irr is 1 in the corresponding bit, modify the corresponding position of the interrupt in IOAPIC AddressSpace through stl_le_phys.

When writing to the AddressSpace, similar to MMIO, the apic_io_ops bound to the MemoryRegion is finally called. So call apic_mem_writel, construct the MSI message and send it through apic_send_msi.

```
apic_deliver_irq => apic_bus_deliver => apic_set_irq => apic_set_bit => apic_set_bit(s->irr, vector_num) Set Interrupt Request Register according to the interrupt vector number
                                                                     => apic_set_bit(s->tmr, vector_num) If it is a horizontal trigger, set the Trigger Mode Register
                                                     => apic_update_irq notify the CPU
```

##### apic_update_irq

```c
/* signal the CPU if an irq is pending */
static void apic_update_irq(APICCommonState *s)
{
    CPUState *cpu;
    DeviceState *dev = (DeviceState *)s;

    cpu = CPU(s->cpu);
    if (!qemu_cpu_is_self(cpu)) {
        cpu_interrupt(cpu, CPU_INTERRUPT_POLL);
    } else if (apic_irq_pending(s) > 0) {
        cpu_interrupt(cpu, CPU_INTERRUPT_HARD);
    } else if (!apic_accept_pic_intr(dev) || !pic_get_output(isa_pic)) {
        cpu_reset_interrupt(cpu, CPU_INTERRUPT_HARD);
    }
}
```

then:

```
cpu_interrupt(cpu, CPU_INTERRUPT_POLL) => cpu_interrupt_handler (kvm_handle_interrupt) => cpu->interrupt_request |= mask
                                                                                       => qemu_cpu_kick
```

Therefore, the interrupt_request of the target cpu will be set, and then kick it will exit to QEMU and return to kvm_cpu_exec. Since the exit reason is KVM_EXIT_INTR, even if it enters kvm_arch_handle_exit, it cannot be processed, so ret = -1, the loop is interrupted, and it exits to the superior call qemu_kvm_cpu_thread_fn , So execute kvm_cpu_exec => kvm_arch_process_async_events in the next loop, find that the CPU_INTERRUPT_POLL of interrupt_request is 1, call apic_poll_irq => apic_update_irq => cpu_interrupt(cpu, CPU_INTERRUPT_HARD). If LAPIC has an unhandled interrupt (apic_irq_pending), it is interrupt_request plus CPU_INTERRUPT_HARD

So in the next kvm_arch_pre_run, if the interrupt can be injected, the interrupt number is taken from LAPIC through cpu_get_pic_interrupt => apic_get_interrupt:

```
=> apic_irq_pending(s) Take the interrupt number with the highest priority from irr
=> apic_reset_bit(s->irr, intno) Set the interrupt number in the bit corresponding to irr to 0
=> apic_set_bit(s->isr, intno) Set the interrupt number in the bit corresponding to isr to 1
=> apic_update_irq(s) If there are other interrupts that have not been processed, set cpu->interrupt_request to CPU_INTERRUPT_HARD again
```

After obtaining the interrupt number, inject the interrupt to KVM through kvm_vcpu_ioctl(cpu, KVM_INTERRUPT, &intr).

If there are interrupts that have not been processed before, cpu->interrupt_request is still CPU_INTERRUPT_HARD at this time, but we can only inject one interrupt at a time, so set request_interrupt_window to 1, so as to ensure that the party guest can immediately return to QEMU when the next interrupt is processed.

The irq injected into KVM here is the **interrupt vector number**.

##### KVM

kvm_arch_vcpu_ioctl => kvm_vcpu_ioctl_interrupt => kvm_queue_interrupt(vcpu, irq->irq, false) Set interrupt to vcpu->arch.interrupt
                                                => kvm_make_request(KVM_REQ_EVENT, vcpu) make a request

Then when QEMU enters KVM through `kvm_vcpu_ioctl(cpu, KVM_RUN, 0)`,


In kvm_arch_vcpu_ioctl_run => vcpu_run => vcpu_enter_guest, a KVM_REQ_EVENT request is detected and inject_pending_event is called for interrupt injection:

```c
static int inject_pending_event(struct kvm_vcpu *vcpu, bool req_int_win)
{
  ...
  if (vcpu->arch.interrupt.pending) {
      kvm_x86_ops->set_irq(vcpu);
      return 0;
  }
  ...
}
```

Finally, vmx_inject_irq writes the interrupt into VMCS.




#### Consider the case where e1000 and IOAPIC are simulated by QEMU, and LAPIC is simulated by KVM (split)

The process of sending interrupts from e1000 to IOAPIC is the same as above until ioapic_service. It will use kvm_irqchip_is_split to determine whether it is in split mode. If it is, KVM is responsible for the simulation of LAPIC, so the interrupt is set by kvm_set_irq (note that for on mode, IOAPIC is also simulated by KVM and will not go here at all, so here is only to determine whether it is split).

So kvm_set_irq => kvm_vm_ioctl(s, s->irq_set_ioctl, &event) injects interrupts into KVM. s->irq_set_ioctl may be KVM_IRQ_LINE or KVM_IRQ_LINE_STATUS according to KVM capabilities, the difference is that the latter will return status.

The irq injected into KVM here is the **GSI** corresponding to the interrupt device. The gsi of e1000 is 22.




##### KVM

kvm_vm_ioctl => kvm_vm_ioctl_irq_line => kvm_set_irq(kvm, KVM_USERSPACE_IRQ_SOURCE_ID, irq_event->irq, irq_event->level, line_status)

Find the corresponding entry from the table, call kvm_set_ioapic_irq => kvm_ioapic_set_irq => ioapic_set_irq => ioapic_service => kvm_irq_delivery_to_apic => kvm_apic_set_irq => __apic_accept_irq Set interrupt to the target LAPIC:

```
=> Set corresponding settings according to delivery_mode, for example, APIC_DM_FIXED is kvm_lapic_set_vector + kvm_lapic_set_irr
=> kvm_make_request(KVM_REQ_EVENT, vcpu)
=> kvm_vcpu_kick(vcpu) Let the target vCPU exit to process the request
```

Next, in vcpu_run => vcpu_enter_guest, since LAPIC is in KVM, first update the interrupt with the highest priority in irr through kvm_x86_ops->hwapic_irr_update (vmx_hwapic_irr_update)?

After detecting a KVM_REQ_EVENT request, call inject_pending_event for interrupt injection:

```c
static int inject_pending_event(struct kvm_vcpu *vcpu, bool req_int_win)
{
  ...
  if (vcpu->arch.interrupt.pending) {
      kvm_x86_ops->set_irq(vcpu);
      return 0;
  }
  ...
}
```

Finally, vmx_inject_irq writes the interrupt into VMCS.



#### Consider the case where e1000 is simulated by QEMU, and IOAPIC and LAPIC are simulated by KVM (on)

At this time, the handler of gsi qemu_irq is kvm_pc_gsi_handler, so:

qemu_set_irq => irq->handler (kvm_pc_gsi_handler) => qemu_set_irq(s->ioapic_irq[n], level) => irq->handler (kvm_ioapic_set_irq) => kvm_set_irq(kvm_state, s->kvm_gsi_base + irq, level) => kvm_vm_ioctl(s, s->irq_set_ioctl, &event) 通过ioctl向KVM注入中断。

The irq injected into KVM here is the **GSI** corresponding to the interrupt device. Since s->kvm_gsi_base is 0, the gsi s->kvm_gsi_base + irq calculated by e1000 is still 22.

Therefore, it can be found that in the case of split and on, no matter where the IOAPIC is simulated, the interrupt is finally injected through the KVM_IRQ_LINE / KVM_IRQ_LINE_STATUS interface. And the interrupted gsi is 22.



##### KVM

The process in KVM is the same as that in split. Therefore, the difference from split is that on needs to query the IOAPIC information of the KVM mode through the interface, and split is responsible for simulation because QEMU is responsible for the simulation, so you don't need to query yourself.

For example, when on queries IOAPIC in hmp, it needs to be checked through kvm_ioapic_dump_state => kvm_ioapic_get => kvm_vm_ioctl(kvm_state, KVM_GET_IRQCHIP, &chip).
