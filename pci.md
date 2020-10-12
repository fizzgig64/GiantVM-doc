

This article will analyze how PCI devices in QEMU are initialized and mounted on the basis of familiarity with QOM and q35 architecture.


## PCI device creation and initialization

```
pci_create_simple_multifunction => pci_create_multifunction
    => qdev_create(&bus->qbus, name) => object_class_by_name
    => qdev_prop_set_int32(dev, "addr", devfn)
    => qdev_init_nofail => dc->realize (pci_qdev_realize)
    => do_pci_register_device => pc->realize (ich9_lpc_realize)
    => pci_add_option_rom // add rom
```

The PCI device object is created and realised is set to true, then the realize function is called to initialize.

The key point of this is the devfn parameter passed in by pci_create_simple_multifunction, which is the device function number of the PCI device, which is generally calculated by the following macro:

```c
#define PCI_DEVFN(slot, func)   ((((slot) & 0x1f) << 3) | ((func) & 0x07))
```

Therefore, the devfn calculated by this macro is an 8bit number, the upper 5bit is the slot number, and the lower 3bit is the func number. So there are 32 slots, and each slot has 8 functions.

For example, for the ISA bridge "ICH9-LPC", its function number is `PCI_DEVFN(ICH9_LPC_DEV, ICH9_LPC_FUNC)`, that is, 31 * 2^3 + 0 = 248

In object_class_by_name, the class instance constructor is called. According to TypeInfo of ICH9-LPC:

```c
static const TypeInfo ich9_lpc_info = {
    .name       = TYPE_ICH9_LPC_DEVICE,
    .parent     = TYPE_PCI_DEVICE,
    .instance_size = sizeof(struct ICH9LPCState),
    .instance_init = ich9_lpc_initfn,
    .class_init  = ich9_lpc_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_HOTPLUG_HANDLER },
        { TYPE_ACPI_DEVICE_IF },
        { }
    }
};
```

The class instance constructor is ich9_lpc_class_init, which initializes the properties of this device:

```c
static void ich9_lpc_class_init(ObjectClass *klass, void *data)
{
    ...
    k->realize = ich9_lpc_realize;
    k->config_write = ich9_lpc_config_write;
    dc->desc = "ICH9 LPC bridge";
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_ICH9_8;
    k->revision = ICH9_A2_LPC_REVISION;
    k->class_id = PCI_CLASS_BRIDGE_ISA;
    ...
}
```

### do_pci_register_device
Set the device instance object.

```
=> Check whether bus->devices[devfn] is empty, if it is not empty, it means the position is occupied by someone, and an error is returned.
=> If the device is hotplugged, if pci_get_function_0 is not empty, it means that the slot is occupied, and an error is returned
    => If the bus has an upstream PCIe port, it can only be placed in the first device of the first slot, that is, devfn=0. Otherwise, it can be placed at the first device of the slot corresponding to devfn
    => pci_init_bus_master
=> pci_config_alloc(pci_dev) allocates configuration space, PCI device is 256B, PCIe is 4096B
=> pci_config_set_vendor_id / pci_config_set_device_id / pci_config_set_revision / pci_config_set_revision Set the device identification according to the initialization data
=> If the device is bridge(is_bridge), pci_init_mask_bridge
=> pci_init_multifunction
=> Set pci_dev->config_read and pci_dev->config_write, if set in the class constructor, use the set, otherwise use the default function pci_default_read_config / pci_default_write_config
```

## BAR(base address register)

According to the PCI specification, BAR is used to describe the size of the address space that PCI devices need to occupy. For example, devices such as network cards need to occupy a larger address space, while some serial devices occupy less address space. It is located in the 24 bytes 0x10-0x27 in the PCI configuration space. If you use 32-bit BAR, you can set up to 6 for each device. If 64bit is used, only 3 can be set at most.

For each BAR, bit0 specifies the type of mapping, 0 is Memory, and 1 is I/O. On real hardware, bit0 is readonly, which is determined by the device manufacturer and cannot be modified by others.

For Memory type, bit1-2 is Locatable, 0 means that the register size is 32bit, which can be mapped to any position in the 32bit memory space; 2 is 64bit; 1 is reserved for PCI Local Bus Specification 3 revision. Bit3 is Prefetchable, 0 is no, and 1 is yes. bit4-end is Base Address (16-byte aligned).

For I/O type, bit1 is Reserved, bit2-end is Base Address (4-byte aligned).

The length of each BAR is determined by the hardware. BIOS/OS dynamically allocates a memory space for it according to the acquired length, and then writes the starting address of the memory space into the BAR as the address base, so [base, base+len ] This section will serve as a communication channel between the software (OS) and PCI devices.

Check the BAR of the PCI device according to qemu monitor:

```
(qemu) info pci
  Bus  0, device   0, function 0:
    Host bridge: PCI device 8086:29c0
      id ""
  Bus  0, device   1, function 0:
    VGA controller: PCI device 1234:1111
      BAR0: 32 bit prefetchable memory at 0xfd000000 [0xfdffffff].
      BAR2: 32 bit memory at 0xfebf0000 [0xfebf0fff].
      BAR6: 32 bit memory at 0xffffffffffffffff [0x0000fffe].
      id ""
  Bus  0, device   2, function 0:
    Ethernet controller: PCI device 8086:100e
      IRQ 11.
      BAR0: 32 bit memory at 0xfebc0000 [0xfebdffff].
      BAR1: I/O at 0xc000 [0xc03f].
      BAR6: 32 bit memory at 0xffffffffffffffff [0x0003fffe].
      id ""
  Bus  0, device  31, function 0:
    ISA bridge: PCI device 8086:2918
      id ""
  Bus  0, device  31, function 2:
    SATA controller: PCI device 8086:2922
      IRQ 10.
      BAR4: I/O at 0xc080 [0xc09f].
      BAR5: 32 bit memory at 0xfebf1000 [0xfebf1fff].
      id ""
  Bus  0, device  31, function 3:
    SMBus: PCI device 8086:2930
      IRQ 10.
      BAR4: I/O at 0x0700 [0x073f].
      id ""
```

It can be found that only VGA controller, Ethernet controller, SATA controller and SMBus have BAR, while Host bridge and PCI/ISA bridge (that is, the aforementioned ICH9-LPC) have no bar. Therefore, the guess is that the bridge device does not need to map the address space to communicate with the OS.

Corresponding to the OS, the corresponding bar can be found by `lspci -x`, for example, for PCI/ISA bridge, 0x10-0x27 is all 0, and for Ethernet controller, 0x10-0x27 is:

```
10: 00 00 bc fe 01 c0 00 00 00 00 00 00 00 00 00 00
20: 00 00 00 00 00 00 00 00
```

A bar occupies 4 bytes, so it can be seen (**x86 is little endian**):

* BAR 0 value is 0xfebc0000, corresponding to Memory type, 32bit, no Prefetchable, base = 0xfebc0000
* BAR 1 value is 0x0000c001, corresponding to I/O type, 32bit, no Prefetchable, base = 0x0000c000
* BAR 6 value is 0xffffffffffffffff, corresponding to Memory type, corresponding to ROM.

It can be found that the result of lspci is consistent with the result of query from QEMU monitor.


### Set BAR

So when is the device's BAR set in QEMU? After some searching, pci_register_bar was found:

```c
void pci_register_bar(PCIDevice *pci_dev, int region_num,
                      uint8_t type, MemoryRegion *memory)
{
    PCIIORegion *r;
    uint32_t addr; /* offset in pci config space */
    uint64_t wmask;
    pcibus_t size = memory_region_size(memory);

    assert(region_num >= 0);
    assert(region_num < PCI_NUM_REGIONS);
    if (size & (size-1)) {
        fprintf(stderr, "ERROR: PCI region size must be pow2 "
                    "type=0x%x, size=0x%"FMT_PCIBUS"\n", type, size);
        exit(1);
    }

    r = &pci_dev->io_regions[region_num];
    r->addr = PCI_BAR_UNMAPPED;
    r->size = size;
    r->type = type;
    r->memory = memory;
    r->address_space = type & PCI_BASE_ADDRESS_SPACE_IO
                        ? pci_dev->bus->address_space_io
                        : pci_dev->bus->address_space_mem;

    wmask = ~(size - 1);
    if (region_num == PCI_ROM_SLOT) {
        /* ROM enable bit is writable */
        wmask |= PCI_ROM_ADDRESS_ENABLE;
    }

    addr = pci_bar(pci_dev, region_num);
    pci_set_long(pci_dev->config + addr, type);

    if (!(r->type & PCI_BASE_ADDRESS_SPACE_IO) &&
        r->type & PCI_BASE_ADDRESS_MEM_TYPE_64) {
        // 64bit base address
        pci_set_quad(pci_dev->wmask + addr, wmask);
        pci_set_quad(pci_dev->cmask + addr, ~0ULL);
    } else {
        // 32bit
        pci_set_long(pci_dev->wmask + addr, wmask & 0xffffffff);
        pci_set_long(pci_dev->cmask + addr, 0xffffffff);
    }
}
```

It initializes the PCIIORegion structure according to the incoming MemoryRegion, which represents the corresponding segment of the address space in the BAR:

```c
typedef struct PCIIORegion {
    pcibus_t addr; /* current PCI mapping address. -1 means not mapped */
#define PCI_BAR_UNMAPPED (~(pcibus_t)0)
    pcibus_t size;
    uint8_t type;
    MemoryRegion *memory;
    MemoryRegion *address_space;
} PCIIORegion;
```

The mapping type (type), size (size), address (addr, the initial value is PCI_BAR_UNMAPPED), etc. are set here, and then set to the corresponding item of the PCIDevice.io_regions array region_num. In other words, all BAR related information is stored in the io_regions array.

At the same time, write the information into the corresponding position of BAR in config, here only the type (bit0) of BAR is set.

For example, for the network card e1000, when pci_e1000_realize, set:

```c
pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);
pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_IO, &d->io);
```

Both d->mmio and d->io are MemoryRegion, created by memory_region_init_io in e1000_mmio_setup, the size of the former is PNPMMIO_SIZE (0x20000), and the size of the latter is IOPORT_SIZE (0x40).


### MemoryRegion mapping

Now that we have created MemoryRegion and set it to io_regions, it must register the corresponding memory region in the virtual machine. We searched in the code with io_regions as a keyword and found pci_do_device_reset. When was it called? ? Through tracing, it can be found that when QEMU registers the devices in the main function, all initialization is ready, and when main_loop is about to be called to enter the main loop, qemu_system_reset is called. The stack is as follows:

```
(gdb) bt
#0  pci_do_device_reset (dev=0x7ffff22a0010) at hw/pci/pci.c:278
#1  0x0000555555a1a657 in pcibus_reset (qbus=0x5555569a5640) at hw/pci/pci.c:304
#2  0x00005555559720ef in qbus_reset_one (bus=0x5555569a5640, opaque=0x0) at hw/core/qdev.c:304
#3  0x0000555555976e53 in qbus_walk_children (bus=0x5555569a5640, pre_devfn=0x0, pre_busfn=0x0, post_devfn=0x55555597206c <qdev_reset_one>, post_busfn=0x55555597208f <qbus_reset_one>, opaque=0x0) at hw/core/bus.c:68
#4  0x0000555555972c50 in qdev_walk_children (dev=0x55555694ec00, pre_devfn=0x0, pre_busfn=0x0, post_devfn=0x55555597206c <qdev_reset_one>, post_busfn=0x55555597208f <qbus_reset_one>, opaque=0x0) at hw/core/qdev.c:602
#5  0x0000555555976e17 in qbus_walk_children (bus=0x555556769d20, pre_devfn=0x0, pre_busfn=0x0, post_devfn=0x55555597206c <qdev_reset_one>, post_busfn=0x55555597208f <qbus_reset_one>, opaque=0x0) at hw/core/bus.c:59
#6  0x00005555559721a2 in qbus_reset_all (bus=0x555556769d20) at hw/core/qdev.c:321
#7  0x00005555559721c5 in qbus_reset_all_fn (opaque=0x555556769d20) at hw/core/qdev.c:327
#8  0x00005555558d94ec in qemu_devices_reset () at vl.c:1765
#9  0x000055555582832c in pc_machine_reset () at /home/binss/work/qemu/qemu-2.8.1.1/hw/i386/pc.c:2181
#10 0x00005555558d9589 in qemu_system_reset (report=false) at vl.c:1778
#11 0x00005555558e0eb0 in main (argc=19, argv=0x7fffffffe498, envp=0x7fffffffe538) at vl.c:4656
```

The call chain is main => qemu_system_reset => mc->reset (pc_machine_reset) => qemu_devices_reset. It will traverse all reset_handlers, get each QEMUResetEntry, and call their callback function re->func. QEMUResetEntry is added to the queue at qemu_register_reset.

The QEMUResetEntry corresponding to `#8` is main-system-bus, which is the system bus. In vl.c, `qemu_register_reset(qbus_reset_all_fn, sysbus_get_default());` registered qbus_reset_all_fn as a reset handler. So call qbus_reset_all_fn here

qbus_reset_all_fn => qbus_reset_all => qbus_walk_children will traverse all devs on the bus and call qdev_walk_children on them. For e1000, this is naturally q35-pcihost (if you don't remember how dev and bus are connected on q35, please refer to q35). Then qdev_walk_children calls qbus_walk_children to all child_buses on q35-pcihost, and then comes to pcie.0. Note that e1000 is hung on this bus. So stop here for the time being, don't consider pcie.0's qbus_walk_children recursion, but after the relationship recursion is completed, call post_busfn (qbus_reset_one) to reset the pcie.0 bus.

So qbus_reset_one => bc->reset (pcibus_reset) will traverse bus->devices and call pci_do_device_reset for each device


As the name suggests, pci_do_device_reset mainly does reset work:

```
=> pci_word_test_and_clear_mask Clear the 2 bytes corresponding to PCI_COMMAND in config (0x04-0x06)
=> pci_word_test_and_clear_mask Clear the 2 bytes corresponding to PCI_STATUS in config (0x06-0x08)
=> Clear 1 byte corresponding to PCI_CACHE_LINE_SIZE and PCI_INTERRUPT_LINE in config
=> Traverse io_regions, if it exists, set the value of the BAR corresponding position in config, only write the type, and leave the rest to the BIOS/OS in the virtual machine for equipment
=> pci_update_mappings
=> msi_reset
=> msix_reset
```

The key here is pci_update_mappings, which is responsible for checking the addr of PCIIORegion in the io_regions array. If addr is not equal to PCI_BAR_UNMAPPED, it means that the address has been mapped, so through memory_region_add_subregion_overlap / memory_region_del_subregion use ioctl to register the memory region corresponding to the BARK to the KVM ):

```c
static void pci_update_mappings(PCIDevice *d)
{
    PCIIORegion *r;
    int i;
    pcibus_t new_addr;

    // The seventh io region (BAR 6) is PCI_ROM_SLOT
    for(i = 0; i < PCI_NUM_REGIONS; i++) {
        r = &d->io_regions[i];

        /* this region isn't registered */
        if (!r->size)
            continue;

        // Read the base address stored in the BAR from config
        new_addr = pci_bar_address(d, i, r->type, r->size);

        /* This bar isn't changed */
        if (new_addr == r->addr)
            continue;

        /* now do the real mapping */
        if (r->addr != PCI_BAR_UNMAPPED) {
            trace_pci_update_mappings_del(d, pci_bus_num(d->bus),
                                          PCI_SLOT(d->devfn),
                                          PCI_FUNC(d->devfn),
                                          i, r->addr, r->size);
            memory_region_del_subregion(r->address_space, r->memory);
        }
        r->addr = new_addr;
        if (r->addr != PCI_BAR_UNMAPPED) {
            trace_pci_update_mappings_add(d, pci_bus_num(d->bus),
                                          PCI_SLOT(d->devfn),
                                          PCI_FUNC(d->devfn),
                                          i, r->addr, r->size);
            memory_region_add_subregion_overlap(r->address_space,
                                                r->addr, r->memory, 1);
        }
    }

    pci_update_vga(d);
}
```

Unfortunately, when qemu_system_reset, the base address of BAR has not been set yet, and it is still in the state of unmap. So MemoryRegion is not registered in KVM.

So when will the two MemoryRegions of e1000 be registered in KVM? The answer is, after entering the virtual machine and starting to run, perform the device discovery process, configure an address base for e1000, and then write it to the configuration space (config) of e1000, then it will be registered, which will be done later analysis.



### Configuration space write and read

So our configuration space is initialized, and the next step is to wait for the virtual machine to read and write. As mentioned earlier, PCI devices have set config_write and config_read, which is what does this. In theory, BIOS/OS should read config through config_read to obtain the configuration space information of the PCI device, and then assign addresses to the PCI device and write back the BAR of the configuration space.


Since it was found that ICH9-LPC does not carry BAR, we will choose e1000 for analysis (when writing this article, I have been holding on to ICH9-LPC for one pass analysis, and finally found that because it does not carry bar, many processes continue directly, and cannot be compared. Good response to the registration process of PCI devices, tears collapsed).

According to e1000_base_info, its class constructor is e1000_class_init, which does not configure its own config function, but in the initialization function pci_e1000_realize set `pci_dev->config_write = e1000_write_config`, so its config_write is e1000_write_config, and config_read uses the default pci_default_read_config.

We find the address of config, and then set the observation point to observe the moment of reading and writing.

1. Allocate memory in do_pci_register_device and set the config content, such as pci_config_set_vendor_id
2. Continue to set config in pci_e1000_realize, including setting the BAR base address to full f in pci_register_bar
3. Because there is ROM (efi-e1000.rom), so call pci_add_option_rom, register PCI_ROM_SLOT as BAR6
4. pci_do_device_reset (mentioned earlier in the call chain) for cleaning and setting
5. KVM_EXIT_IO
    After QEMU => KVM => VM, when the VM runs the port I/O instruction to access the config information, VMExit occurs, VM => KVM => QEMU, QEMU knows that the reason is KVM_EXIT_IO according to exit_reason, so it is taken from cpu->kvm_run io information, make the following call:

    kvm_cpu_exec => kvm_handle_io => address_space_rw => address_space_read => address_space_read_full => address_space_read_continue => memory_region_dispatch_read => memory_region_dispatch_read1 => access_with_adjusted_size => memory_region_read_accessor => mr->ops->read (pci_host_data_read) => pci_data_read => pci_host_config_read_common => pci_default_read_config

    Then the config_read of e1000 is called, the configuration space information of the corresponding position is read and returned to KVM => VM.

    Similarly, when VM needs to write config, VMExit occurs, so VM => KVM => QEMU, the call chain is as follows:

    kvm_cpu_exec => kvm_handle_io => address_space_rw => address_space_write => address_space_write_continue => memory_region_dispatch_write => access_with_adjusted_size => memory_region_write_accessor => mr->ops->write (pci_host_data_write) => pci_data_write => pci_host_config_write_common => pci_dev->config_write (e1000_write_config) => pci_default_write_config

    Then the config_write of e1000 is called, the configuration space information of the corresponding position is read and returned to KVM => VM.

    When returning to QEMU, QEMU knows that the reason is KVM_EXIT_IO according to exit_reason, so it takes out the io information from cpu->kvm_run, such as `{direction = 1, size = 4, port = 3324, count = 1, data_offset = 4096}` . For write operations, pci_default_write_config is called last.

6. KVM_EXIT_MMIO

    After setting config, after Linux completes the initialization of the device, communication can be carried out. When VM accesses the mapped memory area, VMExit occurs, VM => KVM => QEMU, QEMU knows that the reason is KVM_EXIT_MMIO according to exit_reason, so it takes out the mmio information from cpu->kvm_run and makes the following call:

    kvm_cpu_exec => address_space_rw => address_space_write => address_space_write_continue => memory_region_dispatch_write => access_with_adjusted_size => memory_region_write_accessor => e1000_mmio_write

    For the write operation, e1000_mmio_write is finally called, which converts the opaque of the MemoryRegion to E1000State, and calls macreg_writeops[index]. Take the parameters addr=208, val=157, size=4 as an example, the address of the operation at this time is 4273733840 (0xfebc00d0):

    ʻIndex = (addr & 0x1ffff) >> 2`, so index = 52. macreg_writeops[52] is set_ims, which calls set_ics after setting up IMS, which is responsible for setting interrupts, so set_interrupt_cause => pci_set_irq => pci_irq_handler => pci_update_irq_status, set PCI_STATUS([6:7]) in the device configuration space (config) The PCI_STATUS_INTERRUPT bit is set to 1, indicating that the INTx# signal is received.

    For MMIO reading, the process is similar, except that the PCI_STATUS_INTERRUPT bit is set to 0 after reading.



As far as e1000 is concerned, before Linux starts (without any startup information output), the following write operations are performed:

* Write addr[16, 19], which is BAR0, as 4294967295 (0xffffffff)
* Write addr[16, 19], namely BAR0, as 0 (0x0)
* Write addr[16, 19], which is BAR0, as 4237336332 (0xfebc0000)
* Write addr[20, 23], which is BAR1, as 4294967295 (0xffffffff)
* Write addr[20, 23], namely BAR1, as 1(0x1)
* Write addr[20, 23], which is BAR1, as 49152 (0xc000)
* Write addr[4, 5], which is COMMAND, as 259 (0x103, 100000011) to respond to the access of Memory Space and I/O Space. Enable the SERR# driver.

After Linux started, the following write operations were performed:

* Write addr[4, 5], namely COMMAND, as 256 (0x100, 100000000), and enable SERR# driver.
* Write addr[4, 5], which is COMMAND, as 259 (0x103, 100000011) to respond to the access of Memory Space and I/O Space. Enable the SERR# driver.

Then I found that Linux re-checked the length and wrote the value. Write BAR1 as 49153 (0xc001). Visual inspection is to perceive that it is an IO port, and modify its last bit to 1.

Write addr[4, 5], which is COMMAND, as 263 (0x103, 100000111), in response to the access of Memory Space and I/O Space. Enable the SERR# driver. Become a bus master.

After that, in the e1000 workflow, e1000x_rx_ready will be called to check whether it is ready when the package is sent. This also requires reading config[PCI_COMMAND]. Only if it is a bus master can it be considered ready.



### Supplement: locate the config area

We found that the discovery and configuration of devices by BIOS/OS is based on the ability to interact with the configuration space of the device. Therefore, a mechanism is needed to allow BIOS/OS to locate the middle of the configuration of a PCI device, which is the config member of the device in QEMU.

(According to the PCI specification?) This requires two steps. Take writing as an example:

1. Setting goals
    Write the address of the device to be accessed into the config_reg of PCIHostState through pci_host_config_write
2. Setting value stage
    Pass the value to be written into the configuration space through pci_host_data_write, locate the target device through the previously written config_reg, and call the configuration write function corresponding to the device

The following specific analysis:

In the function q35_host_initfn to initialize the host bridge, the MemoryRegion that needs to be accessed when accessing the device config through PIO is set: PCIHostState.data_mem

```c
    memory_region_init_io(&phb->conf_mem, obj, &pci_host_conf_le_ops, phb,
                          "pci-conf-idx", 4);
    memory_region_init_io(&phb->data_mem, obj, &pci_host_data_le_ops, phb,
                          "pci-conf-data", 4);
```

Then bind it to the corresponding port in q35_host_realize:

```c
#define MCH_HOST_BRIDGE_CONFIG_ADDR            0xcf8
#define MCH_HOST_BRIDGE_CONFIG_DATA            0xcfc

static void q35_host_realize(DeviceState *dev, Error **errp)
{
    ...
    sysbus_add_io(sbd, MCH_HOST_BRIDGE_CONFIG_ADDR, &pci->conf_mem);
    sysbus_init_ioports(sbd, MCH_HOST_BRIDGE_CONFIG_ADDR, 4);

    sysbus_add_io(sbd, MCH_HOST_BRIDGE_CONFIG_DATA, &pci->data_mem);
    sysbus_init_ioports(sbd, MCH_HOST_BRIDGE_CONFIG_DATA, 4);
}
```

According to debugging, the MemoryRegion of pci-conf-idx is indeed offset 3320 (0xcfc) and size 4. The offset of pci-conf-data is 3324 (0xcfc), and the size is 4:

```
[struct MemoryRegion *] 0x5555569b70b0 pci-conf-idx size<4> offset<3320>
[struct MemoryRegion *] 0x5555569b71b0 pci-conf-data size<4> offset<3324>
```

Access to this area is to access the config of the PIO device


[kvm_run->io.data_offset, kvm_run->io.data_offset + kvm_run->io.len] is the content to be written

#### Setting the target stage

kvm_handle_io => address_space_rw => address_space_write => address_space_write_continue => memory_region_dispatch_write => access_with_adjusted_size => memory_region_write_accessor => mr->ops->write (pci_host_config_write)

##### address_space_rw
Determine whether to read or write according to direction
addr is port

##### address_space_write

=> mr = address_space_translate(as, addr, &addr1, &l, true) Find MemoryRegion according to addr in AddressSpace, calculate addr1 for the first round
=> address_space_write_continue(as, addr, attrs, buf, len, addr1, l, mr)

So the key here is address_space_translate. It can find the MemoryRegion and addr1 (offset?) of pci-conf-data from address_space_io according to the found address (port)

##### address_space_write_continue
memory_access_size calculates the width of a single write, and then calls

=> result |= memory_region_dispatch_write(mr, addr1, val, 4, attrs); Note that the incoming is addr1 instead of addr
=> address_space_translate calculates the MemoryRegion and addr1 of the next round

Therefore, the total number of calls to address_space_translate (translation times) is the write length/single write width

##### memory_region_dispatch_write
MemoryRegion has a write function, then the accessor is memory_region_write_accessor, which is passed as a parameter

##### access_with_adjusted_size
Call the passed accessor

If size is 4, `r |= access(mr, addr + i, value, access_size, i * 8, access_mask, attrs)` will be called 4 times

Any one of the errors is considered an error

##### memory_region_write_accessor
Use MemoryRegion functions (for example, pci_host_data_le_ops defines write as pci_host_data_write) to manipulate objects stored in MemoryRegion
mr->ops->write(mr->opaque, addr, tmp, size)

##### pci_host_config_write
Convert opaque to PCIHostState and write the value into the config_reg member of PCIHostState.

##### Summary

In the address setting phase, the target port is 3320, so the MemoryRegion pci-conf-idx is found and the value is written into the config_reg member of PCIHostState.

For e1000, the value written here is 2147487760 (0x80001010), which represents the address of the e1000 configuration space.


#### Setting the value phase

kvm_cpu_exec => kvm_handle_io => address_space_rw => address_space_write => address_space_write_continue => memory_region_dispatch_write => access_with_adjusted_size => memory_region_write_accessor => mr->ops->write (pci_host_data_write) => pci_data_write => pci_host_config_write_common => pci_dev->config_write (e1000_write_config) => pci_default_write_config

The previous calling process is the same as setting the target stage, but this time the MemoryRegion is pci-conf-data, so pci_host_data_write is called

##### pci_host_data_write
Convert opaque to PCIHostState, take out PCIBus (s->bus), and convert the address to s->config_reg | (addr & 3)
addr is 0, so the passed parameter addr is s->config_reg


##### pci_data_write
Calculate devfn based on addr. Then find the corresponding PCIDevice from the device array devices of the PCIBus (pcie.0) and then call its config_read / config_write function to operate the address location

##### Summary

In the setting value stage, the target port is 3324, so find the MemoryRegion pci-conf-data, and then read the device address of config_reg in the setting target stage, find the device configuration space according to the address, and then write the value.


#### pci_default_write_config

When setting config, pci_default_write_config will eventually be called. according to:

```c
    if (ranges_overlap(addr, l, PCI_BASE_ADDRESS_0, 24) ||
        ranges_overlap(addr, l, PCI_ROM_ADDRESS, 4) ||
        ranges_overlap(addr, l, PCI_ROM_ADDRESS1, 4) ||
        range_covers_byte(addr, l, PCI_COMMAND))
        pci_update_mappings(d);
```

If the modified [PCI_BASE_ADDRESS_0:PCI_BASE_ADDRESS_0+24] represents BAR0-BAR5, [PCI_ROM_ADDRESS:PCI_ROM_ADDRESS+4], [PCI_ROM_ADDRESS1:PCI_ROM_ADDRESS1+4] represents BAR6, or changes the PCI_mapping_update_mapping to trigger

However, after debugging, the update of PCIIORegion.address in io_regions, that is, the base address, is not updated through pci_update_mappings in the pci_default_write_config of the corresponding BAR, because at this time bit0 and bit1 in PCI_COMMAND are 0, which means that it does not respond to Memory Space and I/O. Space access, so in the process of pci_bar_address conversion address cmd is 0, so PCI_BAR_UNMAPPED will be returned.

Instead, you have to wait until PCI_COMMAND is updated from 256 (0x100) to 259 (0x103), which means that the access to respond to Memory Space and I/O Space is turned on. At this time, pci_update_mappings will also be called to traverse 7 regions and read from config in turn For example, in the previous pci_default_write_config, the base address of BAR0 in the config has been updated to 0xfebc0000, so io_regions[0].address is updated to 0xfebc0000. Then its corresponding MemoryRegion (e1000-mmio) is added to the parent MemoryRegion (pci) as an offset. Trigger the listener of the address space, finally update the flatview, and update to KVM through ioctl.



#### Summary
The PCI device exchanges information with the upper BIOS and OS through its own configuration space. Actually, each field of the configuration space corresponds to a register or ROM in hardware implementation.

When the BIOS/OS starts, it will execute the device discovery logic. When the current device is found, it needs to read the configuration space to obtain relevant information, so VMExit occurs to KVM, and then to QEMU, QEMU will call the config_read set during device initialization to read Device Information. After reading, it will write all 1 detection size to BAR, and then allocate base address to fill in. After filling in, modify the command bit to allow response to memory space and I/O space access.

Then trigger the update of the mapping and set the corresponding IO MemoryRegion to KVM. After that, the OS interacts with the device through PIO/MMIO.




### Mapping area length monitoring

When the BIOS/OS assigns addresses to the BAR, it needs to know the length of the BAR.

According to the PCI specification, BIOS/OS writes all 1s to the corresponding BAR register, and then reads it out to realize the size corresponding to the BAR. For example, if the size is 4k, after writing 0xffffffff, it will read 0xfffff00X. The lowest bit (4bit) is X, which means it is fixed and will not be changed by writing f. As mentioned above, the PCI specification stipulates that the lowest All 4 bits store META information, so they are written to death by the equipment manufacturer and cannot be modified by others:

```
For Memory BARs
0     Region Type       0 = Memory
2-1   Locatable         0 = any 32-bit  /  1 = < 1 MiB  /  2 = any 64-bit
3     Prefetchable      0 = no  /  1 = yes
31-4  Base Address      16-byte aligned

For I/O BARs (Deprecated)
0     Region Type       1 = I/O (deprecated)
1     Reserved
31-2  Base Address      4-byte aligned
```

So after BIOS/OS reads 0xffffff00X, it sets the 4 bits corresponding to X to 0 to get 0xfffff000, and then reverses it to get 0x00000fff, and add 1 to get 0x00001000, so the size is 0x1000, which is 4kb

As for why writing 0xffffffff will read 0xfffff00X, this is guaranteed by hardware characteristics. So in QEMU, how does the simulated device simulate such an effect?

In fact, this is achieved through masks. There are two masks that affect the value of BAR, namely wmask and w1cmask. In the process of device initialization, pci_qdev_realize => do_pci_register_device is called to initialize them:

```c
pci_config_alloc(pci_dev);
pci_init_wmask(pci_dev);
pci_init_w1cmask(pci_dev);
```

The former allocates a piece of memory the size of config for wmask and w1cmask, which is 256 bytes for the PCI device e1000. This means that wmask and w1cmask will cover the entire configuration space and are responsible for writing the entire configuration space as a mask.

The latter initializes some bytes of wmask and w1cmask to ensure that the corresponding position is 1, so as to prevent the config from being masked by the mask when writing. But at this time, the positions corresponding to BAR0-BAR6 are still 0.

Until the register BAR function pci_register_bar, uint64_t wmask = ~(size-1)` is set, and at the end, if the BAR length is 32bit, execute:

```
pci_set_long(pci_dev->wmask + addr, wmask & 0xffffffff);
```

For example, for BAR0 of e1000, the type is 0, and the size is PNPMMIO_SIZE (0x20000), so the local variable wmask is 0xfffffffffffe0000, and since the length of BAR0 is only 32 bits, it needs to be combined with 0xffffffff to get 0xfffe0000. So wmask[16:19] is 00 00 fe ff, which is 0xfffe0000.

So when BIOS/OS writes 0xffffffff to BAR0, it finally calls pci_default_write_config:

```c
void pci_default_write_config(PCIDevice *d, uint32_t addr, uint32_t val_in, int l)
{
    int i, was_irq_disabled = pci_irq_disabled(d);
    uint32_t val = val_in;

    for (i = 0; i < l; val >>= 8, ++i) {
        uint8_t wmask = d->wmask[addr + i];
        uint8_t w1cmask = d->w1cmask[addr + i];
        assert(!(wmask & w1cmask));
        d->config[addr + i] = (d->config[addr + i] & ~wmask) | (val & wmask);
        d->config[addr + i] &= ~(val & w1cmask); /* W1C: Write 1 to Clear */
    }
    if (ranges_overlap(addr, l, PCI_BASE_ADDRESS_0, 24) ||
        ranges_overlap(addr, l, PCI_ROM_ADDRESS, 4) ||
        ranges_overlap(addr, l, PCI_ROM_ADDRESS1, 4) ||
        range_covers_byte(addr, l, PCI_COMMAND))
        pci_update_mappings(d);

    if (range_covers_byte(addr, l, PCI_COMMAND)) {
        pci_update_irq_disabled(d, was_irq_disabled);
        memory_region_set_enabled(&d->bus_master_enable_region,
                                  pci_get_word(d->config + PCI_COMMAND)
                                    & PCI_COMMAND_MASTER);
    }

    msi_write_config(d, addr, val_in, l);
    msix_write_config(d, addr, val_in, l);
}
```

At this time, addr is 16, and length l is 4. So loop 4 times, setting one byte at a time. For each byte, the values ​​of wmask and w1cmask at the corresponding positions will be taken out, so:

1. i=0, wmask = 0 and w1cmask = 0, so config[16] = (0 & 0xffff) | (0xffffffff & 0) = 0
                                    config[16] &= ~(0xffffffff & 0) = 0
2. i=1, wmask = 0, w1cmask = 0, so config[17] = (0 & 0xffff) | (0xffffffff & 0) = 0
                                    config[17] &= ~(0xffffffff & 0) = 0
3. i=1, wmask = 0xfe, w1cmask = 0, so config[18] = (0 & 0x1) | (0xffffffff & 0xfe) = 0xfe
                                        config[18] &= ~(0xffffffff & 0) = 0xfe
4. i=1, wmask = 0xff, w1cmask = 0, so config[19] = (0 & 0) | (0xffffffff & 0xff) = 0xff
                                        config[19] &= ~(0xffffffff & 0) = 0xff

So after the mask, the 0xffffffff to be written is actually 0xfffe0000. Since the read function pci_default_read_config simply does memcpy, the value read by BIOS/OS is 0xfffe0000.

So after getting the BIOS/OS, set the last bit to 0 and invert it to get 0x0001ffff. Then add 1 to get 0x00020000, which is 0x20000, which conforms to the size set by BAR0.

Similarly, for BAR1, its size is IOPORT_SIZE(0x40), wmask[16:19] is c0 ff ff ff, which is 0xffffffc0, so the last actual write is 0xffffffc1, so the BIOS/OS sets the last bit to Reverse after 0 to get 0x0000003f. Then add 1 to get 0x00000040, which is 0x40, which conforms to the size set by BAR0.

After determining the size of the BAR, the BIOS/OS needs to allocate a base address for it, which needs to be aligned with the size of the BAR.


### Summary

BIOS/OS finds the length of BAR by means of write and read, and then assigns base address to it. QEMU realizes the limitation of the maximum value of BAR through the mask mechanism, which simulates the realization on hardware.
