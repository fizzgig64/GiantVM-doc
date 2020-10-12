## Basic

### Page table
Responsible for converting VA to PA. The address of VA is composed of the page number and the offset within the page. When converting, first read the starting address of the page table from the base address register (CR3) of the page table, and then add the page number to get the page table entry of the corresponding page . Take the physical address of the page from it and add the offset to get the PA.

With the expansion of the addressing range (64-bit CPU supports 48-bit virtual address addressing space, and 52-bit physical address addressing space), page tables need to occupy more and more contiguous memory space, plus each Processes must have their own page tables, and the system needs to consume a lot of memory just to maintain the page tables. To this end, the use of the localized characteristics of the memory used by the program, the introduction of multi-level page tables.

The current version of Linux uses four-level page tables:

Page Map Level 4(PML4) => Page Directory Pointer Table(PDPT) => Page Directory(PD) => Page Table(PT)

It is called in some places: Page Global Directory(PGD) => Page Upper Directory(PUD) => Page Middle Directory(PMD) => Page Table(PT)

Under x86_64, the size of an ordinary page is 4KB. Since the address is 64bit, a page table entry occupies 8 Bytes, so a page table can only store 512 entries. Therefore, each level of page table index uses 9 bits, and the page index (offset) uses 12 bits, so only 0-47 bits in a 64-bit address are used.

In 64-bit, EPT uses the same structure as the traditional page table, so if you do not consider the TLB, a GVA to HVA needs to go through 4 * 4 times (considering the case of access to each level of page fault) page table query.

The memory is accessed as many times as there are queries, and continuous access to the memory during the walk will undoubtedly affect performance. To this end, TLB (Translation Lookaside Buffer) is introduced to cache commonly used PTEs. In this way, in the case of a TLB hit, there is no need to search the memory. Utilizing the localized characteristics of the memory used by the program, the hit rate of TLB is often very high, which improves the access speed under the multi-level page table.




### Memory virtualization
QEMU uses the mmap system call to apply for a continuous size of space in the virtual address space of the process as the guest's physical memory.

Under this architecture, there are four levels of mapping for memory address access:

GVA - GPA - HVA - HPA

The GVA-GPA mapping is maintained by the guest OS, and the HVA-HPA is maintained by the host OS. So we need a mechanism to maintain the GPA-HVA mapping. Commonly used implementations are SPT (Shadow Page Table) and EPT/NPT. The former maintains the shadow page table through software, and the latter implements secondary mapping through hardware features.


### Shadow page table
KVM realizes direct mapping by maintaining the page table SPT from GVA to HPA. Then the page table can be addressed and used by the physical MMU. How to achieve it:

KVM sets the page table of the Guest OS to read-only. When the Guest OS is modified, it will trigger a page fault, VMEXIT to KVM. KVM will check the access permissions of the page table entries corresponding to the GVA, and judge based on the error code:

1. If it is caused by Guest OS, inject the exception back. Guest OS calls its own page fault processing function (apply for a page, and fill the GPA of the page into the upper-level page table entry)
2. If it is caused by the inconsistency between the page table of the Guest OS and the SPT, synchronize the SPT, find the mapping relationship between GVA to GPA and then to HVA according to the Guest OS page table and mmap mapping, and then add/update the GVA-HPA table in the SPT item

When the Guest OS switches the process, it loads the base address of the page table of the process to be switched into CR3 of the Guest, causing VM EXIT back to KVM. KVM finds the corresponding SPT through the hash table, and then loads it into CR3 of the machine.

Disadvantages: It is necessary to maintain an SPT for each process, which brings additional memory overhead. Need to keep the guest OS page table and SPT synchronized. Whenever a guest page fault occurs, even if it is caused by the guest's own page fault, it will cause VMExit, which is expensive.


### EPT / NPT
Intel EPT technology introduces the concepts of EPT (Extended Page Table) and EPTP (EPT base pointer). The EPT maintains the mapping from GPA to HPA, and the EPT base pointer is responsible for pointing to the EPT. When the Guest OS is running, the EPT address corresponding to the VM is loaded into EPTP, and the base address of the currently running process page table of the Guest OS is loaded into CR3, so when the address is converted, the GVA is first realized through the page table pointed to by CR3. The conversion of GPA, and then the conversion from GPA to HPA is realized through the EPT pointed to by EPTP.

When EPT page fault occurs, VMExit to KVM is required to update EPT.

AMD NPT (Nested Page Table) is a solution developed by AMD. Its principle is similar to EPT, but the description and implementation are slightly different. Both Guest OS and Host have their own CR3. When the address is converted, the page table pointed to by gCR3 is from GVA to GPA, and then the page table pointed to by nCR3 is from GPA to HPA.

Advantages: Guest's page faults are handled in the guest, and vm exit will not be performed. The address conversion is basically done by the hardware (MMU) looking up the page table.

Disadvantages: Two-level page table query can only be expected to hit TLB.




## Implementation

### QEMU

#### Memory device simulation

##### PCDIMMDevice

```c
typedef struct PCDIMMDevice {
    /* private */
    DeviceState parent_obj;

    /* public */
    uint64_t addr; // starting GPA mapped to
    uint32_t node; // the numa node mapped to
    int32_t slot; // The number of the inserted memory slot, the default is -1, which means automatic allocation
    HostMemoryBackend *hostmem; // corresponding backend
} PCDIMMDevice;
```

The virtual memory bar defined by QOM (qemu object model). It can be managed through QMP or QEMU command line. By adding/removing this object, the memory in the VM can be hot swapped.


##### HostMemoryBackend

```c
struct HostMemoryBackend {
    /* private */
    Object parent;

    /* protected */
    uint64_t size; // Provide memory size
    bool merge, dump;
    bool prealloc, force_prealloc, is_mapped;
    DECLARE_BITMAP(host_nodes, MAX_NODES + 1);
    HostMemPolicy policy;

    MemoryRegion mr; // Owned MemoryRegion
};
```

A section of Host memory defined by QOM provides memory for virtual memory sticks. It can be managed through QMP or QEMU command line.


#### Memory initialization

On the premise that KVM is turned on, QEMU initializes the memory through the following process:


```
main => configure_accelerator => kvm_init => kvm_memory_listener_register(s, &s->memory_listener, &address_space_memory, 0) 初始化
kvm_state.memory_listener
                                          => kml->listener.region_add = kvm_region_add sets the operation for the listener
                                          => memory_listener_register initializes the listener and binds to address_space_memory
                                          => memory_listener_register(&kvm_io_listener, &address_space_io) initialize kvm_io_listener and bind to address_space_io
     => cpu_exec_init_all => memory_map_init creates two global MemoryRegion system_memory("system") and system_io("io")
                                 => address_space_init initialize address_space_memory("memory") and address_space_io("I/O") AddressSpace, and use system_memory and system_io as root
                                    => memory_region_transaction_commit commits the modification, causing a change in the address space
```

Before proceeding further analysis, we first introduce the three structures involved: AddressSpace, MemoryRegion, and MemoryRegionSection:

#### AddressSpace

```c
struct AddressSpace {
    /* All fields are private. */
    struct rcu_head rcu;
    char *name;
    MemoryRegion *root;
    int ref_count;
    bool malloced;

    /* Accessed via RCU.  */
    struct FlatView *current_map; // points to the currently maintained FlatView, compared as old in address_space_update_topology

    int ioeventfd_nb;
    struct MemoryRegionIoeventfd *ioeventfds;
    struct AddressSpaceDispatch *dispatch; // Responsible for finding HVA based on GPA
    struct AddressSpaceDispatch *next_dispatch;
    MemoryListener dispatch_listener;
    QTAILQ_HEAD(memory_listeners_as, MemoryListener) listeners;
    QTAILQ_ENTRY(AddressSpace) address_spaces_link;
};
```

As the name implies, it is used to represent a piece of address space of a virtual machine, such as memory address space and IO address space. Each AddressSpace generally contains a series of MemoryRegion: The root of the AddressSpace points to the root-level MemoryRegion, and the MemoryRegion may have its own several subregions, thus forming a tree structure.

As mentioned above, memory_map_init is called in the memory initialization process, which initializes address_space_memory and address_space_io, where:

* address_space_memory 的 root 为 system_memory
* The root of address_space_io is system_io



#### MemoryRegion

```c
struct MemoryRegion {
    Object parent_obj; // inherited from Object

    /* All fields are private - violators will be prosecuted */

    /* The following fields should fit in a cache line */
    bool romd_mode;
    bool ram;
    bool subpage;
    bool readonly; /* For RAM regions */
    bool rom_device; // read-only
    bool flush_coalesced_mmio;
    bool global_locking;
    uint8_t dirty_log_mask; // dirty map type
    RAMBlock *ram_block; // Point to the corresponding RAMBlock
    Object *owner;
    const MemoryRegionIOMMUOps *iommu_ops;

    const MemoryRegionOps *ops;
    void *opaque;
    MemoryRegion *container; // points to the parent MemoryRegion
    Int128 size; // Memory area size
    hwaddr addr; // The offset in the parent MemoryRegion (see memory_region_add_subregion_common)
    void (*destructor)(MemoryRegion *mr);
    uint64_t align;
    bool terminates;
    bool ram_device;
    bool enabled;
    bool warning_printed; /* For reservations */
    uint8_t vga_logging_count;
    MemoryRegion *alias; // points to the entity MemoryRegion
    hwaddr alias_offset; // The offset of the starting address (GPA) in the entity MemoryRegion
    int32_t priority;
    QTAILQ_HEAD(subregions, MemoryRegion) subregions;                   // subregion 链表
    QTAILQ_ENTRY(MemoryRegion) subregions_link;
    QTAILQ_HEAD(coalesced_ranges, CoalescedMemoryRange) coalesced;
    const char *name;
    unsigned ioeventfd_nb;
    MemoryRegionIoeventfd *ioeventfds;
    QLIST_HEAD(, IOMMUNotifier) iommu_notify;
    IOMMUNotifierFlag iommu_notify_flags;
};
```

MemoryRegion represents a section of memory in the Guest memory layout and has logical (Guest) meaning.

In the process of initializing the VM, the corresponding MemoryRegion is established:

```
pc_init1 / pc_q35_init => pc_memory_init => memory_region_allocate_system_memory initialize MemoryRegion and allocate memory for it
                                         => memory_region_init_alias => memory_region_init              初始化 alias MemoryRegion
                                         => memory_region_init initialize MemoryRegion
                                         => memory_region_init_ram => memory_region_init initialize MemoryRegion and allocate Ramblock
```


##### memory_region_allocate_system_memory

For non-NUMA VMs, allocate memory directly

```
=> allocate_system_memory_nonnuma => memory_region_init_ram_from_file / memory_region_init_ram allocate MemoryRegion corresponding to Ramblock memory
=> vmstate_register_ram sets the idstr of RAMBlock according to the name of the region
```

For NUMA, HostMemoryBackend needs to be set after allocation

```
=> memory_region_init
=> memory_region_add_subregion traverse the memory HostMemoryBackend of all NUMA nodes, and use those mr members that are not empty as subregions of the current MemoryRegion, and the offset starts from 0 to increase
=> vmstate_register_ram_global => vmstate_register_ram sets the idstr of RAMBlock according to the name of the region
```

##### MemoryRegion type

MemoryRegion can be divided into the following three types:

* Root-level MemoryRegion: Directly initialized by memory_region_init, without its own memory, used to manage subregions. Such as system_memory
* Entity MemoryRegion: initialized by memory_region_init_ram, has its own memory (allocated from the QEMU process address space), and the size is size. Such as ram_memory(pc.ram), pci_memory(pci), etc.
* Alias ​​MemoryRegion: initialized by memory_region_init_alias, without its own memory, representing a part of the entity MemoryRegion (such as pc.ram), pointing to the entity MemoryRegion through the alias member, alias_offset is the offset in the entity MemoryRegion. Such as ram_below_4g, ram_above_4g, etc.

The common MemoryRegion relationship in the code is:

```
                  alias
ram_memory (pc.ram) - ram_below_4g(ram-below-4g)
                    - ram_above_4g(ram-above-4g)

             alias
system_io (me) - (pci0-io)
              - (isa_mmio)
              - (isa-io)
              - ...

                     sub
system_memory(system) - ram_below_4g(ram-below-4g)
                      - ram_above_4g(ram-above-4g)
                      -pcms->hotplug_memory.mr hot plug memory

          sub
rom_memory - isa_bios(isa-bios)
           - option_rom_mr(pc.rom)

```

At the same time map AddressSpace to FlatView, get several MemoryRegionSections, call kvm_region_add, register MemoryRegionSection in KVM.


##### MemoryRegionSection

```c
struct MemoryRegionSection {
    MemoryRegion *mr; // Point to the corresponding MemoryRegion
    AddressSpace *address_space;                // 所属 AddressSpace
    hwaddr offset_within_region; // The offset of the starting address (HVA) in the MemoryRegion
    Int128 size;
    hwaddr offset_within_address_space; // The offset within AddressSpace, if the AddressSpace is system memory, it is the GPA starting address
    bool readonly;
};
```

MemoryRegionSection points to a part of MemoryRegion ([offset_within_region, offset_within_region + size]), which is the basic unit of registration to KVM.

After mapping the MemoryRegion in AddressSpace to the linear address space, due to the overlap relationship, the original complete region may be divided into fragments, so MemoryRegionSection is generated.

Looking back at the process of memory initialization, the job done is very simple: create some AddressSpaces and bind listeners. Create the corresponding MemoryRegion as the root of the AddressSpace. Finally, submit the modification to change the address space and update it to KVM. The following will be divided into points.



##### KVMMemoryListener

During the initialization process, we registered memory_listener and kvm_io_listener for address_space_memory and address_space_io, respectively. The former type is KVMMemoryListener, and the latter type is MemoryListener:

```c
typedef struct KVMMemoryListener {
    MemoryListener listener;
    KVMSlot *slots;
    int as_id;
} KVMMemoryListener;

struct MemoryListener {void (*begin)(MemoryListener *listener);
    void (*commit)(MemoryListener *listener);
    void (*region_add)(MemoryListener *listener, MemoryRegionSection *section);
    void (*region_del)(MemoryListener *listener, MemoryRegionSection *section);
    void (*region_nop)(MemoryListener *listener, MemoryRegionSection *section);
    void (*log_start)(MemoryListener *listener, MemoryRegionSection *section,
                      int old, int new);
    void (*log_stop)(MemoryListener *listener, MemoryRegionSection *section,
                     int old, int new);
    void (*log_sync)(MemoryListener *listener, MemoryRegionSection *section);
    void (*log_global_start)(MemoryListener *listener);
    void (*log_global_stop)(MemoryListener *listener);
    void (*eventfd_add)(MemoryListener *listener, MemoryRegionSection *section,
                        bool match_data, uint64_t data, EventNotifier *e);
    void (*eventfd_del)(MemoryListener *listener, MemoryRegionSection *section,
                        bool match_data, uint64_t data, EventNotifier *e);
    void (*coalesced_mmio_add)(MemoryListener *listener, MemoryRegionSection *section,
                               hwaddr addr, hwaddr len);
    void (*coalesced_mmio_del)(MemoryListener *listener, MemoryRegionSection *section,
                               hwaddr addr, hwaddr len);
    /* Lower = earlier (during add), later (during del) */
    unsigned priority;
    AddressSpace *address_space;
    QTAILQ_ENTRY(MemoryListener) link;
    QTAILQ_ENTRY(MemoryListener) link_as;
};
```

You can see that the main body of the KVMMemoryListener is the MemoryListener, and the MemoryListener contains a large number of function pointers to point to the callback function called when the address_space member changes.

kvm_io_listener and dispatch_listener are tied to address_space_io. Therefore, there is a one-to-many relationship between AddressSpace and listener. When AddressSpace changes, all listeners bound to it will be triggered. How is this achieved?

In fact, any operation on AddressSpace and MemoryRegion starts with memory_region_transaction_begin and ends with memory_region_transaction_commit.

These operations include: enabling, destructuring, adding and deleting eventfd, adding and deleting subregions, changing attributes (flag), setting size, opening dirty log, etc., such as:

* memory_region_add_subregion
* memory_region_del_subregion
* memory_region_set_readonly
* memory_region_set_enabled
* memory_region_set_size
* memory_region_set_address
* memory_region_set_alias_offset
* memory_region_readd_subregion
* memory_region_update_container_subregions
* memory_region_set_log
* memory_region_finalize
* ...

Operate the root MemoryRegion of AddressSpace:

* address_space_init
* address_space_destroy

##### memory_region_transaction_begin

```
=> qemu_flush_coalesced_mmio_buffer => kvm_flush_coalesced_mmio_buffer
=> ++memory_region_transaction_depth
```

Some MMIO batch optimizations are made in KVM: When KVM encounters MMIO and VMEXIT, it records the MMIO operation in the kvm_coalesced_mmio structure, and then stuffs it into kvm_coalesced_mmio_ring without exiting to QEMU. Until one time you return to QEMU, the moment before you update the memory space, take out kvm_coalesced_mmio in kvm_coalesced_mmio_ring and do it again to ensure memory consistency. This is what kvm_flush_coalesced_mmio_buffer does.


##### memory_region_transaction_commit

```
=> --memory_region_transaction_depth
=> If memory_region_transaction_depth is 0 and memory_region_update_pending is greater than 0
    => MEMORY_LISTENER_CALL_GLOBAL(begin, Forward) calls the begin function of all listeners in the global list memory_listeners from front to back
    => For all address spaces in address_spaces, call address_space_update_topology to update the slot information maintained in QEMU and KVM.
    => MEMORY_LISTENER_CALL_GLOBAL(commit, Forward) calls the commit function of all listeners in the global list memory_listeners from back to front
```

Call the function corresponding to the listener to update the address space.

##### address_space_update_topology

```
=> address_space_get_flatview                             获取原来 FlatView(AddressSpace.current_map)
=> generate_memory_topology generates a new FlatView
=> address_space_update_topology_pass compares the new and old FlatView, and performs corresponding operations on the inconsistent FlatRange.
```

Since AddressSpace is a tree structure, address_space_update_topology is called to map (flatten) the tree structure to a linear address space using the FlatView model. Compare the new and old FlatView, perform the corresponding operation for the inconsistent FlatRange, and finally operate the KVM.

##### generate_memory_topology

```
=> addrrange_make creates an address space with a start address of 0 and an end address of 2^64 as the guest's linear address space
=> render_memory_region starts from the root-level region, and recursively maps the region to the linear address space to generate a FlatRange to form a FlatView
=> flatview_simplify merges the continuous FlatRange in FlatView into one
```

The root member of AddressSpace is the root MemoryRegion of the address space, and generate_memory_topology is responsible for flattening its tree structure so that it can be mapped to a linear address space to obtain FlatView.

##### address_space_update_topology_pass

Compare the new and old FlatRange of the AddressSpace to see if there is any change. If so, traverse the listeners of the AddressSpace from front to back or from back to front and call the corresponding callback function.

```
=> MEMORY_LISTENER_UPDATE_REGION => section_from_flat_range Constructs MemoryRegionSection according to the range of FlatRange
                                 => MEMORY_LISTENER_CALL
```

For example, as mentioned earlier, in the initialization process, kvm_state.memory_listener is registered as the listener of address_space_memory, and it will be added to the listeners of AddressSpace. So if address_space_memory changes, the call will call the corresponding function in memory_listener.

For example, the callback parameter passed in by MEMORY_LISTENER_UPDATE_REGION is region_add, then memory_listener.region_add (kvm_region_add) is called.



##### kvm_region_add

```
=> kvm_set_phys_mem => kvm_lookup_overlapping_slot
                    => Calculate the starting HVA
                    => kvm_set_user_memory_region => kvm_vm_ioctl(s, KVM_SET_USER_MEMORY_REGION, &mem)
```

kvm_lookup_overlapping_slot is used to determine whether the address range (GPA) of the new region section overlaps with the existing KVMSlot (kml->slots). If it overlaps, it needs to be processed:

Assuming that the original slot can be divided into three parts: prefix slot + overlap slot + suffix slot, the overlap area is overlap

For complete overlap, there are both prefix slot and suffix slot. No need to register a new slot.

For partial overlap, prefix slot = 0 or suffix slot = 0. Then perform the following process:

1. Delete the original slot
2. Register the prefix slot or suffix slot
3. Register overlap slot

Of course, if there is no overlap, just register the new slot directly. Then update the slot to the corresponding kvm_memory_slot in KVM through kvm_vm_ioctl(s, KVM_SET_USER_MEMORY_REGION, &mem).

The slot structure maintained in QEMU also needs to be updated. For the original slot, because it is an item of the kml->slots array, it can be modified directly in kvm_set_phys_mem. For slots that are not in kml->slots, such as prefix, suffix, and overlap, you need to call kvm_alloc_slot => kvm_get_free_slot, it will find a blank (memory_size = 0) in kml->slots and return to the slot, and then perform the slot Set up.

##### kvm_set_phys_mem => kvm_set_user_memory_region

KVM stipulates that the parameter for updating the memory slot is kvm_userspace_memory_region:

```c
struct kvm_userspace_memory_region {
    __u32 slot; // corresponds to the id of kvm_memory_slot
    __u32 flags;
    __u64 guest_phys_addr;                                                  // GPA
    __u64 memory_size; /* bytes */                                          // 大小
    __u64 userspace_addr; /* start of the userspace allocated memory */     // HVA
};
```

It will be calculated and filled in the process of kvm_set_phys_mem => kvm_set_user_memory_region. The process is as follows:

1. According to the starting HVA of the region (memory_region_get_ram_ptr) + the offset of the region section in the region (offset_within_region) + page alignment correction (delta) to get the real starting HVA of the section, and fill in userspace_addr

    In memory_region_get_ram_ptr, if the current region is the alias of another region, it will be traced upwards until it reaches the non-alias region (entity region). Add the alias_offset in the traceback process to get the offset of the current region in the entity region.

    Since the entity region has a corresponding RAMBlock, call qemu_map_ram_ptr to add the host and total offset of the RAMBlock corresponding to the entity region to get the starting HVA of the current region.

2. According to the offset of the region section in the AddressSpace (offset_within_address_space) + page alignment correction (delta), get the real GPA of the section, and fill in start_addr

3. Get the real size of the section according to the size of the region section (size)-page alignment correction (delta), and fill in memory_size





### RAMBlock

As mentioned earlier, MemoryRegion represents a segment of memory in the guest memory layout, which has a logical meaning. So the actual meaning is that who maintains the actual memory information corresponding to this piece of memory?

We can find that there is a ram_block member in MemoryRegion, which is a pointer of RAMBlock type. RAMBlock is responsible for maintaining the actual memory information, such as HVA and GPA. For example, in the process of calculating userspace_addr just now, calculating the starting HVA of the region needs to find the corresponding RAMBlock, and then get its host member to get it.

RAMBlock is defined as follows:

```c
struct RAMBlock {
    struct rcu_head rcu; // Used to protect Read-Copy-Update
    struct MemoryRegion *mr; // corresponding MemoryRegion
    uint8_t *host; // corresponding HVA
    ram_addr_t offset; // Offset in the address space of ram_list (to add up the size of the previous block)
    ram_addr_t used_length; // currently used length
    ram_addr_t max_length; // total length
    void (*resized)(const char*, uint64_t length, void *host);  // resize 函数
    uint32_t flags;
    /* Protected by iothread lock.  */
    char idstr[256];                                            // id
    /* RCU-enabled, writes protected by the ramlist lock */
    QLIST_ENTRY(RAMBlock) next; // Point to the next block in ram_list.blocks
    int fd; // file descriptor of the mapped file
    size_t page_size; // page size, generally consistent with host
};
```

As mentioned earlier, MemoryRegion will call memory_region_* to initialize the MemoryRegion structure. Common functions are as follows:

* memory_region_init_ram => qemu_ram_alloc
    RAMBlock.host created by qemu_ram_alloc is NULL

* memory_region_init_ram_from_file => qemu_ram_alloc_from_file
    The RAMBlock created by qemu_ram_alloc_from_file will call file_ram_alloc to allocate memory using the (device) file of the corresponding path, usually due to the need to use hugepage, and the hugepage device file (such as /dev/hugepages) will be specified through the `-mem-path` parameter

* memory_region_init_ram_ptr => qemu_ram_alloc_from_ptr
    RAMBlock.host is the pointer address passed in, which means that memory is allocated from the memory pointed to by this address

* memory_region_init_resizeable_ram => qemu_ram_alloc_resizeable
    RAMBlock.host is NULL, but resizeable is true, indicating that the memory has not been allocated, but it can be resized.


qemu_ram_alloc_* (qemu_ram_alloc / qemu_ram_alloc_from_file / memory_region_init_ram_ptr / memory_region_init_resizeable_ram) will eventually be called to qemu_ram_alloc_internal => ram_block_add. If it finds that host is NULL, it will call phys_mem_alloc (qemu_anon_ram_alloc) to allocate memory. After making the host point to something, insert the RAMBlock into ram_list.blocks.


##### qemu_anon_ram_alloc

=> qemu_ram_mmap(-1, size, QEMU_VMALLOC_ALIGN, false) => mmap

Use mmap to allocate memory of size in the process address space of QEMU.





### RAMList

ram_list is a global variable that maintains all RAMBlocks in the form of a linked list.

```c
RAMList ram_list = {.blocks = QLIST_HEAD_INITIALIZER(ram_list.blocks) };

typedef struct RAMList {
    QemuMutex mutex;
    RAMBlock *mru_block;
    /* RCU-enabled, writes protected by the ramlist lock. */
    QLIST_HEAD(, RAMBlock) blocks;                              // RAMBlock 链表
    DirtyMemoryBlocks *dirty_memory[DIRTY_MEMORY_NUM]; // Record dirty page information for VGA / TCG / Live Migration
    uint32_t version; // add 1 for every change
} RAMList;
extern RAMList ram_list;
```

Note:

* VGA: Graphics card emulation tracks dirty video memory through dirty_memory for redrawing the interface
* TCG: The dynamic translator tracks the self-adjusted code through dirty_memory, and recompiles it when the upstream instruction changes
* Live Migration: Live migration tracks the dirty page through dirty_memory, and retransmits it after the dirty page is changed




##### AddressSpaceDispatch

according to:

```
address_space_init => address_space_init_dispatch => as->dispatch_listener = (MemoryListener) {
                                                                            .begin = mem_begin,
                                                                            .commit = mem_commit,
                                                                            .region_add = mem_add,
                                                                            .region_nop = mem_add,
                                                                            .priority = 0,
                                                                        };
                                                  => memory_listener_register(as->dispatch_listener)
```

In addition to kvm_state.memory_listener bound to address_space_memory, dispatch_listener is also created and bound. The listener is implemented in order to find the corresponding HVA according to the GPA when the virtual machine exits.

When memory_region_transaction_commit calls the begin function of each listener, mem_begin is called

```
=> g_new0(AddressSpaceDispatch, 1) Create AddressSpaceDispatch structure as the next_dispatch member of AddressSpace
```

The AddressSpaceDispatch structure is as follows:

```c
struct AddressSpaceDispatch {
    struct rcu_head rcu;

    MemoryRegionSection *mru_section;
    /* This is a multi-level map on the physical address space.
     * The bottom level has pointers to MemoryRegionSections.
     */
    PhysPageEntry phys_map;
    PhysPageMap map; // GPA -> HVA mapping, realized through multi-level page tables
    AddressSpace *as;
};
```

The map member is a multi-level (6 levels) page table, and the last level page table points to MemoryRegionSection.

When address_space_update_topology_pass => address_space_update_topology_pass is processing add, mem_add is called:

So call register_subpage / register_multipage to register the page in the page table.

```
=> If the subpage of the MemoryRegion to which the MemoryRegionSection belongs does not exist
    => subpage_init create subpage
    => phys_page_set => phys_map_node_reserve Allocation page directory entry
                     => phys_page_set_level fills the page table, from L5 to L0
=> if it exists
    => container_of(existing->mr, subpage_t, iomem)         取出
=> subpage_register                                         设置 subpage
```

Therefore, after exiting from KVM to QEMU, you can find the corresponding MemoryRegionSection through AddressSpaceDispatch.map, and then find the corresponding HVA



## KVM


### kvm_vm_ioctl_set_memory_region

Add memory. Called when KVM receives the ioctl of KVM_SET_USER_MEMORY_REGION (replaces KVM_SET_MEMORY_REGION because it does not support fine-grained control).

The incoming parameters are as follows:

```c
struct kvm_userspace_memory_region {
    __u32 slot; // corresponds to the id of kvm_memory_slot
    __u32 flags;
    __u64 guest_phys_addr;                                                  // GPA
    __u64 memory_size; /* bytes */                                          // 大小
    __u64 userspace_addr; /* start of the userspace allocated memory */     // HVA
};
```

flags optional:

* KVM_MEM_LOG_DIRTY_PAGES declares that it needs to track writes to the Region and read when it is provided to KVM_GET_DIRTY_LOG
* KVM_MEM_READONLY If readonly (KVM_CAP_READONLY_MEM) is supported, VMEXIT (KVM_EXIT_MMIO) will be triggered when the Region is written

于是 kvm_vm_ioctl_set_memory_region => kvm_set_memory_region => __kvm_set_memory_region

This function will judge user operations based on npages (the number tree contained in the region) and the original npages:

#### KVM_MR_CREATE
Now there is a page but there is no original, to add a new memory area, create and initialize the slot.

#### KVM_MR_DELETE
Now that there is no page but there is, to delete the memory area, mark the slot as KVM_MEMSLOT_INVALID

#### KVM_MR_FLAGS_ONLY / KVM_MR_MOVE
Now that there are pages and there are in the past, it is to modify the memory area. If only the flag is changed, it is KVM_MR_FLAGS_ONLY. Currently, it is only possible to KVM_MEM_LOG_DIRTY_PAGES. Choose whether to create or release the dirty_bitmap according to the flag.

If the GPA changes, it is KVM_MR_MOVE and needs to be moved. In fact, simply mark the original slot as KVM_MEMSLOT_INVALID, and then add a new one.

The newly added/modified slots are updated through install_new_memslots.

#### kvm_memory_slot

The slot operated in __kvm_set_memory_region is the basic unit of memory management in KVM and is defined as follows:

```c
struct kvm_memory_slot {
    gfn_t base_gfn; // starting gfn of slot
    unsigned long npages;               // page 数
    unsigned long *dirty_bitmap;        // 脏页 bitmap
    struct kvm_arch_memory_slot arch; // Structure related, including rmap and lpage_info etc.
    unsigned long userspace_addr; // corresponding starting HVA
    u32 flags;
    short id;
};


struct kvm_arch_memory_slot {struct kvm_rmap_head *rmap[KVM_NR_PAGE_SIZES]; // back link
    struct kvm_lpage_info *lpage_info[KVM_NR_PAGE_SIZES-1]; // Maintain the next level page table whether to close hugepage
    unsigned short *gfn_track[KVM_PAGE_TRACK_MAX];
};
```

The slot is saved in kvm->memslots[as_id]->memslots[id], where as_id is the address space id. In fact, there is only one address space in the usual architecture, and as_id always takes 0. Only x86 requires two address spaces. as_id = 0 is a normal address space, as_id = 1 is a dedicated SRAM space for SMM mode, and id is slot id. The memory of these structures is allocated in kvm_create_vm. Initialize it here.


### Memory Management Unit (MMU)

#### Initialization

```
kvm_init => kvm_arch_init => kvm_mmu_module_init => create mmu_page_header_cache as cache
                                                 => register_shrinker(&mmu_shrinker) register recovery function


kvm_vm_ioctl_create_vcpu =>
kvm_arch_vcpu_create => kvm_x86_ops->vcpu_create (vmx_create_vcpu) => init_rmode_identity_map creates an equivalent map of 1024 pages for real mode
                                                                   => kvm_vcpu_init => kvm_arch_vcpu_init => kvm_mmu_create
kvm_arch_vcpu_setup => kvm_mmu_setup => init_kvm_mmu => init_kvm_tdp_mmu If two dimentional paging (EPT) is supported, initialize it and set the attributes and functions in vcpu->arch.mmu
                                                     => init_kvm_softmmu => kvm_init_shadow_mmu otherwise initialize SPT
```


##### kvm_mmu_create

Initialize mmu related information with vcpu as the unit. Their related definitions in vcpu include:

```c
struct kvm_vcpu_arch {
    ...
    /*
     * Paging state of the vcpu
     *
     * If the vcpu runs in guest mode with two level paging this still saves
     * the paging mode of the l1 guest. This context is always used to
     * handle faults.
     */
    struct kvm_mmu mmu;

    /*
     * Paging state of an L2 guest (used for nested npt)
     *
     * This context will save all necessary information to walk page tables
     * of the an L2 guest. This context is only initialized for page table
     * walking and not for faulting since we never handle l2 page faults on
     * the host.
     */
    struct kvm_mmu nested_mmu;

    /*
     * Pointer to the mmu context currently used for
     * gva_to_gpa translations.
     */
    struct kvm_mmu *walk_mmu;

    // The following is cache, used to improve the allocation speed of commonly used data structures
    // Used to allocate pte_list_desc, which is a linked list item of the reverse mapping linked list parent_ptes, which is allocated in mmu_set_spte => rmap_add => pte_list_add
    struct kvm_mmu_memory_cache mmu_pte_list_desc_cache;
    // Used to allocate page, as kvm_mmu_page.spt
    struct kvm_mmu_memory_cache mmu_page_cache;
    // Used to allocate kvm_mmu_page as a page table page
    struct kvm_mmu_memory_cache mmu_page_header_cache;
    ...
}
```

The cache is used to improve the allocation speed of commonly used data structures in the page table. These caches will call mmu_topup_memory_caches when MMU is initialized (kvm_mmu_load), page fault (tdp_page_fault), etc., to ensure that each cache is sufficient.

```c
// Ensure that each cache is sufficient
static int mmu_topup_memory_caches(struct kvm_vcpu *vcpu)
{
    // If r is not 0, it means that the allocation of /__get_free_page from slab fails and an error is returned directly
    int r;
    // If vcpu->arch.mmu_pte_list_desc_cache is insufficient, allocate from pte_list_desc_cache
    r = mmu_topup_memory_cache(&vcpu->arch.mmu_pte_list_desc_cache,
                   pte_list_desc_cache, 8 + PTE_PREFETCH_NUM);
    if (r)
        goto out;
    // If vcpu->arch.mmu_page_cache is insufficient, allocate directly through __get_free_page
    r = mmu_topup_memory_cache_page(&vcpu->arch.mmu_page_cache, 8);
    if (r)
        goto out;
    // If vcpu->arch.mmu_page_header_cache is insufficient, allocate from mmu_page_header_cache
    r = mmu_topup_memory_cache(&vcpu->arch.mmu_page_header_cache,
                   mmu_page_header_cache, 4);
out:
    return r;
}
```

Two global slab caches, pte_list_desc_cache and mmu_page_header_cache, are created in kvm_mmu_module_init as the cache sources of vcpu->arch.mmu_pte_list_desc_cache and vcpu->arch.mmu_page_header_cache.

You can view the allocated slab through `cat /proc/slabinfo` on the host:

```
# name            <active_objs> <num_objs> <objsize> <objperslab> <pagesperslab> : tunables <limit> <batchcount> <sharedfactor> : slabdata <active_slabs> <num_slabs> <sharedavail>
kvm_mmu_page_header    576    576    168   48    2 : tunables    0    0    0 : slabdata     12     12      0
```



#### Load page table

kvm_vm_ioctl_create_vcpu is only to initialize mmu, for example, set vcpu->arch.mmu.root_hpa to INVALID_PAGE, and this value is not really set until entering the VM (VMLAUNCH/VMRESUME).

```
vcpu_enter_guest => kvm_mmu_reload => kvm_mmu_load => mmu_topup_memory_caches Ensure that each cache is sufficient
                                                   => mmu_alloc_roots => mmu_alloc_direct_roots If the root page table does not exist, allocate a kvm_mmu_page
                                                   => vcpu->arch.mmu.set_cr3 (vmx_set_cr3) For EPT, load the HPA of the spt(strcut page) of the page into VMCS
                                                                                                    For SPT, load the HPA of the spt (strcut page) of the page to cr3
                 => kvm_x86_ops->run (vmx_vcpu_run)
                 => kvm_x86_ops->handle_exit (vmx_handle_exit)
```


#### kvm_mmu_page

Page table page, see Documentation/virtual/kvm/mmu.txt for detailed explanation

```c
struct kvm_mmu_page {
    struct list_head link; // Add to kvm->arch.active_mmu_pages or invalid_list, indicating the state of the current page
    struct hlist_node hash_link; // Add to vcpu->kvm->arch.mmu_page_hash to provide quick search

    /*
     * The following two entries are used to key the shadow page in the
     * hash table.
     */
    gfn_t gfn; // gfn corresponding to the starting address of the management address range
    union kvm_mmu_page_role role; // Basic information, including hardware characteristics and level, etc.

    u64 *spt; // Point to the address of struct page, which contains all page table entries (pte). At the same time page->private will point to the kvm_mmu_page
    /* hold the gfn of each spte inside spt */
    gfn_t *gfns; // gfn corresponding to all page table entries (pte)
    bool unsync; // Used for the last-level page table page, indicating whether the page table entry (pte) of the page is synchronized with the guest (whether the guest has updated tlb)
    int root_count; /* Currently serving as active root */ // Used for the most advanced page table page, count how many EPTP points to itself
    unsigned int unsync_children; // the number of unsynced pte in the page table
    struct kvm_rmap_head parent_ptes; /* rmap pointers to parent sptes */ // Reverse mapping (rmap), maintain the upper-level page table entries that point to yourself

    /* The page is obsolete if mmu_valid_gen != kvm->arch.mmu_valid_gen.  */
    unsigned long mmu_valid_gen; // Algebra, if it is smaller than kvm->arch.mmu_valid_gen, it means it is invalid

    DECLARE_BITMAP(unsync_child_bitmap, 512); // spte bitmap of unsync in the page table

#ifdef CONFIG_X86_32
    /*
     * Used out of the mmu-lock to avoid reading spte values while an
     * update is in progress; see the comments in __get_spte_lockless().
     */
    int clear_spte_count; // Under 32bit, the modification of spte is atomic, so the count is used to detect whether it is being modified, if it is modified, redo is required
#endif

    /* Number of writes since the last time traversal visited this page.  */
    atomic_t write_flooding_count; // Count the number of emulation since the last use, if it exceeds a certain number of times, the page will be unmapped
};

union kvm_mmu_page_role {
    unsigned word;
    struct {
        unsigned level: 4; // The level of the page
        unsigned cr4_pae:1; // cr4.pae, 1 means use 64bit gpte
        unsigned quadrant: 2; // If cr4.pae=0, the gpte is 32bit, but the spte is 64bit, so multiple sptes are needed to represent a gpte, and this field indicates which part of the gpte is
        unsigned direct:1;
        unsigned access:3; //Access authority
        unsigned invalid:1; // Invalid, once unpin it will be destroyed
        unsigned hot: 1; // efer.nxe
        unsigned cr0_wp:1; // cr0.wp, write protection
        unsigned smep_andnot_wp:1;  // cr4.smep && !cr0.wp
        unsigned smap_andnot_wp:1;  // cr4.smap && !cr0.wp
        unsigned :8;

        /*
         * This is left at the top of the word so that
         * kvm_memslots_for_spte_role can extract it with a
         * simple shift.  While there is room, give it a whole
         * byte so it is also faster to load it from memory.
         */
        unsigned smm:8; // in system management mode
    };
};
```


#### EPT Violation

When a guest visits a page for the first time, since there is no GVA to GPA mapping, a page fault of the Guest OS is triggered. Then Guest OS will establish the corresponding pte and repair the page tables at all levels, and finally access the corresponding GPA. Since no GPA to HVA mapping is established, EPT Violation, VMEXIT to KVM is triggered. KVM executes kvm_vmx_exit_handlers[exit_reason] in vmx_handle_exit and finds that exit_reason is EXIT_REASON_EPT_VIOLATION, so it calls handle_ept_violation.

##### handle_ept_violation

```
=> vmcs_readl(EXIT_QUALIFICATION) Get the reason for EPT exit. EXIT_QUALIFICATION is a supplement to Exit reason, see Vol. 3C 27-9 Table 27-7 for details
=> vmcs_read64(GUEST_PHYSICAL_ADDRESS) Get the GPA of the page fault
=> Get error_code according to exit_qualification content, it may be read fault / write fault / fetch fault / ept page table is not present
=> kvm_mmu_page_fault => vcpu->arch.mmu.page_fault (tdp_page_fault)
```

##### tdp_page_fault

```
=> gfn = gpa >> PAGE_SHIFT shift GPA to the right pagesize to get gfn(guest frame number)
=> mapping_level Calculates the level of gfn in the page table, regardless of hugepage, it is L1
=> try_async_pf converts gfn to pfn (physical frame number)
        => kvm_vcpu_gfn_to_memslot => __gfn_to_memslot Find the slot corresponding to gfn
        => __gfn_to_pfn_memslot find the pfn corresponding to gfn
                => __gfn_to_hva_many => __gfn_to_hva_memslot Calculate the starting HVA corresponding to gfn
                => hva_to_pfn Calculate the pfn corresponding to HVA and make sure that the physical page is in memory

=> __direct_map Update EPT, adding new mapping relationship to EPT layer by layer
    => for_each_shadow_entry starting from level4(root), complete the page table layer by layer, for each layer:
        => mmu_set_spte For the page table of level 1, its page table entry is definitely missing, so there is no need to judge directly to fill in the starting hpa of pfn
        => is_shadow_present_pte If the next level page table page does not exist, that is, the current page table entry has no value (*sptep = 0)
            => kvm_mmu_get_page allocates a page table page structure
            => link_shadow_page Fill the HPA of the new page table into the current page table entry (sptep)
```

It can be found that there are two main steps. The first step is to obtain the physical page corresponding to the GPA, if not, it will be allocated. The second step is to update the EPT.

##### try_async_pf
1. Find the corresponding memslot according to gfn
2. Use memslot's initial HVA (userspace_addr) + (gfn-initial gfn (base_gfn) in slot) * page size (PAGE_SIZE) to get the initial HVA corresponding to gfn
3. Allocate a physical page for the HVA. There are two types: hva_to_pfn_fast and hva_to_pfn_slow. hva_to_pfn_fast actually calls __get_user_pages_fast and tries to pin the page, which means that the physical page at the address is in memory. If it fails and degenerates to hva_to_pfn_slow, it will first get the lock of mm->mmap_sem and then call __get_user_pages to pin.
4. If the allocation is successful, call page_to_pfn on the returned struct page to get the corresponding pfn

This function establishes a mapping from gfn to pfn, and at the same time dies the page pin in the host's memory.


##### __direct_map

Complete the page table related to the GPA in the EPT through the iterator kvm_shadow_walk_iterator.

```c
struct kvm_shadow_walk_iterator {
    u64 addr; // In the case of a page fault GPA, the iterative process is to fill in all the page table items involved in the GPA
    hpa_t shadow_addr; // The HPA of the current page table entry is set to vcpu->arch.mmu.root_hpa in shadow_walk_init
    u64 *sptep; // Point to the current page table entry, updated in shadow_walk_okay
    int level; // The current level, set to 4 in shadow_walk_init (x86_64 PT64_ROOT_LEVEL), and subtract 1 in shadow_walk_next
    unsigned index; // The index in the current level page table, updated in shadow_walk_okay
};
```

In each iteration, sptep will point to the page table entry corresponding to the GPA in the current level page table. Our purpose is to fill the GPA of the next level page table into the page table entry (ie set *sptep). Because it is a page fault, there may be a problem that the next-level page table page does not exist. At this time, a page table page needs to be allocated, and then the GPA of the page is filled in *sptep.

For example, for GPA (such as 0xfffff001), the binary is:

```
000000000 000000011 111111111 111111111 000000000001
  PML4      PDPT       PD        PT        Offset
```

Initialization status: level = 4, shadow_addr = root_hpa, addr = GPA

Implementation process:

1. index = the value of addr in the current level. Such as 0 (000000000) when level = 4, 3 (000000011) when level = 3
2. sptep = va(shadow_addr) + index, get the page table entry HVA corresponding to GPA in the current address
3. If *sptep has no value, assign a page as the lower-level page table, and set *sptep as the HPA of the page
4. shadow_addr = *sptep, enter the lower-level page table, loop

When hugepage is enabled, because the scope of page table item management becomes larger, the number of page table levels required is reduced. By default, the page size is 2M, so level 1 is not required.



##### mmu_set_spte

```
=> set_spte => mmu_spte_update => mmu_spte_set => __set_spte Set the physical page (pfn) starting HPA to *sptep, that is, set the value of a pte in the last-level page table
=> rmap_add => page_header(__pa(spte)) Get the page table page where spetp is located
            => kvm_mmu_page_set_gfn set gfn to the gfns of the page table page
            => gfn_to_rmap => __gfn_to_memslot Get the slot corresponding to gfn
                           => __gfn_to_rmap => gfn_to_index Use gfn and slot->base_gfn to calculate the index of the page in the slot
                                    => slot->arch.rmap[level-PT_PAGE_TABLE_LEVEL][idx] Take out the corresponding rmap from this slot
            => pte_list_add adds the address of the current item (spetp) to rmap for reverse mapping
```

Acts on the level 1 page table (PT). Responsible for setting the value of pte(*spetp) in the last-level page table, and adding the address of the current item (spetp) to slot->arch.rmap[level-PT_PAGE_TABLE_LEVEL][idx] as a reverse mapping. Quickly find the kvm_mmu_page through gfn.

In most cases, gfn corresponds to a single kvm_mmu_page, so rmap_head directly points to spetp. However, because one gfn corresponds to multiple kvm_mmu_pages, in this case, rmap uses linked list + array to maintain. A linked list item pte_list_desc can store three spetp. Since pte_list_desc is frequently allocated, it is also allocated from the cache (vcpu->arch.mmu_pte_list_desc_cache).


##### kvm_mmu_get_page

Get the kvm_mmu_page corresponding to gfn. It will try to find the corresponding page table page from vcpu->kvm->arch.mmu_page_hash through gfn. If the page has been allocated before, just return directly. Otherwise, it needs to be allocated from the cache through kvm_mmu_alloc_page, and then added to vcpu->kvm->arch.mmu_page_hash with gfn as the key.

kvm_mmu_alloc_page will allocate kvm_mmu_page and page objects from vcpu->arch.mmu_page_header_cache and vcpu->arch.mmu_page_cache through mmu_memory_cache_alloc. In mmu_topup_memory_caches, these global variables are guaranteed to be sufficient. If the slab is found to be insufficient, it will be supplemented Also mentioned earlier.



##### link_shadow_page

```
=> mmu_spte_set => __set_spte is the value of the current page table entry (*spetp) Set the HPA of the next level page table page
=> mmu_page_add_parent_pte => pte_list_add Add the address of the current item (spetp) to the parent_ptes of the next page table page, and do reverse mapping
```

Acting on the 2-4 level page table (PML4-PDT), if the next level page table is found to be missing during the traversal process, you need to update the page table entry (spetp) pointed to by the current iterator after allocating a page table page. Set to the HPA of the page table page at the next level, so that the page table page can be accessed through the page table entry next time. At the same time, the address of the current page table entry (spetp) needs to be added to the parent_ptes of the next-level page table page as a reverse mapping.

Using two sets of reverse mappings, after shifting the GPA to the right to calculate the gfn, the page table entries in L1 can be obtained through rmap, and then the page table entries in L2-4 can be obtained in turn through parent_ptes. When the Host needs to swap out the page of a certain GPA of the Guest, it directly finds the page table entry related to the gfn through the inverted index and modifies it without having to go through the EPT query again.






## to sum up

### QEMU
Create a series of MemoryRegion, respectively representing the ROM, RAM and other areas in the Guest. MemoryRegion maintains the relationship between each other through alias or subregion, so as to further refine the definition of the region.

For an entity MemoryRegion (non-alias), its corresponding RAMBlock will be created in the process of initializing the memory. RAMBlock allocates memory from the process space of QEMU through mmap, and is responsible for maintaining the starting HVA/GPA/size information of the MemoryRegion management memory.

AddressSpace represents the physical address space of the VM. If the MemoryRegion in the AddressSpace changes, the listener is triggered to flatten the MemoryRegion tree of the AddressSpace to which it belongs to form a one-dimensional FlatView, and compare whether the FlatRange has changed. If it is to call the corresponding method such as region_add to check the changed section region, update the KVMSlot in QEMU, and fill in the kvm_userspace_memory_region structure at the same time, update the kvm_memory_slot in KVM as an ioctl parameter.


### KVM
When QEMU creates vcpu through ioctl, it calls kvm_mmu_create to initialize mmu related information and allocates slab cache for the page table entry structure.

Before KVM enters the guest, vcpu_enter_guest => kvm_mmu_reload will load the root page table address into VMCS, and let the guest use the page table.

When EPT Violation occurs, VMEXIT to KVM. If the page is missing, get the corresponding GPA, calculate the gfn according to the GPA, find the corresponding memory slot according to the gfn, and get the corresponding HVA. Then find the corresponding pfn according to the HVA and make sure that the page is in the memory. After filling in the missing pages, the EPT needs to be updated to complete the missing page table entries. So starting from L4, the page table is completed layer by layer. For the page table pages that are missing on a certain layer, the HPA of the new page will be filled into the upper level page table after allocation from the slab.

In addition to establishing the association between the upper-level page table and the lower-level page table, KVM will also establish a reverse mapping, which can directly find the gfn-related page table entries based on the GPA without having to go through the EPT query again.
