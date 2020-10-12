
This article is to [Shoot4U: Using VMM Assists to Optimize TLB Operations on Preempted vCPUs
](http://dl.acm.org/citation.cfm?id=2892245) / [patch](https://github.com/ouyangjn/shoot4u) study notes.

This method improves performance by flushing the TLB directly in the VMM instead of waiting until the corresponding vCPU is scheduled. At the same time, using Individual-address invalidation, you can only swipe the specified address.

## Background

__invvpid is already defined in kvm:

```c
static inline void __invvpid(int ext, u16 vpid, gva_t gva)
{
    struct {
    u64 vpid: 16;
    u64 rsvd : 48;
    u64 gva;
    } operand = { vpid, 0, gva };

    asm volatile (__ex(ASM_VMX_INVVPID)
          /* CF==1 or ZF==1 --> rc = -1 */
          "; ja 1f ; ud2 ; 1:"
          : : "a"(&operand), "c"(ext) : "cc", "memory");
}
```

The adjustment is the INVVPID instruction, which is used to invalidate the TLB and page cache based on the VPID. Receive two parameters, vpid and memory address

There are four modes:

* Individual-address invalidation(type=0)

    For tags where the tag is VPID and the address is the specified address (parameters are passed in)

* Single-context invalidation(type=1)

    For tags with VPID

* All-contexts invalidation(type=2)

    For all except vpid 0000H (should be VMM)

* Single-context invalidation, retaining global translations(type=3)

    For TLB whose tag is VPID, but keep global translations



It turns out that VMX_VPID_EXTENT_SINGLE_CONTEXT(1) and VMX_VPID_EXTENT_ALL_CONTEXT(2) have been used


```c
static inline void vpid_sync_vcpu_single(struct vcpu_vmx *vmx)
{
        if (vmx->vpid == 0)
                return;

        if (cpu_has_vmx_invvpid_single())
                __invvpid(VMX_VPID_EXTENT_SINGLE_CONTEXT, vmx->vpid, 0);
}

static inline void vpid_sync_vcpu_global(void)
{
        if (cpu_has_vmx_invvpid_global())
                __invvpid(VMX_VPID_EXTENT_ALL_CONTEXT, 0, 0);
}
```

## HOST modification (KVM)

### arch/x86/include/asm/vmx.h

Added 0 operations. I.e. Individual-address invalidation

```c
#define VMX_VPID_EXTENT_INDIVIDUAL_ADDR        0
#define VMX_VPID_EXTENT_INDIVIDUAL_ADDR_BIT      (1ull << 8) /* (40 - 32) */
```

### arch/x86/kvm/vmx.c


Three new operations have been added after tlb_flush

```diff
    .tlb_flush = vmx_flush_tlb,

+   .tlb_flush_vpid_single_ctx = vmx_flush_tlb_single_ctx,
+   .tlb_flush_vpid_single_addr = vmx_flush_tlb_single_addr,
+   .get_vpid = vmx_get_vpid,
```

In the current version, it exists in vmx_x86_ops, which stores all operations supported by the vmx platform. This array will be passed in as a parameter during KVM initialization (kvm_init) and stored in kvm_x86_ops. Equivalent to registering the function.


#### vmx_get_vpid

Read the current vpid from the vcpu_vmx structure. vcpu_vmx contains kvm_vcpu, which represents a vcpu under the vmx platform.


```c
static inline int vmx_get_vpid(struct kvm_vcpu *vcpu)
{
        struct vcpu_vmx *vmx = container_of(vcpu, struct vcpu_vmx, vcpu);
        return vmx->vpid;
}
```

### vmx_flush_tlb_single_ctx

The old method, single/all, erase all

```c
static void vmx_flush_tlb_single_ctx(struct kvm_vcpu *vcpu)
{
    vpid_sync_context(to_vmx(vcpu));
}
```

### vmx_flush_tlb_single_addr

Try to swipe a single address.

vmx_capability saves the information read from MSR MSR_IA32_VMX_EPT_VPID_CAP, where vpid is placed in the upper 32 bits, so it is actually the 8th bit of reading vmx_capability.vpid


```c
static inline bool cpu_has_vmx_invvpid_addr(void)
{
    return vmx_capability.vpid & VMX_VPID_EXTENT_INDIVIDUAL_ADDR_BIT;
}


static inline void vpid_sync_addr(struct vcpu_vmx *vmx, unsigned long addr)
{
    if (vmx->vpid == 0)
        return;

    // If vcpu supports new features, invalidate the address alone
    if (cpu_has_vmx_invvpid_addr())
        __invvpid(VMX_VPID_EXTENT_INDIVIDUAL_ADDR, vmx->vpid, addr);
    else
    // Otherwise use the old method, single/all
        vpid_sync_context(vmx);
}

static void vmx_flush_tlb_single_addr(struct kvm_vcpu *vcpu, unsigned long addr)
{
    // judge
    vpid_sync_addr(to_vmx(vcpu), addr);
}

```




### include / uapi / linux / kvm_para.h

```c
#define KVM_HC_SHOOT4U         12
```


Added in kvm_emulate_hypercall:

```diff
+   case KVM_HC_SHOOT4U:
+       kvm_pv_shoot4u_op(vcpu, a0, a1, a2);
+ ret = 0;
+       break;
```

A new call type is registered, and it is processed by kvm_pv_shoot4u_op when it is called. kvm_pv_shoot4u_op will call vmx_flush_tlb_single_ctx/vmx_flush_tlb_single_addr above according to the defined mode.


### arch / x86 / sqm / x86.c

```c
/* lapic timer advance (tscdeadline mode only) in nanoseconds */
#define SHOOT4U_MODE_DEFAULT   0
#define SHOOT4U_MODE_TEST1     1
#define SHOOT4U_MODE_TEST2     2
#define SHOOT4U_MODE_TEST3     3
unsigned int shoot4u_mode = SHOOT4U_MODE_DEFAULT;
module_param(shoot4u_mode, uint, S_IRUGO | S_IWUSR);
```

Different modes can be turned on:

* 0 Brush off the entire tlb
* 1 Remove the tlb of vpid
* 2 If there is an end address, flash the entire tlb, otherwise try to flash a single address
* 3 If there is an end address, flash the tlb of vpid, otherwise try to flash a single address

It is suspected that the designated area is not currently supported and can only be swiped individually?


```c
/*
 * kvm_pv_shoot4u_op:  Handle tlb shoot down hypercall
 *
 * @apicid - apicid of vcpu to be kicked.
 */

// The current vcpu, the bitmap set by the vcpu contained in the VM, the start and end addresses to be invalidated
static void kvm_pv_shoot4u_op(struct kvm_vcpu *vcpu, unsigned long vcpu_bitmap,
        unsigned long start, unsigned long end)
{
    struct kvm_shoot4u_info info;
    struct kvm *kvm = vcpu->kvm;
    struct kvm_vcpu *v;
    int i;

    info.flush_start = start;
    info.flush_end = end;

    //printk("[shoot4u] inside hypercall handler\n");
    // construct phsical cpu mask from vcpu bitmap
    // For each vcpu in the VM except yourself, check whether it is in the bitmap, if it is, call flush_tlb_func_shoot4u to flush out the tlb above him
    kvm_for_each_vcpu(i, v, kvm) {
        if (v != vcpu && test_bit(v->vcpu_id, (void*)&vcpu_bitmap)) {
            info.vcpu = v;
            //printk("[shoot4u] before send IPI to vcpu %d at pcpu %d\n", v->vcpu_id, v->cpu);
            // it is fine if a vCPU migrates because migratation triggers tlb_flush automatically
            smp_call_function_single(v->cpu, flush_tlb_func_shoot4u, &info, 1);
        }
    }
}

struct kvm_shoot4u_info {
    struct kvm_vcpu *vcpu;
    unsigned long flush_start;
    unsigned long flush_end;
};


// Cross-processor operation
/* shoot4u host IPI handler with invvipd */
static void flush_tlb_func_shoot4u(void *info)
{
    struct kvm_shoot4u_info *f = info;

    //printk("[shoot4u] IPI handler at pCPU %d: invalidate vCPU %d\n", smp_processor_id(), f->vcpu->vcpu_id);
    if (shoot4u_mode == SHOOT4U_MODE_DEFAULT) {
        // all (linear + EPT mappings)
        kvm_x86_ops->tlb_flush(f->vcpu);
    } else if (shoot4u_mode == SHOOT4U_MODE_TEST1) {
        // all (linear mappings only)
        kvm_x86_ops->tlb_flush_vpid_single_ctx(f->vcpu);
    } else if (shoot4u_mode == SHOOT4U_MODE_TEST2) {
        // single or all (linear + EPT mappings)
        if (!f->flush_end)
            kvm_x86_ops->tlb_flush_vpid_single_addr(f->vcpu, f->flush_start);
        else {
            kvm_x86_ops->tlb_flush(f->vcpu);
        }
    } else if (shoot4u_mode == SHOOT4U_MODE_TEST3) {
        // seg fault
        // single or all (linear mappings only)
        if (!f->flush_end)
            kvm_x86_ops->tlb_flush_vpid_single_addr(f->vcpu, f->flush_start);
        else {
            kvm_x86_ops->tlb_flush_vpid_single_ctx(f->vcpu);
        }
    } else {
        ...
    }

    return;
}
```


## Guest

```diff
+#ifdef CONFIG_SHOOT4U
+        pv_mmu_ops.flush_tlb_others = shoot4u_flush_tlb_others;
+#endif
```

Use kvm_hypercall to request other people's tlb

Currently does not support more than 64 vcpu, because it uses long to store, only 64bit

```c
void shoot4u_flush_tlb_others(const struct cpumask *cpumask,
                struct mm_struct *mm, unsigned long start,
                unsigned long end)
{
    // shoot4u currently uses an 8 bytes bitmap to pass target cores
    // thus it supports up to 64 physical cores
    u64 cpu_bitmap = 0;
    int cpu;

    // Set the vcpu to be flushed into the bitmap
    for_each_cpu(cpu, cpumask) {
        if (cpu >= 64) {
            panic("[shoot4u] ERROR: do not support more than 64 cores\n");
        }
        set_bit(cpu, (void *)&cpu_bitmap);
    }

    //printk("[shoot4u] before KVM_HC_SHOOT4U hypercall, cpumask: %llx, start %lx, end %lx\n", cpu_bitmap, start, end);
    kvm_hypercall3(KVM_HC_SHOOT4U, cpu_bitmap, start, end);
}

```

