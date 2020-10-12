
This article mainly analyzes the implementation of pvtimer in [https://lkml.org/lkml/2017/12/8/116](https://lkml.org/lkml/2017/12/8/116).

## Original implementation

Linux kernel will set the timer through lapic_next_deadline, that is, set the next timeout point. The original setting is simple:

```c
static int lapic_next_deadline(unsigned long delta,
                   struct clock_event_device *evt)
{
    u64 tsc;

    tsc = rdtsc ();
    wrmsrl(MSR_IA32_TSC_DEADLINE, tsc + (((u64) delta) * TSC_DIVISOR));
    return 0;
}
```

It is to read the current TSC, add the waiting time delta, and write it into MSR_IA32_TSC_DEADLINE.

> TSC-Deadline Mode
> One of the three operation modes of APIC Timer. Write a non-zero 64-bit value to it to activate the Timer, so that an interrupt is triggered when the TSC reaches this value. This interrupt will only be triggered once, and IA32_TSC_DEADLINE_MSR will be reset to zero after being triggered.

If Linux is a guest running on KVM, VMExit is triggered, returns to KVM, and executes `kvm_set_lapic_tscdeadline_msr(vcpu, data)`

```c
void kvm_set_lapic_tscdeadline_msr(struct kvm_vcpu *vcpu, u64 data)
{
    struct kvm_lapic *apic = vcpu->arch.apic;

    if (!lapic_in_kernel(vcpu) || apic_lvtt_oneshot(apic) ||
            apic_lvtt_period(apic))
        return;

    hrtimer_cancel (& apic-> lapic_timer.timer);
    apic->lapic_timer.tscdeadline = data;
    start_apic_timer(apic);
}
```

Then KVM will cancel the current timing on apic->lapic_timer.timer and reset the new timeout time. Note that the timer set by the vCPU will be added to the timer red-black tree of the physical CPU.


The callback function of apic->lapic_timer.timer is set to apic_timer_fn in kvm_create_lapic:


```c
static enum hrtimer_restart apic_timer_fn(struct hrtimer *data)
{
    struct kvm_timer * ktimer = container_of (data, struct kvm_timer, timer);
    struct kvm_lapic *apic = container_of(ktimer, struct kvm_lapic, lapic_timer);

    apic_timer_expired(apic);

    if (lapic_is_periodic(apic)) {
        hrtimer_add_expires_ns(&ktimer->timer, ktimer->period);
        return HRTIMER_RESTART;
    } else
        return HRTIMER_NORESTART;
}
```


### Guest

Each CPU maintains a per-CPU variable pvtimer_shared_buf of type pvtimer_vcpu_event_info. In kvm_guest_cpu_init, its address will be filled in MSR_KVM_PV_TIMER_EN for KVM to fill:

```c
#define MSR_KVM_PV_TIMER_EN 0x4b564d05
```

The address where pvtimer_vcpu_event_info is stored:

```c
struct pvtimer_vcpu_event_info {
    __u64 expire_tsc;
    __u64 next_sync_tsc;
} __attribute__((__packed__));
```

As mentioned above, the original Linux kernel writes the next timeout point in lapic_next_deadline to MSR_IA32_TSC_DEADLINE, VMExit will occur. The essential idea of ​​the patch is to write it in pvtimer_vcpu_event_info, thus avoiding VMExit.

```c
static int lapic_next_deadline(unsigned long delta,
                   struct clock_event_device *evt)
{
    u64 tsc = rdtsc() + (((u64) delta) * TSC_DIVISOR);

    /* TODO: undisciplined function call */
    if (kvm_pv_timer_next_event(tsc, evt))
        return 0;

    wrmsrl(MSR_IA32_TSC_DEADLINE, tsc);
    return 0;
}

static DEFINE_PER_CPU(int, pvtimer_enabled);
static DEFINE_PER_CPU(struct pvtimer_vcpu_event_info,
             pvtimer_shared_buf) = {0};
#define PVTIMER_PADDING        25000

int kvm_pv_timer_next_event(unsigned long tsc,
        struct clock_event_device *evt)
{
    struct pvtimer_vcpu_event_info *src;
    u64 now;

    if (!this_cpu_read(pvtimer_enabled))
        return false;

    /* Write the currently set timeout time to pvtimer_vcpu_event_info.expire_tsc
     * Take out the expire_tsc set last time, if it:
     * 1. Less than pvtimer's next pv_sync_timer timeout time (pvtimer_vcpu_event_info.next_sync_tsc)
     * Indicates that the timer has timed out before KVM actively checks whether there is a timer timeout, so the timeout period needs to be passed through traditional methods.
     * Set MSR_IA32_TSC_DEADLINE to let KVM call kvm_apic_sync_pv_timer immediately
     * 2. If it is less than the current time, it means that it has timed out and the interrupt needs to be triggered as soon as possible. It can only be set in the traditional way and let KVM call kvm_apic_sync_pv_timer immediately
     * 3. In other cases, it has not timed out and no processing is required
     */
    src = this_cpu_ptr(&pvtimer_shared_buf);
    xchg((u64 *)&src->expire_tsc, tsc);

    barrier();

    if (tsc < src->next_sync_tsc)
        return false;

    rdtscll(now);
    if (tsc < now || tsc - now < PVTIMER_PADDING)
        return false;

    return true;
}
```




### KVM

#### Cache initialization

When the guest sets MSR_KVM_PV_TIMER_EN, it will VMExit to KVM and call kvm_lapic_enable_pv_timer to initialize the vcpu->arch.pv_timer structure.

```c
struct {
     u64 msr_val;
     struct gfn_to_hva_cache data;
} pv_timer;

int kvm_lapic_enable_pv_timer(struct kvm_vcpu *vcpu, u64 data)
{
    u64 addr = data & ~KVM_MSR_ENABLED;
    int ret;

    if (!lapic_in_kernel(vcpu))
        return 1;

    if (!IS_ALIGNED(addr, 4))
        return 1;

     // Save the address of pvtimer_vcpu_event_info
    vcpu->arch.pv_timer.msr_val = data;
    if (!pv_timer_enabled(vcpu))
        return 0;

    // Establish GPA to HVA cache
    ret = kvm_gfn_to_hva_cache_init (vcpu-> kvm, & vcpu-> arch.pv_timer.data,
                    addr, sizeof(struct pvtimer_vcpu_event_info));

    return ret;
}
```





#### pvtimer initialization

In kvm_lapic_init, initialize pvtimer through kvm_pv_timer_init:

```c
#define PVTIMER_SYNC_CPU   (NR_CPUS - 1) /* dedicated CPU */
#define PVTIMER_PERIOD_NS  250000L /* pvtimer default period */

static long pv_timer_period_ns = PVTIMER_PERIOD_NS;

static void kvm_pv_timer_init(void)
{
    ktime_t ktime = ktime_set(0, pv_timer_period_ns);

    hrtimer_init(&pv_sync_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
    pv_sync_timer.function = &pv_sync_timer_callback;

    /* kthread for pv_timer sync buffer */
    pv_timer_polling_worker = kthread_create(pv_timer_polling, NULL,
                        "pv_timer_polling_worker/%d",
                        PVTIMER_SYNC_CPU);
    if (IS_ERR(pv_timer_polling_worker)) {
        pr_warn_once("kvm: failed to create thread for pv_timer\n");
        pv_timer_polling_worker = NULL;
        hrtimer_cancel(&pv_sync_timer);

        return;
    }

    kthread_bind(pv_timer_polling_worker, PVTIMER_SYNC_CPU);
    wake_up_process(pv_timer_polling_worker);
    hrtimer_start(&pv_sync_timer, ktime, HRTIMER_MODE_REL);
}
```

Created a high-precision timer pv_sync_timer, using monotonic (monotonic) time, calling pv_sync_timer_callback once after pv_timer_period_ns (default is 250000 ns)

```c
static enum hrtimer_restart pv_sync_timer_callback(struct hrtimer *timer)
{
    // Push the timer's timeout time back pv_timer_period_ns
    hrtimer_forward_now(timer, ns_to_ktime(pv_timer_period_ns));
    // wake up pv_timer_polling_worker
    wake_up_process(pv_timer_polling_worker);

    // Return to restart to indicate that the timer will be activated again
    return HRTIMER_RESTART;
}
```

kvm_pv_timer_init also creates a kernel process named pv_timer_polling_worker/x, where x is the number of the last CPU, which means it will be executed on that CPU (kthread_bind). This thread executes pv_timer_polling.

With pv_sync_timer, it is equivalent to waking up pv_timer_polling_worker every pv_timer_period_ns and executing pv_timer_polling.


```c
static int pv_timer_polling(void *arg)
{
    struct sqm * sqm;
    struct kvm_vcpu *vcpu;
    int i;
    mm_segment_t oldfs = get_fs();

    while (1) {
        // Set to be in interruptible sleep state
        set_current_state(TASK_INTERRUPTIBLE);

        if (kthread_should_stop()) {
            __set_current_state(TASK_RUNNING);
            break;
        }

        spin_lock(&kvm_lock);
        // The setting is in a runnable state, if selected by the scheduler, it will run immediately
        __set_current_state(TASK_RUNNING);
        list_for_each_entry(kvm, &vm_list, vm_list) {
            set_fs(USER_DS);
            use_mm(kvm->mm);
            kvm_for_each_vcpu(i, vcpu, kvm) {
                kvm_apic_sync_pv_timer(vcpu);
            }
            unuse_mm(kvm->mm);
            set_fs(oldfs);
        }

        spin_unlock(&kvm_lock);

        // Take the initiative to give up control to the next process
        schedule();
    }

    return 0;
}
```

This function traverses all VMs and calls kvm_apic_sync_pv_timer on each vCPU. After the call is completed, pv_timer_polling transfers control to the next process through schedule, and waits for the next timeout of pv_sync_timer to wake up.

```c
void kvm_apic_sync_pv_timer(void *data)
{
    struct kvm_vcpu *vcpu = data;
    struct kvm_lapic *apic = vcpu->arch.apic;
    unsigned long flags, this_tsc_khz = vcpu->arch.virtual_tsc_khz;
    u64 guest_tsc, expire_tsc;
    long rem_tsc;

    if (!lapic_in_kernel(vcpu) || !pv_timer_enabled(vcpu))
        return;

    local_irq_save(flags);
    // Get the current actual TSC value of Guest
    guest_tsc = kvm_read_l1_tsc(vcpu, rdtsc());
    // Calculate how many TSC timeouts the pv_sync_timer has
    rem_tsc = ktime_to_ns(hrtimer_get_remaining(&pv_sync_timer))
            * this_tsc_khz;
    if (rem_tsc <= 0)
        rem_tsc += pv_timer_period_ns * this_tsc_khz;
    do_div(rem_tsc, 1000000L);

    /*
     * make sure guest_tsc and rem_tsc are assigned before to update
     * next_sync_tsc.
     */
    smp_wmb ();
    // Write the TSC of the next pv_sync_timer timeout to pvtimer_vcpu_event_info.next_sync_tsc
    kvm_xchg_guest_cached(vcpu->kvm, &vcpu->arch.pv_timer.data,
        offsetof(struct pvtimer_vcpu_event_info, next_sync_tsc),
        guest_tsc + rem_tsc, 8);

    /* make sure next_sync_tsc is visible */
    smp_wmb ();

    // Read pvtimer_vcpu_event_info.expire_tsc and set it to 0
    // At this time expire_tsc stores the next timeout point of the Guest (previously set)
    expire_tsc = kvm_xchg_guest_cached(vcpu->kvm, &vcpu->arch.pv_timer.data,
            offsetof(struct pvtimer_vcpu_event_info, expire_tsc),
            0UL, 8);

    /* make sure expire_tsc is visible */
    smp_wmb ();

    // If the current Guest timer has not expired, set expire_tsc to apic->lapic_timer.tscdeadline and set the timer
    // Equivalent to the operation performed by KVM after Guest writes MSR_IA32_TSCDEADLINE and finds VMExit
    // If it has timed out, directly inject APIC_LVTT interrupt
    if (expire_tsc) {
        if (expire_tsc > guest_tsc)
            /*
             * As we bind this thread to a dedicated CPU through
             * IPI, the timer is registered on that dedicated
             * CPU here.
             */
            kvm_set_lapic_tscdeadline_msr(apic->vcpu, expire_tsc);
        else
            /* deliver immediately if expired */
            kvm_apic_local_deliver(apic, APIC_LVTT);
    }
    local_irq_restore(flags);
}
```

In addition, the processing when writing MSR_IA32_TSCDEADLINE is modified, from the original direct `kvm_set_lapic_tscdeadline_msr(vcpu, data);` to

```c
if (pv_timer_enabled(vcpu))
    smp_call_function_single(PVTIMER_SYNC_CPU,
            kvm_apic_sync_pv_timer, vcpu, 0);
else
    kvm_set_lapic_tscdeadline_msr(vcpu, data);
```

Of course, if you find that the timer set by Guest has timed out in kvm_apic_sync_pv_timer, you still call kvm_set_lapic_tscdeadline_msr to set apic->lapic_timer.timer.

apic->lapic_timer. When timer expires, APIC_LVTT interrupt will be sent. If timer is in TSC deadline mode, set deadline to 0 as required by the specification

```c
static enum hrtimer_restart apic_timer_fn(struct hrtimer *data) {
    struct kvm_timer * ktimer = container_of (data, struct kvm_timer, timer);
    struct kvm_lapic *apic = container_of(ktimer, struct kvm_lapic, lapic_timer);

    if (pv_timer_enabled(apic->vcpu)) {
        kvm_apic_local_deliver(apic, APIC_LVTT);
        if (apic_lvtt_tscdeadline(apic))
            apic->lapic_timer.tscdeadline = 0;
    } else
        apic_timer_expired(apic);
}
```


## to sum up

The original way to set the APIC timer: Guest writes the time-out timestamp (tsc-deadline timestamp) to MSR_IA32_TSC_DEADLINE to trigger VMExit. KVM will set the timer on the physical CPU.

pvtimer sets the timeout timestamp that was originally set to MSR_IA32_TSCDEADLINE to pvtimer_vcpu_event_info (expire_tsc).

KVM creates a thread, checks pvtimer_vcpu_event_info.expire_tsc regularly, and directly injects a timeout interrupt when it is timed out, and calls kvm_set_lapic_tscdeadline_msr before it times out. It turned out that VMExit occurred after the MSR was set, and the corresponding handler in KVM was called. Now it becomes a regular check. If it is set, it is called. Reduced VMExit.
