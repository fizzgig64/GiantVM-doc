# Linux Interrupt

## Facts of Hardware

 On x86 CPUs, interrupts need to be distributed through the entries in the IDT. If the interrupt processing routine is entered through the Interrupt Gate, the IF (Interrupt Flag) of EFLAGS will be cleared (equivalent to executing the `CLI` instruction) to disable the interrupt , IF will not be reset until the execution of the `IRET` instruction returns from the interrupt processing routine. If the trap gate is passed, no operation will be performed on the IF.

## Linux

In Linux, after entering an interrupt processing routine, the interrupt will be reopened immediately, but the interrupt itself will be shielded on all CPUs to prevent its own reentry. Therefore, in Linux, interrupt handling routines need not be reentrant.

### Junior Affinity

`/proc/interrupts` shows the number of interrupts processed by each CPU. `/proc/irq/IRQ#/smp_affinity` is a bitmap, which indicates which CPUs the IRQ# irq should be sent to. Each bit represents a CPU. If more than one CPU is allocated to a certain irq, then The lowest priority mode is used, and the hardware selects the lowest priority CPU in this group of CPUs as the interrupt destination. The priority can usually be changed by setting the TPR register of LAPIC.

SMP affinity can also be queried and specified by `/proc/irq/IRQ#/smp_affinity_list`, which is a CPU list with the form of `2`, `1-7`, etc. SMP affinity is 32-bit, 64-bit or 128-bit wide. The default value is usually all 1s (that is, all CPUs can receive it). The default value can be queried through `/proc/irq/default_smp_affinity`

In practice, each IRQ is usually bound to a separate core instead of a group of cores, because the same interrupt is better to handle locality on the same core. And the lowest priority mode is not perfect. It only guarantees that one of a group of CPUs can always be selected by arbitration, and it does not guarantee load balance between this group of CPUs. For example, when a group of CPUs have the same priority, it may The CPU selected each time is the same.

Unfortunately, Linux does not set the CPU TPR:

```c
// arch/x86/kernel/apic/apic.c
// The following snippet proves that Linux will only set TPR to 0 and no longer change

/*
 * Set Task Priority to 'accept all'. We never change this
 * later on.
 */
value = apic_read(APIC_TASKPRI);
value &= ~APIC_TPRI_MASK;
apic_write(APIC_TASKPRI, value);
```

Therefore, when smp affinity sets more than one CPU for an IRQ, often only one CPU is processing interrupts.

---

The `irqbalance` tool developed by Red Hat is usually used to manage the smp affinity. This tool can analyze the load of the system and modify the smp affinity at regular intervals (10 seconds by default). It binds each IRQ to a CPU, and dynamically changes the binding according to the load, so that the IRQ is evenly distributed among the CPUs. It also has a Power save mode. When the system interrupt load is found to be low, it will bind all interrupts to the same CPU, reducing the work of other CPUs and allowing other CPUs to enter sleep mode.

---

In the Linux kernel, the work of setting smp affinity is implemented in `kernel/irq/manage.c`, and the key function is `setup_affinity(irq_desc, cpumask)`. This function calls the ʻirq_do_set_affinity` function, which then further calls the ʻirq_set_affinity` method in the `struct irq_chip` to call the affinity setting code of the specific hardware.

Finally, the bitmap of `smp_affinity` is passed into the `cpu_mask_to_apicid_and` method of `struct apic` as the `cpumask` parameter to get the APIC Destination field to fill in the hardware registers. Each configuration method of APIC corresponds to a `struct apic` object, so there is a `cpu_mask_to_apicid_and` implementation.

### APIC initialization

An APIC driver (`struct apic` object) is selected when Linux is started. Under 32-bit, it is implemented by the `generic_apic_probe` function of `arch/x86/kernel/apic/probe_32.c`, which will call each APIC driver object. The `probe` method is used to query whether it is available. In 64-bit mode, the `default_acpi_madt_oem_check` function of `arch/x86/kernel/apic/probe_64.c` is implemented. This function will call the `acpi_madt_oem_check` method of each APIC driver object, according to the ACPI table Select the driver. The order in which the APIC driver is checked is the compilation order specified in the Makefile.

From the compilation order, the system will give priority to x2APIC. In x2APIC and xAPIC modes, the default mode is Logical + Lowest Priority mode, unless the system does not support it and can only choose Physical + Fixed mode.

> **Inference:** For Physical + Fixed mode, the `cpu_mask_to_apicid_and` method will select the first online CPU found in `cpumask` as the APIC Destination. Therefore, if APIC is configured in this mode, `smp_affinity` will degenerate to a fixed selection of a CPU in a group.
