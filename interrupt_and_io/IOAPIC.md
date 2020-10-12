# IOAPIC

IOAPIC is called 82093AA I/O Advanced Programmable Interrupt Controller. According to this model, you can find its [Manual](https://pdos.csail.mit.edu/6.828/2016/readings/ia32/ioapic.pdf).

IOAPIC is accessed through MMIO, and there are actually only two 32-bit registers available for access, namely I/O Register Select (IOREGSEL, only the lower 8 bits are valid) and I/O Window (IOWIN), and their physical addresses are 0xFEC0xy00 and 0xFEC0xy10. (Among them, x and y can be configured through APIC Base Address Relocation Register). The former provides Index, and its 0-7th bits represent the register number to be accessed, and the latter provides Data for reading and writing the IOAPIC register to be accessed.

> **Info:** APIC Base Address Relocation Register is a register on the PIIX3 chip. PIIX3 is Intel's South Bridge in the late 1990s. Later, South Bridge cancelled the register after entering the ICH series, but later added a new configuration register , Change the addresses of the two registers to 0xFEC0x000 and 0xFEC0x010 (where x is configurable)

## Registers

- IOAPIC ID (IOAPICID), located at Index 0x0, bits 24-27 represent IOAPIC ID, used to identify IOAPIC
- IOAPIC Version (IOAPICVER), located at Index 0x1, read only
  - Bits 0-7 indicate APIC Version, and the value should be 0x11
  - Bits 16-23 indicate Maximum Redirection Entry, that is, the number of entries in the Redirection Table -1, and the value should be 0x17 (ie 23)
- IOAPIC Arbitration ID (IOAPICARB), located at Index 0x2, read-only, bits 24-27 represent Arb ID, used for APIC Bus arbitration
  - The communication between LAPIC and between LAPIC and IOAPIC is through APIC Bus. Each LAPIC and IOAPIC has an Arb ID for arbitration. The one with the highest Arb ID wins and sets its own ID to 0. The rest of the APIC The Arb ID is increased by one (except for those with the original Arb ID of 15, the Arb ID should be set to the original Arb ID value of the winner).
    > **Note:** APIC Bus is the technology used by Pentium and P6 family. Starting from Pentium 4, the communication between LAPIC and between LAPIC and IOAPIC is through the system bus and does not use Arb ID for arbitration, so Arb is now discussed. ID has no meaning.
- Redirection Table (IOREDTBL[0:23]), located at Index 0x10-0x3F (64 bits each), responsible for configuring interrupt forwarding functions, each of which is referred to as RTE (Redirection Table Entry)

## Interrupt Redirection

When an interrupt comes to IOAPIC, it will be sent to the CPU(s) according to the Redirection Table. The contents of the RTE entry in this forwarding table are as follows:

- The 56th-63th digits are Destination, which represents the destination CPU(s), the Physical mode is APIC ID, and the Logical mode is MDA
  - In Physical mode, only digits 56-59 are valid, and digits 60-63 must be 0
- The 16th bit is Mask, 0 means accepting interrupts is allowed, 1 means forbidden, the initial value is 1 after reset
- The 15th bit is Trigger Mode, 0 means edge triggered, 1 means level triggered
- The 14th bit is Remote IRR, which is read-only and only meaningful for level triggered interrupts. Take 1 to indicate that the target CPU has accepted the interrupt. After receiving the EOI from the CPU, it changes back to 0 to indicate that the interrupt has been completed.
  > **Note:** The effect of Remote IRR when set to 1, actually prevents the Active signal on the Level Triggered IRQ line from triggering an interrupt again. Imagine that if the Active signal will generate an interrupt, as long as the signal remains Active (eg high), the interrupt will continue to be triggered. This is obviously incorrect, so the interrupt needs to be blocked by the Remote IRR bit. It can be seen that the CPU should try to return the IRQ line to the Inactive state, and then perform EOI, otherwise the interrupt will be generated again.
- The 13th place is Interrupt Input Pin Polarity, 0 means active high, 1 means active low
- The 12th bit is Delivery Status (read only), 0 means idle, 1 means the CPU has not yet accepted the interrupt (the interrupt has not been stored in the IRR)
  - If the target CPU already has two interrupts pending for a Vector, IOAPIC can provide the third pending interrupt for the Vector.
- The 11th bit is Destination Mode, 0 means Physical, and 1 means Logical
- Bits 8-10 are Delivery Mode, with the following values:
  - 000 (Fixed): Send the corresponding interrupt vector number to the target CPU(s) according to the value of Vector
  - 001 (Lowest Priority): Send the corresponding interrupt vector number to the CPU with the lowest Priority among all target CPU(s) determined by Destination according to the value of Vector. About this mode, please refer to Chapter 10 of Volume 3 of Intel IA32 Manual
  - 010 (SMI): Send an SMI to the target CPU(s). In this mode, Vector must be 0, and SMI must be edge triggered
  - 100 (NMI): Send an NMI to the target CPU(s) (take the #NMI pin). At this time, the Vector will be ignored, and the NMI must be edge triggered
  - 101 (INIT): Send an INIT IPI to the target CPU(s), which causes an INIT to occur on the CPU (refer to Table 9-1 of Volume 3 of the Intel IA32 Manual for the CPU status after INIT). In this mode, Vector must be 0. And must be edge triggered
    > **Info:** The APIC ID and Arb ID of the CPU after INIT (only exists on Pentium and P6) remain unchanged
  - 111 (ExtINT): Sending an interrupt signal compatible with 8259A to the target CPU(s) will cause an INTA cycle. The CPU(s) will request the Vector from the external controller during this cycle. ExtINT must be edge triggered
- Bits 0-7 are Vector, which is the interrupt vector number received by the target CPU. The valid range is 16-254 (0-15 reserved, 255 is global broadcast)

### Destination Mode

Physical Mode indicates that the value of Destination is the APIC ID of the destination LAPIC. Logical Mode means that the value of Destination is Message Destination Address (MDA), which can be used to reference a group of LAPIC (that is, for Multicast). For MDA, please refer to Chapter 10 of Volume 3 of Intel IA32 Manual.

If a group of CPUs are referenced using the Logical Mode addressing mode and the Lowest Priority sending mode is selected at the same time, the interrupt will eventually be sent to the CPU with the lowest priority in this group of CPUs. (For the determination of priority, please refer to Chapter 10 of Book 3 of Intel IA32 Manual. One method is based on the TPR register in LAPIC)

### Pin

According to the manual, the 24 interrupt input pins of IOAPIC are usually connected as follows:

- Pin #1 is connected to the keyboard interrupt (IRQ1)
- Pin #2 is connected to IRQ0
- Pin #3-#11,#14,#15, respectively connected to ISA IRQ[3:7,8#,9:11,14:15]
- Pin #12 is connected to the mouse interrupt (IRQ12/M)
- Pin #16-#19 represents PCI IRQ[0:3]
- Pin #20-#21 represents Motherboard IRQ[0:1]
- Pin #23 represents the SMI interrupt. If the Mask is off, the SMI interrupt will be drawn from the #SMIOUT pin of IOAPIC, otherwise it will be forwarded by IOAPIC according to RTE #23

The above description represents the typical connection of the PIIX3 chipset. If you want to understand how the current chipset is connected to these pins, you should also check the datasheet of the latest chipset.

It is worth noting that if the interrupt signal of a device (such as a keyboard controller) is connected to the PIC through the IRQ line, it will also be connected to the interrupt input pin of IOAPIC. For example, the keyboard controller is connected to the PIC through IRQ1, and also connected to IOAPIC through Pin #1. Therefore, if PIC and IOAPIC are enabled at the same time, it may cause the device to generate an interrupt. The CPU receives **twice** interrupts. Therefore, one of them must be shielded and only the other is used. Usually we will shield PIC (this can be done by writing 0xFF to OCW1 implementation of PIC).

In fact, by convention, people usually correspond the first 16 Pins of the first IOAPIC (if there are more than one) in the system to the 16 IRQs of the PIC. That is to say, the interrupt signal in Pin #x and IRQ x is the same (0 <= x <16). There are two exceptions. One is that the INTR output pin of Master PIC must be connected to Pin #0 of IOAPIC (this is also a conventional requirement, MP Spec does not stipulate that the INTR output of PIC must be connected to IOAPIC), so Pin #0 It does not correspond to IRQ0. The second is that Pin #2 corresponds to IRQ0. This is because IRQ2 is a slave PIC and does not need to correspond to IOAPIC Pin, and Pin #0 is not connected to IRQ0, just connecting Pin #2 and IRQ0.

## IOAPIC to ICH9

With the improvement of chip integration, the IOAPIC chip has been integrated into the South Bridge. We can find out what device its pin is connected to and what functions have been added from the previous South Bridge manual (Datasheet). Here introduces the IOAPIC in the ICH9 South Bridge in the Q35 chipset emulated by QEMU.

### IOxAPIC

Starting from ICH1, IOAPIC has been integrated into the South Bridge, and the integrated ICH is no longer the original 82093AA IOAPIC, but a modified version, which can be called IOxAPIC. The following describes the changes that have been made to the IOAPIC in the past:

#### ICH1

Although ICH1 still maintains the IOAPIC Version as 0x11, some changes have actually been made:

- Canceled the APIC Base Address Relocation Register, making Index Register (IOREGSEL) and Data Register (IOWIN) fixed at 0xFEC00000 and 0xFEC00010
- Added two 32-bit Write Only registers, IRQ Pin Assertion Register and EOI Register, located at 0xFEC00020 and 0xFEC00040
- The 15th bit of the IOAPIC Version Register indicates whether the IRQ Assertion Register is supported (previously it was a reserved bit, the default value is 0)

IRQ Pin Assertion Register: Bits 0-4 represent IRQ Number, and the remaining bits are reserved. Whenever val is written to this register, the corresponding IRQ will be interrupted.

> **Info:** IRQ Assertion Register is a mechanism used by early implementations of MSI. At that time, fill in 0xFEC00020 in the MSI Address Register and fill in the IRQ Number in the MSI Data Register to perform an MSI. Later, MSI changed to use Upstream Memory Write to write directly to the CPU, and the CPU directly processed the MSI. This function was cancelled in IOAPIC.

EOI Register: Bits 0-7 represent interrupt vector, and the remaining bits are reserved. Whenever val is written to this register, the Remote IRR bit of the RTE entry whose interrupt vector is val will be cleared.

#### PCI Age (ICH2 - ICH5)

Starting from ICH2, IOAPIC Version has been changed to 0x20 (the document is written as 0x02, which is actually 0x20, and Version 0x2X indicates that PCI 2.2 is supported), with the following changes:

- Added an indirect referenced register, Boot Configuration Register, located at index 0x3, readable and writable, only the lowest bit is valid, 0 means APIC bus sending interrupt message (default 0), 1 means sending via system bus (FSB) Interrupt message

ICH4 has not changed the IOAPIC Version, but has expanded the RTE entry. Starting from ICH4, the 48-55th bits of the RTE represent the EDID (Extended Destination ID). When sending interrupt messages through the system bus, the EDID is the 4-11th bits of the address .

> **PS:** The original document is "They become bits 11:4 of the address", but the destination that LAPIC can accept is only 8 or 32 bits (x2APIC), and the Destination of IOAPIC is only 4 when in Physical Mode Bit, can be spliced ​​with EDID. The specific role of EDID and its use in OS remains to be investigated.

ICH5 has not changed the IOAPIC Version, but deleted the Arbitration ID Register and Boot Configuration Register, marking the complete removal of APIC Bus compatibility (this is already 2003, and several years have passed since the last CPU of the P6 family).

#### PCIe Age (ICH6 - ICH9)

ICH6 supports PCIe for the first time, with the following changes:

- Removed IRQ Pin Assertion Register

Starting from ICH8, the address of the IOAPIC register becomes variable again, and bits 12-15 of the IOAPIC address are controlled by bits 4-7 (APIC Range Select, ASEL) of the OIC (Other Interrupt Control) register in the Chipset Configuration Register. At this time, the address range of IOAPIC is 0xFEC0x000-0xFEC0x040 (where x is configurable)

### IOxAPIC Interrupt Delivery

Initially, IOAPIC realized interrupt sending by sending a Message to APIC Bus. This is still the case in ICH1, but since ICH2, it has supported sending interrupts through the system bus. The so-called sending interrupts through the system bus here is actually the same as the MSI method. They both write specific data to a specific address. The CPU will listen to this write, so it interprets it as an interrupt request and lets LAPIC handle the request. In fact, the address and data format is the same as MSI.

This technology is called Front-Side Interrupt Delivery in ICH2, System Bus Interrupt Delivery in ICH3 and ICH4, and FSB Interrupt Delivery in ICH5-ICH9, which essentially refers to the same thing.

It can be seen that the modern MSI introduced in ICH6 is actually FSB Interrupt Delivery starting from ICH2, but the former is sent directly from the PCI Device to the CPU, and the latter is forwarded to the CPU via IOAPIC, but it seems to the CPU. Are the same.

It should be noted that FSB Interrupt Delivery only supports ordinary interrupts, and does not support SMI, NMI, INIT interrupts. You cannot fill in SMI, NMI or INIT in the Delivery Mode field (ICH2 does not mention this regulation, ICH3-ICH9 has this regulation) . The OS can query whether the chipset supports the FSB Interrupt Delivery feature through the ACPI table (constructed by the BIOS).

### Configuration

The IRQ pin configuration in ICH9 is as follows:

![ich9_ioapic_1](/assets/ich9_ioapic_1.png)
![ich9_ioapic_2](/assets/ich9_ioapic_2.png)
