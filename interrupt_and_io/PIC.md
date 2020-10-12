# PIC
PIC is also 8259A interrupt controller, which can be used with MCS-80/85 (8080/8085) or 8086. The former is different from the latter in operation mode. Below, only 8086 mode, which is compatible with modern PCs, is considered. Manual [see here](http://heim.ifi.uio.no/~inf3151/doc/8259A.pdf).

Each 8259A chip has 8 interrupt pins, which are marked as IRQ0-IRQ7. The 8259A supports cascading and can support up to one Master and eight Slaves, but the conventional way of using it is one Master plus one Slave. At this time, the interrupt of Master is marked as IRQ0-IRQ7, the interrupt of Slave is marked as IRQ8-IRQ15, and the INT output signal of Slave is connected to IRQ2 of Master, so a total of 15 interrupt signals are supported.
> **Info:** When there was only one PIC chip, IRQ2 was already occupied. When IBM switched to the dual PIC chip solution, the original IRQ2 was reconnected to IRQ9

The two basic registers of 8259A are IRR and ISR, both are 8-bit registers, used to record the status of the interrupt currently being processed.

## Basic process
1. When one of the IRQ0-IRQ7 pins receives an interrupt signal, set the corresponding bit in IRR
2. The 8259A sends an interrupt signal to the CPU through the INT signal line
3. After the CPU receives the INT signal, it sends out an INTA signal and inputs it to the INTA input pin of 8259A
4. Clear the bit in the IRR and set the corresponding bit in the ISR
5. The CPU sends out an INTA signal in the next cycle and inputs the INTA input pin of the 8259A. At this time, the 8259A sends an Interrupt Vector to the CPU through the data bus.
6. Finally, if it is in AEOI mode, the bit in the ISR is cleared directly, otherwise, after the CPU has processed the interrupt, an EOI is performed to 8259A to clear the bit in the ISR

> **Note:** If the interrupt signal has disappeared in the fourth step and its IRQ number cannot be judged, a Spurious Interrupt will be generated, and its Interrupt Vector and IRQ7 will be the same. Therefore, when accepting IRQ7 (or IRQ15, etc.), you need to check the ISR of the PIC to see if it is Spurious Interrupt, and then interrupt processing.
>
> PS. Obviously, this is not as advanced as LAPIC assigns a vector number to Spurious Interrupt alone.

### Cascade Mode
In cascade mode, if the Slave receives an interrupt, it will output an interrupt signal through the INT output pin, causing the Master to receive an interrupt on an IRQ pin. Since the Master has been configured in advance and knows that the pin corresponds to the Slave (and knows which one), after sending the INT signal to the CPU, it will select one of the 8 Slaves through a 3-bit CAS selector. The selected Slave outputs Interrupt Vector to the data bus in the second INTA cycle to complete the interrupt transmission.

It should be noted that in cascade mode, after the CPU finishes executing the interrupt processing routine, it needs to perform two EOIs, one for the Master and one for the Slave.

## Programming interface
PIC is manipulated through Port IO, Master occupies two ports 0x20 and 0x21, Slave occupies two ports 0xA0 and 0xA1. Below we refer to the former port as the Command port, and the latter port as the Data port (this is the name on OSDev). The A_0 bit in [Manual](http://heim.ifi.uio.no/~inf3151/doc/8259A.pdf) represents the last digit of the Port number, 0 is Command, and 1 is Data.

### Initialization
There are four Words (8 bits) that can be used during initialization, namely ICW1-ICW4 (Initialization Command Word). Writing to ICW1 will start the initialization process and reinitialize the PIC.

When the A_0 bit is 0 (that is, written to the Command port) and the fourth bit of the input value is 1, it is considered to be ICW1, which will start the initialization process. Then you need to input ICW2-ICW4, they all require A_0 bit 1, that is, input from the Data port.

The content of ICW1 is as follows:
-The 0th bit is IC4, which indicates whether ICW4 is required, and 1 is required
-The first digit is SNGL, 1 means Single, 0 means Cascade, if it is Single mode, ICW3 is not required
-:warning: The second digit is ADI, only useful in 8080/8085 mode, it will be ignored in 8086 mode
-The third digit is LTIM, and 1 means Level Triggered Mode, at this time Edge Triggered Interrupt will be ignored
-The 4th place must be 1
-:warning: Bits 5-7, 8080/8085 mode is only useful, it will be ignored in 8086 mode

ICW1 must be followed by ICW2, and the content is as follows:
-:warning: Bits 0-2, 8080/8085 mode is only useful, it will be ignored in 8086 mode
-Bits 3-7 indicate bits 3-7 of the Interrupt Vector. In other words, IRQ0-IRQ7 of a PIC will be mapped to Offset+0 to Offset+7, where Offset is the value of ICW2 (the last three digits are always regarded as zeros, just fill in zeros when setting)

> **Info:** In real mode, the conventional setting is that the IRQ of Master is mapped to 0x08-0x0F, and the IRQ of Slave is mapped to 0x70-0x77, which is usually set in the BIOS. But after entering the protected mode, 0x08-0x0F conflicts with the default Exception range of the CPU, so the IRQ should be remapped, generally to 0x20-0x2F.

If you are in cascade mode, you need to set ICW3 as follows:
-In Slave mode, bits 0-2 indicate the Slave ID (corresponding to the value of the CAS selector), and the remaining bits are reserved and should be 0
-In Master mode, ICW3 is an 8-bit bitmap, and each bit corresponds to a Slave

> **Info:** In non-Buffered mode, whether the PIC is in Master or Slave state is determined by the level of the SP/EN input pin. If it is 1, it is Master, and if it is 0, it is Slave. In the Buffered mode, it is determined by the M/S bit of ICW4, with 1 for Master and 0 for Slave.

If IC4 is 1, you also need to set ICW4, otherwise ICW4 is treated as all 0s, and the content is as follows:
-The 0th bit is μPM, 0 means 8080/8085 operation mode, 1 means 8086 operation mode
    > It can be seen that if you want to use PIC on modern machines, you must set ICW4
-The first digit is AEOI, take 1 to turn on Automatic EOI mode
-The second digit is M/S, which is used to determine Master and Slave in Buffered mode
-The third bit is BUF, and 1 means to turn on Buffered mode
    -In Buffered mode, the SP/EN pin is used as an output pin to control the opening of the buffer. The so-called Buffer is a Buffer set between PIC and Data Bus.
-The fourth digit is SFNM, and 1 means to turn on Special Fully Nested mode
    -This mode is used for cascading configuration. The Special Fully Nested mode should be turned on by the Master. At this time, the interrupt of the Slave will not be shielded by the In-Service itself. In this mode, the BIOS or OS needs to check whether there is more than one interrupt waiting for EOI in the Slave when the interrupt processing of the Slave is completed. If so, only need to perform EOI on the Slave and not need to perform EOI on the Master, otherwise, the Master should also perform EOI
-Bits 5-7 are reserved and should be all 0s

### Operation
After the initialization is completed, three 8-bit OCW (Operation Command Word) can be used for runtime adjustment.

When the A_0 bit is 1 (that is, the Data port), the input value is OCW1, which represents the Interrupt Mask, and each bit corresponds to an IRQ. Taking 1 to mask the IRQ. At the same time, reading the port can read the value of IMR (Interrupt Mask Register).

When the A_0 bit is 0 (ie Command port), its meaning is determined according to the third and fourth bits of the input value. If the 4th bit is 1, it means ICW1, if the 4th bit is 0, then the 3rd bit is 0 for OCW2, and the 3rd bit is 1 for OCW3.

#### OCW2
Before introducing OCW2, first introduce the following concepts:
##### End of Interrupt
-EOI Command: There are two types of EOI Commands, namely Specific and Non-Specific. Non-Specific EOI will automatically clear the highest priority bit in the ISR [**The highest bit in the manual, suspected to be a clerical error, it should be the highest priority bit **], Specific EOI clears the specified Bit in the ISR
-AEOI Mode: If the AEOI bit in ICW4 is set to 1, it will enter the Automatic EOI mode, and automatically perform a Non-Specific EOI immediately after the interrupt is received (after the INTA cycle ends)

##### Interrupt Priority
-Fully Nested Mode: The default state after the PIC is initialized. At this time, the priority of the interrupt is reduced in the order of IRQ0-IRQ7. High priority interrupts can interrupt low priority interrupts.
    > **Info:** Once there is a high-priority interrupt in the ISR, the low-priority interrupt will not enter the IRR, and in LAPIC, a low-priority interrupt can be stored in the IRR
-Rotation Mode: Priority rotation, one rotation will make a certain IRQ priority become the lowest, and the rest of the IRQ will rotate with it (eg if IRQ4 is the lowest priority 7, then IRQ3 priority will become 6, and so on, finally IRQ5 will change Is the highest priority 0)
    -Automatic Rotation: Attached to the option on the EOI, the EOI has the ability to make the IRQ priority of the cleared Bit the lowest, and let the rest of the IRQs rotate in turn
    -Specific Rotation: Manually perform priority rotation, specify an IRQ priority to become the lowest, and let the rest of the IRQ rotate in turn (this operation has nothing to do with EOI, but can be attached to Specific EOI)

##### Operations of OCW2
The content of OCW2 is as follows:
-Bits 0-2, used in conjunction with the SL bit to specify a certain IRQ
-Bits 3-4 must be 0
-The 5th bit is EOI, taking 1 means this is an EOI Command, which can clear a bit in the ISR
-The sixth digit is SL, which is Specific Level, taking 1 means specifying an IRQ for operation (Specific EOI or Specific Rotation)
-The seventh bit is R, which is Rotation, which is used to decide whether to perform priority rotation

The combination of positions 5-7 of OCW2 is as follows:
-EOI=1, SL=0, R=0: Non-Specific EOI Command, clear the highest priority bit of ISR, the priority remains unchanged
-EOI=1, SL=1, R=0: Specific EOI Command, bits 0-2 of OCW2 are used to specify which bit of ISR to clear
-EOI=1, SL=0, R=1: Non-Specific EOI with Automatic Rotation, clear the highest priority bit of ISR, priority rotation
-EOI=0, SL=0, R=1: Turn on Automatic Rotation in AEOI mode, so that Non-specific EOI automatically performed under AEOI has Automatic Rotation function
-EOI=0, SL=0, R=0: Turn off Automatic Rotation in AEOI mode
-EOI=0, SL=1, R=1: Specific Rotation, bits 0-2 of OCW2 are used to specify which IRQ is the lowest priority
-EOI=1, SL=1, R=1: Specific EOI with Specific Rotation, bits 0-2 of OCW2 are used to specify which Bit of ISR to clear and which IRQ is the lowest priority
-EOI=1, SL=1, R=0: invalid operation, nothing will happen

#### OCW3
The contents of OCW3 are as follows:
-The 0th bit is RIS, that is, Read ISR, 1 means read IRR from Command port, and 0 means read ISR from Command port
-The first bit is RR, Read Register, when set to 1, the RIS bit is enabled, when set to 0, the RIS bit will be ignored
-The second digit is P, 1 means this is a Poll Command, 0 means no effect
-The third digit must be 1, and the fourth digit must be 0
-The fifth digit is SMM, Special Mask Mode, take 1 to turn on Special Mask mode, take 0 to turn off
-The sixth bit is ESMM, Enable Special Mask Mode. When set to 1, the SMM bit is enabled, and when set to 0, the SMM bit will be ignored
-The 7th bit is reserved and should be 0

In fact, OCW3 integrates three functions (can be used at the same time). The first function is to read the value of IRR or ISR from the Command port (bit A_0 is 0). You can choose which one to read through the RR and RIS bits. PIC After initialization, the default is to read the value of IRR.

The second function is Poll mode. After sending a Poll Command through OCW3, the next time the Command port is read (A_0 bit is 0), it is equivalent to an interrupt Accept. If there is an interrupt Pending at this time, the Bit in the ISR will be set, the highest bit of the value read is 1, and the lowest 3 bits are the IRQ number. Otherwise, the highest bit of the value read is 0, indicating that no interrupt has occurred.

The third function is Special Mask mode. In this mode, if a bit is shielded by IMR, it will have the following effects:
-Even if this bit has the highest priority in ISR, it will not be cleared by Non-Specific EOI
-For the remaining unmasked bits, even if the priority is lower than this bit, the corresponding IRQ can still be accepted

## PIC in ICH9
With the improvement of chip integration, the 8259 chip is integrated into the South Bridge. We can find out what equipment its IRQ pin is connected to and what minor changes have been made in the manuals of the South Bridge. Here are the changes and configurations of the 8259 in the ICH9 South Bridge in the Q35 chipset simulated by QEMU.

### Changes
In the ISA bus era, the third bit of ICW1 determines whether it is Level Triggered. Taking 1 means that the entire PIC interrupt is Level Triggered. After entering the PCI bus era (starting from the PIIX chipset), two 8-bit ELCR (Edge/Level Control Register) registers (ELCR0, ELCR1) are added, located at ports 0x4D0 and 0x4D1, each bit controls whether an IRQ is Level Triggered.

### Configuration
The ID of the Slave is set to 010b, and the IRQ Pins of the Master and Slave are set as follows:

![ich9_8259](/assets/ich9_8259.png)

Among them, IRQ0, IRQ1, IRQ2, IRQ8, IRQ13 must be Edge Triggered, that is, the corresponding bit in ELCR must be 0
