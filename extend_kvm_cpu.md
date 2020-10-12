## Let KVM break the limit and support 512 vCPUs

Need to modify qemu and kvm.

1. qemu-system-x86_64: unsupported number of maxcpus

  include/sysemu/sysemu.h

  ```c
  #define MAX_CPUMASK_BITS 288
  ```

  =>

  ```c
  #define MAX_CPUMASK_BITS 512
  ```

2. qemu-system-x86_64: Number of SMP CPUs requested (500) exceeds max CPUs supported by machine 'pc-i440fx-2.8' (255)

  ```
  m->max_cpus = 288;
  ```

  =>

  ```
  m->max_cpus = 512;
  ```

3. Warning: Number of SMP cpus requested (500) exceeds the recommended cpus supported by KVM (240)
Number of SMP cpus requested (500) exceeds the maximum cpus supported by KVM (288)

  Change kvm-all.c

  ```c
  #define KVM_CAP_MAX_VCPUS 66       /* returns max vcpus per vm */
  static int kvm_max_vcpus(KVMState *s)
  {
    int ret = kvm_check_extension(s, KVM_CAP_MAX_VCPUS);
    return (right)? ret: kvm_recommended_vcpus (s);
  }

  int kvm_check_extension(KVMState *s, unsigned int extension)
  {
    int ret;

    ret = kvm_ioctl(s, KVM_CHECK_EXTENSION, extension);
    if (ret < 0) {
      ret = 0;
    }

    return ret;
  }

  static int kvm_init(MachineState *ms)
  {
    ...
    soft_vcpus_limit = kvm_recommended_vcpus(s);
    hard_vcpus_limit = kvm_max_vcpus(s);

    while (nc->name) {
      if (nc->num > soft_vcpus_limit) {
        fprintf(stderr,
           "Warning: Number of %s cpus requested (%d) exceeds "
           "the recommended cpus supported by KVM (%d)\n",
           nc->name, nc->num, soft_vcpus_limit);

        if (nc->num > hard_vcpus_limit) {
          fprintf(stderr, "Number of %s cpus requested (%d) exceeds "
            "the maximum cpus supported by KVM (%d)\n",
            nc->name, nc->num, hard_vcpus_limit);
          exit(1);
        }
      }
      nc++;
    }
  }
  ```

  The number of vcpus returned by the KVM interface is limited. Essentially, kvm must be changed.

  arch/x86/include/asm/kvm_host.h

  ```c
  #define KVM_MAX_VCPUS 288
  #define KVM_SOFT_MAX_VCPUS 240
  ```

  =>

  ```c
  #define KVM_MAX_VCPUS 512
  #define KVM_SOFT_MAX_VCPUS 512
  ```

4. qemu-system-x86_64: current -smp configuration requires Extended Interrupt Mode enabled. You can add an IOMMU using: -device intel-iommu,intremap=on,eim=on

  Add the parameter -device intel-iommu, intremap=on, eim=on when starting

5. qemu-system-x86_64: -device intel-iommu,intremap=on,eim=on: Intel Interrupt Remapping cannot work with kernel-irqchip=on, please use 'split|off'.

  ```bash
  $ sudo $QEMU_SYSTEM_64 -m 512 -hda $DISK -boot c -vnc :1 -enable-kvm -smp 500 -machine q35,kernel-irqchip=split -device intel-iommu,intremap=on,eim=on
  ```

  + Replace `$QEMU_SYSTEM_64` with the path of qemu-system-x86_64 after recompilation, for example `/home/binss/Desktop/qemu/bin/debug/native/x86_64-softmmu/qemu-system-x86_64`
    + Replace `$DISK` with the target img file
    + 可参考：<https://lists.gnu.org/archive/html/qemu-devel/2016-07/msg02930.html>

6. Break the CPU limit of guest OS

  Modify nr_cpu, currently

  ```bash
  $ grep NR_CPUS /boot/config-`uname -r`
  $ CONFIG_NR_CPUS=256
  ```

  Here you can manually modify the config and recompile the kernel, see:
    - [KernelBuild](https://kernelnewbies.org/KernelBuild)
    - [Kernels/Traditional compilation](https://wiki.archlinux.org/index.php/Kernels/Traditional_compilation)

  Or use a newer kernel, such as 4.4.0-62.

