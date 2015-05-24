SudoBIOS is a bootable 32bit protected mode BIOS extender. SudoBIOS uses simple mode switching and buffering to provide interrupt based classic (Compatibility Support Module) BIOS services while in 32bit protected mode. SudoBIOS is a 2nd stage loaded by the boot sector or mbr to absolute address 2000h. A far call to that location returns in 32bit protected mode. A subset of the native BIOS functions (the common usable ones) are supported.

SudoBIOS is written in FASM (2400+ lines of intel syntax x86 assembly language) and will assemble as is in NASM. Both output files are essentially the same (with NASM -O2). The only insignificant difference is the order of the operand (66h) and address (67h) override opcode pairing of the 2 assemblers.

There is a SudoBIOS demo. https://www.dropbox.com/s/3qveuotu4t1gb92/MikeOS32_4.4.img It's a 1.44MB FAT12 USB flash drive image of the real mode MikeOS converted to 32bit, running in 32bit protected mode and accessing BIOS services transparently through SudoBIOS. It will boot and run as either a floppy disk or hard disk format on real hardware and QEMU. The image is a non standard floppy disk so BOCHS has trouble running it as a floppy image, but will work fine as a disk image.

Here is the source code to MikeOS32. https://www.dropbox.com/s/ag9h4gdhr1vpc60/MikeOS32_4.4.zip 
