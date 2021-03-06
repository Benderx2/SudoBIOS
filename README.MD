SudoBIOS - a completely transparent 32 bit protected mode BIOS extender - say goodbye to real mode code. SudoBIOS uses simple mode switching, address translation and buffering to provide interrupt based classic (Compatibility Support Module) BIOS services while in 32 bit protected mode.

A typical application is to have the boot sector or mbr load SudoBIOS as a 2nd stage (to absolute address 0x2000). A far call to that location returns in 32 bit protected mode. A subset of the native BIOS functions (the common usable ones) are supported.

SudoBIOS is written in FASM (2200+ lines of Intel syntax x86 assembly language) and will assemble as is in NASM. Both output files are essentially the same (with NASM -O2). The only insignificant difference is the order of the operand (0x66) and address (0x67) override opcode pairing of the 2 assemblers.

There is a SudoBIOS demo. https://github.com/mikegonta/SudoBIOS/releases. It's a 1.44MB FAT12 USB flash drive image of the real mode MikeOS converted to 32 bit, running in 32 bit protected mode and accessing BIOS services transparently through SudoBIOS. It will boot and run as either a floppy disk or hard disk format on real hardware and QEMU. The image is a non standard floppy disk so BOCHS has trouble running it as a floppy image, but will work fine as a disk image.
