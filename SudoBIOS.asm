; SudoBIOS memory map

; 0x00000000 - 0x00000399 Real Mode Interrupt Vector Table
; 0x00000400 - 0x00000500 BIOS Data Area
; 0x00000501 - 0x00000FFF FREE
; 0x00001000 - 0x00001FFF Protected Mode Interrupt Descriptor Table
; 0x00002000 - 0x00007BFF RESERVED SudoBIOS
;   0x00002000 real mode entry (initialize and switch to PM32)
;   0x00002000 protected mode entry (de-initialize and switch back to RM16)
;   0x0000200C A20 State (BYTE) Variable
;   0x00002030 Global Descriptor Table Pointer
;   0x00002038 Protected Mode Descriptor Table Pointer
;   0x00002040 Real Mode Descriptor Table Pointer
;   0x00005B00 4K SudoBIOS real mode page 0 stack
; 0x00006C00 - 0x00007BFF RESERVED shared 4K low memory page 0 stack
; 0x00007C00 - 0x00008BFF RESERVED 4K PM32 boot sector
; 0x00008C00 - 0x0007FFFF FREE
; 0x00080000 - 0x0008FFFF SudoBIOS drive buffer
; 0x00090000 - 0x0009???? FREE
; 0x0009???? - 0x0009FFFF Extended BIOS Data Area
; 0x000A0000 - 0x000FFFFF ROM Firmware
; 0x00100000 - 0xFFFFFFFF FREE

%define fix define

%define KBC_STATUS 0x64
%define KBC_COMMAND 0x64
%define KBC_DATA 0x60
%define KBC_INPUT_FULL 2
%define KBC_WRITE_OUTPUT 0xD1
%define ENABLE_A20 0xDF
%define CARRY_FLAG 0
%define ZERO_FLAG 6

%define RM_STACK 0x6C00
%define PM_STACK 0x7C00

%define DRIVE_BUFFER 0x80000
%define PM_IDT 0x1000

%define RM_CODE_SEGMENT 0
%define RM_DATA_SEGMENT 0

%define PM_CODE_SELECTOR 8
%define PM_DATA_SELECTOR 0x10
%define RM_CODE_SELECTOR 0x18
%define RM_DATA_SELECTOR 0x20

%define UNSUPPORTED_FUNCTION 0x86
%define ERROR 1

%define _ds esp+4
%define _es esp+6
%define _edi esp+8
%define _di esp+8
%define _esi esp+12
%define _si esp+12
%define _ebp esp+16
%define _bp esp+16

%define _ebx esp+24
%define _bx esp+24
%define _bl esp+24
%define _bh esp+25
%define _edx esp+28
%define _dx esp+28
%define _dl esp+28
%define _dh esp+29
%define _ecx esp+32
%define _cx esp+32
%define _cl esp+32
%define _ch esp+33
%define _eax esp+36
%define _ax esp+36
%define _al esp+36
%define _ah esp+37
%define eflags esp+48

org 0x2000
use16
  pushf
  push cx
  mov cx, 0x121
  jmp SHORT to_pm
use32
  jmp to_rm

align 4
A20_state: dd 0

align 16
  db "(c) Mike Gonta  "
  db "2015-05-22-0953", 0

align 8
gdt_ptr:
  dw 39 ; Global Descriptor Table size - 1
  dd gdt
  dw 0
pm_idt_ptr:
  dw 256*8-1
  dd PM_IDT
  dw 0
rm_idt_ptr:
  dw 256*4-1
  dd 0
  dw 0

use16
align 16
to_pm:
  shl ch, cl
  je .1 ; 8086 - 80186+ limits shift count to 5 bits
  mov cx, [0x1A]
  push cx
  mov cx, [0x18] ; int 06 - invalid opcode
  push cx
  mov cx, invalid_opcode
  mov [0x18], cx
  xor cx, cx
  mov [0x1A], cx
  movzx esp, sp ; must be at least 80386 {4 bytes}
  jcxz .2
.1:
  mov si, must_be_386 ; fail
  call print
  mov si, press_any_key
  call print
  mov ah, 0x10
  int 0x16
  int 0x18
  int 0x19
.2:
  xadd cl, cl ; exchange and add - at least 80486 {3 bytes}
  nop
  mov [A20_state], cl
  pop DWORD [0x18]
  pop cx
  popf

  pushfd
  pushad
  mov eax, [esp+9*4] ; fixup return address
  mov edx, eax
  and edx, 0xFFFF0000
  shr edx, 12
  and eax, 0xFFFF
  add eax, edx
  mov [esp+9*4], eax
  push ss
  pop bp
  and ebp, 0xFFFF
  shl ebp, 4 ; ss

  cli
  mov dl, 2 ; enable A20
  cmp BYTE [A20_state], 4 ; 80386
  jne .3
  call A20_kbc ; assume A20 not set, use KB controller method
  jmp .4 ; assume success
.3:
  push -1
  pop es
  mov bl, 1
  call check_A20
  jne .4
  call A20_BIOS
  mov bl, 2
  call check_A20
  jne .4
  call A20_fast
  mov bl, 3
  call wait_A20
  jne .4
  call A20_kbc
  mov bl, 4
  call wait_A20
  jne .4
  mov BYTE [A20_state], 0
  mov si, A20_error
  call print
  mov ah, 0x10
  int 0x16
.4:

; RM int 0x10 needed for Lenovo B590
; (might be UEFI spash screen, otherwise screen write doesn't work properly)

  mov ah, 8 
  xor bh, bh
  int 0x10

  lgdt [gdt_ptr] ; load Global Descriptor Table
  mov eax, cr0
  or eax, [_CR0_] ; set PE bit - at startup _CR0_ = 1
  mov cr0, eax
  jmp PM_CODE_SELECTOR:next

use16
align 16
invalid_opcode:
  push bp
  mov bp, sp
  mov cx, 4
  add WORD [bp+2], cx ; skip over invalid opcode
  pop bp
  iret

align 16
fatal:
  mov si, must_be_386
  call print
  mov si, press_any_key
  call print
  mov ah, 0x10
  int 0x16
  int 0x18

align 16
print:
  mov ah, 0xE
  xor bh, bh
  cld
.1:
  lodsb
  test al, al
  je .2
  int 0x10
  jmp .1
.2:
  ret

; dl = 2 -> set, dl = 0 -> clear
align 16
A20_BIOS:
  push dx
  mov al, dl
  shr al, 1
  mov ah, 0x24
  int 0x15
  pop dx
  ret

align 16
A20_fast:
  in al, 0x92
  and al, 0xFE
  or al, dl
  out 0x92, al
  ret

align 16
A20_kbc:
  call kbc_flush_input
  mov al, KBC_WRITE_OUTPUT
  out KBC_COMMAND, al
  call kbc_flush_input
  mov al, 0xDD
  or al, dl
  out KBC_DATA, al
  ret

align 16
kbc_flush_input:
  xor cx, cx
.1:
  in al, KBC_STATUS
  test al, KBC_INPUT_FULL
  je .2
  sub cx, 1
  jne .1
.2:
  ret

align 16
check_A20:
  mov [A20_state], bl
  cmp [es:16+A20_state], bl
  jne .1
  push WORD [A20_state]
  add bl, 1
  mov [A20_state], bl
  cmp [es:16+A20_state], bl
  pop WORD [A20_state]
.1:
  ret

align 16
wait_A20:
  xor cx, cx
.1:
  call check_A20
  jne .2
  out 0xED, al ; 1 microsecond delay
  sub cx, 1
  jne .1
.2:
  ret

use32
align 16
next:
  mov eax, PM_DATA_SELECTOR
  mov ds, ax
  mov es, ax
  mov fs, ax
  mov gs, ax
  mov ss, ax

  add esp, ebp

  mov ah, 0x68
  call remap_pic
  mov edi, rm_ivt  ; hook some RM ints and save the original
  xor esi, esi ; original real mode IVT
  mov ecx, 120
  cld
  rep movsd

  xor esi, esi
  mov DWORD [esi+0x00*4], rm_int_0x00
  mov DWORD [esi+0x01*4], rm_int_0x01
  mov DWORD [esi+0x02*4], rm_int_0x02
  mov DWORD [esi+0x03*4], rm_int_0x03
  mov DWORD [esi+0x04*4], rm_int_0x04
  mov DWORD [esi+0x05*4], rm_int_0x05
  mov DWORD [esi+0x06*4], rm_int_0x06
  mov DWORD [esi+0x07*4], rm_int_0x07
  mov DWORD [esi+0x15*4], rm_int_0x15
  mov DWORD [esi+0x19*4], rm_int_0x19
  mov DWORD [esi+0x1B*4], rm_int_0x1B
  mov DWORD [esi+0x1C*4], rm_int_0x1C
  mov DWORD [esi+0x4A*4], rm_int_0x4A
  mov DWORD [esi+0x68*4], rm_irq_0x00
  mov DWORD [esi+0x69*4], rm_irq_0x01
  mov DWORD [esi+0x6A*4], rm_irq_0x02
  mov DWORD [esi+0x6B*4], rm_irq_0x03
  mov DWORD [esi+0x6C*4], rm_irq_0x04
  mov DWORD [esi+0x6D*4], rm_irq_0x05
  mov DWORD [esi+0x6E*4], rm_irq_0x06
  mov DWORD [esi+0x6F*4], rm_irq_0x07
  mov DWORD [esi+0x70*4], rm_irq_0x08
  mov DWORD [esi+0x71*4], rm_irq_0x09
  mov DWORD [esi+0x72*4], rm_irq_0x10
  mov DWORD [esi+0x73*4], rm_irq_0x11
  mov DWORD [esi+0x74*4], rm_irq_0x12
  mov DWORD [esi+0x75*4], rm_irq_0x13
  mov DWORD [esi+0x76*4], rm_irq_0x14
  mov DWORD [esi+0x77*4], rm_irq_0x15

  mov eax, _iret+0x80000
  mov ebx, 0x8E00
  mov ecx, 256
  mov edi, PM_IDT
align 16
.3:
  mov [edi], eax
  mov [edi+4], ebx
  lea edi, [edi+8]
  sub ecx, 1
  jne .3

  mov edi, PM_IDT
  mov WORD [edi+0x08*8], error_iret
  mov WORD [edi+0x0A*8], error_iret
  mov WORD [edi+0x0B*8], error_iret
  mov WORD [edi+0x0C*8], error_iret
  mov WORD [edi+0x0D*8], error_iret
  mov WORD [edi+0x0E*8], error_iret
  mov WORD [edi+0x10*8], Video_0x10
  mov WORD [edi+0x13*8], Drive_0x13
  mov WORD [edi+0x14*8], Serial_0x14
  mov WORD [edi+0x15*8], System_0x15
  mov WORD [edi+0x16*8], Keyboard_0x16
  mov WORD [edi+0x17*8], Parallel_0x17
  mov WORD [edi+0x19*8], pm_int_0x19
  mov WORD [edi+0x1A*8], Time_0x1A
  mov WORD [edi+0x68*8], pm_irq_0x00
  mov WORD [edi+0x69*8], pm_irq_0x01
  mov WORD [edi+0x6A*8], pm_irq_0x02
  mov WORD [edi+0x6B*8], pm_irq_0x03
  mov WORD [edi+0x6C*8], pm_irq_0x04
  mov WORD [edi+0x6D*8], pm_irq_0x05
  mov WORD [edi+0x6E*8], pm_irq_0x06
  mov WORD [edi+0x6F*8], pm_irq_0x07
  mov WORD [edi+0x70*8], pm_irq_0x08
  mov WORD [edi+0x71*8], pm_irq_0x09
  mov WORD [edi+0x72*8], pm_irq_0x10
  mov WORD [edi+0x73*8], pm_irq_0x11
  mov WORD [edi+0x74*8], pm_irq_0x12
  mov WORD [edi+0x75*8], pm_irq_0x13
  mov WORD [edi+0x76*8], pm_irq_0x14
  mov WORD [edi+0x77*8], pm_irq_0x15

  lidt [pm_idt_ptr]
  popa
  popf
  ret

align 16
remap_pic:
  mov al, 0x11 ; edge triggered, interval=8
  out 0x20, al ; cascaded, ICW4 needed, unmask all channels in master
  mov al, ah ; new vector base (68 or 08)
  out 0x21, al
  mov al, 4 ; input 2 has slave
  out 0x21, al
  mov al, 1 ; 8086 mode
  out 0x21, al
  xor al, al
  out 0xA1, al ; unmask all channels in slave
  ret

use32
align 16
to_rm:
  pop ecx
  pusha
  mov eax, [esp+9*4] ; fixup cs:ip
  mov edx, eax
  and edx, 0xF0000
  shr edx, 4 ; cs and will be ds and es
  shl eax, 16
  or eax, edx
  ror eax, 16
  mov [esp+9*4], eax
  mov ebp, esp
  and ebp, 0xF0000
  shr ebp, 4 ; ss

  cli
  mov ah, 8
  call remap_pic
  xor edi, edi ; IVT
  mov esi, rm_ivt
  mov ecx, 120
  cld
  rep movsd

  lidt [rm_idt_ptr]
  mov eax, cr0
  mov DWORD [_CR0_], 1
  btr eax, 31 ; clear PG bit
  jnc .2
  push ebx ; if paging enabled
  mov cr0, eax ; disable paging
  mov ebx, cr3
  mov cr3, ebx ; flush TLB by writing CR3 back to itself
  pop ebx
.2:
  and al, 0xFE ; clear PE bit
  jmp RM_CODE_SELECTOR:.3 ; far jump to next instruction to set
.3:                       ; real mode selectors
  mov cr0, eax ; to real mode
  mov eax, RM_DATA_SELECTOR
  mov ds, ax
  mov es, ax
  mov fs, ax
  mov gs, ax
  mov ss, ax
  jmp 0:.4 ; far jump to next instruction to
.4:        ; flush queue and set real mode cs
use16
  mov ds, dx
  mov es, dx
  mov fs, dx
  mov gs, dx
  mov ss, bp

  xor dl, dl ; disable A20
  cmp BYTE [cs:A20_state], 1
  jbe .7
  cmp BYTE [cs:A20_state], 2
  jne .5
  call A20_BIOS
  jmp .7
.5:
  cmp BYTE [cs:A20_state], 3
  jne .6
  call A20_fast
  jmp .7
.6:
  call A20_kbc
.7:
  mov [cs:A20_state], dl
  popad
  popfd
  retf

align 16
use32
rm_interrupt: ; <ret>, interrupt, ds/es
  push esp
  cmp DWORD [esp], 0x10000
  jbe .1
  pop DWORD [pm_esp]
  push DWORD [esp+4]
  pop DWORD [pm_int]
  push DWORD [esp+8]
  pop DWORD [pm_segments]
  mov esp, PM_STACK
  push DWORD [pm_segments]
  push DWORD [pm_int]
  lea esp, [esp-4] ; dummy return address
  push DWORD [pm_esp]
.1:
  push eax
  cli
  sgdt [gdt_ptr]
  lidt [rm_idt_ptr]
  mov eax, cr0
  mov [_CR0_], eax
  and DWORD [_CR0_], 0x80000001 ; save PG and PE bits
  jns .2
  push ebx ; if paging enabled
  and eax, 0x7FFFFFFF ; clear PG bit
  mov cr0, eax ; disable paging
  mov ebx, cr3
  mov cr3, ebx ; flush TLB by writing CR3 back to itself
  pop ebx
.2:
  and al, 0xFE ; clear PE bit
  jmp RM_CODE_SELECTOR:.3 ; far jump to next instruction to set
.3:                       ; real mode selectors
  mov cr0, eax ; to real mode
  jmp 0:.4 ; far jump to next instruction to
.4:        ; flush queue and set real mode cs
use16
  xor ax, ax
  mov ss, ax
  mov ax, [esp+16] ; ds segment
  mov ds, ax
  mov ax, [esp+18] ; es segment
  mov es, ax
  movzx eax, BYTE [esp+12]
  mov eax, [cs:rm_ivt+eax*4]
  mov [cs:0xFF*4], eax
  pop eax
  int 0xFF
  pushfd ; save eflags
  push es
  push ds ; save segments
  push eax
  cli
  lgdt [cs:gdt_ptr]
  mov eax, cr0
  or eax, [cs:_CR0_] ; set PG bit if previously enabled, set PE bit
  mov cr0, eax ; return to protected mode
  jmp PM_CODE_SELECTOR:.5 ; far jump to flush queue and set pm cs
.5:
use32
  mov eax, PM_DATA_SELECTOR
  mov ds, ax
  mov es, ax
  mov ss, ax
  pop eax
  pop DWORD [pm_segments]
  pop DWORD [pm_eflags]
  pop esp
  push DWORD [pm_segments]
  pop DWORD [esp+8]
  lidt [pm_idt_ptr]
  push DWORD [pm_eflags]
  popf
  sti
  ret

pm_eflags: dd 0
pm_int: dd 0
pm_segments: dd 0
pm_esp: dd 0

use16
align 16
pm_interrupt: ; <ret>, interrupt - interrupts are disabled
  pop DWORD [cs:rm_return]
  push DWORD [cs:rm_return]
  push ds
  push es
  push ss
  push sp
  pop DWORD [cs:rm_sp]
  cmp WORD [cs:rm_ss], 0
  je .1
  lss sp, [cs:rm_stack_ptr]
  push WORD [cs:rm_int]
  sub sp, 6
.1:
  push DWORD [cs:rm_sp]
  push eax
  lgdt [cs:gdt_ptr] ; load Global Descriptor Table
  mov eax, cr0
  or eax, [cs:_CR0_] ; set PG bit if previously enabled, set PE bit
  mov cr0, eax ; return to protected mode
  jmp PM_CODE_SELECTOR:.2 ; far jump to next instruction to
.2:                       ; flush queue and set pm cs
use32
  mov eax, PM_DATA_SELECTOR
  mov ds, ax
  mov es, ax
  mov fs, ax
  mov gs, ax
  mov ss, ax
  lidt [pm_idt_ptr]
  push ebx
  movzx eax, BYTE [esp+18]
  mov ebx, [PM_IDT+eax*8]
  mov [PM_IDT+0xFF*8], bx
  mov ax, [PM_IDT+eax*8+6]
  mov [PM_IDT+0xFF*8+6], ax
  pop ebx
  pop eax
  int 0xFF
  pushf
  cli
  push eax
  lidt [rm_idt_ptr]
  mov eax, cr0
  mov [_CR0_], eax
  and DWORD [_CR0_], 0x80000001 ; save PG and PE bits
  jns .3
  push ebx ; if paging enabled
  and eax, 0x7FFFFFFF ; clear PG bit
  mov cr0, eax ; disable paging
  mov ebx, cr3
  mov cr3, ebx ; flush TLB by writing CR3 back to itself
  pop ebx
.3:
  and al, 0xFE ; clear PE bit
  jmp RM_CODE_SELECTOR:.4 ; far jump to next instruction to set
.4:                       ; real mode selectors
  mov cr0, eax ; to real mode
  jmp 0:.5 ; far jump to flush queue and set real mode cs
.5:
use16
  xor ax, ax
  mov ss, ax
  pop eax
  popfd
  lss sp, [esp]
  pop es ; ss
  pop es
  pop ds
  ret 2

align 4
rm_stack_ptr: dd RM_STACK
rm_return dw 0
rm_int dw 0
rm_sp dw 0
rm_ss dw 0

use16
align 16
rm_int_0x00:
  push 0
  call pm_interrupt
  iret

align 16
rm_int_0x01:
  push 1
  call pm_interrupt
  iret

align 16
rm_int_0x02:
  push 2
  call pm_interrupt
  iret

align 16
rm_int_0x03:
  push 3
  call pm_interrupt
  iret

align 16
rm_int_0x04:
  push 4
  call pm_interrupt
  iret

align 16
rm_int_0x05:
  push 5
  call pm_interrupt
  iret

align 16
rm_int_0x06:
  push 6
  call pm_interrupt
  iret

align 16
rm_int_0x07:
  push 7
  call pm_interrupt
  iret

align 16
rm_int_0x19:
  push 0x19
  call pm_interrupt
  iret

align 16
rm_int_0x1B: ; break key pressed callback
  push 0x1B
  call pm_interrupt
  iret

align 16
rm_int_0x1C: ; timer callback
  push 0x1C
  call pm_interrupt
  iret

align 16
rm_int_0x4A: ; alarm timer callback
  push 0x4A
  call pm_interrupt
  iret

align 16
rm_irq_0x00:
  push 0x68
  call pm_interrupt
  iret

align 16
rm_irq_0x01:
  push 0x69
  call pm_interrupt
  iret

align 16
rm_irq_0x02:
  push 0x6A
  call pm_interrupt
  iret

align 16
rm_irq_0x03:
  push 0x6B
  call pm_interrupt
  iret

align 16
rm_irq_0x04:
  push 0x6C
  call pm_interrupt
  iret

align 16
rm_irq_0x05:
  push 0x6D
  call pm_interrupt
  iret

align 16
rm_irq_0x06:
  push 0x6E
  call pm_interrupt
  iret

align 16
rm_irq_0x07:
  push 0x6F
  call pm_interrupt
  iret

align 16
rm_irq_0x08:
  push 0x70
  call pm_interrupt
  iret

align 16
rm_irq_0x09:
  push 0x71
  call pm_interrupt
  iret

align 16
rm_irq_0x10:
  push 0x72
  call pm_interrupt
  iret

align 16
rm_irq_0x11:
  push 0x73
  call pm_interrupt
  iret

align 16
rm_irq_0x12:
  push 0x74
  call pm_interrupt
  iret

align 16
rm_irq_0x13:
  push 0x75
  call pm_interrupt
  iret

align 16
rm_irq_0x14:
  push 0x76
  call pm_interrupt
  iret

align 16
rm_irq_0x15:
  push 0x77
  call pm_interrupt
  iret

align 16
rm_int_0x15:
  pushf
  cmp ah, 0x4F ; keyboard intercept callback
  je .1
  cmp ah, 0x85 ; SysReq key callback
  je .1
  cmp ah, 0x90
  jb .3
  cmp ah, 0x91
  ja .3
.1:
  popf
  push 0x15
  call pm_interrupt
  jc .2
  btr WORD [esp+4], CARRY_FLAG
  iret
.2:
  bts WORD [esp+4], CARRY_FLAG
  iret
.3:
  popf
  jmp FAR [cs:rm_ivt+15*4]

use32
align 16
pm_irq_0x00: ; System Timer
  pusha
  push 0
  push 0x08
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x01: ; Keyboard
  pusha
  push 0
  push 0x09
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x02: ; IRQ 2 Cascade
  pusha
  push 0
  push 0x0A
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x03: ; Serial Comm2
  pusha
  push 0
  push 0x0B
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x04: ; Serial Comm1
  pusha
  push 0
  push 0x0C
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x05: ; Parallel Printer LPT2
  pusha
  push 0
  push 0x0D
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x06: ; Diskette
  pusha
  push 0
  push 0x0E
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x07: ; Parallel Printer LPT1
  pusha
  push 0
  push 0x0F
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x08: ; Real Time Clock
  pusha
  push 0
  push 0x70
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x09:
  pusha
  push 0
  push 0x71
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x10:
  pusha
  push 0
  push 0x72
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x11:
  pusha
  push 0
  push 0x73
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x12:
  pusha
  push 0
  push 0x74
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x13: ; Math Coprocessor Exception
  pusha
  push 0
  push 0x75
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x14: ; Disk
  pusha
  push 0
  push 0x76
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_irq_0x15:
  pusha
  push 0
  push 0x77
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

align 16
pm_int_0x19: ; Reboot
  mov al, 0xFE
  out 0x64, al
  jmp $+2 ; to keep Qemu happy
  lidt [$]
  int3

use32
align 16
error_iret:
  lea esp, [esp+4]
_iret:
  iret

align 16
Video_0x10:
  pusha
  push 0
  push 0x10
  cmp ah, 0x4F
  je Video_VESA_ReturnVBEControllerInformation_0x104F00
  cmp ah, 0x13
  ja unsupported_function
  movzx ebp, ah
  jmp DWORD [video_functions+ebp*4]
align 4
video_functions:
  dd Video_SetMode_0x1000
  dd Video_SetCursorSize_0x1001
  dd Video_SetCursorPosition_0x1002
  dd Video_GetCursorPositionAndSize_0x1003
  dd unsupported_function
  dd Video_SelectActiveDisplayPage_0x1005
  dd Video_ScrollActivePageUp_0x1006
  dd Video_ScrollActivePageDown_0x1007
  dd Video_ReadCharacterAndAttributeAtCurrentCursorPosition_0x1008
  dd Video_WriteCharacterAndAttributeAtCurrentCursorPosition_0x1009
  dd Video_WriteCharacterAtCurrentCursorPosition_0x100A
  dd Video_SetColorBackground_0x100B00
  dd Video_WritePixel_0x100C
  dd Video_ReadPixel_0x100D
  dd Video_WriteTeletypeToActivePage_0x100E
  dd Video_GetCurrentVideoState_0x100F
  dd Video_ToggleIntensityBlinkingBit_0x101003
  dd unsupported_function
  dd unsupported_function
  dd Video_WriteString_0x1013

Video_SetMode_0x1000:
  ;IN   ah = 0
  ;IN   al = video mode

Video_SetCursorSize_0x1001:
  ;IN   ah = 1
  ;IN   ch = cursor start and options - see 
  ;IN   cl = bottom scan line containing cursor (bits 0-4)

Video_SetCursorPosition_0x1002:
  ;IN   ah = 2
  ;IN   bh = page number (0 to number of pages - 1) - see 
  ;IN   dh = row (0 is top)
  ;IN   dl = column (0 is left)

Video_SelectActiveDisplayPage_0x1005:
  ;IN   ah = 5
  ;IN   al = page number (0 to number of pages - 1) - see 

Video_ScrollActivePageUp_0x1006:
  ;IN   ah = 6
  ;IN   al = number of lines to scroll up (0 = clear entire window)
  ;IN   bh = attribute used to write blank lines at bottom of window
  ;IN   ch = row of window's upper left corner
  ;IN   cl = column of window's upper left corner
  ;IN   dh = row of window's lower right corner
  ;IN   dl = column of window's lower right corner

Video_ScrollActivePageDown_0x1007:
  ;IN   ah = 7
  ;IN   al = number of lines to scroll down (0 = clear entire window)
  ;IN   bh = attribute used to write blank lines at top of window
  ;IN   ch = row of window's upper left corner
  ;IN   cl = column of window's upper left corner
  ;IN   dh = row of window's lower right corner
  ;IN   dl = column of window's lower right corner

Video_WriteCharacterAndAttributeAtCurrentCursorPosition_0x1009:
  ;IN   ah = 9
  ;IN   al = character to display
  ;IN   bh = page number (0 to number of pages - 1) - see 
  ;        = background color in 256-color graphics modes
  ;IN   bl = attribute (text mode) or color (graphics mode) - see 
  ;IN  ecx = number of characters to write

Video_WriteCharacterAtCurrentCursorPosition_0x100A:
  ;IN   ah = 0xA
  ;IN   al = character to display
  ;IN   bh = page number (0 to number of pages - 1) - see 
  ;        = background color in 256-color graphics modes
  ;IN  ecx = number of characters to write

Video_WritePixel_0x100C:
  ;IN   ah = 0xC
  ;IN   al = pixel color
  ;IN   bh = page number
  ;IN  ecx = column
  ;IN  edx = row

Video_WriteTeletypeToActivePage_0x100E:
  ;IN   ah = 0xE
  ;IN   al = character to write
  ;IN   bh = page number
  ;IN   bl = foreground color (graphics modes only)

  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

Video_GetCursorPositionAndSize_0x1003:
  ;IN   ah = 3
  ;IN   bh = page number (0 to number of pages - 1) - see 
  ;OUT  ch = start scan line
  ;OUT  cl = end scan line
  ;OUT  dh = row (0 is top)
  ;OUT  dl = column (0 is left)

  call rm_interrupt
  mov [_cx], cx
  mov [_dx], dx
  lea esp, [esp+8]
  popa
  iret

Video_ReadCharacterAndAttributeAtCurrentCursorPosition_0x1008:
  ;IN   ah = 8
  ;IN   bh = page number (0 to number of pages - 1) - see 
  ;OUT  ah = character's attribute (text mode only) - see 
  ;OUT  al = character

  call rm_interrupt
  mov [_ax], ax
  lea esp, [esp+8]
  popa
  iret

Video_SetColorBackground_0x100B00:
  ;IN   ah = 0xB
  ;IN   bh = 0
  ;IN   bl = background color

  test bh, bh
  jne Video_SetColorPalette_0x100B01
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

Video_SetColorPalette_0x100B01:
  ;IN   ah = 0xB
  ;IN   bh = 1
  ;IN   bl = palette id

  cmp bh, 1
  jne unsupported_function
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

Video_ReadPixel_0x100D:
  ;IN   ah = 0xD
  ;IN   bh = page number
  ;IN  ecx = column
  ;IN  edx = row
  ;OUT  al = pixel color

  call rm_interrupt
  mov [_al], al
  lea esp, [esp+8]
  popa
  iret

Video_GetCurrentVideoState_0x100F:
  ;IN   ah = 0xF
  ;OUT  ah = number of character columns
  ;OUT  al = display mode - see
  ;OUT  bh = active page - see

  call rm_interrupt
  mov [_ax], ax
  mov [_bh], bh
  lea esp, [esp+8]
  popa
  iret

Video_ToggleIntensityBlinkingBit_0x101003:
  ;IN   ah = 0x10
  ;IN   al = 3
  ;IN   bl = new state
  ;          0 background intensity enabled
  ;          1 blinking enabled

  cmp al, 3
  jne unsupported_function
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

Video_WriteString_0x1013:
  ;IN   ah = 0x13
  ;IN   al = write mode
  ;          bit 0: update cursor after writing
  ;          bit 1: string contains alternating characters and attributes
  ;IN   bh = page number.
  ;IN   bl = attribute if string contains only characters.
  ;IN  ecx = number of characters in string.
  ;IN   dh = row at which to start writing.
  ;IN   dl = column at which to start writing.
  ;IN  ebp = string buffer to write

  mov ebp, [_ebp]
  cmp ebp, 0x100000
  jb .2
  mov esi, ebp
  mov edi, data_buffer
  cmp al, 2
  jb .1
  add ecx, ecx
.1:
  cld
  rep movsb
  mov ebp, data_buffer
.2:
  mov ebx, ebp
  and ebx, 0xF0000
  shr ebx, 4
  mov [_es], bx
  mov ebx, [_ebx]
  mov ecx, [_ecx]
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

; VESA BIOS EXTENSION (VBE) Core Functions Standard
; Version: 3.0 September 16, 1998
; http://web.archive.org/web/20041204032351/http://www.vesa.org/vbe3.pdf

Video_VESA:
  btr DWORD [eflags], CARRY_FLAG
  movzx ebp, ah
  jmp DWORD [video_VESA_Functions+ebp*4]
align 4
video_VESA_Functions:
  dd Video_VESA_ReturnVBEControllerInformation_0x104F00
  dd Video_VESA_ReturnVBEModeInformation_0x104F01
  dd Video_VESASetVBEMode_0x104F02
  dd Video_VESAGetVBEMode_0x104F03
  
Video_VESA_ReturnVBEControllerInformation_0x104F00:
  ;IN  eax = 0x4F00
  ;IN  edi = 512 byte buffer for SuperVGA information
  ;OUT  ah = status: 0 - success, 1 - failure
  ;OUT  al = 0x4F if function supported

  cmp edi, 0x100000
  jb .1
  mov edi, data_buffer
.1:
  mov edx, edi
  and edx, 0xF0000
  shr edx, 4
  mov [_es], dx
  call rm_interrupt
  mov [_ax], ax
  cmp ax, 4Fh
  je .2
  or DWORD [eflags], 1
.2:
  cmp DWORD [_edi], 0x100000
  jb .3
  mov edi, [_edi]
  mov esi, data_buffer
  mov ecx, 512/4
  cld
  rep movsd
.3:
  lea esp, [esp+8]
  popa
  iret

Video_VESA_ReturnVBEModeInformation_0x104F01:
  ;IN  eax = 0x4F01
  ;IN  ecx = SuperVGA video mode
  ;IN  edi = 256 byte buffer for mode information
  ;OUT  ah = status: 0 - success, 1 - failure
  ;OUT  al = 0x4F if function supported

  cmp edi, 0x100000
  jb .1
  mov edi, data_buffer
.1:
  mov edx, edi
  and edx, 0xF0000
  shr edx, 4
  mov [_es], dx
  call rm_interrupt
  mov [_ax], ax
  cmp ax, 0x4F
  je .2
  or DWORD [eflags], 1
.2:
  cmp DWORD [_edi], 0x100000
  jb .3
  mov edi, [_edi]
  mov esi, data_buffer
  mov ecx, 256/4
  cld
  rep movsd
.3:
  lea esp, [esp+8]
  popa
  iret

Video_VESASetVBEMode_0x104F02:
  ;IN  eax = 0x4F02
  ;IN  ebx = new video mode
  ;IN  edi = (vbe 3.0+) crtc information block, bit mode bit 11 set
  ;OUT  ah = status: 0 - success, 1 - failure
  ;OUT  al = 0x4F if function supported

  mov edx, edi
  cmp edi, 0x100000
  jb .1
  mov esi, edi
  mov edi, data_buffer
  mov edx, edi
  mov ecx, 64/4
  cld
  rep movsd
  mov edi, edx
.1:
  and edx, 0xF0000
  shr edx, 4
  mov [_es], dx
  call rm_interrupt
  mov [_ax], ax
  cmp ax, 0x4F
  je .2
  or DWORD [eflags], 1
.2:
  lea esp, [esp+8]
  popa
  iret

Video_VESAGetVBEMode_0x104F03:
  ;IN  eax = 0x4F03
  ;OUT  ah = status: 0 - success, 1 - failure
  ;OUT  al = 0x4F if function supported
  ;OUT ebx = new video mode

  call rm_interrupt
  mov [_ax], ax
  cmp ax, 0x4F
  je .1
  or DWORD [eflags], 1
.1:
  movzx ebx, bx
  mov [_ebx], ebx
  lea esp, [esp+8]
  popa
  iret

Drive_0x13:
  pusha
  push 0
  push 0x13
  btr DWORD [eflags], CARRY_FLAG
  movzx ebp, ah
  cmp ebp, 8
  ja Drive_Extensions_0x13
  jmp DWORD [drive_functions+ebp*4]
align 4
drive_functions:
  dd Drive_Reset_0x1300
  dd Drive_GetStatus_0x1301
  dd Drive_ReadSectors_0x1302
  dd Drive_WriteSectors_0x1303
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd Drive_GetDriveParameters_0x1308

Drive_Reset_0x1300:
  ;IN   ah = 0
  ;IN   dl = drive
  ;OUT  ah = status - see

Drive_GetStatus_0x1301:
  ;IN   ah = 1
  ;IN   dl = drive
  ;OUT  ah = status - see

  call rm_interrupt
  mov [_ah], ah
  setc al
  or [eflags], al
  lea esp, [esp+8]
  popa
  iret

Drive_ReadSectors_0x1302:
  ;IN   ah = 2
  ;IN   al = number of sectors to read (must be nonzero)
  ;IN  ebx = data buffer
  ;IN   ch = low eight bits of cylinder number
  ;IN   cl = sector number 1-63 (bits 0-5)
  ;          high two bits of cylinder (bits 6-7)
  ;IN   dh = head number
  ;IN   dl = drive number
  ;OUT  ah = status - see
  ;OUT  al = number of sectors transferred
  cmp ebx, 0x100000
  jb .1
  mov ebx, DRIVE_BUFFER
.1:
  mov ebp, ebx
  and ebp, 0xF0000
  shr ebp, 4
  mov [_es], bp
  call rm_interrupt
  mov [_ax], ax
  jnc .2
  or DWORD [eflags], 1
.2:
  cmp DWORD [_ebx], 0x100000
  jb .3
  mov edi, [_ebx]
  movzx ecx, BYTE [_al]
  shl ecx, 7 ; *512/4
  mov esi, DRIVE_BUFFER
  cld
  rep movsd
.3:
  lea esp, [esp+8]
  popa
  iret

Drive_WriteSectors_0x1303:
  ;IN   ah = 3
  ;IN   al = number of sectors to read (must be nonzero)
  ;IN  ebx = data buffer
  ;IN   ch = low eight bits of cylinder number
  ;IN   cl = sector number 1-63 (bits 0-5)
  ;          high two bits of cylinder (bits 6-7)
  ;IN   dh = head number
  ;IN   dl = drive number
  ;OUT  ah = status - see
  ;OUT  al = number of sectors transferred

  cmp ebx, 0x100000
  jb .1
  mov esi, ebx
  mov ebp, DRIVE_BUFFER
  mov edi, ebp
  movzx ecx, al
  shl ecx, 7 ; *512/4
  cld
  rep movsd
  mov ebx, ebp 
.1:
  mov edx, ebx
  and edx, 0xF0000
  shr edx, 4
  mov [_es], dx
  mov ecx, [_ecx]
  mov edx, [_edx]
  call rm_interrupt
  mov [_ax], ax
  setc al
  or [eflags], al
  lea esp, [esp+8]
  popa
  iret

Drive_GetDriveParameters_0x1308:
  ;IN   ah = 8
  ;IN   dl = drive number
  ;OUT  ah = status - see
  ;OUT  ch = low eight bits of maximum cylinder number
  ;OUT  cl = maximum sector number (bits 5-0)
  ;          high two bits of maximum cylinder number (bits 7-6)
  ;OUT  dh = maximum head number
  ;OUT  dl = number of drives

  call rm_interrupt
  mov [_ax], ax
  mov [_cx], cx
  mov [_dx], dx
  setc al
  or [eflags], al
  lea esp, [esp+8]
  popa
  iret

Drive_Extensions_0x13:
  sub ebp, 41h
  cmp ebp, 8
  ja unsupported_function
  jmp DWORD [drive_extension_functions+ebp*4]
align 4
drive_extension_functions:
  dd Drive_ExtensionsInstallationCheck_0x1341
  dd Drive_ExtensionsReadSectors_0x1342
  dd Drive_ExtensionsWriteSectors_0x1343
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd Drive_ExtensionsGetDriveParameters_0x1348
  dd Drive_ExtensionsMediaChange_0x1349

Drive_ExtensionsInstallationCheck_0x1341:
  ;IN   ah = 0x41
  ;IN  ebx = 0x55AA
  ;IN   dl = drive number
  ;OUT  ah = major version of extensions - see
  ;OUT ebx = 0xAA55 if installed
  ;OUT ecx = api subset support bitmap - see
  ;OUT  dh = extension version

  call rm_interrupt
  mov [_ah], ah
  jnc .1
  or DWORD [eflags], 1
.1:
  movzx ebx, bx
  mov [_ebx], ebx
  cmp bx, 0xAA55
  jne .2
  movzx ecx, cx
  mov [_ecx], ecx
  mov [_dh], dh
.2:
  lea esp, [esp+8]
  popa
  iret

Drive_ExtensionsReadSectors_0x1342:
  ;IN   ah = 0x42
  ;IN   dl = drive number
  ;IN  esi = disk address packet - see
  ;OUT  ah = status - see
  ;          disk address packet's block count field set to number
  ;          of blocks successfully transferred

  mov ebx, disk_address_packet
  mov edi, ebx
  mov ecx, 4
  cld
  rep movsd
  mov esi, ebx
  mov ebx, [esi+4] ; buffer
  cmp ebx, 0x100000
  jb .1
  mov DWORD [esi+4], data_buffer
.1:
  ror WORD [esi+6], 4 ; segment
  call rm_interrupt
  mov [_ah], ah
  jnc .2
  or DWORD [eflags], 1
.2:
  mov edi, [disk_address_packet+4] ; buffer
  cmp edi, 0x100000
  jb .3
  mov esi, DRIVE_BUFFER
  movzx ecx, WORD [disk_address_packet+2] ; count
  shr ecx, 2
  cld
  rep movsd
.3:
  lea esp, [esp+8]
  popa
  iret

Drive_ExtensionsWriteSectors_0x1343:
  ;IN   ah = 0x43
  ;IN   dl = drive number
  ;IN  esi = disk address packet - see
  ;OUT  ah = status - see
  ;          disk address packet's block count field set to number
  ;          of blocks successfully transferred

  xor al, al ; ignore potental write verify
  mov ebx, disk_address_packet
  mov edi, ebx
  mov ecx, 4
  cld
  rep movsd
  mov esi, ebx
  mov ebx, [esi+4] ; buffer
  cmp ebx, 0x100000
  jb .1
  mov DWORD [esi+4], data_buffer
.1:
  ror WORD [esi+6], 4 ; segment
  xchg ebx, esi
  mov edi, DRIVE_BUFFER
  movzx ecx, WORD [ebx+2] ; count
  shr ecx, 2
  cld
  rep movsd
  mov esi, ebx
.2:
  call rm_interrupt
  mov [_ah], ah
  setc al
  or [eflags], al
  lea esp, [esp+8]
  popa
  iret

Drive_ExtensionsGetDriveParameters_0x1348:
  ;IN   ah = 0x48
  ;IN   dl = drive number
  ;IN   esi = buffer for drive parameters - see
  ;OUT  ah = status - see

  cmp esi, 0x100000
  jb .1
  mov esi, data_buffer
.1:
  mov ebx, esi
  and ebx, 0xF0000
  shr ebx, 4
  mov [_ds], bx
  call rm_interrupt
  mov [_ah], ah
  jnc .2
  or DWORD [eflags], 1
.2:
  mov edi, [_esi]
  cmp edi, 0x100000
  jb .3
  mov esi, data_buffer
  movzx ecx, WORD [edi] ; size of buffer
  shr ecx, 1
  cld
  rep movsw
.3:
  lea esp, [esp+8]
  popa
  iret

Drive_ExtensionsMediaChange_0x1349:
  ;IN   ah = 0x48
  ;IN   dl = drive number
  ;OUT  ah = 0, media not changed
  ;        = 6, media may have changed

  call rm_interrupt
  mov [_ah], ah
  setc al
  or [eflags], al
  lea esp, [esp+8]
  popa
  iret

align 16
Serial_0x14:
  pusha
  push 0
  push 0x14
  btr DWORD [eflags], CARRY_FLAG
  movzx ebp, ah
  cmp ebp, 5
  ja unsupported_function
  jmp DWORD [serial_functions+ebp*4]
align 4
serial_functions:
  dd Serial_Initialize_0x1400
  dd Serial_WriteCharacter_0x1401
  dd Serial_ReadCharacter_0x1402
  dd Serial_GetStatus_0x1403
  dd Serial_ExtendedInitialize_0x1404
  dd Serial_ExtendedControl_0x1405

Serial_Initialize_0x1400:
  ;IN   ah = 0
  ;IN   al = port parameters - see
  ;IN  edx = port number (0-3)
  ;OUT  ah = port status - see
  ;OUT  al = modem status - see
  ;          carry set on error

Serial_ReadCharacter_0x1402:
  ;IN   ah = 2
  ;IN  edx = port number (0-3)
  ;OUT  ah = line status - see
  ;OUT  al = received character if ah bit 7 clear
  ;          carry set on error

Serial_GetStatus_0x1403:
  ;IN   ah = 3
  ;IN  edx = port number (0-3)
  ;OUT  ah = line status - see
  ;OUT  al = modem status - see
  ;          carry set on error

  call rm_interrupt
  mov [_ax], ax
  rol ah, 1
  and ah, 1
  or [eflags], ah ; carry flag
  lea esp, [esp+8]
  popa
  iret

Serial_WriteCharacter_0x1401:
  ;IN   ah = 1
  ;IN   al = character to write
  ;IN  edx = port number (0-3)
  ;OUT  ah = port status - see
  ;          carry set on error

Serial_ExtendedInitialize_0x1404:
  ;IN   ah = 4
  ;IN   al = break status
  ;          0 if break
  ;          1 if no break
  ;IN   bh = parity - see
  ;IN   bl = number of stop bits
  ;          0 one stop bit
  ;          1 two stop bits (1.5 if 5 bit word length)
  ;IN   ch = word length - see
  ;IN   cl = bps rate - see
  ;IN  edx = port number
  ;OUT  ah = port status - see
  ;          carry set on error

  call rm_interrupt
  mov [_ah], ah
  rol ah, 1
  and ah, 1
  or [eflags], ah ; carry flag
  lea esp, [esp+8]
  popa
  iret

Serial_ExtendedControl_0x1405:
  ;IN   ah = 5
  ;IN   al = 0 read modem control register
  ;OUT  bl = modem control register - see

  ;IN   al = 1 write modem control register
  ;IN   bl = modem control register - see

  ;IN  edx = port number
  ;OUT  ah = port status - see
  ;          carry set on error

  call rm_interrupt
  cmp BYTE [_al], 0
  jne .1
  mov [_bl], bl
.1:
  mov [_ah], ah
  rol ah, 1
  and ah, 1
  or [eflags], ah ; carry flag
  lea esp, [esp+8]
  popa
  iret

align 16
System_0x15:
  pusha
  push 0
  push 0x15
  btr DWORD [eflags], CARRY_FLAG
System_0x154F:
  ;IN   ah = 0x4F
  ;IN   al = hardware scan code - see

  cmp ah, 0x4F
  jne .1
  or DWORD [eflags], 1
  lea esp, [esp+8]
  popa
  iret
.1:
  movzx ebp, ah
  sub ebp, 0x80
  cmp ebp, 17
  ja System_MemoryMap_0x15E820
  jmp DWORD [system_functions+ebp*4]
align 4
system_functions:
  dd System_DeviceOpen_0x1580
  dd System_DeviceClose_0x1581
  dd System_ProgramTermination_0x1582
  dd System_SetFlagAfterTimeInterval_0x1583
  dd unsupported_function
  dd System_SysReqKeyCallback_0x1585
  dd System_Wait_0x1586
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd unsupported_function
  dd System_DeviceBusy_0x1590
  dd System_DeviceComplete_0x1591

System_DeviceOpen_0x1580:
  ;IN   ah = 0x80
  ;IN  ebx = device id
  ;IN  ecx = process id

System_DeviceClose_0x1581:
  ;IN   ah = 0x81
  ;IN  ebx = device id
  ;IN  ecx = process id

System_ProgramTermination_0x1582:
  ;IN   ah = 0x82
  ;IN  ecx = process id

System_SysReqKeyCallback_0x1585:
  ;IN   ah = 0x85
  ;IN   al = sysreq key action (0 - pressed, 1 - released)

System_DeviceBusy_0x1590:
  ;IN   ah = 0x90
  ;IN   al = device type - see
  ;IN  ebx = request block for type codes 80h through BFh

  mov BYTE [_ah], 0
  lea esp, [esp+8]
  popa
  iret

System_SetFlagAfterTimeInterval_0x1583:
  ;IN   ah = 0x83
  ;IN   al = subfunction (0 - set interval, 1 - cancel wait interval)
  ;IN  ebx = low memory byte whose high bit is to be set at end of interval
  ;IN  edx = microseconds to delay
  ;OUT  ah = status - see
  ;OUT carry set if function busy

  cmp al, 1
  je .2
  test al, al
  jne unsupported_function
  cmp ebx, 0x100000
  jb .1
  mov BYTE [_ah], ERROR
  or DWORD [eflags], 1
  lea esp, [esp+8]
  popa
  iret
.1:
  mov edx, ebx
  and edx, 0xF0000
  shr edx, 4
  mov [_es], dx
  mov ecx, [_edx]
  mov edx, ecx
  shr ecx, 16
.2:
  call rm_interrupt
  mov [_ah], ah
  jnc .3
  or DWORD [eflags], 1
.3:
  lea esp, [esp+8]
  popa
  iret
  
System_Wait_0x1586:
  ;IN   ah = 0x86
  ;IN  edx = interval in microseconds
  ;OUT  ah = status - see
  ;OUT carry set if wait in progress

  mov ecx, edx
  shr ecx, 16
  call rm_interrupt
  mov [_ah], ah
  jnc .1
  or DWORD [eflags], 1
.1:
  lea esp, [esp+8]
  popa
  iret

System_DeviceComplete_0x1591:
  ;IN   ah = 0x91
  ;IN   al = device type - see
  ;IN  ebx = request block for type codes 0x80 through 0xBF

  setc al
  or [eflags], al
  lea esp, [esp+8]
  popa
  iret

System_MemoryMap_0x15E820:
  ;IN  eax = 0xE820
  ;IN  ebx = continuation value or 0 to start at beginning of map
  ;IN  ecx = size of buffer for result, in bytes (should be >= 20)
  ;IN  edx = 0x534D4150 ('smap')
  ;IN  edi = buffer for result - see
  ;OUT eax = 0x534D4150 ('smap')
  ;OUT ebx = next offset from which to copy or 0 if all done
  ;OUT ecx = actual length returned in bytes

  ;error flag (carry flag) set on error
  ;OUT  ah = error code (0x86 - function not supported)

  cmp ax, 0xE820
  jne unsupported_function
  cmp edi, 0x100000
  jb .1
  mov edi, data_buffer
.1:
  mov ebp, edi
  and ebp, 0xF0000
  shr ebp, 4
  mov [_es], bp
  call rm_interrupt
  jnc .2
  or DWORD [eflags], 1
.2:
  mov [_eax], eax
  mov [_ebx], ebx
  mov [_ecx], ecx
  cmp DWORD [_edi], 0x100000
  jb .3
  mov edi, [_edi]
  mov esi, data_buffer
  shr ecx, 2
  cld
  rep movsd
.3:
  lea esp, [esp+8]
  popa
  iret

align 16
Keyboard_0x16:
  pusha
  push 0
  push 0x16
  mov dl, ah
  sub dl, 0x10
  cmp dl, 2
  ja .1
  sub ah, 0x10
.1:
  movzx ebp, ah
  cmp ebp, 5
  ja unsupported_function
  jmp DWORD [keyboard_functions+ebp*4]
align 4
keyboard_functions:
  dd Keyboard_GetInput_0x1600
  dd Keyboard_GetStatus_0x1601
  dd Keyboard_GetShiftFlagStatus_0x1602
  dd Keyboard_SetTypematicDelayAndRate_0x1603
  dd unsupported_function
  dd Keyboard_PutData_0x1605

Keyboard_GetInput_0x1600:
  ;IN   ah = 0
  ;OUT  ah = bios scan code
  ;OUT  al = ascii character

Keyboard_GetShiftFlagStatus_0x1602:
  ;IN   ah = 2
  ;OUT  al = shift eflags 1 - see
  ;OUT  ah = shift eflags 2 - see

  call rm_interrupt
  mov [_ax], ax
  lea esp, [esp+8]
  popa
  iret

Keyboard_GetStatus_0x1601:
  ;IN   ah = 1
  ;OUT zero flag clear if keystroke available
  ;OUT  ah = bios scan code
  ;OUT  al = ascii character

  ;OUT zero flag set if no keystroke available

  btr DWORD [eflags], ZERO_FLAG
  call rm_interrupt
  jz .1
  mov [_ax], ax
  bts DWORD [eflags], ZERO_FLAG
.1:
  lea esp, [esp+8]
  popa
  iret

Keyboard_SetTypematicDelayAndRate_0x1603:
  ;IN   ah = 3
  ;IN   al = 5 - set repeat rate and delay
  ;IN   bh = delay value (0 = 250ms to 3 = 1000ms)
  ;IN   bl = repeat rate (0 = 30/sec to 12 = 10/sec [default] to 31 = 2/sec)

  cmp al, 5
  jne Keyboard_GetTypematicDelayAndRate_0x160306
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

Keyboard_GetTypematicDelayAndRate_0x160306:
  ;IN   ah = 3
  ;IN   al = 6 - get repeat rate and delay
  ;OUT  bh = delay value (0 = 250ms to 3 = 1000ms)
  ;OUT  bl = repeat rate (0 = 30/sec to 12 = 10/sec [default] to 31 = 2/sec)

  cmp al, 6
  jne unsupported_function
  call rm_interrupt
  mov [_bx], bx
  lea esp, [esp+8]
  popa
  iret

Keyboard_PutData_0x1605:
  ;IN   ah = 5
  ;IN   ch = bios scan code
  ;IN   cl = ascii character
  ;OUT  al = status, 0 - successful, 1 - keyboard buffer full
  ;OUT zero flag clear if successful, set if buffer full [NEW]

  btr DWORD [eflags], ZERO_FLAG
  call rm_interrupt
  mov [_al], al
  and al, 1
  shl al, 6
  or [eflags], al ; zero flag
  lea esp, [esp+8]
  popa
  iret

align 16
Parallel_0x17:
  pusha
  push 0
  push 0x17
  movzx ebp, ah
  cmp ebp, 2
  ja unsupported_function
  jmp DWORD [parallel_functions+ebp*4]
align 4
parallel_functions:
  dd Parallel_WriteCharacter_0x1700
  dd Parallel_Initialize_0x1701
  dd Parallel_GetStatus_0x1702

Parallel_WriteCharacter_0x1700:
  ;IN   ah = 0
  ;IN   al = character to write
  ;IN  edx = port number (0-2)
  ;OUT  ah = printer status - see
  ;          carry set on error

Parallel_Initialize_0x1701:
  ;IN   ah = 1
  ;IN  edx = port number (0-2)
  ;OUT  ah = printer status - see
  ;          carry set on error

Parallel_GetStatus_0x1702:
  ;IN   ah = 2
  ;IN  edx = port number (0-2)
  ;OUT  ah = printer status - see
  ;          carry set on error

  call rm_interrupt
  mov [_ah], ah
  shr ah, 3
  and ah, 1
  or [eflags], ah ; carry flag
  lea esp, [esp+8]
  popa
  iret

align 16
Time_0x1A:
  pusha
  push 0
  push 0x1A
  cmp ah, 7
  ja unsupported_function
  movzx ebp, ah
  jmp DWORD [time_functions+ebp*4]
align 4
time_functions:
  dd Time_GetSystemTime_0x1A00
  dd Time_SetSystemTime_0x1A01
  dd Time_GetTime_0x1A02
  dd Time_SetTime_0x1A03
  dd Time_GetDate_0x1A04
  dd Time_SetDate_0x1A05
  dd Time_SetAlarm_0x1A06
  dd Time_CancelAlarm_0x1A07

Time_GetSystemTime_0x1A00:
  ;IN   ah = 0
  ;OUT edx = number of clock ticks since midnight
  ;OUT  al = midnight flag, nonzero if midnight passed since time last read

  call rm_interrupt
  mov [_al], al
  shl ecx, 16
  mov cx, dx
  mov [_edx], ecx
  lea esp, [esp+8]
  popa
  iret

Time_SetSystemTime_0x1A01:
  ;IN   ah = 1
  ;IN  edx = number of clock ticks since midnight

  mov ecx, edx
  shr ecx, 16
  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

Time_GetTime_0x1A02:
  ;IN   ah = 2
  ;OUT  ch = hour (bcd)
  ;OUT  cl = minutes (bcd)
  ;OUT  dh = seconds (bcd)
  ;OUT  dl = daylight savings flag (0 standard time, 1 daylight time)
  ;OUT carry flag set on error (i.e. clock not running or in middle of update)

Time_GetDate_0x1A04:
  ;IN   ah = 4
  ;OUT  ch = century (bcd)
  ;OUT  cl = year (bcd)
  ;OUT  dh = month (bcd)
  ;OUT  dl = day (bcd)
  ;OUT carry flag set on error

  call rm_interrupt
  mov [_cx], cx
  mov [_dx], dx
  jnc .1
  mov BYTE [_ah], ERROR
  or DWORD [eflags], 1
.1:
  lea esp, [esp+8]
  popa
  iret

Time_SetTime_0x1A03:
  ;IN   ah = 3
  ;IN   ch = hour (bcd)
  ;IN   cl = minutes (bcd)
  ;IN   dh = seconds (bcd)
  ;IN   dl = daylight savings flag (0 standard time, 1 daylight time)

Time_SetDate_0x1A05:
  ;IN   ah = 5
  ;IN   ch = century (bcd)
  ;IN   cl = year (bcd)
  ;IN   dh = month (bcd)
  ;IN   dl = day (bcd)

Time_CancelAlarm_0x1A07:
  ;IN   ah = 7

  call rm_interrupt
  lea esp, [esp+8]
  popa
  iret

Time_SetAlarm_0x1A06:
  ;IN   ah = 6
  ;IN   ch = hour (bcd)
  ;IN   cl = minutes (bcd)
  ;IN   dh = seconds (bcd)
  ;OUT carry flag set on error (alarm already set or clock stopped for update)

  call rm_interrupt
  jnc .1
  mov BYTE [_ah], ERROR
  or DWORD [eflags], 1
.1:
  lea esp, [esp+8]
  popa
  iret

unsupported_function:
  mov BYTE [_ah], UNSUPPORTED_FUNCTION
  or DWORD [eflags], 1
  lea esp, [esp+8]
  popa
  iret

align 8
gdt:
  dw 0x0000, 0, 0x0000, 0x00 ; null selector
  dw 0xFFFF, 0, 0x9A00, 0xCF ; maximum pm code selector
  dw 0xFFFF, 0, 0x9200, 0xCF ; maximum pm data selector
  dw 0xFFFF, 0, 0x9A00, 0x00 ; 64Kb rm code selector
  dw 0xFFFF, 0, 0x9200, 0x00 ; 64Kb rm data selector

align 4
_CR0_: dd 1
_esp: dd 0
error_code: dd 0
disk_address_packet: dd 0, 0, 0, 0                    

A20_error: db "Failed to enable A20 line!", 0Dh, 0Ah
press_any_key: db "Press any key to continue ...", 0Dh, 0Ah, 0
must_be_386: db "FATAL: Must be at least an 80386!", 0Dh, 0Ah, 0

rm_ivt: times 200h dd 0
data_buffer: times 200h dd 0

times ((27*512)-48)-($-$$) db 0

db "look and see -     many look         but few see"
