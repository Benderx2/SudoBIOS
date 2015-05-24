typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;

struct regs{
  union{
    DWORD edi;
    WORD di;
  };
  union{
    DWORD esi;
    WORD si;
  };
  union{
    DWORD ebp;
    WORD bp;
  };
  struct{
    WORD ds;
    WORD es;
  };
  union{
    DWORD ebx;
    WORD bx;
    struct{
      BYTE bl;
      BYTE bh;
    };
  };
  union{
    DWORD edx;
    WORD dx;
    struct{
      BYTE dl;
      BYTE dh;
    };
  };
  union{
    DWORD ecx;
    WORD cx;
    struct{
      BYTE cl;
      BYTE ch;
    };
  };
  union{
    DWORD eax;
    WORD ax;
    struct{
      BYTE al;
      BYTE ah;
    };
  };
  union{
    DWORD eflags;
    struct{
      DWORD _carry : 1;
      DWORD __f2__ : 5;
      DWORD _zero : 1;
      DWORD __f3__ : 25;
    };
  };
} __attribute__ ((__packed__));

struct regs regs;
DWORD _esp;

#define CARRY regs._carry
#define carry regs._carry
#define ZERO regs._zero
#define zero regs._zero

#define ERROR regs._carry
#define error regs._carry
#define READY regs._zero
#define ready regs._zero

#define eflags regs.eflags
#define ah regs.ah
#define al regs.al
#define ax regs.ax
#define eax regs.eax
#define ch regs.ch
#define cl regs.cl
#define cx regs.cx
#define ecx regs.ecx
#define dh regs.dh
#define dl regs.dl
#define dx regs.dx
#define edx regs.edx
#define bh regs.bh
#define bl regs.bl
#define bx regs.bx
#define ebx regs.ebx
#define es regs.es
#define ds regs.ds
#define bp regs.bp
#define ebp regs.ebp
#define si regs.si
#define esi regs.esi
#define di regs.di
#define edi regs.edi

void pre_int(void){
  __asm__ __volatile__(
 "  pusha\n\t"
 "  mov [_esp], esp\n\t"
 "  cli\n\t"
 "  mov esp, offset regs\n\t"
 "  popa\n\t"
 "  mov esp, [_esp]\n\t"
 "  sti\n\t"
  : : : "memory"
  );
}

void post_int(void){
  __asm__ __volatile__(
 "  mov [_esp], esp\n\t"
 "  cli\n\t"
 "  mov esp, offset regs+36\n\t"
 "  pushf\n\t"
 "  pusha\n\t"
 "  mov esp, [_esp]\n\t"
 "  sti\n\t"
 "  popa\n\t"
  : : : "memory"\
  );
}

#define int_0x10 \
__asm__ __volatile__( \
"  call pre_int\n\t" \
"  int 0x10\n\t" \
"  call post_int\n\t" \
)

#define int_0x13 \
__asm__ __volatile__( \
"  call pre_int\n\t" \
"  int 0x13\n\t" \
"  call post_int\n\t" \
)

#define int_0x14 \
__asm__ __volatile__( \
"  call pre_int\n\t" \
"  int 0x14\n\t" \
"  call post_int\n\t" \
)

#define int_0x15 \
__asm__ __volatile__( \
"  call pre_int\n\t" \
"  int 0x15\n\t" \
"  call post_int\n\t" \
)

#define int_0x16 \
__asm__ __volatile__( \
"  call pre_int\n\t" \
"  int 0x16\n\t" \
"  call post_int\n\t" \
)

#define int_0x17 \
__asm__ __volatile__( \
"  call pre_int\n\t" \
"  int 0x17\n\t" \
"  call post_int\n\t" \
)

#define int_0x19 \
__asm__ __volatile__( \
"  int 0x19\n\t" \
)
 
#define int_0x1A \
__asm__ __volatile__( \
"  call pre_int\n\t" \
"  int 0x1A\n\t" \
"  call post_int\n\t" \
)
