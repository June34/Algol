// Copyright (c) 2016 Angel Terrones (<angelterrones@gmail.com>)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


// taken from syscalls.c @ riscv-tests repo.

#include "encoding.h"

#define SYS_write 64
#define SYS_exit 93
#define SYS_stats 1234

#define read_csr_safe(reg) ({ register long __tmp asm("a0");    \
            asm volatile ("csrr %0, " #reg : "=r"(__tmp));      \
            __tmp; })

static long syscall(long num, long arg0, long arg1, long arg2)
{
    register long a7 asm("a7") = num;
    register long a0 asm("a0") = arg0;
    register long a1 asm("a1") = arg1;
    register long a2 asm("a2") = arg2;
    asm volatile ("scall" : "+r"(a0) : "r"(a1), "r"(a2), "r"(a7));
    return a0;
}

void tohost_exit(long code)
{
    write_csr(mtohost, (code << 1) | 1);
    while (1);
}

void exit(int code){
    syscall(SYS_exit, code, 0, 0);
    while(1);
}

long handle_trap(long cause, long epc, long regs[32]){
    // do some magic
    int *csr_ins;
    asm("jal %0, 1f; nop; 1:" : "=r"(csr_ins));

    if(cause == CAUSE_ILLEGAL_INSTRUCTION && (*(int *)epc & *csr_ins) == *csr_ins)
        while(1);
    else if(cause != CAUSE_USER_ECALL)
        tohost_exit(1337);
    else if(regs[17] == SYS_write){
        char * const print_addr = (char *)0x80000000;
        char * const buf = (char *const)regs[11];
        const int bytes = regs[12];

        for(int i = 0; i < bytes; i++)
            *print_addr = buf[i];
    }
    else if(regs[17] == SYS_exit)
        tohost_exit(regs[10]);
    else
        while(1);

    return epc + 4;
}

int __attribute((weak)) main(int argc, char** argv){
    (void)argc;
    (void)argv;
    return -1;
}

void _init(){
    int ret = main(0, 0);
    exit(ret);
}

#undef strlen
int strlen(const char *s){
    int size = 0;
    while(s[size] != '\0')
        size++;
    return size;
}

void printstr(const char* s)
{
    syscall(SYS_write, 1, (long)s, strlen(s));
}

#undef putchar
int putchar(int ch){
    static char buf[64] __attribute__((aligned(64)));
    static int buflen = 0;

    buf[buflen++] = ch;

    if (ch == '\n' || buflen == sizeof(buf)){
        syscall(SYS_write, 1, (long)buf, buflen);
        buflen = 0;
    }

    return 0;
}
