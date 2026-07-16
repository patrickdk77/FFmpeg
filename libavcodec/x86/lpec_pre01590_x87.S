/*
 * radix-3 preprocess -- x87 port (Intel syntax, x86_64 SysV).
 * void lpec_pre01590_x87(double *lo, double *hi, int n,
 * const double *cos_wr, const double *cos_wi);
 * void lpec_pre01590_x87_iter(int k, int u, int ecx_off, int edi_byte,
 * double *lo, double *hi,
 * const double *cos_wr, const double *cos_wi,
 * double *defer);
 * Caller writes hi[(edi_byte-8)/8] from *defer when k>0 before calling.
 * On return, *defer holds the deferred value for the next iteration.
 * LPEC_PRE01590_X87=1 LPEC_PRE01590_NOFIX=1 -> full x87 (bisect).
 * LPEC_PRE01590_HYBRID=1 -> C NOFIX k<fix_k, x87 per-k k>=fix_k.
 *
 * Copyright (c) 2026 Patrick Domack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
    .section .note.GNU-stack,"",@progbits

    .text
    .intel_syntax noprefix
    .globl lpec_pre01590_x87
    .globl lpec_pre01590_x87_n
    .globl lpec_pre01590_x87_iter
    .globl lpec_pre01590_x87_dump_postbfly
    .globl lpec_pre01590_dump_enable
    .extern lpec_pre01590_st_ring
    .extern lpec_pre01590_st_count
    .type  lpec_pre01590_x87, @function
    .type  lpec_pre01590_x87_n, @function
    .type  lpec_pre01590_x87_iter, @function

    .equ L_U,        0
    .equ L_ECX_OFF,  8
    .equ LFRAME,     16
    .equ L_TW_STEP,  20

lpec_pre01590_x87:
    /* max_iters = 0 => run all (compat). */
    xor     eax, eax
    jmp     .Lx87_common

lpec_pre01590_x87_n:
    mov     eax, r9d                 /* max_iters (6th SysV arg) */

.Lx87_common:
    push    rbx
    push    r12
    push    r13
    push    r14
    push    r15
    sub     rsp, LFRAME + 12

    mov     [rsp + LFRAME], eax      /* max_iters, 0 = all */

    mov     r10, rdi
    mov     r11, rsi
    mov     r12, rcx
    mov     r13, r8

    mov     r15d, edx                /* n */
    mov     eax, r15d
    xor     edx, edx
    mov     ecx, 3
    div     ecx
    test    eax, eax
    jle     .Ldone

    mov     [rsp + L_U], eax

    /* tw_step = 1536 / n (@015ce: 0x1800 / (4*n)) */
    mov     eax, 1536
    xor     edx, edx
    mov     ecx, r15d
    div     ecx
    mov     [rsp + L_TW_STEP], eax

    xor     r14d, r14d
    mov     ebx, eax
    shl     ebx, 4
    xor     r9d, r9d
    mov     [rsp + L_ECX_OFF], r9d

    finit
    fld     qword ptr [rip + .Lrt3h]

    jmp     .Lloop_entry

.Lloop_defer:
    fstp    qword ptr [r11 + rbx - 8]

.Lloop_entry:
    mov     ecx, [rsp + L_ECX_OFF]

    fld     qword ptr [r10 + rbx]
    fld     qword ptr [r10 + rbx]
    movsxd  r8, r14d
    fld     qword ptr [r10 + r8*8]
    fld     qword ptr [r10 + r8*8]
    fld     qword ptr [rip + .Lrt3h]
    fld     qword ptr [rip + .Lhalf]
    fxch    st(5)

    mov     eax, [rsp + L_U]
    lea     r15d, [ecx + eax*8]
    add     ecx, 8
    mov     [rsp + L_ECX_OFF], ecx

    mov     r9d, [rsp + L_TW_STEP]
    call    .Lcore

    add     ebx, 8
    mov     eax, [rsp + LFRAME]
    test    eax, eax
    jz      .Lx87_no_cap
    cmp     r14d, eax
    jge     .Lx87_final_defer
.Lx87_no_cap:
    cmp     r14d, [rsp + L_U]
    jl      .Lloop_defer

.Lx87_final_defer:
    fstp    qword ptr [r11 + rbx - 8]
    fstp    qword ptr [rsp + L_U]

.Ldone:
    add     rsp, LFRAME + 12
    pop     r15
    pop     r14
    pop     r13
    pop     r12
    pop     rbx
    ret

/*
 * rdi=k rsi=u edx=ecx_off ecx=edi_byte r8=lo r9=hi
 * [rsp+8]=cos_wr [rsp+16]=cos_wi [rsp+24]=defer*
*/
lpec_pre01590_x87_iter:
    push    rbx
    push    r12
    push    r13
    push    r14
    push    r15
    push    rbp
    sub     rsp, 8

    mov     r10, r8
    mov     r11, r9
    mov     r14d, edi
    mov     [rsp], esi               /* u */
    mov     ebx, ecx                 /* edi_byte */
    mov     ecx, edx                 /* ecx_off */

    mov     r12, [rsp + 64]
    mov     r13, [rsp + 72]
    mov     rbp, [rsp + 80]

    /* tw_step = 1536 / (3*u) = 512 / u */
    mov     eax, 512
    xor     edx, edx
    div     dword ptr [rsp]          /* u */
    mov     [rsp + 4], eax           /* tw_step (above saved u on iter stack) */

    test    r14d, r14d
    jnz     .Lno_rt3_seed
    finit
    fld     qword ptr [rip + .Lrt3h]
.Lno_rt3_seed:

    movsxd  r8, r14d
    fld     qword ptr [r10 + rbx]
    fld     qword ptr [r10 + rbx]
    fld     qword ptr [r10 + r8*8]
    fld     qword ptr [r10 + r8*8]
    fld     qword ptr [rip + .Lrt3h]
    fld     qword ptr [rip + .Lhalf]
    fxch    st(5)

    mov     eax, [rsp]
    lea     r15d, [ecx + eax*8]

    mov     r9d, [rsp + 4]           /* tw_step */
    call    .Lcore

    fstp    qword ptr [rbp]

    add     rsp, 8
    pop     rbp
    pop     r15
    pop     r14
    pop     r13
    pop     r12
    pop     rbx
    ret

/* Shared: lo/hi radix-3 iter + twiddles. r10=lo r11=hi r12=cos r13=cos_wi
 * r14d=k (inc before tw), r15=i1 byte, r8=k, ebx=i2 byte. One value on stack.*/
.Lcore:
    fadd    qword ptr [r10 + r15]
    fxch    st(4)
    fsubr   qword ptr [r10 + r15]
    fmul    st, st(6)
    fxch    st(4)
    fadd    st(3), st
    fmulp   st(5), st
    fxch    st(2)
    fstp    qword ptr [r10 + r8*8]

    fld     qword ptr [r11 + rbx]
    fadd    qword ptr [r11 + r15]
    fld     qword ptr [r11 + r15]
    fsub    qword ptr [r11 + rbx]
    fmul    st, st(6)
    fld     qword ptr [r11 + r8*8]
    fadd    st, st(2)
    fxch    st(4)
    fmulp   st(2), st
    fld     qword ptr [r11 + r8*8]
    fchs
    faddp   st(2), st
    fxch    st(3)
    fstp    qword ptr [r11 + r8*8]

    fld     st(0)
    fsub    st, st(4)
    fxch    st(5)
    fsubp   st(2), st
    faddp   st(3), st
    fld     st(0)
    fsub    st, st(2)
    fxch    st(1)
    faddp   st(2), st

    push    rax
    mov     rax, rsp
    and     rsp, -16
    sub     rsp, 128
    fnsave  [rsp]
    cmp     dword ptr [rip + lpec_pre01590_dump_enable], 0
    je      1f
    mov     edx, dword ptr [rip + lpec_pre01590_st_count]
    cmp     edx, 128
    jge     1f
    shl     edx, 7
    lea     rcx, [rip + lpec_pre01590_st_ring]
    add     rcx, rdx
    mov     edx, 27
2:
    mov     esi, [rsp + rdx*4 - 4]
    mov     [rcx + rdx*4 - 4], esi
    sub     edx, 1
    jnz     2b
    mov     [rcx + 108], r14d
    inc     dword ptr [rip + lpec_pre01590_st_count]
1:
    frstor  [rsp]
    mov     rsp, rax
    pop     rax

    inc     r14d

    mov     ecx, r14d
    dec     ecx
    imul    ecx, r9d
    fld     qword ptr [r12 + rcx*8]
    fmul    st, st(4)
    fld     qword ptr [r13 + rcx*8]
    fmul    st, st(2)
    faddp   st(1), st
    fxch    st(1)
    fmul    qword ptr [r12 + rcx*8]
    fxch    st(4)
    fmul    qword ptr [r13 + rcx*8]
    fsubp   st(4), st
    fstp    qword ptr [r10 + r15]
    fxch    st(2)
    fstp    qword ptr [r11 + r15]

    mov     r15d, ecx
    shl     r15d, 4
    fld     qword ptr [r12 + r15]
    fmul    st, st(1)
    fld     qword ptr [r13 + r15]
    fmul    st, st(3)
    faddp   st(1), st
    fxch    st(2)
    fmul    qword ptr [r12 + r15]
    fxch    st(1)
    fmul    qword ptr [r13 + r15]
    fsubp   st(1), st
    fxch    st(1)
    fstp    qword ptr [r10 + rbx]
    ret

    .size lpec_pre01590_x87, .-lpec_pre01590_x87
    .size lpec_pre01590_x87_n, .-lpec_pre01590_x87_n
    .size lpec_pre01590_x87_iter, .-lpec_pre01590_x87_iter

    .section .rodata
    .align 8
.Lrt3h: .double 0.866025403784438646
.Lhalf: .double 0.5
