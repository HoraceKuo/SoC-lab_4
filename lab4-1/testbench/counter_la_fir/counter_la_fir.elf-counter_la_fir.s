	.file	"counter_la_fir.c"
	.option nopic
	.attribute arch, "rv32i2p0"
	.attribute unaligned_access, 0
	.attribute stack_align, 16
	.text
.Ltext0:
	.cfi_sections	.debug_frame
	.file 0 "/home/ubuntu/Horace/caravel-soc_fpga-lab-main/lab-exmem_fir/testbench/counter_la_fir" "counter_la_fir.c"
	.align	2
	.type	flush_cpu_icache, @function
flush_cpu_icache:
.LFB21:
	.file 1 "../../firmware/system.h"
	.loc 1 15 1
	.cfi_startproc
	addi	sp,sp,-16
	.cfi_def_cfa_offset 16
	sw	s0,12(sp)
	.cfi_offset 8, -4
	addi	s0,sp,16
	.cfi_def_cfa 8, 0
	.loc 1 26 1
	nop
	lw	s0,12(sp)
	.cfi_restore 8
	.cfi_def_cfa 2, 16
	addi	sp,sp,16
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE21:
	.size	flush_cpu_icache, .-flush_cpu_icache
	.align	2
	.type	flush_cpu_dcache, @function
flush_cpu_dcache:
.LFB22:
	.loc 1 29 1
	.cfi_startproc
	addi	sp,sp,-16
	.cfi_def_cfa_offset 16
	sw	s0,12(sp)
	.cfi_offset 8, -4
	addi	s0,sp,16
	.cfi_def_cfa 8, 0
	.loc 1 33 1
	nop
	lw	s0,12(sp)
	.cfi_restore 8
	.cfi_def_cfa 2, 16
	addi	sp,sp,16
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE22:
	.size	flush_cpu_dcache, .-flush_cpu_dcache
	.align	2
	.globl	main
	.type	main, @function
main:
.LFB316:
	.file 2 "counter_la_fir.c"
	.loc 2 35 1
	.cfi_startproc
	addi	sp,sp,-32
	.cfi_def_cfa_offset 32
	sw	ra,28(sp)
	sw	s0,24(sp)
	.cfi_offset 1, -4
	.cfi_offset 8, -8
	addi	s0,sp,32
	.cfi_def_cfa 8, 0
	.loc 2 64 10
	li	a5,637534208
	addi	a5,a5,160
	.loc 2 64 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 65 10
	li	a5,637534208
	addi	a5,a5,156
	.loc 2 65 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 66 10
	li	a5,637534208
	addi	a5,a5,152
	.loc 2 66 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 67 10
	li	a5,637534208
	addi	a5,a5,148
	.loc 2 67 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 68 10
	li	a5,637534208
	addi	a5,a5,144
	.loc 2 68 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 69 10
	li	a5,637534208
	addi	a5,a5,140
	.loc 2 69 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 70 10
	li	a5,637534208
	addi	a5,a5,136
	.loc 2 70 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 71 10
	li	a5,637534208
	addi	a5,a5,132
	.loc 2 71 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 72 10
	li	a5,637534208
	addi	a5,a5,128
	.loc 2 72 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 73 10
	li	a5,637534208
	addi	a5,a5,124
	.loc 2 73 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 74 10
	li	a5,637534208
	addi	a5,a5,120
	.loc 2 74 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 75 10
	li	a5,637534208
	addi	a5,a5,116
	.loc 2 75 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 76 10
	li	a5,637534208
	addi	a5,a5,112
	.loc 2 76 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 77 10
	li	a5,637534208
	addi	a5,a5,108
	.loc 2 77 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 78 10
	li	a5,637534208
	addi	a5,a5,104
	.loc 2 78 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 79 10
	li	a5,637534208
	addi	a5,a5,100
	.loc 2 79 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 81 10
	li	a5,637534208
	addi	a5,a5,96
	.loc 2 81 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 82 10
	li	a5,637534208
	addi	a5,a5,92
	.loc 2 82 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 83 10
	li	a5,637534208
	addi	a5,a5,88
	.loc 2 83 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 84 10
	li	a5,637534208
	addi	a5,a5,84
	.loc 2 84 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 85 10
	li	a5,637534208
	addi	a5,a5,80
	.loc 2 85 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 86 10
	li	a5,637534208
	addi	a5,a5,76
	.loc 2 86 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 87 10
	li	a5,637534208
	addi	a5,a5,72
	.loc 2 87 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 88 10
	li	a5,637534208
	addi	a5,a5,68
	.loc 2 88 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 89 10
	li	a5,637534208
	addi	a5,a5,64
	.loc 2 89 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 90 10
	li	a5,637534208
	addi	a5,a5,56
	.loc 2 90 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 91 10
	li	a5,637534208
	addi	a5,a5,52
	.loc 2 91 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 92 10
	li	a5,637534208
	addi	a5,a5,48
	.loc 2 92 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 93 10
	li	a5,637534208
	addi	a5,a5,44
	.loc 2 93 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 94 10
	li	a5,637534208
	addi	a5,a5,40
	.loc 2 94 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 95 10
	li	a5,637534208
	addi	a5,a5,36
	.loc 2 95 43
	li	a4,8192
	addi	a4,a4,-2040
	sw	a4,0(a5)
	.loc 2 97 10
	li	a5,637534208
	addi	a5,a5,60
	.loc 2 97 43
	li	a4,8192
	addi	a4,a4,-2039
	sw	a4,0(a5)
	.loc 2 101 3
	li	a5,-268410880
	.loc 2 101 50
	li	a4,1
	sw	a4,0(a5)
	.loc 2 104 3
	li	a5,637534208
	.loc 2 104 36
	li	a4,1
	sw	a4,0(a5)
	.loc 2 105 8
	nop
.L4:
	.loc 2 105 10 discriminator 1
	li	a5,637534208
	lw	a4,0(a5)
	.loc 2 105 43 discriminator 1
	li	a5,1
	beq	a4,a5,.L4
	.loc 2 109 60
	li	a5,-268423168
	addi	a4,a5,12
	.loc 2 109 114
	li	a5,0
	sw	a5,0(a4)
	.loc 2 109 3
	li	a4,-268423168
	addi	a4,a4,28
	.loc 2 109 57
	sw	a5,0(a4)
	.loc 2 110 59
	li	a5,-268423168
	addi	a4,a5,8
	.loc 2 110 112
	li	a5,-1
	sw	a5,0(a4)
	.loc 2 110 3
	li	a4,-268423168
	addi	a4,a4,24
	.loc 2 110 56
	sw	a5,0(a4)
	.loc 2 111 59
	li	a5,-268423168
	addi	a4,a5,4
	.loc 2 111 112
	li	a5,0
	sw	a5,0(a4)
	.loc 2 111 3
	li	a4,-268423168
	addi	a4,a4,20
	.loc 2 111 56
	sw	a5,0(a4)
	.loc 2 112 53
	li	a4,-268423168
	.loc 2 112 100
	li	a5,0
	sw	a5,0(a4)
	.loc 2 112 3
	li	a4,-268423168
	addi	a4,a4,16
	.loc 2 112 50
	sw	a5,0(a4)
	.loc 2 115 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 115 36
	li	a4,-1421869056
	sw	a4,0(a5)
	.loc 2 118 3
	li	a5,-268423168
	addi	a5,a5,56
	.loc 2 118 56
	sw	zero,0(a5)
	.loc 2 121 59
	li	a5,-268423168
	addi	a4,a5,8
	.loc 2 121 112
	li	a5,0
	sw	a5,0(a4)
	.loc 2 121 3
	li	a4,-268423168
	addi	a4,a4,24
	.loc 2 121 56
	sw	a5,0(a4)
	.loc 2 131 13
	call	fir
	sw	a0,-20(s0)
	.loc 2 132 38
	lw	a5,-20(s0)
	lw	a5,0(a5)
	.loc 2 132 43
	slli	a4,a5,16
	.loc 2 132 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 132 36
	sw	a4,0(a5)
	.loc 2 133 43
	lw	a5,-20(s0)
	addi	a5,a5,4
	.loc 2 133 38
	lw	a5,0(a5)
	.loc 2 133 47
	slli	a4,a5,16
	.loc 2 133 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 133 36
	sw	a4,0(a5)
	.loc 2 134 43
	lw	a5,-20(s0)
	addi	a5,a5,8
	.loc 2 134 38
	lw	a5,0(a5)
	.loc 2 134 47
	slli	a4,a5,16
	.loc 2 134 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 134 36
	sw	a4,0(a5)
	.loc 2 135 43
	lw	a5,-20(s0)
	addi	a5,a5,12
	.loc 2 135 38
	lw	a5,0(a5)
	.loc 2 135 47
	slli	a4,a5,16
	.loc 2 135 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 135 36
	sw	a4,0(a5)
	.loc 2 136 43
	lw	a5,-20(s0)
	addi	a5,a5,16
	.loc 2 136 38
	lw	a5,0(a5)
	.loc 2 136 47
	slli	a4,a5,16
	.loc 2 136 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 136 36
	sw	a4,0(a5)
	.loc 2 137 43
	lw	a5,-20(s0)
	addi	a5,a5,20
	.loc 2 137 38
	lw	a5,0(a5)
	.loc 2 137 47
	slli	a4,a5,16
	.loc 2 137 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 137 36
	sw	a4,0(a5)
	.loc 2 138 43
	lw	a5,-20(s0)
	addi	a5,a5,24
	.loc 2 138 38
	lw	a5,0(a5)
	.loc 2 138 47
	slli	a4,a5,16
	.loc 2 138 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 138 36
	sw	a4,0(a5)
	.loc 2 139 43
	lw	a5,-20(s0)
	addi	a5,a5,28
	.loc 2 139 38
	lw	a5,0(a5)
	.loc 2 139 47
	slli	a4,a5,16
	.loc 2 139 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 139 36
	sw	a4,0(a5)
	.loc 2 140 43
	lw	a5,-20(s0)
	addi	a5,a5,32
	.loc 2 140 38
	lw	a5,0(a5)
	.loc 2 140 47
	slli	a4,a5,16
	.loc 2 140 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 140 36
	sw	a4,0(a5)
	.loc 2 141 43
	lw	a5,-20(s0)
	addi	a5,a5,36
	.loc 2 141 38
	lw	a5,0(a5)
	.loc 2 141 47
	slli	a4,a5,16
	.loc 2 141 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 141 36
	sw	a4,0(a5)
	.loc 2 142 43
	lw	a5,-20(s0)
	addi	a5,a5,40
	.loc 2 142 38
	lw	a5,0(a5)
	.loc 2 142 48
	slli	a4,a5,16
	.loc 2 142 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 142 36
	sw	a4,0(a5)
	.loc 2 146 3
	li	a5,637534208
	addi	a5,a5,12
	.loc 2 146 36
	li	a4,-1420754944
	sw	a4,0(a5)
	.loc 2 147 1
	nop
	lw	ra,28(sp)
	.cfi_restore 1
	lw	s0,24(sp)
	.cfi_restore 8
	.cfi_def_cfa 2, 32
	addi	sp,sp,32
	.cfi_def_cfa_offset 0
	jr	ra
	.cfi_endproc
.LFE316:
	.size	main, .-main
.Letext0:
	.file 3 "/opt/riscv/lib/gcc/riscv32-unknown-elf/12.1.0/include/stdint-gcc.h"
	.section	.debug_info,"",@progbits
.Ldebug_info0:
	.4byte	0xdd
	.2byte	0x5
	.byte	0x1
	.byte	0x4
	.4byte	.Ldebug_abbrev0
	.byte	0x3
	.4byte	.LASF13
	.byte	0x1d
	.4byte	.LASF0
	.4byte	.LASF1
	.4byte	.Ltext0
	.4byte	.Letext0-.Ltext0
	.4byte	.Ldebug_line0
	.byte	0x1
	.byte	0x1
	.byte	0x6
	.4byte	.LASF2
	.byte	0x1
	.byte	0x2
	.byte	0x5
	.4byte	.LASF3
	.byte	0x1
	.byte	0x4
	.byte	0x5
	.4byte	.LASF4
	.byte	0x1
	.byte	0x8
	.byte	0x5
	.4byte	.LASF5
	.byte	0x1
	.byte	0x1
	.byte	0x8
	.4byte	.LASF6
	.byte	0x1
	.byte	0x2
	.byte	0x7
	.4byte	.LASF7
	.byte	0x4
	.4byte	.LASF14
	.byte	0x3
	.byte	0x34
	.byte	0x1b
	.4byte	0x5c
	.byte	0x1
	.byte	0x4
	.byte	0x7
	.4byte	.LASF8
	.byte	0x1
	.byte	0x8
	.byte	0x7
	.4byte	.LASF9
	.byte	0x5
	.byte	0x4
	.byte	0x5
	.string	"int"
	.byte	0x1
	.byte	0x4
	.byte	0x7
	.4byte	.LASF10
	.byte	0x6
	.string	"fir"
	.byte	0x2
	.byte	0x16
	.byte	0xd
	.4byte	0x8a
	.4byte	0x8a
	.byte	0x7
	.byte	0
	.byte	0x8
	.byte	0x4
	.4byte	0x6a
	.byte	0x9
	.4byte	.LASF15
	.byte	0x2
	.byte	0x22
	.byte	0x6
	.4byte	.LFB316
	.4byte	.LFE316-.LFB316
	.byte	0x1
	.byte	0x9c
	.4byte	0xc0
	.byte	0xa
	.string	"j"
	.byte	0x2
	.byte	0x24
	.byte	0x6
	.4byte	0x6a
	.byte	0xb
	.string	"tmp"
	.byte	0x2
	.byte	0x83
	.byte	0x7
	.4byte	0x8a
	.byte	0x2
	.byte	0x91
	.byte	0x6c
	.byte	0
	.byte	0x2
	.4byte	.LASF11
	.byte	0x1c
	.4byte	.LFB22
	.4byte	.LFE22-.LFB22
	.byte	0x1
	.byte	0x9c
	.byte	0x2
	.4byte	.LASF12
	.byte	0xe
	.4byte	.LFB21
	.4byte	.LFE21-.LFB21
	.byte	0x1
	.byte	0x9c
	.byte	0
	.section	.debug_abbrev,"",@progbits
.Ldebug_abbrev0:
	.byte	0x1
	.byte	0x24
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x3e
	.byte	0xb
	.byte	0x3
	.byte	0xe
	.byte	0
	.byte	0
	.byte	0x2
	.byte	0x2e
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0x21
	.byte	0x1
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0x21
	.byte	0x25
	.byte	0x27
	.byte	0x19
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x40
	.byte	0x18
	.byte	0x7a
	.byte	0x19
	.byte	0
	.byte	0
	.byte	0x3
	.byte	0x11
	.byte	0x1
	.byte	0x25
	.byte	0xe
	.byte	0x13
	.byte	0xb
	.byte	0x3
	.byte	0x1f
	.byte	0x1b
	.byte	0x1f
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x10
	.byte	0x17
	.byte	0
	.byte	0
	.byte	0x4
	.byte	0x16
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x5
	.byte	0x24
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x3e
	.byte	0xb
	.byte	0x3
	.byte	0x8
	.byte	0
	.byte	0
	.byte	0x6
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0x19
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x3c
	.byte	0x19
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x7
	.byte	0x18
	.byte	0
	.byte	0
	.byte	0
	.byte	0x8
	.byte	0xf
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x9
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0x19
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x6
	.byte	0x40
	.byte	0x18
	.byte	0x7c
	.byte	0x19
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0xa
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0xb
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x2
	.byte	0x18
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_aranges,"",@progbits
	.4byte	0x1c
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0
	.2byte	0
	.2byte	0
	.4byte	.Ltext0
	.4byte	.Letext0-.Ltext0
	.4byte	0
	.4byte	0
	.section	.debug_line,"",@progbits
.Ldebug_line0:
	.section	.debug_str,"MS",@progbits,1
.LASF11:
	.string	"flush_cpu_dcache"
.LASF6:
	.string	"unsigned char"
.LASF8:
	.string	"long unsigned int"
.LASF7:
	.string	"short unsigned int"
.LASF13:
	.string	"GNU C17 12.1.0 -mabi=ilp32 -mtune=rocket -misa-spec=2.2 -march=rv32i -g -ffreestanding"
.LASF15:
	.string	"main"
.LASF10:
	.string	"unsigned int"
.LASF9:
	.string	"long long unsigned int"
.LASF12:
	.string	"flush_cpu_icache"
.LASF5:
	.string	"long long int"
.LASF3:
	.string	"short int"
.LASF14:
	.string	"uint32_t"
.LASF4:
	.string	"long int"
.LASF2:
	.string	"signed char"
	.section	.debug_line_str,"MS",@progbits,1
.LASF0:
	.string	"counter_la_fir.c"
.LASF1:
	.string	"/home/ubuntu/Horace/caravel-soc_fpga-lab-main/lab-exmem_fir/testbench/counter_la_fir"
	.ident	"GCC: (g1ea978e3066) 12.1.0"
