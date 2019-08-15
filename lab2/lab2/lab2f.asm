        .data
        alphabet:	.ascii "abcdefghijklmnopqrstuvwxyz"
        msg1:		.asciiz "The letters are: "
        .text
        .globl main
        main:
        li $v0, 5		#syscall for read_int
        syscall
        add $s1, $v0, $0
        li $v0, 4		#syscall for print_str
        la $a0, msg1
        syscall
        la $t4, alphabet	#load address of alphabet to $t4
        add $t6, $t6, $0	#state the interation condition 
        add $a0, $a0, $0	#set the initial value of $a0
        iteration:
        addi $t6, $t6, 1
        lb $a0, 0($t4)		#load the $t4 th value to $a0
        addi $t4, $t4, 1
        li $v0, 11
        syscall
        bne $t6, $s1, iteration
        
	li   $v0, 10            # system call for exit
	syscall                 # we are out of here.
