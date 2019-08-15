        .data
        msg1:	.word 0:24
        .text
        .globl main
        main:
        li $v0, 8		# syscall for read_str
        la $a0, msg1		# load address of msg1 to store string
        li $a1, 100		# msg1 is 100 bytes
        syscall
        jump: 
        lb $t0, 0($a0)		# load the character into $t0
        li $t1, 'a'		# get value of 'a'
        addi $a0, $a0, 1
        blt $t0, $t1, iteration # do nothing if letter is less than 'a'
        li $t1, 'z'		# get value of 'z'
        bgt $t0, $t1, iteration # do nothing if letter is greater than 'z'
        addi $t6, $t6, 1 
        
        iteration:
        beq $t0, $t7, skip	# set the condition to see if we end the iteration
        j jump			# jump back to do the iteration again
        skip:
        add $a0, $0, $t6	# load the content of $t6 to $a0
        li $v0, 1		# print out the integer
        syscall
	li   $v0, 10            # system call for exit
	syscall                 # we are out of here.
