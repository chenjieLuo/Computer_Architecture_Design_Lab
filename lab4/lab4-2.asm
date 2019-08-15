.data
        msg1: .asciiz "Enter the first number\n"
        msg2: .asciiz "Enter the second number\n"
        msg:  .asciiz "The product is "
        .text
        .globl main
        .globl my_mul

        main:

        la $a0, msg1        #load address of msg1 into $a0
        li $v0, 4
        syscall             #print msg1
        li $v0, 5
        syscall             #read_int
        add, $t0, $v0, $0   #put input in $t0
        addi $t1, $t0, -1
        
        add $a1, $t1, $0    #load input - 1 into $a1
        add $a0, $t0, $0    #put first number in $a0
        add $fp, $sp, $0    #set fp to top of stack prior
        #to function call
        jal my_mul          #do mul, result is in $v0
	
        add $t0, $v0, $0    #save the result in $t0
        add $a0, $t0, $0    #put computation result(ans) in $a0
 	addi $a1, $a1, -1   #now calculate ans to third number
        beq $a1, $0, exit   #if the $a1 equals 0, jump out of the iteration
        j my_mul
       
        exit:la $a0, msg
        li $v0, 4
        syscall             #print msg
        add $a0, $t0, $0    #put computation result in $a0
        li $v0, 1
        syscall             #print result number

        
        addi $v0,$0,10      #end the program
	syscall 
 	
        my_mul:                 #multiply $a0 with $a1
        #does not handle negative $a1!
        #Note: This is an inefficient way to multipy!
        addi $sp, $sp, -4   #make room for $s0 on the stack
        sw $s0, 0($sp)      #push $s0

        add $s0, $a1, $0   #set $s0 equal to $a1
        add $v0, $0, $0    #set $v0 to 0
        mult_loop:
        beq $s0, $0, mult_eol 	#if $s0 is equal to 0, just out of the loop

        add $v0, $v0, $a0	#operate multiplication
        addi $s0, $s0, -1
        j mult_loop

        mult_eol:
        lw $s0, 0($sp)      #pop $s0
        jr $ra 		    #return to jal
