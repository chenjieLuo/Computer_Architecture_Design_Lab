.data
        msg1: .asciiz "Enter a integer\n"
        msg2: .asciiz "The factorial of the number is \n"
        .text
        .globl main
        .globl my_mul
        main:
        jal print	#use function to print 
        jal read	#use function to read integer
        jal my_mul	#use function to multilpy 
        jal result	#use function to print out factorial result
       
        addi $v0, $0, 10
        syscall             #syscall for exit
        
         
  print:la $a0, msg1        #load address of msg1 into $a0
        li $v0, 4
        syscall             #print msg1
        jr $ra
        
   read:li $v0, 5           #read int
        syscall
        jr $ra
                 
        my_mul:
        add $t0, $v0, $0    #put in $t0
        add $a0, $t0, $0    #put first number in $a0
        add $fp, $sp, $0    #set fp to top of stack prior   
        addi $sp, $sp, -8   #make room for $s0
        sw $s0, 4($sp)      #push $s0
        sw $s1, 0($sp)
        add $s0, $a0, $0    #set $s0 equal to $a0
        add $s1, $a0, $0
        addi $s1, $s1, -1
        add $v0, $0, $0     #set $v0 to 0
        
mult_loop: addi $s0, $s0, -1 #this function operates multiplication
        beq $s0, $0, jump
        add $v0, $v0, $a0
        j mult_loop
        
  jump: addi $s1, $s1, -1   #take factorial so $s1 = $s1 - 1 and multiply again
        add $s0, $s1, 0
        add $a0, $v0, 0
        beq $s1, $0, mult_eol
        j mult_loop
        
        mult_eol:
        lw $s0, 0($sp)     #pop $s0
        addi $sp, $sp, 4   #pop one word of the stack
        jr $ra
        
        result:       
        add, $t0, $v0, $0   #put in $t0
        addi $sp, $sp, -4   #make room for $v0
        sw $v0, 0($sp)
        la $a0, msg2	    #print out the second string
        add $v0, $a0, 0
        li $v0, 4           
        syscall
        add $a0, $t0, 0
        li $v0, 1	    #print out the integer
        syscall
        lw $v0, 0($sp)
        addi $sp, $sp, 4
        jr $ra
        
        
        
