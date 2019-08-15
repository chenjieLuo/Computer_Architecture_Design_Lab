.data
my_array:        		.space 40       	#set the content of my_array as 10
.text
    .globl main		
    main:
    
    addi $t2, $0, 3				#load the initial value 3 into register $t2
    addi $t3, $t3, 0				#set $t3's value to zero
    addi $t4, $0, 10				# set the times of iteration
    la $s0, my_array				#load the address of word into $s0
    jump: beq $t3, $t4, go			
    	sw $t2, 0($s0)
    	addi $a0, $t2, 0
    	li $v0, 1		# print out the integer
        syscall
    	addi $t2, $t2, 1	#$t2 stores the value of j, so j++
    	addi $s0, $s0, 4	#move to the next word
    	addi $t3, $t3, 1	#counter adds 1
    	j jump
    go:
    	
        li $v0 10		#end the program
        syscall   