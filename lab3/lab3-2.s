.data
UIN:        		.word 35       		#set the content of UIN as 35
.text
    .globl main		
    main:
    la $s0, UIN					#load the address of UIN into $s0
    lw $t1, 0($s0)				#load the content of $t1 into $t2
    add $v1, $t1, $0				#load the content of $t2 into $v1
    addi $t5, $t5,10				#set the times of loop runs as 10
    jump: beq $t2, $t5, go			#conduct the for loop in C code
    addi $v1, $v1, -1				#UIN = UIN - 1
    addi $t2, $t2, 1				#counter plus one 
    j jump
    go: 
        add $a0, $v1, $0
        li $v0, 1		# print out the integer
        syscall
        li $v0 10		#end the program
        syscall   
    
    
    
    
    
    
    
    
    
