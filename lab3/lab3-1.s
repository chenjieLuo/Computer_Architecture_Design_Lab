.text
.globl main

main:   li      $a1, 10			#load the value 10 into $a1
        add     $t0, $zero, $zero	#set the value of $t0 to zero
loop:   beq     $a1, $zero, finish	#compare $a1 with zero, if equal, jump to finish
        add     $t0, $t0, $a0		
        sub     $a1, $a1, 1		
        j       loop			#continue iteration
finish: addi $t0, $t0, 100		#end iteration
        add  $v0, $t0, $zero
