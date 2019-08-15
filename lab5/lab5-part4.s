.text
funca :
nop
nop
nop
li $s0, 11
addi $s2 , $zero ,1
sub $s1 , $s1 , $s1
loop :
sll $t0 , $s2 ,31
slt $t1 , $t0 , $zero
bne $t1 , $zero , skip
add $s1 , $s1 , $s2
skip :
addi $s2 , $s2 ,1
beq $s2 , $s0 , done
beq $0, $0, loop
done :
addi $v0 , $s1 , 0
jr $ra
