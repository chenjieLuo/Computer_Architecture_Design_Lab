.text               # text section
    .globl main         # call main by MARS
    main:
    addi $t1, $0, 10	    # load immediate value (10) into $t1
    addi $t2, $0, 11	    # load immediate value (11) into $t2
    add $t3, $t1, $t1   # add two $t1 into $t3
    add $t3, $t3, $t1   # add $t1 into $t3
    add $t3, $t3, $t1   # add  $t1 into $t3
    add $t3, $t3, $t2   # add  $t2 into $t3
    add $t3, $t3, $t2   # add  $t2 into $t3
    li $v0 10
    syscall           # exit system call
