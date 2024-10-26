.data
    n: .word 10
    
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
    # You should store the output into x10
    addi sp, sp, -8
    sw x1, 0(sp) # return address
    jal x1, recur
    add x10, t0, x0
    lw x1, 0(sp)
    addi sp, sp, 8
    jalr x0, 0(x1) # return
recur:
    addi sp, sp, -8
    sw x1, 0(sp) # return address
    sw a0, 4(sp) # n
    srli a0, a0, 2 # n/4
    blt x0, a0, El # if n <= 3
    addi sp, sp, 8 # base case, pop stack
    addi t0, x0, 3 # t0 = 3 in base case
    jalr x0, 0(x1) # return
El:                # else
    jal x1, recur  # recursive call. n/4 is stored in a0
    lw x1, 0(sp)   # return address
    lw a0, 4(sp)   # n in a0
    addi a1, x0, 3   # recursion logic
    mul t0, a1, t0
    addi a2, x0, 10
    mul a2, a2, a0
    add t0, t0, a2
    addi t0, t0, 3
    addi sp, sp, 8 # pop stack
    jalr x0, 0(x1) #return 

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall