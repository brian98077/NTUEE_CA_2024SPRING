.globl __start

.rodata
    msg0: .string "This is HW1-1: T(n) = 3T(n/4) + 10n + 3, T(1)=T(2)=T(3) = 3\n"
    msg1: .string "Enter a number: "
    msg2: .string "The result is: "

.text


__start:
  # Prints msg0
    addi a0, x0, 4
    la a1, msg0
    ecall

  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall

  # Reads an int
    addi a0, x0, 5
    ecall

########################################################################################### 
  # Write your main function here. 
  # Input n is in a0. You should store the result T(n) into t0
  # HW1-1 T(n) = 3T(n/4) + 10n + 3, T(1)=T(2)=T(3) = 3, round down the result of division
  # ex. addi t0, a0, 1

addi x5, x0, 4   # x5 = 4
addi x6, x0, 3   # x6 = 3
addi x7, x0, 10  # x7 = 10
jal x1, recursion# go to recursion
beq x0, x0, result

recursion:
addi x2, x2, -4  # move stack pointer
sw x1, 0(x2)     # store data into stack
bge a0, x5, func1# if a0 >= 4, goto to func1 
lw x1, 0(x2)     # load data
addi t0, x0, 3   # else, return 3
addi x2, x2, 4   # move back stack pointer
jalr x0, 0(x1)

func1:
addi x2, x2, -4
sw a0, 0(x2)
srli a0, a0, 2   # a0 = floor(a0 / 4)
jal x1, recursion# jump to recursion
lw x15, 0(x2)
addi x2, x2, 4
mul x13, x7, x15 # x13 = x15 * 10
lw x1, 0(x2)
addi x2, x2, 4
mul x12, x6, t0  # x12 = 3T(n/4)
add x14, x12, x13# x14 = x12 + x13
addi t0, x14, 3  # t0 = x14 + 3
jalr x0, 0(x1)


###########################################################################################

result:
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall

  # Prints the result in t0
    addi a0, x0, 1
    add a1, x0, t0
    ecall
    
  # Ends the program with status code 0
    addi a0, x0, 10
    ecall