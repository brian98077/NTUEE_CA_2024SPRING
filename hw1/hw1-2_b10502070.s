.globl __start

.rodata
    msg0: .string "This is HW1-2: \n"
    msg1: .string "Enter offset: "
    msg2: .string "Plaintext:  "
    msg3: .string "Ciphertext: "
.text

################################################################################
  # print_char function
  # Usage: 
  #     1. Store the beginning address in x20
  #     2. Use "j print_char"
  #     The function will print the string stored from x20 
  #     When finish, the whole program with return value 0

print_char:
    addi a0, x0, 4
    la a1, msg3
    ecall

    add a1,x0,x20
    ecall

  # Ends the program with status code 0
    addi a0,x0,10
    ecall
    
################################################################################

__start:
  # Prints msg
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
    add a6, a0, x0
    
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    addi a0,x0,8
    li a1, 0x10150
    addi a2,x0,2047
    ecall
  # Load address of the input string into a0
    add a0,x0,a1


################################################################################ 
  # Write your main function here. 
  # a0 stores the begining Plaintext
  # x16 stores the offset
  # Do store beginning address 66048 (=0x10200) into x20 
  # ex. j print_char
  
################################################################################
lui x20, 16         # x20 = 66048, use lui to load constant > 12bits
addi x20, x20, 512
addi x13, x0, 10
add x12, x0, a0     # position of input
add x6, x0, x20     # position of output
addi x14, x0, 123
addi x15, x0, 97

add x15, x15, x0
addi x18, x0, 48    # count of space (number 0 start from ASCII 48)
addi x17, x0, 32    # ASCII of space
j loop

loop:
lb x7, 0(x12)
beq x7, x13, exit   # if input text is \n, exit
beq x7, x17, space  # if input text is space
add x7, x7, a6      # offset
bge x7, x14, upper  # exceed z
blt x7, x15, lower  # less than a
sb x7, 0(x6)
j moveindex

moveindex:
addi x12, x12, 1
addi x6, x6, 1
j loop


upper:
addi x7, x7, -26
sb x7, 0(x6)
j moveindex

lower:
addi x7, x7, 26
sb x7, 0(x6)
j moveindex

space:
add x7, x0, x18
sb x7, 0(x6)
addi x18, x18, 1
j moveindex

exit:
j print_char