ldi r26, 0x50 
ldi r27, 0x09 
ldi r20, 0x40 
ldi r21, 0x41 
ldi r22, 0x42 
st X+, r20 
st X+, r21
st X+, r22
st X, r20
ld r19, -X
ld r18, -X
ld r17, -X
ld r16, X

sts 0x0aa5, r20
lds r20, 0x0aa5
ldi r24, 7
ldi r25, 8
mov r25, r24
nop
ldi r29, 15
nop
inc r24
inc r29
nop
add r29, r24

ldi r16, 0
ldi r17, 1
ldi r18, 2
ldi r19, 3
ldi r20, 4
subi r20, 2
sub r20, r19
