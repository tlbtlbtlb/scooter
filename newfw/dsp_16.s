#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/eeprom.h>
#include <math.h>

.global mulfixed_4dot12
.func mulfixed_4dot12
mulfixed_4dot12:

        muls r24, r22
        movw r18, r0
        clr r20
        clr r21
        muls r25, r22
        add r19, r0
        adc r20, r1
        muls r24, r23
        add r19, r0
        adc r20, r1
        muls r25, r23        
        add r20, r0
        adc r21, r1
	subi r18,lo8(-(2048))
	sbci r19,hi8(-(2048))
        sbci r20,0
        sbci r21,0

        ;; move to r24 (return register) and shift right 8
        mov r24,r19
        mov r25,r20
        mov r26,r21

        ;; shift right 4
        asr r26
        ror r25
        ror r24
        asr r26
        ror r25
        ror r24
        asr r26
        ror r25
        ror r24
        asr r26
        ror r25
        ror r24

        clr r1

_saturate_4dot12:
        ;; r26 should be 0 or ff, depending on the top bit of r25
        tst r26
        brmi _negative16
_positive16:
        brne _ret_maxpos16
        tst r25
        brmi _ret_maxpos16
        ret
        
_negative16:
        neg r26
        brne _ret_maxneg16
        tst r25
        brpl _ret_maxneg16
        ret

_ret_maxpos16:    
        ldi r25,0x7f
        ldi r24,0xff
        ret
_ret_maxneg16:    
        ldi r25,0x80
        ldi r24,0x01
        ret
.endfunc
        

.global addfixed_4dot12
.func addfixed_4dot12
addfixed_4dot12:
        clr r1
        clr r26
        add r24, r22
        adc r25, r23
        adc r26, r1

        rjmp _saturate_4dot12
.endfunc        

.global subfixed_4dot12
.func subfixed_4dot12
subfixed_4dot12:
        clr r1
        clr r26
        sub r24, r22
        sbc r25, r23
        sbc r26, r1

        rjmp _saturate_4dot12
.endfunc

