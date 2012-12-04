#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/eeprom.h>
#include <math.h>

;;; dsp_t dspmul(dsp_t x, dsp_t y)
.global dspmul
.func dspmul
dspmul:
        ;; Compute Z=X*Y
        ;; X:           25 . 24 23 22
        ;; Y:           21 . 20 19 18
        ;; Z:         5  4 . 31 30 29 28 27 26
        ;; ret:         25 . 24 23 22

        push r1
        push r2
        push r3
        push r4
        push r5
        push r6
        push r28
        push r29
        clr r2                  ; use for zero

        clr r3                  
        tst r25
        brpl _x_pos
        com r25
        com r24
        com r23
        com r22
        adc r22, r2
        adc r23, r2
        adc r24, r2
        adc r25, r2
        com r3
_x_pos:
        tst r21
        brpl _y_pos
        com r21
        com r20
        com r19
        com r18
        adc r18, r2
        adc r19, r2
        adc r20, r2
        adc r21, r2
        com r3
_y_pos:  

        mul r22, r18
        movw r26, r0
        mul r23, r19
        movw r28, r0
        mul r24, r20
        movw r30, r0
        mul r25, r21
        movw r4, r0

        ;; Do result 5 4 31 30 29 28 27
        clr r6
        mul r22, r19
        add r27, r0
        adc r28, r1
        adc r6, r2
        mul r23, r18
        add r27, r0
        adc r28, r1
        adc r6, r2
        
        add r29, r6
        adc r30, r2
        adc r31, r2
        adc r4, r2
        adc r5, r2

        clr r6
        mul r22, r20
        add r28, r0
        adc r29, r1
        adc r6, r2
        mul r24, r18
        add r28, r0
        adc r29, r1
        adc r6, r2
        
        add r30, r6
        adc r31, r2
        adc r4, r2
        adc r5, r2
        
        clr r6
        mul r22, r21
        add r29, r0
        adc r30, r1
        adc r6, r2
        mul r23, r20
        add r29, r0
        adc r30, r1
        adc r6, r2
        mul r24, r19
        add r29, r0
        adc r30, r1
        adc r6, r2
        mul r25, r18
        add r29, r0
        adc r30, r1
        adc r6, r2
        
        add r31, r6
        adc r4, r2
        adc r5, r2

        
        mul r23, r21
        add r30, r0
        adc r31, r1
        adc r4, r2
        adc r5, r2
        mul r25, r19
        add r30, r0
        adc r31, r1
        adc r4, r2
        adc r5, r2

        mul r24, r21
        add r31, r0
        adc r4, r1
        adc r5, r2
        mul r25, r20
        add r31, r0
        adc r4, r1
        adc r5, r2

        ldi r18, 0x80
        add r28, r18
        adc r29, r2
        adc r30, r2
        adc r31, r2
        adc r4, r2
        adc r5, r2

        tst r5
        brne _overflow
        tst r4
        brmi _overflow
        rjmp _no_overflow
_overflow:
        ldi r31, 0x7f
        mov r4, r31
        ldi r31, 0xff
        ldi r30, 0xff
        ldi r29, 0xff
        
_no_overflow:   
        tst r3
        brpl _z_pos

        com r4
        com r31
        com r30
        com r29
        adc r29, r2
        adc r30, r2
        adc r31, r2
        adc r4, r2

_z_pos:         
        mov r25, r4
        mov r24, r31
        mov r23, r30
        mov r22, r29
                
        pop r29
        pop r28
        pop r6
        pop r5
        pop r4
        pop r3
        pop r2
        pop r1
        ret

.endfunc        

                                  
;;; int32_t addfixed_dsp(int32_t x, int32_t y)
.global dspadd
.func dspadd
dspadd:
        ;; Compute Z=X+Y
        ;; X:     25 24 . 23 22
        ;; Y:     21 20 . 19 18
        ;; Z:     25 24 . 23 22

        clr r26
        add r22, r18
        adc r23, r19
        adc r24, r20
        adc r25, r21
        adc r26, r1             ; hope it's zero

        ret
.endfunc

;;; int32_t subfixed_dsp(int32_t x, int32_t y)
.global dspsub
.func dspsub
dspsub:
        ;; Compute Z=X-Y
        ;; X:     25 24 . 23 22
        ;; Y:     21 20 . 19 18
        ;; Z:     25 24 . 23 22

        sub r22, r18
        sbc r23, r19
        sbc r24, r20
        sbc r25, r21
        ret
.endfunc
