; Print.s
; Student names: change this to your names or look very silly
; Last modification date: change this to the last modification date or look very silly
; Runs on TM4C123
; EE319K lab 7 device driver for any LCD
;
; As part of Lab 7, students need to implement these LCD_OutDec and LCD_OutFix
; This driver assumes two low-level LCD functions
; SSD1306_OutChar   outputs a single 8-bit ASCII character
; SSD1306_OutString outputs a null-terminated string 

    IMPORT   SSD1306_OutChar
    IMPORT   SSD1306_OutString
    EXPORT   LCD_OutDec
    EXPORT   LCD_OutFix
    PRESERVE8
    AREA    |.text|, CODE, READONLY, ALIGN=2
    THUMB



;-----------------------LCD_OutDec-----------------------
; Output a 32-bit number in unsigned decimal format
; Input: R0 (call by value) 32-bit unsigned number
; Output: none
; Invariables: This function must not permanently modify registers R4 to R11
LCD_OutDec

N EQU 4  ; # to get unit place for
CNT EQU 0 ; count of numbers
FP RN 11 ; frame pointer
N1 RN 1
N2 RN 2

;Init 1
	PUSH{R11, R0} ; store R11 and N on stack  
	SUB SP, #4 ; subtract SP by 4 to make room for CNT 
	MOV FP, SP ; FP = SP
;Init 2
	PUSH{LR, R4}
	MOV R0, #0
	STR R0, [FP, #CNT] ; CNT=0
	MOV R4, #10 ; for division
read
;1
	LDR R0, [FP, #CNT]
	ADD R0, R0, #1 
	STR R0, [FP, #CNT]
;2
	LDR R0, [FP, #N]
	MOV N1, R0 ; loading current value of N to reg1
	MOV N2, N1 ; copy into reg2
	UDIV N1, N1, R4 ; N1/10
	STR N1, [FP, #N] ; store new value of N 
	MUL N1, N1, R4 ; N1*10 
	SUB N2, N2, N1 ; diff (N2-N1)
	PUSH {N2, R6} ; push diff on stack
;3
	LDR R0, [FP, #N] ; load new N
	CMP R0, #0
	BNE read
	
write
;4
	POP {N2, R6}
	MOV R0, N2
	ADD R0, R0, #0x30 ; char -> ASCII
	BL SSD1306_OutChar
;5
	LDR R3, [FP, #CNT]
	SUB R3, R3, #1 ; decrement CNT 
	STR R3, [FP, #CNT]
;6
	CMP R3, #0
	BHI write ; if CNT != 0, then loop
;7
	POP{LR, R4}
	ADD SP, SP, #8 ; deallocate 
	POP{R11, R0}
	
    BX  LR
;* * * * * * * * End of LCD_OutDec * * * * * * * *

; -----------------------LCD _OutFix----------------------
; Output characters to LCD display in fixed-point format
; unsigned decimal, resolution 0.01, range 0.00 to 9.99
; Inputs:  R0 is an unsigned 32-bit number
; Outputs: none
; E.g., R0=0,    then output "0.00 "
;       R0=3,    then output "0.03 "
;       R0=89,   then output "0.89 "
;       R0=123,  then output "1.23 "
;       R0=999,  then output "9.99 "
;       R0>999,  then output "*.** "
; Invariables: This function must not permanently modify registers R4 to R11
LCD_OutFix
;Init 1
	PUSH{R11, R0} ; store R11 and N on stack  
	SUB SP, #4 ; subtract SP by 4 to make room for CNT 
	MOV FP, SP ; FP = SP
;Init 2
	PUSH{LR, R4}
	MOV R0, #0
	STR R0, [FP, #CNT] ; CNT=0
	MOV R4, #10 ; for division
;1
	LDR R0, [FP, #N]
	CMP R0, #1000
	BLO InRange 
;2
	MOV R0, #0x2A ; *  
	BL SSD1306_OutChar
	MOV R0, #0x2E ; .
	BL SSD1306_OutChar
	MOV R0, #0x2A ; *
	BL SSD1306_OutChar	
	MOV R0, #0x2A ; *
	BL SSD1306_OutChar	; *.**
	B exit 
;6
InRange
	LDR R0, [FP, #CNT]
	ADD R0, R0, #1 
	STR R0, [FP, #CNT]
;7
	LDR R0, [FP, #N]
	MOV N1, R0 ; loading current value of N to reg1
	MOV N2, N1 ; copy into reg2
	UDIV N1, N1, R4 ; N1/10
	STR N1, [FP, #N] ; store new value of N 
	MUL N1, N1, R4 ; N1*10 
	SUB N2, N2, N1 ; diff (N2-N1)
	PUSH {N2, R6} ; push diff on stack	
;8
	LDR R0, [FP, #CNT]
	CMP R0, #3
	BLO InRange ; if less than, not 2 digits to right of dec yet
;9
	POP {N2, R6}
	MOV R0, N2
	ADD R0, R0, #0x30 ; char -> ASCII
	BL SSD1306_OutChar     ; output digit 
	
	MOV R0, #0x2E ; *  
	BL SSD1306_OutChar   ; output .

	POP {N2, R6}
	MOV R0, N2
	ADD R0, R0, #0x30 ; char -> ASCII
	BL SSD1306_OutChar     ; output digit 
	
	POP {N2, R6}
	MOV R0, N2
	ADD R0, R0, #0x30 ; char -> ASCII
	BL SSD1306_OutChar     ; output digit 
	
exit
	POP {LR, R4}
	ADD SP, SP, #8 ; deallocate 
	POP {R11, R0}
  
     BX   LR
 
     ALIGN
;* * * * * * * * End of LCD_OutFix * * * * * * * *

     ALIGN          ; make sure the end of this section is aligned
     END            ; end of file
