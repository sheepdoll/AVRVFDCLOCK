.include "m88def.inc"
; System Process equates
;----------------------------------------------------------------------
							; 
.equ kBaud31250		= 15	; this sets the MIDI baud rate to 31250

.equ RX_BUFFER_SIZE = 64



; fflag bits
; I/D S 
.equ EmI  = 7		; Entry mode increment
.equ EmS  = 6		; Entry mode Shift
.equ DcD  = 5		; Display off (stops clocks)
.equ DcC  = 4		; Display cursor 
.equ DcB  = 3		; Blink Display cursor (enable timer overflow)
.equ ShRL = 2		; left right shifts
.equ BadClock =1	; clock needs to be set

; ------------------------------------------------------------------

.def zero		= r1	; keep a zero handy
.def const7		= r2	; temporary shift register (cause 88 breaks things)
.def nibbleMask	= r3	; should always be 0x0F
.def oRH		= r4	; mask for port C reads and writes
.def oRT		= r5	; out ring tail
.def iRH		= r6	; in ring head
.def iRT		= r7	; in ring tail
.def ashift		= r8	; extended wave generation
.def bshift		= r9	; extended wave generation

.def TIME0		= r10
.def TIME1		= r11
.def TIME2		= r12
.def TIME3		= r13	;
.def TTEMP		= r14	; used for masking time frames
.def ssreg    	= r15

.def temp1		= R16	; interrupt load temp, invalid inside interrupt
.def MaxBits	= R17	; Bit counter for start pulse reload
.def FFLAGS		= R18	;
.def IOMask		= R19	; used in midi parser
.def idx		= R20	; temporary register
.def c_tmp     	= r21 	; passed arguments
.def ARGL     	= r22 	; passed arguments
.def ARGH     	= r23 	; to OS, usually a pointer
.def ACC      	= r24 	; Old fashioned double wideaccumulator
.def BCC      	= r25 	;  

; TASKSTATE semephors and task no 76543210
;                                 KSSSIITT
.equ keywaiting		= 7	; a keypress occured		
.equ task2Semaphore	= 6 ; to be used by
.equ task1Semaphore	= 5 ; file load and
.equ task0Semaphore	= 4 ; playback
; bits 3 and 2 are task ID when timer was interrupted
; bits 0 and 1 are task ID of current process

;==============================================================================
;               
;                     D A T A   S E G M E N T 
;
;==============================================================================
.equ MSGQ_SIZE = 64

.DSEG                		; Start data segment 
.ORG sram_start            	; Start data segment 


.equ DIGITS = 6  ; prototype HW only can handle 6 digits Normal is 9
;
SegMUX:
; bits
; xxxx 1234 56ab kjhg2 cdlm neg1f -- normal
; xxxx 5643 21de nmlg1 fahj kbg2c -- invert
;
; mux 9x3 (6x3)
; 08 0x xx ,04 0x xx,02 0x xx,01 0x xx,00 8x xx.00 4x xx
	.byte DIGITS*3
CGRAMADR:
	.byte 1
CGRAM:
	.byte 8*2		; allow for custom chars?
AC:
DDRAMADR:
	.byte 1			; address counter for incremental reads and writes
DDRAM:
	.byte 80		; buffer of chars to display
WINDOWL:
	.byte 1			; Left window in the DDRAM of visible symols
WINDOWR:
	.byte 1			; Right window in the DDRAM
CURSOR:
	.byte 1			; location 0 to DIGITS of display cursor
FLAGS:
	.byte 1			; shifton|left/right|cursor on|cursor blink			

P0328:				; calculated start of inRing
	.byte	2
M032A:				; inRing command text
	.byte	RX_BUFFER_SIZE
D0369:				; inRing in index command count
	.byte	1
D036A:				; inRing out calculated start
	.byte	1		
M036B:				; 
	.byte	1
M036C:				; outRing
	.byte	15
D037B:				; outRing in index  reply count
	.byte	1
D037C:				; outRing out index reply sent count
	.byte	1		



.if 0
.ORG RAMEND-(NOOFTASKS * STACKSIZE); - STACKSIZE
STKRESERVE: .BYTE (NOOFTASKS * STACKSIZE); - STACKSIZE
.ORG STKRESERVE
MSGQEND:
.ORG MSGQEND-MSGQ_SIZE
MSGQ:   .BYTE MSGQ_SIZE             	;Buffer for writing LCD
.ORG MSGQ - ((NOOFTASKS * 4)+4)	; OS globals
world_save:	.byte 4
TTSL:   .BYTE NOOFTASKS *2		; 0x60 task timers
SPTS:   .BYTE NOOFTASKS *2		; 0x68 task saved stack pointers
.endif

; interrupt table
.cseg

.org 0
	rjmp Main		; reset device

.org INT0addr	; External Interrupt 0
	reti
;	rjmp USBDown
	
.org INT1addr	; External Interrupt 1
	reti
;	rjmp USBRead



; ***** INTERRUPT VECTORS ************************************************
.org	PCI0addr	; Pin Change Interrupt Request 0
	reti
.org	PCI1addr	; Pin Change Interrupt Request 0
	reti
.org	PCI2addr	; Pin Change Interrupt Request 1
	reti
.org	WDTaddr		; Watchdog Time-out Interrupt
	reti
.org	OC2Aaddr	; Timer/Counter2 Compare Match A
	rjmp BlinkBlink
.org	OC2Baddr	; Timer/Counter2 Compare Match A
	reti
.org	OVF2addr	; Timer/Counter2 Overflow
	rjmp CLOCK
.org	ICP1addr	; Timer/Counter1 Capture Event
	reti
.org	OC1Aaddr	; Timer/Counter1 Compare Match A
	reti ;rjmp STARTSEQ
.org	OC1Baddr	; Timer/Counter1 Compare Match B
	reti ;rjmp SuppressLoadPulse
.org	OVF1addr	; Timer/Counter1 Overflow
	reti
.org	OC0Aaddr	; TimerCounter0 Compare Match A
	reti 
.org	OC0Baddr	; TimerCounter0 Compare Match B
	rjmp EnableLoadPulse
.org	OVF0addr	; Timer/Couner0 Overflow
	rjmp SHIFTOUT
.org	SPIaddr		; SPI Serial Transfer Complete
	reti
.org	URXCaddr	; USART Rx Complete
	rjmp	L13EA
.org	UDREaddr	; USART, Data Register Empty
	rjmp	L1409
.org	UTXCaddr	; USART Tx Complete
	reti
.org	ADCCaddr	; ADC Conversion Complete
	reti
.org	ERDYaddr	; EEPROM Ready
	reti
.org	ACIaddr		; Analog Comparator
	reti ;rjmp CISflip
.org	TWIaddr		; Two-wire Serial Interface
	reti
.org	SPMRaddr	; Store Program Memory Read
	reti


.org INT_VECTORS_SIZE

; Interrupt handlers

BlinkBlink:
	; uses cursor location to either blink segment d
	; or to blink the whole digit

	; tbw

	ret


CLOCK:
	in ssreg,SREG  
	push ZH
	push ZL
	push R0

  

	mov R16,TIME0
	;sec = (cjl_dir_buffer[k+22] & 0x1F) * 2;
 	andi R16,0x1F		; time is stored in M$ Fat format
	inc R16
	cpi R16,30
	brlt L0219
	rjmp L0221
L0219: 
	rjmp L0214
L0221:
	; seconds overflow -- increment minutes
	mov R16,TIME0
	andi R16,0xE0	; minutes cross the byte boundry
	swap R16
	lsr  R16
	mov TTEMP,R16	; need to use a dedicated temp register for copy
	mov R16,TIME1
	push TIME1
	andi R16,0xF8 ; preserve hour bits
	mov TIME1,R16
	pop R16
    ;min = ((cjl_dir_buffer[k+22] & 0xE0)>>5) +  ((cjl_dir_buffer[k+23] & 0x07)<<3);
 	andi R16,0x07
	lsl R16			
	lsl R16
	lsl R16

	or R16,TTEMP	; we now have the minute ready to increment
	inc R16			; bump minutes
	cpi R16,60
	brlt L0230
		
	; update hour
    ;hour = ((cjl_dir_buffer[k+23] & 0xF8)>>3);
	mov R16,TIME1
	lsr R16
	lsr R16
	lsr R16

	inc R16
	cpi R16,24
	brlo L0264
	


	;date = cjl_dir_buffer[k+24] & 0x1F;
 	;mnum = ((cjl_dir_buffer[k+24] & 0xE0)>>5) + ((cjl_dir_buffer[k+25] & 0x01)<<3);
 	;year = ((cjl_dir_buffer[k+25] & 0xFE)>>1) + 1980;


	clr R16

L0264:
	push R16

	lsl R16			
	lsl R16
	lsl R16

	mov TIME1,R16   ; restore hour register
	pop R16

	; debug hour display here if needed

bin2bcd8HOUR:
	clr	TTEMP				;clear result MSD
bBCD8_1HOUR:
	subi	R16,10			;input = input - 10
	brcs	bBCD8_2HOUR		;abort if carry set
	inc	TTEMP				;inc MSD
	rjmp	bBCD8_1HOUR		;loop again

bBCD8_2HOUR:
	subi	R16,-10	;compensate extra subtraction


	lsl  R16		; multiply by 2 to get index into ASCII table
	
	ldi ZH,high(ASCIITAB*2)
	ldi ZL,low(ASCIITAB*2)
	
	add ZL,R16
	adc ZH,zero
	lpm	
	lds R16,SEGMUX+4	; this pattern gets the or
	andi R16,0xC0
	or R0,R16
	sts SegMUX+4,r0 
					 
	adiw Z,1
	lpm
	sts SegMUX+5,r0

	; now do the other digit
	mov R16,TTEMP
	lsl  R16		; multiply by 2 to get index into ASCII table
	
	ldi ZH,high(ASCIITAB*2)
	ldi ZL,low(ASCIITAB*2)
	
	add ZL,R16
	adc ZH,zero
	lpm	
	lds R16,SEGMUX+1  ; need to or at this field
	andi R16,0xC0
	or R0,R16
	sts SegMUX+1,r0 
					
	adiw Z,1
	lpm
	sts SegMUX+2,r0

  


L0247:

	clr R16			; update minutes to zero

L0230:
	; minute were updated R16 contains packed minutes
	push R16	; for unpacking
bin2bcd8MIN:
	clr	TTEMP				;clear result MSD
bBCD8_1MIN:
	subi	R16,10			;input = input - 10
	brcs	bBCD8_2MIN		;abort if carry set
	inc	TTEMP				;inc MSD
	rjmp	bBCD8_1MIN		;loop again

bBCD8_2MIN:
	subi	R16,-10	;compensate extra subtraction



	lsl  R16		; multiply by 2 to get index into ASCII table
	
	ldi ZH,high(ASCIITAB*2)
	ldi ZL,low(ASCIITAB*2)
	
	add ZL,R16
	adc ZH,zero
	lpm	
	sts SegMUX+10,r0 ; we do not need to do the or for the digit as this is
					 ; one that does not have the bits set in static display
	adiw Z,1
	lpm
	sts SegMUX+11,r0

	; now do the other digit
	mov R16,TTEMP
	lsl  R16		; multiply by 2 to get index into ASCII table
	
	ldi ZH,high(ASCIITAB*2)
	ldi ZL,low(ASCIITAB*2)
	
	add ZL,R16
	adc ZH,zero
	lpm	
	sts SegMUX+7,r0 ; we do not need to do the or for the digit as this is
					 ; one that does not have the bits set in static display
	adiw Z,1
	lpm
	sts SegMUX+8,r0

	pop R16			 ; unpack time for save back to registers
	;mov TTEMP,R16
	clr TIME0
	clc
	ror R16
	ror TIME0
	ror R16
	ror TIME0
	ror R16
	ror TIME0
	andi R16,0x07
	or TIME1,R16

	clr R16			; zero seconds

L0214:
	; seconds were updated
	mov TTEMP,R16	; save updated seconds
	mov R16,TIME0
	andi R16,0xE0
	or R16,TTEMP	; update seconds
	mov TIME0,R16

	; test display update
	mov R16,TTEMP
	lsl R16 		; multiply back by two
	; divide by 10 for display

	;***** Subroutine Register Variables

;.def	fbin	=r16		;8-bit binary value
;.def	tBCDL	=r16		;BCD result MSD
;.def	tBCDH	=r17		;BCD result LSD

;***** Code

bin2bcd8SEC:
	clr	TTEMP				;clear result MSD
bBCD8_1SEC:
	subi	R16,10			;input = input - 10
	brcs	bBCD8_2SEC		;abort if carry set
	inc	TTEMP				;inc MSD
	rjmp	bBCD8_1SEC		;loop again

bBCD8_2SEC:
	subi	R16,-10	;compensate extra subtraction



	lsl  R16		; multiply by 2 to get index into ASCII table

	
	ldi ZH,high(ASCIITAB*2)
	ldi ZL,low(ASCIITAB*2)
	
	add ZL,R16
	adc ZH,zero
	lpm	
	sts SegMUX+16,r0 ; we do not need to do the or for the digit as this is
					 ; one that does not have the bits set in static display
	adiw Z,1
	lpm
	sts SegMUX+17,r0

	; now do the other digit
	mov R16,TTEMP
	lsl  R16		; multiply by 2 to get index into ASCII table
	
	ldi ZH,high(ASCIITAB*2)
	ldi ZL,low(ASCIITAB*2)
	
	add ZL,R16
	adc ZH,zero
	lpm	
	sts SegMUX+13,r0 ; we do not need to do the or for the digit as this is
					 ; one that does not have the bits set in static display
	adiw Z,1
	lpm
	sts SegMUX+14,r0

	pop r0
	pop ZL
	pop ZH

	out SREG,ssreg
	reti



SHIFTOUT: ; normal mode

	in ssreg,SREG  ;if we have time
PRESHIFT:
	sbrc ashift,0
	rjmp setBitShift
	cbi PORTB,Portb0
	rjmp cbd
setBitShift:
	sbi PORTB,Portb0

cbd:
	lsr ashift

	dec MaxBits
	breq STARTSEQ

	dec bshift
	brne L0269

	ld ashift,-Y		; load in next group of bits byte
	mov bshift,const7
;	tst YL
;	breq STARTFRAME		; does this happen?  Max bits should expire first
	
	rjmp L0269

STARTFRAME:
	ldi YL,low(SegMUX+DIGITS*3)
	ldi YH,high(SegMUX+DIGITS*3)


STARTSEQ:
	; reset shift register counters

	ldi MaxBits,21		; 20 bits in patten
	mov bshift,const7
	ld ashift,-Y
	tst YH
	breq STARTFRAME		; catch start of frame


SuppressLoadPulse:
	; some shift registers keep counting when the load pulse is active
	
	; disable clock output
	in R16,TCCR0A
	andi R16, ~((1 << COM0B1) | (1 << COM0B0))
	out TCCR0A,R16

	; clear any pending timer flags
	in R16,TIFR0
	ori R16,(1 <<OCF0B) | (1 << OCF0A) | (1 << TOV0) 
	out TIFR0,r16

	; arm for catching skipped clock pulse
	; play fast and loose with the interrupts
	lds R16,TIMSK0
	ori R16,(1 << OCIE0B)
	sts TIMSK0,R16


	; write load pulse manually
	cbi PORTB,Portb0  ; clear last bit of data line

	sbi PORTD,PortD3

L0269:
	; wait for skipped pulse
	out SREG,ssreg
	reti



Ov0BTST:
EnableLoadPulse:
	in ssreg,SREG  ;if we have time
	in R16,TCCR0A
	ori R16, (1 << COM0B1) | (1 << COM0B0)
	out TCCR0A,R16

	; play fast and loose with the interrupts
	lds R16,TIMSK0
	andi R16,~(1 << OCIE0B)
	sts TIMSK0,R16

	; clear load pulse manually
	cbi PORTD,PortD3

	out SREG,ssreg
	reti



L13DC:
	; Init or change the baud rate

	ldi	BCC,00			; HI not used?

	sts	UBRR0H,BCC		; set the Baud divisor
	sts	UBRR0L,ACC
		
;	ldi	ACC,1<<U2X		; Double speed
;	sts	UCSR0A,ACC		
	
	ldi	ACC,1<<UCSZ01 | 1<<UCSZ00 ; 8N1
	sts	UCSR0C,ACC

	;					 set the interrupt handler flags
	ldi	ACC,1<<RXCIE0 | 1<<RXEN0 | 1<<TXEN0
	sts	UCSR0B,ACC		

	sts	D036A,BCC		; in ring in
	sts	D0369,BCC		; in ring out
	sts	D037C,BCC		; out ring in
	sts	D037B,BCC		; out ring out
	ret
; pc=0x13EA(0x27D4)
;

; RX handler, 

L13EA:					; Receive RX handler
	in	ssreg,SREG		; protect status
	WDR					; keep from resetting in active mode
	lds r16,UDR0
	inc	 iRH
	and iRH,nibbleMask	; mask ring to size nibbleMask is a constant
	push XH
	push XL
	mov	 XL,iRH
	ldi	 XH,00
	subi XL,low(0xFFFF-P0328) ;kD7	; 329 is the real start
	sbci XH,high(0xFFFF-P0328);kFC
	st	X,r16		; save input byte into ring
	pop XL
	pop XH
	out	SREG,ssreg		; restore state
	reti

;

L1409:					; TX complete handler
	in	 ssreg,SREG		; save status state
	WDR					; message may be important keep alive
	cp	 r4,r5			; when counters equal
	breq L1422			; ring is empty

	inc	 r5				; cycle the ring
	;and  r5,nibbleMask		; large sysex data dumps can happen

	mov	 XL,r5			; de reference the array index
	ldi	 XH,00
	subi XL,low(0xFFFF-D036A) ; k95 +36B is the real start 
	sbci XH,high(0xFFFF-D036A); kFC
	ld	 r16,X			; get data from ring
	sts UDR0,r16
L1422:
	reti	
	
					; keep the system quiet when no data to echo
;	lds r16,UCSR0B		; slight difference this touches r16
;	cbr	r16,1<<UDRIE0	; which will be voided on return
;	sts	UCSR0B,r16
	
L0322:	
;	reti

; pc=0x142A(0x2854)

_available:
	cp iRH,iRT
	brlo L0639
	;if (head >= tail) return head - tail;
	mov ARGL,iRH
	sub ARGL,iRT
	ret
L0639:
	;return RX_BUFFER_SIZE + head - tail;
	ldi ARGL,RX_BUFFER_SIZE
	add ARGL,iRH
	sub ARGL,iRT
	ret
;
;*************************************
; access input ring

;*************************************
;
_getchar:        
	cp iRH,iRT
	breq  _getchar      		;  L142A Ring is empty

	WDR
	inc	  iRT					; ++	
	and   iRT,nibbleMask
	push ZL						; a method to protect Z during SYSEX messages
	push ZH
	mov	 ZL,iRT
	ldi	 ZH,00
	subi ZL,low(0xFFFF-P0328) 	; kD7	; 329 is the real start
	sbci ZH,high(0xFFFF-P0328)	; kFC
	ld   ARGL,Z					; r16 = inRing[r16]
	pop ZH
	pop ZL
	ret




; pc=0x143C(0x2878)
;
_putchar:				; L143C  transmit byte
	inc	 r4
;	and  r4,nibbleMask			; large data dumps can happen

L1440:
	cp	 r4,r5			; test outRing busy
	breq L1440			; ring empty

	WDR
	mov	 ZL,r4			; deref outRing[r18++]
	ldi	 ZH,00
	subi ZL,low(0xFFFF-D036A)	; k95 +36A 
	sbci ZH,high(0xFFFF-D036A); kFC
	std	 Z+00,ARGL		; save byte into ring

	lds c_tmp,UCSR0B		; slight difference this touches r16
	sbr	c_tmp,1<<UDRIE0	; which will be voided on return
	sts	UCSR0B,c_tmp
	ret

; HD44780 emulator

ClearDisplay: 	; 0b00000001  -- 0x01

	ldi ACC,' '		; write blank chars to DDRAM
	ldi idx,80	

	ldi ZH,high(DDRAM)	; get the symbol from the buffer	
	ldi ZL,low(DDRAM)

L0709:
	st Z+,ACC
	dec idx
	brne L0709
	
	sbr fFlags, (1 << EmI) ; Entry mode increment
		

; fall through to cursor home

;----------------------------------------------------------
CursorHome:		; 0b0000001x  -- 0x02 0x03
	
	clr ACC
	sts AC,ACC			; clear address counter to zero
	sts CURSOR,ACC		; set cursor home
	sts WINDOWR,ACC		; home left visible window
	ldi c_tmp,DIGITS
	add ACC,c_tmp
	sts WINDOWL,ACC

	; simple way to set the bit display tables

	; init digit mux shift register bits
	; there are 6 | 9 character fields in each frame each char feild is
	; 3 | 5 bytes.  
	; 08 0x xx ,04 0x xx,02 0x xx,01 0x xx,00 8x xx.00 4x xx

	ldi ACC,0x40
	sts SegMUX+1,ACC
	ldi ACC,0x80
	sts SegMUX+4,ACC
	ldi ACC,0x01
	sts SegMUX+6,ACC
	ldi ACC,0x02
	sts SegMUX+9,ACC
	ldi ACC,0x04
	sts SegMUX+12,ACC
	ldi ACC,0x08
	sts SegMUX+15,ACC

	ldi idx,DIGITS		; re-write the first window of data
	ldi ARGL,1		
	ldi c_tmp,3

UpdateHome:
	rcall _putSymbol
	add ARGL,c_tmp
	dec idx
	brne UpdateHome


ret

;----------------------------------------------------------
EntryModeSet:	; 0b00000100 -- 0x04 I == 0 decrement cusror S == 0 no shift
				;		0101	0x05 I == 0 decrement cusror S == 1 shift 
				;		0110	0x06 I == 1 increment cusror S == 0 no shift
				;		0111	0x07 I == 1 increment cusror S == 1 shift

	; this function only sets the mode flags,  The actual data is
	; written by the TWI slave recieve data handler

	lsl ARGL
	brcc EntryMDec

EntryMInc:
	sbr fFlags, (1 << EmI) ; Entry mode increment
	rjmp EntryMSft

EntryMDec:
	cbr fFlags, (1 << EmI) ; Entry mode increment

EntryMSft:
	lsl ARGL
	brcc EntryMSOff

EntryMSOn:
	sbr fFlags, (1 << EmS) ; Entry mode shift

EntryMSOff:
	sbr fFlags, (1 << EmS) ; Entry mode shift

	ret


;-------------------------------------------------------------------------
DisplayControl:	; 0b00001000 -- 0x08 Display off Cursor off Blink off							
				;		1001 -- 0x09 Display off Cursor off Blink on
				;		1010 -- 0x0A Display off Cursor on  Blink off
				;		1011 -- 0x0B Display off Cursor on  Blink on
				;		1100 -- 0x0C Display on  Cursor off Blink off
				;		1101 -- 0x0D Display on  Cursor on  Blink on
				;		1110 -- 0x0E Display on  Cursor on  Blink off
				;		1111 -- 0x0F Display on  Cursor on  Blink on
	lsl ARGL
	brcc DisplayOFF
DisplayON:

	sbr fFlags, (1 << DcD)	; We really do not need a flag here as the control
							; manages the timers

	lds ACC,TCCR1B			; start filament pulse generator
	ori ACC, (1<<CS10)		; no prescale
	sts TCCR1B,ACC  		; start timer

	; may want to delay here to let filament warm up

	in ACC,TCCR0B
	ori ACC, (2 << CS00)
	out TCCR0B,ACC

	ldi ACC,1 << TOIE0		; enable timer 0 interrupt
	sts TIMSK0,ACC

	cbi PORTD,PortD4		; Enable shift register (starts display)


	rjmp VisCursor			; continue checking Display control flags

DisplayOFF:
		; Display off (stops clocks)

	cbr fFlags, (1 << DcD)	; (this flag is redundant)

	sbi PORTD,PortD4		; disable shift register (should blank display)

	lds ACC,TIMSK0
	andi ACC, ~((1 << OCIE0A) | (1 << OCIE0B) | (1 << TOIE0))
	sts TIMSK0,ACC

	in ACC,TCCR0B			; stop shift register
	andi ACC, ~(7 << CS00)
	out TCCR0B,ACC

	lds ACC,TIMSK1
	andi ACC, ~((1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1))
	sts TIMSK1,ACC


	lds ACC,TCCR1B			; stop filament pulse generator
	andi ACC, ~(7<<CS10)
	sts TCCR1B,ACC  		


VisCursor:
	lsl ARGL
	brcc CursorOFF

CursorON:
	sbr fFlags, (1 << DcC)
	; there is not much to do here as the cursor is either handled
	; by the interrupt routine or by the blink timer

	rjmp BlinkCntrl

CursorOFF:
	cbr fFlags, (1 << DcC)

BlinkCntrl:
	lsl ARGL
	brcc BlinkOFF

BlinkON:
	sbr fFlags, (1 << DcB)
	; tbw  set up compare match A on timer 2 for a reasonable blink rate
	
	;lds ACC,TIMSK2
	;ori ACC, (1 << OCIE2A)
	;sts TIMSK2,ACC

	ret

BlinkOFF:
	cbr fFlags, (1 << DcB)
	; disable timer 2 compare match

	lds ACC,TIMSK2
	andi ACC, ~(1 << OCIE2A)
	sts TIMSK2,ACC
	ret

;---------------------------------------------------------
CursorShift:	; 0b000100** -- 0x10 Cursor Shift left 
				; 0b000101** -- 0x14 Cursor Shift Right
				; 0b000110** -- 0x18 Display Shift Left
				; 0b000111** -- 0x1C Display Shift Right

	lsl ARGL
	brcs ShiftDisplay	; command is to shif the display

ChangeCursor:			; changing the cursor mostly changes address pointers
	lsl ARGL			; nothing happens until data is actually written
	brcc CursorRT		; into the location

CursorLFT:
	lds ACC,CURSOR		; if the cursor is visible the interupt routines
	dec ACC				; will display it.	
	sbrc ACC,7
	ldi ACC,DIGITS
	sts CURSOR,ACC

	lds ACC,AC			; this is the location in which the next character is
	dec ACC				; written into
	sbrc ACC,7
	clr ACC				; wrap buffer if it goes negative
	sts AC,ACC
	ret

CursorRT:
	lds ACC,CURSOR		; see above, this only updates the visible cursor
	inc ACC
	cpi ACC,DIGITS*3
	brlo L0739
	ldi ACC,80			; wrap cursor
L0739:
	sts CURSOR,ACC

	lds ACC,AC			; deal with updating the address counter
	inc ACC
	cpi ACC,80
	brlo L0758
	ldi ACC,0			; wrap buffer
L0758:
	sts AC,ACC
	ret

ShiftDisplay:
	lsl ARGL
	brcc DISPLAYRT
DISPLAYLFT:

	ldi ZH,high(SegMUX+15)	
	ldi ZL,low(SegMUX+15)	

	ldi idx,DIGITS

FShiftLeft:
	ldd ACC,Z+0			; load in the frame base address
	ldd BCC,Z+1
	andi BCC,0xC0		; mask out the segment data
	lsl BCC				; shift the active segment left
	rol ACC
	sbrc ACC,4			; active line bit is split
	ldi BCC,0x40		; wrap bit
	ldd c_tmp,Z+1		; reload segment data
	andi c_tmp,0x3F		; mask bits
	or BCC,c_tmp		; merge segment with line data
	std Z+1,BCC			; move char with next multiplex
	std Z+0,ACC
	sbiw Z,3			; next character field
	dec idx				; for all char visible
	brne FShiftLeft

	; load next visible char  from ram table

	lds idx,WINDOWL		; use a window pointer, Not sure if this is the same as
	inc idx				; the cursor pointer.  The cursor pointer is only effective
	cpi idx,80			; between 0 and DIGITS in size 
	brlo L0787
	clr idx

L0787:
	sts WINDOWL,idx		; move window pointer
	mov c_tmp,idx
	subi c_tmp,DIGITS
	sbrs c_tmp,7		; handle wrap of window pointer
	rjmp L0794
	ldi ACC,0x80		; new limit less
	add c_tmp,ACC		;  the negative of the old limit
L0794:
	sts WINDOWR,c_tmp	; window has been moved
	ldi ARGL,16			; physical 
	rjmp _putSymbol

DISPLAYRT:
	; this code tbw is similar to the above code but reversed
	; (or we could use flags)
	ldi ARGL,1
	lds idx,WINDOWR

_putSymbol:
	; puts an active symbol to the display at idx at location specified by ARGL
	; where ARGL points to the segment bit address location
	push ARGL			; protect these so this can be called from a loop
	push idx
	push c_tmp

	ldi ZH,high(DDRAM)	; get the symbol from the buffer	
	ldi ZL,low(DDRAM)

	add ZL,idx			; symbol at index
	clr ZH

	ld ACC,Z			; adjust for rom table lookup			
	clr BCC	
	lsl ACC			

	; tbw if char index is less than 8 get the symbol from the CGRAM
	
	ldi ZH,high(ASCIITAB*2)	; the rom buffer	
	ldi ZL,low(ASCIITAB*2)

	add ZL,ACC				; get the segment bits offset
	adc ZH,BCC	

	lpm ACC,Z+				; the bits to write
	lpm BCC,Z

	; tbw use a flag to load in the left or right window location

	ldi ZH,high(SegMUX)		; left window address
	ldi ZL,low(SegMUX)
	
	add ZL,ARGL				; this is the offset in the frame
	adc ZH,zero				; offsets point to the byte that contains the
							; segment data, these typically start at offset 1
							; in 6 digit mode.	

	ld c_tmp,Z				; this is the merged byte in 6 digit mode
	ori c_tmp,0xC0
	and ACC,c_tmp
	st Z,ACC				; update display segment bits
	std Z+1,BCC

	; the display will self update with the next refresh cycle interrupt

	pop c_tmp
	pop idx
	pop ARGL

	ret

;--------------------------------------------------------------------
FunctionSet:	; 0b001000** -- 0x20 Interface 4 bit 1 Line 5x8 font
				; 0b001001** -- 0x24 Interface 4 bit 1 Line 5x10 font
				; 0b001010** -- 0x28 Interface 4 bit 2 Line 5x8 font (* most common)
				; 0b001011** -- 0x2C Interface 4 bit 2 Line 5x10 font
				; 0b001100** -- 0x30 Interface 8 bit 1 Line 5x8 font
				; 0b001101** -- 0x34 Interface 8 bit 1 Line 5x10 font
				; 0b001110** -- 0x38 Interface 8 bit 2 Line 5x8 font (* default)
				; 0b001111** -- 0x3C Interface 8 bit 2 Line 5x10 font
	; continue shifting ARGL to set functions
	lsl ARGL
	brcc PC+2
	nop			; set 8 bit mode flag
	lsl ARGL
	brcc PC+2
	nop			; set number of lines (not used)
	lsl ARGL
	brcc PC+2
	nop			; set font table (might use this for display inversion)
	
	ret

SetCGRAMAddr:	; 0b01aaaaaa -- 0x40 Sets memory address pointer of CGRAM
				; this function is not compatable with a real dot matrix
				; display.  Given that we can code custom chars in our own
				; rom font -- it is left in due to the simplicity of the 
				; implementation

	lsr ARGL	; normalize address
	lsr ARGL
	sts CGRAMADR,ARGL
	ret				

SetDDRAMAddr:	; 0b1aaaaaaa -- 0x80 Sets memory address pointer of DDRAM
	lsr ARGL	; normalize address
	sts DDRAMADR,ARGL
	ret				

;**********************************************************************************
ParseCommand:

	; command byte is passed in ARGL

	; use a carry set jump table
	lsl ARGL
	brcc PC+2
	rjmp SetDDRAMAddr
	lsl ARGL
	brcc PC+2
	rjmp SetCGRAMAddr
	lsl ARGL
	brcc PC+2
	rjmp FunctionSet
	lsl ARGL
	brcc PC+2
	rjmp CursorShift
	lsl ARGL
	brcc PC+2
	rjmp DisplayControl
	lsl ARGL
	brcc PC+2
	rjmp EntryModeSet
	lsl ARGL
	brcc PC+2
	rjmp CursorHome
	lsl ARGL
	brcc PC+2
	rjmp ClearDisplay

	; NOP recieved

ret



;*************************************
; MEP mph MLH Main reset entry point
;*************************************
Main:

; init serial ports for MIDI baud rate
	ldi	r16,low(RAMEND) ; setup stack
	out	SPL,r16
	ldi	r16,high(RAMEND) 
	out	SPH,r16

;   did the dog bite?  turn it off
	CLI
	WDR
	in ARGL,MCUSR
	andi ARGL,(0xFF & (0<<WDRF))
	out MCUSR,ARGL

	; we should run with the dog enabled

	ldi ARGH,(1<<WDE) | (1 << WDP0) ; really long time
	rcall L1113


;#if 0
;	WDR
;	CLI	; bad, but what can we do , on must be atomic
;	lds ARGL,WDTCSR					; start timed sequence
;	ori ARGL,(1<<WDCE) | (1<<WDE)
;	sts WDTCSR,ARGL					; use sts to get 4 cycles
;	ldi ARGL,(0 << WDE)				; chain the dog
;	sts WDTCSR,ARGL
;	SEI ; end atomic sequence
;	ret
;#endif 

	CLI

	clr zero
	clr FFLAGS

	clr r4			; clear the out ring pointers
	clr r5			; 
	clr r6			; clear the in ring pointers
	clr r7


; timers are in dos time format

; time
	
	ldi ACC, ((58 << 5)&0xE0) | 26		;0x39 = 57  0x1A = 26 sec
	mov TIME0,ACC  ; mmmsssss
	ldi ACC, ((22 << 3)&0xF8) | ((58 >> 3)&0x07) ; 0x16 = 22 h
	mov TIME1,ACC  ; hhhhhmmm

; date
	clr TIME2
	clr TIME3



	ldi YL,low(SegMUX+0xFF)
	ldi YH,high(SegMUX+0xFF)
L0413:
	st -Y,zero
	cp YL,zero
	brne L0413

	ldi	r16,kBaud31250	; kBaud 15 for midi clock at 8 Mhz	
	rcall	L13DC		; init uart


	ldi ACC,0x3F		; mask for input and output rings
	mov nibbleMask,ACC


	ldi ACC,8			; reload value for shift register Max6921 has 20 bits		
	mov const7,ACC		



	; Option switches	; there are 64 possible configurations
	out PORTC,nibbleMask		; By chance the mask for the output pull
						; up is 0x3F


	sbi PORTD,PortD4	; Keep enable high
	; Enable timer cascade and Clear shift register
	ldi ACC, 1 << DDD5  | 1 << DDD4 | 1 << DDD3
	out DDRD,ACC       		

	; enable shift register drive lines
	ldi ACC, 1 << DDB1  | 1 << DDB2  | 1 << DDB0
	out DDRB,ACC       		


;****************************** Timers

	; set up timer 2 for real time clock operation

	lds ACC,TIMSK2
	andi ACC, ~((1 << OCIE2B) | (1 << OCIE2A) | (1 << TOIE2))
	sts TIMSK2,ACC

	lds ACC,ASSR
	ori ACC, (1<<AS2)
	sts ASSR,ACC

	clr ACC			; prepare to sync to second
	sts TCNT2,ACC
	
	sts TCCR2A,ACC	; NO PWM compare match or output

	; set up for dos style 2 second file timestamp creation 
	ldi ACC, (6 << CS20) 
	sts TCCR2B,ACC      ; start timer

	; wait for ASSR to clear
L0505:
	lds ACC, ASSR
	andi ACC, (1<<TCN2UB) | (1<<OCR2AUB) | (1<<OCR2BUB) | (1<<TCR2AUB) | (1<<TCR2BUB)
	brne L0505

	ldi ACC,TIMSK2
	ori ACC, (1 << TOIE2)
	sts TIMSK2,ACC		; arm to start chalking time ticks as soon as we are ready


	sbr fFlags,(1<<BadClock)	; clock is dirty

;
;	tbw set up blink cursor timer on OCRA2

;   
	SEI			; allow interrupts for display initialization


;*********** shift register on timer 1

	; MAX6921 does not like clock pulses under the load signal
	; timer 1 is used to create a PWM for the filiment voltage


	; setup timer 1 for 61 KHZ PWM
	

	lds ACC,TIMSK1
	andi ACC, ~((1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1))
	sts TIMSK1,ACC

	; 66 131 262
	
	ldi ACC,High(66)
	sts OCR1AH,ACC
	ldi ACC,low(66)
	sts OCR1AL,ACC  ; PB1 -> VFD filament pulse

	clr ACC
	sts OCR1BH,ACC
	ldi ACC,33
	sts OCR1BL,ACC  ; PB2 (hardware not connected)

	ldi ACC, (1<<COM1A0) ; fast pwm mode 15 
	
	; mode 15 fast PWM

	ori ACC, (3<<WGM10)   ; upper bits of mode 15
	sts TCCR1A,ACC
	ldi ACC, (3<<WGM12)   ; lower bits of mode 15



;************************** set up timer 0 for creating shift register clock 

	lds ACC,TIMSK0
	andi ACC, ~((1 << OCIE0A) | (1 << OCIE0B) | (1 << TOIE0))
	sts TIMSK0,ACC

	
	ldi ACC,0x13
	out OCR0A,ACC  ; PD6 -> header  (hardware not connected)

	ldi ACC,10
	out OCR0B,ACC  ; PD5 -> (T1 -> SCK cascade not used)

	; connect OC0B to shift register 

	; compare output B (pin PD5), PWM fast mode, -- no prescale

	ldi ACC, 1 << COM0B1 | 1 << COM0B0 | 1 << WGM01 | 1 << WGM00
	out TCCR0A,ACC

	ldi ACC, (1 << WGM02) | (2 << CS00)
	out TCCR0B,ACC

;******************************************************************
; 	Enable display defaults
;******************************************************************

	ldi MaxBits,21			; 20 bits in field pattern

	mov bshift,const7
	ldi YL,low(SegMUX+DIGITS*3)
	ldi YH,high(SegMUX+DIGITS*3)
	ld ashift,-Y

							; set some defaults using the emulator
	ldi ARGL,0x30			; normal function set
	rcall ParseCommand
	
	ldi ARGL,0x01			; clear display
	rcall ParseCommand		; this will clear DDRAM and init flags

	ldi ARGL,0x0C0			; start display timers
	rcall ParseCommand


; display something for testing


;	test DDRAM

	ldi XH,high(DDRAM)
	ldi XL,low(DDRAM)

	ldi ACC,'H'					; not very origional, but traditional
	st X+,ACC
	ldi ACC,'E'
	st X+,ACC
	ldi ACC,'L'
	st X+,ACC
	ldi ACC,'L'
	st X+,ACC
	ldi ACC,'O'
	st X+,ACC
	ldi ACC,' '
	st X+,ACC
	ldi ACC,'W'
	st X+,ACC
	ldi ACC,'O'
	st X+,ACC
	ldi ACC,'R'
	st X+,ACC
	ldi ACC,'L'
	st X+,ACC
	ldi ACC,'D'
	st X+,ACC

	ldi idx,DIGITS		; re-write the first window of data
	ldi ARGL,1		
	ldi c_tmp,3

TestWindow:
	rcall _putSymbol
	add ARGL,c_tmp
	dec idx
	brne TestWindow

					 

IdleLoop:
	WDR

	ldi ACC, (2 << SM0) | (1 << SE)
;	out SMCR,ACC
;	sleep
	nop

	; example calls this a dummy write
	; set up for dos style 2 second file timestamp creation 
	ldi ACC, (6 << CS20) 
	sts TCCR2B,ACC      ; start timer

	; wait for ASSR to clear
L0601:
	lds ACC, ASSR
	andi ACC, (1<<TCN2UB) | (1<<OCR2AUB) | (1<<OCR2BUB) | (1<<TCR2AUB) | (1<<TCR2BUB)
	brne L0601
		


	
eop: WDR rjmp eop

L1113:
	WDR
	CLI	; bad, but what can we do , on must be atomic
	lds ARGL,WDTCSR					; start timed sequence
	ori ARGL,(1<<WDCE) | (1<<WDE)
	sts WDTCSR,ARGL					; use sts to get 4 cycles
	mov ARGL,ARGH
	sts WDTCSR,ARGL
	SEI ; end atomic sequence
	ret

SYSRESETRT:
	sbi PORTD,PortD4	; Disable shift register
	cbi DDRD,DDD3		; blank IO

	ldi ARGL,0xFF		; echo reset 
	rcall _putchar

L1266:					; wait for completion
	cp r4,r5
	brne L1266

	lds ACC,UCSR0B		; 	make sure we are not xmitting
	cbr	ACC,1<<UDRIE0	;   if we are tranmitting someone is trying
	sts	UCSR0B,ACC		;   to violate physics	

	ldi ARGH,(1<<WDE)	;   minimum time

	rcall L1113			;   enable the dog


;	rjmp 0 ; should let the dog bite
	; kick the dog
reset: rjmp reset			; let the dog bite  if enabled.





; Normal table
ASCIITAB:
;   0/8	      1/9        2/A      3/B       4/C       5/D       6/E       7/F
; control group
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 ; 0
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 ; 1
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
; display chars
.db 0x00,0x00,0x10,0x80,0x14,0x00,0x15,0xD2,0x25,0xD3,0x08,0x89,0x2A,0x76,0x04,0x00 ; 2
.db 0x20,0x45,0x30,0xC0,0x0F,0x3A,0x05,0x12,0x00,0x08,0x01,0x02,0x00,0x04,0x08,0x08
.db 0x38,0xCD,0x18,0x80,0x31,0x46,0x31,0xC0,0x11,0x83,0x21,0xC3,0x21,0xC7,0x28,0x10 ; 3
.db 0x31,0xC7,0x31,0xC3,0x00,0x05,0x04,0x08,0x08,0x20,0x01,0x42,0x02,0x08,0x31,0x11
.db 0x35,0x45,0x31,0x87,0x35,0xD0,0x20,0x45,0x34,0xD0,0x21,0x47,0x21,0x07,0x21,0xC5 ; 4
.db 0x11,0x87,0x24,0x50,0x10,0xC4,0x08,0x27,0x00,0x45,0x1A,0x85,0x12,0xA5,0x30,0xC5
.db 0x31,0x07,0x30,0xE5,0x31,0x27,0x23,0xC0,0x24,0x10,0x10,0xC5,0x08,0x0D,0x10,0xAD ; 5
.db 0x0A,0x28,0x0A,0x10,0x28,0x48,0x20,0x45,0x02,0x20,0x30,0xC0,0x00,0x38,0x00,0x40
.db 0x04,0x00,0x31,0x87,0x35,0xD0,0x20,0x45,0x34,0xD0,0x21,0x47,0x21,0x07,0x21,0xC5 ; 6
.db 0x11,0x87,0x24,0x50,0x10,0xC4,0x08,0x27,0x00,0x45,0x1A,0x85,0x12,0xA5,0x30,0xC5
.db 0x31,0x07,0x30,0xE5,0x31,0x27,0x23,0xC0,0x24,0x10,0x10,0xC5,0x08,0x0D,0x10,0xAD ; 7
.db 0x0A,0x28,0x0A,0x10,0x28,0x48,0x22,0x4A,0x04,0x10,0x29,0x60,0x13,0x01,0x3F,0xFF
; Inverted table
; high bit control
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 ; 8
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 ; 9
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
; display chars
.db 0x00,0x00,0x00,0x05,0x00,0x14,0x25,0x17,0x25,0xD3,0x08,0x89,0x37,0x68,0x00,0x10 ; A
.db 0x30,0xC0,0x20,0x45,0x0F,0x3A,0x05,0x12,0x08,0x00,0x01,0x02,0x10,0x00,0x08,0x08
.db 0x38,0xCD,0x00,0x0D,0x31,0x46,0x20,0x47,0x01,0x87,0x21,0xC3,0x31,0xC3,0x04,0x48 ; B
.db 0x31,0xC7,0x21,0xC7,0x10,0x80,0x08,0x10,0x02,0x08,0x21,0x02,0x08,0x20,0x04,0xC6
.db 0x30,0xD6,0x11,0xC7,0x24,0x57,0x30,0xC0,0x24,0x55,0x31,0xC2,0x11,0xC2,0x30,0xC3 ; C
.db 0x11,0x87,0x24,0x50,0x30,0x05,0x13,0x88,0x30,0x80,0x10,0xAD,0x12,0xA5,0x30,0xC5
.db 0x11,0xC6,0x32,0xC5,0x13,0xC6,0x20,0x63,0x04,0x50,0x30,0x85,0x18,0x88,0x1A,0x85 ; D
.db 0x0A,0x28,0x04,0x28,0x28,0x48,0x30,0xC0,0x02,0x20,0x20,0x45,0x0E,0x00,0x20,0x00
.db 0x00,0x10,0x11,0xC7,0x24,0x57,0x30,0xC0,0x24,0x55,0x31,0xC2,0x11,0xC2,0x30,0xC3 ; E
.db 0x11,0x87,0x24,0x50,0x30,0x05,0x13,0x88,0x30,0x80,0x10,0xAD,0x12,0xA5,0x30,0xC5
.db 0x11,0xC6,0x32,0xC5,0x13,0xC6,0x20,0x63,0x04,0x50,0x30,0x85,0x18,0x88,0x1A,0x85 ; F
.db 0x0A,0x28,0x04,0x28,0x28,0x48,0x29,0x60,0x04,0x10,0x22,0x4A,0x00,0xA6,0x3F,0xFF








