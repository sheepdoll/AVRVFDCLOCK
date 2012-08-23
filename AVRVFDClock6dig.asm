.include "m88def.inc"
; System Process equates
;----------------------------------------------------------------------
							; 
.equ kBaud31250		= 15	; this sets the MIDI baud rate to 31250


.equ DIGITS = 6  ; prototype HW only can handle 6 digits Normal is 9


.equ RX_BUFFER_SIZE = 64
.equ TWI_BUFFER_SIZE = 4

.equ	Set_YEAR 	= 2012
.equ	Set_Month 	= 07
.equ	Set_Day		= 23

.equ	Set_Hour	= 16
.equ	Set_Minute	= 45
.equ	Set_Second	= 0

; need to test/fix  (cursor limits

; clock display to main loop (simulation)


; fflag bits
; I/D S 
.equ EmI  = 7		; Entry mode increment
.equ EmS  = 6		; Entry mode Shift
.equ DcD  = 5		; Display off (stops clocks)
.equ DcC  = 4		; Display cursor 
.equ DcB  = 3		; Blink Display cursor (enable timer overflow)
.equ ShRL = 2		; left right shifts
.equ BadClock = 1	; clock needs to be set
.equ TickTock = 0	; clock ticked

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
	.byte 1			; location 0 to DIGITS of display cursor in steps of 3
					; offset by 1
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

; from AVR311 -- these may be C typedef structs
TWI_state:
	.byte	1				; 0x100
TWI_statusReg:				; 0x102
	.equ lastTransOK 	= 0
	.equ RxDataInBuf	= 1
	.equ genAddressCall = 2
	.byte 1
TWI_buf:					; 0x103
	.byte	TWI_BUFFER_SIZE	
TWI_busy:
	.byte	1				; 0x107
TWI_msgSize:								
	.byte	1				; 0x108
TWI_bufPtr:
	.byte	1				; 0x109



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
	rjmp TWI_vect
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
	; this is the main tick tock interface that must run continiously

	; there seems to be a problem when running 2 second ints, the
	; clock gains 1 second every minute

	; try tracking complete seconds in TIME3 (if needed we can keep the
	; date in SRAM or EEPROM 

	in ssreg,SREG  

	sbr fFlags,(1 << TickTock)	; tell the main program we ticked
	; time is stored in M$ Fat format
	; TIME3					 TIME2
	; 15 14 12 11 10 09 08 | 07 06 05 04 03 02 01 00
	; : Year-1980  : :  Month     : :   Day      :
	
	; TIME1					 TIME0
	; 15 14 12 11 10 09 08 | 07 06 05 04 03 02 01 00
	; :  Hour   : :      Minute     : : Seconds/2  :

DOSTIME:
	mov R16,TIME0
	andi R16,0x1F
	inc R16				; Add 2 seconds to cout (interrupt should be called
						; every 2 seconds.)  
						
	cpi R16,30			; Valid range of seconds is 0 to 29
	brlt L0214			; update seconds and exit

	; seconds overflow -- increment minutes
SECOVERFLOW:
	mov TTEMP,TIME0		; de reference the minute
	ldi R16,0xE0
	and TTEMP,R16

	mov R16,TIME1
	andi R16,0x07

	lsl TTEMP
	rol R16
	rol TTEMP
	rol R16
	rol TTEMP
	rol R16				;min = ((cjl_dir_buffer[k+22] & 0xE0)>>5) +  ((cjl_dir_buffer[k+23] & 0x07)<<3);

	inc R16				; tick the minute

	cpi R16,60			; minutes are valid 0 to 59
	brlt L0230			; update minutes and seconds
		
	; update hour
	mov R16,TIME1
	lsr R16
	lsr R16
	lsr R16				; divide hour by 8 for increment

	inc R16				; Tick the hour
	cpi R16,24			; valid hours are 0 to 23
	brlo L0264			; update hours minutes and seconds
	
	; tbw code to handle date (YMD)

	clr R16				; clear the hour 


L0264:
	; update hour
	lsl R16				; multiply hour by 8 for packing			
	lsl R16
	lsl R16

	mov TIME1,R16   	; restore hour register

	clr R16				; update minutes to zero

L0230:
	; update mimutes 
	clr TIME0			; we can clear the whole time value as seconds
						; always return to 0 at the 59th tick
	lsr R16				; R16 contains packed minutes
	ror TIME0			; we can unpack using a simple shift through
	ror R16				; both registers
	ror TIME0
	ror R16
	ror TIME0
	andi R16,0x07		; mask the shifted high bits

	mov TTEMP,r16
	ldi r16,0xF8
	and TIME1,R16
	or TIME1,TTEMP		; time 1 was cleared at minute update check

	clr R16				; zero seconds

L0214:
	; seconds were updated
	mov TTEMP,R16		; save updated seconds
	mov R16,TIME0
	andi R16,0xE0		; mask the high bits of the minute
	or R16,TTEMP		; merge seconds back 
	mov TIME0,R16		; update seconds


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
	
	ldi ACC,1
	sts CURSOR,ACC		; set cursor home
	clr ACC
	sts AC,ACC			; clear address counter to zero
	sts WINDOWL,ACC		; home left visible window
	ldi c_tmp,DIGITS
	add ACC,c_tmp
	sts WINDOWR,ACC

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

	ldi idx,0		; re-write the first window of data
	ldi ARGL,1		
	ldi c_tmp,3

UpdateHome:
	rcall _putSymbol
	add ARGL,c_tmp
	inc idx
	cpi idx,DIGITS
	brlo UpdateHome
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
	subi ACC,3			; cursor shifts by 3.	
	sbrc ACC,7
	ldi ACC,DIGITS*3	; wrap if underflow
	sts CURSOR,ACC

	lds ACC,AC			; this is the location in which the next character is
	dec ACC				; written into
	sbrc ACC,7
	ldi ACC,80				; wrap buffer if it goes negative
	sts AC,ACC
	ret

CursorRT:
	lds ACC,CURSOR		; see above, this only updates the visible cursor
	subi ACC,-3			; invers subtraction is addition
	cpi ACC,DIGITS*3
	brlo L0739
	ldi ACC,1			; wrap cursor
L0739:
	sts CURSOR,ACC

	lds ACC,AC			; deal with updating the address counter
	inc ACC
	cpi ACC,80
	brlo L0758
	clr ACC				; wrap buffer
L0758:
	sts AC,ACC
	ret

ShiftDisplay:
	lsl ARGL
	brcs DISPLAYRT
DISPLAYLFT:

	ldi ZH,high(SegMUX+1)	
	ldi ZL,low(SegMUX+1)	

	ldi idx,DIGITS
	dec idx

FShiftLeft:
	ldd ARGL,Z+0		; load in the frame base address

	andi ARGL,0xC0		; mask out the segment data
	ldd ACC,Z+3
	ldd BCC,Z+4
	andi ACC,0x3F
	or ACC,ARGL

	std Z+0,ACC
	std Z+1,BCC

	adiw Z,3

	dec idx				; for all char visible
	brne FShiftLeft

	lds idx,WINDOWR
	ldi ARGL,16			; physical 
	rcall _putSymbol

	; load next visible char  from ram table

	lds idx,WINDOWL		; use a window pointer, Not sure if this is the same as
	inc idx				; the cursor pointer.  The cursor pointer is only effective
	cpi idx,80			; between 0 and DIGITS in size 
	brlo L0787
	clr idx

L0787:
	sts WINDOWL,idx		; move window pointer
	
	ldi c_tmp,DIGITS
	add idx,c_tmp

	cpi idx,80
	brlo L0794
	subi idx,80			; gets a bit tricky here as our window wraps
L0794:
	sts WINDOWR,idx 	; window has been moved
	ret

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
	adc ZH,zero

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
	andi c_tmp,0xC0
	or ACC,c_tmp
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
;---------------------------------------------------------

UpdateDDRAM:
	; byte to update is in ARGL
	push idx


	lds idx,AC				; get address counter

	ldi XH,high(DDRAM)
	ldi XL,low(DDRAM)

	add XL,idx				
	adc XH,zero

	st X,ARGL				; save data to DDRAM

	lds c_tmp,WINDOWR
	cp idx,c_tmp
	brge L0991				; out of range
				
	lds c_tmp,WINDOWL
	cp idx,c_tmp
	brlo L0991				; out of range

	; in rage

	; shift display or

	; place data at cursor

	lds ARGL,CURSOR
	lds idx,AC				; reload address counter
	rcall _putSymbol		; write data at cursor

	subi ARGL,-3			; advance cursor
	cpi ARGL,DIGITS*3
	brlo L1039	
	ldi ARGL,1
	
L1039:
	sts CURSOR,ARGL

L0991:
	lds idx,AC				; reload address counter

	; test entry mode
	sbrs fFlags,EmI
	rjmp L1067 
; increment mode
	inc idx					; advance AC
	cpi idx,80
	brlo  L1059
	clr idx					; wrap to start
	rjmp L1059
L1067:
	dec idx					; backup AC
	sbr idx,7
	ldi idx,80				; wrap to end					

L1059:
	sts AC,idx				; record new cursor
	
	pop idx
	ret

UpdateGCRAM:

	ret
;----------------------------------------------------------

; read data and cg routines

; read clock

; set clock




;*************************************
; MEP mph MLH Main reset entry point
;*************************************
Main:

; init serial ports for MIDI baud rate
	sbi PORTD,PortD4	; Keep SR enable high during powerup

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

	CLI

	clr zero
	clr FFLAGS

	clr r4			; clear the out ring pointers
	clr r5			; 
	clr r6			; clear the in ring pointers
	clr r7


; timers are in dos time format

; time
	
	ldi ACC, ((Set_Minute << 5)&0xE0) | (Set_Second & 0x1F)		;0x39 = 57  0x1A = 26 sec
	mov TIME0,ACC  ; mmmsssss
	ldi ACC, ((Set_Hour << 3)&0xF8) | ((Set_Minute >> 3)&0x07) ; 0x16 = 22 h
	mov TIME1,ACC  ; hhhhhmmm


; date
	ldi ACC,(((Set_Year-1980) << 1) & 0xFE) | ((Set_Month >> 3) & 0x01)
	mov TIME3,ACC
	ldi ACC, ((Set_Month << 5)&0xE0) | (Set_Day & 0x1F)
	mov TIME2,ACC



	ldi YL,low(SegMUX+0xFF)
	ldi YH,high(SegMUX+0xFF)
L0413:
	st -Y,zero
	cp YL,zero
	brne L0413

	; init shift out registers to safe value	

	; these registers are dedicated to the shift register	
	; this is a price for simplicity as the Y register is not
	; available for general use	

	ldi MaxBits,21		; 20 bits in field pattern
	ldi ACC,0x3F		; mask for input and output rings
	mov nibbleMask,ACC

	ldi ACC,8			; reload value for shift register Max6921 has 20 bits		
	mov const7,ACC		

	mov bshift,const7
	ldi YL,low(SegMUX+DIGITS*3)
	ldi YH,high(SegMUX+DIGITS*3)
	ld ashift,-Y


	; this will need to be changed to use TWI instead of the UASART

	ldi	r16,kBaud31250	; kBaud 15 for midi clock at 8 Mhz	
	rcall	L13DC		; init uart



	; Option switches		; there are 64 possible configurations
	out PORTC,nibbleMask	; By chance the mask for the output pull
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
	sts TCCR1B,ACC



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

	ldi ACC, (1 << WGM02) 
	out TCCR0B,ACC

;******************************************************************
; 	Enable display defaults
;******************************************************************

							; set some defaults using the emulator
	ldi ARGL,0x30			; normal function set
	rcall ParseCommand
	
	ldi ARGL,0x01			; clear display
	rcall ParseCommand		; this will clear DDRAM and init flags

	ldi ARGL,0x0C			; start display timers
	rcall ParseCommand


	; check if clock needs setting

	; display clock setting screen  ( we have several choices )
	; dedicated bits on port C 

	; IR remote control (lot of work)

	; master read/write from main controller  (bad clock bit could be part of the 
	; time frame.


; display something for testing
	ldi ARGL,0x80					; set DDRAM entry address
	rcall ParseCommand


	ldi ARGL,'H'					; not very origional, but traditional
	rcall UpdateDDRAM
	ldi ARGL,'E'
	rcall UpdateDDRAM
	ldi ARGL,'L'
	rcall UpdateDDRAM
	ldi ARGL,'L'
	rcall UpdateDDRAM
	ldi ARGL,'O'
	rcall UpdateDDRAM
	ldi ARGL,' '
	rcall UpdateDDRAM
	ldi ARGL,'W'
	rcall UpdateDDRAM
	ldi ARGL,'O'
	rcall UpdateDDRAM
	ldi ARGL,'R'
	rcall UpdateDDRAM
	ldi ARGL,'L'
	rcall UpdateDDRAM
	ldi ARGL,'D'
	rcall UpdateDDRAM
	ldi ARGL,' '
	rcall UpdateDDRAM

	; all charaters to buffer

	ldi ARGL,0x21
L1293:
	rcall UpdateDDRAM
	inc ARGL
	cpi ARGL,80+16
	brlo L1293

	ldi ARGL,'{'
	rcall UpdateDDRAM
	ldi ARGL,'|'
	rcall UpdateDDRAM
	ldi ARGL,'}'
	rcall UpdateDDRAM
	ldi ARGL,'~'
	rcall UpdateDDRAM
	ldi ARGL,0x7F
	rcall UpdateDDRAM
	


					 

IdleLoop:
	WDR
	
	sbrs fFlags,TickTock
	rjmp IdleLoop

	cbr fFlags,(1<<TickTock)

; test clock
.if 1
	; this is probably a better place to decode the date time 
	; information for testing
	ldi ARGL,0x80					; set DDRAM entry address
	rcall ParseCommand

	; display Hour

	; ARGH:L is treated as dos little endian

	mov ARGH,TIME1
	andi ARGH,0xF8
	lsr ARGH
	lsr ARGH
	lsr ARGH   ;hour = ((cjl_dir_buffer[k+23] & 0xF8)>>3);	

	rcall bin2bcd8
	subi ARGH,0xD0	; add 0x30 using inverse subtraction
	subi ARGL,0xD0	; add 0x30 using inverse subtraction
	push ARGH
	rcall UpdateDDRAM
	pop ARGL
	rcall UpdateDDRAM

	mov ARGH,TIME1
	mov ARGL,TIME0

	andi ARGH,0x07
	andi ARGL,0xE0
	lsl ARGL
	rol ARGH
	rol ARGL
	rol ARGH
	rol ARGL
	rol ARGH	;min = ((cjl_dir_buffer[k+22] & 0xE0)>>5) +  ((cjl_dir_buffer[k+23] & 0x07)<<3);

	rcall bin2bcd8
	subi ARGH,0xD0	; add 0x30 using inverse subtraction
	subi ARGL,0xD0	; add 0x30 using inverse subtraction
	push ARGH
	rcall UpdateDDRAM
	pop ARGL
	rcall UpdateDDRAM


	mov ARGH,TIME3	;sec = (cjl_dir_buffer[k+22] & 0x1F) * 2;
;	andi ARGH,0x1F
;	lsl ARGH
 
	rcall bin2bcd8
	subi ARGH,0xD0	; add 0x30 using inverse subtraction
	subi ARGL,0xD0	; add 0x30 using inverse subtraction
	push ARGH
	rcall UpdateDDRAM
	pop ARGL
	rcall UpdateDDRAM



	;date = cjl_dir_buffer[k+24] & 0x1F;
 	;mnum = ((cjl_dir_buffer[k+24] & 0xE0)>>5) + ((cjl_dir_buffer[k+25] & 0x01)<<3);
 	;year = ((cjl_dir_buffer[k+25] & 0xFE)>>1) + 1980;


.endif

	; update the display by shifting left
;	ldi ARGL,0x18			; should be shift display left
;	rcall ParseCommand

	rjmp IdleLoop

.if 0
	ldi ACC, (3 << SM0) | (1 << SE)
	out SMCR,ACC
	sleep
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
.endif		


	
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



;***** Subroutine Register Variables

;.def	fbin	=ARGL		;8-bit binary value
;.def	tBCDL	=ARGL		;BCD result MSD
;.def	tBCDH	=ARGH		;BCD result LSD

;***** Code
; this is optimized for calling the displaywrite routines
; so the Least significant digit is in ARGH or little endian


bin2bcd8:
	clr	ARGL				;clear result MSD
bBCD8_1:
	subi	ARGH,10			;input = input - 10
	brcs	bBCD8_2		;abort if carry set
	inc	ARGL				;inc MSD
	rjmp	bBCD8_1		;loop again

bBCD8_2:
	subi	ARGH,-10	;compensate extra subtraction

	ret


; functions from AVR 311 coded for ASM  -- these could be
; converted into include files for readability

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master staus codes                      
.equ TWI_START                  = 0x08  // START has been transmitted  
.equ TWI_REP_START              = 0x10  // Repeated START has been transmitted
.equ TWI_ARB_LOST               = 0x38  // Arbitration lost

// TWI Master Transmitter staus codes                      
.equ TWI_MTX_ADR_ACK            = 0x18  // SLA+W has been tramsmitted and ACK received
.equ TWI_MTX_ADR_NACK           = 0x20  // SLA+W has been tramsmitted and NACK received 
.equ TWI_MTX_DATA_ACK           = 0x28  // Data byte has been tramsmitted and ACK received
.equ TWI_MTX_DATA_NACK          = 0x30  // Data byte has been tramsmitted and NACK received 

// TWI Master Receiver staus codes  
.equ TWI_MRX_ADR_ACK            = 0x40  // SLA+R has been tramsmitted and ACK received
.equ TWI_MRX_ADR_NACK           = 0x48  // SLA+R has been tramsmitted and NACK received
.equ TWI_MRX_DATA_ACK           = 0x50  // Data byte has been received and ACK tramsmitted
.equ TWI_MRX_DATA_NACK          = 0x58  // Data byte has been received and NACK tramsmitted

// TWI Slave Transmitter staus codes
.equ TWI_STX_ADR_ACK            = 0xA8  // Own SLA+R has been received; ACK has been returned
.equ TWI_STX_ADR_ACK_M_ARB_LOST = 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
.equ TWI_STX_DATA_ACK           = 0xB8  // Data byte in TWDR has been transmitted; ACK has been received
.equ TWI_STX_DATA_NACK          = 0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
.equ TWI_STX_DATA_ACK_LAST_BYTE = 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received

// TWI Slave Receiver staus codes
.equ TWI_SRX_ADR_ACK            = 0x60  // Own SLA+W has been received ACK has been returned
.equ TWI_SRX_ADR_ACK_M_ARB_LOST = 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
.equ TWI_SRX_GEN_ACK            = 0x70  // General call address has been received; ACK has been returned
.equ TWI_SRX_GEN_ACK_M_ARB_LOST = 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
.equ TWI_SRX_ADR_DATA_ACK       = 0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
.equ TWI_SRX_ADR_DATA_NACK      = 0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
.equ TWI_SRX_GEN_DATA_ACK       = 0x90  // Previously addressed with general call; data has been received; ACK has been returned
.equ TWI_SRX_GEN_DATA_NACK      = 0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
.equ TWI_SRX_STOP_RESTART       = 0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
.equ TWI_NO_STATE               = 0xF8  // No relevant state information available; TWINT = “0”
.equ TWI_BUS_ERROR              = 0x00  // Bus error due to an illegal START or STOP condition


; code from AVR311 -- some of this may use C stack frames

;/****************************************************************************
;This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
;that is whenever a TWI event has occurred. This function should not be called directly from the main
;application.
;****************************************************************************/
;ISR()
TWI_vect:

;{
	push	r1
	push	r0
	in	r0, SREG	; (0x3F) 63
	push	r0
	eor	r1, r1
	push	r24
	push	r25
	push	r30
	push	r31

;  static unsigned char TWI_bufPtr;
  
;  switch (TWSR)
	  lds	r24,TWSR
 	  cpi	r24,TWI_SRX_GEN_DATA_ACK	; 
	  brne	L14c	;.+2      	; 0x14c <__vector_24+0x1c>
	  rjmp	L1fa	;.+174    	; 0x1fa <__vector_24+0xca>
L14c: cpi	r24,0x91
	  brcc	L176;	.+38     	; 0x176 <__vector_24+0x46>
	  cpi	r24,TWI_SRX_GEN_ACK
	  brne	L156	;.+2      	; 0x156 <__vector_24+0x26>
	  rjmp	L1d4	;.+126    	; 0x1d4 <__vector_24+0xa4>
L156: cpi	r24,0x71	; 113
	  brcc	L168	;.+14     	; 0x168 <__vector_24+0x38>
	  and	r24, r24
	  brne	L160	;.+2      	; 0x160 <__vector_24+0x30>
	  rjmp	L21e	;.+190    	; 0x21e <__vector_24+0xee>
L160: cpi	r24,TWI_SRX_ADR_ACK
	  breq	L166	;.+2      	; 0x166 <__vector_24+0x36>
	  rjmp	L22e	;.+200    	; 0x22e <__vector_24+0xfe>
L166: rjmp	L1de	;.+118    	; 0x1de <__vector_24+0xae>
L168: cpi	r24,TWI_SRX_ADR_DATA_ACK
	  brne	L16e	;.+2      	; 0x16e <__vector_24+0x3e>
	  rjmp	L1fa	;.+140    	; 0x1fa <__vector_24+0xca>
L16e: cpi	r24,TWI_SRX_ADR_DATA_NACK
	  breq	L174	;.+2      	; 0x174 <__vector_24+0x44>
	  rjmp	L22e	;.+186    	; 0x22e <__vector_24+0xfe>
L174: rjmp	L21e	;.+168    	; 0x21e <__vector_24+0xee>
L176: cpi	r24,TWI_STX_ADR_ACK
	  breq	L19e	;.+36     	; 0x19e <__vector_24+0x6e>
	  cpi	r24, 0xA9	; 169
	  brcc	L18c	;.+14     	; 0x18c <__vector_24+0x5c>
	  cpi	r24,TWI_SRX_GEN_DATA_NACK
	  brne	L184	;.+2      	; 0x184 <__vector_24+0x54>
	  rjmp	L21e	;.+154    	; 0x21e <__vector_24+0xee>
L184: cpi	r24,TWI_SRX_STOP_RESTART
	  breq	L18a	;.+2      	; 0x18a <__vector_24+0x5a>
	  rjmp	L22e	;.+164    	; 0x22e <__vector_24+0xfe>
L18a: rjmp	L236	;.+170    	; 0x236 <__vector_24+0x106>
L18c: cpi	r24,TWI_STX_DATA_NACK
	  breq	L1bc	;.+44     	; 0x1bc <__vector_24+0x8c>
L190: cpi	r24,TWI_STX_DATA_ACK_LAST_BYTE
	  brne	L196	;.+2      	; 0x196 <__vector_24+0x66>
	  rjmp	L21e	;.+136    	; 0x21e <__vector_24+0xee>
L196: cpi	r24,TWI_STX_DATA_ACK
	  breq	L19c	;.+2      	; 0x19c <__vector_24+0x6c>
	  rjmp	L22e	;.+146    	; 0x22e <__vector_24+0xfe>
L19c: rjmp	L1a2	;.+4      	; 0x1a2 <__vector_24+0x72>

;  {
;    case TWI_STX_ADR_ACK:            // Own SLA+R has been received; ACK has been returned
;//  case TWI_STX_ADR_ACK_M_ARB_LOST: // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
;		TWI_bufPtr   = 0;             // Set buffer pointer to first data location

L19e:
	sts	TWI_bufPtr, r1		; 0x0109
    
;	case TWI_STX_DATA_ACK:           // Data byte in TWDR has been transmitted; ACK has been received
;      TWDR = TWI_buf[TWI_bufPtr++];

L1a2:
	lds	r24,TWI_bufPtr
	mov	r30,r24
	ldi	r31,0x00
	subi r30,low(-TWI_buf)	;0xFD TWI_buf -- need to fix this address
	sbci r31,high(-TWI_buf)	;0xFE	; 0x0103 
	ld	r25, Z
	sts	TWDR,r25
	subi	r24,0xFF		; complementy add
	sts	TWI_bufPtr,r24
	rjmp L1ec	;.+48     		; 0x1ec <__vector_24+0xbc>

;   	(0<<TWWC);                                 //
;		TWI_busy = 1;
;      break;

;    case TWI_STX_DATA_NACK:          // Data byte in TWDR has been transmitted; NACK has been received. 
;                                     // I.e. this could be the end of the transmission.
;      if (TWI_bufPtr == TWI_msgSize) // Have we transceived all expected data?

L1bc:
	lds	r25, TWI_bufPtr
	lds	r24,TWI_msgSize	; 0x0108
	cp	r25, r24
	brne L22e	;.+102    	; 0x22e <__vector_24+0xfe>
;      {
;        TWI_statusReg.lastTransOK = TRUE;               // Set status bits to completed successfully. 
	lds	r24,TWI_statusReg		; 0x0102
	ori	r24,(1<<lastTransOK)	; 0x01
	sts	TWI_statusReg,r24
	rjmp L236	;.+98     	; 0x236 <__vector_24+0x106>
      
;      TWI_busy = 0;   // Transmit is finished, we are not busy anymore
;      break;     

;    case TWI_SRX_GEN_ACK:            // General call address has been received; ACK has been returned
;//    case TWI_SRX_GEN_ACK_M_ARB_LOST: // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
;      TWI_statusReg.genAddressCall = TRUE;
L1d4:
	lds	r24,TWI_statusReg
	ori	r24,(1 << genAddressCall)	; 0x04
	sts	TWI_statusReg,r24

;    case TWI_SRX_ADR_ACK:            // Own SLA+W has been received ACK has been returned
;//    case TWI_SRX_ADR_ACK_M_ARB_LOST: // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned    
;                                                        // Dont need to clear TWI_S_statusRegister.generalAddressCall due to that it is the default state.
;      TWI_statusReg.RxDataInBuf = TRUE;      
L1de:
	lds	r24,TWI_statusReg
	ori	r24, (1 <<RxDataInBuf)		;0x02
	sts	TWI_statusReg, r24
;     TWI_bufPtr   = 0;             // Set buffer pointer to first data location
	sts	TWI_bufPtr, r1
      
;                                   // Reset the TWI Interupt to wait for a new event.
;      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
L1ec:
;             (1<<TWIE)|(1<<TWINT)|            // Enable TWI Interupt and clear the flag to send byte
;             (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)| // Expect ACK on this transmission
;             (0<<TWWC);  
	ldi	r24,(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC)	; 0xC5
	sts	TWCR,r24
             ;  
;   TWI_busy = 1;
L1f2:
	ldi	r24, 0x01
	sts	TWI_busy,r24
      
;      break;
	rjmp L240	;.+70     	; 0x240 <__vector_24+0x110>
 ;   case TWI_SRX_ADR_DATA_ACK:       // Previously addressed with own SLA+W; data has been received; ACK has been returned
 ;   case TWI_SRX_GEN_DATA_ACK:       // Previously addressed with general call; data has been received; ACK has been returned
 ;     TWI_buf[TWI_bufPtr++]     = TWDR;
 L1fa:
 	lds	r24, TWI_bufPtr
	lds	r25, TWDR
	mov	r30, r24
	ldi	r31, 0x00
	subi r30,low(-TWI_buf)	;0xFD TWI_buf -- need to fix this address
	sbci r31,high(-TWI_buf)	;0xFE	; 0x0103 
	st	Z, r25
	subi	r24, 0xFF		; complementry add
	sts	TWI_bufPtr, r24
 
 ;     TWI_statusReg.lastTransOK = TRUE;                 // Set flag transmission successfull.       
 L212:
 	lds	r24,TWI_statusReg
	ori	r24,(1 <<lastTransOK)		; 0x01
	sts	0x0102,r24
	rjmp L1ec	;.-50     	; 0x1ec <__vector_24+0xbc>

;    case TWI_SRX_ADR_DATA_NACK:      // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
;    case TWI_SRX_GEN_DATA_NACK:      // Previously addressed with general call; data has been received; NOT ACK has been returned
;    case TWI_STX_DATA_ACK_LAST_BYTE: // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
;//    case TWI_NO_STATE              // No relevant state information available; TWINT = “0”
;    case TWI_BUS_ERROR:        	  // Bus error due to an illegal START or STOP condition
;      TWI_state = TWSR;              //Store TWI State as errormessage, operation also clears noErrors bit

L21e:
	lds	r24, TWSR
	sts	TWI_state,r24						;0x0100, r24
 ;     TWCR =   (1<<TWSTO)|(1<<TWINT)	//Recover from TWI_BUS_ERROR, this will release the SDA and SCL pins thus enabling other devices to use the bus
	ldi	r24,(1<<TWSTO)|(1<<TWINT)		; 0x90
	sts	TWCR,r24
;      break;
	rjmp L240	;.+18     	; 0x240 <__vector_24+0x110>

;    default:     
;      TWI_state = TWSR;                                 // Store TWI State as errormessage, operation also clears the Success bit.      
L22e:
	lds	r24, TWSR
	sts	TWI_state, r24

;      TWCR = (1<<TWEN)|                            // Enable TWI-interface and release TWI pins
;             (1<<TWIE)|(1<<TWINT)|                 // Keep interrupt enabled and clear the flag
;             (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|      // Acknowledge on any new requests.
;             (0<<TWWC);                            //
L236:
	ldi	r24,(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC)	; 0xC5
	sts	TWCR,r24
      
;   TWI_busy = 0; // Unknown status, so we wait for a new address match that might be something we can handle
	sts	TWI_busy,r1
  }
}
L240:
	pop	r31
	pop	r30
	pop	r25
	pop	r24
	pop	r0
	out	SREG, r0	; 63
	pop	r0
	pop	r1
	reti

;--------------------------------------------------------------------------

; TWI AVR311 library functions


TWI_Slave_Initialise:
;/****************************************************************************
;Call this function to set up the TWI slave to its initial standby state.
;Remember to enable interrupts from the main application after initializing the TWI.
;Pass both the slave address and the requrements for triggering on a general call in the
;same byte. Use e.g. this notation when calling this function:
;TWI_Slave_Initialise( (TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) );
;The TWI module is configured to NACK on any requests. Use a TWI_Start_Transceiver function to 
;start the TWI.
; ****************************************************************************/
;void TWI_Slave_Initialise( unsigned char TWI_ownAddress, unsigned char TWI_ownAddressMask  )
;{
;  TWAR = TWI_ownAddress;                      // Set own TWI slave address. Accept TWI General Calls.
	sts	TWAR,ARGL

;  TWAMR= TWI_ownAddressMask;                  // Set own TWI slave address mask to enable respons on several addresses.
	sts	TWAMR, ARGH

;  TWCR = (1<<TWEN)|                           // Enable TWI-interface and release TWI pins.
;   (0<<TWIE)|(0<<TWINT)|                      // Disable TWI Interupt.
;   (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Do not ACK on any requests, yet.
;    (0<<TWWC);                                //

	ldi	ACC,(1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); 0x04
	sts	TWCR, ACC

;   TWI_busy = 0;
	sts	TWI_busy,zero
;}    
	ret

TWI_Transceiver_Busy:
; Call this function to test if the TWI_ISR is busy transmitting.
; ****************************************************************************/
; unsigned char TWI_Transceiver_Busy( void )
; {
;  return TWI_busy;
; }
	lds	ARGL,TWI_busy
	ret


TWI_Get_State_Info:
;/****************************************************************************
;Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
;until the TWI_ISR has completed with the previous operation. If there was an error, then the function 
;will return the TWI State code. 
;****************************************************************************/
;unsigned char TWI_Get_State_Info( void )
;{
;  while ( TWI_Transceiver_Busy() ) {}             // Wait until TWI has completed the transmission.
	lds	ACC,TWI_busy
	and	ACC, ACC
	brne L094	;.+6      	; 0x94 <TWI_Get_State_Info+0xe>
;  return ( TWI_state );                         // Return error state. 
;}
	lds	ARGL,TWI_state
	ret
L094:
	rjmp L094	;.-2      	; 0x94 <TWI_Get_State_Info+0xe>

TWI_Start_Transceiver_With_Data:
;****************************************************************************/
;void TWI_Start_Transceiver_With_Data( unsigned char *msg, unsigned char msgSize )
;{
;  unsigned char temp;
	; register mapping 
	; X = *msg (could be in ARGH:ARGL)
	; ARGL is msgSize

;  while ( TWI_Transceiver_Busy() ) {}             // Wait until TWI is ready for next transmission.
	lds	ACC, TWI_busy
	and	ACC, ACC
	brne L0d2	;.+52     	; 0xd2 <TWI_Start_Transceiver_With_Data+0x3c>

;  TWI_msgSize = msgSize;                        // Number of data to transmit.
	sts	TWI_msgSize, ARGL
	; call straight with X
	
	;movw	X,ARGL				; r26, r24
	ldi	ZL,low(TWI_buf)			; 0x03
	ldi	ZH,high(TWI_buf)		; 0x01

;/****************************************************************************
;Call this function to send a prepared message, or start the Transceiver for reception. Include
;a pointer to the data to be sent if a SLA+W is received. The data will be copied to the TWI buffer. 
;Also include how many bytes that should be sent. Note that unlike the similar Master function, the
;Address byte is not included in the message buffers.
;The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
;then initialize the next operation and return.
;****************************************************************************/


L0a8:

	ldi	ARGH, 0x00	; r23
	add	ARGL, r30	; r22
	adc	ARGH, r31	; r23	; temp < msgSize

;  for ( temp = 0; temp < msgSize; temp++ )      // Copy data that may be transmitted if the TWI Master requests data.
	rjmp L0B4	;.+4      	; 0xb4 <TWI_Start_Transceiver_With_Data+0x1e>
;  {
;    TWI_buf[ temp ] = msg[ temp ];
L0b0:
	ld	ACC, X+		; X -> passed in message
	st	Z+,ACC		; Z -> static buffer

L0b4:
	cp	r30,ARGL	; r22
	cpc	r31,ARGH	; r23
	brne L0b0		; .-10     	; 0xb0 <TWI_Start_Transceiver_With_Data+0x1a>

;  TWI_statusReg.all = 0;      
	sts	TWI_statusReg, zero
;  TWI_state         = TWI_NO_STATE ;
	ldi	ACC,TWI_NO_STATE
	sts	TWI_state, ACC

	ldi	ACC,(1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); 0x04
	sts	TWCR, ACC
;  TWI_busy = 1;

	ldi	ACC, 0x01
	sts	TWI_busy,ACC
;}
	ret
L0d2:
	rjmp L0d2	;.-2      	; 0xd2 <TWI_Start_Transceiver_With_Data+0x3c>

TWI_Start_Transceiver:
;/****************************************************************************
;Call this function to start the Transceiver without specifing new transmission data. Useful for restarting
;a transmission, or just starting the transceiver for reception. The driver will reuse the data previously put
;in the transceiver buffers. The function will hold execution (loop) until the TWI_ISR has completed with the 
;previous operation, then initialize the next operation and return.
;****************************************************************************/
;void TWI_Start_Transceiver( void )
;{
;  while ( TWI_Transceiver_Busy() ) {}             // Wait until TWI is ready for next transmission.
	lds	ARGL, TWI_busy
	and	ARGL, ARGL
	brne L0f2	;.+22     	; 0xf2 <TWI_Start_Transceiver+0x1e>

;  TWI_statusReg.all = 0;      
	sts	TWI_statusReg, zero

;  TWI_state         = TWI_NO_STATE ;
	ldi	ACC,TWI_NO_STATE
	sts	TWI_state, ACC

	ldi	ACC,(1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); 0x04
	sts	TWCR, ACC
;  TWI_busy = 1;

	ldi	ACC, 0x01
	sts	TWI_busy,ACC
;}
	ret
L0f2:
	rjmp L0f2	;.-2      	; 0xf2 <TWI_Start_Transceiver+0x1e>


TWI_Get_Data_From_Transceiver:
;****************************************************************************/
;unsigned char TWI_Get_Data_From_Transceiver( unsigned char *msg, unsigned char msgSize )
;{

; *msg is in X , msgSize is in ARGL
;  unsigned char i;

;  while ( TWI_Transceiver_Busy() ) {}             // Wait until TWI is ready for next transmission.
	lds	ACC, TWI_busy
	and	ACC, ACC
	brne L12e	;.+50     	; 0x12e <TWI_Get_Data_From_Transceiver+0x3a>

;  if( TWI_statusReg.lastTransOK )               // Last transmission completed successfully.              
	lds	r18,TWI_statusReg
	sbrs	r18,lastTransOK
	rjmp L126	;.+34     	; 0x126 <TWI_Get_Data_From_Transceiver+0x32>

	ldi	ZL,low(TWI_buf)			; 0x03
	ldi	ZH,high(TWI_buf)		; 0x01

	;movw	X,ARGL				; r26, r24


;Call this function to read out the received data from the TWI transceiver buffer. I.e. first call
;TWI_Start_Transceiver to get the TWI Transceiver to fetch data. Then Run this function to collect the
;data when they have arrived. Include a pointer to where to place the data and the number of bytes
;to fetch in the function call. The function will hold execution (loop) until the TWI_ISR has completed 
;with the previous operation, before reading out the data and returning.
;If there was an error in the previous transmission the function will return the TWI State code.
;****************************************************************************/
	ldi	ARGH, 0x00	; 0
 	add	ARGL, r30
	adc	ARGH, r31
	rjmp L116	;.+4      	; 0x116 <TWI_Get_Data_From_Transceiver+0x22>

;  if( TWI_statusReg.lastTransOK )               // Last transmission completed successfully.              
;  {                                             
;    for ( i=0; i<msgSize; i++ )                 // Copy data from Transceiver buffer.
;    {
;      msg[ i ] = TWI_buf[ i ];
L112:
	ld	ACC, Z+
	st	X+,ACC

L116:
	cp	ZL,ARGL
	cpc	ZH,ARGH
	brne L112	;.-10     	; 0x112 <TWI_Get_Data_From_Transceiver+0x1e>

;   TWI_statusReg.RxDataInBuf = FALSE;          // Slave Receive data has been read from buffer.
	lds	ACC,TWI_statusReg
 	andi ACC,~(1 << RxDataInBuf) ; 0xFD	; 253
	sts	TWI_statusReg, ACC
;  }
;  return( TWI_statusReg.lastTransOK );                                   
	lds	ARGL,TWI_statusReg
;}
L126:
	andi ACC,lastTransOK
	ret
L12e:
	rjmp L12e	;.-2      	; 0x12e <TWI_Get_Data_From_Transceiver+0x3a>





; Normal table
ASCIITAB:
;   0/8	      1/9        2/A      3/B       4/C       5/D       6/E       7/F
; control group
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 ; 0
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 ; 1
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
; display chars
.db 0x00,0x00,0x10,0x80,0x14,0x00,0x15,0xD2,0x25,0xD3,0x08,0x89,0x2A,0x76,0x04,0x00
.db 0x20,0x45,0x30,0xC0,0x0F,0x3A,0x05,0x12,0x00,0x08,0x01,0x02,0x00,0x04,0x08,0x08
.db 0x38,0xCD,0x18,0x80,0x31,0x46,0x31,0xC0,0x11,0x83,0x21,0xC3,0x21,0xC7,0x28,0x10
.db 0x31,0xC7,0x31,0xC3,0x00,0x05,0x04,0x08,0x08,0x20,0x01,0x42,0x02,0x08,0x31,0x11
.db 0x35,0x45,0x31,0x87,0x35,0xD0,0x20,0x45,0x34,0xD0,0x21,0x47,0x21,0x07,0x21,0xC5
.db 0x11,0x87,0x24,0x50,0x10,0xC4,0x08,0x27,0x00,0x45,0x1A,0x85,0x12,0xA5,0x30,0xC5
.db 0x31,0x07,0x30,0xE5,0x31,0x27,0x23,0xC0,0x24,0x10,0x10,0xC5,0x08,0x0D,0x10,0xAD
.db 0x0A,0x28,0x0A,0x10,0x28,0x48,0x20,0x45,0x02,0x20,0x30,0xC0,0x00,0x28,0x00,0x40
.db 0x04,0x00,0x31,0x87,0x35,0xD0,0x20,0x45,0x34,0xD0,0x21,0x47,0x21,0x07,0x21,0xC5
.db 0x11,0x87,0x24,0x50,0x10,0xC4,0x08,0x27,0x00,0x45,0x1A,0x85,0x12,0xA5,0x30,0xC5
.db 0x31,0x07,0x30,0xE5,0x31,0x27,0x23,0xC0,0x24,0x10,0x10,0xC5,0x08,0x0D,0x10,0xAD
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
.db 0x00,0x00,0x00,0x05,0x00,0x14,0x25,0x17,0x25,0xD3,0x08,0x89,0x37,0x68,0x00,0x10
.db 0x30,0xC0,0x20,0x45,0x0F,0x3A,0x05,0x12,0x08,0x00,0x01,0x02,0x10,0x00,0x08,0x08
.db 0x38,0xCD,0x00,0x0D,0x31,0x46,0x20,0x47,0x01,0x87,0x21,0xC3,0x31,0xC3,0x04,0x48
.db 0x31,0xC7,0x21,0xC7,0x10,0x80,0x08,0x10,0x02,0x08,0x21,0x02,0x08,0x20,0x04,0xC6
.db 0x30,0xD6,0x11,0xC7,0x24,0x57,0x30,0xC0,0x24,0x55,0x31,0xC2,0x11,0xC2,0x30,0xC3
.db 0x11,0x87,0x24,0x50,0x30,0x05,0x13,0x88,0x30,0x80,0x10,0xAD,0x12,0xA5,0x30,0xC5
.db 0x11,0xC6,0x32,0xC5,0x13,0xC6,0x20,0x63,0x04,0x50,0x30,0x85,0x18,0x88,0x1A,0x85
.db 0x0A,0x28,0x04,0x28,0x28,0x48,0x30,0xC0,0x02,0x20,0x20,0x45,0x0A,0x00,0x20,0x00
.db 0x00,0x10,0x11,0xC7,0x24,0x57,0x30,0xC0,0x24,0x55,0x31,0xC2,0x11,0xC2,0x30,0xC3
.db 0x11,0x87,0x24,0x50,0x30,0x05,0x13,0x88,0x30,0x80,0x10,0xAD,0x12,0xA5,0x30,0xC5
.db 0x11,0xC6,0x32,0xC5,0x13,0xC6,0x20,0x63,0x04,0x50,0x30,0x85,0x18,0x88,0x1A,0x85
.db 0x0A,0x28,0x04,0x28,0x28,0x48,0x29,0x60,0x04,0x10,0x22,0x4A,0x00,0xA6,0x3F,0xFF









