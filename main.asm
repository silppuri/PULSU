;;;;;;; Description ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; Pulse generator
; 1. RPG to select frequency displayed in (MHz) the top right corner of LCD
;    Possible values are: ###
; 2. POT to modify the amplitude of the pulse [-15v; +15v] sent to DAC-A
; 3. Button cycles pulse mode. SIN, SQR, TRI
; 4. Alive LED blinks every 2,5 sec

;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Mainline
;   Initial
;     InitLCD
;       LoopTime
;     DisplayC
;       T40
;   BlinkAlive
;   Button
;   	ButtonToggle
;     		DisplayC
;       		T40
;   RPG
;   RPGCounter
;     HexDisplay
;       ConvertLowerNibble
;       DisplayV
;         T40
;   ReadPot
;     HexDisplay
;       ConvertLowerNibble
;       DisplayV
;         T40
;   PotToDAC
;     SPItransfer
;   LoopTime
;
;;;;;;; Assembler directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	list  P=PIC18F452, F=INHX32, C=160, N=0, ST=OFF, MM=OFF, R=DEC, X=ON
	#include P18F452.inc
	__CONFIG  _CONFIG1H, _HS_OSC_1H  ;HS oscillator
	__CONFIG  _CONFIG2L, _PWRT_ON_2L & _BOR_ON_2L & _BORV_42_2L  ;Reset
	__CONFIG  _CONFIG2H, _WDT_OFF_2H  ;Watchdog timer disabled
	__CONFIG  _CONFIG3H, _CCP2MX_ON_3H  ;CCP2 to RC1 (rather than to RB3)
	__CONFIG  _CONFIG4L, _LVP_OFF_4L  ;RB5 enabled for I/O
	errorlevel -314, -315          ;Ignore lfsr messages

;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

cblock  0x000                  	; Beginning of Access RAM
	COUNT                      	; Counter available as local to subroutines
	TABL_COUNT					; Counter for vawe loops
	ALIVECNT                   	; Counter for blinking "Alive" LED (D2)
	DELRPG                     	; Used DELRPG to let the programmer know whether
                                   	;  or not the RPG moved and at which direction 
                                   	;  (-1 for left or +1 for right).
	OLDPORTD                   	; Holds the old value of PortD to compare with
                                   	;  the new value. Used in RPG.
	OLDBUTTON                   ; Byte that holds old button state.
	MODE						; holds mode state. 0 = OFF, 1 = SIN, 2=SQUARE, 3=TRI
	PBCOUNT                    	; Used in PButton subroutine.  If it's less than
                                   	;  PBthresh, then ISC is set.  Otherwise, ISA is set.
                                   	;  This lets PButton know if it's a short or long push.
	POTVALUE                   	; Holds the current value of the POT.
	HEXSTR:4                   	; String used to display the hex values located
                                   	;  on the right side of the LCD (output of the POT and the DAC)
	BYTE                       	; Temporary variable used for anything.
	SPIRECEIVE              	; Not used in this program, but if something
                               	    ;  was to be sent back to the SPI, it would 
                                   	;  be saved in this variable.
	RPGCNT                     	; Used internally for the RPG subroutine.
	CNT_TITLE		      		; Counter for the titles.
	VALUE			       		; Variable used by RPG.
	OUTPUT
endc

#include c:\math18\mathvars.inc

;;;;;;; Macro definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; These are the binary variables used for the PButton and RPG subroutines.
OLDPB   equ 3                  ; Old state of pushbutton.
RGPTresh equ 6			       ; RPG treshold for fast turns.


; Lets the programmer store a literal value in a RAM location directly.
;

MOVLF   macro  literal,dest
        movlw  literal
        movwf  dest
        endm

; Used to point TBLPTRH to a constant string (stored in program memory) so that 
; it can be displayed on the LCD.
;

POINT   macro  stringname
        MOVLF  high stringname, TBLPTRH
        MOVLF  low stringname,  TBLPTRL
        endm

;;;;;;; Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	org   0x0000                   ; Reset vector.
	nop 
	goto  Mainline

	org   0x0008                   ; High priority interrupt vector.
	goto  $                        ; If interrupt is generated, go into this infinite loop.

	org   0x0018                   ; Low priority interrupt vector.
	goto  $                        ; If interrupt is generated, go into this infinite loop.

;;;;;;; Mainline program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Mainline
        rcall  Initial                 ;Initialize everything.

;LOOP
L1
	btg    PORTB,RB0             ; Toggles the B0/INT0 pin to measure the total time.  Should be 10ms.
	rcall  BlinkAlive            ; Blinks the D2 LED every 2.5 seconds.
	rcall  Button            	 ; Checks to see if SW3 is pushed or not.
	rcall  sine_wave			 ; generate one value of sine wave
	rcall  triang_wave			 ; generate one value of triangle wave
	rcall  square_wave			 ; generate one value of square wave
	;rcall  AdjustFrequency
	rcall  ValueToDac
	rcall  RPG                   ; Tests whether or not the RPG turned.
	rcall  RPGCounter            ; Updates VALUE according to DELRPG. 
	; rcall  ReadPot               ; Reads the current value of the POT through the A/D converter.
	; rcall  PotToDAC              ; Sends the value of the POT and its complement to the DAC, 
	rcall  LoopTime              ; The remainder of the 10ms is spent in here by using Timer1.
;ENDLOOP
	bra	L1
PL1

;;;;;;; Initial subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine performs all initializations of variables and registers.

Initial
	MOVLF B'11001111', ADCON1      ; Enable PORTA & PORTE digital I/O pins for the A/D converter.
	MOVLF B'01100001', ADCON0      ; Sets up the A/D converter for the POT.
	MOVLF B'11100001', TRISA       ; Set I/O for PORTA. '1' is input while '0' is output.
	MOVLF B'11011100', TRISB       ; Set I/O for PORTB.
	MOVLF B'11010010', TRISC       ; Set I/0 for PORTC.
	MOVLF B'00001111', TRISD       ; Set I/0 for PORTD.
	MOVLF B'00000000', TRISE       ; Set I/0 for PORTE.
	MOVLF B'00010000', PORTA       ; Turn off all four LEDs driven from PORTA.
	MOVLF B'00100000', SSPCON1     ; Sets up the SPI interface for the DAC
	MOVLF B'11000000', SSPSTAT     ;  to use with a 2.5 MHz clock.
; This sets up Timer1 and CCP1.
	MOVLF B'00001000', T3CON       ; Sets up so that T1 is used with CCP1.
	MOVLF B'10000001', T1CON       ; continue setting up T1 to CCP1.
	MOVLF B'00001011', CCP1CON     ; Set up to trigger special event so that PIR1, CCP1IF will be set.
	MOVLF B'01100001', CCPR1H      ; Set the comparator to 25,000.
	MOVLF B'10101000', CCPR1L	
                                       ; Initialize variables for RPG and PButton subroutines.
	movff PORTD, OLDPORTD          ; Initialize "old" value for RPG.
	;clrf RPGCNT                    ; Initialize the RPG counter.

	;MOVLF B'00001000', PBSTATE     ; Initialize pushbutton states.
	clrf OLDBUTTON					; clear button state
	clrf MODE
	clrf VALUE

	MOVLF 0x00, HEXSTR+3           ; Terminating byte for HEXSTR will never change.


	rcall InitLCD                  ; Initialize LCD.
	POINT NAME                     ; Display name on the LCD.
	rcall DisplayC		

	MOVLF D'1', CNT_TITLE	       ; Initialize CNT_TITLE
	
	return
;;;;;;; InitLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Initialize the Optrex 8x2 character LCD.
; First wait for 0.1 second, to get past display's power-on reset time.
InitLCD
	MOVLF 10, COUNT				; Wait 0.1 sec
	;REPEAT
L2
	rcall LoopTime
	decf COUNT, F				; Call LoopTime 10 times == 0.1 Sec
	bnz L2

RL2
	bcf		PORTE, 0			; RS = 0
	POINT	LCDStr				; Set table pointer for initialization string
	tblrd*						; get first byt from string to TABLAT

L3
	bsf		PORTE, 1			; DRIVE PORTE high
	movff	TABLAT, PORTD		; send upper nibble
	bcf		PORTE, 1			; Drive PORTE low so LCD will process input
	rcall	LoopTime			; wait 10ms
	bsf		PORTE, 1			; drive PORTE high
	swapf	TABLAT, W			; swap nibbles
	movwf	PORTD				; send lower nibble
	bcf		PORTE, 1			; drive PORTE low
	rcall 	LoopTime			; wait 10ms
	tblrd+*						; increment pointer and get next byte
	movf	TABLAT, F			; is 0
	;until zero
	bnz L3

RL3
	return

;;;;;;; T40 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Pause for 40 microseconds  or 40/0.4 = 100 clock cycles.
; Assumes 10/4 = 2.5 MHz internal clock rate.

T40
	movlw  100/3                   ; Each REPEAT loop takes 3 cycles.
	movwf  COUNT
	;REPEAT
L4
	decf  COUNT,F
	;UNTIL ZERO
	bnz	L4
RL4
	return

;;;;;;;;DisplayC subroutine;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine is called with TBLPTR containing the address of a constant
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position.
; This subroutine expects a normal one-byte cursor-positioning code, 0xhh, or
; an occasionally used two-byte cursor-positioning code of the form 0x00hh.

DisplayC
	bcf  PORTE,0			; Drive RS pin low for cursor-positioning code.
	tblrd*          		; Get byte from string into TABLAT.
	movf TABLAT,F   		; Check for leading zero byte.
	;IF == 0
	bnz	L5
	tblrd+*         		; If zero, get next byte.
	;ENDIF_
L5
	;repeat
L6
	bsf   PORTE,1                ; Drive E pin high.
	movff TABLAT,PORTD           ; Send upper nibble.
	bcf   PORTE,1                ; Drive E pin low so LCD will accept nibble.
	bsf   PORTE,1                ; Drive E pin high again.
	swapf TABLAT,W               ; Swap nibbles.
	movwf PORTD                  ; Write lower nibble.
	bcf   PORTE,1                ; Drive E pin low so LCD will process byte.
	rcall T40                    ; Wait 40 usec.
	bsf   PORTE,0                ; Drive RS pin high for displayable characters.
	tblrd+*                      ; Increment pointer, then get next byte.
	movf  TABLAT,F               ; Is it zero?
	;Do until zero
	bnz	L6
RL6
   	return

;;;;;;; ButtonPressed subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine detects if button has been pressed, and toggles leds 
; accordingly.
;

Button
	movf	PORTD, W
	andlw	b'00001000'       ;Leaves only the button bit, others <= 0
	cpfseq	OLDBUTTON
	bra	ButtonToggle	  ;If changed, go to PushButtonToggle
	return
	
;;;;;;; PushButtonToggle subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine checks if BP and toggles the mode accordingly
;
;
ButtonToggle
	movwf OLDBUTTON			
	btfsc OLDBUTTON, 3		; find falling edges
	return					; return on rising
	; jos muutos on tapahtunut tulee suoritus tänne
	; logiikka valikolle tulee tähän
	btg	PORTA, RA1	  	;LED D6 is controlled by PORTA,RA1
	decfsz CNT_TITLE	; decrement skip if zero
	bra mode_logic		; mene valikkologiikkaan
	movlw D'4'
	movwf CNT_TITLE

mode_logic
	movlw D'1'
	subwf CNT_TITLE, 0 ; oliko ykkönen
	bz mode_l1	;oli 1
	movlw D'2'
	subwf CNT_TITLE, 0 ; oliko kakkonen
	bz mode_l2 ;oli 2
	movlw D'3'
	subwf CNT_TITLE, 0 ; oliko kolmonen
	bz mode_l3 ;oli 3
	movlw D'4'
	subwf CNT_TITLE, 0 ; oliko nelonen
	bz mode_l4 ;oli 4

mode_l1
	POINT NAME
	MOVLF D'0', MODE
	rcall DisplayC
	bra no_wave
	return
mode_l2
	POINT SIN
	MOVLF D'1', MODE
	rcall DisplayC
	MOVLF 	high Sine_tbl, TBLPTRH	; load sine table
	MOVLF 	low Sine_tbl, TBLPTRL
	MOVLF 	255, TABL_COUNT				; load counter
	return		
mode_l3
	POINT SQARE
	MOVLF D'2', MODE
	rcall DisplayC
	MOVLF 	high Sqr_tbl, TBLPTRH	; load sine table
	MOVLF 	low Sqr_tbl, TBLPTRL
	MOVLF 	255, TABL_COUNT				; load counter
	return	
mode_l4
	POINT	TRIANG
	MOVLF	D'3', MODE
	rcall	DisplayC
	MOVLF	high Triangle_tbl, TBLPTRH	; load triangle table
	MOVLF	low Triangle_tbl, TBLPTRL
	MOVLF	255, TABL_COUNT
	return

; Mode == 0
no_wave
	return

;sine_wave
sine_wave
	MOVLF	1, W
	cpfseq	MODE			; compare if the mode is correct
	return					; otherwise return
	tblrd*+
	movf	TABLAT, W
	movwf	OUTPUT
	movff 	VALUE, TEMP
	
	tblrd*+
	;mulwf	VALUE			; multiply value (amplitude) with sine table_value
	decfsz	TABL_COUNT	; skip if zero to load the table again (must fetch the first value of table)
	;UNTIL ZERO

	return
	; load the table again
	MOVLF 	high Sine_tbl, TBLPTRH	; load sine table
	MOVLF 	low Sine_tbl, TBLPTRL
	MOVLF 	255, TABL_COUNT				; load counter
	return

;trianle wave
triang_wave
	MOVLF	3, W
	cpfseq	MODE			; compare if the mode is correct
	return					; otherwise return
	tblrd*+
	movf	TABLAT, W
	movwf	OUTPUT
	movff 	VALUE, TEMP
	tblrd*+
	;mulwf	VALUE			; multiply value (amplitude) with sine table_value
	decfsz	TABL_COUNT, F	; skip if zero to load the table again (must fetch the first value of table)
	;UNTIL ZERO
	return
	; load the table again
	MOVLF 	high Triangle_tbl, TBLPTRH	; load sine table
	MOVLF 	low Triangle_tbl, TBLPTRL
	MOVLF 	255, TABL_COUNT				; load counter
	return

;square wave
; This subroutine checks if the mode is correct and reads value from table
square_wave
	MOVLF	2, W
	cpfseq	MODE			; compare if the mode is correct
	return					; otherwise return
	tblrd*+
	movf	TABLAT, W
	movwf	OUTPUT
	movff 	VALUE, TEMP	
	tblrd*+
	;mulwf	VALUE			; multiply value (amplitude) with sine table_value
	decfsz	TABL_COUNT, F	; skip if zero to load the table again (must fetch the first value of table)
	;UNTIL ZERO
	return
	; load the table again
	MOVLF 	high Sqr_tbl, TBLPTRH	; load sine table
	MOVLF 	low Sqr_tbl, TBLPTRL
	MOVLF 	255, TABL_COUNT			; load counter
	return


; AdjustFrequency
AdjustFrequency
	movf 	VALUE, W
	subwf	CCPR1H
	return
	;MOVLF	0, W
	;cpfseq	TEMP
	;bra DIV
	;return

;DIV
	;decf TEMP
	;rrncf OUTPUT
	;bra AdjustAmplitude

; value to dac
ValueToDac
	                                  ; Instructions dealing with DAC.
        bcf   PORTC,RC0                ; Clear PORTC,RC0 to change DAC output.
        MOVLF 0x21,BYTE                ; Load control byte 0x21 to BYTE for the output to go to DAC-A.
        rcall SPItransfer              ; Transfer what is in BYTE.
        movff OUTPUT,BYTE            	; Load the byte to be converted.
        rcall SPItransfer                      
        bsf   PORTC,RC0                ; Set RC0 to finish transfer.
		return	


;;;;;;; SPItransfer subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine controls the SPI.

SPItransfer
        bcf    PIR1,SSPIF              ; Clear PIR1,SSPIF to ready for transfer. 
        movff  BYTE,SSPBUF             ; Initiates write when anything is placed in SSPBUF.
        ;REPEAT_                       ; Wait until transfer is finished.
L42
        ;UNTIL_ PIR1,SSPIF == 1
        btfss PIR1,SSPIF
        bra	L42
RL42
        movff  SSPBUF,SPIRECEIVE       ; If result is desired, it is stored in SPIRECEIVE.
        return
	
;;;;;;; RPG subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine decyphers RPG changes into values of DELRPG of 0, +1, or -1.
; DELRPG = +1 for CW change, 0 for no change, and -1 for CCW change.
; (demo4.asm used for reference)

RPG
	movf   RPGCNT,w		       ; Copy RPGCNT to W.
	btfss  STATUS, Z	       ; skip if zero
	decf   RPGCNT		       ; if not zero, decrement
	bcf    PORTA,RA3
        clrf   DELRPG                  ; Clear for "no change" return value.
        movf   PORTD,W                 ; Copy PORTD into W.
        movwf  TEMP                    ;  and TEMP.
        xorwf  OLDPORTD,W              ; Any change?
        andlw  B'00000011'             ; If not, set the Z flag.
        ;IF_  .NZ.                     ; If the two bits have changed then...
        bz	L8
          rrcf OLDPORTD,W              ; Form what a CCW change would produce.
          ;IF_  .C.                    ; Make new bit 1 = complement of old bit 0.
          bnc	L9
            bcf  WREG,1
          ;ELSE_
          bra	L10
L9
            bsf  WREG,1
          ;ENDIF_
L10
          xorwf  TEMP,W                ; Did the RPG actually change to this output?
          andlw  B'00000011'
          ;IF_  .Z.                    ; If so, then change  DELRPG to -1 for CCW.
          bnz	L11
            decf DELRPG,F
          ;ELSE_                       ; Otherwise, change DELRPG to  +1 for CW.
          bra	L12
L11
            incf DELRPG,F
          ;ENDIF_
L12
        ;ENDIF_
	movf   RPGCNT,w		       ; Copy RPGCNT to W.
	btfsc  STATUS, Z	       ; skip if not zero
	MOVLF	RGPTresh, RPGCNT       ; set RGPCNT to treshold		
L8
        movff  TEMP,OLDPORTD           ; Save PORTD as OLDPORTD for ten ms from now.
        return

;;;;;;; RPGCounter subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Updates the VALUE according to DELRPG
RPGCounter
	movf	DELRPG, W
	addwf	VALUE			; add DELRPG and VALUE
	MOVLF	0xC6, HEXSTR	; cursor position byte
	movff	VALUE, BYTE		; load result to BYTE
	rcall	HexDisplay
	return
;;;;;; HexDisplay subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine takes in BYTE and converts into an ASCII string in hex and stores it into
; HEXSTR for display.

HexDisplay
        rcall ConvertLowerNibble       ; Converts lower nibble of BYTE to hex and stores it in W.
        movwf HEXSTR+2                 ; Move the ASCII value into the most significant byte of HEXSTR.

        swapf BYTE,F                   ; Swap nibbles of BYTE to convert the upper nibble.

        rcall ConvertLowerNibble       ; Convert the old upper nibble of BYTE to hex and stores it in W.
        movwf HEXSTR+1                 ; Move the ASCII value into the 2nd byte of HEXSTR.

        lfsr  0, HEXSTR                ; Loads address of HEXSTR to fsr0 to display HEXSTR.
        rcall DisplayV                 ; Call DisplayV to display HEXSTR on LCD.
        return

;;;;;; ConvertLowerNibble subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine takes lower nibble of BYTE and converts it into its ASCII hex value.

ConvertLowerNibble                              
        movf BYTE, W                   ; Loads BYTE into W.
        andlw B'00001111'              ; Masks out the upper nibble.
        sublw 0x09                     ; Test if it's greater than 9. 
        ;IF_ .N.                       ; If, after masking, it is greater than 9, then it is a letter.
        bnn	L30
          movf BYTE,W                  ; Load BYTE into W.
          andlw B'00001111'            ; Mask out the upper nibble.
          addlw 0x37                   ; Add offset to obtain the letter's ASCII value.
        ;ELSE_                         ; If it's less than 9, then it's a number.
        bra	L31
L30
          movf BYTE,W
          andlw B'00001111'
          iorlw 0x30                   ; Then add offset of 30 to obtain the numeric's ASCII value.
        ;ENDIF_
L31
        return

;;;;;;; DisplayV subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine is called with FSR0 containing the address of a variable
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position.
	
DisplayV
        bcf  PORTE,0                   ; Drive RS pin low for cursor positioning code.
        ;REPEAT_
L7
          bsf   PORTE,1                ; Drive E pin high.
          movff INDF0,PORTD            ; Send upper nibble.
          bcf   PORTE,1                ; Drive E pin low so LCD will accept nibble.
          bsf   PORTE,1                ; Drive E pin high again.
          swapf INDF0,W                ; Swap nibbles.
          movwf PORTD                  ; Write lower nibble.
          bcf   PORTE,1                ; Drive E pin low so LCD will process byte.
          rcall T40                    ; Wait 40 usec.
          bsf   PORTE,0                ; Drive RS pin high for displayable characters.
          movf  PREINC0,W              ; Increment pointer, then get next byte.
        ;UNTIL_  .Z.                   ; Is it zero?
        bnz	L7
RL7
        return
;;;;;;; BlinkAlive subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine briefly blinks the LED next to the PIC every two-and-a-half
; seconds.

BlinkAlive
	bsf  PORTA,RA4					; Turn off LED.
	decf ALIVECNT,F					; Decrement loop counter and return if not zero.
	;IF == 0
	bnz	BlinkAliveEnd
	MOVLF 100,ALIVECNT				; Reinitialize BLNKCNT.
	bcf   PORTA,RA4              	; Turn on LED for ten milliseconds every 2.5 sec.
	;ENDIF
BlinkAliveEnd
	return

;;;;;;; LoopTime subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;Gives a 10ms looptime by using Timer1 and comparing it to 25,000.

LoopTime
	; Repeat until TIMER1 has reached 25,000. 
L46
	;UNTIL PIR1, CCP1IF == 1	       
	btfss PIR1,CCP1IF
	bra	L46
RL46
	bcf    PIR1, CCP1IF            ; When it has, reset it to start over.
	return

;;;;;;; Constant strings ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

LCDStr  db  0x33,0x32,0x28,0x01,0x0c,0x06,0x00  ; Initialization string for LCD.
NAME	db  0x80,'I','D','L','E',' ',' ',0x00
SIN		db  0x80,'S','I','N','E',' ',' ',0x00   ; Declaration of name string on LCD (max 6 chars).
SQARE  	db  0x80,'S','Q','U','A','R','E',0x00	; All strings must be of same lenght, 
TRIANG  db  0x80,'T','R','I','A','N','G',0x00	; else the last letters of previous line

		; 256 step sinewave table
Sine_tbl:
	dt	0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae
	dt	0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8
	dt	0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,0xea,0xec,0xed,0xef,0xf0,0xf2,0xf3,0xf5
	dt	0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfc,0xfd,0xfe,0xfe,0xff,0xff,0xff,0xff,0xff
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0xfe,0xfd,0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7
	dt	0xf6,0xf5,0xf3,0xf2,0xf0,0xef,0xed,0xec,0xea,0xe8,0xe6,0xe4,0xe2,0xe0,0xde,0xdc
	dt	0xda,0xd8,0xd5,0xd3,0xd1,0xce,0xcc,0xc9,0xc7,0xc4,0xc1,0xbf,0xbc,0xb9,0xb6,0xb3
	dt	0xb0,0xae,0xab,0xa8,0xa5,0xa2,0x9f,0x9c,0x98,0x95,0x92,0x8f,0x8c,0x89,0x86,0x83
	dt	0x80,0x7c,0x79,0x76,0x73,0x70,0x6d,0x6a,0x67,0x63,0x60,0x5d,0x5a,0x57,0x54,0x51
	dt	0x4f,0x4c,0x49,0x46,0x43,0x40,0x3e,0x3b,0x38,0x36,0x33,0x31,0x2e,0x2c,0x2a,0x27
	dt	0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,0x15,0x13,0x12,0x10,0x0f,0x0d,0x0c,0x0a
	dt	0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x03,0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x02,0x03,0x03,0x04,0x05,0x06,0x07,0x08
	dt	0x09,0x0a,0x0c,0x0d,0x0f,0x10,0x12,0x13,0x15,0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23
	dt	0x25,0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,0x38,0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c
	dt	0x4f,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,0x67,0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c

Triangle_tbl:	; 256 step trianglewave table
	dt	0x00,0x02,0x04,0x06,0x08,0x0a,0x0c,0x0e,0x10,0x12,0x14,0x16,0x18,0x1a,0x1c,0x1e
	dt	0x20,0x22,0x24,0x26,0x28,0x2a,0x2c,0x2e,0x30,0x32,0x34,0x36,0x38,0x3a,0x3c,0x3e
	dt	0x40,0x42,0x44,0x46,0x48,0x4a,0x4c,0x4e,0x50,0x52,0x54,0x56,0x58,0x5a,0x5c,0x5e
	dt	0x60,0x62,0x64,0x66,0x68,0x6a,0x6c,0x6e,0x70,0x72,0x74,0x76,0x78,0x7a,0x7c,0x7e
	dt	0x80,0x82,0x84,0x86,0x88,0x8a,0x8c,0x8e,0x90,0x92,0x94,0x96,0x98,0x9a,0x9c,0x9e
	dt	0xa0,0xa2,0xa4,0xa6,0xa8,0xaa,0xac,0xae,0xb0,0xb2,0xb4,0xb6,0xb8,0xba,0xbc,0xbe
	dt	0xc0,0xc2,0xc4,0xc6,0xc8,0xca,0xcc,0xce,0xd0,0xd2,0xd4,0xd6,0xd8,0xda,0xdc,0xde
	dt	0xe0,0xe2,0xe4,0xe6,0xe8,0xea,0xec,0xee,0xf0,0xf2,0xf4,0xf6,0xf8,0xfa,0xfc,0xfe
	dt	0xff,0xfd,0xfb,0xf9,0xf7,0xf5,0xf3,0xf1,0xef,0xef,0xeb,0xe9,0xe7,0xe5,0xe3,0xe1
	dt	0xdf,0xdd,0xdb,0xd9,0xd7,0xd5,0xd3,0xd1,0xcf,0xcf,0xcb,0xc9,0xc7,0xc5,0xc3,0xc1
	dt	0xbf,0xbd,0xbb,0xb9,0xb7,0xb5,0xb3,0xb1,0xaf,0xaf,0xab,0xa9,0xa7,0xa5,0xa3,0xa1
	dt	0x9f,0x9d,0x9b,0x99,0x97,0x95,0x93,0x91,0x8f,0x8f,0x8b,0x89,0x87,0x85,0x83,0x81
	dt	0x7f,0x7d,0x7b,0x79,0x77,0x75,0x73,0x71,0x6f,0x6f,0x6b,0x69,0x67,0x65,0x63,0x61
	dt	0x5f,0x5d,0x5b,0x59,0x57,0x55,0x53,0x51,0x4f,0x4f,0x4b,0x49,0x47,0x45,0x43,0x41
	dt	0x3f,0x3d,0x3b,0x39,0x37,0x35,0x33,0x31,0x2f,0x2f,0x2b,0x29,0x27,0x25,0x23,0x21
	dt	0x1f,0x1d,0x1b,0x19,0x17,0x15,0x13,0x11,0x0f,0x0f,0x0b,0x09,0x07,0x05,0x03,0x01

Sqr_tbl:		; 256 step squarewave table
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	dt	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
	dt	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff

#include c:\math18\FXD2416U.INC
#include c:\math18\FXD0808U.INC
#include c:\math18\FXM1608U.INC

end