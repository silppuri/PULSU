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
	rcall  Button            ; Checks to see if SW3 is pushed or not.
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
	;MOVLF B'00100000', SSPCON1     ; Sets up the SPI interface for the DAC
	;MOVLF B'11000000', SSPSTAT     ;  to use with a 2.5 MHz clock.
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
	bra no_wave
mode_l2
	POINT SIN
	MOVLF D'1', MODE
	bra sine_wave		
mode_l3
	POINT SQARE
	MOVLF D'2', MODE
	bra square_wave
mode_l4
	POINT TRIANG
	MOVLF D'3', MODE
	bra triang_wave
no_wave
	rcall DisplayC
	return

;sine_wave
sine_wave
	rcall DisplayC
	return

;trianle wave
triang_wave
	rcall DisplayC
	return

;square wave
square_wave
	rcall DisplayC
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
	bra    CNTUPDATE	       ; 	
	bsf    PORTA,RA3	       ; Flash D4, for a fast turn of the RPG.
	bcf    STATUS,C		       ; Shift value 3 times to left = multiply by 8
	rlcf   DELRPG		       ; Carry is forced to 0.
	bcf    STATUS,C
	rlcf   DELRPG 
	bcf    STATUS,C
	rlcf   DELRPG 		       ; *****

CNTUPDATE
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
	MOVLF	0x86, HEXSTR	; cursor position byte
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
NAME	db  0x80,'P','U','L','S','U',' ',0x00
SIN	db  0x80,'S','I','N',' ',' ',' ',0x00   ; Declaration of name string on LCD (max 6 chars).
SQARE  	db  0x80,'S','Q','U','A','R','E',0x00	; All strings must be of same lenght, 
TRIANG  db  0x80,'T','R','I','A','N','G',0x00	; else the last letters of previous line

#include c:\math18\FXD2416U.INC
#include c:\math18\FXD0808U.INC
#include c:\math18\FXM1608U.INC

end