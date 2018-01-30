;******************************************************************************
;   This file is a basic template for assembly code for a PIC18F4550. Copy    *
;   this file into your project directory and modify or add to it as needed.  *
;                                                                             *
;   The PIC18FXXXX architecture allows two interrupt configurations. This     *
;   template code is written for priority interrupt levels and the IPEN bit   *
;   in the RCON register must be set to enable priority levels. If IPEN is    *
;   left in its default zero state, only the interrupt vector at 0x008 will   *
;   be used and the WREG_TEMP, BSR_TEMP and STATUS_TEMP variables will not    *
;   be needed.                                                                *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on the         *
;   features of the assembler.                                                *
;                                                                             *
;   Refer to the PIC18FXX50/XX55 Data Sheet for additional                    *
;   information on the architecture and instruction set.                      *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:    Practica 6                                                *
;    Date:        06/09/15                                                    *
;    File Version: 1.0                                                        *
;                                                                             *
;    Author:   Jose Luis Bravo Leon
;                             *
;    Company:   Acad. Computación ICE - ESIME Zac.                            *
;                                                                             * 
;******************************************************************************
;                                                                             *
;    Files required: P18F4550.INC                                             *
;                                                                             *
;******************************************************************************

	LIST P=18F4550, F=INHX32	;directive to define processor
	#include <P18F4550.INC>		;processor specific variable definitions

;******************************************************************************
;Configuration bits

    CONFIG PLLDIV   = 5         ;(20 MHz crystal on PICDEM FS USB board)
    CONFIG CPUDIV   = OSC1_PLL2	
    CONFIG USBDIV   = 2         ;Clock source from 96MHz PLL/2
    CONFIG FOSC     = HSPLL_HS
    CONFIG FCMEN    = OFF
    CONFIG IESO     = OFF
    CONFIG PWRT     = OFF
    CONFIG BOR      = ON
    CONFIG BORV     = 3
    CONFIG VREGEN   = ON	;USB Voltage Regulator
    config WDT      = OFF
    config WDTPS    = 32768
    config MCLRE    = ON
    config LPT1OSC  = OFF
    config PBADEN   = OFF	;NOTE: modifying this value here won't have an effect
        			;on the application.  See the top of the main() function.
        			;By default the RB4 I/O pin is used to detect if the
        			;firmware should enter the bootloader or the main application
        			;firmware after a reset.  In order to do this, it needs to
        			;configure RB4 as a digital input, thereby changing it from
        			;the reset value according to this configuration bit.
    config CCP2MX   = ON
    config STVREN   = ON
    config LVP      = OFF
    config ICPRT    = OFF      	; Dedicated In-Circuit Debug/Programming
    config XINST    = OFF      	; Extended Instruction Set
    config CP0      = OFF
    config CP1      = OFF
    config CP2      = OFF
    config CP3      = OFF
    config CPB      = OFF
    config CPD      = OFF
    config WRT0     = OFF
    config WRT1     = OFF
    config WRT2     = OFF
    config WRT3     = OFF
    config WRTB     = OFF      	; Boot Block Write Protection
    config WRTC     = OFF
    config WRTD     = OFF
    config EBTR0    = OFF
    config EBTR1    = OFF
    config EBTR2    = OFF
    config EBTR3    = OFF
    config EBTRB    = OFF
;******************************************************************************
; Definicion de variables


RH 		equ 0x00		; registros para la funcion PWM
RL		equ 0x01		; 
Rtmrh 	equ 0x02		; registros para el TMR0
Rtmrl	equ 0x03		; 
RUn		equ 0x04		; registros para despliegue en BCD
RDec	equ 0x05
COUNTER EQU	0X10

;******************************************************************************

;Reset vector

;*****************************************************************************
RESET_VECTOR	ORG	0
		goto	_Reset		;go to start of user program
;******************************************************************************
HIGHPRIORITY_VECTOR	ORG	0x0008
		goto	HighPr_ISR	;go to start of ISR
;****************************************************************************** 
LOWPRIORITY_VECTOR	ORG	0x0018
		goto	LowPr_ISR	;go to start of ISR		

;******************************************************************************
;Start of Reset in Bootloader.
;******************************************************************************
		ORG	0x1000
_Reset	goto	INICIO
;******************************************************************************
; Start of High Priority ISR
;******************************************************************************
		ORG 0x1008
HighPr_ISR
		goto INT


;******************************************************************************
; Start of Low Priority ISR
;******************************************************************************
		ORG 0x1018
LowPr_ISR
		
		retfie
;*****************************************************************************
; INICIO DEL PROGRAMA PRINCIPAL
;*****************************************************************************

INICIO:  
		CLRF	COUNTER
		CALL 	CPUERTOS
		MOVLW	0XA0
		MOVWF	INTCON
		CALL 	CTMR0
ET1:	NOP
		GOTO	ET1
			
	
					; FIN DEL PROGRAMA PRINCIPAL
;******************************************************************************
; SECCION DE SUBRUTINAS
;******************************************************************************
CPUERTOS:
		MOVLW	0X0F
		MOVWF	ADCON1
		CLRF	TRISA
		CLRF	TRISD	
		return


CTMR0:
		MOVLW	0XA0
		MOVWF	INTCON
		MOVLW	0X44
		MOVWF	TMR0H
		MOVLW	0X80	
		MOVWF	TMR0L
		MOVWF	T0CON
		RETURN


; *****************************************
;Rutina de servicio de interrupcion
INT:
		BTFSC	INTCON,1
		GOTO	RSINT0
		BTFSC	INTCON,2
		GOTO	RSTMR0
		RETFIE

RSTMR0:
		BTG		PORTA,1
		MOVLW	0X44
		MOVWF	TMR0H
		MOVLW	0X80
		MOVWF	TMR0L
		BCF		INTCON,2	
		retfie	

RSINT0:
		INCF 	COUNTER
		MOVLW	0X0A
		CPFSEQ	COUNTER
		GOTO 	ETQ2
		INCF	PORTD,1
		CLRF	COUNTER
ETQ2:	BCF		INTCON,1
		RETFIE

;******************************************************************************
;End of program
	END
