;Archivo: Lab05.s
;Dispositivo: PIC16F887
;Autor: Rodrigo García
;Carné: 20178
;Programa: contador, multiplexado
;Hardware: LEDs, Display 
;
;Creado: 22 febrero, 2023


processor 16F887
#include <xc.inc>
 
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF             ; Watchdog Timer Enable bit (WDT enabled)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF            ; Brown Out Reset Selection bits (BOR enabled)
  CONFIG  IESO = OFF             ; Internal External Switchover bit (Internal/External Switchover mode is enabled)
  CONFIG  FCMEN = OFF            ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
  CONFIG  LVP = OFF             ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

//----------------------------------MACROS---------------------------------------------

  RESET_TMR0 macro
    BANKSEL TMR0        ; Banco TMR0
    movlw 100           ; Cargar valor inicial a W
    movwf TMR0          ; Cargar el valor inicial al TIMER0
    bcf T0IF            ; Limpiar la bandera  de overflow del TIMER0
    endm
  
//--------------------------VARIABLES EN MEMORIA--------------------------------
PSECT udata_shr			
    W_TEMP:		DS 1	
    STATUS_TEMP:	DS 1	
    
    Valor:              DS 1   
    Flags:              DS 1
    Nibbles:            DS 2
    Valor_Display:      DS 2
  
 //-----------------------------VECTOR RESET------------------------------------
 PSECT resVect, class = CODE, abs, delta = 2;
 ORG 00h			; Posición 0000h RESET
 resetVec:			; Etiqueta para el vector de reset
    PAGESEL main
    goto main
  
 PSECT intVect, class = CODE, abs, delta = 2, abs
 ORG 04h			; Posición de la interrupción
 
//--------------------------INTERRUPCIONES------------------------------- 
PUSH:
    movwf   W_TEMP		; 
    swapf   STATUS, W		; intercambiar status con registro w
    movwf   STATUS_TEMP		; cargar valor a la variable temporal
    
ISR: 
    btfsc   RBIF		; interrupción PORTB, SI=1 NO=0
    call    INT_IOCB		; si es igual a 1, ejecutar interrupción
    btfsc   T0IF
    call    INT_TMR0
    
POP:
    swapf   STATUS_TEMP, W	
    movwf   STATUS		
    swapf   W_TEMP, F		
    swapf   W_TEMP, W		
    retfie	
//----------------------------INTERRUPCIONES SUBRUTINAS------------------------------------    
INT_IOCB:			
    BANKSEL PORTB		
    btfss   PORTB, 0		; Revisar si el primer botón ha cambiado a 0
    incf    PORTA		; Si cambió a 0, incrementar el valor del puerto A
    btfss   PORTB, 1		; Revisar si el segundo botón ha cambiado a 0
    decf    PORTA		; Si cambió a 0, disminuir el valor del puerto A
    bcf	    RBIF		; Limpiar bandera del puerto B
    return 
    
INT_TMR0:
    call SELECT_DISPLAY         ;Se llama a la subrutina de selección de display
    RESET_TMR0                  ;Reinicio del TMR0
    return

SELECT_DISPLAY:
    bcf   PORTD, 0	    ;Colocar el puertoD0 como salida
    bcf   PORTD, 1          ;Colocar el puertoD1 como salida
    btfsc Flags, 0          ;Revisar si la bandera esta en cero
    goto  DISPLAY_1         ;Ir a la subrutina del primer display
    goto  DISPLAY_2         ;Ir a la subrutina del segundo display
    return

DISPLAY_1:
    movf   Valor_Display, W   ;Se mueve el valor del display a W
    movwf  PORTC              ;Se mueve el valor al puerto C
    bsf    PORTD, 1           ;Se enciende el puerto D1 para encender dicho display
    goto   SWITCH             ;Ir a la subrutina del switch
    return
    
DISPLAY_2:
    movf   Valor_Display+1, W  ;Se mueve el valor del display a W
    movwf  PORTC               ;Se mueve el valor al puerto C
    bsf    PORTD, 0            ;Se enciende el puerto D0 para encender dicho display
    goto   SWITCH              ;Ir a la subrutina del switch
    return
    
SWITCH:
    movlw  0x01      ;Se mueve este valor al acumulador
    xorwf  Flags, F  ;Se realiza un XOR entre el valor guardado y la bandera
    return
 //---------------------------TABLA DEL DISPLAY--------------------------------
PSECT Tabla, class = CODE, abs, delta = 2
ORG 200h			; Posición de la tabla

Tabla:
    clrf PCLATH
    bsf PCLATH, 1	
    andlw 0x0F
    addwf PCL			
    retlw 00111111B		; 0
    retlw 00000110B		; 1
    retlw 01011011B		; 2
    retlw 01001111B		; 3
    retlw 01100110B		; 4
    retlw 01101101B		; 5
    retlw 01111101B		; 6
    retlw 00000111B		; 7
    retlw 01111111B		; 8 
    retlw 01101111B		; 9
    retlw 01110111B		; A
    retlw 01111100B		; b
    retlw 00111001B		; C
    retlw 01011110B		; d
    retlw 01111001B		; E
    retlw 01110001B		; F
    
PSECT code, delta=2, abs
ORG 100h			; Posición 100h para el codigo    
    
//------------------------------MAIN-------------------------------------
main:
    call    IO_CONFIG		; Configuración de pines
    call    TMR0_CONFIG		; Configuriación TMR0
    call    CLK_CONFIG   	; Configuración de reloj
    call    IOCRB_CONFIG	; Configuración del puerto b
    call    INT_CONFIG		; Configuración de interrupciones
    BANKSEL PORTA
//--------------------------LOOP-----------------------------------------------
loop:
    movf    PORTA, W            ;Mover valor del puertoA a W
    movwf   Valor               ;Mover el valor W a variable "valor"
    call    SEPARAR_NIBBLES     ;Llamar subrutina para separa nibbles
    call    DISPLAY             ;Llamar subrutina del display
    goto loop

//------------------------------SUBRUTINAS--------------------------------------
IOCRB_CONFIG:
    BANKSEL IOCB		; Seleccionar banco del IOCB
    bsf	    IOCB, 0		; Activar IOCB UP
    bsf	    IOCB, 1		; Activar IOCB DOWN
    
    BANKSEL PORTA		; Seleccionar banco 00
    movf    PORTB, W		; Cargar el valor del puerto B a w
    bcf	    RBIF		; Limpiar bandera de interrupción del puerto B
    return

IO_CONFIG:
    BANKSEL ANSEL		
    clrf    ANSEL		
    clrf    ANSELH		; Colocar puertos en digital
    
    BANKSEL TRISA		; Seleccionar banco 01
    //LEDS
    clrf    TRISA		; Colocar pines del puerto A como salida 
    //DISPLAYS
    clrf    TRISC		; Colocar pines del puerto C como salida
    //CAMBIO DE DISPLAYS
    clrf    TRISD               ; Colocar pines del puerto D como salida
    //PUSHBUTTONS
    bsf	    TRISB, 0		; Colocar pines del puerto B como entrada
    bsf	    TRISB, 1		
    
    
    bcf	    OPTION_REG, 7	; Limpiar RBPU para colocar Pull-Up en el puerto B
    
    BANKSEL PORTA
    clrf   PORTA
    clrf   PORTC
    clrf   PORTD
    clrf   Flags
    return	
    
CLK_CONFIG:
    BANKSEL OSCCON		; Configuración de oscilador
    bsf	    SCS			; Usar oscilador interno
    bcf	    IRCF0		
    bsf	    IRCF1		;Oscilador a 250 kHz
    bcf	    IRCF2		
    return
    
INT_CONFIG:
    BANKSEL INTCON
    bsf GIE			; Activar interrupciones
    bsf RBIE			; Activar cambio de interrupciones en portB
    bcf RBIF			; Limpiar bandera de cambio del portB
    
    bsf T0IE
    bcf T0IF
    return

TMR0_CONFIG:
    BANKSEL TRISA
    bcf     T0CS              ;Utilizar reloj interno
    bcf     PSA
    bcf     PS2
    bcf     PS1
    bsf     PS0               ;Prescaler 1:4      
    
    RESET_TMR0
    return
    
SEPARAR_NIBBLES:
    movf    Valor, W    ;Mover variable a W
    andlw   00001111B   ;Se le hace un AND al valor de W con el valor 0X0F
    movwf   Nibbles     ;Se mueve el valor a la variable Nibbles
    swapf   Valor, W    ;Se cambia de posición el nibble
    andlw   00001111B   ;Se le hace un AND al valor de W con el valor 0X0F
    movwf   Nibbles+1   ;Se mueve el valor a la variable Nibbles+1
    return
    
DISPLAY:
    movf   Nibbles, W
    call   Tabla
    movwf  Valor_Display
    
    movf   Nibbles+1, W
    call   Tabla
    movwf  Valor_Display+1
    return
    
end