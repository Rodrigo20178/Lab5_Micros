;Archivo: PostLab05.s
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

PROCESSOR 16F887
#include <xc.inc>
  

PSECT udata_bank0
    Valor:	    DS 1
    Centenas:	    DS 1
    Decenas:	    DS 1
    Unidades:	    DS 1
    Flags:          DS 1
    Valor_Display:  DS 3
   

//----------------------------------MACROS---------------------------------------------
  
   RESET_TMR0 macro
    BANKSEL TMR0        ; Banco TMR0
    movlw 100           ; Cargar valor inicial a W
    movwf TMR0          ; Cargar el valor inicial al TIMER0
    bcf T0IF            ; Limpiar la bandera  de overflow del TIMER0
    endm
//--------------------------VARIABLES EN MEMORIA--------------------------------       
PSECT udata_shr			         
    W_TEMP:	            DS 1	
    STATUS_TEMP:	    DS 1	
   
     
PSECT resVect, class=CODE, abs, delta=2	
ORG 00h	                              ; Posición 0000h para el reset
//-----------------------------VECTOR RESET------------------------------------
			
resVect:
    PAGESEL	main		
    goto	main

PSECT intVect, class=CODE, abs, delta=2 
ORG 04h
//--------------------------INTERRUPCIONES-------------------------------
			
PUSH:
    movwf   W_TEMP		 
    swapf   STATUS, W		; Intercambiar status con registro w
    movwf   STATUS_TEMP		; Cargar valor a la variable temporal
    
ISR: 
    btfsc   RBIF		; Interrupción PORTB, SI=1 NO=0
    call    INT_IOCB		; Si es igual a 1, ejecutar interrupción
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
    call    SELECT_DISPLAY    ; Se llama a la subrutina de selección de display
    RESET_TMR0                ; Reinicio del TMR0
    return
    
SELECT_DISPLAY:
    bcf	    PORTD, 0        ;Colocar el PORTD como salida
    bcf	    PORTD, 1		
    bcf	    PORTD, 2	
    btfsc   Flags, 0	    ; Verificar que la bandera del display (Centenas) este apagada
    goto    DISPLAY_3       ; Si está encendida, ir a la subrutina DISPLAY_3
    btfsc   Flags, 1	    ; Verificar que la bandera del display (Decenas) este apagada
    goto    DISPLAY_2       ; Si está encendida, ir a la subrutina DISPLAY_2
    btfsc   Flags, 2	    ; Verificar que la bandera del display (Unidades) este apagada
    goto    DISPLAY_1       ; Si está encendida, ir a la subrutina DISPLAY_1
   
DISPLAY_1:
    movf    Valor_Display, W     ; Se mueve valor de las unidades a W
    movwf   PORTC		 ; Se mueve el valor al display
    bsf	    PORTD, 2		 ; Se enciende set-display de unidades
    bcf	    Flags, 2		 ; Se apaga la bandera de unidades
    bsf	    Flags, 1		 ; Se enciende la bandera de decenas
    
    return

DISPLAY_2:
    movf    Valor_Display+1, W	; Se mueve valor de las decenas a W
    movwf   PORTC		; Se mueve el valor al display
    bsf	    PORTD, 1		; Se enciende set-display de decenas
    bcf	    Flags, 1	        ; Se apaga la bandera de decenas
    bsf	    Flags, 0		; Se enciende la bandera de centenas
 
    return
    
DISPLAY_3:
    movf    Valor_Display+2, W	; Se mueve valor de las centenas a W
    movwf   PORTC		; Se mueve el valor al display
    bsf	    PORTD, 0		; Se enciende set-display de centenas
    bcf	    Flags, 0		; Se apaga la bandera de centena
    bsf	    Flags, 2	        ; Se enciende la bandera de unidades
    
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
//------------------------------MAIN-------------------------------------
PSECT code, abs, delta=2  
ORG 100h
main:
    call    IO_CONFIG		; Configuración de pines
    call    TMR0_CONFIG		; Configuriación TMR0
    call    CLK_CONFIG   	; Configuración de reloj
    call    IOCRB_CONFIG	; Configuración del puerto b
    call    INT_CONFIG		; Configuración de interrupciones	   
    BANKSEL PORTA 
//--------------------------LOOP----------------------------------------------- 
loop:
    call    DISPLAY	        ; Llamar subrutina del display
    call    CENTENAS_CONFIG     ; Llamar subrutina para obtener las centenas/decenas/unidades
    goto    loop	       	    
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
    clrf    ANSEL	; I/O digitales
    clrf    ANSELH	; I/O digitales
    
    BANKSEL TRISA	; Banco 01
    //LEDS
    clrf    TRISA	; Colocar pines del PORTA como salida
    //DISPLAYS
    clrf    TRISC	; Colocar pines del PORTC como salida
    //CAMBIO DE DISPLAYS
    clrf    TRISD	; Colocar pines del PORTD como salida
    
    bsf	    TRISB, 0	; Colocar pines del PORTB como entrada
    bsf	    TRISB, 1	
    
    bcf	    OPTION_REG, 7   ; Limpiar RBPU para colocar Pull-Up en el puerto B 
    
    BANKSEL PORTA	; Banco 00
    clrf    PORTA	; Limpiar el PORTA
    clrf    PORTB	; Limpiar el PORTB
    clrf    PORTC	; Limpiar el PORTC
    clrf    PORTD	; Limpiar el PORTD
    clrf    Centenas	; Limpiar variable Centenas
    clrf    Decenas	; Limpiar variable Decenas
    clrf    Unidades    ; Limpiar variable Unidades
    clrf    Flags	; Limpiar variable Flags
    
    return    
    
CLK_CONFIG:
    BANKSEL OSCCON		; Configuración de oscilador
    bsf	    SCS			; Usar oscilador interno
    bcf	    IRCF0		
    bsf	    IRCF1		; Oscilador a 250 kHz
    bcf	    IRCF2		
    return

INT_CONFIG:
  BANKSEL INTCON
    bsf GIE			; Activar interrupciones
    bsf RBIE			; Activar cambio de interrupciones en portB
    bcf RBIF			; Limpiar bandera de cambio del portB
    
    bsf	T0IE			; Activar interrupciones en timer0
    bcf	T0IF			; Limpiar bandera del timer0
    
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
     
CENTENAS_CONFIG:
    clrf    Centenas		 ; Se limpia variable Centenas
    clrf    Decenas		 ; Se limpia variable Decenas
    clrf    Unidades		 ; Se limpia variable Unidades
    
    movf    PORTA, W		; Se mueve el valor de PORTA a W
    movwf   Valor		; Se mueve el valor de W a la variable Valor
    
    movlw   100			; Se mueve 100 a W
    subwf   Valor, F		; Se resta 100 a Valor y se guarda
    incf    Centenas		; Se incrementa en 1 la variable Centenas
    btfsc   STATUS, 0		; Se verifica si está apagada la bandera de BORROW
    goto    $-4			; Si está encedida se regresa 4 instrucciones atras
    decf    Centenas		; Si no está encedida se resta 1 a la variable Centenas
    movlw   100			; Se mueve 100 a W
    addwf   Valor, F		; Se añaden los 100 a la variable Valor
    call    DECENAS_CONFIG	; Se llama la subrutina para obtener las decenas
    
    return
    
DECENAS_CONFIG:
    movlw   10			; Se mueve 10 a W
    subwf   Valor, F		; Se resta 10 a Valor y se guarda 
    incf    Decenas		; Se incrementa en 1 la variable Decenas
    btfsc   STATUS, 0		; Se verifica si está apagada la bandera de BORROW 
    goto    $-4			; Si está encedida se regresa 4 instrucciones atras
    decf    Decenas		; Si no está encedida se resta 1 a la variable Decenas
    movlw   10			; Se mueve 10 a W
    addwf   Valor, F		; Se añaden los 10 a la varible Valor
    call    UNIDADES_CONFIG	; Se llama la subrutina para obtener las unidades
    
    return
    
UNIDADES_CONFIG:
    movlw   1			; Se mueve 1 a W
    subwf   Valor, F		; Se resta 1 a Valor y se guarda
    incf    Unidades		; Se incrementa en 1 la variable Unidades
    btfsc   STATUS, 0		; Se verifica si está apagada la bandera de BORROW
    goto    $-4			; Si está encedida se regresa 4 instrucciones atras
    decf    Unidades		; Si no está encedida se resta 1 a la variable Decenas
    movlw   1			; Se mueve 1 a W
    addwf   Valor, F		; Se añade 1 a la variable Valor

    return   
    
DISPLAY:
    movf    Unidades, W		; Se mueve valor de Unidades a W
    call    Tabla		; Se busca valor a cargar en PORTC
    movwf   Valor_Display       ; Se guarda en nueva variable en Display1
    
    movf    Decenas, W		; Se mueve valor de Decenas a W
    call    Tabla		; Se busca valor a cargar en PORTC
    movwf   Valor_Display+1	; Se guarda en nueva variable Display2
    
    movf    Centenas, W		; Se mueve valor de Centenas a W
    call    Tabla		; Se busca valor a cargar en PORTC
    movwf   Valor_Display+2	; Se guarda en nueva variable Display3
    return
    
end

