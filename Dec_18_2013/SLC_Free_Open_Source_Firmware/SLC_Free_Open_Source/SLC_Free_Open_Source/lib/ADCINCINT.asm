;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: ADCINCINT.asm
;;  Version: 1.20, Updated on 2012/9/21 at 11:56:47
;;
;;  DESCRIPTION: Assembler interrupt service routine for the ADCINC
;;               A/D Converter User Module. This code works for both the
;;               first and second-order modulator topologies.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress Semiconductor 2012. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "memory.inc"
include "ADCINC.inc"


;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------

export _ADCINC_ADConversion_ISR

export _ADCINC_iResult
export  ADCINC_iResult
export _ADCINC_fStatus
export  ADCINC_fStatus
export _ADCINC_bState
export  ADCINC_bState
export _ADCINC_fMode
export  ADCINC_fMode
export _ADCINC_bNumSamples
export  ADCINC_bNumSamples

;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------
AREA InterruptRAM(RAM,REL)
 ADCINC_iResult:
_ADCINC_iResult:                           BLK  2 ;Calculated answer
  iTemp:                                   BLK  2 ;internal temp storage
 ADCINC_fStatus:
_ADCINC_fStatus:                           BLK  1 ;ADC Status
 ADCINC_bState:
_ADCINC_bState:                            BLK  1 ;State value of ADC count
 ADCINC_fMode:
_ADCINC_fMode:                             BLK  1 ;Integrate and reset mode.
 ADCINC_bNumSamples:
_ADCINC_bNumSamples:                       BLK  1 ;Number of samples to take.

;-----------------------------------------------
;  EQUATES
;-----------------------------------------------

;@PSoC_UserCode_INIT@ (Do not change this line.)
;---------------------------------------------------
; Insert your custom declarations below this banner
;---------------------------------------------------

;------------------------
;  Constant Definitions
;------------------------


;------------------------
; Variable Allocation
;------------------------

//14Point7 ASM Code Start
AREA InterruptRAM (RAM, REL, CON)
ADC_Counter:: //ADC Counter
_ADC_Counter:: BLK  1
Ri_Max_x1:: //Peak of Nermest AC coupled signal for temperature sensing
_Ri_Max_x1:: BLK  1
Ri_Min_x1:: //Trough of Nermest AC coupled signal for temperature sensing
_Ri_Min_x1:: BLK  1
ip_x1:: //Voltage Across Pump Current sense Resistor, it is in 
_ip_x1:: BLK  1
ADC_IF::// Byte value used to track which ADC sample last occurred, ADC_IF[0]=1 if Ri_Min,ADC_IF[1]=1 if Ri_Max,ADC_IF[2]=1 if VR
_ADC_IF:: BLK  1
//14Point7 ASM Code End

;---------------------------------------------------
; Insert your custom declarations above this banner
;---------------------------------------------------
;@PSoC_UserCode_END@ (Do not change this line.)


AREA UserModules (ROM, REL)

;-----------------------------------------------------------------------------
;  FUNCTION NAME: _ADCINC_ADConversion_ISR
;
;  DESCRIPTION: Perform final filter operations to produce output samples.
;
;-----------------------------------------------------------------------------
;
;    The decimation rate is established by the PWM interrupt. Four timer
;    clocks elapse for each modulator output (decimator input) since the
;    phi1/phi2 generator divides by 4. This means the timer period and thus
;    it's interrupt must equal 4 times the actual decimation rate.  The
;    decimator is ru  for 2^(#bits-6).
;
_ADCINC_ADConversion_ISR:
    dec  [ADCINC_bState]
if1:
    jc endif1 ; no underflow
    reti
endif1:
    cmp [ADCINC_fMode],0
if2: 
    jnz endif2  ;leaving reset mode
    push A                            ;read decimator
    mov  A, reg[DEC_DL]
    mov  [iTemp + LowByte],A
    mov  A, reg[DEC_DH]
    mov  [iTemp + HighByte], A
    pop A
    mov [ADCINC_fMode],1
    mov [ADCINC_bState],((1<<(ADCINC_bNUMBITS- 6))-1)
    reti
endif2:
    ;This code runs at end of integrate
    ADCINC_RESET_INTEGRATOR_M
    push A
    mov  A, reg[DEC_DL]
    sub  A,[iTemp + LowByte]
    mov  [iTemp +LowByte],A
    mov  A, reg[DEC_DH]
    sbb  A,[iTemp + HighByte]
    asr  A
    rrc  [iTemp + LowByte]

       ;Covert to Unipolar
IF  ADCINC_9_OR_MORE_BITS
    add  A, (1<<(ADCINC_bNUMBITS - 9))
ELSE
    add [iTemp + LowByte], (1<<(ADCINC_bNUMBITS - 1)) ;work on lower Byte
    adc A,0 
ENDIF
       ;check for overflow
IF     ADCINC_8_OR_MORE_BITS
    cmp A,(1<<(ADCINC_bNUMBITS - 8))
if3: 
    jnz endif3 ;overflow
    dec A
    mov [iTemp + LowByte],ffh
endif3:
ELSE
    cmp [iTemp + LowByte],(1<<(ADCINC_bNUMBITS))
if4: 
    jnz endif4 ;overflow
    dec [iTemp + LowByte]
endif4:
ENDIF
IF ADCINC_SIGNED_DATA
IF ADCINC_9_OR_MORE_BITS
    sub A,(1<<(ADCINC_bNUMBITS - 9))
ELSE
    sub [iTemp +LowByte],(1<<(ADCINC_bNUMBITS - 1))
    sbb A,0
ENDIF
ENDIF
    mov  [ADCINC_iResult + LowByte],[iTemp +LowByte]
    mov  [ADCINC_iResult + HighByte],A
    mov  [ADCINC_fStatus],1
ConversionReady:
    ;@PSoC_UserCode_BODY@ (Do not change this line.)
    ;---------------------------------------------------
    ; Insert your custom code below this banner
    ;---------------------------------------------------
	//14Point7 ASM Code Start
	mov A,[_ADC_Counter]
    sub A,0
    JZ ADC_0
    
    sub A,1
    JZ ADC_1
    
    sub A,1
   	JZ ADC_2 
   	
    sub A,1
    JZ ADC_3

    ADC_0: //Ri_Max, Peak of AC coupled Temperature sense signal 
	//and reg[PRT2DR],127 //clear Ri_Out Pin so that the next ADC sample will Acquire the Trough (Ri_Min) of AC coupled Temperature sense signal 
	and reg[PRT0DR],251 //clear Ri_Out Pin so that the next ADC sample will Acquire the Trough (Ri_Min) of AC coupled Temperature sense signal 
    mov [_Ri_Max_x1],[ADCINC_iResult + LowByte]
    mov [_ADC_Counter],1
    or [_ADC_IF],2 // Set ADC_IF[1] to indicate that new Ri_Max data is available
	jmp Cont
    
    ADC_1://Ri_Min, Trough of AC coupled Temperature sense signal 
	mov  reg[AMX_IN],15 //setup Mux so that Next ADC sample is a differential Ip Vs Ia (voltage across pump current sense resistor, P0[7] Vs P0[6]
	//mov  reg[AMX_IN],9 //cal lin
	//or reg[PRT2DR],128 //set Ri_Out Pin
	or reg[PRT0DR],4 //set Ri_Out Pin
    mov [_Ri_Min_x1],[ADCINC_iResult + LowByte]
    or [_ADC_IF],1 // Set ADC_IF[0] to indicate that new Ri_Min data is available
	mov [_ADC_Counter],2

    jmp Cont

    ADC_2://Voltage Across Pump Current sense resistor
	//and reg[PRT2DR],127 //clear Ri_Out Pin, Next ADC sample is not the Ri_Min, but Ri_Out is cleared so that ~1khz AC signal is mantained
	and reg[PRT0DR],251 //clear Ri_Out Pin so that the next ADC sample will Acquire the Trough (Ri_Min) of AC coupled Temperature sense signal 
    mov [_ip_x1],[ADCINC_iResult + LowByte]
    or [_ADC_IF],4 // Set ADC_IF[2] to indicate that new VR data is available
	mov [_ADC_Counter],3
	jmp Cont
	
	ADC_3://Voltage Across Pump Current sense resistor again, this ADC sample is need to keep timing on the ~1khz AC Temperature Sense Signal so might as well get another Sample of VR
	//or reg[PRT2DR],128 //Set Ri_Out Pin so that the next ADC sample will Acquire the Peak (Ri_Max) of AC coupled Temperature sense signal 
	or reg[PRT0DR],4 //set Ri_Out Pin
	mov  reg[AMX_IN],0 // Setup Mux so that Next ADC sample is differential Ri_in Vs Vref, P0[1] Vs P0[0]
	mov [_ip_x1],[ADCINC_iResult + LowByte]
	or [_ADC_IF],4 // Set ADC_IF[2] to indicate that new VR data is available
    mov [_ADC_Counter],0
	
	Cont:
	//14Point7 ASM Code End
	
    ;  Sample data is now in iResult
    ;
    ;  NOTE: This interrupt service routine has already
    ;  preserved the values of the A CPU register. If
    ;  you need to use the X register you must preserve
    ;  its value and restore it before the return from
    ;  interrupt.
    ;---------------------------------------------------
    ; Insert your custom code above this banner
    ;---------------------------------------------------
    ;@PSoC_UserCode_END@ (Do not change this line.)
    pop A
    cmp [ADCINC_bNumSamples],0
if5: 
    jnz endif5 ; Number of samples is zero
    mov [ADCINC_fMode],0
    mov [ADCINC_bState],0
    ADCINC_ENABLE_INTEGRATOR_M
    reti       
endif5:
    dec [ADCINC_bNumSamples]
if6:
    jz endif6  ; count not zero
    mov [ADCINC_fMode],0
    mov [ADCINC_bState],0
    ADCINC_ENABLE_INTEGRATOR_M
    reti       
endif6:
    ;All samples done
    ADCINC_STOPADC_M
 reti 
; end of file ADCINCINT.asm
