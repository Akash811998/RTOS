;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

  .def setPSP
  .def setMSP
  .def getPSP
  .def getMSP
  .def getSvcnumber
  .def getR0fromPSP

;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const
;GPIO_PORTF_DATA_R       .field   0x400253FC

;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

; Blocking function that returns only when SW1 is pressed
setPSP:
               MSR PSP,R0
               BX LR
setMSP:
			   MSR MSP,R0
			   BX LR
getPSP:
			   MRS R0,PSP
			   BX LR
getMSP:
			   MRS R0,MSP
			   BX LR
getSvcnumber:
			    MRS R0,PSP
               ADD R0,#24
               LDR R1,[R0]
               SUB R1,#2
               LDRB R3,[R1]
               MOV R0,R3
               BX LR

getR0fromPSP:
			   MRS R1,PSP
			   LDR R0,[R1]
			   BX LR

.endm

