;/* ============================================================================
; * The MIT License (MIT)
; *
; * Copyright (c) 2019 Toshiba Electronic Devices & Storage Corporation
; * Permission is hereby granted, free of charge, to any person obtaining a copy
; * of this software and associated documentation files (the "Software"), to deal
; * in the Software without restriction, including without limitation the rights
; * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; * copies of the Software, and to permit persons to whom the Software is
; * furnished to do so, subject to the following conditions:
; *
; * The above copyright notice and this permission notice shall be included in
; * all copies or substantial portions of the Software.
; *
; * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; * THE SOFTWARE.
; * ========================================================================= */
;
;/*! History:
; *  30 Sep 2019 : Base lined
; *  VERSION     : 1.0.0
; */
;
;/*
; *********************************************************************************************************
; *
; * Target        : TC9562
; * Filename      : tc9562_startup.s
; *
; *********************************************************************************************************
; */
;
;/*
;*********************************************************************************************************
;*                                              STACK DEFINITIONS
;*********************************************************************************************************
;*/

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp    


;/*
;*********************************************************************************************************
;*                                              STACK DEFINITIONS
;*********************************************************************************************************
;*/

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


;/*
;*********************************************************************************************************
;* 													Vector Table
;*********************************************************************************************************
;*/
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors		
            IMPORT  SysTick_Handler

__Vectors   DCD     __initial_sp					; 0:  Stack Pointer
			DCD     Reset_IRQHandler				; 1:  Reset 
			DCD     NMI_IRQHandler				    ; 2:  NMI 
			DCD     HardFault_IRQHandler			; 3:  Hard Fault 
			DCD     MemManage_IRQHandler       		; 4:  MPU Fault 
			DCD     BusFault_IRQHandler        		; 5:  Bus Fault 
			DCD     UsageFault_IRQHandler      		; 6:  Usage Fault 
			DCD     App_Spurious_IRQHandler   		; 7:  App Spurious 
			DCD     App_Spurious_IRQHandler    		; 8:  App Spurious 
			DCD     App_Spurious_IRQHandler    		; 9:  App Spurious 
			DCD     App_Spurious_IRQHandler    		; 10: App Spurious 
			DCD     SVC_IRQHandler             		; 11: SVCall 
			DCD     DebugMon_IRQHandler        		; 12: Debug Monitor 
			DCD     App_Spurious_IRQHandler    		; 13: App Spurious 
			DCD     PendSV_IRQHandler          		; 14: PendSV 
			DCD     SysTick_Handler            		; 15: SysTick 
								         
			DCD     INTIO1_IRQHandler           	; 0:  External interrupt input (INTI01)                        
			DCD     INTIO2_IRQHandler          		; 1:  External interrupt input (INIT02)                        
			DCD     INTIO3_IRQHandler          		; 2:  External interrupt input (INTI03)                       
			DCD     EXT_INT_IRQHandler         		; 3:  External interrupt input (INT_i)      
			DCD     I2C_SLAVE_IRQHandler       		; 4:  I2C slave interrupt                      
			DCD     I2C_MASTER_IRQHandler      		; 5:  I2C mater interrupt                      
			DCD     SPI_SLAVE_IRQHandler       		; 6:  SPI slave interrupt                      
			DCD     qSPI_IRQHandler			        ; 7:  qSPI INterrupt                                     
			DCD     MAC_LPI_EXIT_IRQHandler    		; 8:  eMAC LPI exit interrupt                                   
			DCD     MAC_POWER_IRQHandler       		; 9:  eMAC Power management interrupt                           
			DCD     MAC_EVENTS_IRQHandler      		; 10: eMAC event from LPI, GRMII, Management counter...
			DCD     EMACTXDMA0_IRQHandler      	    ; 11: MAC interrupt from eMAC Tx DMA Channel 0   
			DCD     EMACTXDMA1_IRQHandler      	    ; 12: MAC interrupt from eMAC Tx DMA Channel 1 
			DCD     EMACTXDMA2_IRQHandler      	    ; 13: MAC interrupt from eMAC Tx DMA Channel 2   
			DCD     EMACTXDMA3_IRQHandler      	    ; 14: MAC interrupt from eMAC Tx DMA Channel 3   
			DCD     EMACTXDMA4_IRQHandler      	    ; 15: MAC interrupt from eMAC Tx DMA Channel 4
			DCD     EMACTXDMA5_IRQHandler      	    ; 16: MAC interrupt from eMAC Tx DMA Channel 5 
			DCD     EMACTXDMA6_IRQHandler      	    ; 17: MAC interrupt from eMAC Tx DMA Channel 6
			DCD     EMACTXDMA7_IRQHandler      	    ; 18: MAC interrupt from eMAC Tx DMA Channel 7					
			DCD     EMACRXDMA0_IRQHandler      	    ; 19: MAC interrupt from eMAC Rx DMA Channel 0       
			DCD     EMACRXDMA1_IRQHandler      	    ; 20: MAC interrupt from eMAC Rx DMA Channel 1        
			DCD     EMACRXDMA2_IRQHandler      	    ; 21: MAC interrupt from eMAC Rx DMA Channel 2      
			DCD     EMACRXDMA3_IRQHandler      	    ; 22: MAC interrupt from eMAC Rx DMA Channel 3      
			DCD     EMACRXDMA4_IRQHandler      	    ; 23: MAC interrupt from eMAC Rx DMA Channel 4  
			DCD     EMACRXDMA5_IRQHandler      	    ; 24: MAC interrupt from eMAC Rx DMA Channel 5
			DCD     EMACRXDMA6_IRQHandler      	    ; 25: MAC interrupt from eMAC Rx DMA Channel 6
			DCD     EMACRXDMA7_IRQHandler      	    ; 26: MAC interrupt from eMAC Rx DMA Channel 7         
			DCD     PPO_IRQHandler      			; 27: MAC PPO
			DCD	     0								; 28: RESERVED
			DCD	     0								; 29: RESERVED
			DCD     GDMA0_IRQHandler      			; 30: GDMA Channel 0 interrupt.        
			DCD     GDMA1_IRQHandler      			; 31: GDMA Channel 1 interrupt.         
			DCD     GDMA2_IRQHandler      			; 32: GDMA Channel 2 interrupt.    
			DCD     GDMA3_IRQHandler      			; 33: GDMA Channel 3 interrupt.          
			DCD     GDMA4_IRQHandler      			; 34: GDMA Channel 4 interrupt.        
			DCD     GDMA5_IRQHandler     			; 35: GDMA Channel 5 interrupt.   
			DCD     GDMAGEN_IRQHandler            	; 36: GDMA general interrupt   
			DCD     SHA_IRQHandler           		; 37: SHA interrupt   
			DCD     UART0_IRQHandler           		; 38: UART0 interrupt
			DCD     UART1_IRQHandler           		; 39: UART1 interrupt   
			DCD     MSIGEN_IRQHandler           	; 40: MSIGEN interrupt   
			DCD     PCIEC0_IRQHandler           	; 41: PCIe controller interrupt 0 
			DCD     PCIEC1_IRQHandler           	; 42: PCIe controller interrupt 1   
			DCD     PCIEC2_IRQHandler        		; 43: PCIe controller interrupt 2     
			DCD     PCIE_L12_IRQHandler             ; 44: PCIe L12 change interrupt             
			DCD     MCU_IRQHandler         			; 45: MCU Flag interrupt
			DCD	    0								; 46: RESERVED
			DCD     0								; 47: RESERVED
			DCD     WDT_IRQHandler         			; 48: Watchdog-timer interrupt
			DCD     TDM0_IRQHandler          		; 49: TDM Stream 0 input error
			DCD     TDM1_IRQHandler          		; 50: TDM Stream 1 input error
			DCD     TDM2_IRQHandler          		; 51: TDM Stream 2 input error
			DCD     TDM3_IRQHandler        			; 52: TDM Stream 3 input error
			DCD     TDMOUT_IRQHandler       	 	; 53: TDM output error 
		                                
            AREA    |.text|, CODE, READONLY


Reset_IRQHandler    PROC
                EXPORT  Reset_IRQHandler  								
                IMPORT  __main
                LDR     R0, =__main
                BX      R0
                ENDP
					
NMI_IRQHandler     	PROC
                EXPORT  NMI_IRQHandler  
                B       .
                ENDP
					
HardFault_IRQHandler	PROC
                EXPORT  HardFault_IRQHandler   
                B       .
                ENDP
					
MemManage_IRQHandler	PROC
                EXPORT  MemManage_IRQHandler      
                B       .
                ENDP
					
BusFault_IRQHandler	PROC
                EXPORT  BusFault_IRQHandler    
                B       .
                ENDP
					
UsageFault_IRQHandler	PROC
                EXPORT  UsageFault_IRQHandler    
                B       .
                ENDP
					
SVC_IRQHandler     	PROC
                EXPORT  SVC_IRQHandler   
                B       .
                ENDP
					
DebugMon_IRQHandler	PROC
                EXPORT  DebugMon_IRQHandler    
                B       .
                ENDP
					
PendSV_IRQHandler  	PROC
                EXPORT  PendSV_IRQHandler   
                B       .
                ENDP

App_Spurious_IRQHandler	PROC
                EXPORT  App_Spurious_IRQHandler   
                B       .
                ENDP
					
App_Reserved_IRQHandler	PROC
                EXPORT  App_Reserved_IRQHandler  
                B       .
                ENDP
																	
CPU_IntDis		PROC
				EXPORT CPU_IntDis
				CPSID   I
				BX      LR
				ENDP

CPU_IntEn		PROC
				EXPORT CPU_IntEn
				CPSIE   I
				BX      LR
				ENDP
		
Default_Handler PROC
			EXPORT  INTIO1_IRQHandler               [WEAK]
			EXPORT  INTIO2_IRQHandler          		[WEAK]
			EXPORT  INTIO3_IRQHandler          		[WEAK]
			EXPORT  EXT_INT_IRQHandler         		[WEAK]
			EXPORT  I2C_SLAVE_IRQHandler       		[WEAK]
			EXPORT  I2C_MASTER_IRQHandler   		[WEAK]
			EXPORT  SPI_SLAVE_IRQHandler      		[WEAK]
			EXPORT  qSPI_IRQHandler            		[WEAK]
			EXPORT  MAC_LPI_EXIT_IRQHandler  		[WEAK]
			EXPORT  MAC_POWER_IRQHandler            [WEAK]
			EXPORT  MAC_EVENTS_IRQHandler           [WEAK]
			EXPORT  EMACTXDMA0_IRQHandler           [WEAK]
			EXPORT  EMACTXDMA1_IRQHandler      	[WEAK]
			EXPORT  EMACTXDMA2_IRQHandler      	[WEAK]
			EXPORT  EMACTXDMA3_IRQHandler      	[WEAK]
			EXPORT  EMACTXDMA4_IRQHandler     	[WEAK]
			EXPORT  EMACTXDMA5_IRQHandler     	[WEAK]
			EXPORT  EMACTXDMA6_IRQHandler     	[WEAK]
			EXPORT  EMACTXDMA7_IRQHandler     	[WEAK]
			EXPORT  EMACRXDMA0_IRQHandler     	[WEAK]
			EXPORT  EMACRXDMA1_IRQHandler      	[WEAK]                
			EXPORT  EMACRXDMA2_IRQHandler     	[WEAK]
			EXPORT  EMACRXDMA3_IRQHandler     	[WEAK]
			EXPORT  EMACRXDMA4_IRQHandler     	[WEAK]                              
			EXPORT  EMACRXDMA5_IRQHandler      	[WEAK]
			EXPORT  EMACRXDMA6_IRQHandler      	[WEAK]
			EXPORT  EMACRXDMA7_IRQHandler      	[WEAK]
								
			EXPORT  PPO_IRQHandler      			[WEAK]
			EXPORT  GDMA0_IRQHandler      			[WEAK]
			EXPORT  GDMA1_IRQHandler      			[WEAK]
			EXPORT  GDMA2_IRQHandler      			[WEAK]
			EXPORT  GDMA3_IRQHandler      			[WEAK]
			EXPORT  GDMA4_IRQHandler      			[WEAK]
			EXPORT  GDMA5_IRQHandler     			[WEAK]
			EXPORT  GDMAGEN_IRQHandler            	[WEAK]
			EXPORT  SHA_IRQHandler           			[WEAK]
			EXPORT  UART0_IRQHandler           		[WEAK]
			EXPORT  UART1_IRQHandler           		[WEAK]
			EXPORT  MSIGEN_IRQHandler           		[WEAK]
			EXPORT  PCIEC0_IRQHandler           		[WEAK]
			EXPORT  PCIEC1_IRQHandler           		[WEAK]
			EXPORT  PCIEC2_IRQHandler        		[WEAK]

			EXPORT  PCIE_L12_IRQHandler             	[WEAK]
			EXPORT  MCU_IRQHandler            		[WEAK]
			EXPORT  WDT_IRQHandler         			[WEAK]
			EXPORT  TDM0_IRQHandler         			[WEAK]
			EXPORT  TDM1_IRQHandler          		[WEAK]
			EXPORT  TDM2_IRQHandler          		[WEAK]
			EXPORT  TDM3_IRQHandler          		[WEAK]
			EXPORT  TDMOUT_IRQHandler        		[WEAK]

INTIO1_IRQHandler
INTIO2_IRQHandler
INTIO3_IRQHandler
EXT_INT_IRQHandler
I2C_SLAVE_IRQHandler
I2C_MASTER_IRQHandler
SPI_SLAVE_IRQHandler
qSPI_IRQHandler
MAC_LPI_EXIT_IRQHandler
MAC_POWER_IRQHandler
MAC_EVENTS_IRQHandler
EMACTXDMA0_IRQHandler
EMACTXDMA1_IRQHandler
EMACTXDMA2_IRQHandler
EMACTXDMA3_IRQHandler
EMACTXDMA4_IRQHandler
EMACTXDMA5_IRQHandler
EMACTXDMA6_IRQHandler
EMACTXDMA7_IRQHandler
EMACRXDMA0_IRQHandler
EMACRXDMA1_IRQHandler         
EMACRXDMA2_IRQHandler
EMACRXDMA3_IRQHandler
EMACRXDMA4_IRQHandler                     
EMACRXDMA5_IRQHandler
EMACRXDMA6_IRQHandler
EMACRXDMA7_IRQHandler
					
PPO_IRQHandler
GDMA0_IRQHandler
GDMA1_IRQHandler
GDMA2_IRQHandler
GDMA3_IRQHandler
GDMA4_IRQHandler
GDMA5_IRQHandler
GDMAGEN_IRQHandler
SHA_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
MSIGEN_IRQHandler
PCIEC0_IRQHandler
PCIEC1_IRQHandler
PCIEC2_IRQHandler

PCIE_L12_IRQHandler
MCU_IRQHandler
WDT_IRQHandler
TDM0_IRQHandler
TDM1_IRQHandler
TDM2_IRQHandler
TDM3_IRQHandler
TDMOUT_IRQHandler
    B       .
    ENDP


    ALIGN

; User Initial Stack & Heap
    IF      :DEF:__MICROLIB

    EXPORT  __initial_sp
    EXPORT  __heap_base
    EXPORT  __heap_limit

    ELSE

    IMPORT  __use_two_region_memory
    EXPORT  __user_initial_stackheap
__user_initial_stackheap

    LDR     R0, =  Heap_Mem
    LDR     R1, =(Stack_Mem + Stack_Size)
    LDR     R2, = (Heap_Mem +  Heap_Size)
    LDR     R3, = Stack_Mem
    BX      LR

    ALIGN

    ENDIF


    END
