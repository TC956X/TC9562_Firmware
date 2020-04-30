# Toshiba Electronic Devices & Storage Corporation TC9562 PCIe Ethernet Bridge Firmware
Release Date: March 31 2020
Relase Version: V_1.1.0

===============================================================================


Introduction:
=============
The folder contains a Keil project, which is the firmware for PCIe interface.

Environment:
============
1. Keil uVision IDE Version 5 or Version 4 is used.  
   Evaluation licence should be OK, as the image size is much smaller than 32k. This project does not
   use any RTOS. It is a OSless firmware.
2. Keil JLink standard/Pro JTAG connector is used.

Procedures
==========
1. Open the project file, compile it and run it. 
2. A binary file will be generated, and it is located at ./InOut
3. A windows tool may be used to convert the binary into header file format. If windows environment
   does not support this tool, please ignore this step.

 Notes
==========
1. In general, this OSless Version of firmware is configured to switch from PCIe HW sequencer to PCIe SW sequencer on POR sequence.
    #define TC9562_SWITCH_HW_TO_SW_SEQ macro is enabled 
	
	In case switch from PCIe HW sequencer to PCIe SW sequencer is not required, it can be disabled 
	#define TC9562_SWITCH_HW_TO_SW_SEQ  macro is disabled 

2. Some debugging counters are available:
	#define  TC9562_M3_DBG_CNT_START    0x2000F800  // Debugging count SRAM area start address
	/* TC9562_M3_DBG_CNT_START + 4*0:  DMA TX0
	*   .........................
	* TC9562_M3_DBG_CNT_START + 4*5:   DMA TX5
	* TC9562_M3_DBG_CNT_START + 4*6:   DMA RX0
	*   .........................
	* TC9562_M3_DBG_CNT_START + 4*11:  DMA RX5
	* TC9562_M3_DBG_CNT_START + 4*12:  INTC WDT Expiry Count
	* TC9562_M3_DBG_CNT_START + 4*13:  MAC LPI/PWR/EVENT
	* TC9562_M3_DBG_CNT_START + 4*15:  guiM3Ticks 
	*/
	
3. Firmware Version 
	#define TC9562_M3_DBG_VER_START     0x2000F900 // Firmware Version SRAM area start address	
	
Release Versions:
===================================================
TC9562_Firmware_PCIeBridge_20190930_V1.0.0:
===================================================
1. Initial Version
===================================================
TC9562_Firmware_PCIeBridge_20200331_V1.1.0:
===================================================
1.. __wfi() instruction added.