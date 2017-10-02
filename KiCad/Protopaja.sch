EESchema Schematic File Version 2
LIBS:Protopaja-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:Protopaja
LIBS:Protopaja-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Futuwear V2"
Date "2017-08-25"
Rev "2.1"
Comp "Protopaja"
Comment1 "Drawn by Bruce Clayhills"
Comment2 "Schematic of the Futuwear V2 clothing main PCB"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ESP32-WROOM U2
U 1 1 5971E31E
P 5590 4400
F 0 "U2" H 4890 5650 60  0000 C CNN
F 1 "ESP32-WROOM" H 6090 5650 60  0000 C CNN
F 2 "ESP32-footprints-Lib:ESP32-WROOM" H 5940 5750 60  0001 C CNN
F 3 "" H 5140 4850 60  0001 C CNN
	1    5590 4400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR22
U 1 1 5971E3B1
P 6625 1225
F 0 "#PWR22" H 6625 1075 50  0001 C CNN
F 1 "+3.3V" H 6625 1365 50  0000 C CNN
F 2 "" H 6625 1225 50  0001 C CNN
F 3 "" H 6625 1225 50  0001 C CNN
	1    6625 1225
	1    0    0    -1  
$EndComp
$Comp
L C_Small C7
U 1 1 5971F372
P 4125 1150
F 0 "C7" H 4135 1220 50  0000 L CNN
F 1 "2uF" H 4135 1070 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4125 1150 50  0001 C CNN
F 3 "" H 4125 1150 50  0001 C CNN
	1    4125 1150
	1    0    0    -1  
$EndComp
$Comp
L C_Small C9
U 1 1 5971FF75
P 5600 1150
F 0 "C9" H 5610 1220 50  0000 L CNN
F 1 "2uF" H 5610 1070 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5600 1150 50  0001 C CNN
F 3 "" H 5600 1150 50  0001 C CNN
	1    5600 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR16
U 1 1 59720FC3
P 4125 1350
F 0 "#PWR16" H 4125 1100 50  0001 C CNN
F 1 "GND" H 4125 1200 50  0000 C CNN
F 2 "" H 4125 1350 50  0001 C CNN
F 3 "" H 4125 1350 50  0001 C CNN
	1    4125 1350
	1    0    0    -1  
$EndComp
Text Notes 4650 700  0    60   ~ 0
LDO-Circuit\n
$Comp
L +3.3V #PWR2
U 1 1 59722043
P 800 2850
F 0 "#PWR2" H 800 2700 50  0001 C CNN
F 1 "+3.3V" H 800 2990 50  0000 C CNN
F 2 "" H 800 2850 50  0001 C CNN
F 3 "" H 800 2850 50  0001 C CNN
	1    800  2850
	1    0    0    -1  
$EndComp
$Comp
L C_Small C3
U 1 1 59722221
P 1250 3200
F 0 "C3" H 1260 3270 50  0000 L CNN
F 1 "1uF" H 1260 3120 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1250 3200 50  0001 C CNN
F 3 "" H 1250 3200 50  0001 C CNN
	1    1250 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR3
U 1 1 5972240F
P 800 3500
F 0 "#PWR3" H 800 3250 50  0001 C CNN
F 1 "GND" H 800 3350 50  0000 C CNN
F 2 "" H 800 3500 50  0001 C CNN
F 3 "" H 800 3500 50  0001 C CNN
	1    800  3500
	1    0    0    -1  
$EndComp
Text Notes 1000 2600 0    59   ~ 0
MCU 3.3V IN\n
$Comp
L C_Small C5
U 1 1 59727220
P 2450 3400
F 0 "C5" H 2460 3470 50  0000 L CNN
F 1 "1nF" H 2460 3320 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2450 3400 50  0001 C CNN
F 3 "" H 2450 3400 50  0001 C CNN
	1    2450 3400
	1    0    0    -1  
$EndComp
$Comp
L R_Small R5
U 1 1 59727281
P 2450 2900
F 0 "R5" H 2480 2920 50  0000 L CNN
F 1 "12KΩ" H 2480 2860 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 2450 2900 50  0001 C CNN
F 3 "" H 2450 2900 50  0001 C CNN
	1    2450 2900
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR6
U 1 1 59727C57
P 2450 2700
F 0 "#PWR6" H 2450 2550 50  0001 C CNN
F 1 "+3.3V" H 2450 2840 50  0000 C CNN
F 2 "" H 2450 2700 50  0001 C CNN
F 3 "" H 2450 2700 50  0001 C CNN
	1    2450 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR7
U 1 1 597288AB
P 2450 3600
F 0 "#PWR7" H 2450 3350 50  0001 C CNN
F 1 "GND" H 2450 3450 50  0000 C CNN
F 2 "" H 2450 3600 50  0001 C CNN
F 3 "" H 2450 3600 50  0001 C CNN
	1    2450 3600
	1    0    0    -1  
$EndComp
Text Notes 2600 2625 0    59   ~ 0
CHIP_PU HIGH
$Comp
L GND #PWR23
U 1 1 5972E666
P 6640 5250
F 0 "#PWR23" H 6640 5000 50  0001 C CNN
F 1 "GND" H 6640 5100 50  0000 C CNN
F 2 "" H 6640 5250 50  0001 C CNN
F 3 "" H 6640 5250 50  0001 C CNN
	1    6640 5250
	1    0    0    -1  
$EndComp
$Comp
L R_Small R12
U 1 1 59766A27
P 9555 1175
F 0 "R12" H 9585 1195 50  0000 L CNN
F 1 "2.2KΩ" H 9585 1135 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9555 1175 50  0001 C CNN
F 3 "" H 9555 1175 50  0001 C CNN
	1    9555 1175
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R15
U 1 1 59766A94
P 10270 1380
F 0 "R15" H 10345 1355 50  0000 L CNN
F 1 "330Ω" H 10035 1435 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 10270 1380 50  0001 C CNN
F 3 "" H 10270 1380 50  0001 C CNN
	1    10270 1380
	-1   0    0    1   
$EndComp
$Comp
L C_Small C10
U 1 1 59766AE7
P 10270 1880
F 0 "C10" H 10280 1950 50  0000 L CNN
F 1 "0.1uF" H 10280 1800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10270 1880 50  0001 C CNN
F 3 "" H 10270 1880 50  0001 C CNN
	1    10270 1880
	1    0    0    -1  
$EndComp
$Comp
L D D7
U 1 1 5978C76C
P 10480 1125
F 0 "D7" H 10480 1225 50  0000 C CNN
F 1 "D" H 10480 1025 50  0000 C CNN
F 2 "Diodes_SMD:D_0805" H 10480 1125 50  0001 C CNN
F 3 "" H 10480 1125 50  0001 C CNN
	1    10480 1125
	1    0    0    -1  
$EndComp
$Comp
L R_Small R16
U 1 1 5978F63E
P 10520 1900
F 0 "R16" H 10550 1920 50  0000 L CNN
F 1 "1MΩ" H 10550 1860 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 10520 1900 50  0001 C CNN
F 3 "" H 10520 1900 50  0001 C CNN
	1    10520 1900
	1    0    0    -1  
$EndComp
$Comp
L C_Small C6
U 1 1 597BF9EA
P 2940 1450
F 0 "C6" H 2950 1520 50  0000 L CNN
F 1 "10uF" H 2950 1370 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2940 1450 50  0001 C CNN
F 3 "" H 2940 1450 50  0001 C CNN
	1    2940 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR10
U 1 1 597C0D8F
P 2940 1640
F 0 "#PWR10" H 2940 1390 50  0001 C CNN
F 1 "GND" H 2940 1490 50  0000 C CNN
F 2 "" H 2940 1640 50  0001 C CNN
F 3 "" H 2940 1640 50  0001 C CNN
	1    2940 1640
	1    0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 597CAE78
P 740 1440
F 0 "C1" H 750 1510 50  0000 L CNN
F 1 "10uF" H 750 1360 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 740 1440 50  0001 C CNN
F 3 "" H 740 1440 50  0001 C CNN
	1    740  1440
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR1
U 1 1 597CC4A0
P 740 1670
F 0 "#PWR1" H 740 1420 50  0001 C CNN
F 1 "GND" H 740 1520 50  0000 C CNN
F 2 "" H 740 1670 50  0001 C CNN
F 3 "" H 740 1670 50  0001 C CNN
	1    740  1670
	1    0    0    -1  
$EndComp
$Comp
L R_Small R2
U 1 1 597CD609
P 1090 1790
F 0 "R2" H 1120 1810 50  0000 L CNN
F 1 "680Ω" H 1120 1750 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1090 1790 50  0001 C CNN
F 3 "" H 1090 1790 50  0001 C CNN
	1    1090 1790
	1    0    0    -1  
$EndComp
$Comp
L R_Small R3
U 1 1 597D24FD
P 1755 1990
F 0 "R3" H 1785 2010 50  0000 L CNN
F 1 "560Ω" H 1785 1950 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1755 1990 50  0001 C CNN
F 3 "" H 1755 1990 50  0001 C CNN
	1    1755 1990
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR5
U 1 1 597D3152
P 2400 1990
F 0 "#PWR5" H 2400 1740 50  0001 C CNN
F 1 "GND" H 2400 1840 50  0000 C CNN
F 2 "" H 2400 1990 50  0001 C CNN
F 3 "" H 2400 1990 50  0001 C CNN
	1    2400 1990
	1    0    0    -1  
$EndComp
$Comp
L R_Small R9
U 1 1 5979AEE1
P 6040 5620
F 0 "R9" H 6070 5640 50  0000 L CNN
F 1 "5KΩ" H 6070 5580 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6040 5620 50  0001 C CNN
F 3 "" H 6040 5620 50  0001 C CNN
	1    6040 5620
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR19
U 1 1 5979BE1F
P 6040 5830
F 0 "#PWR19" H 6040 5580 50  0001 C CNN
F 1 "GND" H 6040 5680 50  0000 C CNN
F 2 "" H 6040 5830 50  0001 C CNN
F 3 "" H 6040 5830 50  0001 C CNN
	1    6040 5830
	1    0    0    -1  
$EndComp
$Comp
L R_Small R6
U 1 1 597A3D2E
P 2590 1400
F 0 "R6" H 2620 1420 50  0000 L CNN
F 1 "2KΩ" H 2620 1360 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 2590 1400 50  0001 C CNN
F 3 "" H 2590 1400 50  0001 C CNN
	1    2590 1400
	1    0    0    -1  
$EndComp
Text Notes 1900 700  0    59   ~ 0
CHARGER\n
$Comp
L R_Small R13
U 1 1 597D8BE7
P 6335 2065
F 0 "R13" V 6270 1975 50  0000 L CNN
F 1 "1MΩ" V 6410 1990 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6335 2065 50  0001 C CNN
F 3 "" H 6335 2065 50  0001 C CNN
	1    6335 2065
	0    1    1    0   
$EndComp
Text Notes 8700 725  0    59   ~ 0
Battery Protection Circuit\n
$Comp
L R_Small R10
U 1 1 597F0EC8
P 6600 3550
F 0 "R10" H 6450 3500 50  0000 L CNN
F 1 "10KΩ" H 6350 3600 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6600 3550 50  0001 C CNN
F 3 "" H 6600 3550 50  0001 C CNN
	1    6600 3550
	-1   0    0    1   
$EndComp
$Comp
L R_Small R11
U 1 1 597F1027
P 6900 3550
F 0 "R11" H 6950 3600 50  0000 L CNN
F 1 "10KΩ" H 6950 3500 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6900 3550 50  0001 C CNN
F 3 "" H 6900 3550 50  0001 C CNN
	1    6900 3550
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR21
U 1 1 597F3871
P 6600 3400
F 0 "#PWR21" H 6600 3250 50  0001 C CNN
F 1 "+3.3V" H 6600 3540 50  0000 C CNN
F 2 "" H 6600 3400 50  0001 C CNN
F 3 "" H 6600 3400 50  0001 C CNN
	1    6600 3400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR24
U 1 1 597F3ACF
P 6900 3400
F 0 "#PWR24" H 6900 3250 50  0001 C CNN
F 1 "+3.3V" H 6900 3540 50  0000 C CNN
F 2 "" H 6900 3400 50  0001 C CNN
F 3 "" H 6900 3400 50  0001 C CNN
	1    6900 3400
	1    0    0    -1  
$EndComp
$Comp
L BQ29733DSER IC2
U 1 1 598113D8
P 9020 1480
F 0 "IC2" H 9520 1630 50  0000 C CNN
F 1 "BQ29733DSER" H 9520 1130 50  0000 C CNN
F 2 "BQ29702DSER:DSE_(S-PWSON-N6)" H 9520 1030 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/bq2971.pdf" H 9520 930 50  0001 C CNN
F 4 "Li-Ion/Li Polymer Advanced Single-Cell Battery Protector IC Family" H 9520 830 50  0001 C CNN "Description"
F 5 "RS" H 9520 730 50  0001 C CNN "Supplier_Name"
F 6 "" H 9520 630 50  0001 C CNN "RS Part Number"
F 7 "Texas Instruments" H 9520 530 50  0001 C CNN "Manufacturer_Name"
F 8 "BQ29702DSER" H 9520 430 50  0001 C CNN "Manufacturer_Part_Number"
F 9 "" H 9520 330 50  0001 C CNN "Allied_Number"
F 10 "" H 9520 230 50  0001 C CNN "Other Part Number"
F 11 "" H 9870 130 50  0001 C CNN "Height"
	1    9020 1480
	1    0    0    -1  
$EndComp
$Comp
L MCP73831T-2ATI_OT IC1
U 1 1 5981D6C0
P 1460 1230
F 0 "IC1" H 1960 1380 50  0000 C CNN
F 1 "MCP73831T-2ATI_OT" H 1960 880 50  0000 C CNN
F 2 "MCP73831T-2ATI_OT:SOT95P270X145-5N" H 1960 780 50  0001 C CNN
F 3 "http://docs-europe.electrocomponents.com/webdocs/1381/0900766b81381747.pdf" H 1960 680 50  0001 C CNN
F 4 "Microchip MCP73831T-2ATI/OT, Battery Charge Controller Lithium-Ion/Polymer, 500mA, 5-pin SOT-23" H 1960 580 50  0001 C CNN "Description"
F 5 "RS" H 1960 480 50  0001 C CNN "Supplier_Name"
F 6 "7386617P" H 1960 380 50  0001 C CNN "RS Part Number"
F 7 "Microchip" H 1960 280 50  0001 C CNN "Manufacturer_Name"
F 8 "MCP73831T-2ATI/OT" H 1960 180 50  0001 C CNN "Manufacturer_Part_Number"
F 9 "70388655" H 1960 80  50  0001 C CNN "Allied_Number"
F 10 "" H 1960 -20 50  0001 C CNN "Other Part Number"
F 11 "1.45" H 2310 -120 50  0001 C CNN "Height"
	1    1460 1230
	1    0    0    -1  
$EndComp
NoConn ~ 4640 4000
NoConn ~ 4640 4100
NoConn ~ 5140 5450
NoConn ~ 5340 5450
NoConn ~ 5440 5450
NoConn ~ 5540 5450
NoConn ~ 5640 5450
NoConn ~ 5740 5450
NoConn ~ 5840 5450
NoConn ~ 6490 4450
NoConn ~ 6490 4250
NoConn ~ 6490 4150
NoConn ~ 6490 3650
Text Label 1465 5260 2    47   ~ 0
RXD0
Text Label 1460 5190 2    47   ~ 0
TXD0
NoConn ~ 9020 1480
$Comp
L +BATT #PWR32
U 1 1 5984ACAC
P 10755 1125
F 0 "#PWR32" H 10755 975 50  0001 C CNN
F 1 "+BATT" H 10755 1265 50  0000 C CNN
F 2 "" H 10755 1125 50  0001 C CNN
F 3 "" H 10755 1125 50  0001 C CNN
	1    10755 1125
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR11
U 1 1 5984CCC8
P 3240 1135
F 0 "#PWR11" H 3240 985 50  0001 C CNN
F 1 "+BATT" H 3240 1275 50  0000 C CNN
F 2 "" H 3240 1135 50  0001 C CNN
F 3 "" H 3240 1135 50  0001 C CNN
	1    3240 1135
	1    0    0    -1  
$EndComp
Text Label 7100 4050 0    60   ~ 0
SDA
Text Label 7100 3750 0    60   ~ 0
SCL
Text Label 6625 3850 0    60   ~ 0
TXD0
Text Label 6625 3950 0    60   ~ 0
RXD0
Text Label 4490 4600 2    60   ~ 0
GPIO25
Text Label 4490 4700 2    60   ~ 0
GPIO26
Text Label 4490 4800 2    60   ~ 0
GPIO27
Text Label 4500 4900 2    60   ~ 0
GPIO14
$Comp
L PWR_FLAG #FLG1
U 1 1 5988CE3F
P 740 1230
F 0 "#FLG1" H 740 1305 50  0001 C CNN
F 1 "PWR_FLAG" H 740 1380 50  0000 C CNN
F 2 "" H 740 1230 50  0001 C CNN
F 3 "" H 740 1230 50  0001 C CNN
	1    740  1230
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR15
U 1 1 59896F9B
P 3900 900
F 0 "#PWR15" H 3900 750 50  0001 C CNN
F 1 "+BATT" H 3900 1040 50  0000 C CNN
F 2 "" H 3900 900 50  0001 C CNN
F 3 "" H 3900 900 50  0001 C CNN
	1    3900 900 
	1    0    0    -1  
$EndComp
Text Label 10685 2440 0    60   ~ 0
-BATT
$Comp
L PWR_FLAG #FLG4
U 1 1 598A5F92
P 10520 1580
F 0 "#FLG4" H 10520 1655 50  0001 C CNN
F 1 "PWR_FLAG" H 10520 1730 50  0000 C CNN
F 2 "" H 10520 1580 50  0001 C CNN
F 3 "" H 10520 1580 50  0001 C CNN
	1    10520 1580
	0    1    1    0   
$EndComp
Text Label 4490 3800 2    60   ~ 0
MCU_PWR
Text Label 1450 2950 0    60   ~ 0
MCU_PWR
Text GLabel 1340 1035 1    47   Input ~ 0
USB_IN
Text GLabel 1250 4075 2    47   Input ~ 0
USB_IN
$Comp
L GND #PWR4
U 1 1 598D6024
P 1020 4750
F 0 "#PWR4" H 1020 4500 50  0001 C CNN
F 1 "GND" H 1020 4600 50  0000 C CNN
F 2 "" H 1020 4750 50  0001 C CNN
F 3 "" H 1020 4750 50  0001 C CNN
	1    1020 4750
	1    0    0    -1  
$EndComp
Text Notes 1050 3950 0    47   ~ 0
MINI-USB-B\n
Text Label 5940 5660 2    47   ~ 0
IO15
Text Label 2115 5335 0    47   ~ 0
RTS
NoConn ~ 6490 4750
Text Label 6650 4550 0    47   ~ 0
GPIO17
Text Label 6650 4650 0    47   ~ 0
GPIO16
$Comp
L PWR_FLAG #FLG2
U 1 1 598467B7
P 3240 1290
F 0 "#FLG2" H 3240 1365 50  0001 C CNN
F 1 "PWR_FLAG" H 3240 1440 50  0000 C CNN
F 2 "" H 3240 1290 50  0001 C CNN
F 3 "" H 3240 1290 50  0001 C CNN
	1    3240 1290
	-1   0    0    1   
$EndComp
Text Label 2850 3150 0    47   ~ 0
EN
Text Label 4490 3900 2    47   ~ 0
EN
Text Label 7350 4850 0    47   ~ 0
IO0
$Comp
L 09.03201.02 S1
U 1 1 5981785E
P 5800 950
F 0 "S1" H 6150 1100 50  0000 C CNN
F 1 "09.03201.02" H 6150 700 50  0000 C CNN
F 2 "09.03201:SHDR3W60P0X254_1X3_1000X250X740P" H 6150 600 50  0001 C CNN
F 3 "http://eao.com/fileadmin/countries/uk/eoz/pdf/EOZ_1K2_slide_switch.pdf" H 6150 500 50  0001 C CNN
F 4 "EAO - 09.03201.02 - SWITCH, SPST-CO, 0.5A, 12V, PCB, RED" H 6150 400 50  0001 C CNN "Description"
F 5 "RS" H 6150 300 50  0001 C CNN "Supplier_Name"
F 6 "" H 6150 200 50  0001 C CNN "RS Part Number"
F 7 "EAO" H 6150 100 50  0001 C CNN "Manufacturer_Name"
F 8 "09.03201.02" H 6150 0   50  0001 C CNN "Manufacturer_Part_Number"
F 9 "" H 6150 -100 50  0001 C CNN "Allied_Number"
F 10 "" H 6150 -200 50  0001 C CNN "Other Part Number"
F 11 "7.4" H 6350 -300 50  0001 C CNN "Height"
	1    5800 950 
	1    0    0    -1  
$EndComp
$Comp
L XC6210B332PR U1
U 1 1 5981C71F
P 4950 1050
F 0 "U1" H 4775 1250 60  0000 C CNN
F 1 "XC6210B332PR" H 4950 850 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT89-5_Housing_Handsoldering" H 4900 850 60  0001 C CNN
F 3 "" H 4900 850 60  0001 C CNN
	1    4950 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR18
U 1 1 598218AC
P 5600 1375
F 0 "#PWR18" H 5600 1125 50  0001 C CNN
F 1 "GND" H 5600 1225 50  0000 C CNN
F 2 "" H 5600 1375 50  0001 C CNN
F 3 "" H 5600 1375 50  0001 C CNN
	1    5600 1375
	1    0    0    -1  
$EndComp
$Comp
L CP2102-GM IC3
U 1 1 598292E4
P 9325 4375
F 0 "IC3" H 10475 5225 50  0000 L CNN
F 1 "CP2102-GM" H 10475 3125 50  0000 L CNN
F 2 "CP2102-GM:QFN50P500X500X100-29N" H 10475 3025 50  0001 L CNN
F 3 "http://docs-europe.electrocomponents.com/webdocs/065a/0900766b8065a805.pdf" H 10475 2925 50  0001 L CNN
F 4 "CP2102-GM, USB Controller 12Mbit/s USB 2.0, 3.3 V, 28-Pin, QFN" H 10475 2825 50  0001 L CNN "Description"
F 5 "RS" H 10475 2725 50  0001 C CNN "Supplier_Name"
F 6 "5268841" H 10475 2625 50  0001 C CNN "RS Part Number"
F 7 "Silicon Laboratories" H 10475 2525 50  0001 C CNN "Manufacturer_Name"
F 8 "CP2102-GM" H 10475 2425 50  0001 C CNN "Manufacturer_Part_Number"
F 9 "R1015739" H 10475 2325 50  0001 C CNN "Allied_Number"
F 10 "" H 10475 2225 50  0001 C CNN "Other Part Number"
F 11 "1" H 10475 2125 50  0001 C CNN "Height"
	1    9325 4375
	1    0    0    -1  
$EndComp
NoConn ~ 10025 5775
NoConn ~ 10125 5775
NoConn ~ 10225 5775
NoConn ~ 10625 4975
NoConn ~ 10625 4875
NoConn ~ 10625 4775
NoConn ~ 10625 4575
NoConn ~ 10625 4475
NoConn ~ 10625 4375
NoConn ~ 10325 3375
NoConn ~ 9725 5775
NoConn ~ 9825 5775
$Comp
L GND #PWR31
U 1 1 5982D96A
P 9375 3425
F 0 "#PWR31" H 9375 3175 50  0001 C CNN
F 1 "GND" H 9375 3275 50  0000 C CNN
F 2 "" H 9375 3425 50  0001 C CNN
F 3 "" H 9375 3425 50  0001 C CNN
	1    9375 3425
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR28
U 1 1 5982E7A6
P 8925 4625
F 0 "#PWR28" H 8925 4375 50  0001 C CNN
F 1 "GND" H 8925 4475 50  0000 C CNN
F 2 "" H 8925 4625 50  0001 C CNN
F 3 "" H 8925 4625 50  0001 C CNN
	1    8925 4625
	1    0    0    -1  
$EndComp
Text GLabel 1275 4275 2    47   Input ~ 0
D+
Text GLabel 1275 4375 2    47   Input ~ 0
D-
Text GLabel 9225 4675 0    47   Input ~ 0
D+
Text GLabel 9225 4775 0    47   Input ~ 0
D-
NoConn ~ 1190 4475
$Comp
L C_Small C11
U 1 1 5983780D
P 9175 6000
F 0 "C11" H 9185 6070 50  0000 L CNN
F 1 "10uF" H 9185 5920 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9175 6000 50  0001 C CNN
F 3 "" H 9175 6000 50  0001 C CNN
	1    9175 6000
	1    0    0    -1  
$EndComp
$Comp
L C_Small C12
U 1 1 59837894
P 9475 6000
F 0 "C12" H 9485 6070 50  0000 L CNN
F 1 "0.1uF" H 9485 5920 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9475 6000 50  0001 C CNN
F 3 "" H 9475 6000 50  0001 C CNN
	1    9475 6000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR30
U 1 1 59837BD7
P 9050 6175
F 0 "#PWR30" H 9050 5925 50  0001 C CNN
F 1 "GND" H 9050 6025 50  0000 C CNN
F 2 "" H 9050 6175 50  0001 C CNN
F 3 "" H 9050 6175 50  0001 C CNN
	1    9050 6175
	1    0    0    -1  
$EndComp
Text GLabel 9175 5700 1    47   Input ~ 0
USB_IN
$Comp
L C_Small C4
U 1 1 5983AAFD
P 8675 5050
F 0 "C4" H 8685 5120 50  0000 L CNN
F 1 "10uF" H 8685 4970 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8675 5050 50  0001 C CNN
F 3 "" H 8675 5050 50  0001 C CNN
	1    8675 5050
	1    0    0    -1  
$EndComp
$Comp
L C_Small C8
U 1 1 5983ADB2
P 8950 5050
F 0 "C8" H 8960 5120 50  0000 L CNN
F 1 "0.1uF" H 8960 4970 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8950 5050 50  0001 C CNN
F 3 "" H 8950 5050 50  0001 C CNN
	1    8950 5050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR27
U 1 1 5983B81C
P 8675 5300
F 0 "#PWR27" H 8675 5050 50  0001 C CNN
F 1 "GND" H 8675 5150 50  0000 C CNN
F 2 "" H 8675 5300 50  0001 C CNN
F 3 "" H 8675 5300 50  0001 C CNN
	1    8675 5300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR26
U 1 1 59840AF6
P 8675 4750
F 0 "#PWR26" H 8675 4600 50  0001 C CNN
F 1 "+3.3V" H 8675 4890 50  0000 C CNN
F 2 "" H 8675 4750 50  0001 C CNN
F 3 "" H 8675 4750 50  0001 C CNN
	1    8675 4750
	1    0    0    -1  
$EndComp
$Comp
L R_Small R1
U 1 1 59844CC7
P 1350 5850
F 0 "R1" H 1380 5870 50  0000 L CNN
F 1 "12KΩ" V 1275 5775 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1350 5850 50  0001 C CNN
F 3 "" H 1350 5850 50  0001 C CNN
	1    1350 5850
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R4
U 1 1 59844FBE
P 1350 6450
F 0 "R4" H 1380 6470 50  0000 L CNN
F 1 "12KΩ" V 1275 6375 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1350 6450 50  0001 C CNN
F 3 "" H 1350 6450 50  0001 C CNN
	1    1350 6450
	0    -1   -1   0   
$EndComp
$Comp
L Q_NPN_BCE Q3
U 1 1 59846505
P 1775 5850
F 0 "Q3" H 1975 5900 50  0000 L CNN
F 1 "PBSS4120T" H 1975 5800 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 1975 5950 50  0001 C CNN
F 3 "" H 1775 5850 50  0001 C CNN
	1    1775 5850
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BCE Q4
U 1 1 598465B0
P 1775 6450
F 0 "Q4" H 1975 6500 50  0000 L CNN
F 1 "PBSS4120T" H 1975 6400 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 1975 6550 50  0001 C CNN
F 3 "" H 1775 6450 50  0001 C CNN
	1    1775 6450
	1    0    0    1   
$EndComp
Text Label 2125 5550 0    47   ~ 0
EN
Text Label 2125 6750 0    47   ~ 0
IO0
Text Label 875  5850 2    47   ~ 0
DTR
Text Label 900  6450 2    47   ~ 0
RTS
Text Label 2115 5190 0    47   ~ 0
RXD
Text Label 2120 5260 0    47   ~ 0
TXD
Text Label 2115 5420 0    47   ~ 0
CTS
Text Label 1465 5335 2    47   ~ 0
IO13
Text Label 1460 5420 2    47   ~ 0
IO15
$Comp
L R_Small R7
U 1 1 5986CC80
P 10250 3025
F 0 "R7" V 10325 3025 50  0000 L CNN
F 1 "470Ω" V 10175 2925 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 10250 3025 50  0001 C CNN
F 3 "" H 10250 3025 50  0001 C CNN
	1    10250 3025
	0    -1   -1   0   
$EndComp
Text Label 10575 3025 0    47   ~ 0
TXD
Text Label 10575 3150 0    47   ~ 0
RXD
Text Label 9725 3175 2    47   ~ 0
DTR
Text Label 10125 3300 1    47   ~ 0
RTS
Text Label 10225 3300 1    47   ~ 0
CTS
NoConn ~ 10625 4675
NoConn ~ 9825 3375
NoConn ~ 9325 4375
NoConn ~ 9325 4475
NoConn ~ 9925 5775
Text Notes 8550 3100 0    47   ~ 0
CP2102-GM\n\n
NoConn ~ 6500 950 
$Comp
L CP1_Small C2
U 1 1 598AAB6A
P 800 3200
F 0 "C2" H 810 3270 50  0000 L CNN
F 1 "100uF-TTL" H 810 3120 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:CP_Tantalum_Case-A_EIA-3216-18_Hand" H 800 3200 50  0001 C CNN
F 3 "" H 800 3200 50  0001 C CNN
	1    800  3200
	1    0    0    -1  
$EndComp
$Comp
L R_Small R18
U 1 1 598BDB40
P 9160 2040
F 0 "R18" H 9190 2060 50  0000 L CNN
F 1 "1MΩ" V 9085 1990 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9160 2040 50  0001 C CNN
F 3 "" H 9160 2040 50  0001 C CNN
	1    9160 2040
	0    -1   1    0   
$EndComp
NoConn ~ 4640 5000
Text Label 4490 4400 2    60   ~ 0
GPIO32
Text Label 4490 4500 2    60   ~ 0
GPIO33
Text Label 5240 5660 2    47   ~ 0
IO13
Text Label 6650 4350 0    47   ~ 0
GPIO18
$Comp
L ASMT-YTC7-0AA02 LED1
U 1 1 59842EC1
P 3550 7425
F 0 "LED1" H 4550 7575 50  0000 C CNN
F 1 "ASMT-YTC7-0AA02" H 4550 7075 50  0000 C CNN
F 2 "ASMT-YTC7-0AA02:ASMT-YTC7-0AA02" H 4550 6975 50  0001 C CNN
F 3 "https://docs.broadcom.com/docs/AV02-3819EN" H 4550 6875 50  0001 C CNN
F 4 "Broadcom ASMT-YTC7-0AA02 3 RGB LED, 469 / 529 / 623 nm PLCC 6, Square Lens SMD package" H 4550 6775 50  0001 C CNN "Description"
F 5 "RS" H 4550 6675 50  0001 C CNN "Supplier_Name"
F 6 "8305097" H 4550 6575 50  0001 C CNN "RS Part Number"
F 7 "Broadcom" H 4550 6475 50  0001 C CNN "Manufacturer_Name"
F 8 "ASMT-YTC7-0AA02" H 4550 6375 50  0001 C CNN "Manufacturer_Part_Number"
F 9 "" H 4550 6275 50  0001 C CNN "Allied_Number"
F 10 "" H 4550 6175 50  0001 C CNN "Other Part Number"
F 11 "" H 5400 6075 50  0001 C CNN "Height"
	1    3550 7425
	-1   0    0    1   
$EndComp
$Comp
L R_Small R8
U 1 1 59843344
P 1300 7225
F 0 "R8" V 1350 7300 50  0000 L CNN
F 1 "40Ω" V 1350 7000 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1300 7225 50  0001 C CNN
F 3 "" H 1300 7225 50  0001 C CNN
	1    1300 7225
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R14
U 1 1 598439CE
P 1300 7325
F 0 "R14" V 1350 7150 50  0000 L CNN
F 1 "40Ω" V 1250 7400 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1300 7325 50  0001 C CNN
F 3 "" H 1300 7325 50  0001 C CNN
	1    1300 7325
	0    1    1    0   
$EndComp
$Comp
L R_Small R17
U 1 1 59843E7B
P 1300 7425
F 0 "R17" V 1350 7250 50  0000 L CNN
F 1 "240Ω" V 1250 7500 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1300 7425 50  0001 C CNN
F 3 "" H 1300 7425 50  0001 C CNN
	1    1300 7425
	0    1    1    0   
$EndComp
Text Label 1025 7325 2    47   ~ 0
GPIO17
Text Label 1025 7225 2    47   ~ 0
GPIO16
Text Label 1025 7425 2    47   ~ 0
GPIO18
$Comp
L GND #PWR14
U 1 1 5984853F
P 3675 7500
F 0 "#PWR14" H 3675 7250 50  0001 C CNN
F 1 "GND" H 3675 7350 50  0000 C CNN
F 2 "" H 3675 7500 50  0001 C CNN
F 3 "" H 3675 7500 50  0001 C CNN
	1    3675 7500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR17
U 1 1 5984A891
P 4590 5150
F 0 "#PWR17" H 4590 4900 50  0001 C CNN
F 1 "GND" H 4590 5000 50  0000 C CNN
F 2 "" H 4590 5150 50  0001 C CNN
F 3 "" H 4590 5150 50  0001 C CNN
	1    4590 5150
	1    0    0    -1  
$EndComp
$Comp
L CSD17575Q3 Q1
U 1 1 59853CE7
P 6655 2080
F 0 "Q1" H 7155 2230 50  0000 C CNN
F 1 "CSD17575Q3" H 7155 1630 50  0000 C CNN
F 2 "CSD17575Q3:CSD16327" H 7155 1530 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/csd17575q3" H 7155 1430 50  0001 C CNN
F 4 "30-V, N-Channel NexFET(TM) Power MOSFET  " H 7155 1330 50  0001 C CNN "Description"
F 5 "RS" H 7155 1230 50  0001 C CNN "Supplier_Name"
F 6 "" H 7155 1130 50  0001 C CNN "RS Part Number"
F 7 "Texas Instruments" H 7155 1030 50  0001 C CNN "Manufacturer_Name"
F 8 "CSD17575Q3" H 7155 930 50  0001 C CNN "Manufacturer_Part_Number"
F 9 "" H 7155 830 50  0001 C CNN "Allied_Number"
F 10 "" H 7155 730 50  0001 C CNN "Other Part Number"
F 11 "" H 7505 630 50  0001 C CNN "Height"
	1    6655 2080
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR20
U 1 1 598649D1
P 6595 2490
F 0 "#PWR20" H 6595 2240 50  0001 C CNN
F 1 "GND" H 6595 2340 50  0000 C CNN
F 2 "" H 6595 2490 50  0001 C CNN
F 3 "" H 6595 2490 50  0001 C CNN
	1    6595 2490
	1    0    0    -1  
$EndComp
$Comp
L CSD17575Q3 Q2
U 1 1 5985C35C
P 8865 2080
F 0 "Q2" H 9365 2230 50  0000 C CNN
F 1 "CSD17575Q3" H 9365 1630 50  0000 C CNN
F 2 "CSD17575Q3:CSD16327" H 9365 1530 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/csd17575q3" H 9365 1430 50  0001 C CNN
F 4 "30-V, N-Channel NexFET(TM) Power MOSFET  " H 9365 1330 50  0001 C CNN "Description"
F 5 "RS" H 9365 1230 50  0001 C CNN "Supplier_Name"
F 6 "" H 9365 1130 50  0001 C CNN "RS Part Number"
F 7 "Texas Instruments" H 9365 1030 50  0001 C CNN "Manufacturer_Name"
F 8 "CSD17575Q3" H 9365 930 50  0001 C CNN "Manufacturer_Part_Number"
F 9 "" H 9365 830 50  0001 C CNN "Allied_Number"
F 10 "" H 9365 730 50  0001 C CNN "Other Part Number"
F 11 "" H 9715 630 50  0001 C CNN "Height"
	1    8865 2080
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR29
U 1 1 59888474
P 9005 1225
F 0 "#PWR29" H 9005 975 50  0001 C CNN
F 1 "GND" H 9005 1075 50  0000 C CNN
F 2 "" H 9005 1225 50  0001 C CNN
F 3 "" H 9005 1225 50  0001 C CNN
	1    9005 1225
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG3
U 1 1 5988D03E
P 6330 2510
F 0 "#FLG3" H 6330 2585 50  0001 C CNN
F 1 "PWR_FLAG" H 6330 2660 50  0000 C CNN
F 2 "" H 6330 2510 50  0001 C CNN
F 3 "" H 6330 2510 50  0001 C CNN
	1    6330 2510
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG5
U 1 1 59898C90
P 10575 2505
F 0 "#FLG5" H 10575 2580 50  0001 C CNN
F 1 "PWR_FLAG" H 10575 2655 50  0000 C CNN
F 2 "" H 10575 2505 50  0001 C CNN
F 3 "" H 10575 2505 50  0001 C CNN
	1    10575 2505
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 J2
U 1 1 5989FB25
P 2250 4425
F 0 "J2" H 2300 4275 50  0000 C CNN
F 1 "B2B-PH-K-S(LF)(SN)" H 2025 4650 50  0000 C CNN
F 2 "Connectors_JST:JST_PH_S2B-PH-K_02x2.00mm_Angled" H 2250 4425 50  0001 C CNN
F 3 "" H 2250 4425 50  0001 C CNN
	1    2250 4425
	-1   0    0    1   
$EndComp
Wire Wire Line
	800  2950 1450 2950
Wire Wire Line
	800  2850 800  3100
Wire Wire Line
	1250 2950 1250 3100
Connection ~ 1250 2950
Wire Wire Line
	800  3300 800  3500
Wire Wire Line
	1250 3300 1250 3400
Wire Wire Line
	1250 3400 800  3400
Connection ~ 800  3400
Connection ~ 800  2950
Wire Wire Line
	4640 3800 4490 3800
Wire Notes Line
	600  3800 1920 3800
Wire Notes Line
	600  3800 600  2510
Wire Wire Line
	4640 3900 4490 3900
Wire Wire Line
	2450 2700 2450 2800
Wire Wire Line
	2450 3000 2450 3300
Wire Wire Line
	2450 3500 2450 3600
Wire Wire Line
	2450 3150 2850 3150
Connection ~ 2450 3150
Wire Notes Line
	2250 2500 2250 3850
Wire Notes Line
	2250 3850 3250 3850
Wire Notes Line
	3250 3850 3250 2500
Wire Notes Line
	3250 2500 2250 2500
Wire Wire Line
	6490 4050 7100 4050
Wire Wire Line
	6490 3750 7100 3750
Wire Wire Line
	6490 4950 6640 4950
Wire Wire Line
	6640 4950 6640 5250
Wire Wire Line
	6490 5050 6640 5050
Connection ~ 6640 5050
Wire Wire Line
	10270 1280 10270 1125
Wire Wire Line
	10630 1125 10755 1125
Wire Wire Line
	3240 1135 3240 1290
Wire Wire Line
	2940 1550 2940 1640
Wire Wire Line
	740  1540 740  1670
Wire Wire Line
	1090 1990 1655 1990
Wire Wire Line
	1090 1990 1090 1890
Wire Wire Line
	1855 1990 1945 1990
Wire Wire Line
	2245 1990 2400 1990
Wire Wire Line
	6490 4850 7350 4850
Wire Wire Line
	6040 5450 6040 5520
Wire Wire Line
	6040 5720 6040 5830
Wire Wire Line
	10270 1480 10270 1780
Wire Wire Line
	10520 1580 10520 1800
Wire Wire Line
	10270 2440 10270 1980
Wire Wire Line
	10520 2440 10520 2000
Wire Notes Line
	600  600  600  2300
Wire Notes Line
	600  2300 3500 2300
Wire Notes Line
	3500 2300 3500 600 
Wire Notes Line
	3500 600  600  600 
Wire Notes Line
	10900 600  7400 600 
Wire Wire Line
	5940 5450 5940 5660
Wire Wire Line
	1465 5260 2120 5260
Wire Wire Line
	1460 5190 2115 5190
Wire Wire Line
	1465 5335 2115 5335
Wire Wire Line
	4500 4900 4640 4900
Wire Wire Line
	4490 4700 4640 4700
Wire Wire Line
	4490 4800 4640 4800
Connection ~ 6600 3750
Wire Wire Line
	6600 3400 6600 3450
Wire Wire Line
	6600 3650 6600 3750
Wire Wire Line
	6900 3650 6900 4050
Connection ~ 6900 4050
Wire Wire Line
	6900 3400 6900 3450
Wire Wire Line
	10020 1580 10520 1580
Wire Wire Line
	10020 1175 10020 1480
Wire Wire Line
	740  1230 1460 1230
Wire Wire Line
	1340 1035 1340 1230
Wire Wire Line
	740  1230 740  1340
Connection ~ 1340 1230
Wire Wire Line
	1090 1280 1090 1230
Connection ~ 1090 1230
Wire Wire Line
	1460 1430 1460 1990
Connection ~ 1460 1990
Wire Wire Line
	2460 1230 3240 1230
Wire Wire Line
	2940 1350 2940 1230
Connection ~ 2940 1230
Wire Wire Line
	2460 1330 2520 1330
Wire Wire Line
	2520 1330 2520 1300
Wire Wire Line
	2520 1300 2590 1300
Wire Wire Line
	2460 1430 2520 1430
Wire Wire Line
	2520 1430 2520 1500
Wire Wire Line
	2520 1500 2590 1500
Wire Wire Line
	10020 1680 10065 1680
Wire Wire Line
	10065 1680 10065 2440
Connection ~ 10270 1580
Wire Wire Line
	6490 3950 6625 3950
Wire Wire Line
	6490 3850 6625 3850
Wire Wire Line
	4640 4600 4490 4600
Wire Wire Line
	10270 1125 10330 1125
Wire Notes Line
	600  2500 1920 2500
Wire Notes Line
	1920 2500 1920 3800
Wire Wire Line
	1190 4075 1250 4075
Wire Wire Line
	1020 4710 1020 4750
Wire Notes Line
	600  3875 600  4925
Wire Notes Line
	600  4925 1600 4925
Wire Notes Line
	1600 4925 1600 3875
Wire Notes Line
	1600 3875 600  3875
Wire Wire Line
	790  4675 790  4710
Wire Wire Line
	790  4710 1020 4710
Wire Wire Line
	890  4675 890  4710
Connection ~ 890  4710
Wire Wire Line
	2115 5420 1460 5420
Wire Wire Line
	6490 4650 6650 4650
Wire Wire Line
	6490 4550 6650 4550
Wire Wire Line
	2550 1500 2550 1550
Connection ~ 2550 1500
Connection ~ 3240 1230
Wire Wire Line
	3900 950  4525 950 
Wire Wire Line
	3900 950  3900 900 
Wire Wire Line
	4125 1050 4125 950 
Connection ~ 4125 950 
Wire Wire Line
	4525 1150 4400 1150
Wire Wire Line
	4400 1150 4400 950 
Connection ~ 4400 950 
Wire Wire Line
	4125 1250 4125 1350
Wire Wire Line
	5350 950  5800 950 
Wire Wire Line
	5600 1050 5600 950 
Connection ~ 5600 950 
Wire Wire Line
	5350 1150 5450 1150
Wire Wire Line
	5600 1250 5600 1375
Wire Wire Line
	5450 1150 5450 1325
Wire Wire Line
	5450 1325 5600 1325
Connection ~ 5600 1325
Wire Notes Line
	3750 600  3750 1650
Wire Notes Line
	3750 1650 6775 1650
Wire Notes Line
	3750 600  6775 600 
Wire Wire Line
	9625 3375 9625 3325
Wire Wire Line
	9625 3325 9375 3325
Wire Wire Line
	9375 3325 9375 3425
Wire Wire Line
	9325 4575 8925 4575
Wire Wire Line
	8925 4575 8925 4625
Wire Wire Line
	1190 4275 1275 4275
Wire Wire Line
	1190 4375 1275 4375
Wire Wire Line
	9225 4675 9325 4675
Wire Wire Line
	9225 4775 9325 4775
Wire Wire Line
	9625 5825 9625 5775
Wire Wire Line
	9175 5825 9625 5825
Wire Wire Line
	9175 5700 9175 5900
Wire Wire Line
	9475 5900 9475 5825
Connection ~ 9475 5825
Wire Wire Line
	9175 6100 9175 6175
Wire Wire Line
	9050 6175 9475 6175
Wire Wire Line
	9475 6175 9475 6100
Connection ~ 9175 6175
Connection ~ 9175 5825
Wire Wire Line
	8675 4875 9325 4875
Wire Wire Line
	8675 4750 8675 4950
Wire Wire Line
	8950 4950 8950 4875
Connection ~ 8950 4875
Wire Wire Line
	9325 4975 9175 4975
Wire Wire Line
	9175 4975 9175 4875
Connection ~ 9175 4875
Wire Wire Line
	8950 5150 8950 5225
Wire Wire Line
	8950 5225 8675 5225
Wire Wire Line
	8675 5150 8675 5300
Connection ~ 8675 5225
Connection ~ 8675 4875
Wire Wire Line
	900  6450 1250 6450
Wire Wire Line
	875  5850 1250 5850
Wire Wire Line
	1450 5850 1575 5850
Wire Wire Line
	1450 6450 1575 6450
Wire Wire Line
	1875 6250 975  6250
Wire Wire Line
	975  6250 975  5850
Connection ~ 975  5850
Wire Wire Line
	1875 6050 1150 6050
Wire Wire Line
	1150 6050 1150 6450
Connection ~ 1150 6450
Wire Wire Line
	1875 5650 1875 5550
Wire Wire Line
	1875 6650 1875 6750
Wire Wire Line
	1875 5550 2125 5550
Wire Wire Line
	1875 6750 2125 6750
Wire Notes Line
	600  5050 2500 5050
Wire Notes Line
	2500 5050 2500 6850
Wire Notes Line
	2500 6850 600  6850
Wire Notes Line
	600  6850 600  5050
Wire Wire Line
	9725 3375 9725 3175
Wire Wire Line
	9925 3375 9925 3025
Wire Wire Line
	9925 3025 10150 3025
Wire Wire Line
	10350 3025 10575 3025
Wire Wire Line
	10025 3375 10025 3150
Wire Wire Line
	10025 3150 10575 3150
Wire Wire Line
	10125 3375 10125 3300
Wire Wire Line
	10225 3375 10225 3300
Wire Notes Line
	8500 2900 8500 6400
Wire Notes Line
	8500 6400 11000 6400
Wire Notes Line
	11000 6400 11000 2900
Wire Notes Line
	11000 2900 8500 2900
Wire Wire Line
	5800 1050 5800 1300
Wire Wire Line
	6625 1300 6625 1225
Wire Notes Line
	6775 600  6775 1650
Wire Notes Line
	6120 2700 11000 2700
Wire Wire Line
	4640 4500 4490 4500
Wire Wire Line
	4640 4400 4490 4400
Wire Wire Line
	5240 5450 5240 5660
Wire Wire Line
	6490 4350 6650 4350
Wire Wire Line
	1550 7225 1400 7225
Wire Wire Line
	1550 7325 1400 7325
Wire Wire Line
	1550 7425 1400 7425
Wire Wire Line
	1200 7225 1025 7225
Wire Wire Line
	1200 7325 1025 7325
Wire Wire Line
	1200 7425 1025 7425
Wire Wire Line
	3550 7225 3675 7225
Wire Wire Line
	3675 7225 3675 7500
Wire Wire Line
	3550 7325 3675 7325
Connection ~ 3675 7325
Wire Wire Line
	3550 7425 3675 7425
Connection ~ 3675 7425
Wire Wire Line
	4640 5100 4590 5100
Wire Wire Line
	4590 5100 4590 5150
Wire Wire Line
	7655 2080 7715 2080
Wire Wire Line
	7655 2380 7865 2380
Wire Wire Line
	7715 2280 7655 2280
Wire Wire Line
	7655 2180 7715 2180
Wire Wire Line
	6655 2080 6595 2080
Wire Wire Line
	6595 2080 6595 2490
Wire Wire Line
	6595 2280 6655 2280
Wire Wire Line
	6655 2180 6595 2180
Connection ~ 6595 2180
Wire Wire Line
	7865 2080 7805 2080
Wire Wire Line
	7805 2280 7865 2280
Wire Wire Line
	7865 2180 7805 2180
Wire Wire Line
	8865 2080 8925 2080
Wire Wire Line
	8925 2280 8865 2280
Wire Wire Line
	8865 2180 8925 2180
Connection ~ 6595 2280
Wire Wire Line
	6515 2380 6655 2380
Wire Wire Line
	8925 2080 8925 2440
Connection ~ 8925 2180
Connection ~ 8925 2280
Wire Wire Line
	9020 1680 8985 1680
Wire Wire Line
	8985 1680 8985 2380
Wire Wire Line
	8985 2380 8865 2380
Wire Wire Line
	7715 2080 7715 2380
Connection ~ 7715 2180
Connection ~ 7715 2280
Connection ~ 7715 2380
Wire Wire Line
	7805 2080 7805 2380
Connection ~ 7805 2380
Connection ~ 7805 2180
Connection ~ 7805 2280
Wire Wire Line
	8925 2440 10685 2440
Connection ~ 10065 2440
Connection ~ 10270 2440
Connection ~ 10520 2440
Wire Wire Line
	9020 1580 8855 1580
Wire Wire Line
	8855 1580 8855 1870
Wire Wire Line
	8855 1870 6515 1870
Wire Wire Line
	6515 1870 6515 2380
Wire Wire Line
	6435 2065 6515 2065
Connection ~ 6515 2065
Wire Wire Line
	6185 2445 6595 2445
Connection ~ 6595 2445
Wire Wire Line
	6235 2065 6185 2065
Wire Wire Line
	6185 2065 6185 2445
Wire Wire Line
	9060 2040 8985 2040
Connection ~ 8985 2040
Wire Wire Line
	9260 2040 9310 2040
Wire Wire Line
	9310 2040 9310 2440
Connection ~ 9310 2440
Wire Wire Line
	10020 1175 9655 1175
Wire Wire Line
	9455 1175 9005 1175
Wire Wire Line
	9005 1175 9005 1225
Wire Notes Line
	7390 600  6850 600 
Wire Notes Line
	6850 600  6850 1780
Wire Notes Line
	6850 1780 6120 1780
Wire Notes Line
	6120 1780 6120 2700
Wire Notes Line
	11000 2700 11000 600 
Wire Notes Line
	11000 600  10880 600 
Wire Wire Line
	6330 2510 6330 2445
Connection ~ 6330 2445
Wire Wire Line
	10575 2505 10575 2440
Connection ~ 10575 2440
$Comp
L +BATT #PWR8
U 1 1 598A2C1F
P 2525 4275
F 0 "#PWR8" H 2525 4125 50  0001 C CNN
F 1 "+BATT" H 2525 4415 50  0000 C CNN
F 2 "" H 2525 4275 50  0001 C CNN
F 3 "" H 2525 4275 50  0001 C CNN
	1    2525 4275
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 4375 2525 4375
Wire Wire Line
	2525 4375 2525 4275
Text Label 2625 4575 0    60   ~ 0
-BATT
Wire Wire Line
	2450 4475 2525 4475
Wire Wire Line
	2525 4475 2525 4575
Wire Wire Line
	2525 4575 2625 4575
Wire Notes Line
	2025 4700 2025 4050
Wire Notes Line
	2025 4050 2950 4050
Wire Notes Line
	2950 4050 2950 4700
Wire Notes Line
	2950 4700 2025 4700
Wire Wire Line
	3150 5350 3275 5350
Wire Wire Line
	3150 5450 3275 5450
Wire Wire Line
	3150 5550 3275 5550
Wire Wire Line
	3150 5650 3275 5650
Wire Wire Line
	3150 5750 3275 5750
Text Label 3265 6225 0    60   ~ 0
SCL
Text Label 3260 6325 0    60   ~ 0
SDA
Text Label 3275 5550 0    60   ~ 0
GPIO25
Text Label 3275 5450 0    60   ~ 0
GPIO26
Text Label 3275 5350 0    60   ~ 0
GPIO27
Text Label 3275 5250 0    60   ~ 0
GPIO14
Text Label 3275 5750 0    60   ~ 0
GPIO32
Text Label 3275 5650 0    60   ~ 0
GPIO33
Wire Wire Line
	3150 5250 3275 5250
$Comp
L GND #PWR12
U 1 1 598BE5AB
P 3550 6455
F 0 "#PWR12" H 3550 6205 50  0001 C CNN
F 1 "GND" H 3550 6305 50  0000 C CNN
F 2 "" H 3550 6455 50  0001 C CNN
F 3 "" H 3550 6455 50  0001 C CNN
	1    3550 6455
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR13
U 1 1 598BEECD
P 3575 6125
F 0 "#PWR13" H 3575 5975 50  0001 C CNN
F 1 "+3.3V" H 3575 6265 50  0000 C CNN
F 2 "" H 3575 6125 50  0001 C CNN
F 3 "" H 3575 6125 50  0001 C CNN
	1    3575 6125
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 J3
U 1 1 59878279
P 2950 6275
F 0 "J3" H 2950 6525 50  0000 C CNN
F 1 "CONN_01X04" V 3050 6275 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 2950 6275 50  0001 C CNN
F 3 "" H 2950 6275 50  0001 C CNN
	1    2950 6275
	-1   0    0    1   
$EndComp
Wire Notes Line
	2775 5125 2775 6675
Wire Notes Line
	2775 6675 3700 6675
Wire Notes Line
	3700 6675 3700 5125
Text Label 4490 4200 2    60   ~ 0
GPIO34
Text Label 4490 4300 2    60   ~ 0
GPIO35
Wire Wire Line
	4640 4300 4490 4300
Wire Wire Line
	4640 4200 4490 4200
$Comp
L CONN_01X08 J4
U 1 1 5989AB2B
P 2950 5600
F 0 "J4" H 2950 6050 50  0000 C CNN
F 1 "CONN_01X08" V 3050 5600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 2950 5600 50  0001 C CNN
F 3 "" H 2950 5600 50  0001 C CNN
	1    2950 5600
	-1   0    0    1   
$EndComp
Text Label 3300 5950 0    60   ~ 0
GPIO34
Text Label 3300 5850 0    60   ~ 0
GPIO35
Wire Wire Line
	3150 5850 3300 5850
Wire Wire Line
	3150 5950 3300 5950
Wire Notes Line
	3700 5125 2775 5125
Wire Wire Line
	3260 6325 3150 6325
Wire Wire Line
	3265 6225 3150 6225
Wire Wire Line
	3575 6125 3150 6125
Wire Wire Line
	3550 6455 3550 6425
Wire Wire Line
	3550 6425 3150 6425
Wire Notes Line
	750  7000 3750 7000
Wire Notes Line
	3750 7000 3750 7725
Wire Notes Line
	3750 7725 750  7725
Wire Notes Line
	750  7725 750  7000
Text Notes 850  7100 0    60   ~ 0
RGB - LED
$Comp
L LED D1
U 1 1 59967FE7
P 2095 1990
F 0 "D1" H 2095 2090 50  0000 C CNN
F 1 "LED" H 2095 1890 50  0000 C CNN
F 2 "" H 2095 1990 50  0001 C CNN
F 3 "" H 2095 1990 50  0001 C CNN
	1    2095 1990
	-1   0    0    1   
$EndComp
$Comp
L LED D5
U 1 1 599680E3
P 1090 1430
F 0 "D5" H 1090 1530 50  0000 C CNN
F 1 "LED" H 1090 1330 50  0000 C CNN
F 2 "" H 1090 1430 50  0001 C CNN
F 3 "" H 1090 1430 50  0001 C CNN
	1    1090 1430
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1090 1690 1090 1580
$Comp
L USB_OTG J1
U 1 1 5996A101
P 890 4275
F 0 "J1" H 690 4725 50  0000 L CNN
F 1 "USB_OTG" H 690 4625 50  0000 L CNN
F 2 "" H 1040 4225 50  0001 C CNN
F 3 "" H 1040 4225 50  0001 C CNN
	1    890  4275
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR9
U 1 1 5996C51C
P 2550 1550
F 0 "#PWR9" H 2550 1300 50  0001 C CNN
F 1 "GND" H 2550 1400 50  0000 C CNN
F 2 "" H 2550 1550 50  0001 C CNN
F 3 "" H 2550 1550 50  0001 C CNN
	1    2550 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 1300 6625 1300
$Comp
L +3.3V #PWR25
U 1 1 59A01036
P 7200 4450
F 0 "#PWR25" H 7200 4300 50  0001 C CNN
F 1 "+3.3V" H 7200 4590 50  0000 C CNN
F 2 "" H 7200 4450 50  0001 C CNN
F 3 "" H 7200 4450 50  0001 C CNN
	1    7200 4450
	1    0    0    -1  
$EndComp
$Comp
L R_Small R19
U 1 1 59A0170A
P 7200 4650
F 0 "R19" H 7230 4670 50  0000 L CNN
F 1 "10kΩ" H 7230 4610 50  0000 L CNN
F 2 "" H 7200 4650 50  0001 C CNN
F 3 "" H 7200 4650 50  0001 C CNN
	1    7200 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 4450 7200 4550
Wire Wire Line
	7200 4750 7200 4850
Connection ~ 7200 4850
$EndSCHEMATC
