EESchema Schematic File Version 4
LIBS:UAV2021_ver2-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Regulator_Linear:MIC5219-3.3YM5 U?
U 1 1 60DCD7CA
P 5400 3800
AR Path="/60DCD7CA" Ref="U?"  Part="1" 
AR Path="/60DCC9C2/60DCD7CA" Ref="U?"  Part="1" 
F 0 "U?" H 5400 4142 50  0000 C CNN
F 1 "MIC5219-3.3YM5" H 5400 4051 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 5400 4125 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/MIC5219-500mA-Peak-Output-LDO-Regulator-DS20006021A.pdf" H 5400 3800 50  0001 C CNN
	1    5400 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3700 5000 3700
Wire Wire Line
	5100 3800 5000 3800
Wire Wire Line
	5000 3800 5000 3700
Connection ~ 5000 3700
Wire Wire Line
	5000 3700 4600 3700
Wire Wire Line
	5700 3800 5800 3800
Wire Wire Line
	5800 3800 5800 3900
$Comp
L power:GND #PWR?
U 1 1 60DCDEF9
P 5400 4350
F 0 "#PWR?" H 5400 4100 50  0001 C CNN
F 1 "GND" H 5405 4177 50  0000 C CNN
F 2 "" H 5400 4350 50  0001 C CNN
F 3 "" H 5400 4350 50  0001 C CNN
	1    5400 4350
	1    0    0    -1  
$EndComp
$Comp
L power:+7.5V #PWR?
U 1 1 60DCE5FC
P 4600 3500
F 0 "#PWR?" H 4600 3350 50  0001 C CNN
F 1 "+7.5V" H 4615 3673 50  0000 C CNN
F 2 "" H 4600 3500 50  0001 C CNN
F 3 "" H 4600 3500 50  0001 C CNN
	1    4600 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 60DCF0DC
P 6250 3500
F 0 "#PWR?" H 6250 3350 50  0001 C CNN
F 1 "+3.3V" H 6265 3673 50  0000 C CNN
F 2 "" H 6250 3500 50  0001 C CNN
F 3 "" H 6250 3500 50  0001 C CNN
	1    6250 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3700 6250 3500
Wire Wire Line
	4600 3500 4600 3700
$Comp
L Device:C C?
U 1 1 60DCFDDF
P 5800 4050
F 0 "C?" H 5915 4096 50  0000 L CNN
F 1 "470p" H 5915 4005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5838 3900 50  0001 C CNN
F 3 "~" H 5800 4050 50  0001 C CNN
	1    5800 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60DD04D2
P 6250 4050
F 0 "C?" H 6365 4096 50  0000 L CNN
F 1 "2.2u" H 6365 4005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6288 3900 50  0001 C CNN
F 3 "~" H 6250 4050 50  0001 C CNN
	1    6250 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3900 6250 3700
Connection ~ 6250 3700
$Comp
L power:GND #PWR?
U 1 1 60DD14BD
P 5800 4350
F 0 "#PWR?" H 5800 4100 50  0001 C CNN
F 1 "GND" H 5805 4177 50  0000 C CNN
F 2 "" H 5800 4350 50  0001 C CNN
F 3 "" H 5800 4350 50  0001 C CNN
	1    5800 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60DD17E5
P 6250 4350
F 0 "#PWR?" H 6250 4100 50  0001 C CNN
F 1 "GND" H 6255 4177 50  0000 C CNN
F 2 "" H 6250 4350 50  0001 C CNN
F 3 "" H 6250 4350 50  0001 C CNN
	1    6250 4350
	1    0    0    -1  
$EndComp
Text Notes 6550 4100 0    50   ~ 0
When not use BP, 0.1uF
Wire Wire Line
	5700 3700 6250 3700
Wire Wire Line
	5400 4100 5400 4350
Wire Wire Line
	5800 4200 5800 4350
Wire Wire Line
	6250 4200 6250 4350
Text Notes 3950 3100 0    50   ~ 0
Power circuit
$EndSCHEMATC
