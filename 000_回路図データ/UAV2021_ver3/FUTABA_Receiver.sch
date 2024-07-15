EESchema Schematic File Version 4
LIBS:UAV2021_ver2-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
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
L Device:R R?
U 1 1 60DC6C2E
P 5050 2900
F 0 "R?" H 5120 2946 50  0000 L CNN
F 1 "10k" H 5120 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4980 2900 50  0001 C CNN
F 3 "~" H 5050 2900 50  0001 C CNN
	1    5050 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 2750 5050 2650
Wire Wire Line
	5050 3250 5050 3150
Wire Wire Line
	5050 3150 5500 3150
Connection ~ 5050 3150
Wire Wire Line
	5050 3150 5050 3050
Wire Wire Line
	4750 3450 4650 3450
$Comp
L Device:R R?
U 1 1 60DC7EAC
P 4500 3450
F 0 "R?" V 4293 3450 50  0000 C CNN
F 1 "3.3k" V 4384 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4430 3450 50  0001 C CNN
F 3 "~" H 4500 3450 50  0001 C CNN
	1    4500 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 3450 4250 3450
$Comp
L power:+3.3V #PWR?
U 1 1 60DC8DFF
P 5050 2650
F 0 "#PWR?" H 5050 2500 50  0001 C CNN
F 1 "+3.3V" H 5065 2823 50  0000 C CNN
F 2 "" H 5050 2650 50  0001 C CNN
F 3 "" H 5050 2650 50  0001 C CNN
	1    5050 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60DC8709
P 5050 3900
F 0 "#PWR?" H 5050 3650 50  0001 C CNN
F 1 "GND" H 5055 3727 50  0000 C CNN
F 2 "" H 5050 3900 50  0001 C CNN
F 3 "" H 5050 3900 50  0001 C CNN
	1    5050 3900
	1    0    0    -1  
$EndComp
Text HLabel 4250 3450 0    50   Input ~ 0
RCV_IN
Text HLabel 5500 3150 2    50   Output ~ 0
RCV_OUT
$Comp
L Transistor_FET:2N7002K Q?
U 1 1 61144C43
P 4950 3450
F 0 "Q?" H 5156 3496 50  0000 L CNN
F 1 "2N7002K" H 5156 3405 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5150 3375 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30896.pdf" H 4950 3450 50  0001 L CNN
	1    4950 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3650 5050 3900
Text Notes 3450 2350 0    50   ~ 0
SBUS receiver circuit
$EndSCHEMATC
