EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 6750 850  0    102  ~ 20
DAQ system
$Comp
L Device:R_POT RV1
U 1 1 60A87F25
P 6450 1900
F 0 "RV1" H 6650 2000 50  0000 R CNN
F 1 "10k" H 6400 1700 50  0000 R CNN
F 2 "" H 6450 1900 50  0001 C CNN
F 3 "~" H 6450 1900 50  0001 C CNN
	1    6450 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 60A88AE0
P 6900 1900
F 0 "R4" V 6693 1900 50  0000 C CNN
F 1 "100k" V 6784 1900 50  0000 C CNN
F 2 "" V 6830 1900 50  0001 C CNN
F 3 "~" H 6900 1900 50  0001 C CNN
	1    6900 1900
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 60A88F1A
P 7250 2100
F 0 "C1" H 7365 2146 50  0000 L CNN
F 1 "0.1u" H 7365 2055 50  0000 L CNN
F 2 "" H 7288 1950 50  0001 C CNN
F 3 "~" H 7250 2100 50  0001 C CNN
	1    7250 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 60A89C7E
P 7250 2400
F 0 "#PWR08" H 7250 2150 50  0001 C CNN
F 1 "GND" H 7255 2227 50  0000 C CNN
F 2 "" H 7250 2400 50  0001 C CNN
F 3 "" H 7250 2400 50  0001 C CNN
	1    7250 2400
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM358 U2
U 1 1 60A8C442
P 7900 2000
F 0 "U2" H 7900 2367 50  0000 C CNN
F 1 "LM358" H 7900 2276 50  0000 C CNN
F 2 "" H 7900 2000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 7900 2000 50  0001 C CNN
	1    7900 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 1900 7250 1900
Wire Wire Line
	7250 1950 7250 1900
Connection ~ 7250 1900
Wire Wire Line
	7250 1900 7600 1900
Wire Wire Line
	7250 2250 7250 2400
Wire Wire Line
	6750 1900 6600 1900
$Comp
L power:GND #PWR07
U 1 1 60A8FFE8
P 6450 2400
F 0 "#PWR07" H 6450 2150 50  0001 C CNN
F 1 "GND" H 6455 2227 50  0000 C CNN
F 2 "" H 6450 2400 50  0001 C CNN
F 3 "" H 6450 2400 50  0001 C CNN
	1    6450 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 2000 8450 2000
Wire Wire Line
	8450 2000 8450 2450
Wire Wire Line
	8450 2450 7600 2450
Wire Wire Line
	7600 2450 7600 2100
Text Notes 8300 1950 0    63   ~ 13
MCU ADC
Wire Wire Line
	6450 2050 6450 2400
Text Notes 5700 1550 0    71   ~ 14
Angle sensor
$Comp
L Device:C C2
U 1 1 60C533E1
P 5950 1850
F 0 "C2" H 6065 1896 50  0000 L CNN
F 1 "0.1u" H 6065 1805 50  0000 L CNN
F 2 "" H 5988 1700 50  0001 C CNN
F 3 "~" H 5950 1850 50  0001 C CNN
	1    5950 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 1700 5950 1600
Wire Wire Line
	5950 1600 6450 1600
Wire Wire Line
	6450 1600 6450 1750
$Comp
L power:GND #PWR09
U 1 1 60C5A65E
P 5950 2150
F 0 "#PWR09" H 5950 1900 50  0001 C CNN
F 1 "GND" H 5955 1977 50  0000 C CNN
F 2 "" H 5950 2150 50  0001 C CNN
F 3 "" H 5950 2150 50  0001 C CNN
	1    5950 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 2000 5950 2150
$Comp
L power:GND #PWR03
U 1 1 60C5DA75
P 2900 3850
F 0 "#PWR03" H 2900 3600 50  0001 C CNN
F 1 "GND" H 2905 3677 50  0000 C CNN
F 2 "" H 2900 3850 50  0001 C CNN
F 3 "" H 2900 3850 50  0001 C CNN
	1    2900 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 60C77EC1
P 1600 1650
F 0 "#PWR01" H 1600 1500 50  0001 C CNN
F 1 "+3.3V" H 1615 1823 50  0000 C CNN
F 2 "" H 1600 1650 50  0001 C CNN
F 3 "" H 1600 1650 50  0001 C CNN
	1    1600 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60C78C11
P 1600 2000
F 0 "R1" H 1670 2046 50  0000 L CNN
F 1 "10k" H 1670 1955 50  0000 L CNN
F 2 "" V 1530 2000 50  0001 C CNN
F 3 "~" H 1600 2000 50  0001 C CNN
	1    1600 2000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR04
U 1 1 60C7AC0B
P 2800 1200
F 0 "#PWR04" H 2800 1050 50  0001 C CNN
F 1 "+3.3V" H 2815 1373 50  0000 C CNN
F 2 "" H 2800 1200 50  0001 C CNN
F 3 "" H 2800 1200 50  0001 C CNN
	1    2800 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 60C7F40B
P 3400 1200
F 0 "#PWR05" H 3400 1050 50  0001 C CNN
F 1 "+3.3V" H 3450 1350 50  0000 L CNN
F 2 "" H 3400 1200 50  0001 C CNN
F 3 "" H 3400 1200 50  0001 C CNN
	1    3400 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3000 3850 3000
Wire Wire Line
	3850 3000 3850 3100
Wire Wire Line
	3850 3100 3700 3100
Wire Wire Line
	3700 2800 3850 2800
Wire Wire Line
	3850 2800 3850 2900
Wire Wire Line
	3850 2900 3700 2900
Wire Wire Line
	3700 2300 3850 2300
Wire Wire Line
	3850 2300 3850 2400
Wire Wire Line
	3850 2400 3700 2400
Wire Wire Line
	3700 2500 3850 2500
Wire Wire Line
	3850 2500 3850 2600
Wire Wire Line
	3850 2600 3700 2600
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 60C8AB19
P 4300 2400
F 0 "J1" H 4380 2392 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 4380 2301 50  0000 L CNN
F 2 "" H 4300 2400 50  0001 C CNN
F 3 "~" H 4300 2400 50  0001 C CNN
	1    4300 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2400 3850 2400
Connection ~ 3850 2400
Wire Wire Line
	4100 2500 3850 2500
Connection ~ 3850 2500
$Comp
L power:+3.3V #PWR02
U 1 1 60C90BBA
P 1600 2700
F 0 "#PWR02" H 1600 2550 50  0001 C CNN
F 1 "+3.3V" H 1450 2850 50  0000 L CNN
F 2 "" H 1600 2700 50  0001 C CNN
F 3 "" H 1600 2700 50  0001 C CNN
	1    1600 2700
	1    0    0    -1  
$EndComp
Text Notes 2650 850  0    102  ~ 20
Motor Driver \n
$Comp
L Driver_Motor:TB6612FNG U1
U 1 1 60C512DF
P 3100 2700
F 0 "U1" H 2900 3600 50  0000 C CNN
F 1 "TB6612FNG" H 3750 1950 50  0000 C CNN
F 2 "Package_SO:SSOP-24_5.3x8.2mm_P0.65mm" H 3550 3300 50  0001 C CNN
F 3 "https://toshiba.semicon-storage.com/us/product/linear/motordriver/detail.TB6612FNG.html" H 3550 3300 50  0001 C CNN
	1    3100 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1700 3300 1700
Connection ~ 3300 1700
Wire Wire Line
	2150 2900 2150 3700
Wire Wire Line
	2150 3700 2500 3700
Wire Wire Line
	2150 2900 2500 2900
Wire Wire Line
	3400 3700 3300 3700
Wire Wire Line
	3300 3700 3200 3700
Connection ~ 3300 3700
Wire Wire Line
	3200 3700 3100 3700
Connection ~ 3200 3700
Wire Wire Line
	3100 3700 2900 3700
Connection ~ 3100 3700
Connection ~ 2800 3700
Wire Wire Line
	2900 3850 2900 3700
Connection ~ 2900 3700
Wire Wire Line
	2900 3700 2800 3700
Text Notes 4300 2400 0    79   ~ 16
Motor\n\n
$Comp
L power:+3.3V #PWR011
U 1 1 60CDF903
P 6450 1450
F 0 "#PWR011" H 6450 1300 50  0001 C CNN
F 1 "+3.3V" H 6465 1623 50  0000 C CNN
F 2 "" H 6450 1450 50  0001 C CNN
F 3 "" H 6450 1450 50  0001 C CNN
	1    6450 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 1450 6450 1600
Connection ~ 6450 1600
Text Notes 1900 2550 0    63   ~ 13
MCU PWM
Wire Wire Line
	2500 3000 2500 3100
Wire Wire Line
	2500 3100 2500 3700
Connection ~ 2500 3100
Connection ~ 2500 3700
Wire Wire Line
	2500 3700 2800 3700
Wire Wire Line
	3300 1700 3400 1700
$Comp
L Device:C C3
U 1 1 60CE61C2
P 2500 1400
F 0 "C3" H 2615 1446 50  0000 L CNN
F 1 "0.1u" H 2615 1355 50  0000 L CNN
F 2 "" H 2538 1250 50  0001 C CNN
F 3 "~" H 2500 1400 50  0001 C CNN
	1    2500 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1200 3400 1250
Connection ~ 3400 1700
$Comp
L Device:C C5
U 1 1 60CEA15C
P 3600 1400
F 0 "C5" H 3715 1446 50  0000 L CNN
F 1 "0.1u" H 3715 1355 50  0000 L CNN
F 2 "" H 3638 1250 50  0001 C CNN
F 3 "~" H 3600 1400 50  0001 C CNN
	1    3600 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 1250 3400 1250
Connection ~ 3400 1250
Wire Wire Line
	3400 1250 3400 1700
$Comp
L power:GND #PWR010
U 1 1 60CEB565
P 3600 1650
F 0 "#PWR010" H 3600 1400 50  0001 C CNN
F 1 "GND" H 3605 1477 50  0000 C CNN
F 2 "" H 3600 1650 50  0001 C CNN
F 3 "" H 3600 1650 50  0001 C CNN
	1    3600 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 1550 3600 1600
$Comp
L power:GND #PWR06
U 1 1 60CECC24
P 2500 1650
F 0 "#PWR06" H 2500 1400 50  0001 C CNN
F 1 "GND" H 2505 1477 50  0000 C CNN
F 2 "" H 2500 1650 50  0001 C CNN
F 3 "" H 2500 1650 50  0001 C CNN
	1    2500 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C6
U 1 1 60CEE79E
P 4100 1400
F 0 "C6" H 4215 1446 50  0000 L CNN
F 1 "10u" H 4215 1355 50  0000 L CNN
F 2 "" H 4100 1400 50  0001 C CNN
F 3 "~" H 4100 1400 50  0001 C CNN
	1    4100 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C4
U 1 1 60CEED30
P 2050 1400
F 0 "C4" H 2165 1446 50  0000 L CNN
F 1 "10u" H 2165 1355 50  0000 L CNN
F 2 "" H 2050 1400 50  0001 C CNN
F 3 "~" H 2050 1400 50  0001 C CNN
	1    2050 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1250 3600 1250
Connection ~ 3600 1250
Wire Wire Line
	4100 1550 4100 1600
Wire Wire Line
	4100 1600 3600 1600
Connection ~ 3600 1600
Wire Wire Line
	3600 1600 3600 1650
Wire Wire Line
	1600 2300 1600 2150
Wire Wire Line
	1600 2300 2500 2300
Wire Wire Line
	1600 1650 1600 1850
Wire Wire Line
	2800 1200 2800 1250
Wire Wire Line
	2050 1250 2500 1250
Wire Wire Line
	2500 1250 2800 1250
Connection ~ 2500 1250
Connection ~ 2800 1250
Wire Wire Line
	2800 1250 2800 1700
Wire Wire Line
	2050 1550 2500 1550
Wire Wire Line
	2500 1650 2500 1550
Connection ~ 2500 1550
Wire Wire Line
	1600 2800 1600 2700
Wire Wire Line
	1600 2800 2500 2800
$EndSCHEMATC
