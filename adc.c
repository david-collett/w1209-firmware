/*
 * This file is part of the W1209 firmware replacement project
 * (https://github.com/mister-grumbler/w1209-firmware).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Control functions for analog-to-digital converter (ADC).
 * The ADC1 interrupt (22) is used to get signal on end of convertion event.
 * The port D6 (pin 3) is used as analog input (AIN6).
 */

#include "adc.h"
#include "stm8s003/adc.h"
#include "params.h"

// Averaging bits
#define ADC_AVERAGING_BITS      4

#define ADC_RAW_32K_SIZE      (sizeof rawAdc32k / sizeof rawAdc32k[0])
#define ADC_RAW_32K_BASE_TEMP 1110

static unsigned char waitAdc = 1 ;

/* Choose your suitable R2 resister constant. */
#define R2 20000 // R2 = 20k [203] ohm (default)
//#<OFF>#define R2 10000 // R2 = 10k [103] ohm
//#<OFF>#define R2 5100 // R2 = 5.1k [512] ohm

/* Choose your suitable B-constant of the RT NTC thermistor. */
#define B3380K // RT NTC 10K that has B-constant = 3380K (default)
//#<OFF>#define B3435K // RT NTC 10K that has B-constant = 3435K
//#<OFF>#define B3950K // RT NTC 10K that has B-constant = 3950K

/* Adc digital converter (Floating point probably be solved on compiling.) */
#define adcDig32k(RT) (unsigned int)(32768.f*(RT)/((R2)+(RT)))

#ifdef B3380K
// The raw ADC 32k lookup table for the NTC RT 10K B-constant = 3380K.
const unsigned int rawAdc32k[] = {
    adcDig32k(790.3)    , // 111
    adcDig32k(808.66)   , // 110
    adcDig32k(827.54)   , // 109
    adcDig32k(846.97)   , // 108
    adcDig32k(866.96)   , // 107
    adcDig32k(887.53)   , // 106
    adcDig32k(908.702)  , // 105
    adcDig32k(930.49)   , // 104
    adcDig32k(952.93)   , // 103
    adcDig32k(976.031)  , // 102
    adcDig32k(999.82)   , // 101
    adcDig32k(1024.32)  , // 100
    adcDig32k(1049.56)  , // 99
    adcDig32k(1075.56)  , // 98
    adcDig32k(1102.35)  , // 97
    adcDig32k(1129.96)  , // 96
    adcDig32k(1158.41)  , // 95
    adcDig32k(1187.75)  , // 94
    adcDig32k(1217.99)  , // 93
    adcDig32k(1249.17)  , // 92
    adcDig32k(1281.33)  , // 91
    adcDig32k(1314.5)   , // 90
    adcDig32k(1348.72)  , // 89
    adcDig32k(1384.03)  , // 88
    adcDig32k(1420.47)  , // 87
    adcDig32k(1458.08)  , // 86
    adcDig32k(1496.9)   , // 85
    adcDig32k(1536.98)  , // 84
    adcDig32k(1578.37)  , // 83
    adcDig32k(1621.12)  , // 82
    adcDig32k(1665.27)  , // 81
    adcDig32k(1710.89)  , // 80
    adcDig32k(1758.03)  , // 79
    adcDig32k(1806.74)  , // 78
    adcDig32k(1857.1)   , // 77
    adcDig32k(1909.16)  , // 76
    adcDig32k(1962.99)  , // 75
    adcDig32k(2018.66)  , // 74
    adcDig32k(2076.25)  , // 73
    adcDig32k(2135.83)  , // 72
    adcDig32k(2197.47)  , // 71
    adcDig32k(2261.28)  , // 70
    adcDig32k(2327.32)  , // 69
    adcDig32k(2395.7)   , // 68
    adcDig32k(2466.5)   , // 67
    adcDig32k(2539.84)  , // 66
    adcDig32k(2615.81)  , // 65
    adcDig32k(2694.52)  , // 64
    adcDig32k(2776.09)  , // 63
    adcDig32k(2860.64)  , // 62
    adcDig32k(2948.3)   , // 61
    adcDig32k(3039.19)  , // 60
    adcDig32k(3133.45)  , // 59
    adcDig32k(3231.24)  , // 58
    adcDig32k(3332.69)  , // 57
    adcDig32k(3438)     , // 56
    adcDig32k(3547.27)  , // 55
    adcDig32k(3660.73)  , // 54
    adcDig32k(3778.55)  , // 53
    adcDig32k(3900.92)  , // 52
    adcDig32k(4028.04)  , // 51
    adcDig32k(4160.14)  , // 50
    adcDig32k(4297.43)  , // 49
    adcDig32k(4440.14)  , // 48
    adcDig32k(4588.53)  , // 47
    adcDig32k(4742.86)  , // 46
    adcDig32k(4903.4)   , // 45
    adcDig32k(5070.44)  , // 44
    adcDig32k(5244.3)   , // 43
    adcDig32k(5425.23)  , // 42
    adcDig32k(5613.6)   , // 41
    adcDig32k(5809.87)  , // 40
    adcDig32k(6014.28)  , // 39
    adcDig32k(6227.27)  , // 38
    adcDig32k(6449.24)  , // 37
    adcDig32k(6680.6)   , // 36
    adcDig32k(6921.92)  , // 35
    adcDig32k(7173.58)  , // 34
    adcDig32k(7436.12)  , // 33
    adcDig32k(7710.1)   , // 32
    adcDig32k(7996.05)  , // 31
    adcDig32k(8294.61)  , // 30
    adcDig32k(8606.4)   , // 29
    adcDig32k(8932.11)  , // 28
    adcDig32k(9272.43)  , // 27
    adcDig32k(9628.1)   , // 26
    adcDig32k(10000)    , // 25
    adcDig32k(10388.9)  , // 24
    adcDig32k(10795.7)  , // 23
    adcDig32k(11221.3)  , // 22
    adcDig32k(11666.8)  , // 21
    adcDig32k(12133.2)  , // 20
    adcDig32k(12621.6)  , // 19
    adcDig32k(13133.2)  , // 18
    adcDig32k(13669.4)  , // 17
    adcDig32k(14231.3)  , // 16
    adcDig32k(14820.5)  , // 15
    adcDig32k(15438.5)  , // 14
    adcDig32k(16086.8)  , // 13
    adcDig32k(16767.1)  , // 12
    adcDig32k(17481.4)  , // 11
    adcDig32k(18231.4)  , // 10
    adcDig32k(19019.3)  , // 9
    adcDig32k(19847.2)  , // 8
    adcDig32k(20717.4)  , // 7
    adcDig32k(21632.5)  , // 6
    adcDig32k(22594.9)  , // 5
    adcDig32k(23607.7)  , // 4
    adcDig32k(24673.6)  , // 3
    adcDig32k(25796)    , // 2
    adcDig32k(26978.1)  , // 1
    adcDig32k(28223.7)  , // 0
    adcDig32k(29536.6)  , // -1
    adcDig32k(30921)    , // -2
    adcDig32k(32381.2)  , // -3
    adcDig32k(33922)    , // -4
    adcDig32k(35548.4)  , // -5
    adcDig32k(37265.9)  , // -6
    adcDig32k(39080.2)  , // -7
    adcDig32k(40997.5)  , // -8
    adcDig32k(43024)    , // -9
    adcDig32k(45168.3)  , // -10
    adcDig32k(47436.5)  , // -11
    adcDig32k(49837.3)  , // -12
    adcDig32k(52379.4)  , // -13
    adcDig32k(55072.4)  , // -14
    adcDig32k(57926.4)  , // -15
    adcDig32k(60952.1)  , // -16
    adcDig32k(64161)    , // -17
    adcDig32k(67567)    , // -18
    adcDig32k(71182.2)  , // -19
    adcDig32k(75021.7)  , // -20
    adcDig32k(79101.3)  , // -21
    adcDig32k(83437.9)  , // -22
    adcDig32k(88049.8)  , // -23
    adcDig32k(92956.8)  , // -24
    adcDig32k(98180)    , // -25
    adcDig32k(103743)   , // -26
    adcDig32k(109669)   , // -27
    adcDig32k(115988)   , // -28
    adcDig32k(122726)   , // -29
    adcDig32k(129917)   , // -30
    adcDig32k(137593)   , // -31
    adcDig32k(145792)   , // -32
    adcDig32k(154554)   , // -33
    adcDig32k(163923)   , // -34
    adcDig32k(173946)   , // -35
    adcDig32k(184674)   , // -36
    adcDig32k(196163)   , // -37
    adcDig32k(208474)   , // -38
    adcDig32k(221672)   , // -39
    adcDig32k(235830)   , // -40
    adcDig32k(251027)   , // -41
    adcDig32k(267347)   , // -42
    adcDig32k(284884)   , // -43
    adcDig32k(303740)   , // -44
    adcDig32k(324026)   , // -45
    adcDig32k(345863)   , // -46
    adcDig32k(369386)   , // -47
    adcDig32k(394738)   , // -48
    adcDig32k(422081)   , // -49
    adcDig32k(451590)   , // -50
    adcDig32k(483454)   , // -51
};
#endif

#ifdef B3435K
// The raw ADC 32k lookup table for the NTC RT 10K B-constant = 3435K.
const unsigned int rawAdc32k[] = {
    adcDig32k(758.326)  , // 111
    adcDig32k(776.23)   , // 110
    adcDig32k(794.66)   , // 109
    adcDig32k(813.62)   , // 108
    adcDig32k(833.14)   , // 107
    adcDig32k(853.23)   , // 106
    adcDig32k(873.922)  , // 105
    adcDig32k(895.23)   , // 104
    adcDig32k(917.17)   , // 103
    adcDig32k(939.766)  , // 102
    adcDig32k(963.048)  , // 101
    adcDig32k(987.037)  , // 100
    adcDig32k(1011.76)  , // 99
    adcDig32k(1037.23)  , // 98
    adcDig32k(1063.5)   , // 97
    adcDig32k(1090.57)  , // 96
    adcDig32k(1118.49)  , // 95
    adcDig32k(1147.27)  , // 94
    adcDig32k(1176.97)  , // 93
    adcDig32k(1207.6)   , // 92
    adcDig32k(1239.2)   , // 91
    adcDig32k(1271.81)  , // 90
    adcDig32k(1305.46)  , // 89
    adcDig32k(1340.2)   , // 88
    adcDig32k(1376.07)  , // 87
    adcDig32k(1413.1)   , // 86
    adcDig32k(1451.35)  , // 85
    adcDig32k(1490.85)  , // 84
    adcDig32k(1531.66)  , // 83
    adcDig32k(1573.82)  , // 82
    adcDig32k(1617.4)   , // 81
    adcDig32k(1662.44)  , // 80
    adcDig32k(1708.99)  , // 79
    adcDig32k(1757.13)  , // 78
    adcDig32k(1806.91)  , // 77
    adcDig32k(1858.4)   , // 76
    adcDig32k(1911.67)  , // 75
    adcDig32k(1966.78)  , // 74
    adcDig32k(2023.81)  , // 73
    adcDig32k(2082.84)  , // 72
    adcDig32k(2143.95)  , // 71
    adcDig32k(2207.23)  , // 70
    adcDig32k(2272.76)  , // 69
    adcDig32k(2340.64)  , // 68
    adcDig32k(2410.96)  , // 67
    adcDig32k(2483.83)  , // 66
    adcDig32k(2559.35)  , // 65
    adcDig32k(2637.63)  , // 64
    adcDig32k(2718.8)   , // 63
    adcDig32k(2802.97)  , // 62
    adcDig32k(2890.28)  , // 61
    adcDig32k(2980.85)  , // 60
    adcDig32k(3074.84)  , // 59
    adcDig32k(3172.38)  , // 58
    adcDig32k(3273.63)  , // 57
    adcDig32k(3378.76)  , // 56
    adcDig32k(3487.94)  , // 55
    adcDig32k(3601.35)  , // 54
    adcDig32k(3719.18)  , // 53
    adcDig32k(3841.62)  , // 52
    adcDig32k(3968.88)  , // 51
    adcDig32k(4101.19)  , // 50
    adcDig32k(4238.77)  , // 49
    adcDig32k(4381.87)  , // 48
    adcDig32k(4530.74)  , // 47
    adcDig32k(4685.64)  , // 46
    adcDig32k(4846.87)  , // 45
    adcDig32k(5014.71)  , // 44
    adcDig32k(5189.5)   , // 43
    adcDig32k(5371.52)  , // 42
    adcDig32k(5561.15)  , // 41
    adcDig32k(5758.76)  , // 40
    adcDig32k(5964.73)  , // 39
    adcDig32k(6179.46)  , // 38
    adcDig32k(6403.37)  , // 37
    adcDig32k(6636.93)  , // 36
    adcDig32k(6880.6)   , // 35
    adcDig32k(7134.91)  , // 34
    adcDig32k(7400.4)   , // 33
    adcDig32k(7677.5)   , // 32
    adcDig32k(7967)     , // 31
    adcDig32k(8269.41)  , // 30
    adcDig32k(8585.41)  , // 29
    adcDig32k(8915.71)  , // 28
    adcDig32k(9261)     , // 27
    adcDig32k(9622.2)   , // 26
    adcDig32k(10000)    , // 25
    adcDig32k(10395.3)  , // 24
    adcDig32k(10809.1)  , // 23
    adcDig32k(11242.4)  , // 22
    adcDig32k(11696.1)  , // 21
    adcDig32k(12171.4)  , // 20
    adcDig32k(12669.5)  , // 19
    adcDig32k(13191.6)  , // 18
    adcDig32k(13739)    , // 17
    adcDig32k(14313.3)  , // 16
    adcDig32k(14915.7)  , // 15
    adcDig32k(15547.9)  , // 14
    adcDig32k(16211.7)  , // 13
    adcDig32k(16908.7)  , // 12
    adcDig32k(17641)    , // 11
    adcDig32k(18410.4)  , // 10
    adcDig32k(19219.3)  , // 9
    adcDig32k(20069.8)  , // 8
    adcDig32k(20964.4)  , // 7
    adcDig32k(21905.8)  , // 6
    adcDig32k(22896.6)  , // 5
    adcDig32k(23940)    , // 4
    adcDig32k(25038.9)  , // 3
    adcDig32k(26196.8)  , // 2
    adcDig32k(27417.3)  , // 1
    adcDig32k(28704.3)  , // 0
    adcDig32k(30061.8)  , // -1
    adcDig32k(31494.2)  , // -2
    adcDig32k(33006.2)  , // -3
    adcDig32k(34602.9)  , // -4
    adcDig32k(36289.7)  , // -5
    adcDig32k(38072.2)  , // -6
    adcDig32k(39956.6)  , // -7
    adcDig32k(41949.6)  , // -8
    adcDig32k(44058)    , // -9
    adcDig32k(46290.2)  , // -10
    adcDig32k(48653.5)  , // -11
    adcDig32k(51157)    , // -12
    adcDig32k(53810)    , // -13
    adcDig32k(56622.7)  , // -14
    adcDig32k(59606)    , // -15
    adcDig32k(62771)    , // -16
    adcDig32k(66132)    , // -17
    adcDig32k(69700.5)  , // -18
    adcDig32k(73492.2)  , // -19
    adcDig32k(77522.5)  , // -20
    adcDig32k(81808.6)  , // -21
    adcDig32k(86368.6)  , // -22
    adcDig32k(91222)    , // -23
    adcDig32k(96391.1)  , // -24
    adcDig32k(101898)   , // -25
    adcDig32k(107768)   , // -26
    adcDig32k(114028)   , // -27
    adcDig32k(120707)   , // -28
    adcDig32k(127837)   , // -29
    adcDig32k(135452)   , // -30
    adcDig32k(143590)   , // -31
    adcDig32k(152290)   , // -32
    adcDig32k(161596)   , // -33
    adcDig32k(171556)   , // -34
    adcDig32k(182221)   , // -35
    adcDig32k(193648)   , // -36
    adcDig32k(205898)   , // -37
    adcDig32k(219036)   , // -38
    adcDig32k(233136)   , // -39
    adcDig32k(248276)   , // -40
    adcDig32k(264544)   , // -41
    adcDig32k(282031)   , // -42
    adcDig32k(300842)   , // -43
    adcDig32k(321090)   , // -44
    adcDig32k(342894)   , // -45
    adcDig32k(366392)   , // -46
    adcDig32k(391730)   , // -47
    adcDig32k(419068)   , // -48
    adcDig32k(448585)   , // -49
    adcDig32k(480475)   , // -50
    adcDig32k(514947)   , // -51
};
#endif

#ifdef B3950K
// The raw ADC 32k lookup table for the NTC RT 10K B-constant = 3950K.
const unsigned int rawAdc32k[] = {
    adcDig32k(515.13)   , // 111
    adcDig32k(529.14)   , // 110
    adcDig32k(543.61)   , // 109
    adcDig32k(558.55)   , // 108
    adcDig32k(573.99)   , // 107
    adcDig32k(589.94)   , // 106
    adcDig32k(606.416)  , // 105
    adcDig32k(623.45)   , // 104
    adcDig32k(641.049)  , // 103
    adcDig32k(659.246)  , // 102
    adcDig32k(678.06)   , // 101
    adcDig32k(697.52)   , // 100
    adcDig32k(717.65)   , // 99
    adcDig32k(738.464)  , // 98
    adcDig32k(760.005)  , // 97
    adcDig32k(782.296)  , // 96
    adcDig32k(805.37)   , // 95
    adcDig32k(829.249)  , // 94
    adcDig32k(853.98)   , // 93
    adcDig32k(879.58)   , // 92
    adcDig32k(906.104)  , // 91
    adcDig32k(933.577)  , // 90
    adcDig32k(962.04)   , // 89
    adcDig32k(991.54)   , // 88
    adcDig32k(1022.11)  , // 87
    adcDig32k(1053.81)  , // 86
    adcDig32k(1086.67)  , // 85
    adcDig32k(1120.75)  , // 84
    adcDig32k(1156.1)   , // 83
    adcDig32k(1192.77)  , // 82
    adcDig32k(1230.83)  , // 81
    adcDig32k(1270.32)  , // 80
    adcDig32k(1311.32)  , // 79
    adcDig32k(1353.88)  , // 78
    adcDig32k(1398.08)  , // 77
    adcDig32k(1443.99)  , // 76
    adcDig32k(1491.68)  , // 75
    adcDig32k(1541.24)  , // 74
    adcDig32k(1592.74)  , // 73
    adcDig32k(1646.28)  , // 72
    adcDig32k(1701.95)  , // 71
    adcDig32k(1759.84)  , // 70
    adcDig32k(1820.05)  , // 69
    adcDig32k(1882.7)   , // 68
    adcDig32k(1947.89)  , // 67
    adcDig32k(2015.74)  , // 66
    adcDig32k(2086.37)  , // 65
    adcDig32k(2159.93)  , // 64
    adcDig32k(2236.53)  , // 63
    adcDig32k(2316.34)  , // 62
    adcDig32k(2399.5)   , // 61
    adcDig32k(2486.16)  , // 60
    adcDig32k(2576.51)  , // 59
    adcDig32k(2670.72)  , // 58
    adcDig32k(2768.98)  , // 57
    adcDig32k(2871.48)  , // 56
    adcDig32k(2978.44)  , // 55
    adcDig32k(3090.07)  , // 54
    adcDig32k(3206.6)   , // 53
    adcDig32k(3328.29)  , // 52
    adcDig32k(3455.39)  , // 51
    adcDig32k(3588.18)  , // 50
    adcDig32k(3726.95)  , // 49
    adcDig32k(3872)     , // 48
    adcDig32k(4023.64)  , // 47
    adcDig32k(4182.23)  , // 46
    adcDig32k(4348.14)  , // 45
    adcDig32k(4521.73)  , // 44
    adcDig32k(4703.4)   , // 43
    adcDig32k(4893.63)  , // 42
    adcDig32k(5092.8)   , // 41
    adcDig32k(5301.5)   , // 40
    adcDig32k(5520.08)  , // 39
    adcDig32k(5749.21)  , // 38
    adcDig32k(5989.41)  , // 37
    adcDig32k(6241.3)   , // 36
    adcDig32k(6505.53)  , // 35
    adcDig32k(6782.77)  , // 34
    adcDig32k(7073.8)   , // 33
    adcDig32k(7379.26)  , // 32
    adcDig32k(7700.1)   , // 31
    adcDig32k(8037.14)  , // 30
    adcDig32k(8391.31)  , // 29
    adcDig32k(8763.61)  , // 28
    adcDig32k(9155.1)   , // 27
    adcDig32k(9566.8)   , // 26
    adcDig32k(10000)    , // 25
    adcDig32k(10455.9)  , // 24
    adcDig32k(10936)    , // 23
    adcDig32k(11441.5)  , // 22
    adcDig32k(11974)    , // 21
    adcDig32k(12535.3)  , // 20
    adcDig32k(13127)    , // 19
    adcDig32k(13751)    , // 18
    adcDig32k(14409.2)  , // 17
    adcDig32k(15103.9)  , // 16
    adcDig32k(15837.2)  , // 15
    adcDig32k(16611.5)  , // 14
    adcDig32k(17429.6)  , // 13
    adcDig32k(18294.1)  , // 12
    adcDig32k(19208)    , // 11
    adcDig32k(20174.6)  , // 10
    adcDig32k(21197.1)  , // 9
    adcDig32k(22279.3)  , // 8
    adcDig32k(23425.1)  , // 7
    adcDig32k(24638.7)  , // 6
    adcDig32k(25924.6)  , // 5
    adcDig32k(27287.5)  , // 4
    adcDig32k(28732.8)  , // 3
    adcDig32k(30266)    , // 2
    adcDig32k(31893.1)  , // 1
    adcDig32k(33620.6)  , // 0
    adcDig32k(35455.4)  , // -1
    adcDig32k(37405)    , // -2
    adcDig32k(39477.3)  , // -3
    adcDig32k(41681.3)  , // -4
    adcDig32k(44026)    , // -5
    adcDig32k(46521.8)  , // -6
    adcDig32k(49179.4)  , // -7
    adcDig32k(52010.6)  , // -8
    adcDig32k(55028.2)  , // -9
    adcDig32k(58245.7)  , // -10
    adcDig32k(61678.1)  , // -11
    adcDig32k(65341)    , // -12
    adcDig32k(69253.1)  , // -13
    adcDig32k(73431.9)  , // -14
    adcDig32k(77898)    , // -15
    adcDig32k(82674)    , // -16
    adcDig32k(87783)    , // -17
    adcDig32k(93252)    , // -18
    adcDig32k(99109.3)  , // -19
    adcDig32k(105384)   , // -20
    adcDig32k(112112)   , // -21
    adcDig32k(119327)   , // -22
    adcDig32k(127071)   , // -23
    adcDig32k(135385)   , // -24
    adcDig32k(144317)   , // -25
    adcDig32k(153918)   , // -26
    adcDig32k(164242)   , // -27
    adcDig32k(175353)   , // -28
    adcDig32k(187317)   , // -29
    adcDig32k(200204)   , // -30
    adcDig32k(214096)   , // -31
    adcDig32k(229079)   , // -32
    adcDig32k(245249)   , // -33
    adcDig32k(262710)   , // -34
    adcDig32k(281577)   , // -35
    adcDig32k(301975)   , // -36
    adcDig32k(324044)   , // -37
    adcDig32k(347933)   , // -38
    adcDig32k(373811)   , // -39
    adcDig32k(401860)   , // -40
    adcDig32k(432284)   , // -41
    adcDig32k(465304)   , // -42
    adcDig32k(501167)   , // -43
    adcDig32k(540146)   , // -44
    adcDig32k(582536)   , // -45
    adcDig32k(628672)   , // -46
    adcDig32k(678921)   , // -47
    adcDig32k(733685)   , // -48
    adcDig32k(793417)   , // -49
    adcDig32k(858615)   , // -50
    adcDig32k(929829)   , // -51
};
#endif

static unsigned int result;
static unsigned long averaged;

/**
 * @brief Initialize ADC's configuration registers.
 */
void initADC()
{
    ADC_CR1 |= 0x70;    // Prescaler f/18 (SPSEL)
    ADC_CSR |= 0x06;    // select AIN6
    ADC_CSR |= 0x20;    // Interrupt enable (EOCIE)
    ADC_CR1 |= 0x01;    // Power up ADC
    result = 0;
    averaged = 0;
}

/**
 * @brief Sets bit in ADC control register to start data convertion.
 */
void startADC()
{
    ADC_CR1 |= 0x01;
}

/**
 * @brief Gets raw result of last data conversion.
 * @return raw result.
 */
unsigned int getAdcResult()
{
    return result;
}

/**
 * @brief Gets averaged over 2^ADC_AVERAGING_BITS times result of data
 *  convertion.
 * @return averaged result.
 */
unsigned int getAdcAveraged()
{
    return (unsigned int) (averaged >> ADC_AVERAGING_BITS);
}

/**
 * @brief Calculation of real temperature using averaged result of
 *  AnalogToDigital conversion and the lookup table.
 * @return temperature in tenth of degrees of Celsius.
 */
int getTemperature()
{
    unsigned char rightBound = ADC_RAW_32K_SIZE-1;
    unsigned char leftBound = 0;

    int correct = getParamById (PARAM_TEMPERATURE_CORRECTION) ;

    unsigned int val32k = averaged << (5-ADC_AVERAGING_BITS) ;

    if(val32k>=rawAdc32k[0]&&val32k<rawAdc32k[ADC_RAW_32K_SIZE-1]) {

      // search through the rawAdc32k lookup table
      while ( (rightBound - leftBound) > 1) {
          unsigned char midId = (leftBound + rightBound) >> 1;

          if (val32k < rawAdc32k[midId]) {
              rightBound = midId;
          } else {
              leftBound = midId;
          }
      }

      // calculate the interpolated temperature value and return it
      {
        int i = (int) leftBound ;
        if(val32k>=rawAdc32k[i]&&val32k<rawAdc32k[i+1]) {
          unsigned int denom = (rawAdc32k[i+1]-rawAdc32k[i]) ;
          unsigned int delta = (rawAdc32k[i+1]-val32k) ;
          while(delta>(unsigned int)(-1)/10U) denom>>=1, delta>>=1 ;
          if(!denom) denom=1 ;
          {
            int base_temp = ADC_RAW_32K_BASE_TEMP - (i+1)*10U ;
            unsigned int delta_temp = 10U * delta / denom ; // interpolation
            return base_temp + (int)delta_temp + correct ;
          }
        }
      }

    }

    // temperature overflow
    return
        ADC_RAW_32K_BASE_TEMP
            - ( val32k < rawAdc32k[0] ? 0 : (ADC_RAW_32K_SIZE-1) * 10 ) ;
}

/**
 * @brief This function is ADC's interrupt request handler
 *  so keep it extremely small and fast.
 */
void ADC1_EOC_handler() __interrupt (22)
{
    result = ADC_DRH << 2;
    result |= ADC_DRL;
    ADC_CSR &= ~0x80;   // reset EOC

    if(waitAdc) {
      waitAdc--;
      return ;
    }

    // Averaging result
    if (averaged == 0) {
        averaged = result << ADC_AVERAGING_BITS;
    } else {
        averaged += result - (averaged >> ADC_AVERAGING_BITS);
    }
}
