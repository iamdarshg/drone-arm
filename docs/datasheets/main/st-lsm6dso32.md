Title: untitled

URL Source: http://www.st.com/resource/en/datasheet/lsm6dso32.pdf

Number of Pages: 153

Markdown Content:
This is information on a product in full production. March 2020 DocID032891 Rev 1 1/153 

# LSM6DSO32 

# iNEMO inertial module: always-on 3D accelerometer and 3D gyroscope 

Datasheet  - production data 

# Features 

 Power consumption: 0.55 mA in combo high-performance mode 

 “Always-on" experience with low power consumption for both accelerometer and gyroscope 

 Smart FIFO up to 9 kbytes 

 ±4/±8/±16/±32 g full scale 

 ±125/±250/±500/±1000/±2000 dps full scale 

 Analog supply voltage: 1.71 V to 3.6 V 

 Independent IO supply (1.62 V) 

 Compact footprint: 2.5 mm x 3 mm x 0.83 mm 

 SPI / I²C & MIPI I3C SM serial interface with main processor data synchronization 

 Advanced pedometer, step detector and step counter 

 Significant Motion Detection, Tilt detection 

 Standard interrupts: free-fall, wakeup, 6D/4D orientation, click and double-click 

 Programmable finite state machine: accelerometer, gyroscope and external sensors 

 Embedded temperature sensor 

 ECOPACK, RoHS and “Green” compliant 

# Applications 

 Wearables, smart watches, and sports equipment 

 Motion tracking and gesture detection 

 Hard fall detection 

 Sensor hub 

 Navigation 

 IoT and connected devices 

 Smart power saving for handheld devices 

# Description 

The LSM6DSO32 is a system-in-package featuring a 3D digital accelerometer and 3D digital gyroscope boosting power performance to 0.55 mA in high-performance mode and enabling always-on low-power features for an optimal motion experience for the consumer. 

The LSM6DSO32 supports main OS requirements, offering real, virtual and batch sensors with 9 kbytes for dynamic data batching. ST’s family of MEMS sensor modules leverages the robust and mature manufacturing processes already used for the production of micromachined accelerometers and gyroscopes. The various sensing elements are manufactured using specialized micromachining processes, while the IC interfaces are developed using CMOS technology that allows the design of a dedicated circuit which is trimmed to better match the characteristics of the sensing element. 

The LSM6DSO32 has a full-scale acceleration range of ±4/±8/±16±32 g and an angular rate range of ±125/±250/±500/±1000/±2000 dps. 

High robustness to mechanical shock makes the LSM6DSO32 the preferred choice of system designers for the creation and manufacturing of reliable products. The LSM6DSO32 is available in a plastic land grid array (LGA) package. 

LGA-14L (2.5 x 3 x 0.83 mm) typ. 

Table 1. Device summary Part number Temp. range [°C] Package Packing 

LSM6DSO32 -40 to +85 LGA-14L (2.5x3x0.83 mm) Tray LSM6DSO32TR -40 to +85 Tape & Reel   

> www.st.com Contents LSM6DSO32
> 2/153 DocID032891 Rev 1

# Contents 

## 1 Overview . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 18 2 Embedded low-power features . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 19 

2.1 Tilt detection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 20 2.2 Significant Motion Detection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 20 2.3 Finite State Machine . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 20 

## 3 Pin description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 22 

3.1 Pin connections . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 23 

## 4 Module specifications . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 25 

4.1 Mechanical characteristics . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 25 4.2 Electrical characteristics . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 28 4.3 Temperature sensor characteristics . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 29 4.4 Communication interface characteristics . . . . . . . . . . . . . . . . . . . . . . . . . 30 4.4.1 SPI - serial peripheral interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 30 4.4.2 I²C - inter-IC control interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 31 4.5 Absolute maximum ratings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 32 4.6 Terminology . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 33 4.6.1 Sensitivity . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 33 4.6.2 Zero-g and zero-rate level . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 33 

## 5 Digital interfaces . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 34 

5.1 I²C/SPI interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 34 5.1.1 I²C serial interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 34 5.1.2 SPI bus interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 37 5.2 MIPI I3C SM interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 41 5.2.1 MIPI I3C SM slave interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 41 5.2.2 MIPI I3C SM CCC supported commands . . . . . . . . . . . . . . . . . . . . . . . . 41 5.3 I²C/MIPI I3C SM coexistence in LSM6DSO32 . . . . . . . . . . . . . . . . . . . . . . 43 5.4 Master I²C interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 44 

## 6 Functionality . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 45 DocID032891 Rev 1 3/153  

> LSM6DSO32 Contents
> 153

6.1 Operating modes . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 45 6.2 Accelerometer power modes . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 45 6.2.1 Accelerometer ultra-low-power mode . . . . . . . . . . . . . . . . . . . . . . . . . . 45 6.3 Gyroscope power modes . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 46 6.4 Block diagram of filters . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 46 6.4.1 Block diagrams of the accelerometer filters . . . . . . . . . . . . . . . . . . . . . . 46 6.4.2 Block diagrams of the gyroscope filters . . . . . . . . . . . . . . . . . . . . . . . . . 48 6.5 FIFO . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 49 6.5.1 Bypass mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 50 6.5.2 FIFO mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 50 6.5.3 Continuous mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 50 6.5.4 Continuous-to-FIFO mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 51 6.5.5 Bypass-to-Continuous mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 51 6.5.6 Bypass-to-FIFO mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 51 6.5.7 FIFO reading procedure . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 52 

## 7 Application hints . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 53 

7.1 LSM6DSO32 electrical connections in Mode 1 . . . . . . . . . . . . . . . . . . . . 53 7.2 LSM6DSO32 electrical connections in Mode 2 . . . . . . . . . . . . . . . . . . . . 54 

## 8 Register mapping . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 57 9 Register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 60 

9.1 FUNC_CFG_ACCESS (01h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 60 9.2 PIN_CTRL (02h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 60 9.3 FIFO_CTRL1 (07h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 61 9.4 FIFO_CTRL2 (08h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 61 9.5 FIFO_CTRL3 (09h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 62 9.6 FIFO_CTRL4 (0Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 63 9.7 COUNTER_BDR_REG1 (0Bh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 64 9.8 COUNTER_BDR_REG2 (0Ch) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 64 9.9 INT1_CTRL (0Dh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 65 9.10 INT2_CTRL (0Eh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 66 9.11 WHO_AM_I (0Fh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 66 9.12 CTRL1_XL (10h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 67 Contents LSM6DSO32  

> 4/153 DocID032891 Rev 1

9.13 CTRL2_G (11h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 68 9.14 CTRL3_C (12h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 69 9.15 CTRL4_C (13h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 70 9.16 CTRL5_C (14h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 71 9.17 CTRL6_C (15h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 72 9.18 CTRL7_G (16h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 73 9.19 CTRL8_XL (17h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 73 9.20 CTRL9_XL (18h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 76 9.21 CTRL10_C (19h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 76 9.22 ALL_INT_SRC (1Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 77 9.23 WAKE_UP_SRC (1Bh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 77 9.24 TAP_SRC (1Ch) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 78 9.25 D6D_SRC (1Dh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 78 9.26 STATUS_REG (1Eh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 9.27 OUT_TEMP_L (20h), OUT_TEMP_H (21h) . . . . . . . . . . . . . . . . . . . . . . . 79 9.28 OUTX_L_G (22h) and OUTX_H_G (23h) . . . . . . . . . . . . . . . . . . . . . . . . 79 9.29 OUTY_L_G (24h) and OUTY_H_G (25h) . . . . . . . . . . . . . . . . . . . . . . . . 80 9.30 OUTZ_L_G (26h) and OUTZ_H_G (27h) . . . . . . . . . . . . . . . . . . . . . . . . . 80 9.31 OUTX_L_A (28h) and OUTX_H_A (29h) . . . . . . . . . . . . . . . . . . . . . . . . . 80 9.32 OUTY_L_A (2Ah) and OUTY_H_A (2Bh) . . . . . . . . . . . . . . . . . . . . . . . . 81 9.33 OUTZ_L_A (2Ch) and OUTZ_H_A (2Dh) . . . . . . . . . . . . . . . . . . . . . . . . 81 9.34 EMB_FUNC_STATUS_MAINPAGE (35h) . . . . . . . . . . . . . . . . . . . . . . . . 82 9.35 FSM_STATUS_A_MAINPAGE (36h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . 82 9.36 FSM_STATUS_B_MAINPAGE (37h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . 83 9.37 STATUS_MASTER_MAINPAGE (39h) . . . . . . . . . . . . . . . . . . . . . . . . . . . 83 9.38 FIFO_STATUS1 (3Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 84 9.39 FIFO_STATUS2 (3Bh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 84 9.40 TIMESTAMP0 (40h), TIMESTAMP1 (41h), TIMESTAMP2 (42h), and TIMESTAMP3 (43h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 85 9.41 TAP_CFG0 (56h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 86 9.42 TAP_CFG1 (57h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 87 9.43 TAP_CFG2 (58h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 87 9.44 TAP_THS_6D (59h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 88 DocID032891 Rev 1 5/153  

> LSM6DSO32 Contents
> 153

9.45 INT_DUR2 (5Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 88 9.46 WAKE_UP_THS (5Bh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 89 9.47 WAKE_UP_DUR (5Ch) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 89 9.48 FREE_FALL (5Dh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 90 9.49 MD1_CFG (5Eh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 91 9.50 MD2_CFG (5Fh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 92 9.51 I3C_BUS_AVB (62h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 9.52 INTERNAL_FREQ_FINE (63h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 9.53 X_OFS_USR (73h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 9.54 Y_OFS_USR (74h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 9.55 Z_OFS_USR (75h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 9.56 FIFO_DATA_OUT_TAG (78h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 9.57 FIFO_DATA_OUT_X_L (79h) and FIFO_DATA_OUT_X_H (7Ah) . . . . . . 96 9.58 FIFO_DATA_OUT_Y_L (7Bh) and FIFO_DATA_OUT_Y_H (7Ch) . . . . . 96 9.59 FIFO_DATA_OUT_Z_L (7Dh) and FIFO_DATA_OUT_Z_H (7Eh) . . . . . . 96 

## 10 Embedded functions register mapping . . . . . . . . . . . . . . . . . . . . . . . . . 97 11 Embedded functions register description . . . . . . . . . . . . . . . . . . . . . . 99 

11.1 PAGE_SEL (02h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 99 11.2 EMB_FUNC_EN_A (04h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 99 11.3 EMB_FUNC_EN_B (05h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 11.4 PAGE_ADDRESS (08h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 11.5 PAGE_VALUE (09h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 11.6 EMB_FUNC_INT1 (0Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 101 11.7 FSM_INT1_A (0Bh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 102 11.8 FSM_INT1_B (0Ch) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 103 11.9 EMB_FUNC_INT2 (0Eh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 104 11.10 FSM_INT2_A (0Fh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 105 11.11 FSM_INT2_B (10h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 106 11.12 EMB_FUNC_STATUS (12h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 107 11.13 FSM_STATUS_A (13h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 107 11.14 FSM_STATUS_B (14h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 108 11.15 PAGE_RW (17h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 108 Contents LSM6DSO32  

> 6/153 DocID032891 Rev 1

11.16 EMB_FUNC_FIFO_CFG (44h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 109 11.17 FSM_ENABLE_A (46h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 109 11.18 FSM_ENABLE_B (47h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110 11.19 FSM_LONG_COUNTER_L (48h) and FSM_LONG_COUNTER_H (49h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110 11.20 FSM_LONG_COUNTER_CLEAR (4Ah) . . . . . . . . . . . . . . . . . . . . . . . . .111 11.21 FSM_OUTS1 (4Ch) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .111 11.22 FSM_OUTS2 (4Dh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 112 11.23 FSM_OUTS3 (4Eh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 112 11.24 FSM_OUTS4 (4Fh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 113 11.25 FSM_OUTS5 (50h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 113 11.26 FSM_OUTS6 (51h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 114 11.27 FSM_OUTS7 (52h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 114 11.28 FSM_OUTS8 (53h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 115 11.29 FSM_OUTS9 (54h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 115 11.30 FSM_OUTS10 (55h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 116 11.31 FSM_OUTS11 (56h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 116 11.32 FSM_OUTS12 (57h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 117 11.33 FSM_OUTS13 (58h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 117 11.34 FSM_OUTS14 (59h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 118 11.35 FSM_OUTS15 (5Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 118 11.36 FSM_OUTS16 (5Bh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 119 11.37 EMB_FUNC_ODR_CFG_B (5Fh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 119 11.38 STEP_COUNTER_L (62h) and STEP_COUNTER_H (63h) . . . . . . . . . 120 11.39 EMB_FUNC_SRC (64h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 120 11.40 EMB_FUNC_INIT_A (66h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 121 11.41 EMB_FUNC_INIT_B (67h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 121 

## 12 Embedded advanced features pages . . . . . . . . . . . . . . . . . . . . . . . . . 122 13 Embedded advanced features register description . . . . . . . . . . . . . . 125 

13.1 Page 0 - Embedded advanced features registers . . . . . . . . . . . . . . . . . 125 13.1.1 MAG_SENSITIVITY_L (BAh) and MAG_SENSITIVITY_H (BBh) . . . . 125 13.1.2 MAG_OFFX_L (C0h) and MAG_OFFX_H (C1h) . . . . . . . . . . . . . . . . . 125 DocID032891 Rev 1 7/153 

LSM6DSO32 Contents 

> 153

13.1.3 MAG_OFFY_L (C2h) and MAG_OFFY_H (C3h) . . . . . . . . . . . . . . . . . 126 13.1.4 MAG_OFFZ_L (C4h) and MAG_OFFZ_H (C5h) . . . . . . . . . . . . . . . . . 126 13.1.5 MAG_SI_XX_L (C6h) and MAG_SI_XX_H (C7h) . . . . . . . . . . . . . . . . 127 13.1.6 MAG_SI_XY_L (C8h) and MAG_SI_XY_H (C9h) . . . . . . . . . . . . . . . . 127 13.1.7 MAG_SI_XZ_L (CAh) and MAG_SI_XZ_H (CBh) . . . . . . . . . . . . . . . . 128 13.1.8 MAG_SI_YY_L (CCh) and MAG_SI_YY_H (CDh) . . . . . . . . . . . . . . . 128 13.1.9 MAG_SI_YZ_L (CEh) and MAG_SI_YZ_H (CFh) . . . . . . . . . . . . . . . . 129 13.1.10 MAG_SI_ZZ_L (D0h) and MAG_SI_ZZ_H (D1h) . . . . . . . . . . . . . . . . 129 13.1.11 MAG_CFG_A (D4h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 130 13.1.12 MAG_CFG_B (D5h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 130 13.2 Page 1 - Embedded advanced features registers . . . . . . . . . . . . . . . . . 131 13.2.1 FSM_LC_TIMEOUT_L (7Ah) and FSM_LC_TIMEOUT_H (7Bh) . . . . 131 13.2.2 FSM_PROGRAMS (7Ch) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 131 13.2.3 FSM_START_ADD_L (7Eh) and FSM_START_ADD_H (7Fh) . . . . . . 132 13.2.4 PEDO_CMD_REG (83h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 13.2.5 PEDO_DEB_STEPS_CONF (84h) . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 13.2.6 PEDO_SC_DELTAT_L (D0h) and PEDO_SC_DELTAT_H (D1h) . . . . 133 

## 14 Sensor hub register mapping . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 134 15 Sensor hub register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 

15.1 SENSOR_HUB_1 (02h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 15.2 SENSOR_HUB_2 (03h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 15.3 SENSOR_HUB_3 (04h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 15.4 SENSOR_HUB_4 (05h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 15.5 SENSOR_HUB_5 (06h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 15.6 SENSOR_HUB_6 (07h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 15.7 SENSOR_HUB_7 (08h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 15.8 SENSOR_HUB_8 (09h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 15.9 SENSOR_HUB_9 (0Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 15.10 SENSOR_HUB_10 (0Bh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 15.11 SENSOR_HUB_11 (0Ch) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 15.12 SENSOR_HUB_12 (0Dh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 15.13 SENSOR_HUB_13 (0Eh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 15.14 SENSOR_HUB_14 (0Fh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 Contents LSM6DSO32  

> 8/153 DocID032891 Rev 1

15.15 SENSOR_HUB_15 (10h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 15.16 SENSOR_HUB_16 (11h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 15.17 SENSOR_HUB_17 (12h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 15.18 SENSOR_HUB_18 (13h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 15.19 MASTER_CONFIG (14h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 142 15.20 SLV0_ADD (15h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 142 15.21 SLV0_SUBADD (16h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 143 15.22 SLAVE0_CONFIG (17h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 143 15.23 SLV1_ADD (18h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 15.24 SLV1_SUBADD (19h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 15.25 SLAVE1_CONFIG (1Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 15.26 SLV2_ADD (1Bh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 15.27 SLV2_SUBADD (1Ch) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 15.28 SLAVE2_CONFIG (1Dh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 15.29 SLV3_ADD (1Eh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 15.30 SLV3_SUBADD (1Fh) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 15.31 SLAVE3_CONFIG (20h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 15.32 DATAWRITE_SLV0 (21h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 147 15.33 STATUS_MASTER (22h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 147 

## 16 Soldering information . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 148 17 Package information . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 149 

17.1 LGA-14L package information . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 149 17.2 LGA-14 packing information . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 150 

## 18 Revision history . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 152 DocID032891 Rev 1 9/153 

LSM6DSO32 List of tables 

> 153

# List of tables 

Table 1. Device summary . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 1 Table 2. Pin description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 24 Table 3. Mechanical characteristics . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 25 Table 4. Electrical characteristics . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 28 Table 5. Temperature sensor characteristics . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 29 Table 6. SPI slave timing values (in mode 3) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 30 Table 7. I²C slave timing values . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 31 Table 8. Absolute maximum ratings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 32 Table 9. Serial interface pin description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 34 Table 10. I²C terminology . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 34 Table 11. SAD+Read/Write patterns . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 35 Table 12. Transfer when master is writing one byte to slave . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 35 Table 13. Transfer when master is writing multiple bytes to slave . . . . . . . . . . . . . . . . . . . . . . . . . . . 35 Table 14. Transfer when master is receiving (reading) one byte of data from slave . . . . . . . . . . . . . 36 Table 15. Transfer when master is receiving (reading) multiple bytes of data from slave . . . . . . . . . 36 Table 16. MIPI I3C SM CCC commands . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 41 Table 17. Master I²C pin details . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 44 Table 18. Gyroscope LPF2 bandwidth selection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 48 Table 19. Internal pin status . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 55 Table 20. Registers address map . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 57 Table 21. FUNC_CFG_ACCESS register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 60 Table 22. FUNC_CFG_ACCESS register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 60 Table 23. PIN_CTRL register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 60 Table 24. PIN_CTRL register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 60 Table 25. FIFO_CTRL1 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 61 Table 26. FIFO_CTRL1 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 61 Table 27. FIFO_CTRL2 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 61 Table 28. FIFO_CTRL2 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 61 Table 29. FIFO_CTRL3 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 62 Table 30. FIFO_CTRL3 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 62 Table 31. FIFO_CTRL4 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 63 Table 32. FIFO_CTRL4 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 63 Table 33. COUNTER_BDR_REG1 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 64 Table 34. COUNTER_BDR_REG1 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 64 Table 35. COUNTER_BDR_REG2 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 64 Table 36. COUNTER_BDR_REG2 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 64 Table 37. INT1_CTRL register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 65 Table 38. INT1_CTRL register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 65 Table 39. INT2_CTRL register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 66 Table 40. INT2_CTRL register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 66 Table 41. WhoAmI register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 66 Table 42. CTRL1_XL register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 67 Table 43. CTRL1_XL register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 67 Table 44. Accelerometer ODR register setting . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 67 Table 45. Accelerometer full-scale selection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 67 Table 46. CTRL2_G register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 68 Table 47. CTRL2_G register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 68 Table 48. Gyroscope ODR configuration setting . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 68 List of tables LSM6DSO32  

> 10/153 DocID032891 Rev 1

Table 49. CTRL3_C register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 69 Table 50. CTRL3_C register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 69 Table 51. CTRL4_C register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 70 Table 52. CTRL4_C register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 70 Table 53. CTRL5_C register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 71 Table 54. CTRL5_C register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 71 Table 55. Angular rate sensor self-test mode selection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 71 Table 56. Linear acceleration sensor self-test mode selection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 71 Table 57. CTRL6_C register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 72 Table 58. CTRL6_C register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 72 Table 59. Trigger mode selection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 72 Table 60. Gyroscope LPF1 bandwidth selection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 72 Table 61. CTRL7_G register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 73 Table 62. CTRL7_G register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 73 Table 63. CTRL8_XL register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 73 Table 64. CTRL8_XL register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 74 Table 65. Accelerometer bandwidth configurations . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 74 Table 66. CTRL9_XL register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 76 Table 67. CTRL9_XL register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 76 Table 68. CTRL10_C register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 76 Table 69. CTRL10_C register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 76 Table 70. ALL_INT_SRC register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 77 Table 71. ALL_INT_SRC register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 77 Table 72. WAKE_UP_SRC register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 77 Table 73. WAKE_UP_SRC register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 77 Table 74. TAP_SRC register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 78 Table 75. TAP_SRC register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 78 Table 76. D6D_SRC register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 78 Table 77. D6D_SRC register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 78 Table 78. STATUS_REG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 Table 79. STATUS_REG register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 Table 80. OUT_TEMP_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 Table 81. OUT_TEMP_H register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 Table 82. OUT_TEMP register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 Table 83. OUTX_L_G register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 Table 84. OUTX_H_G register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 Table 85. OUTX_H_G register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79 Table 86. OUTY_L_G register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 87. OUTY_H_G register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 88. OUTY_H_G register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 89. OUTZ_L_G register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 90. OUTZ_H_G register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 91. OUTZ_H_G register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 92. OUTX_L_A register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 93. OUTX_H_A register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 94. OUTX_H_A register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80 Table 95. OUTY_L_A register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 81 Table 96. OUTY_H_A register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 81 Table 97. OUTY_H_A register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 81 Table 98. OUTZ_L_A register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 81 Table 99. OUTZ_H_A register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 81 Table 100. OUTZ_H_A register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 81 DocID032891 Rev 1 11/153 

LSM6DSO32 List of tables 

> 153

Table 101. EMB_FUNC_STATUS_MAINPAGE register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 82 Table 102. EMB_FUNC_STATUS_MAINPAGE register description . . . . . . . . . . . . . . . . . . . . . . . . . . 82 Table 103. FSM_STATUS_A_MAINPAGE register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 82 Table 104. FSM_STATUS_A_MAINPAGE register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 82 Table 105. FSM_STATUS_B_MAINPAGE register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 83 Table 106. FSM_STATUS_B_MAINPAGE register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 83 Table 107. STATUS_MASTER_MAINPAGE register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 83 Table 108. STATUS_MASTER_MAINPAGE register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . 83 Table 109. FIFO_STATUS1 register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 84 Table 110. FIFO_STATUS1 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 84 Table 111. FIFO_STATUS2 register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 84 Table 112. FIFO_STATUS2 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 84 Table 113. TIMESTAMP output registers . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 85 Table 114. TIMESTAMP output register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 85 Table 115. TAP_CFG0 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 86 Table 116. TAP_CFG0 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 86 Table 117. TAP_CFG1 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 87 Table 118. TAP_CFG1 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 87 Table 119. TAP priority decoding . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 87 Table 120. TAP_CFG2 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 87 Table 121. TAP_CFG2 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 87 Table 122. TAP_THS_6D register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 88 Table 123. TAP_THS_6D register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 88 Table 124. Threshold for D4D/D6D function. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 88 Table 125. INT_DUR2 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 88 Table 126. INT_DUR2 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 88 Table 127. WAKE_UP_THS register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 89 Table 128. WAKE_UP_THS register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 89 Table 129. WAKE_UP_DUR register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 89 Table 130. WAKE_UP_DUR register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 89 Table 131. FREE_FALL register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 90 Table 132. FREE_FALL register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 90 Table 133. Threshold for free-fall function . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 90 Table 134. MD1_CFG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 91 Table 135. MD1_CFG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 91 Table 136. MD2_CFG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 92 Table 137. MD2_CFG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 92 Table 138. I3C_BUS_AVB register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 Table 139. I3C_BUS_AVB register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 Table 140. INTERNAL_FREQ_FINE register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 Table 141. INTERNAL_FREQ_FINE register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 Table 142. X_OFS_USR register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 Table 143. X_OFS_USR register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93 Table 144. Y_OFS_USR register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 Table 145. Y_OFS_USR register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 Table 146. Z_OFS_USR register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 Table 147. Z_OFS_USR register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 Table 148. FIFO_DATA_OUT_TAG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 Table 149. FIFO_DATA_OUT_TAG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94 Table 150. FIFO tag . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 95 Table 151. FIFO_DATA_OUT_X_H and FIFO_DATA_OUT_X_L registers . . . . . . . . . . . . . . . . . . . . . 96 Table 152. FIFO_DATA_OUT_X_H and FIFO_DATA_OUT_X_L register description . . . . . . . . . . . . 96 List of tables LSM6DSO32  

> 12/153 DocID032891 Rev 1

Table 153. FIFO_DATA_OUT_Y_H and FIFO_DATA_OUT_Y_L registers . . . . . . . . . . . . . . . . . . . . . 96 Table 154. FIFO_DATA_OUT_Y_H and FIFO_DATA_OUT_Y_L register description . . . . . . . . . . . . 96 Table 155. FIFO_DATA_OUT_Z_H and FIFO_DATA_OUT_Z_L registers . . . . . . . . . . . . . . . . . . . . . 96 Table 156. FIFO_DATA_OUT_Z_H and FIFO_DATA_OUT_Z_L register description. . . . . . . . . . . . . 96 Table 157. Register address map - embedded functions . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 97 Table 158. PAGE_SEL register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 99 Table 159. PAGE_SEL register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 99 Table 160. EMB_FUNC_EN_A register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 99 Table 161. EMB_FUNC_EN_A register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 99 Table 162. EMB_FUNC_EN_B register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 Table 163. EMB_FUNC_EN_B register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 Table 164. PAGE_ADDRESS register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 Table 165. PAGE_ADDRESS register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 Table 166. PAGE_VALUE register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 Table 167. PAGE_VALUE register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100 Table 168. EMB_FUNC_INT1 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 101 Table 169. EMB_FUNC_INT1 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 101 Table 170. FSM_INT1_A register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 102 Table 171. FSM_INT1_A register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 102 Table 172. FSM_INT1_B register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 103 Table 173. FSM_INT1_B register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 103 Table 174. EMB_FUNC_INT2 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 104 Table 175. EMB_FUNC_INT2 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 104 Table 176. FSM_INT2_A register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 105 Table 177. FSM_INT2_A register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 105 Table 178. FSM_INT2_B register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 106 Table 179. FSM_INT2_B register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 106 Table 180. EMB_FUNC_STATUS register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 107 Table 181. EMB_FUNC_STATUS register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 107 Table 182. FSM_STATUS_A register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 107 Table 183. FSM_STATUS_A register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 107 Table 184. FSM_STATUS_B register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 108 Table 185. FSM_STATUS_B register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 108 Table 186. PAGE_RW register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 108 Table 187. PAGE_RW register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 108 Table 188. EMB_FUNC_FIFO_CFG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 109 Table 189. EMB_FUNC_FIFO_CFG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 109 Table 190. FSM_ENABLE_A register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 109 Table 191. FSM_ENABLE_A register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 109 Table 192. FSM_ENABLE_B register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110 Table 193. FSM_ENABLE_B register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110 Table 194. FSM_LONG_COUNTER_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110 Table 195. FSM_LONG_COUNTER_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110 Table 196. FSM_LONG_COUNTER_H register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110 Table 197. FSM_LONG_COUNTER_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110 Table 198. FSM_LONG_COUNTER_CLEAR register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 111 Table 199. FSM_LONG_COUNTER_CLEAR register description . . . . . . . . . . . . . . . . . . . . . . . . . . . 111 Table 200. FSM_OUTS1 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 111 Table 201. FSM_OUTS1 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 111 Table 202. FSM_OUTS2 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 112 Table 203. FSM_OUTS2 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 112 Table 204. FSM_OUTS3 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 112 DocID032891 Rev 1 13/153 

LSM6DSO32 List of tables 

> 153

Table 205. FSM_OUTS3 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 112 Table 206. FSM_OUTS4 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 113 Table 207. FSM_OUTS4 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 113 Table 208. FSM_OUTS5 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 113 Table 209. FSM_OUTS5 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 113 Table 210. FSM_OUTS6 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 114 Table 211. FSM_OUTS6 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 114 Table 212. FSM_OUTS7 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 114 Table 213. FSM_OUTS7 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 114 Table 214. FSM_OUTS8 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 115 Table 215. FSM_OUTS8 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 115 Table 216. FSM_OUTS9 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 115 Table 217. FSM_OUTS9 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 115 Table 218. FSM_OUTS10 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 116 Table 219. FSM_OUTS10 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 116 Table 220. FSM_OUTS11 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 116 Table 221. FSM_OUTS11 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 116 Table 222. FSM_OUTS12 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 117 Table 223. FSM_OUTS12 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 117 Table 224. FSM_OUTS13 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 117 Table 225. FSM_OUTS13 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 117 Table 226. FSM_OUTS14 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 118 Table 227. FSM_OUTS14 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 118 Table 228. FSM_OUTS15 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 118 Table 229. FSM_OUTS15 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 118 Table 230. FSM_OUTS16 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 119 Table 231. FSM_OUTS16 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 119 Table 232. EMB_FUNC_ODR_CFG_B register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 119 Table 233. EMB_FUNC_ODR_CFG_B register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 119 Table 234. STEP_COUNTER_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 120 Table 235. STEP_COUNTER_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 120 Table 236. STEP_COUNTER_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 120 Table 237. STEP_COUNTER_H register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 120 Table 238. EMB_FUNC_SRC register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 120 Table 239. EMB_FUNC_SRC register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 120 Table 240. EMB_FUNC_INIT_A register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 121 Table 241. EMB_FUNC_INIT_A register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 121 Table 242. EMB_FUNC_INIT_B register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 121 Table 243. EMB_FUNC_INIT_B register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 121 Table 244. Register address map - embedded advanced features page 0 . . . . . . . . . . . . . . . . . . . . 122 Table 245. Register address map - embedded advanced features page 1 . . . . . . . . . . . . . . . . . . . . 123 Table 246. MAG_SENSITIVITY_L register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125 Table 247. MAG_SENSITIVITY_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125 Table 248. MAG_SENSITIVITY_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125 Table 249. MAG_SENSITIVITY_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125 Table 250. MAG_OFFX_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125 Table 251. MAG_OFFX_L register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125 Table 252. MAG_OFFX_H register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125 Table 253. MAG_OFFX_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125 Table 254. MAG_OFFY_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 126 Table 255. MAG_OFFY_L register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 126 Table 256. MAG_OFFY_H register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 126 List of tables LSM6DSO32  

> 14/153 DocID032891 Rev 1

Table 257. MAG_OFFY_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 126 Table 258. MAG_OFFZ_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 126 Table 259. MAG_OFFZ_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 126 Table 260. MAG_OFFZ_H register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 126 Table 261. MAG_OFFZ_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 126 Table 262. MAG_SI_XX_L register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127 Table 263. MAG_SI_XX_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127 Table 264. MAG_SI_XX_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127 Table 265. MAG_SI_XX_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127 Table 266. MAG_SI_XY_L register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127 Table 267. MAG_SI_XY_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127 Table 268. MAG_SI_XY_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127 Table 269. MAG_SI_XY_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127 Table 270. MAG_SI_XZ_L register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128 Table 271. MAG_SI_XZ_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128 Table 272. MAG_SI_XZ_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128 Table 273. MAG_SI_XZ_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128 Table 274. MAG_SI_YY_L register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128 Table 275. MAG_SI_YY_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128 Table 276. MAG_SI_YY_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128 Table 277. MAG_SI_YY_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128 Table 278. MAG_SI_YZ_L register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129 Table 279. MAG_SI_YZ_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129 Table 280. MAG_SI_YZ_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129 Table 281. MAG_SI_YZ_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129 Table 282. MAG_SI_ZZ_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129 Table 283. MAG_SI_ZZ_L register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129 Table 284. MAG_SI_ZZ_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129 Table 285. MAG_SI_ZZ_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129 Table 286. MAG_CFG_A register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 130 Table 287. MAG_CFG_A description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 130 Table 288. MAG_CFG_B register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 130 Table 289. MAG_CFG_B description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 130 Table 290. FSM_LC_TIMEOUT_L register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 131 Table 291. FSM_LC_TIMEOUT_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 131 Table 292. FSM_LC_TIMEOUT_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 131 Table 293. FSM_LC_TIMEOUT_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 131 Table 294. FSM_PROGRAMS register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 131 Table 295. FSM_PROGRAMS register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 131 Table 296. FSM_START_ADD_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 Table 297. FSM_START_ADD_L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 Table 298. FSM_START_ADD_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 Table 299. FSM_START_ADD_H register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 Table 300. PEDO_CMD_REG register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 Table 301. PEDO_CMD_REG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 Table 302. PEDO_DEB_STEPS_CONF register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 Table 303. PEDO_DEB_STEPS_CONF register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132 Table 304. PEDO_SC_DELTAT_L register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 133 Table 305. PEDO_SC_DELTAT_H register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 133 Table 306. PEDO_SC_DELTAT_H/L register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 133 Table 307. Register address map - sensor hub registers . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 134 Table 308. SENSOR_HUB_1 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 DocID032891 Rev 1 15/153 

LSM6DSO32 List of tables 

> 153

Table 309. SENSOR_HUB_1 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 Table 310. SENSOR_HUB_2 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 Table 311. SENSOR_HUB_2 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 Table 312. SENSOR_HUB_3 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 Table 313. SENSOR_HUB_3 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 136 Table 314. SENSOR_HUB_4 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 Table 315. SENSOR_HUB_4 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 Table 316. SENSOR_HUB_5 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 Table 317. SENSOR_HUB_5 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 Table 318. SENSOR_HUB_6 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 Table 319. SENSOR_HUB_6 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 137 Table 320. SENSOR_HUB_7 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 Table 321. SENSOR_HUB_7 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 Table 322. SENSOR_HUB_8 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 Table 323. SENSOR_HUB_8 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 Table 324. SENSOR_HUB_9 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 Table 325. SENSOR_HUB_9 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 138 Table 326. SENSOR_HUB_10 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 Table 327. SENSOR_HUB_10 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 Table 328. SENSOR_HUB_11 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 Table 329. SENSOR_HUB_11 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 Table 330. SENSOR_HUB_12 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 Table 331. SENSOR_HUB_12 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 139 Table 332. SENSOR_HUB_13 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 Table 333. SENSOR_HUB_13 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 Table 334. SENSOR_HUB_14 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 Table 335. SENSOR_HUB_14 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 Table 336. SENSOR_HUB_15 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 Table 337. SENSOR_HUB_15 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 140 Table 338. SENSOR_HUB_16 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 Table 339. SENSOR_HUB_16 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 Table 340. SENSOR_HUB_17 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 Table 341. SENSOR_HUB_17 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 Table 342. SENSOR_HUB_17 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 Table 343. SENSOR_HUB_17 register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 141 Table 344. MASTER_CONFIG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 142 Table 345. MASTER_CONFIG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 142 Table 346. SLV0_ADD register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 142 Table 347. SLV_ADD register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 142 Table 348. SLV0_SUBADD register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 143 Table 349. SLV0_SUBADD register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 143 Table 350. SLAVE0_CONFIG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 143 Table 351. SLAVE0_CONFIG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 143 Table 352. SLV1_ADD register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 Table 353. SLV1_ADD register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 Table 354. SLV1_SUBADD register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 Table 355. SLV1_SUBADD register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 Table 356. SLAVE1_CONFIG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 Table 357. SLAVE1_CONFIG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 144 Table 358. SLV2_ADD register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 Table 359. SLV2_ADD register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 Table 360. SLV2_SUBADD register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 List of tables LSM6DSO32  

> 16/153 DocID032891 Rev 1

Table 361. SLV2_SUBADD register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 Table 362. SLAVE2_CONFIG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 Table 363. SLAVE2_CONFIG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 145 Table 364. SLV3_ADD register. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 Table 365. SLV3_ADD register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 Table 366. SLV3_SUBADD register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 Table 367. SLV3_SUBADD register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 Table 368. SLAVE3_CONFIG register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 Table 369. SLAVE3_CONFIG register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146 Table 370. DATAWRITE_SLV0 register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 147 Table 371. DATAWRITE_SLV0 register description. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 147 Table 372. STATUS_MASTER register . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 147 Table 373. STATUS_MASTER register description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 147 Table 374. Reel dimensions for carrier tape of LGA-14 package . . . . . . . . . . . . . . . . . . . . . . . . . . . . 151 Table 375. Document revision history. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 152 DocID032891 Rev 1 17/153 

LSM6DSO32 List of figures 

> 153

# List of figures 

Figure 1. Generic state machine . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 21 Figure 2. State machine in the LSM6DSO32 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 21 Figure 3. Pin connections . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 22 Figure 4. LSM6DSO32 connection modes. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 23 Figure 5. SPI slave timing diagram (in mode 3) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 30 Figure 6. I²C slave timing diagram . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 31 Figure 7. Read and write protocol (in mode 3). . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 37 Figure 8. SPI read protocol (in mode 3) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 38 Figure 9. Multiple byte SPI read protocol (2-byte example) (in mode 3) . . . . . . . . . . . . . . . . . . . . . . 38 Figure 10. SPI write protocol (in mode 3). . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 39 Figure 11. Multiple byte SPI write protocol (2-byte example) (in mode 3) . . . . . . . . . . . . . . . . . . . . . . 39 Figure 12. SPI read protocol in 3-wire mode (in mode 3) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 40 Figure 13. I²C and MIPI I3C SM both active (INT1 pin not connected) . . . . . . . . . . . . . . . . . . . . . . . . . 43 Figure 14. Only MIPI I3C SM active (INT1 pin connected to VDD_IO) . . . . . . . . . . . . . . . . . . . . . . . . . 43 Figure 15. Block diagram of filters . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 46 Figure 16. Accelerometer chain . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 46 Figure 17. Accelerometer composite filter . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 47 Figure 18. Gyroscope digital chain - Mode 1 and Mode 2 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 48 Figure 19. LSM6DSO32 electrical connections in Mode 1. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 53 Figure 20. LSM6DSO32 electrical connections in Mode 2. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 54 Figure 21. Accelerometer block diagram . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 75 Figure 22. LGA-14L 2.5x3x0.86 mm package outline and mechanical data . . . . . . . . . . . . . . . . . . . 149 Figure 23. Carrier tape information for LGA-14 package . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 150 Figure 24. LGA-14 package orientation in carrier tape . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 150 Figure 25. Reel information for carrier tape of LGA-14 package . . . . . . . . . . . . . . . . . . . . . . . . . . . . 151 Overview LSM6DSO32  

> 18/153 DocID032891 Rev 1

# 1 Overview 

The LSM6DSO32 is a system-in-package featuring a high-performance 3-axis digital accelerometer and 3-axis digital gyroscope. 

The LSM6DSO32 delivers best-in-class motion sensing that can detect orientation and gestures in order to empower application developers and consumers with features and capabilities that are more sophisticated than simply orienting their devices to portrait and landscape mode. 

The event-detection interrupts enable efficient and reliable motion tracking and contextual awareness, implementing hardware recognition of free-fall events, 6D orientation, click and double-click sensing, activity or inactivity, stationary/motion detection and wakeup events. 

The LSM6DSO32 supports main OS requirements, offering real, virtual and batch mode sensors. In addition, the LSM6DSO32 can efficiently run the sensor-related features specified in Android, saving power and enabling faster reaction time. In particular, the LSM6DSO32 has been designed to implement hardware features such as significant motion detection, stationary/motion detection, tilt, pedometer functions, timestamping and to support the data acquisition of an external magnetometer. 

The LSM6DSO32 offers hardware flexibility to connect the pins with different mode connections to external sensors to expand functionalities such as adding a sensor hub, etc. 

Up to 9 kbytes of FIFO with compression and dynamic allocation of significant data (i.e. external sensors, timestamp, etc.) allows overall power saving of the system. 

Like the entire portfolio of MEMS sensor modules, the LSM6DSO32 leverages the robust and mature in-house manufacturing processes already used for the production of micromachined accelerometers and gyroscopes. The various sensing elements are manufactured using specialized micromachining processes, while the IC interfaces are developed using CMOS technology that allows the design of a dedicated circuit which is trimmed to better match the characteristics of the sensing element. 

The LSM6DSO32 is available in a small plastic land grid array (LGA) package of 

2.5 x 3.0 x 0.83 mm to address ultra-compact solutions. DocID032891 Rev 1 19/153 

LSM6DSO32 Embedded low-power features 

> 153

# 2 Embedded low-power features 

The LSM6DSO32 has been designed to be fully compliant with Android, featuring the following on-chip functions: 

 9 kbytes data buffering, data can be compressed two or three times – 100% efficiency with flexible configurations and partitioning – Possibility to store timestamp 

 Event-detection interrupts (fully configurable): – Free-fall – Wakeup – 6D orientation – Click and double-click sensing – Activity/inactivity recognition – Stationary/Motion detection 

 Specific IP blocks with negligible power consumption and high-performance: – Pedometer functions: step detector and step counters – Tilt – Significant Motion Detection – Finite State Machine (FSM) for accelerometer, gyroscope, and external sensors 

 Sensor hub – Up to 6 total sensors: 2 internal (accelerometer and gyroscope) and 4 external sensors Embedded low-power features LSM6DSO32  

> 20/153 DocID032891 Rev 1

# 2.1 Tilt detection 

The tilt function helps to detect activity change and has been implemented in hardware using only the accelerometer to achieve targets of both ultra-low power consumption and robustness during the short duration of dynamic accelerations. 

The tilt function is based on a trigger of an event each time the device's tilt changes and can be used with different scenarios, for example: 

a) Triggers when phone is in a front pants pocket and the user goes from sitting to standing or standing to sitting; b) Doesn’t trigger when phone is in a front pants pocket and the user is walking, running or going upstairs. 

# 2.2 Significant Motion Detection 

The Significant Motion Detection (SMD) function generates an interrupt when a ‘significant motion’, that could be due to a change in user location, is detected. In the LSM6DSO32 device this function has been implemented in hardware using only the accelerometer. 

SMD functionality can be used in location-based applications in order to receive a notification indicating when the user is changing location. 

# 2.3 Finite State Machine 

The LSM6DSO32 can be configured to generate interrupt signals activated by user-defined motion patterns. To do this, up to 16 embedded finite state machines can be programmed independently for motion detection such as glance gestures, absolute wrist tilt, shake and double-shake detection. 

Definition of Finite State Machine 

A state machine is a mathematical abstraction used to design logic connections. It is a behavioral model composed of a finite number of states and transitions between states, similar to a flow chart in which one can inspect the way logic runs when certain conditions are met. The state machine begins with a start state, goes to different states through transitions dependent on the inputs, and can finally end in a specific state (called stop state). The current state is determined by the past states of the system.  Figure  1: Generic state 

machine  shows a generic state machine. DocID032891 Rev 1 21/153 

LSM6DSO32 Embedded low-power features 

> 153

Figure 1. Generic state machine Finite State Machine in the LSM6DSO32 

The LSM6DSO32 works as a combo accelerometer-gyroscope sensor, generating acceleration and angular rate output data. It is also possible to connect an external sensor (magnetometer) by using the Sensor Hub feature (Mode 2). These data can be used as input of up to 16 programs in the embedded Finite State Machine ( Figure  2: State machine 

in the LSM6DSO32 ). 

All 16 finite state machines are independent: each one has its dedicated memory area and it is independently executed. An interrupt is generated when the end state is reached or when some specific command is performed. 

Figure 2. State machine in the LSM6DSO32 Pin description LSM6DSO32 

22/153 DocID032891 Rev 1 

# 3 Pin description Figure 3. Pin connections 

1. Leave pin electrically unconnected and soldered to PCB. 

## ΩΩ Y

## Ω R

## Ω P

## ZYXNC (1) NC (1) INT2 VDD SDO/SA0 CS SCL SDA SDx INT1 SCx GND GND VDDIO DocID032891 Rev 1 23/153 

LSM6DSO32 Pin description 

> 153

# 3.1 Pin connections 

The LSM6DSO32 offers flexibility to connect the pins in order to have two different mode connections and functionalities. In detail: 

 Mode 1 : I²C / MIPI I3C SM slave interface or SPI (3- and 4-wire) serial interface is available; 

 Mode 2 : I²C / MIPI I3C SM slave interface or SPI (3- and 4-wire) serial interface and I²C interface master for external sensor connections are available. 

Figure 4. LSM6DSO32 connection modes 

In the following table each mode is described for the pin connections and function. 

# +267 /60'62 +267 /60'62 /60'60 /60'60 ([WHUQDO VHQVRUV ,  &  63, Z ,  &  63, Z 0DVWHU,  &0RGH 0RGH 0,3,,& 60   0,3,,& 60  Pin description LSM6DSO32 

24/153 DocID032891 Rev 1 

Table 2. Pin description Pin# Name Mode 1 function Mode 2 function 

1SDO SPI 4-wire interface serial data output (SDO) SPI 4-wire interface serial data output (SDO) SA0 I²C least significant bit of the device address (SA0) MIPI I3C SM least significant bit of the static address (SA0) I²C least significant bit of the device address (SA0) MIPI I3C SM least significant bit of the static address (SA0) 2 SDx Connect to VDDIO or GND I²C serial data master (MSDA) 3 SCx Connect to VDDIO or GND I²C serial clock master (MSCL) 4 INT1 Programmable interrupt 1 / If device is used as MIPI I3C SM pure slave, this pin must be set to ‘1’. Programmable interrupt 1 / If device is used as MIPI I3C SM pure slave, this pin must be set to ‘1’. 5 VDDIO (1) Power supply for I/O pins 6 GND 0 V supply 7 GND 0 V supply 8 VDD (1) Power supply 9 INT2 Programmable interrupt 2 (INT2) / Data enable (DEN) Programmable interrupt 2 (INT2)/ Data enable (DEN)/ I²C master external synchronization signal (MDRDY) 10 NC Leave unconnected (2) Leave unconnected (2) 11 NC Leave unconnected (2) Leave unconnected (2) 12 CS I²C/MIPI I3C SM /SPI mode selection (1: SPI idle mode / I²C/MIPI I3C SM communication enabled; 0: SPI communication mode / I²C/MIPI I3C SM disabled) I²C/MIPI I3C SM /SPI mode selection (1: SPI idle mode / I²C/MIPI I3C SM communication enabled; 0: SPI communication mode / I²C/MIPI I3C SM disabled) 13 SCL I²C/MIPI I3C SM serial clock (SCL) SPI serial port clock (SPC) I²C/MIPI I3C SM serial clock (SCL) SPI serial port clock (SPC) 14 SDA I²C/MIPI I3C SM serial data (SDA) SPI serial data input (SDI) 3-wire interface serial data output (SDO) I²C/MIPI I3C SM serial data (SDA) SPI serial data input (SDI) 3-wire interface serial data output (SDO) 1. Recommended 100 nF filter capacitor. 2. Leave pin electrically unconnected and soldered to PCB. DocID032891 Rev 1 25/153 

LSM6DSO32 Module specifications 

> 153

# 4 Module specifications 4.1 Mechanical characteristics 

@ Vdd = 1.8 V, T = 25 °C, unless otherwise noted. 

Table 3. Mechanical characteristics Symbol Parameter Test conditions Min. Typ. (1) Max. Unit 

LA_FS Linear acceleration measurement range ±4 

g

±8 ±16 ±32 G_FS Angular rate measurement range ±125 dps ±250 ±500 ±1000 ±2000 LA_So Linear acceleration sensitivity (2) FS = ±4 g 0.122 mg/LSB FS = ±8 g 0.244 FS = ±16 g 0.488 FS = ±32 g 0.976 G_So Angular rate sensitivity (2) FS = ±125 dps 4.375 mdps/LSB FS = ±250 dps 8.75 FS = ±500 dps 17.50 FS = ±1000 dps 35 FS = ±2000 dps 70 LA_So% Linear acceleration sensitivity tolerance (3) at component level ±0.5 %G_So% Angular rate sensitivity tolerance (3) at component level ±0.5 %LA_SoDr Linear acceleration sensitivity change vs. temperature (4) from -40° to +85° ±0.007 %/°C G_SoDr Angular rate sensitivity change vs. temperature (4) from -40° to +85° ±0.005 %/°C LA_TyOff Linear acceleration zero-g level offset accuracy (5) ±20 mg

G_TyOff Angular rate zero-rate level offset accuracy (5) ±0.5 dps LA_OffDr Linear acceleration zero-g level change vs. temperature (4) ±0.1 mg/ °C G_OffDr Angular rate typical zero-rate level change vs. temperature (4) ±0.01 dps/°C Module specifications LSM6DSO32 

26/153 DocID032891 Rev 1 

Rn Rate noise density in high-performance mode (6) 3.8 mdps/ Hz 

RnRMS Gyroscope RMS noise in normal/low-power mode (7) 75 mdps An Acceleration noise density in high-performance mode (8) FS = ±4 g 120 μg/√Hz FS = ±8 g 130 FS = ±16 g 160 FS = ±32 g 220 RMS Acceleration RMS noise in normal/low-power mode (9)(10) FS = ±4 g 3.2 mg(RMS) FS = ±8 g 3.4 FS = ±16 g 4.0 FS = ±32 g 5.4 Acceleration RMS noise in ultra-low-power mode (9)(10) FS = ±4 g 9.5 LA_ODR Linear acceleration output data rate 1.6 (11) 12.5 26 52 104 208 416 833 1666 3332 6664 Hz G_ODR Angular rate output data rate 12.5 26 52 104 208 416 833 1666 3332 6664 Vst Linear acceleration self-test output change (12)(13)(14) 50 1700 mg

Angular rate self-test output change (15)(16) FS = ±250 dps 20 80 dps FS = ±2000 dps 150 700 dps Top Operating temperature range -40 +85 °C 1. Typical specifications are not guaranteed. 2. Sensitivity values after factory calibration test and trimming. 

Table 3. Mechanical characteristics (continued) Symbol Parameter Test conditions Min. Typ. (1) Max. Unit DocID032891 Rev 1 27/153 

LSM6DSO32 Module specifications 

> 153

3. Subject to change. 4. Measurements are performed in a uniform temperature setup and they are based on characterization data in a limited number of samples. Not measured during final test for production. 5. Values after factory calibration test and trimming. 6. Gyroscope rate noise density in high-performance mode is independent of the ODR and FS setting. 7. Gyroscope RMS noise in normal/low-power mode is independent of the ODR and FS setting. 8. Accelerometer noise density in high-performance mode is independent of the ODR. 9. Accelerometer RMS noise in normal/low-power/ultra-low-power mode is independent of the ODR. 10. Noise RMS related to BW = ODR/2. 11. This ODR is available when the accelerometer is in low-power mode. 12. The sign of the linear acceleration self-test output change is defined by the STx_XL bits in a dedicated register for all axes. 13. The linear acceleration self-test output change is defined with the device in stationary condition as the absolute value of: OUTPUT[LSb] (self-test enabled) - OUTPUT[LSb] (self-test disabled). 1LSb = 0.122 m g at ±4 g full scale. 14. Accelerometer self-test limits are full-scale independent. 15. The sign of the angular rate self-test output change is defined by the STx_G bits in a dedicated register for all axes. 16. The angular rate self-test output change is defined with the device in stationary condition as the absolute value of: OUTPUT[LSb] (self-test enabled) - OUTPUT[LSb] (self-test disabled). 1LSb = 70 mdps at ±2000 dps full scale. Module specifications LSM6DSO32 

28/153 DocID032891 Rev 1 

# 4.2 Electrical characteristics 

@ Vdd = 1.8 V, T = 25 °C, unless otherwise noted. 

Table 4. Electrical characteristics 

Symbol Parameter Test conditions Min. Typ. (1) 

1. T ypical specifications are not guaranteed. 

Max. Unit 

Vdd Supply voltage 1.71 1.8 3.6 VVdd_IO Power supply for I/O 1.62 3.6 VIddHP Gyroscope and accelerometer current consumption in high-performance mode 0.55 mA LA_IddHP Accelerometer current consumption in high-performance mode 170 μA LA_IddLP Accelerometer current consumption in low-power mode ODR = 52 Hz ODR = 1.6 Hz 26 4.5 μA LA_IddULP Accelerometer current consumption in ultra-low-power mode ODR = 52 Hz ODR = 1.6 Hz 9.5 4.4 μA IddPD Gyroscope and accelerometer current consumption during power-down 3 μA Ton Turn-on time 35 ms V IH Digital high-level input voltage 0.7 * VDD_IO VV IL Digital low-level input voltage 0.3 * VDD_IO VVOH High-level output voltage I OH = 4 mA (2) 2. 4 mA is the maximum driving capability, i.e. the maximum DC current that can be sourced/sunk by the digital pin in order to guarantee the correct digital output voltage levels V OH and V OL .VDD_IO -0.2 VV OL Low-level output voltage I OL = 4 mA (2) 0.2 VTop Operating temperature range -40 +85 °C DocID032891 Rev 1 29/153 

LSM6DSO32 Module specifications 

> 153

# 4.3 Temperature sensor characteristics 

@ Vdd = 1.8 V, T = 25 °C unless otherwise noted. 

Table 5. Temperature sensor characteristics Symbol Parameter Test condition Min. Typ. (1) 

1. Typical specifications are not guaranteed. 

Max. Unit 

TODR (2) 2. When the accelerometer is in low-power mode or ultra-low-power mode and the gyroscope part is turned off, the TODR value is equal to the accelerometer ODR. Temperature refresh rate 52 Hz Toff Temperature offset (3) 3. The output of the temperature sensor is 0 LSB (typ.) at 25 °C. -15 +15 °C TSen Temperature sensitivity 256 LSB/°C TST Temperature stabilization time (4) 4. Time from power ON to valid data based on characterization data. 500 μs T_ADC_res Temperature ADC resolution 16 bit Top Operating temperature range -40 +85 °C Module specifications LSM6DSO32 

30/153 DocID032891 Rev 1 

# 4.4 Communication interface characteristics 4.4.1 SPI - serial peripheral interface 

Subject to general operating conditions for Vdd and Top. 

Figure 5. SPI slave timing diagram (in mode 3) 

Note: Measurement points are done at 0.2·Vdd_IO and 0.8·Vdd_IO, for both input and output ports. 

Table 6. SPI slave timing values (in mode 3) Symbol Parameter Value (1) Unit Min Max 

t c(SPC) SPI clock cycle 100 ns f c(SPC) SPI clock frequency 10 MHz tsu(CS) CS setup time 5ns th(CS) CS hold time 20 tsu(SI) SDI input setup time 5th(SI) SDI input hold time 15 tv(SO) SDO valid output time 50 th(SO) SDO output hold time 5tdis(SO) SDO output disable time 50 1. Values are guaranteed at 10 MHz clock frequency for SPI with both 4 and 3 wires, based on characterization results, not tested in production DocID032891 Rev 1 31/153 

LSM6DSO32 Module specifications 

> 153

## 4.4.2 I²C - inter-IC control interface 

Subject to general operating conditions for Vdd and Top. 

Figure 6. I ²C slave timing diagram 

Note: Measurement points are done at 0.2·Vdd_IO and 0.8·Vdd_IO, for both ports. 

Table 7. I ²C slave timing values Symbol Parameter I²C standard mode (1) I²C fast mode (1) Unit Min Max Min Max 

f(SCL) SCL clock frequency 0 100 0 400 kHz t w(SCLL) SCL clock low time 4.7 1.3 μs tw(SCLH) SCL clock high time 4.0 0.6 tsu(SDA) SDA setup time 250 100 ns th(SDA) SDA data hold time 0 3.45 0 0.9 μs t h(ST) START condition hold time 4 0.6 μs t su(SR) Repeated START condition setup time 4.7 0.6 tsu(SP) STOP condition setup time 4 0.6 tw(SP:SR) Bus free time between STOP and START condition 4.7 1.3 1. Data based on standard I²C protocol requirement, not tested in production. 

6'$ 6&/ W VX 63 W Z 6&// W VX 6'$ W VX 65 W K 67 W Z 6&/+ W K 6'$ W Z 6365 67$57 5(3($7(' 67$57 6723 67$57 Module specifications LSM6DSO32 

32/153 DocID032891 Rev 1 

# 4.5 Absolute maximum ratings 

Stresses above those listed as “Absolute maximum ratings” may cause permanent damage to the device. This is a stress rating only and functional operation of the device under these conditions is not implied. Exposure to maximum rating conditions for extended periods may affect device reliability. 

Note: Supply voltage on any pin should never exceed 4.8 V. 

Table 8. Absolute maximum ratings Symbol Ratings Maximum value Unit 

Vdd / Vdd_IO Supply voltage -0.3 to 4.8 VT STG Storage temperature range -40 to +125 °C Sg Acceleration g for 0.2 ms 20,000 g

ESD Electrostatic discharge protection (HBM) 2 kV Vin Input voltage on any control pin (including CS, SCL/SPC, SDA/SDI/SDO, SDO/SA0) -0.3 to Vdd_IO +0.3 V

This device is sensitive to mechanical shock, improper handling can cause permanent damage to the part. This device is sensitive to electrostatic discharge (ESD), improper handling can cause permanent damage to the part. DocID032891 Rev 1 33/153 

LSM6DSO32 Module specifications 

> 153

# 4.6 Terminology 4.6.1 Sensitivity 

Linear acceleration sensitivity can be determined, for example, by applying 1 g acceleration to the device. Because the sensor can measure DC accelerations, this can be done easily by pointing the selected axis towards the ground, noting the output value, rotating the sensor 180 degrees (pointing towards the sky) and noting the output value again. By doing so, ±1 g acceleration is applied to the sensor. Subtracting the larger output value from the smaller one, and dividing the result by 2, leads to the actual sensitivity of the sensor. This value changes very little over temperature and over time. The sensitivity tolerance describes the range of sensitivities of a large number of sensors (see  Table  3). 

An angular rate gyroscope is a device that produces a positive-going digital output for counterclockwise rotation around the axis considered. Sensitivity describes the gain of the sensor and can be determined by applying a defined angular velocity to it. This value changes very little over temperature and time (see  Table  3). 

## 4.6.2 Zero-g and zero-rate level 

Linear acceleration zero-g level offset (TyOff) describes the deviation of an actual output signal from the ideal output signal if no acceleration is present. A sensor in a steady state on a horizontal surface will measure 0 g on both the X-axis and Y-axis, whereas the Z-axis will measure 1 g. Ideally, the output is in the middle of the dynamic range of the sensor (content of OUT registers 00h, data expressed as 2’s complement number). A deviation from the ideal value in this case is called zero-g offset. 

Offset is to some extent a result of stress to MEMS sensor and therefore the offset can slightly change after mounting the sensor onto a printed circuit board or exposing it to extensive mechanical stress. Offset changes little over temperature, see “Linear acceleration zero-g level change vs. temperature” in  Table  3. The zero-g level tolerance 

(TyOff) describes the standard deviation of the range of zero-g levels of a group of sensors. 

Zero-rate level describes the actual output signal if there is no angular rate present. The zero-rate level of precise MEMS sensors is, to some extent, a result of stress to the sensor and therefore the zero-rate level can slightly change after mounting the sensor onto a printed circuit board or after exposing it to extensive mechanical stress. This value changes very little over temperature and time (see  Table  3). Digital interfaces LSM6DSO32 

34/153 DocID032891 Rev 1 

# 5 Digital interfaces 5.1 I²C/SPI interface 

The registers embedded inside the LSM6DSO32 may be accessed through both the I²C and SPI serial interfaces. The latter may be SW configured to operate either in 3-wire or 4-wire interface mode. The device is compatible with SPI modes 0 and 3. 

The serial interfaces are mapped onto the same pins. To select/exploit the I²C interface, the CS line must be tied high (i.e connected to Vdd_IO). 

## 5.1.1 I²C serial interface 

The LSM6DSO32 I²C is a bus slave. The I²C is employed to write the data to the registers, whose content can also be read back. 

The relevant I²C terminology is provided in the table below. 

There are two signals associated with the I²C bus: the serial clock line (SCL) and the Serial DAta line (SDA). The latter is a bidirectional line used for sending and receiving the data to/from the interface. Both the lines must be connected to Vdd_IO through external pull-up resistors. When the bus is free, both the lines are high. 

The I²C interface is implemented with fast mode (400 kHz) I²C standards as well as with the standard mode. 

In order to disable the I²C block, (I2C_disable) = 1 must be written in  CTRL4_C (13h) .

Table 9. Serial interface pin description Pin name Pin description 

CS SPI enable I²C/SPI mode selection (1: SPI idle mode / I²C communication enabled; 0: SPI communication mode / I²C disabled) SCL/SPC I²C Serial Clock (SCL) SPI Serial Port Clock (SPC) SDA/SDI/SDO I²C Serial Data (SDA) SPI Serial Data Input (SDI) 3-wire Interface Serial Data Output (SDO) SDO/SA0 SPI Serial Data Output (SDO) I²C less significant bit of the device address 

Table 10. I ²C terminology Term Description 

Transmitter The device which sends data to the bus Receiver The device which receives data from the bus Master The device which initiates a transfer, generates clock signals and terminates a transfer Slave The device addressed by the master DocID032891 Rev 1 35/153 

LSM6DSO32 Digital interfaces 

> 153

I²C operation 

The transaction on the bus is started through a START (ST) signal. A START condition is defined as a HIGH to LOW transition on the data line while the SCL line is held HIGH. After this has been transmitted by the master, the bus is considered busy. The next byte of data transmitted after the start condition contains the address of the slave in the first 7 bits and the eighth bit tells whether the master is receiving data from the slave or transmitting data to the slave. When an address is sent, each device in the system compares the first seven bits after a start condition with its address. If they match, the device considers itself addressed by the master. 

The Slave ADdress (SAD) associated to the LSM6DSO32 is 110101xb. The SDO/SA0 pin can be used to modify the less significant bit of the device address. If the SDO/SA0 pin is connected to the supply voltage, LSb is ‘1’ (address 1101011b); else if the SDO/SA0 pin is connected to ground, the LSb value is ‘0’ (address 1101010b). This solution permits to connect and address two different inertial modules to the same I²C bus. 

Data transfer with acknowledge is mandatory. The transmitter must release the SDA line during the acknowledge pulse. The receiver must then pull the data line LOW so that it remains stable low during the HIGH period of the acknowledge clock pulse. A receiver which has been addressed is obliged to generate an acknowledge after each byte of data received. 

The I²C embedded inside the LSM6DSO32 behaves like a slave device and the following protocol must be adhered to. After the start condition (ST) a slave address is sent, once a slave acknowledge (SAK) has been returned, an 8-bit sub-address (SUB) is transmitted. The increment of the address is configured by the  CTRL3_C (12h)  (IF_INC). 

The slave address is completed with a Read/Write bit. If the bit is ‘1’ (Read), a repeated START (SR) condition must be issued after the two sub-address bytes; if the bit is ‘0’ (Write) the master will transmit to the slave with direction unchanged.  Table  11  explains how the 

SAD+Read/Write bit pattern is composed, listing all the possible configurations. 

Table 11. SAD+Read/Write patterns      

> Command SAD[6:1] SAD[0] = SA0 R/W SAD+R/W
> Read

110101 0 1 11010101 (D5h) Write 110101 0 0 11010100 (D4h) Read 110101 1 1 11010111 (D7h) Write 110101 1 0 11010110 (D6h) 

Table 12. Transfer when master is writing one byte to slave         

> Master ST SAD + W SUB DATA SP Slave SAK SAK SAK

Table 13. Transfer when master is writing multiple bytes to slave           

> Master ST SAD + W SUB DATA DATA SP Slave SAK SAK SAK SAK

Digital interfaces LSM6DSO32  

> 36/153 DocID032891 Rev 1

Data are transmitted in byte format (DATA). Each data transfer contains 8 bits. The number of bytes transferred per transfer is unlimited. Data is transferred with the Most Significant bit (MSb) first. If a slave receiver doesn’t acknowledge the slave address (i.e. it is not able to receive because it is performing some real-time function) the data line must be left HIGH by the slave. The master can then abort the transfer. A LOW to HIGH transition on the SDA line while the SCL line is HIGH is defined as a STOP condition. Each data transfer must be terminated by the generation of a STOP (SP) condition. 

In the presented communication format MAK is Master acknowledge and NMAK is No Master Acknowledge. 

Table 14. Transfer when master is receiving (reading) one byte of data from slave            

> Master ST SAD + W SUB SR SAD + R NMAK SP Slave SAK SAK SAK DATA

Table 15. Transfer when master is receiving (reading) multiple bytes of data from slave                 

> Master ST SAD+W SUB SR SAD+R MAK MAK NMAK SP Slave SAK SAK SAK DATA DAT ADATA DocID032891 Rev 1 37/153

LSM6DSO32 Digital interfaces 

> 153

## 5.1.2 SPI bus interface 

The LSM6DSO32 SPI is a bus slave. The SPI allows writing and reading the registers of the device. 

The serial interface communicates to the application using 4 wires: CS , SPC , SDI and SDO .

Figure 7. Read and write protocol (in mode 3) 

CS is the serial port enable and it is controlled by the SPI master. It goes low at the start of the transmission and goes back high at the end. SPC is the serial port clock and it is controlled by the SPI master. It is stopped high when CS is high (no transmission). SDI and 

SDO are, respectively, the serial port data input and output. Those lines are driven at the falling edge of SPC and should be captured at the rising edge of SPC .

Both the read register and write register commands are completed in 16 clock pulses or in multiples of 8 in case of multiple read/write bytes. Bit duration is the time between two falling edges of SPC . The first bit (bit 0) starts at the first falling edge of SPC after the falling edge of CS while the last bit (bit 15, bit 23, ...) starts at the last falling edge of SPC just before the rising edge of CS .

bit 0 : R W bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0) from the device is read. In latter case, the chip will drive SDO at the start of bit 8. 

bit 1-7 : address AD(6:0). This is the address field of the indexed register. 

bit 8-15 : data DI(7:0) (write mode). This is the data that is written into the device (MSb first). 

bit 8-15 : data DO(7:0) (read mode). This is the data that is read from the device (MSb first). 

In multiple read/write commands further blocks of 8 clock periods will be added. When the 

CTRL3_C (12h)  (IF_INC) bit is ‘0’, the address used to read/write data remains the same for 

every block. When the  CTRL3_C (12h)  (IF_INC) bit is ‘1’, the address used to read/write 

data is increased at every block. 

The function and the behavior of SDI and SDO remain unchanged.        

> &6 63& 6', 6'2 5: $' $' $' $' $' $' ', ', ', ', ', ', ', ', '2 '2 '2 '2 '2 '2 '2 '2 $'

Digital interfaces LSM6DSO32 

38/153 DocID032891 Rev 1 

SPI read Figure 8. SPI read protocol (in mode 3) 

The SPI Read command is performed with 16 clock pulses. A multiple byte read command is performed by adding blocks of 8 clock pulses to the previous one. 

bit 0 : READ bit. The value is 1. 

bit 1-7 : address AD(6:0). This is the address field of the indexed register. 

bit 8-15 : data DO(7:0) (read mode). This is the data that will be read from the device (MSb first). 

bit 16-... : data DO(...-8). Further data in multiple byte reads. 

Figure 9. Multiple byte SPI read protocol (2-byte example) (in mode 3) 

&6 63& 6', 6'2 5: '2 '2 '2 '2 '2 '2 '2 '2 $' $' $' $' $' $' $' 

&6 63& 6', 6'2 5: '2 '2 '2 '2 '2 '2 '2 '2 $' $' $' $' $' $' '2'2 '2 '2'2'2'2 '2 $' DocID032891 Rev 1 39/153 

LSM6DSO32 Digital interfaces 

> 153

SPI write Figure 10. SPI write protocol (in mode 3) 

The SPI Write command is performed with 16 clock pulses. A multiple byte write command is performed by adding blocks of 8 clock pulses to the previous one. 

bit 0 : WRITE bit. The value is 0. 

bit 1 -7 : address AD(6:0). This is the address field of the indexed register. 

bit 8-15 : data DI(7:0) (write mode). This is the data that is written inside the device (MSb first). 

bit 16-... : data DI(...-8). Further data in multiple byte writes. 

Figure 11. Multiple byte SPI write protocol (2-byte example) (in mode 3) 

&6 63& 6', 5: ', ', ', ', ', ', ', ', $' $' $' $' $' $' $' 

&6 63& 6', 5: $' $' $' $' $' $' ', ', ', ', ', ', ', ', ',',',',',',', ', $' Digital interfaces LSM6DSO32 

40/153 DocID032891 Rev 1 

SPI read in 3-wire mode 

A 3-wire mode is entered by setting the  CTRL3_C (12h)  (SIM) bit equal to ‘1’ (SPI serial 

interface mode selection). 

Figure 12. SPI read protocol in 3-wire mode (in mode 3) 

The SPI read command is performed with 16 clock pulses: 

bit 0 : READ bit. The value is 1. 

bit 1-7 : address AD(6:0). This is the address field of the indexed register. 

bit 8-15 : data DO(7:0) (read mode). This is the data that is read from the device (MSb first). 

A multiple read command is also available in 3-wire mode. 

&6 63& 6',2 5: '2 '2 '2 '2 '2 '2 '2 '2 $' $' $' $' $' $' $' DocID032891 Rev 1 41/153 

LSM6DSO32 Digital interfaces 

> 153

# 5.2 MIPI I3C SM interface 5.2.1 MIPI I3C SM slave interface 

The LSM6DSO32 interface includes a MIPI I3C SM  SDR only slave interface (compliant with release 1.0 of the specification) with MIPI I3C SM  SDR embedded features: 

 CCC command 

 Direct CCC communication (SET and GET) 

 Broadcast CCC communication 

 Private communications 

 Private read and write for single byte 

 Multiple read and write 

 In-Band Interrupt request 

Error Detection and Recovery Methods (S0-S6) 

Note: Refer to  Section  5.3: I²C/MIPI I3C SM  coexistence in LSM6DSO32  for details concerning the 

choice of the interface when powering up the device. 

## 5.2.2 MIPI I3C SM CCC supported commands 

The list of MIPI I3C SM  CCC commands supported by the device is detailed in the following table. 

Table 16. MIPI I3C SM CCC commands Command Command code Default Description 

ENTDAA 0x07 DAA procedure SETDASA 0x87 Assign Dynamic Address using Static Address 0x6B/0x6A depending on SDO pin ENEC 0x80 / 0x00 Slave activity control (direct and broadcast) DISEC 0x81/ 0x01 Slave activity control (direct and broadcast) ENTAS0 0x82 / 0x02 Enter activity state (direct and broadcast) ENTAS1 0x83 / 0x03 Enter activity state (direct and broadcast) ENTAS2 0x84 / 0x04 Enter activity state (direct and broadcast) ENTAS3 0x85 / 0x05 Enter activity state (direct and broadcast) SETXTIME 0x98 / 0x28 Timing information exchange GETXTIME 0x99 0x07 0x00 0x05 0x92 Timing information exchange RSTDAA 0x86 / 0x06 Reset the assigned dynamic address (direct and broadcast) SETMWL 0x89 / 0x08 Define maximum write length during private write (direct and broadcast) Digital interfaces LSM6DSO32 

42/153 DocID032891 Rev 1 

SETMRL 0x8A / 0x09 Define maximum read length during private read (direct and broadcast) SETNEWDA 0x88 Change dynamic address GETMWL 0x8B 0x00 0x08 (2 byte) Get maximum write length during private write GETMRL 0x8C 0x00 0x10 0x09 (3 byte) Get maximum read length during private read GETPID 0x8D 0x02 0x08 0x00 0x6C 0x10 0x0B Device ID register GETBCR 0x8E 0x07 (1 byte) Bus characteristics register GETDCR 0x8F 0x44 default MIPI I3C SM Device Characteristic Register GETSTATUS 0x90 0x00 0x00 (2 byte) Status register GETMXDS 0x94 0x00 0x20 (2 byte) Return max data speed 

Table 16. MIPI I3C SM CCC commands Command Command code Default Description DocID032891 Rev 1 43/153 

LSM6DSO32 Digital interfaces 

> 153

# 5.3 I²C/MIPI I3C SM coexistence in LSM6DSO32 

In the LSM6DSO32, the SDA and SCL lines are common to both I²C and MIPI I3C SM . The I²C bus requires anti-spike filters on the SDA and SCL pins that are not compatible with MIPI I3C SM  timing. 

The device can be connected to both I²C and MIPI I3C SM  or only to the MIPI I3C SM  bus depending on the connection of the INT1 pin when the device is powered up: 

 INT1 pin floating (internal pull-down): I²C/MIPI I3C SM both active, see Figure 13 

 INT1 pin connected to VDD_IO: only MIPI I3C SM active, see Figure 14 

Figure 13. I²C and MIPI I3C SM both active (INT1 pin not connected)   

> 1. Address assignment (DAA or ENTDA) must be performed with I²C Fast Mode Plus Timing. When the slave is addressed, the I²C slave is disabled and the timing is compatible with MIPI I3C SM specifications.

Figure 14. Only MIPI I3C SM active (INT1 pin connected to VDD_IO)     

> 1. When the slave is MIPI I3C SM only, the I²C slave is always disabled. The address can be assigned using MIPI I3C SM SDR timing.

I²C/I3C (both active) Master sends I²C r/w I²C bus case Slave performs requested r/w Master assigns DA to the slave (1) I3C bus case I3C private R/W with and without 7Eh CCC commands Slave event management Error detection and recovery Master resets DA INT1 pin not connected I²C/I3C both active 

I²C/I3C I3C bus case Dynamic Address Assignment Master resets DA I3C private R/W with and without 7Eh CCC commands Slave event management Error detection and recovery (1) INT1 pin connected to VDD_IO Only I3C active Digital interfaces LSM6DSO32 

44/153 DocID032891 Rev 1 

# 5.4 Master I ²C interface 

If the LSM6DSO32 is configured in Mode 2, a master I²C line is available. The master serial interface is mapped in the following dedicated pins. 

Table 17. Master I ²C pin details Pin name Pin description 

MSCL I²C serial clock master MSDA I²C serial data master MDRDY I²C master external synchronization signal DocID032891 Rev 1 45/153 

LSM6DSO32 Functionality 

> 153

# 6 Functionality 6.1 Operating modes 

In the LSM6DSO32, the accelerometer and the gyroscope can be turned on/off independently of each other and are allowed to have different ODRs and power modes. 

The LSM6DSO32 has three operating modes available: 

 only accelerometer active and gyroscope in power-down 

 only gyroscope active and accelerometer in power-down 

 both accelerometer and gyroscope sensors active with independent ODR 

The accelerometer is activated from power-down by writing ODR_XL[3:0] in  CTRL1_XL 

(10h)  while the gyroscope is activated from power-down by writing ODR_G[3:0] in 

CTRL2_G (11h) . For combo-mode the ODRs are totally independent. 

# 6.2 Accelerometer power modes 

In the LSM6DSO32, the accelerometer can be configured in five different operating modes: power-down, ultra-low-power, low-power, normal mode and high-performance mode. The operating mode selected depends on the value of the XL_HM_MODE bit in  CTRL6_C (15h) .

If XL_HM_MODE is set to '0', high-performance mode is valid for all ODRs (from 12.5 Hz up to 6.66 kHz). 

To enable the low-power and normal mode, the XL_HM_MODE bit has to be set to '1'. Low-power mode is available for lower ODRs (1.6, 12.5, 26, 52 Hz) while normal mode is available for ODRs equal to 104 and 208 Hz. 

## 6.2.1 Accelerometer ultra-low-power mode 

The LSM6DSO32 can be configured in ultra-low-power (ULP) mode by setting the XL_ULP_EN bit to 1 in  CTRL5_C (14h)  register. This mode can be used in accelerometer-

only mode (gyroscope sensor must be configured in power-down mode) and for ODR_XL values between 1.6 Hz and 208 Hz. 

When ULP mode is intended to be used, the bit XL_HM_MODE must be set to 0. 

When ULP mode is switched ON/OFF, the accelerometer must be configured in power-down condition. 

The embedded functions based on accelerometer data (free-fall, 6D/4D, tap, double-tap, wake-up, activity/inactivity, stationary/motion, step-counter, step-detection, significant motion, tilt) and the FIFO batching functionality are still supported when ULP mode is enabled. Functionality LSM6DSO32 

46/153 DocID032891 Rev 1 

# 6.3 Gyroscope power modes 

In the LSM6DSO32, the gyroscope can be configured in four different operating modes: power-down, low-power, normal mode and high-performance mode. The operating mode selected depends on the value of the G_HM_MODE bit in  CTRL7_G (16h) . If G_HM_MODE 

is set to '0', high-performance mode is valid for all ODRs (from 12.5 Hz up to 6.66 kHz). 

To enable the low-power and normal mode, the G_HM_MODE bit has to be set to '1'. Low-power mode is available for lower ODRs (12.5, 26, 52 Hz) while normal mode is available for ODRs equal to 104 and 208 Hz. 

# 6.4 Block diagram of filters Figure 15. Block diagram of filters 6.4.1 Block diagrams of the accelerometer filters 

In the LSM6DSO32, the filtering chain for the accelerometer part is composed of the following: 

 Analog filter (anti-aliasing) 

 Digital filter (LPF1) 

 Composite filter 

Details of the block diagram appear in the following figure. 

Figure 16. Accelerometer chain 

$'& $'& /RZ3DVV *\UR 5HJV DUUD\ ),)2 LQWHUIDFH ,QWHUUXSW PQJ 9ROWDJHDQGFXUUHQW UHIHUHQFHV 7ULPPLQJFLUFXLW DQG7HVWLQWHUIDFH &ORFNDQGSKDVH JHQHUDWRU 3RZHU PDQDJHPHQW )73 &6 6&/63& 6'$6',6'2 6'26$ ,17 ,17 0(066(1625/RZ3DVV ;/ *\UR IURQWHQG ;/ IURQWHQG 7HPSHUDWXUH VHQVRU , & , &63, 

## $'& 'LJLWDO /3)LOWHU $QDORJ $QWLDOLDVLQJ /3)LOWHU 2'5B;/>@ &RPSRVLWH )LOWHU /3) DocID032891 Rev 1 47/153 

LSM6DSO32 Functionality 

> 153

Figure 17. Accelerometer composite filter  

> 1. The cutoff value of the LPF1 output is ODR/2 when the accelerometer is in high-performance mode. This value is equal to 700 Hz when the accelerometer is in low-power or normal mode.

Note: Advanced functions include pedometer, step detector and step counter, significant motion detection, tilt functions and Finite State Machine. 

6/23( ),/7(5 +3&)B;/>@    « 63, , &  +3B6/23(B;/B(1 /3)B;/B(1 'LJLWDO +3)LOWHU +3&)B;/>@ 'LJLWDO /3)LOWHU /3) +3&)B;/>@ 6'7DS '' /2:B3$66B21B' 6/23(B)'6 :DNHXS $FWLYLW\ ,QDFWLYLW\ )UHHIDOO $GYDQFHG IXQFWLRQV ),)2 86(5 2))6(7 865B2))B21B287 865B2))B: 2)6B865>@ 865B2))B21B:8 /3) RXWSXW 0,3,,& 60 Functionality LSM6DSO32 

48/153 DocID032891 Rev 1 

## 6.4.2 Block diagrams of the gyroscope filters Figure 18. Gyroscope digital chain - Mode 1 and Mode 2 

The digital LPF2 filter cannot be configured by the user and its cutoff frequency depends on the selected gyroscope ODR, as indicated in the following table. 

Data can be acquired from the output registers and FIFO over the primary I²C/I³C/SPI interface. 

Table 18. Gyroscope LPF2 bandwidth selection Gyroscope ODR [Hz] LPF2 cutoff [Hz] 

12.5 4.2 26 8.3 52 16.6 104 33.0 208 66.8 417 135.9 833 295.5 1667 1108.1 3333 1320.7 6667 1441.8 

$'& +3) +3B(1B* /3) /3) /3)B6(/B* 63, ,  & ),)2 )7<3( >@ 2'5B*>@ 0,3,,& 60  )60 DocID032891 Rev 1 49/153 

LSM6DSO32 Functionality 

> 153

# 6.5 FIFO 

The presence of a FIFO allows consistent power saving for the system since the host processor does not need continuously poll data from the sensor, but It can wake up only when needed and burst the significant data out from the FIFO. 

The LSM6DSO32 embeds 3 kbytes of data in FIFO (up to 9 kbytes with the compression feature enabled) to store the following data: 

 Gyroscope 

 Accelerometer 

 External sensors (up to 4) 

 Step counter 

 Timestamp 

 Temperature 

Writing data in the FIFO can be configured to be triggered by the: 

 Accelerometer / gyroscope data-ready signal 

 Sensor hub data-ready signal 

 Step detection signal 

The applications have maximum flexibility in choosing the rate of batching for physical sensors with FIFO-dedicated configurations: accelerometer, gyroscope and temperature sensor batching rates can be selected by the user. External sensor writing in FIFO can be triggered by the accelerometer data-ready signal or by an external sensor interrupt. The step counter can be stored in FIFO with associated timestamp each time a step is detected. It is possible to select decimation for timestamp batching in FIFO with a factor of 1, 8, or 32. 

The reconstruction of a FIFO stream is a simple task thanks to the FIFO_DATA_OUT_TAG byte that allows recognizing the meaning of a word in FIFO. 

FIFO allows correct reconstruction of the timestamp information for each sensor stored in FIFO. If a change in the ODR or BDR (Batching Data Rate) configuration is performed, the application can correctly reconstruct the timestamp and know exactly when the change was applied without disabling FIFO batching. FIFO stores information of the new configuration and timestamp in which the change was applied in the device. 

Finally, FIFO embeds a compression algorithm that the user can enable in order to have up to 9 kbyte data stored in FIFO and take advantage of interface communication length for FIFO flushing and communication power consumption. 

The programmable FIFO watermark threshold can be set in  FIFO_CTRL1 (07h)  and 

FIFO_CTRL2 (08h)  using the WTM[8:0] bits. To monitor the FIFO status, dedicated 

registers ( FIFO_STATUS1 (3Ah) , FIFO_STATUS2 (3Bh) ) can be read to detect FIFO 

overrun events, FIFO full status, FIFO empty status, FIFO watermark status and the number of unread samples stored in the FIFO. To generate dedicated interrupts on the INT1 and INT2 pins of these status events, the configuration can be set in  INT1_CTRL (0Dh)  and 

INT2_CTRL (0Eh) .Functionality LSM6DSO32  

> 50/153 DocID032891 Rev 1

The FIFO buffer can be configured according to six different modes: 

 Bypass mode 

 FIFO mode 

 Continuous mode 

 Continuous-to-FIFO mode 

 Bypass-to-continuous mode 

 Bypass-to-FIFO mode 

Each mode is selected by the FIFO_MODE_[2:0] bits in the  FIFO_CTRL4 (0Ah)  register. 

## 6.5.1 Bypass mode 

In Bypass mode ( FIFO_CTRL4 (0Ah) (FIFO_MODE_[2:0] = 000), the FIFO is not operational 

and it remains empty. Bypass mode is also used to reset the FIFO when in FIFO mode. 

## 6.5.2 FIFO mode 

In FIFO mode ( FIFO_CTRL4 (0Ah) (FIFO_MODE_[2:0] = 001) data from the output 

channels are stored in the FIFO until it is full. 

To reset FIFO content, Bypass mode should be selected by writing  FIFO_CTRL4 

(0Ah) (FIFO_MODE_[2:0]) to '000'. After this reset command, it is possible to restart FIFO 

mode by writing  FIFO_CTRL4 (0Ah) (FIFO_MODE_[2:0]) to '001'. 

The FIFO buffer memorizes up to 9 kbytes of data (with compression enabled) but the depth of the FIFO can be resized by setting the WTM [8:0] bits in  FIFO_CTRL1 (07h)  and 

FIFO_CTRL2 (08h) . If the STOP_ON_WTM bit in  FIFO_CTRL2 (08h)  is set to '1', FIFO 

depth is limited up to the WTM [8:0] bits in  FIFO_CTRL1 (07h)  and  FIFO_CTRL2 (08h) .

## 6.5.3 Continuous mode 

Continuous mode ( FIFO_CTRL4 (0Ah) (FIFO_MODE_[2:0] = 110) provides a continuous 

FIFO update: as new data arrives, the older data is discarded. 

A FIFO threshold flag  FIFO_STATUS2 (3Bh) (FIFO_WTM_IA) is asserted when the number 

of unread samples in FIFO is greater than or equal to  FIFO_CTRL1 (07h)  and  FIFO_CTRL2 

(08h) (WTM [8:0]). 

It is possible to route the FIFO_WTM_IA flag to the INT1 pin by writing in register 

INT1_CTRL (0Dh) (INT1_FIFO_TH) = '1' or to the INT2 pin by writing in register  INT2_CTRL 

(0Eh) (INT2_FIFO_TH) = '1'. 

A full-flag interrupt can be enabled,  INT1_CTRL (0Dh) (INT1_FIFO_FULL) = '1' or 

INT2_CTRL (0Eh) (INT2_FIFO_FULL) = '1', in order to indicate FIFO saturation and 

eventually read its content all at once. 

If an overrun occurs, at least one of the oldest samples in FIFO has been overwritten and the FIFO_OVR_IA flag in  FIFO_STATUS2 (3Bh)  is asserted. 

In order to empty the FIFO before it is full, it is also possible to pull from FIFO the number of unread samples available in FIFO_STATUS1 (3Ah)  and  FIFO_STATUS2 

(3Bh) (DIFF_FIFO_[9:0]). DocID032891 Rev 1 51/153 

LSM6DSO32 Functionality 

> 153

## 6.5.4 Continuous-to-FIFO mode 

In Continuous-to-FIFO mode ( FIFO_CTRL4 (0Ah) (FIFO_MODE_[2:0] = 011), FIFO 

behavior changes according to the trigger event detected in one of the following interrupt events: 

 Single tap 

 Double tap 

 Wake-up 

 Free-fall 

 D6D 

When the selected trigger bit is equal to '1', FIFO operates in FIFO mode. 

When the selected trigger bit is equal to '0', FIFO operates in Continuous mode. 

## 6.5.5 Bypass-to-Continuous mode 

In Bypass-to-Continuous mode ( FIFO_CTRL4 (0Ah) (FIFO_MODE_[2:0] = '100'), data 

measurement storage inside FIFO operates in Continuous mode when selected triggers are equal to '1', otherwise FIFO content is reset (Bypass mode). 

FIFO behavior changes according to the trigger event detected in one of the following interrupt events: 

 Single tap 

 Double tap 

 Wake-up 

 Free-fall 

 D6D 

## 6.5.6 Bypass-to-FIFO mode 

In Bypass-to-FIFO mode ( FIFO_CTRL4 (0Ah) (FIFO_MODE_[2:0] = '111'), data 

measurement storage inside FIFO operates in FIFO mode when selected triggers are equal to '1', otherwise FIFO content is reset (Bypass mode). 

FIFO behavior changes according to the trigger event detected in one of the following interrupt events: 

 Single tap 

 Double tap 

 Wake-up 

 Free-fall 

 D6D Functionality LSM6DSO32  

> 52/153 DocID032891 Rev 1

## 6.5.7 FIFO reading procedure 

The data stored in FIFO are accessible from dedicated registers and each FIFO word is composed of 7 bytes: one tag byte ( FIFO_DATA_OUT_TAG (78h) , in order to identify the 

sensor, and 6 bytes of fixed data (FIFO_DATA_OUT registers from (79h) to (7Eh)). 

The DIFF_FIFO_[9:0] field in the  FIFO_STATUS1 (3Ah)  and  FIFO_STATUS2 (3Bh) 

registers contains the number of words (1 byte TAG + 6 bytes DATA) collected in FIFO. 

In addition, it is possible to configure a counter of the batch events of accelerometer or gyroscope sensors. The flag COUNTER_BDR_IA in  FIFO_STATUS2 (3Bh)  alerts that the 

counter reaches a selectable threshold (CNT_BDR_TH_[10:0] field in 

COUNTER_BDR_REG1 (0Bh)  and  COUNTER_BDR_REG2 (0Ch) ). This allows triggering 

the reading of FIFO with the desired latency of one single sensor. The sensor is selectable using the TRIG_COUNTER_BDR bit in  COUNTER_BDR_REG1 (0Bh) . As for the other 

FIFO status events, the flag COUNTER_BDR_IA can be routed on the INT1 or INT2 pins by asserting the corresponding bits (INT1_CNT_BDR of  INT1_CTRL (0Dh)  and 

INT2_CNT_BDR of  INT2_CTRL (0Eh) ). 

In order to maximize the amount of accelerometer and gyroscope data in FIFO, the user can enable the compression algorithm by setting to 1 both the FIFO_COMPR_EN bit in 

EMB_FUNC_EN_B (05h)  (embedded functions registers bank) and the 

FIFO_COMPR_RT_EN bit in  FIFO_CTRL2 (08h) . When compression is enabled, it is also 

possible to force writing non-compressed data at a selectable rate using the UNCOPTR_RATE_[1:0] field in  FIFO_CTRL2 (08h) .

Meta information about accelerometer and gyroscope sensor configuration changes can be managed by enabling the ODR_CHG_EN bit in  FIFO_CTRL2 (08h) .DocID032891 Rev 1 53/153 

LSM6DSO32 Application hints 

> 153

# 7 Application hints 7.1 LSM6DSO32 electrical connections in Mode 1 Figure 19. LSM6DSO32 electrical connections in Mode 1 

1. Leave pin electrically unconnected and soldered to PCB. 

The device core is supplied through the Vdd line. Power supply decoupling capacitors (C1, C2 = 100 nF ceramic) should be placed as near as possible to the supply pin of the device (common design practice). 

The functionality of the device and the measured acceleration/angular rate data is selectable and accessible through the SPI/I²C/MIPI I3C SM  interface. 

The functions, the threshold and the timing of the two interrupt pins for each sensor can be completely programmed by the user through the SPI/I²C/MIPI I3C SM  interface. 

> *1'RU9'',2 9GGB,2 *1' Q)

& 6'26$ 6'[ 6&[ ,17 *1' 9'',2 &6 6&/ 6'$    723 9,(: 9'' ,17 1& 1&  9GG *1' Q) & *1' 6&/ 6'$ 9GGB,2 5SX 5SX 3XOOXSWREHDGGHG 5SX N2KP , &FRQILJXUDWLRQ +267 /60'62 ,  &63, Z 0RGH 0,3,,& 60  Application hints LSM6DSO32 

54/153 DocID032891 Rev 1 

# 7.2 LSM6DSO32 electrical connections in Mode 2 Figure 20. LSM6DSO32 electrical connections in Mode 2 

1. Leave pin electrically unconnected and soldered to PCB. 

The device core is supplied through the Vdd line. Power supply decoupling capacitors (C1, C2 = 100 nF ceramic) should be placed as near as possible to the supply pin of the device (common design practice). 

The functionality of the device and the measured acceleration/angular rate data is selectable and accessible through the SPI/I²C/MIPI I3C SM  primary interface. 

The functions, the threshold and the timing of the two interrupt pins for each sensor can be completely programmed by the user through the SPI/I²C/MIPI I3C SM  primary interface. 

> 9GGB,2 *1' Q)

& 6'26$ 06'$ 06&/ ,17 *1' 9'',2 &6 6&/ 6'$    723 9,(: 9'' 1& 1&  9GG *1' Q) & 0'5'<,17 *1' +267 /60'62 /60'60 /60'60 ([WHUQDO VHQVRUV 0DVWHU,  &0RGH 6&/ 6'$ 9GGB,2 5SX 5SX 3XOOXSWREHDGGHG 5SX N2KP , &FRQILJXUDWLRQ ,  &63, Z 0,3,,& 60  DocID032891 Rev 1 55/153 

LSM6DSO32 Application hints 

> 153

Table 19. Internal pin status pin# Name Mode 1 function Mode 2 function Pin status Mode 1 Pin status Mode 2 

1SDO SPI 4-wire interface serial data output (SDO) SPI 4-wire interface serial data output (SDO) Default: input without pull-up. Pull-up is enabled if bit SDO_PU_EN = 1 in reg. 02h. Default: input without pull-up. Pull-up is enabled if bit SDO_PU_EN = 1 in reg. 02h. SA0 I²C least significant bit of the device address (SA0) MIPI I3C SM least significant bit of the static address (SA0) I²C least significant bit of the device address (SA0) MIPI I3C SM least significant bit of the static address (SA0) 2 SDx Connect to VDDIO or GND I²C serial data master (MSDA) Default: input without pull-up. Pull-up is enabled if bit SHUB_PU_EN = 1 in reg 14h in sensor hub registers (see Note to enable pull-up). Default: input without pull-up. Pull-up is enabled if bit SHUB_PU_EN = 1 in reg 14h in sensor hub registers (see Note to enable pull-up). 3 SCx Connect to VDDIO or GND I²C serial clock master (MSCL) Default: input without pull-up. Pull-up is enabled if bit SHUB_PU_EN = 1 in reg 14h in sensor hub registers (see Note to enable pull-up). Default: input without pull-up. Pull-up is enabled if bit SHUB_PU_EN = 1 in reg 14h in sensor hub registers (see Note to enable pull-up). 4 INT1 Programmable interrupt 1 / If device is used as MIPI I3C SM pure slave, this pin must be set to ‘1’. Programmable interrupt 1 / If device is used as MIPI I3C SM pure slave, this pin must be set to ‘1’. Default: input with pull-down (1) Default: input with pull-down (1) 5 VDDIO Power supply for I/O pins Power supply for I/O pins 6 GND 0 V supply 0 V supply 7 GND 0 V supply 0 V supply 8 VDD Power supply Power supply 9 INT2 Programmable interrupt 2 (INT2) / Data enabled (DEN) Programmable interrupt 2 (INT2) / Data enabled (DEN) / I²C master external synchronization signal (MDRDY) Default: output forced to ground Default: output forced to ground 10 NC Leave unconnected Leave unconnected Default: input with pull-up. Default: input with pull-up. 11 NC Connect to VDDIO or leave unconnected Connect to VDDIO or leave unconnected Default: input with pull-up. Default: input with pull-up Application hints LSM6DSO32 

56/153 DocID032891 Rev 1 

Internal pull-up value is from 30 kΩ to 50 kΩ, depending on VDDIO. 

Note: The procedure to enable the pull-up on pins 2 and 3 is as follows: 

1. Write 40h in register at address 01h (enable access to the sensor hub registers) 

2. Write 08h in register at address 14h (enable the pull-up on pins 2 and 3) 

3. Write 00h in register at address 01h (disable access to the sensor hub registers) 

12 CS I²C/SPI mode selection (1:SPI idle mode / I²C communication enabled; 0: SPI communication mode / I²C disabled) I²C/SPI mode selection (1:SPI idle mode / I²C communication enabled; 0: SPI communication mode / I²C disabled) Default: input with pull-up. Pull-up is disabled if bit I2C_disable = 1 in reg 13h and I3C_disable = 1 in reg 18h. Default: input with pull-up. Pull-up is disabled if bit I2C_disable = 1 in reg 13h and I3C_disable = 1 in reg 18h. 13 SCL I²C/MIPI I3C SM serial clock (SCL) / SPI serial port clock (SPC) I²C/MIPI I3C SM serial clock (SCL) / SPI serial port clock (SPC) Default: input without pull-up Default: input without pull-up 14 SDA I²C/MIPI I3C SM serial data (SDA) / SPI serial data input (SDI) / 3-wire interface serial data output (SDO) I²C/MIPI I3C SM serial data (SDA) / SPI serial data input (SDI) / 3-wire interface serial data output (SDO) Default: input without pull-up Default: input without pull-up 1. INT1 must be set to '0' or left unconnected during power-on if the I²C/SPI interfaces are used. 

Table 19. Internal pin status (continued) pin# Name Mode 1 function Mode 2 function Pin status Mode 1 Pin status Mode 2 DocID032891 Rev 1 57/153 

LSM6DSO32 Register mapping 

> 153

# 8 Register mapping 

The table given below provides a list of the 8/16-bit registers embedded in the device and the corresponding addresses. 

Table 20. Registers address map Name Type Register address Default Comment Hex Binary 

FUNC_CFG_ACCESS RW 01 00000001 00000000 PIN_CTRL RW 02 00000010 00111111 RESERVED - 03-06 FIFO_CTRL1 RW 07 00000111 00000000 FIFO_CTRL2 RW 08 00001000 00000000 FIFO_CTRL3 RW 09 00001001 00000000 FIFO_CTRL4 RW 0A 00001010 00000000 COUNTER_BDR_REG1 RW 0B 00001011 00000000 COUNTER_BDR_REG2 RW 0C 00001100 00000000 INT1_CTRL RW 0D 00001101 00000000 INT2_CTRL RW 0E 00001110 00000000 WHO_AM_I R 0F 00001111 01101100 CTRL1_XL RW 10 00010000 00000000 CTRL2_G RW 11 00010001 00000000 CTRL3_C RW 12 00010010 00000100 CTRL4_C RW 13 00010011 00000000 CTRL5_C RW 14 00010100 00000000 CTRL6_C RW 15 00010101 00000000 CTRL7_G RW 16 00010110 00000000 CTRL8_XL RW 17 0001 0111 00000000 CTRL9_XL RW 18 00011000 11100000 CTRL10_C RW 19 00011001 00000000 ALL_INT_SRC R 1A 00011010 output WAKE_UP_SRC R 1B 00011011 output TAP_SRC R 1C 00011100 output D6D_SRC R 1D 00011101 output STATUS_REG R 1E 00011110 output RESERVED - 1F 00011111 OUT_TEMP_L R 20 00100000 output Register mapping LSM6DSO32 

58/153 DocID032891 Rev 1 

OUT_TEMP_H R 21 00100001 output OUTX_L_G R 22 00100010 output OUTX_H_G R 23 00100011 output OUTY_L_G R 24 00100100 output OUTY_H_G R 25 00100101 output OUTZ_L_G R 26 00100110 output OUTZ_H_G R 27 00100111 output OUTX_L_A R 28 00101000 output OUTX_H_A R 29 00101001 output OUTY_L_A R 2A 00101010 output OUTY_H_A R 2B 00101011 output OUTZ_L_A R 2C 00101100 output OUTZ_H_A R 2D 00101101 output RESERVED - 2E-34 EMB_FUNC_STATUS_ MAINPAGE R 35 00110101 output FSM_STATUS_A_ MAINPAGE R 36 00110110 output FSM_STATUS_B_ MAINPAGE R 37 00110111 output RESERVED - 38 STATUS_MASTER_ MAINPAGE R 39 00111001 output FIFO_STATUS1 R 3A 00111010 output FIFO_STATUS2 R 3B 00111011 output RESERVED - 3C-3F TIMESTAMP0 R 40 01000000 output TIMESTAMP1 R 41 01000001 output TIMESTAMP2 R 42 01000010 output TIMESTAMP3 R 43 01000011 output RESERVED - 44-55 TAP_CFG0 RW 56 01010110 00000000 TAP_CFG1 RW 57 01010111 00000000 TAP_CFG2 RW 58 01011000 00000000 TAP_THS_6D RW 59 01011001 00000000 

Table 20. Registers address map (continued) Name Type Register address Default Comment Hex Binary DocID032891 Rev 1 59/153 

LSM6DSO32 Register mapping 

> 153

INT_DUR2 RW 5A 01011010 00000000 WAKE_UP_THS RW 5B 01011011 00000000 WAKE_UP_DUR RW 5C 01011100 00000000 FREE_FALL RW 5D 01011101 00000000 MD1_CFG RW 5E 01011110 00000000 MD2_CFG RW 5F 01011111 00000000 RESERVED - 60-61 00000000 I3C_BUS_AVB RW 62 01100010 00000000 INTERNAL_FREQ_FINE R 63 01100011 output RESERVED - 64-72 X_OFS_USR RW 73 01110011 00000000 Y_OFS_USR RW 74 01110100 00000000 Z_OFS_USR RW 75 01110101 00000000 RESERVED - 76-77 FIFO_DATA_OUT_TAG R 78 01111000 output FIFO_DATA_OUT_X_L R 79 01111001 output FIFO_DATA_OUT_X_H R 7A 01111010 output FIFO_DATA_OUT_Y_L R 7B 01111011 output FIFO_DATA_OUT_Y_H R 7C 01111100 output FIFO_DATA_OUT_Z_L R 7D 01111101 output FIFO_DATA_OUT_Z_H R 7E 01111110 output 

Table 20. Registers address map (continued) Name Type Register address Default Comment Hex Binary Register description LSM6DSO32 

60/153 DocID032891 Rev 1 

# 9 Register description 

The device contains a set of registers which are used to control its behavior and to retrieve linear acceleration, angular rate and temperature data. The register addresses, made up of 7 bits, are used to identify them and to write the data through the serial interface. 

# 9.1 FUNC_CFG_ACCESS (01h) 

Enable embedded functions register (r/w) 

Table 22. FUNC_CFG_ACCESS register description 9.2 PIN_CTRL (02h) 

SDO pin pull-up enable/disable register (r/w) 

Table 24. PIN_CTRL register description Table 21. FUNC_CFG_ACCESS register 

FUNC_CFG_ ACCESS SHUB_ REG_ ACCESS 0 (1) 1. This bit must be set to '0' for the correct operation of the device. 0(1) 0 (1) 0(1) 0(1) 0 (1) FUNC_CFG_ ACCESS Enable access to the embedded functions configuration registers. Default value: 0 (1) 1. Details concerning the embedded functions configuration registers are available in Section 10: Embedded functions register mapping and Section 11: Embedded functions register description .SHUB_REG_ ACCESS Enable access to the sensor hub (I²C master) registers. Default value: 0 (2) 2. Details concerning the sensor hub registers are available in Section 14: Sensor hub register mapping and 

Section 15: Sensor hub register description .

Table 23. PIN_CTRL register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. SDO_ PU_EN 1(2) 2. This bit must be set to '1' for the correct operation of the device. 1(2) 1(2) 1 (2) 1 (2) 1 (2) SDO_PU_EN Enable pull-up on SDO pin (0: SDO pin pull-up disconnected (default); 1: SDO pin with pull-up) DocID032891 Rev 1 61/153 

LSM6DSO32 Register description 

> 153

# 9.3 FIFO_CTRL1 (07h) 

FIFO control register 1 (r/w) 

Table 26. FIFO_CTRL1 register description 9.4 FIFO_CTRL2 (08h) 

FIFO control register 2 (r/w) 

Table 28. FIFO_CTRL2 register description Table 25. FIFO_CTRL1 register 

WTM7 WTM6 WTM5 WTM4 WTM3 WTM2 WTM1 WTM0 WTM[7:0] FIFO watermark threshold, in conjunction with WTM8 in FIFO_CTRL2 (08h) 

1 LSB = 1 sensor (6 bytes) + TAG (1 byte) written in FIFO Watermark flag rises when the number of bytes written in the FIFO is greater than or equal to the threshold level. 

Table 27. FIFO_CTRL2 register 

STOP_ON _WTM FIFO_ COMPR_ RT_EN 0(1) 1. This bit must be set to '0' for the correct operation of the device. ODRCHG _EN 0(1) UNCOPTR _RATE_1 UNCOPTR _RATE_0 WTM8 STOP_ON_ WTM Sensing chain FIFO stop values memorization at threshold level (0: FIFO depth is not limited (default); 1: FIFO depth is limited to threshold level, defined in FIFO_CTRL1 (07h) and 

FIFO_CTRL2 (08h) )FIFO_COMPR_ RT_EN (1) 1. This bit is effective if the FIFO_COMPR_EN bit of EMB_FUNC_EN_B (05h) is set to 1. Enables/Disables compression algorithm runtime ODRCHG_EN Enables ODR CHANGE virtual sensor to be batched in FIFO UNCOPTR_ RATE_[1:0] This field configures the compression algorithm to write non-compressed data at each rate. (0: Non-compressed data writing is not forced; 1: Non-compressed data every 8 batch data rate; 2: Non-compressed data every 16 batch data rate; 3: Non-compressed data every 32 batch data rate) WTM8 FIFO watermark threshold, in conjunction with WTM_FIFO[7:0] in FIFO_CTRL1 (07h) 

1 LSB = 1 sensor (6 bytes) + TAG (1 byte) written in FIFO Watermark flag rises when the number of bytes written in the FIFO is greater than or equal to the threshold level. Register description LSM6DSO32 

62/153 DocID032891 Rev 1 

# 9.5 FIFO_CTRL3 (09h) 

FIFO control register 3 (r/w) 

Table 30. FIFO_CTRL3 register description Table 29. FIFO_CTRL3 register 

BDR_ GY_3 BDR_ GY_2 BDR_ GY_1 BDR_ GY_0 BDR_ XL_3 BDR_ XL_2 BDR_ XL_1 BDR_ XL_0 BDR_GY_[3:0] Selects Batching Data Rate (writing frequency in FIFO) for gyroscope data. (0000: Gyro not batched in FIFO (default); 0001: 12.5 Hz; 0010: 26 Hz; 0011: 52 Hz; 0100: 104 Hz; 0101: 208 Hz; 0110: 417 Hz; 0111: 833 Hz; 1000: 1667 Hz; 1001: 3333 Hz; 1010: 6667 Hz; 1011: 6.5 Hz; 1100-1111: not allowed) BDR_XL_[3:0] Selects Batching Data Rate (writing frequency in FIFO) for accelerometer data. (0000: Accelerometer not batched in FIFO (default); 0001: 12.5 Hz; 0010: 26 Hz; 0011: 52 Hz; 0100: 104 Hz; 0101: 208 Hz; 0110: 417 Hz; 0111: 833 Hz; 1000: 1667 Hz; 1001: 3333 Hz; 1010: 6667 Hz; 1011: 1.6 Hz; 1100-1111: not allowed) DocID032891 Rev 1 63/153 

LSM6DSO32 Register description 

> 153

# 9.6 FIFO_CTRL4 (0Ah) 

FIFO control register 4 (r/w) 

Table 32. FIFO_CTRL4 register description Table 31. FIFO_CTRL4 register 

DEC_TS_ BATCH_1 DEC_TS_ BATCH_0 ODR_T_ BATCH_1 ODR_T_ BATCH_0 0(1) 1. This bit must be set to '0' for the correct operation of the device. FIFO_ MODE2 FIFO_ MODE1 FIFO_ MODE0 DEC_TS_ BATCH_[1:0] Selects decimation for timestamp batching in FIFO. Writing rate will be the maximum rate between XL and GYRO BDR divided by decimation decoder. (00: Timestamp not batched in FIFO (default); 01: Decimation 1: max(BDR_XL[Hz],BDR_GY[Hz]) [Hz]; 10: Decimation 8: max(BDR_XL[Hz],BDR_GY[Hz])/8 [Hz]; 11: Decimation 32: max(BDR_XL[Hz],BDR_GY[Hz])/32 [Hz]) ODR_T_ BATCH_[1:0] Selects batching data rate (writing frequency in FIFO) for temperature data (00: Temperature not batched in FIFO (default); 01: 1.6 Hz; 10: 12.5 Hz; 11: 52 Hz) FIFO_ MODE[2:0] FIFO mode selection (000: Bypass mode: FIFO disabled; 001: FIFO mode: stops collecting data when FIFO is full; 010: Reserved; 011: Continuous-to-FIFO mode: Continuous mode until trigger is deasserted, then FIFO mode; 100: Bypass-to-Continuous mode: Bypass mode until trigger is deasserted, then Continuous mode; 101: Reserved; 110: Continuous mode: if the FIFO is full, the new sample overwrites the older one; 111: Bypass-to-FIFO mode: Bypass mode until trigger is deasserted, then FIFO mode.) Register description LSM6DSO32 

64/153 DocID032891 Rev 1 

# 9.7 COUNTER_BDR_REG1 (0Bh) 

Counter batch data rate register 1 (r/w) 

Table 34. COUNTER_BDR_REG1 register description 9.8 COUNTER_BDR_REG2 (0Ch) 

Counter batch data rate register 2 (r/w) 

Table 36. COUNTER_BDR_REG2 register description Table 33. COUNTER_BDR_REG1 register 

dataready _pulsed 

RST_ COUNTER _BDR 

TRIG_ COUNTER _BDR 0(1) 1. This bit must be set to '0' for the correct operation of the device. 0(1) CNT_BDR _TH_10 CNT_BDR _TH_9 CNT_BDR _TH_8 

dataready_pulsed Enables pulsed data-ready mode (0: Data-ready latched mode (returns to 0 only after an interface reading) (default); 1: Data-ready pulsed mode (the data ready pulses are 75 μs long) 

RST_ COUNTER_BDR 

Resets the internal counter of batching events for a single sensor. This bit is automatically reset to zero if it was set to ‘1’. TRIG_ COUNTER_BDR Selects the trigger for the internal counter of batching events between XL and gyro. (0: XL batching event; 1: GYRO batching event) 

CNT_BDR_TH_ [10:8] 

In conjunction with CNT_BDR_TH_[7:0] in COUNTER_BDR_REG2 (0Ch) ,sets the threshold for the internal counter of batching events. When this counter reaches the threshold, the counter is reset and the COUNTER_BDR_IA flag in FIFO_STATUS2 (3Bh) is set to ‘1’. 

Table 35. COUNTER_BDR_REG2 register 

CNT_BDR _TH_7 CNT_BDR _TH_6 CNT_BDR _TH_5 CNT_BDR _TH_4 CNT_BDR _TH_3 CNT_BDR _TH_2 CNT_BDR _TH_1 CNT_BDR _TH_0 CNT_BDR_ TH_[7:0] In conjunction with CNT_BDR_TH_[10:8] in COUNTER_BDR_REG1 (0Bh) , sets the threshold for the internal counter of batching events. When this counter reaches the threshold, the counter is reset and the COUNTER_BDR_IA flag in 

FIFO_STATUS2 (3Bh) is set to ‘1’. DocID032891 Rev 1 65/153 

LSM6DSO32 Register description 

> 153

# 9.9 INT1_CTRL (0Dh) 

INT1 pin control register (r/w) 

Each bit in this register enables a signal to be carried over INT1 when the MIPI I3C SM 

dynamic address is not assigned (I²C or SPI is used). Some bits can be also used to trigger an IBI (In-Band Interrupt) when the MIPI I3C SM  interface is used. The output of the pin will be the OR combination of the signals selected here and in  MD1_CFG (5Eh) .

Table 38. INT1_CTRL register description Table 37. INT1_CTRL register 

DEN_ DRDY_ flag INT1_CNT _BDR INT1_ FIFO _FULL INT1_ FIFO_ OVR INT1_ FIFO_TH INT1_ BOOT INT1_ DRDY_G INT1_ DRDY_XL DEN_DRDY_flag Sends DEN_DRDY (DEN stamped on Sensor Data flag) to INT1 pin INT1_CNT_ BDR Enables COUNTER_BDR_IA interrupt on INT1 INT1_FIFO_FULL Enables FIFO full flag interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3C SM interface is used. INT1_FIFO_OVR Enables FIFO overrun interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3C SM interface is used. INT1_FIFO_TH Enables FIFO threshold interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3C SM interface is used. INT1_BOOT Enables boot status on INT1 pin INT1_ DRDY_G Enables gyroscope data-ready interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3C SM interface is used. INT1_ DRDY_XL Enables accelerometer data-ready interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3C SM interface is used. Register description LSM6DSO32 

66/153 DocID032891 Rev 1 

# 9.10 INT2_CTRL (0Eh) 

INT2 pin control register (r/w) 

Each bit in this register enables a signal to be carried over INT2 when the MIPI I3C SM 

dynamic address in not assigned (I²C or SPI is used). Some bits can be also used to trigger an IBI when the MIPI I3C SM  interface is used. The output of the pin will be the OR combination of the signals selected here and in  MD2_CFG (5Fh) .

Table 40. INT2_CTRL register description 9.11 WHO_AM_I (0Fh) 

WHO_AM_I register (r). This is a read-only register. Its value is fixed at 6Ch. 

Table 39. INT2_CTRL register 

0(1) 1. This bit must be set to '0' for the correct operation of the device. INT2_CNT _BDR INT2_ FIFO_ FULL INT2_ FIFO_ OVR INT2_ FIFO_TH INT2_DRDY _TEMP INT2_ DRDY_G INT2_ DRDY_XL INT2_CNT_BDR Enables COUNTER_BDR_IA interrupt on INT2 INT2_FIFO_FULL Enables FIFO full flag interrupt on INT2 pin INT2_FIFO_OVR Enables FIFO overrun interrupt on INT2 pin INT_FIFO_TH Enables FIFO threshold interrupt on INT2 pin INT2_DRDY_TEMP Enables temperature sensor data-ready interrupt on INT2 pin. It can be also used to trigger an IBI when the MIPI I3C SM interface is used and INT2_ON_INT1 = ‘1’ in CTRL4_C (13h) .INT2_DRDY_G Gyroscope data-ready interrupt on INT2 pin INT2_DRDY_XL Accelerometer data-ready interrupt on INT2 pin 

Table 41. WhoAmI register 

0 1 1 0 1 1 0 0DocID032891 Rev 1 67/153 

LSM6DSO32 Register description 

> 153

# 9.12 CTRL1_XL (10h) 

Accelerometer control register 1 (r/w) 

Table 43. CTRL1_XL register description 

Table 42. CTRL1_XL register 

ODR_XL3 ODR_XL2 ODR_XL1 ODR_XL0 FS1_XL FS0_XL LPF2_XL_ EN 0 (1) 1. This bit must be set to '0' for the correct operation of the device. ODR_XL[3:0] Accelerometer ODR selection (see Table 44 )FS[1:0]_XL Accelerometer full-scale selection (see Table 45 )LPF2_XL_EN Accelerometer high-resolution selection (0: output from first stage digital filtering selected (default); 1: output from LPF2 second filtering stage selected) 

Table 44. Accelerometer ODR register setting ODR_ XL3 ODR_ XL2 ODR_ XL1 ODR_ XL0 ODR selection [Hz] when XL_HM_MODE = 1 in 

CTRL6_C (15h) 

ODR selection [Hz] when XL_HM_MODE = 0 in 

CTRL6_C (15h) 

0 0 0 0 Power-down Power-down 1 0 1 1 1.6 Hz (low power only) 12.5 Hz (high performance) 0 0 0 1 12.5 Hz (low power) 12.5 Hz (high performance) 0 0 1 0 26 Hz (low power) 26 Hz (high performance) 0 0 1 1 52 Hz (low power) 52 Hz (high performance) 0 1 0 0 104 Hz (normal mode) 104 Hz (high performance) 0 1 0 1 208 Hz (normal mode) 208 Hz (high performance) 0 1 1 0 416 Hz (high performance) 416 Hz (high performance) 0 1 1 1 833 Hz (high performance) 833 Hz (high performance) 1 0 0 0 1.66 kHz (high performance) 1.66 kHz (high performance) 1 0 0 1 3.33 kHz (high performance) 3.33 kHz (high performance) 1 0 1 0 6.66 kHz (high performance) 6.66 kHz (high performance) 1 1 x x Not allowed Not allowed 

Table 45. Accelerometer full-scale selection FS[1:0]_XL Accelerometer full-scale 

00 (default) ±4 g

01 ±32 g

10 ±8 g

11 ±16 gRegister description LSM6DSO32 

68/153 DocID032891 Rev 1 

# 9.13 CTRL2_G (11h) 

Gyroscope control register 2 (r/w) 

0 0 0 0 Power down  Power down 

0 0 0 1 12.5 Hz (low power)  12.5 Hz (high performance) 

0 0 1 0 26 Hz (low power)  26 Hz (high performance) 

0 0 1 1 52 Hz (low power)  52 Hz (high performance) 

0 1 0 0 104 Hz (normal mode)  104 Hz (high performance) 

0 1 0 1 208 Hz (normal mode)  208 Hz (high performance) 

0 1 1 0 416 Hz (high performance)  416 Hz (high performance) 

0 1 1 1 833 Hz (high performance)  833 Hz (high performance) 

1 0 0 0 1.66 kHz (high performance)  1.66 kHz (high performance) 

1 0 0 1 3.33 kHz (high performance  3.33 kHz (high performance) 

1 0 1 0 6.66 kHz (high performance  6.66 kHz (high performance) 

1 0 1 1 Not available  Not available 

Table 47. CTRL2_G register description 

Table 46. CTRL2_G register 

ODR_G3 ODR_G2 ODR_G1 ODR_G0 FS1_G FS0_G FS_125 0 (1) 1. This bit must be set to '0' for the correct operation of the device. ODR_G[3:0] Gyroscope output data rate selection. Default value: 0000 (Refer to Table 48 )FS[1:0]_G Gyroscope chain full-scale selection (00: ±250 dps; 01: ±500 dps; 10: ±1000 dps; 11: ±2000 dps) FS_125 Selects gyro chain full-scale ±125 dps (0: FS selected through bits FS[1:0]_G; 1: FS set to ±125 dps) 

Table 48. Gyroscope ODR configuration setting ODR_G3 ODR_G2 ODR_G1 ODR_G0 ODR [Hz] when G_HM_MODE = 1 in CTRL7_G (16h) 

ODR [Hz] when G_HM_MODE = 0 in CTRL7_G (16h) DocID032891 Rev 1 69/153 

LSM6DSO32 Register description 

> 153

# 9.14 CTRL3_C (12h) 

Control register 3 (r/w) 

Table 50. CTRL3_C register description Table 49. CTRL3_C register 

BOOT BDU H_LACTIVE PP_OD SIM IF_INC 0 (1) 1. This bit must be set to '0' for the correct operation of the device. SW_RESET BOOT Reboots memory content. Default value: 0 (0: normal mode; 1: reboot memory content) This bit is automatically cleared. BDU Block Data Update. Default value: 0 (0: continuous update; 1: output registers are not updated until MSB and LSB have been read) H_LACTIVE Interrupt activation level. Default value: 0 (0: interrupt output pins active high; 1: interrupt output pins active low) PP_OD Push-pull/open-drain selection on INT1 and INT2 pins. Default value: 0 (0: push-pull mode; 1: open-drain mode) SIM SPI Serial Interface Mode selection. Default value: 0 (0: 4-wire interface; 1: 3-wire interface) IF_INC Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI). Default value: 1 (0: disabled; 1: enabled) SW_RESET Software reset. Default value: 0 (0: normal mode; 1: reset device) This bit is automatically cleared. Register description LSM6DSO32 

70/153 DocID032891 Rev 1 

# 9.15 CTRL4_C (13h) 

Control register 4 (r/w) 

Table 52. CTRL4_C register description Table 51. CTRL4_C register 

0(1) 1. This bit must be set to '0' for the correct operation of the device. SLEEP_G INT2_on_ INT1 0 (1) DRDY_ MASK I2C_disable LPF1_ SEL_G 0(1) SLEEP_G Enables gyroscope Sleep mode. Default value:0 (0: disabled; 1: enabled) INT2_on_INT1 All interrupt signals available on INT1 pin enable. Default value: 0 (0: interrupt signals divided between INT1 and INT2 pins; 1: all interrupt signals in logic or on INT1 pin) DRDY_MASK Enables data available (0: disabled; 1: mask DRDY on pin (both XL & Gyro) until filter settling ends (XL and Gyro independently masked). I2C_disable Disables I²C interface. Default value: 0 (0: SPI, I²C and MIPI I3C SM interfaces enabled (default); 1: I²C interface disabled) LPF1_SEL_G Enables gyroscope digital LPF1; the bandwidth can be selected through FTYPE [2:0] in CTRL6_C (15h) .(0: disabled; 1: enabled) DocID032891 Rev 1 71/153 

LSM6DSO32 Register description 

> 153

# 9.16 CTRL5_C (14h) 

Control register 5 (r/w) 

Table 54. CTRL5_C register description 

Table 53. CTRL5_C register 

XL_ULP_EN ROUNDING1 ROUNDING0 0 ST1_G ST0_G ST1_XL ST0_XL XL_ULP_EN Accelerometer ultra-low-power mode enable (1) . Default value: 0 (0: Ultra-low-power mode disabled; 1: Ultra-low-power mode enabled) 1. Further details about the accelerometer ultra-low-power mode are provided in Section 6.2.1: Accelerometer ultra-low-power mode .ROUNDING[1:0] Circular burst-mode (rounding) read from the output registers. Default value: 00 (00: no rounding; 01: accelerometer only; 10: gyroscope only; 11: gyroscope + accelerometer) ST[1:0]_G Angular rate sensor self-test enable. Default value: 00 (00: Self-test disabled; Other: refer to Table 55 )ST[1:0]_XL Linear acceleration sensor self-test enable. Default value: 00 (00: Self-test disabled; Other: refer to Table 56 )

Table 55. Angular rate sensor self-test mode selection ST1_G ST0_G Self-test mode 

0 0 Normal mode 0 1 Positive sign self-test 1 0 Not allowed 1 1 Negative sign self-test 

Table 56. Linear acceleration sensor self-test mode selection ST1_XL ST0_XL Self-test mode 

0 0 Normal mode 0 1 Positive sign self-test 1 0 Negative sign self-test 1 1 Not allowed Register description LSM6DSO32 

72/153 DocID032891 Rev 1 

# 9.17 CTRL6_C (15h) 

Control register 6 (r/w) 

Table 57. CTRL6_C register 

TRIG_EN LVL1_EN LVL2_EN XL_HM_MODE USR_ OFF_W FTYPE_2 FTYPE_1 FTYPE_0 

Table 58. CTRL6_C register description 

TRIG_EN DEN data edge-sensitive trigger enable. Refer to Table 59 .LVL1_EN DEN data level-sensitive trigger enable. Refer to Table 59 .LVL2_EN DEN level-sensitive latched enable. Refer to Table 59 .XL_HM_MODE High-performance operating mode disable for accelerometer. Default value: 0 (0: high-performance operating mode enabled; 1: high-performance operating mode disabled) USR_OFF_W Weight of XL user offset bits of registers X_OFS_USR (73h) , Y_OFS_USR (74h) ,

Z_OFS_USR (75h) 

0 = 2 -10 g/LSB 1 = 2 -6 g/LSB FTYPE[2:0] Gyroscope's low-pass filter (LPF1) bandwidth selection 

Table 60 shows the selectable bandwidth values. 

Table 59. Trigger mode selection TRIG_EN, LVL1_EN, LVL2_EN Trigger mode 

100 Edge-sensitive trigger mode is selected 010 Level-sensitive trigger mode is selected 011 Level-sensitive latched mode is selected 110 Level-sensitive FIFO enable mode is selected 

Table 60. Gyroscope LPF1 bandwidth selection FTYPE [2:0] 12.5 Hz 26 Hz 52 Hz 104 Hz 208 Hz 416 Hz 833 Hz 1.67 kHz 3.33 kHz 6.67 kHz 

000 4.2 8.3 16.6 33.0 67.0 136.6 239.2 304.2 328.5 335.5 001 4.2 8.3 16.6 33.0 67.0 130.5 192.4 220.7 229.6 232.0 010 4.2 8.3 16.6 33.0 67.0 120.3 154.2 166.6 170.1 171.1 011 4.2 8.3 16.6 33.0 67.0 137.1 281.8 453.2 559.2 609.0 100 4.2 8.3 16.7 33.0 62.4 86.7 96.6 99.6 100.4 100.6 101 4.2 8.3 16.8 31.0 43.2 48.0 49.4 49.8 49.9 49.9 110 4.1 7.8 13.4 19.0 23.1 24.6 25.0 25.1 25.1 25.1 111 3.9 6.7 9.7 11.5 12.2 12.4 12.5 12.5 12.5 12.5 DocID032891 Rev 1 73/153 

LSM6DSO32 Register description 

> 153

# 9.18 CTRL7_G (16h) 

Control register 7 (r/w) 

Table 62. CTRL7_G register description 9.19 CTRL8_XL (17h) 

Control register 8 (r/w) 

Table 61. CTRL7_G register 

G_HM_ MODE HP_EN_G HPM1_G HPM0_G 0 (1) 1. This bit must be set to '0' for the correct operation of the device. 0 (1) USR_ OFF_ON _ OUT 0 (1) G_HM_MODE Disables high-performance operating mode for gyroscope. Default: 0 (0: high-performance operating mode enabled; 1: high-performance operating mode disabled) HP_EN_G Enables gyroscope digital high-pass filter. The filter is enabled only if the gyro is in HP mode. Default value: 0 (0: HPF disabled; 1: HPF enabled) HPM_G[1:0] Gyroscope digital HP filter cutoff selection. Default: 00 (00: 16 mHz; 01: 65 mHz; 10: 260 mHz; 11: 1.04 Hz) USR_OFF_ ON_ OUT Enables accelerometer user offset correction block; it's valid for the low-pass path - see Figure 17: Accelerometer composite filter . Default value: 0 (0: accelerometer user offset correction block bypassed; 1: accelerometer user offset correction block enabled) 

Table 63. CTRL8_XL register 

HPCF_XL _2 HPCF_XL _1 HPCF_XL _0 HP_REF_ MODE_XL FASTSETTL _MODE_XL HP_SLOPE _XL_EN 0(1) 1. This bit must be set to '0' for the correct operation of the device. LOW_PASS _ON_6D Register description LSM6DSO32 

74/153 DocID032891 Rev 1 

Table 64. CTRL8_XL register description 

HPCF_XL_[2:0] Accelerometer LPF2 and HP filter configuration and cutoff setting. Refer to 

Table 65 .HP_REF_ MODE_XL Enables accelerometer high-pass filter reference mode (valid for high-pass path -HP_SLOPE_XL_EN bit must be ‘1’). Default value: 0 (0: disabled, 1: enabled (1) )1. When enabled, the first output data have to be discarded. FASTSETTL _MODE_XL Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the second samples after writing this bit. Active only during device exit from power-down mode. Default value: 0 (0: disabled, 1: enabled) HP_SLOPE_ XL_EN Accelerometer slope filter / high-pass filter selection. Refer to Figure 21 .LOW_PASS _ON_6D LPF2 on 6D function selection. Refer to Figure 21 . Default value: 0 (0: ODR/2 low-pass filtered data sent to 6D interrupt function; 1: LPF2 output data sent to 6D interrupt function) 

Table 65. Accelerometer bandwidth configurations Filter type HP_SLOPE_ XL_EN LPF2_XL_EN HPCF_XL_[2:0] Bandwidth 

Low pass 00 - ODR/2 1000 ODR/4 001 ODR/10 010 ODR/20 011 ODR/45 100 ODR/100 101 ODR/200 110 ODR/400 111 ODR/800 High pass 1 -000 SLOPE (ODR/4) 001 ODR/10 010 ODR/20 011 ODR/45 100 ODR/100 101 ODR/200 110 ODR/400 111 ODR/800 DocID032891 Rev 1 75/153 

LSM6DSO32 Register description 

> 153

Figure 21. Accelerometer block diagram 

6/23( ),/7(5 +3&)B;/>@    « 63, , &  +3B6/23(B;/B(1 /3)B;/B(1 'LJLWDO +3)LOWHU +3&)B;/>@ 'LJLWDO /3)LOWHU /3) +3&)B;/>@ 6'7DS '' /2:B3$66B21B' 6/23(B)'6 :DNHXS $FWLYLW\ ,QDFWLYLW\ )UHHIDOO $GYDQFHG IXQFWLRQV ),)2 86(5 2))6(7 865B2))B21B287 865B2))B: 2)6B865>@ 865B2))B21B:8 /3) RXWSXW 0,3,,& 60 Register description LSM6DSO32 

76/153 DocID032891 Rev 1 

# 9.20 CTRL9_XL (18h) 

Control register 9 (r/w) 

Table 67. CTRL9_XL register description 9.21 CTRL10_C (19h) 

Control register 10 (r/w) 

Table 69. CTRL10_C register description Table 66. CTRL9_XL register 

DEN_X DEN_Y DEN_Z DEN_XL_G DEN_XL_EN DEN_LH I3C_disable 0 (1) 1. This bit must be set to '0' for the correct operation of the device. DEN_X DEN value stored in LSB of X-axis. Default value: 1 (0: DEN not stored in X-axis LSB; 1: DEN stored in X-axis LSB) DEN_Y DEN value stored in LSB of Y-axis. Default value: 1 (0: DEN not stored in Y-axis LSB; 1: DEN stored in Y-axis LSB) DEN_Z DEN value stored in LSB of Z-axis. Default value: 1 (0: DEN not stored in Z-axis LSB; 1: DEN stored in Z-axis LSB) DEN_XL_G DEN stamping sensor selection. Default value: 0 (0: DEN pin info stamped in the gyroscope axis selected by bits [7:5]; 1: DEN pin info stamped in the accelerometer axis selected by bits [7:5]) DEN_XL_EN Extends DEN functionality to accelerometer sensor. Default value: 0 (0: disabled; 1: enabled) DEN_LH DEN active level configuration. Default value: 0 (0: active low; 1: active high) I3C_disable Disables MIPI I3C SM communication protocol (1) (0: SPI, I²C, MIPI I3C SM interfaces enabled (default); 1: MIPI I3C SM interface disabled) 1. It is recommended to set this bit to '1' during the initial device configuration phase, when the MIPI I3C SM interface is not used. 

Table 68. CTRL10_C register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. 0(1) TIMESTAMP _EN 0 (1) 0 (1) 0 (1) 0(1) 0(1) TIMESTAMP_EN Enables timestamp counter. default value: 0 (0: disabled; 1: enabled) The counter is readable in TIMESTAMP0 (40h), TIMESTAMP1 (41h), TIMESTAMP2 (42h), and TIMESTAMP3 (43h) .DocID032891 Rev 1 77/153 

LSM6DSO32 Register description 

> 153

# 9.22 ALL_INT_SRC (1Ah) 

Source register for all interrupts (r) 

Table 71. ALL_INT_SRC register description 9.23 WAKE_UP_SRC (1Bh) 

Wake-up interrupt source register (r) 

Table 70. ALL_INT_SRC register 

TIMESTAMP_ ENDCOUNT 0 SLEEP_ CHANGE_IA D6D_IA DOUBLE_ TAP SINGLE_ TAP WU_IA FF_IA TIMESTAMP_ENDCOUNT Alerts timestamp overflow within 6.4 ms SLEEP_CHANGE_IA Detects change event in activity/inactivity status. Default value: 0 (0: change status not detected; 1: change status detected) D6D_IA Interrupt active for change in position of portrait, landscape, face-up, face-down. Default value: 0 (0: change in position not detected; 1: change in position detected) DOUBLE_TAP Double-tap event status. Default value: 0 (0:event not detected, 1: event detected) SINGLE_TAP Single-tap event status. Default value:0 (0: event not detected, 1: event detected) WU_IA Wake-up event status. Default value: 0 (0: event not detected, 1: event detected) FF_IA Free-fall event status. Default value: 0 (0: event not detected, 1: event detected) 

Table 72. WAKE_UP_SRC register 

0 SLEEP_ CHANGE_IA FF_IA SLEEP_ STATE WU_IA X_WU Y_WU Z_WU 

Table 73. WAKE_UP_SRC register description 

SLEEP_ CHANGE_IA Detects change event in activity/inactivity status. Default value: 0 (0: change status not detected; 1: change status detected) FF_IA Free-fall event detection status. Default value: 0 (0: free-fall event not detected; 1: free-fall event detected) SLEEP_ STATE Sleep status bit. Default value: 0 (0: Activity status; 1: Inactivity status) WU_IA Wakeup event detection status. Default value: 0 (0: wakeup event not detected; 1: wakeup event detected.) X_WU Wakeup event detection status on X-axis. Default value: 0 (0: wakeup event on X-axis not detected; 1: wakeup event on X-axis detected) Y_WU Wakeup event detection status on Y-axis. Default value: 0 (0: wakeup event on Y-axis not detected; 1: wakeup event on Y-axis detected) Z_WU Wakeup event detection status on Z-axis. Default value: 0 (0: wakeup event on Z-axis not detected; 1: wakeup event on Z-axis detected) Register description LSM6DSO32 

78/153 DocID032891 Rev 1 

# 9.24 TAP_SRC (1Ch) 

Tap source register (r) 

# 9.25 D6D_SRC (1Dh) 

Portrait, landscape, face-up and face-down source register (r) 

Table 74. TAP_SRC register 

0 TAP_IA SINGLE_ TAP DOUBLE_ TAP TAP_SIGN X_TAP Y_TAP Z_TAP 

Table 75. TAP_SRC register description 

TAP_IA Tap event detection status. Default: 0 (0: tap event not detected; 1: tap event detected) SINGLE_TAP Single-tap event status. Default value: 0 (0: single tap event not detected; 1: single tap event detected) DOUBLE_TAP Double-tap event detection status. Default value: 0 (0: double-tap event not detected; 1: double-tap event detected.) TAP_SIGN Sign of acceleration detected by tap event. Default: 0 (0: positive sign of acceleration detected by tap event; 1: negative sign of acceleration detected by tap event) X_TAP Tap event detection status on X-axis. Default value: 0 (0: tap event on X-axis not detected; 1: tap event on X-axis detected) Y_TAP Tap event detection status on Y-axis. Default value: 0 (0: tap event on Y-axis not detected; 1: tap event on Y-axis detected) Z_TAP Tap event detection status on Z-axis. Default value: 0 (0: tap event on Z-axis not detected; 1: tap event on Z-axis detected) 

Table 76. D6D_SRC register 

DEN_DRDY D6D_IA ZH ZL YH YL XH XL 

Table 77. D6D_SRC register description 

DEN_ DRDY DEN data-ready signal. It is set high when data output is related to the data coming from a DEN active condition. (1) 1. The DEN data-ready signal can be latched or pulsed depending on the value of the dataready_pulsed bit of the COUNTER_BDR_REG1 (0Bh) register. D6D_ IA Interrupt active for change position portrait, landscape, face-up, face-down. Default value: 0 (0: change position not detected; 1: change position detected) ZH Z-axis high event (over threshold). Default value: 0 (0: event not detected; 1: event (over threshold) detected) ZL Z-axis low event (under threshold). Default value: 0 (0: event not detected; 1: event (under threshold) detected) YH Y-axis high event (over threshold). Default value: 0 (0: event not detected; 1: event (over-threshold) detected) YL Y-axis low event (under threshold). Default value: 0 (0: event not detected; 1: event (under threshold) detected) XH X-axis high event (over threshold). Default value: 0 (0: event not detected; 1: event (over threshold) detected) XL X-axis low event (under threshold). Default value: 0 (0: event not detected; 1: event (under threshold) detected) DocID032891 Rev 1 79/153 

LSM6DSO32 Register description 

> 153

# 9.26 STATUS_REG (1Eh) 

Status register (r) 

(0: no set of data available at accelerometer output; 

1: a new set of data is available at accelerometer output) 

# 9.27 OUT_TEMP_L (20h), OUT_TEMP_H (21h) 

Temperature data output register (r). L and H registers together express a 16-bit word in two’s complement. 

# 9.28 OUTX_L_G (22h) and OUTX_H_G (23h) 

Angular rate sensor pitch axis (X) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement. 

Table 78. STATUS_REG register 

0 0 0 0 0 TDA GDA XLDA 

Table 79. STATUS_REG register description 

TDA Temperature new data available. Default: 0 (0: no set of data is available at temperature sensor output; 1: a new set of data is available at temperature sensor output) GDA Gyroscope new data available. Default value: 0 (0: no set of data available at gyroscope output; 1: a new set of data is available at gyroscope output) XLDA Accelerometer new data available. Default value: 0 

Table 80. OUT_TEMP_L register 

Temp7 Temp6 Temp5 Temp4 Temp3 Temp2 Temp1 Temp0 

Table 81. OUT_TEMP_H register 

Temp15 Temp14 Temp13 Temp12 Temp11 Temp10 Temp9 Temp8 

Table 82. OUT_TEMP register description 

Temp[15:0] Temperature sensor output data The value is expressed as two’s complement sign extended on the MSB. 

Table 83. OUTX_L_G register 

D7 D6 D5 D4 D3 D2 D1 D0 

Table 84. OUTX_H_G register 

D15 D14 D13 D12 D11 D10 D9 D8 

Table 85. OUTX_H_G register description 

D[15:0] Pitch axis (X) angular rate value Register description LSM6DSO32 

80/153 DocID032891 Rev 1 

# 9.29 OUTY_L_G (24h) and OUTY_H_G (25h) 

Angular rate sensor roll axis (Y) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement. 

# 9.30 OUTZ_L_G (26h) and OUTZ_H_G (27h) 

Angular rate sensor yaw axis (Z) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement. 

# 9.31 OUTX_L_A (28h) and OUTX_H_A (29h) 

Linear acceleration sensor X-axis output register (r). The value is expressed as a 16-bit word in two’s complement. 

Table 86. OUTY_L_G register 

D7 D6 D5 D4 D3 D2 D1 D0 

Table 87. OUTY_H_G register 

D15 D14 D13 D12 D11 D10 D9 D8 

Table 88. OUTY_H_G register description 

D[15:0] Roll axis (Y) angular rate value 

Table 89. OUTZ_L_G register 

D7 D6 D5 D4 D3 D2 D1 D0 

Table 90. OUTZ_H_G register 

D15 D14 D13 D12 D11 D10 D9 D8 

Table 91. OUTZ_H_G register description 

D[15:0] Yaw axis (Z) angular rate value 

Table 92. OUTX_L_A register 

D7 D6 D5 D4 D3 D2 D1 D0 

Table 93. OUTX_H_A register 

D15 D14 D13 D12 D11 D10 D9 D8 

Table 94. OUTX_H_A register description 

D[15:0] X-axis linear acceleration value. DocID032891 Rev 1 81/153 

LSM6DSO32 Register description 

> 153

# 9.32 OUTY_L_A (2Ah) and OUTY_H_A (2Bh) 

Linear acceleration sensor Y-axis output register (r). The value is expressed as a 16-bit word in two’s complement. 

# 9.33 OUTZ_L_A (2Ch) and OUTZ_H_A (2Dh) 

Linear acceleration sensor Z-axis output register (r). The value is expressed as a 16-bit word in two’s complement. 

Table 95. OUTY_L_A register 

D7 D6 D5 D4 D3 D2 D1 D0 

Table 96. OUTY_H_A register 

D15 D14 D13 D12 D11 D10 D9 D8 

Table 97. OUTY_H_A register description 

D[15:0] Y-axis linear acceleration value 

Table 98. OUTZ_L_A register 

D7 D6 D5 D4 D3 D2 D1 D0 

Table 99. OUTZ_H_A register 

D15 D14 D13 D12 D11 D10 D9 D8 

Table 100. OUTZ_H_A register description 

D[15:0] Z-axis linear acceleration value Register description LSM6DSO32 

82/153 DocID032891 Rev 1 

# 9.34 EMB_FUNC_STATUS_MAINPAGE (35h) 

Embedded function status register (r) 

Table 102. EMB_FUNC_STATUS_MAINPAGE register description 9.35 FSM_STATUS_A_MAINPAGE (36h) 

Finite State Machine status register (r) 

Table 104. FSM_STATUS_A_MAINPAGE register description Table 101. EMB_FUNC_STATUS_MAINPAGE register 

IS_FSM_LC 0 IS_ SIGMOT IS_ TILT IS_STEP_ DET 0 0 0IS_FSM_LC Interrupt status bit for FSM long counter timeout interrupt event. (1: interrupt detected; 0: no interrupt) IS_SIGMOT Interrupt status bit for significant motion detection (1: interrupt detected; 0: no interrupt) IS_TILT Interrupt status bit for tilt detection (1: interrupt detected; 0: no interrupt) IS_STEP_DET Interrupt status bit for step detection (1: interrupt detected; 0: no interrupt) 

Table 103. FSM_STATUS_A_MAINPAGE register 

IS_FSM8 IS_FSM7 IS_FSM6 IS_FSM5 IS_FSM4 IS_FSM3 IS_FSM2 IS_FSM1 IS_FSM8 Interrupt status bit for FSM8 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM7 Interrupt status bit for FSM7 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM6 Interrupt status bit for FSM6 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM5 Interrupt status bit for FSM5 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM4 Interrupt status bit for FSM4 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM3 Interrupt status bit for FSM3 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM2 Interrupt status bit for FSM2 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM1 Interrupt status bit for FSM1 interrupt event. (1: interrupt detected; 0: no interrupt) DocID032891 Rev 1 83/153 

LSM6DSO32 Register description 

> 153

# 9.36 FSM_STATUS_B_MAINPAGE (37h) 

Finite State Machine status register (r) 

Table 106. FSM_STATUS_B_MAINPAGE register description 9.37 STATUS_MASTER_MAINPAGE (39h) 

Sensor hub source register (r) 

Table 105. FSM_STATUS_B_MAINPAGE register 

IS_FSM16 IS_FSM15 IS_FSM14 IS_FSM13 IS_FSM12 IS_FSM11 IS_FSM10 IS_FSM9 IS_FSM16 Interrupt status bit for FSM16 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM15 Interrupt status bit for FSM15 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM14 Interrupt status bit for FSM14 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM13 Interrupt status bit for FSM13 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM12 Interrupt status bit for FSM12 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM11 Interrupt status bit for FSM11 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM10 Interrupt status bit for FSM10 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM9 Interrupt status bit for FSM9 interrupt event. (1: interrupt detected; 0: no interrupt) 

Table 107. STATUS_MASTER_MAINPAGE register 

WR_ ONCE_ DONE SLAVE3_ NACK SLAVE2_ NACK SLAVE1_ NACK SLAVE0_ NACK 0 0 SENS_HUB_ ENDOP 

Table 108. STATUS_MASTER_MAINPAGE register description 

WR_ONCE_DONE When the bit WRITE_ONCE in MASTER_CONFIG (14h) is configured as 1, this bit is set to 1 when the write operation on slave 0 has been performed and completed. Default value: 0 SLAVE3_NACK This bit is set to 1 if Not acknowledge occurs on slave 3 communication. Default value: 0 SLAVE2_NACK This bit is set to 1 if Not acknowledge occurs on slave 2 communication. Default value: 0 SLAVE1_NACK This bit is set to 1 if Not acknowledge occurs on slave 1 communication. Default value: 0 SLAVE0_NACK This bit is set to 1 if Not acknowledge occurs on slave 0 communication. Default value: 0 SENS_HUB_ENDOP Sensor hub communication status. Default value: 0 (0: sensor hub communication not concluded; 1: sensor hub communication concluded) Register description LSM6DSO32 

84/153 DocID032891 Rev 1 

# 9.38 FIFO_STATUS1 (3Ah) 

FIFO status register 1 (r) 

Table 110. FIFO_STATUS1 register description 9.39 FIFO_STATUS2 (3Bh) 

FIFO status register 2 (r) 

Table 112. FIFO_STATUS2 register description Table 109. FIFO_STATUS1 register 

DIFF_ FIFO_7 DIFF_ FIFO_6 DIFF_ FIFO_5 DIFF_ FIFO_4 DIFF_ FIFO_3 DIFF_ FIFO_2 DIFF_ FIFO_1 DIFF_ FIFO_0 DIFF_ FIFO_[7:0] Number of unread sensor data (TAG + 6 bytes) stored in FIFO In conjunction with DIFF_FIFO[9:8] in FIFO_STATUS2 (3Bh). 

Table 111. FIFO_STATUS2 register 

FIFO_ WTM_IA FIFO_ OVR_IA FIFO_ FULL_IA COUNTER _BDR_IA FIFO_OVR_ LATCHED 0 DIFF_ FIFO_9 DIFF_ FIFO_8 FIFO_ WTM_IA FIFO watermark status. Default value: 0 (0: FIFO filling is lower than WTM; 1: FIFO filling is equal to or greater than WTM) Watermark is set through bits WTM[8:0] in FIFO_CTRL2 (08h) and FIFO_CTRL1 (07h) .FIFO_ OVR_IA FIFO overrun status. Default value: 0 (0: FIFO is not completely filled; 1: FIFO is completely filled) FIFO_ FULL_IA Smart FIFO full status. Default value: 0 (0: FIFO is not full; 1: FIFO will be full at the next ODR) COUNTER_ BDR_IA Counter BDR reaches the CNT_BDR_TH_[10:0] threshold set in 

COUNTER_BDR_REG1 (0Bh) and COUNTER_BDR_REG2 (0Ch) . Default value: 0 This bit is reset when these registers are read. FIFO_OVR_ LATCHED Latched FIFO overrun status. Default value: 0 This bit is reset when this register is read. DIFF_ FIFO_[9:8] Number of unread sensor data (TAG + 6 bytes) stored in FIFO. Default value: 00 In conjunction with DIFF_FIFO[7:0] in FIFO_STATUS1 (3Ah) DocID032891 Rev 1 85/153 

LSM6DSO32 Register description 

> 153

# 9.40 TIMESTAMP0 (40h), TIMESTAMP1 (41h), TIMESTAMP2 (42h), and TIMESTAMP3 (43h) 

Timestamp first data output register (r). The value is expressed as a 32-bit word and the bit resolution is 25 μs. 

Table 114. TIMESTAMP output register description Table 113. TIMESTAMP output registers 

D31 D30 D29 D28 D27 D26 D25 D24 D23 D22 D21 D20 D19 D18 D17 D16 D15 D14 D13 D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 D[31:0] Timestamp output registers: 1LSB = 25 μs Register description LSM6DSO32 

86/153 DocID032891 Rev 1 

# 9.41 TAP_CFG0 (56h) 

Activity/inactivity functions, configuration of filtering and tap recognition functions (r/w) 

Table 116. TAP_CFG0 register description Table 115. TAP_CFG0 register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. INT_CLR _ON_ READ SLEEP_ STATUS_ ON_INT SLOPE_ FDS TAP_X_EN TAP_Y_EN TAP_Z_EN LIR INT_CLR_ON_ READ This bit allows immediately clearing the latched interrupts of an event detection upon the read of the corresponding status register. It must be set to 1 together with LIR. Default value: 0 (0: latched interrupt signal cleared at the end of the ODR period; 1: latched interrupt signal immediately cleared) SLEEP_STATUS _ON_INT Activity/inactivity interrupt mode configuration. If INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits are enabled, drives the sleep status or sleep change on the INT pins. Default value: 0 (0: sleep change notification on INT pins; 1: sleep status reported on INT pins) SLOPE_FDS HPF or SLOPE filter selection on wake-up and Activity/Inactivity functions. Default value: 0 ( 0: SLOPE filter applied; 1: HPF applied) TAP_X_EN Enable X direction in tap recognition. Default value: 0 (0: X direction disabled; 1: X direction enabled) TAP_Y_EN Enable Y direction in tap recognition. Default value: 0 (0: Y direction disabled; 1: Y direction enabled) TAP_Z_EN Enable Z direction in tap recognition. Default value: 0 (0: Z direction disabled; 1: Z direction enabled) LIR Latched Interrupt. Default value: 0 (0: interrupt request not latched; 1: interrupt request latched) DocID032891 Rev 1 87/153 

LSM6DSO32 Register description 

> 153

# 9.42 TAP_CFG1 (57h) 

Tap configuration register (r/w) 

Table 118. TAP_CFG1 register description 

# 9.43 TAP_CFG2 (58h) 

Enables interrupt and inactivity functions, and tap recognition functions (r/w) 

Table 121. TAP_CFG2 register description Table 117. TAP_CFG1 register 

TAP_PRI ORITY_2 TAP_PRI ORITY_1 TAP_PRIO RITY_0 TAP_THS _X_4 TAP_THS_ X_3 TAP_THS_ X_2 TAP_THS_ X_1 TAP_THS _X_0 TAP_PRIORITY_[2:0] Selection of axis priority for TAP detection (see Table 119 )TAP_THS_X_[4:0] X-axis tap recognition threshold. Default value: 0 1 LSB = FS_XL / (2 5)

Table 119. TAP priority decoding TAP_PRIORITY_[2:0] Max. priority Mid. priority Min. priority 

000 X Y Z001 Y X Z010 X Z Y011 Z Y X100 X Y Z101 Y Z X110 Z X Y111 Z Y X

Table 120. TAP_CFG2 register 

INTERRUPTS_ ENABLE INACT_ EN1 INACT_ EN0 TAP_THS _Y_4 TAP_THS _Y_3 TAP_THS _Y_2 TAP_THS _Y_1 TAP_THS _Y_0 INTERRUPTS_ ENABLE Enable basic interrupts (6D/4D, free-fall, wake-up, tap, inactivity). Default value: 0 (0: interrupt disabled; 1: interrupt enabled) INACT_EN[1:0] Enable activity/inactivity (sleep) function. Default value: 00 (00: stationary/motion-only interrupts generated, XL and gyro do not change; 01: sets accelerometer ODR to 12.5 Hz (low-power mode), gyro does not change; 10: sets accelerometer ODR to 12.5 Hz (low-power mode), gyro to sleep mode; 11: sets accelerometer ODR to 12.5 Hz (low-power mode), gyro to power-down mode) TAP_THS_ Y_[4:0] Y-axis tap recognition threshold. Default value: 0 1 LSB = FS_XL / (2 5)Register description LSM6DSO32 

88/153 DocID032891 Rev 1 

# 9.44 TAP_THS_6D (59h) 

Portrait/landscape position and tap function threshold register (r/w) 

00  68 degrees 

01  47 degrees 

10  Reserved 

11  Reserved 

Table 123. TAP_THS_6D register description 

# 9.45 INT_DUR2 (5Ah) 

Tap recognition function setting register (r/w) 

Table 122. TAP_THS_6D register 

D4D_ EN SIXD_ THS1 SIXD_ THS0 TAP_ THS_Z_4 TAP_ THS_Z_3 TAP_ THS_Z_2 TAP_ THS_Z_1 TAP_ THS_Z_0 D4D_EN 4D orientation detection enable. Z-axis position detection is disabled. Default value: 0 (0: disabled; 1: enabled) SIXD_THS[1:0] Threshold for 4D/6D function. Default value: 00 For details, refer to Table 124 .TAP_THS_Z_[4:0] Z-axis recognition threshold. Default value: 0 1 LSB = FS_XL / (2 5)

Table 124. Threshold for D4D/D6D function SIXD_THS[1:0] Threshold value 

Table 125. INT_DUR2 register 

DUR3 DUR2 DUR1 DUR0 QUIET1 QUIET0 SHOCK1 SHOCK0 

Table 126. INT_DUR2 register description 

DUR[3:0] Duration of maximum time gap for double tap recognition. Default: 0000 When double tap recognition is enabled, this register expresses the maximum time between two consecutive detected taps to determine a double tap event. The default value of these bits is 0000b which corresponds to 16*ODR_XL time. If the DUR[3:0] bits are set to a different value, 1LSB corresponds to 32*ODR_XL time. QUIET[1:0] Expected quiet time after a tap detection. Default value: 00 Quiet time is the time after the first detected tap in which there must not be any overthreshold event. The default value of these bits is 00b which corresponds to 2*ODR_XL time. If the QUIET[1:0] bits are set to a different value, 1LSB corresponds to 4*ODR_XL time. SHOCK[1:0] Maximum duration of overthreshold event. Default value: 00 Maximum duration is the maximum time of an overthreshold signal detection to be recognized as a tap event. The default value of these bits is 00b which corresponds to 4*ODR_XL time. If the SHOCK[1:0] bits are set to a different value, 1LSB corresponds to 8*ODR_XL time. DocID032891 Rev 1 89/153 

LSM6DSO32 Register description 

> 153

# 9.46 WAKE_UP_THS (5Bh) 

Single/double-tap selection and wake-up configuration (r/w) 

Table 128. WAKE_UP_THS register description 9.47 WAKE_UP_DUR (5Ch) 

Free-fall, wakeup and sleep mode functions duration setting register (r/w) 

Table 130. WAKE_UP_DUR register description Table 127. WAKE_UP_THS register 

SINGLE_ DOUBLE_ TAP USR_OFF _ON_WU WK_THS5 WK_THS4 WK_THS3 WK_THS2 WK_THS1 WK_THS0 SINGLE_ DOUBLE_TAP Single/double-tap event enable. Default: 0 (0: only single-tap event enabled; 1: both single and double-tap events enabled) USR_OFF_ ON_WU Drives the low-pass filtered data with user offset correction (instead of high-pass filtered data) to the wakeup function. WK_THS[5:0] Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in 

WAKE_UP_DUR (5Ch) . Default value: 000000 

Table 129. WAKE_UP_DUR register 

FF_DUR5 WAKE_ DUR1 WAKE_ DUR0 WAKE_ THS_W SLEEP_ DUR3 SLEEP_ DUR2 SLEEP_ DUR1 SLEEP_ DUR0 FF_DUR5 Free fall duration event. Default: 0 For the complete configuration of the free-fall duration, refer to FF_DUR[4:0] in 

FREE_FALL (5Dh) configuration. 1 LSB = 1 ODR_time WAKE_DUR[1:0] Wake up duration event. Default: 00 1LSB = 1 ODR_time WAKE_THS_W Weight of 1 LSB of wakeup threshold. Default: 0 (0: 1 LSB = FS_XL / (2 6); 1: 1 LSB = FS_XL / (2 8 ) ) SLEEP_DUR[3:0] Duration to go in sleep mode. Default value: 0000 (this corresponds to 16 ODR) 1 LSB = 512 ODR Register description LSM6DSO32 

90/153 DocID032891 Rev 1 

# 9.48 FREE_FALL (5Dh) 

Free-fall function duration setting register (r/w) 

Table 131. FREE_FALL register 

FF_DUR4 FF_DUR3 FF_DUR2 FF_DUR1 FF_DUR0 FF_THS2 FF_THS1 FF_THS0 

Table 132. FREE_FALL register description 

FF_DUR[4:0] Free-fall duration event. Default: 0 For the complete configuration of the free fall duration, refer to FF_DUR5 in 

WAKE_UP_DUR (5Ch) configuration FF_THS[2:0] Free fall threshold setting. Default: 000 For details refer to Table 133 .

Table 133. Threshold for free-fall function FF_THS[2:0] Threshold value 

000 312 m g

001 438 m g

010 500 m g

011 Reserved 100 Reserved 101 Reserved 110 Reserved 111 Reserved DocID032891 Rev 1 91/153 

LSM6DSO32 Register description 

> 153

# 9.49 MD1_CFG (5Eh) 

Functions routing on INT1 register (r/w) 

Table 135. MD1_CFG register description 

Table 134. MD1_CFG register 

INT1_ SLEEP_ CHANGE INT1_ SINGLE_ TAP INT1_WU INT1_FF INT1_ DOUBLE_ TAP INT1_6D INT1_EMB _FUNC INT1_ SHUB INT1_SLEEP_ CHANGE (1) 1. Activity/Inactivity interrupt mode (sleep change or sleep status) depends on the SLEEP_STATUS_ON_INT bit in TAP_CFG0 (56h) register. Routing of activity/inactivity recognition event on INT1. Default: 0 (0: routing of activity/inactivity event on INT1 disabled; 1: routing of activity/inactivity event on INT1 enabled) INT1_SINGLE_TAP Routing of single-tap recognition event on INT1. Default: 0 (0: routing of single-tap event on INT1 disabled; 1: routing of single-tap event on INT1 enabled) INT1_WU Routing of wakeup event on INT1. Default value: 0 (0: routing of wakeup event on INT1 disabled; 1: routing of wakeup event on INT1 enabled) INT1_FF Routing of free-fall event on INT1. Default value: 0 (0: routing of free-fall event on INT1 disabled; 1: routing of free-fall event on INT1 enabled) INT1_DOUBLE_TAP Routing of tap event on INT1. Default value: 0 (0: routing of double-tap event on INT1 disabled; 1: routing of double-tap event on INT1 enabled) INT1_6D Routing of 6D event on INT1. Default value: 0 (0: routing of 6D event on INT1 disabled; 1: routing of 6D event on INT1 enabled) INT1_EMB_FUNC Routing of embedded functions event on INT1. Default value: 0 (0: routing of embedded functions event on INT1 disabled; 1: routing embedded functions event on INT1 enabled) INT1_SHUB Routing of sensor hub communication concluded event on INT1. Default value: 0 (0: routing of sensor hub communication concluded event on INT1 disabled; 1: routing of sensor hub communication concluded event on INT1 enabled) Register description LSM6DSO32 

92/153 DocID032891 Rev 1 

# 9.50 MD2_CFG (5Fh) 

Functions routing on INT2 register (r/w) 

Table 137. MD2_CFG register description Table 136. MD2_CFG register 

INT2_ SLEEP_ CHANGE INT2_ SINGLE_ TAP INT2_WU INT2_FF INT2_ DOUBLE_ TAP INT2_6D INT2_ EMB_ FUNC INT2_ TIMESTAMP INT2_SLEEP_CHANGE (1) 1. Activity/Inactivity interrupt mode (sleep change or sleep status) depends on the SLEEP_STATUS_ON_INT bit in TAP_CFG0 (56h) register. Routing of activity/inactivity recognition event on INT2. Default: 0 (0: routing of activity/inactivity event on INT2 disabled; 1: routing of activity/inactivity event on INT2 enabled) INT2_SINGLE_TAP Single-tap recognition routing on INT2. Default: 0 (0: routing of single-tap event on INT2 disabled; 1: routing of single-tap event on INT2 enabled) INT2_WU Routing of wakeup event on INT2. Default value: 0 (0: routing of wakeup event on INT2 disabled; 1: routing of wake-up event on INT2 enabled) INT2_FF Routing of free-fall event on INT2. Default value: 0 (0: routing of free-fall event on INT2 disabled; 1: routing of free-fall event on INT2 enabled) INT2_DOUBLE_TAP Routing of tap event on INT2. Default value: 0 (0: routing of double-tap event on INT2 disabled; 1: routing of double-tap event on INT2 enabled) INT2_6D Routing of 6D event on INT2. Default value: 0 (0: routing of 6D event on INT2 disabled; 1: routing of 6D event on INT2 enabled) INT2_EMB_FUNC Routing of embedded functions event on INT2. Default value: 0 (0: routing of embedded functions event on INT2 disabled; 1: routing embedded functions event on INT2 enabled) INT2_TIMESTAMP Enables routing on INT2 pin of the alert for timestamp overflow within 6.4 ms DocID032891 Rev 1 93/153 

LSM6DSO32 Register description 

> 153

# 9.51 I3C_BUS_AVB (62h) 

I3C_BUS_AVB register (r/w) 

Table 139. I3C_BUS_AVB register description 9.52 INTERNAL_FREQ_FINE (63h) 

Internal frequency register (r) 

Table 141. INTERNAL_FREQ_FINE register description 9.53 X_OFS_USR (73h) 

Accelerometer X-axis user offset correction (r/w). The offset value set in the X_OFS_USR offset register is internally subtracted from the acceleration value measured on the X-axis. 

Table 138. I3C_BUS_AVB register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. 0 (1) 0(1) I3C_Bus_ Avb_Sel1 I3C_Bus_ Avb_Sel0 0 (1) 0 (1) PD_DIS_ INT1 PD_DIS_INT1 This bit allows disabling the INT1 pull-down. (0: Pull-down on INT1 enabled (pull-down is effectively connected only when no interrupts are routed to the INT1 pin or when I3C dynamic address is assigned); 1: Pull-down on INT1 disabled (pull-down not connected) I3C_Bus_Avb_ Sel[1:0] These bits are used to select the bus available time when I3C IBI is used. Default value: 00 (00: bus available time equal to 50 μsec (default); 01: bus available time equal to 2 μsec; 10: bus available time equal to 1 msec; 11: bus available time equal to 25 msec) 

Table 140. INTERNAL_FREQ_FINE register 

FREQ_ FINE7 FREQ_ FINE6 FREQ_ FINE5 FREQ_ FINE4 FREQ_ FINE3 FREQ_ FINE2 FREQ_ FINE1 FREQ_ FINE0 FREQ_FINE[7:0] Difference in percentage of the effective ODR (and Timestamp Rate) with respect to the typical. Step: 0.15%. 8-bit format, 2's complement. 

Table 142. X_OFS_USR register 

X_OFS_ USR_7 X_OFS_ USR_6 X_OFS_ USR_5 X_OFS_ USR_4 X_OFS_ USR_3 X_OFS_ USR_2 X_OFS_ USR_1 X_OFS_ USR_0 

Table 143. X_OFS_USR register description 

X_OFS_USR_ [7:0] Accelerometer X-axis user offset correction expressed in two’s complement, weight depends on USR_OFF_W in CTRL6_C (15h) . The value must be in the range [-127 127]. Register description LSM6DSO32 

94/153 DocID032891 Rev 1 

# 9.54 Y_OFS_USR (74h) 

Accelerometer Y-axis user offset correction (r/w). The offset value set in the Y_OFS_USR offset register is internally subtracted from the acceleration value measured on the Y-axis. 

Table 145. Y_OFS_USR register description 9.55 Z_OFS_USR (75h) 

Accelerometer Z-axis user offset correction (r/w). The offset value set in the Z_OFS_USR offset register is internally subtracted from the acceleration value measured on the Z-axis. 

Table 147. Z_OFS_USR register description 9.56 FIFO_DATA_OUT_TAG (78h) 

FIFO tag register (r) 

Table 149. FIFO_DATA_OUT_TAG register description Table 144. Y_OFS_USR register 

Y_OFS_ USR_7 Y_OFS_ USR_6 Y_OFS_ USR_5 Y_OFS_ USR_4 Y_OFS_ USR_3 Y_OFS_ USR_2 Y_OFS_ USR_1 Y_OFS_ USR_0 Y_OFS_ USR_[7:0] Accelerometer Y-axis user offset calibration expressed in 2’s complement, weight depends on USR_OFF_W in CTRL6_C (15h) . The value must be in the range [-127, +127]. 

Table 146. Z_OFS_USR register 

Z_OFS_ USR_7 Z_OFS_ USR_6 Z_OFS_ USR_5 Z_OFS_ USR_4 Z_OFS_ USR_3 Z_OFS_ USR_2 Z_OFS_ USR_1 Z_OFS_ USR_0 Z_OFS_ USR_[7:0] Accelerometer Z-axis user offset calibration expressed in 2’s complement, weight depends on USR_OFF_W in CTRL6_C (15h) . The value must be in the range [-127, +127]. 

Table 148. FIFO_DATA_OUT_TAG register 

TAG_ SENSOR_ 4TAG_ SENSOR_ 3TAG_ SENSOR_ 2TAG_ SENSOR_ 1TAG_ SENSOR_ 0TAG_CNT _1 TAG_CNT _0 TAG_ PARITY TAG_SENSOR_[4:0] FIFO tag: identifies the sensor in: 

FIFO_DATA_OUT_X_L (79h) and FIFO_DATA_OUT_X_H (7Ah) ,

FIFO_DATA_OUT_Y_L (7Bh) and FIFO_DATA_OUT_Y_H (7Ch) , and 

FIFO_DATA_OUT_Z_L (7Dh) and FIFO_DATA_OUT_Z_H (7Eh) 

For details, refer to Table 150: FIFO tag 

TAG_CNT_[1:0] 2-bit counter which identifies sensor time slot TAG_PARITY Parity check of TAG content DocID032891 Rev 1 95/153 

LSM6DSO32 Register description 

> 153

Table 150. FIFO tag TAG_SENSOR_[4:0] Sensor name 

0x01 Gyroscope NC 0x02 Accelerometer NC 0x03 Temperature 0x04 Timestamp 0x05 CFG_Change 0x06 Accelerometer NC_T_2 0x07 Accelerometer NC_T_1 0x08 Accelerometer 2xC 0x09 Accelerometer 3xC 0x0A Gyroscope NC_T_2 0x0B Gyroscope NC_T_1 0x0C Gyroscope 2xC 0x0D Gyroscope 3xC 0x0E Sensor Hub Slave 0 0x0F Sensor Hub Slave 1 0x10 Sensor Hub Slave 2 0x11 Sensor Hub Slave 3 0x12 Step Counter 0x19 Sensor Hub Nack Register description LSM6DSO32 

96/153 DocID032891 Rev 1 

# 9.57 FIFO_DATA_OUT_X_L (79h) and FIFO_DATA_OUT_X_H (7Ah) 

FIFO data output X (r) 

Table 152. FIFO_DATA_OUT_X_H and FIFO_DATA_OUT_X_L register description 9.58 FIFO_DATA_OUT_Y_L (7Bh) and FIFO_DATA_OUT_Y_H (7Ch) 

FIFO data output Y (r) 

Table 154. FIFO_DATA_OUT_Y_H and FIFO_DATA_OUT_Y_L register description 9.59 FIFO_DATA_OUT_Z_L (7Dh) and FIFO_DATA_OUT_Z_H (7Eh) 

FIFO data output Z (r) 

Table 156. FIFO_DATA_OUT_Z_H and FIFO_DATA_OUT_Z_L register description Table 151. FIFO_DATA_OUT_X_H and FIFO_DATA_OUT_X_L registers 

D15 D14 D13 D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 D[15:0] FIFO X-axis output 

Table 153. FIFO_DATA_OUT_Y_H and FIFO_DATA_OUT_Y_L registers 

D15 D14 D13 D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 D[15:0] FIFO Y-axis output 

Table 155. FIFO_DATA_OUT_Z_H and FIFO_DATA_OUT_Z_L registers 

D15 D14 D13 D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 D[15:0] FIFO Z-axis output DocID032891 Rev 1 97/153 

LSM6DSO32 Embedded functions register mapping 

> 153

# 10 Embedded functions register mapping 

The table given below provides a list of the registers for the embedded functions available in the device and the corresponding addresses. Embedded functions registers are accessible when FUNC_CFG_EN is set to '1' in  FUNC_CFG_ACCESS (01h) .

Table 157. Register address map - embedded functions Name Type Register address Default Comment Hex Binary 

PAGE_SEL r/w 02 00000010 00000001 EMB_FUNC_EN_A r/w 04 00000100 00000000 EMB_FUNC_EN_B r/w 05 00000101 00000000 PAGE_ADDRESS r/w 08 00001000 00000000 PAGE_VALUE r/w 09 00001001 00000000 EMB_FUNC_INT1 r/w 0A 00001010 00000000 FSM_INT1_A r/w 0B 00001011 00000000 FSM_INT1_B r/w 0C 00001100 00000000 EMB_FUNC_INT2 r/w 0E 00001110 00000000 FSM_INT2_A r/w 0F 00001111 00000000 FSM_INT2_B r/w 10 00010000 00000000 EMB_FUNC_STATUS r 12 00010010 output FSM_STATUS_A r 13 00010011 output FSM_STATUS_B r 14 00010100 output PAGE_RW r/w 17 00010111 00000000 RESERVED - 18-43 EMB_FUNC_FIFO_CFG r/w 44 01000100 00000000 FSM_ENABLE_A r/w 46 01000110 00000000 FSM_ENABLE_B r/w 47 01000111 00000000 FSM_LONG_COUNTER_L r/w 48 01001000 00000000 FSM_LONG_COUNTER_H r/w 49 01001001 00000000 FSM_LONG_COUNTER_CLEAR r/w 4A 01001010 00000000 FSM_OUTS1 r 4C 01001100 output FSM_OUTS2 r 4D 01001101 output FSM_OUTS3 r 4E 01001110 output FSM_OUTS4 r 4F 01001111 output FSM_OUTS5 r 50 01010000 output FSM_OUTS6 r 51 01010001 output Embedded functions register mapping LSM6DSO32 

98/153 DocID032891 Rev 1 

Registers marked as Reserved must not be changed. Writing to those registers may cause permanent damage to the device. 

The content of the registers that are loaded at boot should not be changed. They contain the factory calibration values. Their content is automatically restored when the device is powered up. 

FSM_OUTS7 r 52 01010010 output FSM_OUTS8 r 53 01010011 output FSM_OUTS9 r 54 01010100 output FSM_OUTS10 r 55 01010101 output FSM_OUTS11 r 56 01010110 output FSM_OUTS12 r 57 01010111 output FSM_OUTS13 r 58 01011000 output FSM_OUTS14 r 59 01011001 output FSM_OUTS15 r 5A 01011010 output FSM_OUTS16 r 5B 01011011 output RESERVED - 5E 01011110 EMB_FUNC_ODR_CFG_B r/w 5F 01011111 01001011 STEP_COUNTER_L r 62 01100010 output STEP_COUNTER_H r 63 01100011 output EMB_FUNC_SRC r/w 64 01100100 output EMB_FUNC_INIT_A r/w 66 01100110 00000000 EMB_FUNC_INIT_B r/w 67 01100111 00000000 

Table 157. Register address map - embedded functions (continued) Name Type Register address Default Comment Hex Binary DocID032891 Rev 1 99/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11 Embedded functions register description 11.1 PAGE_SEL (02h) 

Enable advanced features dedicated page (r/w) 

Table 159. PAGE_SEL register description 11.2 EMB_FUNC_EN_A (04h) 

Embedded functions enable register (r/w) 

Table 161. EMB_FUNC_EN_A register description Table 158. PAGE_SEL register 

PAGE_SEL3 PAGE_SEL2 PAGE_SEL1 PAGE_SEL0 0 (1) 1. This bit must be set to '0' for the correct operation of the device. 0(1) 0(1) 1 (2) 2. This bit must be set to '1' for the correct operation of the device. PAGE_SEL[3:0] Select the advanced features dedicated page Default value: 0000 

Table 160. EMB_FUNC_EN_A register 

0(1) 1. This bit must be set to '0' for the correct operation of the device. 0(1) SIGN_ MOTION_ EN TILT_EN PEDO_EN 0 (1) 0(1) 0(1) SIGN_MOTION_EN Enable significant motion detection function. Default value: 0 (0: significant motion detection function disabled; 1: significant motion detection function enabled) TILT_EN Enable tilt calculation. Default value: 0 (0: tilt algorithm disabled; 1: tilt algorithm enabled) PEDO_EN Enable pedometer algorithm. Default value: 0 (0: pedometer algorithm disabled; 1: pedometer algorithm enabled) Embedded functions register description LSM6DSO32 

100/153 DocID032891 Rev 1 

# 11.3 EMB_FUNC_EN_B (05h) 

Embedded functions enable register (r/w) 

Table 163. EMB_FUNC_EN_B register description 11.4 PAGE_ADDRESS (08h) 

Page address register (r/w) 

Table 165. PAGE_ADDRESS register description 11.5 PAGE_VALUE (09h) 

Page value register (r/w) 

Table 167. PAGE_VALUE register description Table 162. EMB_FUNC_EN_B register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. 0 (1) 0(1) PEDO_ ADV_EN FIFO_CO MPR_EN 0 (1) 0 (1) FSM_EN PEDO_ADV_EN Enable pedometer false-positive rejection block and advanced detection feature block. Default value: 0 (0: Pedometer advanced features block disabled; 1: Pedometer advanced features block enabled) FIFO_COMPR_EN (1) 1. This bit is effective if the FIFO_COMPR_RT_EN bit of FIFO_CTRL2 (08h) is set to 1. Enable FIFO compression feature. Default value: 0 (0: FIFO compression feature disabled; 1: FIFO compression feature enabled) FSM_EN Enable Finite State Machine (FSM) feature. Default value: 0 (0: FSM feature disabled; 1: FSM feature enabled) 

Table 164. PAGE_ADDRESS register 

PAGE_ ADDR7 PAGE_ ADDR6 PAGE_ ADDR5 PAGE_ ADDR4 PAGE_ ADDR3 PAGE_ ADDR2 PAGE_ ADDR1 PAGE_ ADDR0 PAGE_ADDR[7:0] After setting the bit PAGE_WRITE / PAGE_READ in register PAGE_RW (17h) ,this register is used to set the address of the register to be written/read in the advanced features page selected through the bits PAGE_SEL[3:0] in register 

PAGE_SEL (02h) .

Table 166. PAGE_VALUE register 

PAGE_ VALUE7 PAGE_ VALUE6 PAGE_ VALUE5 PAGE_ VALUE4 PAGE_ VALUE3 PAGE_ VALUE2 PAGE_ VALUE1 PAGE_ VALUE0 PAGE_VALUE[7:0] These bits are used to write (if the bit PAGE_WRITE = 1 in register PAGE_RW (17h) ) or read (if the bit PAGE_READ = 1 in register PAGE_RW (17h) ) the data at the address PAGE_ADDR[7:0] of the selected advanced features page. DocID032891 Rev 1 101/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.6 EMB_FUNC_INT1 (0Ah) 

INT1 pin control register (r/w). 

Each bit in this register enables a signal to be carried over INT1. The pin's output will supply the OR combination of the selected signals. 

Table 169. EMB_FUNC_INT1 register description Table 168. EMB_FUNC_INT1 register 

INT1_ FSM_ LC 0(1) 1. This bit must be set to '0' for the correct operation of the device. INT1_ SIG_MOT INT1_TILT INT1_ STEP_ DETECTOR 0 (1) 0(1) 0(1) INT1_FSM_LC (1) 1. This bit is effective if the INT1_EMB_FUNC bit of MD1_CFG (5Eh) is set to 1. Routing of FSM long counter timeout interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_SIG_MOT (1) Routing of significant motion event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_TILT (1) Routing of tilt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_STEP_ DETECTOR (1) Routing of pedometer step recognition event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) Embedded functions register description LSM6DSO32 

102/153 DocID032891 Rev 1 

# 11.7 FSM_INT1_A (0Bh) 

INT1 pin control register (r/w). 

Each bit in this register enables a signal to be carried over INT1. The pin's output will supply the OR combination of the selected signals. 

Table 171. FSM_INT1_A register description Table 170. FSM_INT1_A register 

INT1_ FSM8 INT1_ FSM7 INT1_ FSM6 INT1_ FSM5 INT1_ FSM4 INT1_ FSM3 INT1_ FSM2 INT1_ FSM1 INT1_FSM8 (1) 1. This bit is effective if the INT1_EMB_FUNC bit of MD1_CFG (5Eh) is set to 1. Routing of FSM8 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM7 (1) Routing of FSM7 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM6 (1) Routing of FSM6 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM5 (1) Routing of FSM5 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM4 (1) Routing of FSM4 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM3 (1) Routing of FSM3 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM2 (1) Routing of FSM2 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM1 (1) Routing of FSM1 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) DocID032891 Rev 1 103/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.8 FSM_INT1_B (0Ch) 

INT1 pin control register (r/w). 

Each bit in this register enables a signal to be carried over INT1. The pin's output will supply the OR combination of the selected signals. 

Table 173. FSM_INT1_B register description Table 172. FSM_INT1_B register 

INT1_ FSM16 INT1_ FSM15 INT1_ FSM14 INT1_ FSM13 INT1_ FSM12 INT1_ FSM11 INT1_ FSM10 INT1_ FSM9 INT1_FSM16 (1) 1. This bit is effective if the INT1_EMB_FUNC bit of MD1_CFG (5Eh) is set to 1. Routing of FSM16 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM15 (1) Routing of FSM15 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM14 (1) Routing of FSM14 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM13 (1) Routing of FSM13 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM12 (1) Routing of FSM12 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM11 (1) Routing of FSM11 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM10 (1) Routing of FSM10 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) INT1_FSM9 (1) Routing of FSM9 interrupt event on INT1. Default value: 0 (0: routing on INT1 disabled; 1: routing on INT1 enabled) Embedded functions register description LSM6DSO32 

104/153 DocID032891 Rev 1 

# 11.9 EMB_FUNC_INT2 (0Eh) 

INT2 pin control register (r/w). 

Each bit in this register enables a signal to be carried over INT2. The pin's output will supply the OR combination of the selected signals. 

Table 175. EMB_FUNC_INT2 register description Table 174. EMB_FUNC_INT2 register 

INT2_ FSM_LC 0 (1) 1. This bit must be set to '0' for the correct operation of the device. INT2_SIG _MOT INT2_TILT INT2_ STEP_ DETECTOR 0 (1) 0 (1) 0(1) INT2_FSM_LC (1) 1. This bit is effective if the INT2_EMB_FUNC bit of MD2_CFG (5Fh) is set to 1. Routing of FSM long counter timeout interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_SIG_MOT (1) Routing of significant motion event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_TILT (1) Routing of tilt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_STEP_ DETECTOR (1) Routing of pedometer step recognition event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) DocID032891 Rev 1 105/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.10 FSM_INT2_A (0Fh) 

INT2 pin control register (r/w). 

Each bit in this register enables a signal to be carried over INT2. The pin's output will supply the OR combination of the selected signals. 

Table 177. FSM_INT2_A register description Table 176. FSM_INT2_A register 

INT2_ FSM8 INT2_ FSM7 INT2_ FSM6 INT2_ FSM5 INT2_ FSM4 INT2_ FSM3 INT2_ FSM2 INT2_ FSM1 INT2_FSM8 (1) 1. This bit is effective if the INT2_EMB_FUNC bit of MD2_CFG (5Fh) is set to 1. Routing of FSM8 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM7 (1) Routing of FSM7 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM6 (1) Routing of FSM6 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM5 (1) Routing of FSM5 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM4 (1) Routing of FSM4 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM3 (1) Routing of FSM3 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM2 (1) Routing of FSM2 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM1 (1) Routing of FSM1 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) Embedded functions register description LSM6DSO32 

106/153 DocID032891 Rev 1 

# 11.11 FSM_INT2_B (10h) 

INT2 pin control register (r/w). 

Each bit in this register enables a signal to be carried over INT2. The pin's output will supply the OR combination of the selected signals. 

Table 179. FSM_INT2_B register description 

Table 178. FSM_INT2_B register 

INT2_ FSM16 INT2_ FSM15 INT2_ FSM14 INT2_ FSM13 INT2_ FSM12 INT2_ FSM11 INT2_ FSM10 INT2_ FSM9 INT2_FSM16 (1) 1. This bit is effective if the INT2_EMB_FUNC bit of MD2_CFG (5Fh) is set to 1. Routing of FSM16 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM15 (1) Routing of FSM15 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM14 (1) Routing of FSM14 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM13 (1) Routing of FSM13 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM12 (1) Routing of FSM12 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM11 (1) Routing of FSM11 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM10 (1) Routing of FSM10 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) INT2_FSM9 (1) Routing of FSM9 interrupt event on INT2. Default value: 0 (0: routing on INT2 disabled; 1: routing on INT2 enabled) DocID032891 Rev 1 107/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.12 EMB_FUNC_STATUS (12h) 

Embedded function status register (r) 

Table 181. EMB_FUNC_STATUS register description 11.13 FSM_STATUS_A (13h) 

Finite State Machine status register (r) 

Table 183. FSM_STATUS_A register description Table 180. EMB_FUNC_STATUS register 

IS_FSM_LC 0 IS_ SIGMOT IS_ TILT IS_STEP_ DET 0 0 0IS_FSM_LC Interrupt status bit for FSM long counter timeout interrupt event. (1: interrupt detected; 0: no interrupt) IS_SIGMOT Interrupt status bit for significant motion detection (1: interrupt detected; 0: no interrupt) IS_TILT Interrupt status bit for tilt detection (1: interrupt detected; 0: no interrupt) IS_STEP_DET Interrupt status bit for step detection (1: interrupt detected; 0: no interrupt) 

Table 182. FSM_STATUS_A register 

IS_FSM8 IS_FSM7 IS_FSM6 IS_FSM5 IS_FSM4 IS_FSM3 IS_FSM2 IS_FSM1 IS_FSM8 Interrupt status bit for FSM8 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM7 Interrupt status bit for FSM7 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM6 Interrupt status bit for FSM6 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM5 Interrupt status bit for FSM5 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM4 Interrupt status bit for FSM4 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM3 Interrupt status bit for FSM3 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM2 Interrupt status bit for FSM2 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM1 Interrupt status bit for FSM1 interrupt event. (1: interrupt detected; 0: no interrupt) Embedded functions register description LSM6DSO32 

108/153 DocID032891 Rev 1 

# 11.14 FSM_STATUS_B (14h) 

Finite State Machine status register (r) 

Table 185. FSM_STATUS_B register description 11.15 PAGE_RW (17h) 

Enable read and write mode of advanced features dedicated page (r/w) 

Table 187. PAGE_RW register description Table 184. FSM_STATUS_B register 

IS_FSM16 IS_FSM15 IS_FSM14 IS_FSM13 IS_FSM12 IS_FSM11 IS_FSM10 IS_FSM9 IS_FSM16 Interrupt status bit for FSM16 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM15 Interrupt status bit for FSM15 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM14 Interrupt status bit for FSM14 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM13 Interrupt status bit for FSM13 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM12 Interrupt status bit for FSM12 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM11 Interrupt status bit for FSM11 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM10 Interrupt status bit for FSM10 interrupt event. (1: interrupt detected; 0: no interrupt) IS_FSM9 Interrupt status bit for FSM9 interrupt event. (1: interrupt detected; 0: no interrupt) 

Table 186. PAGE_RW register 

EMB_ FUNC_LIR PAGE_ WRITE PAGE_ READ 0(1) 1. This bit must be set to '0' for the correct operation of the device. 0 (1) 0(1) 0(1) 0 (1) EMB_FUNC_LIR Latched Interrupt mode for Embedded Functions. Default value: 0 (0: Embedded Functions interrupt request not latched; 1: Embedded Functions interrupt request latched) PAGE_WRITE Enable writes to the selected advanced features dedicated page (1) .Default value: 0 (1: enable; 0: disable) 1. Page selected by PAGE_SEL[3:0] in PAGE_SEL (02h) register. PAGE_READ Enable reads from the selected advanced features dedicated page (1) .Default value: 0 (1: enable; 0: disable) DocID032891 Rev 1 109/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.16 EMB_FUNC_FIFO_CFG (44h) 

Embedded functions batching configuration register (r/w) 

Table 189. EMB_FUNC_FIFO_CFG register description 11.17 FSM_ENABLE_A (46h) 

FSM enable register (r/w) 

Table 191. FSM_ENABLE_A register description Table 188. EMB_FUNC_FIFO_CFG register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. PEDO_ FIFO_EN 0(1) 0(1) 0(1) 0 (1) 0 (1) 0 (1) PEDO_FIFO_EN Enable FIFO batching of step counter values. Default value: 0 

Table 190. FSM_ENABLE_A register 

FSM8_EN FSM7_EN FSM6_EN FSM5_EN FSM4_EN FSM3_EN FSM2_EN FSM1_EN FSM8_EN FSM8 enable. Default value: 0 (0: FSM8 disabled; 1: FSM8 enabled) FSM7_EN FSM7 enable. Default value: 0 (0: FSM7 disabled; 1: FSM7 enabled) FSM6_EN FSM6 enable. Default value: 0 (0: FSM6 disabled; 1: FSM6 enabled) FSM5_EN FSM5 enable. Default value: 0 (0: FSM5 disabled; 1: FSM5 enabled) FSM4_EN FSM4 enable. Default value: 0 (0: FSM4 disabled; 1: FSM4 enabled) FSM3_EN FSM3 enable. Default value: 0 (0: FSM3 disabled; 1: FSM3 enabled) FSM2_EN FSM2 enable. Default value: 0 (0: FSM2 disabled; 1: FSM2 enabled) FSM1_EN FSM1 enable. Default value: 0 (0: FSM1 disabled; 1: FSM1 enabled) Embedded functions register description LSM6DSO32 

110/153 DocID032891 Rev 1 

# 11.18 FSM_ENABLE_B (47h) 

FSM enable register (r/w) 

Table 193. FSM_ENABLE_B register description 11.19 FSM_LONG_COUNTER_L (48h) and FSM_LONG_COUNTER_H (49h) 

FSM long counter status register (r/w) 

Long counter value is an unsigned integer value (16-bit format); this value can be reset using the LC_CLEAR bit in  FSM_LONG_COUNTER_CLEAR (4Ah)  register. 

Table 192. FSM_ENABLE_B register 

FSM16_EN FSM15_EN FSM14_EN FSM13_EN FSM12_EN FSM11_EN FSM10_EN FSM9_EN FSM16_EN FSM16 enable. Default value: 0 (0: FSM16 disabled; 1: FSM16 enabled) FSM15_EN FSM15 enable. Default value: 0 (0: FSM15 disabled; 1: FSM15 enabled) FSM14_EN FSM14 enable. Default value: 0 (0: FSM14 disabled; 1: FSM14 enabled) FSM13_EN FSM13 enable. Default value: 0 (0: FSM13 disabled; 1: FSM13 enabled) FSM12_EN FSM12 enable. Default value: 0 (0: FSM12 disabled; 1: FSM12 enabled) FSM11_EN FSM11 enable. Default value: 0 (0: FSM11 disabled; 1: FSM11 enabled) FSM10_EN FSM10 enable. Default value: 0 (0: FSM10 disabled; 1: FSM10 enabled) FSM9_EN FSM9 enable. Default value: 0 (0: FSM9 disabled; 1: FSM9 enabled) 

Table 194. FSM_LONG_COUNTER_L register 

FSM_LC_ 7FSM_LC_ 6FSM_LC_ 5FSM_LC_ 4FSM_LC_ 3FSM_LC_ 2FSM_LC_ 1FSM_LC_ 0

Table 195. FSM_LONG_COUNTER_L register description 

FSM_LC_[7:0] Long counter current value (LSbyte). Default value: 00000000 

Table 196. FSM_LONG_COUNTER_H register 

FSM_LC_ 15 FSM_LC_ 14 FSM_LC_ 13 FSM_LC_ 12 FSM_LC_ 11 FSM_LC_ 10 FSM_LC_ 9FSM_LC_ 8

Table 197. FSM_LONG_COUNTER_H register description 

FSM_LC_[15:8] Long counter current value (MSbyte). Default value: 00000000 DocID032891 Rev 1 111/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.20 FSM_LONG_COUNTER_CLEAR (4Ah) 

FSM long counter reset register (r/w) 

Table 199. FSM_LONG_COUNTER_CLEAR register description 11.21 FSM_OUTS1 (4Ch) 

FSM1 output register (r) 

Table 201. FSM_OUTS1 register description Table 198. FSM_LONG_COUNTER_CLEAR register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. 0 (1) 0(1) 0(1) 0(1) 0 (1) FSM_LC_ CLEARED FSM_LC_ CLEAR FSM_LC_ CLEARED This read-only bit is automatically set to 1 when the long counter reset is done. Default value: 0 FSM_LC_CLEAR Clear FSM long counter value. Default value: 0 

Table 200. FSM_OUTS1 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM1 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM1 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM1 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM1 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM1 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM1 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM1 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM1 output: negative event detected on the vector. (0: event not detected; 1: event detected) Embedded functions register description LSM6DSO32 

112/153 DocID032891 Rev 1 

# 11.22 FSM_OUTS2 (4Dh) 

FSM2 output register (r) 

Table 203. FSM_OUTS2 register description 11.23 FSM_OUTS3 (4Eh) 

FSM3 output register (r) 

Table 205. FSM_OUTS3 register description Table 202. FSM_OUTS2 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM2 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM2 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM2 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM2 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM2 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM2 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM2 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM2 output: negative event detected on the vector. (0: event not detected; 1: event detected) 

Table 204. FSM_OUTS3 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM3 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM3 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM3 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM3 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM3 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM3 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM3 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM3 output: negative event detected on the vector. (0: event not detected; 1: event detected) DocID032891 Rev 1 113/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.24 FSM_OUTS4 (4Fh) 

FSM4 output register (r) 

Table 207. FSM_OUTS4 register description 11.25 FSM_OUTS5 (50h) 

FSM5 output register (r) 

Table 209. FSM_OUTS5 register description Table 206. FSM_OUTS4 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM4 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM4 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM4 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM4 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM4 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM4 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM4 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM4 output: negative event detected on the vector. (0: event not detected; 1: event detected) 

Table 208. FSM_OUTS5 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM5 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM5 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM5 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM5 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM5 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM5 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM5 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM5 output: negative event detected on the vector. (0: event not detected; 1: event detected) Embedded functions register description LSM6DSO32 

114/153 DocID032891 Rev 1 

# 11.26 FSM_OUTS6 (51h) 

FSM6 output register (r) 

Table 211. FSM_OUTS6 register description 11.27 FSM_OUTS7 (52h) 

FSM7 output register (r) 

Table 213. FSM_OUTS7 register description Table 210. FSM_OUTS6 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM6 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM6 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM6 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM6 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM6 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM6 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM6 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM6 output: negative event detected on the vector. (0: event not detected; 1: event detected) 

Table 212. FSM_OUTS7 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM7 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM7 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM7 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM7 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM7 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM7 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM7 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM7 output: negative event detected on the vector. (0: event not detected; 1: event detected) DocID032891 Rev 1 115/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.28 FSM_OUTS8 (53h) 

FSM8 output register (r) 

Table 215. FSM_OUTS8 register description 11.29 FSM_OUTS9 (54h) 

FSM9 output register (r) 

Table 217. FSM_OUTS9 register description Table 214. FSM_OUTS8 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM8 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM8 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM8 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM8 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM8 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM8 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM8 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM8 output: negative event detected on the vector. (0: event not detected; 1: event detected) 

Table 216. FSM_OUTS9 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM9 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM9 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM9 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM9 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM9 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM9 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM9 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM9 output: negative event detected on the vector. (0: event not detected; 1: event detected) Embedded functions register description LSM6DSO32 

116/153 DocID032891 Rev 1 

# 11.30 FSM_OUTS10 (55h) 

FSM10 output register (r) 

Table 219. FSM_OUTS10 register description 11.31 FSM_OUTS11 (56h) 

FSM11 output register (r) 

Table 221. FSM_OUTS11 register description Table 218. FSM_OUTS10 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM10 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM10 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM10 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM10 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM10 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM10 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM10 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM10 output: negative event detected on the vector. (0: event not detected; 1: event detected) 

Table 220. FSM_OUTS11 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM11 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM11 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM11 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM11 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM11 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM11 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM11 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM11 output: negative event detected on the vector. (0: event not detected; 1: event detected) DocID032891 Rev 1 117/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.32 FSM_OUTS12 (57h) 

FSM12 output register (r) 

Table 223. FSM_OUTS12 register description 11.33 FSM_OUTS13 (58h) 

FSM13 output register (r) 

Table 225. FSM_OUTS13 register description Table 222. FSM_OUTS12 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM12 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM12 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM12 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM12 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM12 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM12 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM12 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM12 output: negative event detected on the vector. (0: event not detected; 1: event detected) 

Table 224. FSM_OUTS13 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM13 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM13 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM13 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM13 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM13 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM13 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM13 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM13 output: negative event detected on the vector. (0: event not detected; 1: event detected) Embedded functions register description LSM6DSO32 

118/153 DocID032891 Rev 1 

# 11.34 FSM_OUTS14 (59h) 

FSM14 output register (r) 

Table 227. FSM_OUTS14 register description 11.35 FSM_OUTS15 (5Ah) 

FSM15 output register (r) 

Table 229. FSM_OUTS15 register description Table 226. FSM_OUTS14 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM14 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM14 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM14 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM14 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM14 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM14 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM14 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM14 output: negative event detected on the vector. (0: event not detected; 1: event detected) 

Table 228. FSM_OUTS15 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM15 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM15 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM15 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM15 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM15 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM15 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM15 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM15 output: negative event detected on the vector. (0: event not detected; 1: event detected) DocID032891 Rev 1 119/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.36 FSM_OUTS16 (5Bh) 

FSM16 output register (r) 

Table 231. FSM_OUTS16 register description 11.37 EMB_FUNC_ODR_CFG_B (5Fh) 

Finite State Machine output data rate configuration register (r/w) 

Table 233. EMB_FUNC_ODR_CFG_B register description Table 230. FSM_OUTS16 register 

P_X N_X P_Y N_Y P_Z N_Z P_V N_V P_X FSM16 output: positive event detected on the X-axis. (0: event not detected; 1: event detected) N_X FSM16 output: negative event detected on the X-axis. (0: event not detected; 1: event detected) P_Y FSM16 output: positive event detected on the Y-axis. (0: event not detected; 1: event detected) N_Y FSM16 output: negative event detected on the Y-axis. (0: event not detected; 1: event detected) P_Z FSM16 output: positive event detected on the Z-axis. (0: event not detected; 1: event detected) N_Z FSM16 output: negative event detected on the Z-axis. (0: event not detected; 1: event detected) P_V FSM16 output: positive event detected on the vector. (0: event not detected; 1: event detected N_V FSM16 output: negative event detected on the vector. (0: event not detected; 1: event detected) 

Table 232. EMB_FUNC_ODR_CFG_B register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. 1 (2) 2. This bit must be set to '1' for the correct operation of the device. 0(1) FSM_ ODR1 FSM_ ODR0 0 (1) 1 (2) 1 (2) FSM_ODR[1:0] Finite State Machine ODR configuration: (00: 12.5 Hz; 01: 26 Hz (default); 10: 52 Hz; 11: 104 Hz) Embedded functions register description LSM6DSO32 

120/153 DocID032891 Rev 1 

# 11.38 STEP_COUNTER_L (62h) and STEP_COUNTER_H (63h) 

Step counter output register (r) 

# 11.39 EMB_FUNC_SRC (64h) 

Embedded function source register (r) 

Table 239. EMB_FUNC_SRC register description 

Table 234. STEP_COUNTER_L register 

STEP_7 STEP_6 STEP_5 STEP_4 STEP_3 STEP_2 STEP_1 STEP_0 

Table 235. STEP_COUNTER_L register description 

STEP_[7:0] Step counter output (LSbyte) 

Table 236. STEP_COUNTER_H register 

STEP_15 STEP_14 STEP_13 STEP_12 STEP_11 STEP_10 STEP_9 STEP_8 

Table 237. STEP_COUNTER_H register description 

STEP_[15:8] Step counter output (MSbyte) 

Table 238. EMB_FUNC_SRC register 

PEDO_ RST_ STEP 0 STEP_ DETECTED STEP_ COUNT_ DELTA_IA STEP_ OVERFLOW STEPCOU NTER_BIT _SET 0 0PEDO_RST_ STEP Reset pedometer step counter. Read/write bit. (0: disabled; 1: enabled) STEP_ DETECTED Step detector event detection status. Read-only bit. (0: step detection event not detected; 1: step detection event detected) STEP_COUNT_ DELTA_IA Pedometer step recognition on delta time status. Read-only bit. (0: no step recognized during delta time; 1: at least one step recognized during delta time) STEP_ OVERFLOW Step counter overflow status. Read-only bit. (0: step counter value < 2 16 ; 1: step counter value reached 2 16 )STEPCOUNTER_ BIT_SET This bit is equal to 1 when the step count is increased. If a timer period is programmed in PEDO_SC_DELTAT_L (D0h) and PEDO_SC_DELTAT_H (D1h) 

embedded advanced features (page 1) registers, this bit is kept to 0. Read-only bit. DocID032891 Rev 1 121/153 

LSM6DSO32 Embedded functions register description 

> 153

# 11.40 EMB_FUNC_INIT_A (66h) 

Embedded functions initialization register (r/w) 

Table 241. EMB_FUNC_INIT_A register description 11.41 EMB_FUNC_INIT_B (67h) 

Embedded functions initialization register (r/w) 

Table 243. EMB_FUNC_INIT_B register description 

Table 240. EMB_FUNC_INIT_A register 

0(1) 1. This bit must be set to '0' for the correct operation of the device. 0(1) SIG_MOT _INIT TILT _INIT STEP_DET _INIT 0(1) 0(1) 0 (1) SIG_MOT_INIT Significant Motion Detection algorithm initialization request. Default value: 0 TILT_INIT Tilt algorithm initialization request. Default value: 0 STEP_DET_INIT Pedometer Step Counter/Detector algorithm initialization request. Default value: 0 

Table 242. EMB_FUNC_INIT_B register 

0 (1) 1. This bit must be set to '0' for the correct operation of the device. 0 (1) 0(1) 0(1) FIFO_ COMPR_ INIT 0 (1) 0 (1) FSM_INIT FIFO_COMPR_INIT FIFO compression feature initialization request. Default value: 0 FSM_INIT FSM initialization request. Default value: 0 Embedded advanced features pages LSM6DSO32 

122/153 DocID032891 Rev 1 

# 12 Embedded advanced features pages 

The table given below provides a list of the registers for the embedded advanced features page 0. These registers are accessible when PAGE_SEL[3:0] are set to 0000 in  PAGE_SEL 

(02h) .

Table 244. Register address map - embedded advanced features page 0 Name Type Register address Default Comment Hex Binary 

MAG_SENSITIVITY_L r/w BA 10111010 00100100 MAG_SENSITIVITY_H r/w BB 10111011 00010110 MAG_OFFX_L r/w C0 11000000 00000000 MAG_OFFX_H r/w C1 11000001 00000000 MAG_OFFY_L r/w C2 11000010 00000000 MAG_OFFY_H r/w C3 11000011 00000000 MAG_OFFZ_L r/w C4 11000100 00000000 MAG_OFFZ_H r/w C5 11000101 00000000 MAG_SI_XX_L r/w C6 11000110 00000000 MAG_SI_XX_H r/w C7 11000111 00111100 MAG_SI_XY_L r/w C8 11001000 00000000 MAG_SI_XY_H r/w C9 11001001 00000000 MAG_SI_XZ_L r/w CA 11001010 00000000 MAG_SI_XZ_H r/w CB 11001011 00000000 MAG_SI_YY_L r/w CC 11001100 00000000 MAG_SI_YY_H r/w CD 11001101 00111100 MAG_SI_YZ_L r/w CE 11001110 00000000 MAG_SI_YZ_H r/w CF 11001111 00000000 MAG_SI_ZZ_L r/w D0 11010000 00000000 MAG_SI_ZZ_H r/w D1 11010001 00111100 MAG_CFG_A r/w D4 11010100 00000101 MAG_CFG_B r/w D5 11010101 00000010 DocID032891 Rev 1 123/153 

LSM6DSO32 Embedded advanced features pages 

> 153

The table given below provides a list of the registers for the embedded advanced features page 1. These registers are accessible when PAGE_SEL[3:0] are set to 0001 in 

PAGE_SEL (02h) .

Registers marked as Reserved must not be changed. Writing to those registers may cause permanent damage to the device. 

The content of the registers that are loaded at boot should not be changed. They contain the factory calibration values. Their content is automatically restored when the device is powered up. 

Write procedure example: 

Example: write value 06h register at address 84h (PEDO_DEB_STEPS_CONF) in Page 1 

1.  Write bit FUNC_CFG_EN = 1 

in FUNC_CFG_ACCESS (01h) 

// Enable access to embedded functions registers 

2.  Write bit PAGE_WRITE = 1 

in PAGE_RW (17h) register 

// Select write operation mode 

3.  Write 0001 in PAGE_SEL[3:0] field 

of register PAGE_SEL (02h) 

// Select page 1 

4.  Write 84h in PAGE_ADDR register (08h)  // Set address 

5.  Write 06h in PAGE_DATA register (09h)  // Set value to be written 

6.  Write bit PAGE_WRITE = 0 

in PAGE_RW (17h) register 

// Write operation disabled 

7.  Write bit FUNC_CFG_EN = 0 in FUNC_CFG_ACCESS (01h) 

// Disable access to embedded functions registers 

Table 245. Register address map - embedded advanced features page 1 Name Type Register address Default Comment Hex Binary 

FSM_LC_TIMEOUT_L r/w 7A 01111010 00000000 FSM_LC_TIMEOUT_H r/w 7B 01111011 00000000 FSM_PROGRAMS r/w 7C 01111100 00000000 FSM_START_ADD_L r/w 7E 01111110 00000000 FSM_START_ADD_H r/w 7F 01111111 00000000 PEDO_CMD_REG r/w 83 10000011 00000000 PEDO_DEB_STEPS_CONF r/w 84 10000100 00001010 PEDO_SC_DELTAT_L r/w D0 11010000 00000000 PEDO_SC_DELTAT_H r/w D1 11010001 00000000 Embedded advanced features pages LSM6DSO32 

124/153 DocID032891 Rev 1 

Read procedure example: 

Example: read value of register at address 84h (PEDO_DEB_STEPS_CONF) in Page 1 

1.  Write bit FUNC_CFG_EN = 1 

in FUNC_CFG_ACCESS (01h) 

// Enable access to embedded functions registers 

2.  Write bit PAGE_READ = 1 

in PAGE_RW (17h) register 

// Select read operation mode 

3.  Write 0001 in PAGE_SEL[3:0] field 

of register PAGE_SEL (02h) 

// Select page 1 

4.  Write 84h in PAGE_ADDR register (08h)  // Set address 

5.  Read value of PAGE_DATA register (09h)  // Get register value 

6.  Write bit PAGE_READ = 0 

in PAGE_RW (17h) register 

// Read operation disabled 

7.  Write bit FUNC_CFG_EN = 0 in FUNC_CFG_ACCESS (01h) 

// Disable access to embedded functions registers 

Note: Steps 1 and 2 of both procedures are intended to be performed at the beginning of the procedure. Steps 6 and 7 of both procedures are intended to be performed at the end of the procedure. If the procedure involves multiple operations, only steps 3, 4 and 5 must be repeated for each operation. If, in particular, the multiple operations involve consecutive registers, only step 5 can be performed. DocID032891 Rev 1 125/153 

LSM6DSO32 Embedded advanced features register description 

> 153

# 13 Embedded advanced features register description 13.1 Page 0 - Embedded advanced features registers 13.1.1 MAG_SENSITIVITY_L (BAh) and MAG_SENSITIVITY_H (BBh) 

External magnetometer sensitivity value register for the Finite State Machine (r/w). 

This register corresponds to the LSB-to-gauss conversion value of the external magnetometer sensor. The register value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

Default value of MAG_SENS[15:0] is 0x1624, corresponding to 0.0015 gauss/LSB. 

## 13.1.2 MAG_OFFX_L (C0h) and MAG_OFFX_H (C1h) 

Offset for X-axis hard-iron compensation register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

Table 246. MAG_SENSITIVITY_L register 

MAG_ SENS_7 MAG_ SENS_6 MAG_ SENS_5 MAG_ SENS_4 MAG_ SENS_3 MAG_ SENS_2 MAG_ SENS_1 MAG_ SENS_0 

Table 247. MAG_SENSITIVITY_L register description 

MAG_SENS_[7:0] External magnetometer sensitivity (LSbyte). Default value: 00100100 

Table 248. MAG_SENSITIVITY_H register 

MAG_ SENS_15 MAG_ SENS_14 MAG_ SENS_13 MAG_ SENS_12 MAG_ SENS_11 MAG_ SENS_10 MAG_ SENS_9 MAG_ SENS_8 

Table 249. MAG_SENSITIVITY_H register description 

MAG_SENS_[15:8] External magnetometer sensitivity (MSbyte). Default value: 00010110 

Table 250. MAG_OFFX_L register 

MAG_OFF X_7 MAG_OFF X_6 MAG_OFF X_5 MAG_OFF X_4 MAG_OFF X_3 MAG_OFF X_2 MAG_OFF X_1 MAG_OFF X_0 

Table 251. MAG_OFFX_L register description 

MAG_OFFX_[7:0] Offset for X-axis hard-iron compensation (LSbyte). Default value: 00000000 

Table 252. MAG_OFFX_H register 

MAG_OFF X_15 MAG_OFF X_14 MAG_OFF X_13 MAG_OFF X_12 MAG_OFF X_11 MAG_OFF X_10 MAG_OFF X_9 MAG_OFF X_8 

Table 253. MAG_OFFX_H register description 

MAG_OFFX_[15:8] Offset for X-axis hard-iron compensation (MSbyte). Default value: 00000000 Embedded advanced features register description LSM6DSO32 

126/153 DocID032891 Rev 1 

## 13.1.3 MAG_OFFY_L (C2h) and MAG_OFFY_H (C3h) 

Offset for Y-axis hard-iron compensation register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

## 13.1.4 MAG_OFFZ_L (C4h) and MAG_OFFZ_H (C5h) 

Offset for Z-axis hard-iron compensation register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

Table 254. MAG_OFFY_L register 

MAG_OFF Y_7 MAG_OFF Y_6 MAG_OFF Y_5 MAG_OFF Y_4 MAG_OFF Y_3 MAG_OFF Y_2 MAG_OFF Y_1 MAG_OFF Y_0 

Table 255. MAG_OFFY_L register description 

MAG_OFFY_[7:0] Offset for Y-axis hard-iron compensation (LSbyte). Default value: 00000000 

Table 256. MAG_OFFY_H register 

MAG_OFF Y_15 MAG_OFF Y_14 MAG_OFF Y_13 MAG_OFF Y_12 MAG_OFF Y_11 MAG_OFF Y_10 MAG_OFF Y_9 MAG_OFF Y_8 

Table 257. MAG_OFFY_H register description 

MAG_OFFY_[15:8] Offset for Y-axis hard-iron compensation (MSbyte). Default value: 00000000 

Table 258. MAG_OFFZ_L register 

MAG_OFF Z_7 MAG_OFF Z_6 MAG_OFF Z_5 MAG_OFF Z_4 MAG_OFF Z_3 MAG_OFF Z_2 MAG_OFF Z_1 MAG_OFF Z_0 

Table 259. MAG_OFFZ_L register description 

MAG_OFFZ_[7:0] Offset for Z-axis hard-iron compensation (LSbyte). Default value: 00000000 

Table 260. MAG_OFFZ_H register 

MAG_OFF Z_15 MAG_OFF Z_14 MAG_OFF Z_13 MAG_OFF Z_12 MAG_OFF Z_11 MAG_OFF Z_10 MAG_OFF Z_9 MAG_OFF Z_8 

Table 261. MAG_OFFZ_H register description 

MAG_OFFZ_[15:8] Offset for Z-axis hard-iron compensation (MSbyte). Default value: 00000000 DocID032891 Rev 1 127/153 

LSM6DSO32 Embedded advanced features register description 

> 153

## 13.1.5 MAG_SI_XX_L (C6h) and MAG_SI_XX_H (C7h) 

Soft-iron (3x3 symmetric) matrix correction register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

## 13.1.6 MAG_SI_XY_L (C8h) and MAG_SI_XY_H (C9h) 

Soft-iron (3x3 symmetric) matrix correction register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

Table 262. MAG_SI_XX_L register 

MAG_SI_ XX_7 MAG_SI_ XX_6 MAG_SI_ XX_5 MAG_SI_ XX_4 MAG_SI_ XX_3 MAG_SI_ XX_2 MAG_SI_ XX_1 MAG_SI_ XX_0 

Table 263. MAG_SI_XX_L register description 

MAG_SI_XX_[7:0] Soft-iron correction row1 col1 coefficient (LSbyte). Default value: 00000000 

Table 264. MAG_SI_XX_H register 

MAG_SI_ XX_15 MAG_SI_ XX_14 MAG_SI_ XX_13 MAG_SI_ XX_12 MAG_SI_ XX_11 MAG_SI_ XX_10 MAG_SI_ XX_9 MAG_SI_ XX_8 

Table 265. MAG_SI_XX_H register description 

MAG_SI_XX_[15:8] Soft-iron correction row1 col1 coefficient (MSbyte). Default value: 00111100 

Table 266. MAG_SI_XY_L register 

MAG_SI_ XY_7 MAG_SI_ XY_6 MAG_SI_ XY_5 MAG_SI_ XY_4 MAG_SI_ XY_3 MAG_SI_ XY_2 MAG_SI_ XY_1 MAG_SI_ XY_0 

Table 267. MAG_SI_XY_L register description 

MAG_SI_XY_[7:0] Soft-iron correction row1 col2 (and row2 col1) coefficient (LSbyte). Default value: 00000000 

Table 268. MAG_SI_XY_H register 

MAG_SI_ XY_15 MAG_SI_ XY_14 MAG_SI_ XY_13 MAG_SI_ XY_12 MAG_SI_ XY_11 MAG_SI_ XY_10 MAG_SI_ XY_9 MAG_SI_ XY_8 

Table 269. MAG_SI_XY_H register description 

MAG_SI_XY_[15:8] Soft-iron correction row1 col2 (and row2 col1) coefficient (MSbyte). Default value: 00000000 Embedded advanced features register description LSM6DSO32 

128/153 DocID032891 Rev 1 

## 13.1.7 MAG_SI_XZ_L (CAh) and MAG_SI_XZ_H (CBh) 

Soft-iron (3x3 symmetric) matrix correction register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

## 13.1.8 MAG_SI_YY_L (CCh) and MAG_SI_YY_H (CDh) 

Soft-iron (3x3 symmetric) matrix correction register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

Table 270. MAG_SI_XZ_L register 

MAG_SI_ XZ_7 MAG_SI_ XZ_6 MAG_SI_ XZ_5 MAG_SI_ XZ_4 MAG_SI_ XZ_3 MAG_SI_ XZ_2 MAG_SI_ XZ_1 MAG_SI_ XZ_0 

Table 271. MAG_SI_XZ_L register description 

MAG_SI_XZ_[7:0] Soft-iron correction row1 col3 (and row3 col1) coefficient (LSbyte). Default value: 00000000 

Table 272. MAG_SI_XZ_H register 

MAG_SI_ XZ_15 MAG_SI_ XZ_14 MAG_SI_ XZ_13 MAG_SI_ XZ_12 MAG_SI_ XZ_11 MAG_SI_ XZ_10 MAG_SI_ XZ_9 MAG_SI_ XZ_8 

Table 273. MAG_SI_XZ_H register description 

MAG_SI_XZ_[15:8] Soft-iron correction row1 col3 (and row3 col1) coefficient (MSbyte). Default value: 00000000 

Table 274. MAG_SI_YY_L register 

MAG_SI_ YY_7 MAG_SI_ YY_6 MAG_SI_ YY_5 MAG_SI_ YY_4 MAG_SI_ YY_3 MAG_SI_ YY_2 MAG_SI_ YY_1 MAG_SI_ YY_0 

Table 275. MAG_SI_YY_L register description 

MAG_SI_YY_[7:0] Soft-iron correction row2 col2 coefficient (LSbyte). Default value: 00000000 

Table 276. MAG_SI_YY_H register 

MAG_SI_ YY_15 MAG_SI_ YY_14 MAG_SI_ YY_13 MAG_SI_ YY_12 MAG_SI_ YY_11 MAG_SI_ YY_10 MAG_SI_ YY_9 MAG_SI_ YY_8 

Table 277. MAG_SI_YY_H register description 

MAG_SI_YY_[15:8] Soft-iron correction row2 col2 coefficient (MSbyte). Default value: 00111100 DocID032891 Rev 1 129/153 

LSM6DSO32 Embedded advanced features register description 

> 153

## 13.1.9 MAG_SI_YZ_L (CEh) and MAG_SI_YZ_H (CFh) 

Soft-iron (3x3 symmetric) matrix correction register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

## 13.1.10 MAG_SI_ZZ_L (D0h) and MAG_SI_ZZ_H (D1h) 

Soft-iron (3x3 symmetric) matrix correction register (r/w). 

The value is expressed as half-precision floating-point format: SEEEEEFFFFFFFFFF 

(S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits). 

Table 278. MAG_SI_YZ_L register 

MAG_SI_ YZ_7 MAG_SI_ YZ_6 MAG_SI_ YZ_5 MAG_SI_ YZ_4 MAG_SI_ YZ_3 MAG_SI_ YZ_2 MAG_SI_ YZ_1 MAG_SI_ YZ_0 

Table 279. MAG_SI_YZ_L register description 

MAG_SI_YZ_[7:0] Soft-iron correction row2 col3 (and row3 col2) coefficient (LSbyte). Default value: 00000000 

Table 280. MAG_SI_YZ_H register 

MAG_SI_ YZ_15 MAG_SI_ YZ_14 MAG_SI_ YZ_13 MAG_SI_ YZ_12 MAG_SI_ YZ_11 MAG_SI_ YZ_10 MAG_SI_ YZ_9 MAG_SI_ YZ_8 

Table 281. MAG_SI_YZ_H register description 

MAG_SI_YZ_[15:8] Soft-iron correction row2 col3 (and row3 col2) coefficient (MSbyte). Default value: 00000000 

Table 282. MAG_SI_ZZ_L register 

MAG_SI_ ZZ_7 MAG_SI_ ZZ_6 MAG_SI_ ZZ_5 MAG_SI_ ZZ_4 MAG_SI_ ZZ_3 MAG_SI_ ZZ_2 MAG_SI_ ZZ_1 MAG_SI_ ZZ_0 

Table 283. MAG_SI_ZZ_L register description 

MAG_SI_ZZ_[7:0] Soft-iron correction row3 col3 coefficient (LSbyte). Default value: 00000000 

Table 284. MAG_SI_ZZ_H register 

MAG_SI_ ZZ_15 MAG_SI_ ZZ_14 MAG_SI_ ZZ_13 MAG_SI_ ZZ_12 MAG_SI_ ZZ_11 MAG_SI_ ZZ_10 MAG_SI_ ZZ_9 MAG_SI_ ZZ_8 

Table 285. MAG_SI_ZZ_H register description 

MAG_SI_ZZ_[15:8] Soft-iron correction row3 col3 coefficient (MSbyte). Default value: 00111100 Embedded advanced features register description LSM6DSO32 

130/153 DocID032891 Rev 1 

## 13.1.11 MAG_CFG_A (D4h) 

External magnetometer coordinates (Z and Y axes) rotation register (r/w) 

## 13.1.12 MAG_CFG_B (D5h) 

External magnetometer coordinates (X-axis) rotation register (r/w) 

Table 286. MAG_CFG_A register 

0(1) 1. This bit must be set to ‘0’ for the correct operation of the device. MAG_Y_ AXIS2 MAG_Y_ AXIS1 MAG_Y_ AXIS0 0 (1) MAG_Z_ AXIS2 MAG_Z_ AXIS1 MAG_Z_ AXIS0 

Table 287. MAG_CFG_A description 

MAG_Y_AXIS[2:0] Magnetometer Y-axis coordinates rotation (to be aligned to accelerometer/gyroscope axes orientation) (000: Y = Y; (default) 001: Y = -Y; 010: Y = X; 011: Y = -X; 100: Y = -Z; 101: Y = Z; Others: Y = Y) MAG_Z_AXIS[2:0] Magnetometer Z-axis coordinates rotation (to be aligned to accelerometer/gyroscope axes orientation) (000: Z = Y; 001: Z = -Y; 010: Z = X; 011: Z = -X; 100: Z = -Z; 101: Z = Z; (default) Others: Z = Y) 

Table 288. MAG_CFG_B register 

0(1) 1. This bit must be set to ‘0’ for the correct operation of the device. 0(1) 0(1) 0 (1) 0 (1) MAG_X_ AXIS2 MAG_X_ AXIS1 MAG_X_ AXIS0 

Table 289. MAG_CFG_B description 

MAG_X_AXIS[2:0] Magnetometer X-axis coordinates rotation (to be aligned to accelerometer/gyroscope axes orientation) (000: X = Y; 001: X = -Y; 010: X = X; (default) 011: X = -X; 100: X = -Z; 101: X = Z; Others: X = Y) DocID032891 Rev 1 131/153 

LSM6DSO32 Embedded advanced features register description 

> 153

# 13.2 Page 1 - Embedded advanced features registers 13.2.1 FSM_LC_TIMEOUT_L (7Ah) and FSM_LC_TIMEOUT_H (7Bh) 

FSM long counter timeout register (r/w). 

The long counter timeout value is an unsigned integer value (16-bit format). When the long counter value reached this value, the FSM generates an interrupt. 

## 13.2.2 FSM_PROGRAMS (7Ch) 

FSM number of programs register (r/w) 

Table 290. FSM_LC_TIMEOUT_L register 

FSM_LC_ TIMEOUT 7FSM_LC_ TIMEOUT 6FSM_LC_ TIMEOUT 5FSM_LC_ TIMEOUT 4FSM_LC_ TIMEOUT 3FSM_LC_ TIMEOUT 2FSM_LC_ TIMEOUT 1FSM_LC_ TIMEOUT 0

Table 291. FSM_LC_TIMEOUT_L register description 

FSM_LC_ TIMEOUT[7:0] FSM long counter timeout value (LSbyte). Default value: 00000000 

Table 292. FSM_LC_TIMEOUT_H register 

FSM_LC_ TIMEOUT 15 FSM_LC_ TIMEOUT 14 FSM_LC_ TIMEOUT 13 FSM_LC_ TIMEOUT 12 FSM_LC_ TIMEOUT 11 FSM_LC_ TIMEOUT 10 FSM_LC_ TIMEOUT 9FSM_LC_ TIMEOUT 8

Table 293. FSM_LC_TIMEOUT_H register description 

FSM_LC_ TIMEOUT[15:8] FSM long counter timeout value (MSbyte). Default value: 00000000 

Table 294. FSM_PROGRAMS register 

FSM_N_ PROG7 FSM_N_ PROG6 FSM_N_ PROG5 FSM_N_ PROG4 FSM_N_ PROG3 FSM_N_ PROG2 FSM_N_ PROG1 FSM_N_ PROG0 

Table 295. FSM_PROGRAMS register description 

FSM_N_PROG[7:0] Number of FSM programs; must be less than or equal to 16. Default value: 00000000 Embedded advanced features register description LSM6DSO32 

132/153 DocID032891 Rev 1 

## 13.2.3 FSM_START_ADD_L (7Eh) and FSM_START_ADD_H (7Fh) 

FSM start address register (r/w). First available address is 0x033C. 

## 13.2.4 PEDO_CMD_REG (83h) 

Pedometer configuration register (r/w) 

Table 301. PEDO_CMD_REG register description 13.2.5 PEDO_DEB_STEPS_CONF (84h) 

Pedometer debounce configuration register (r/w) 

Table 303. PEDO_DEB_STEPS_CONF register description Table 296. FSM_START_ADD_L register 

FSM_ START7 FSM_ START6 FSM_ START5 FSM_ START4 FSM_ START3 FSM_ START2 FSM_ START1 FSM_ START0 

Table 297. FSM_START_ADD_L register description 

FSM_START[7:0] FSM start address value (LSbyte). Default value: 00000000 

Table 298. FSM_START_ADD_H register 

FSM_ START15 FSM_ START14 FSM_ START13 FSM_ START12 FSM_ START11 FSM_ START10 FSM_ START9 FSM_ START8 

Table 299. FSM_START_ADD_H register description 

FSM_START[15:8] FSM start address value (MSbyte). Default value: 00000000 

Table 300. PEDO_CMD_REG register 

0(1) 1. This bit must be set to '0' for the correct operation of the device. 0(1) 0 (1) 0 (1) CARRY_ COUNT_EN FP_ REJECTION_ EN 0(1) AD_ DET_EN CARRY_COUNT_EN Set when user wants to generate interrupt only on count overflow event. FP_REJECTION_EN (1) 1. This bit is effective if the PEDO_ADV_EN bit of EMB_FUNC_EN_B (05h) is set to 1. Enables the false-positive rejection feature. AD_DET_EN (2) 2. This bit is effective if both the FP_REJECTION_EN bit in PEDO_CMD_REG (83h) register and the PEDO_ADV_EN bit of EMB_FUNC_EN_B (05h) are set to 1. Enables the advanced detection feature. 

Table 302. PEDO_DEB_STEPS_CONF register 

DEB_ STEP7 DEB_ STEP6 DEB_ STEP5 DEB_ STEP4 DEB_ STEP3 DEB_ STEP2 DEB_ STEP1 DEB_ STEP0 DEB_STEP[7:0] Debounce threshold. Minimum number of steps to increment the step counter (debounce). Default value: 00001010 DocID032891 Rev 1 133/153 

LSM6DSO32 Embedded advanced features register description 

> 153

## 13.2.6 PEDO_SC_DELTAT_L (D0h) and PEDO_SC_DELTAT_H (D1h) 

Time period register for step detection on delta time (r/w) 

Table 304. PEDO_SC_DELTAT_L register        

> PD_SC_7 PD_SC_6 PD_SC_5 PD_SC_4 PD_SC_3 PD_SC_2 PD_SC_1 PD_SC_0

Table 305. PEDO_SC_DELTAT_H register        

> PD_SC_15 PD_SC_14 PD_SC_13 PD_SC_12 PD_SC_11 PD_SC_10 PD_SC_9 PD_SC_8

Table 306. PEDO_SC_DELTAT_H/L register description 

PD_SC_[15:0] Time period value (1LSB = 6.4 ms) Sensor hub register mapping LSM6DSO32 

134/153 DocID032891 Rev 1 

# 14 Sensor hub register mapping 

The table given below provides a list of the registers for the sensor hub functions available in the device and the corresponding addresses. The sensor hub registers are accessible when bit SHUB_REG_ACCESS is set to '1' in  FUNC_CFG_ACCESS (01h) .

Table 307. Register address map - sensor hub registers Name Type Register address Default Comment Hex Binary 

SENSOR_HUB_1 r 02 00000010 output SENSOR_HUB_2 r 03 00000011 output SENSOR_HUB_3 r 04 00000100 output SENSOR_HUB_4 r 05 00000101 output SENSOR_HUB_5 r 06 00000110 output SENSOR_HUB_6 r 07 00000111 output SENSOR_HUB_7 r 08 00001000 output SENSOR_HUB_8 r 09 00001001 output SENSOR_HUB_9 r 0A 00001010 output SENSOR_HUB_10 r 0B 00001011 output SENSOR_HUB_11 r 0C 00001100 output SENSOR_HUB_12 r 0D 00001101 output SENSOR_HUB_13 r 0E 00001110 output SENSOR_HUB_14 r 0F 00001111 output SENSOR_HUB_15 r 10 00010000 output SENSOR_HUB_16 r 11 00010001 output SENSOR_HUB_17 r 12 00010010 output SENSOR_HUB_18 r 13 00010011 output MASTER_CONFIG rw 14 00010100 00000000 SLV0_ADD rw 15 00010101 00000000 SLV0_SUBADD rw 16 00010110 00000000 SLV0_CONFIG rw 17 0001 0111 00000000 SLV1_ADD rw 18 00011000 00000000 SLV1_SUBADD rw 19 00011001 00000000 SLV1_CONFIG rw 1A 00011010 00000000 SLV2_ADD rw 1B 00011011 00000000 SLV2_SUBADD rw 1C 00011100 00000000 SLV2_CONFIG rw 1D 00011101 00000000 DocID032891 Rev 1 135/153 

LSM6DSO32 Sensor hub register mapping 

> 153

Registers marked as Reserved must not be changed. Writing to those registers may cause permanent damage to the device. 

The content of the registers that are loaded at boot should not be changed. They contain the factory calibration values. Their content is automatically restored when the device is powered up.                     

> SLV3_ADD rw 1E 00011110 00000000 SLV3_SUBADD rw 1F 00011111 00000000 SLV3_CONFIG rw 20 00100000 00000000 DATAWRITE_SLV0 rw 21 00100001 00000000 STATUS_MASTER r22 00100010 output

Table 307. Register address map - sensor hub registers Name Type Register address Default Comment Hex Binary Sensor hub register description LSM6DSO32 

136/153 DocID032891 Rev 1 

# 15 Sensor hub register description 15.1 SENSOR_HUB_1 (02h) 

Sensor hub output register (r) 

First byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from x = 0 to x = 3). 

# 15.2 SENSOR_HUB_2 (03h) 

Sensor hub output register (r) 

Second byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

# 15.3 SENSOR_HUB_3 (04h) 

Sensor hub output register (r) 

Third byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from x = 0 to x = 3). 

Table 308. SENSOR_HUB_1 register 

Sensor Hub1_7 Sensor Hub1_6 Sensor Hub1_5 Sensor Hub1_4 Sensor Hub1_3 Sensor Hub1_2 Sensor Hub1_1 Sensor Hub1_0 

Table 309. SENSOR_HUB_1 register description 

SensorHub1[7:0] First byte associated to external sensors 

Table 310. SENSOR_HUB_2 register 

Sensor Hub2_7 Sensor Hub2_6 Sensor Hub2_5 Sensor Hub2_4 Sensor Hub2_3 Sensor Hub2_2 Sensor Hub2_1 Sensor Hub2_0 

Table 311. SENSOR_HUB_2 register description 

SensorHub2[7:0] Second byte associated to external sensors 

Table 312. SENSOR_HUB_3 register 

Sensor Hub3_7 Sensor Hub3_6 Sensor Hub3_5 Sensor Hub3_4 Sensor Hub3_3 Sensor Hub3_2 Sensor Hub3_1 Sensor Hub3_0 

Table 313. SENSOR_HUB_3 register description 

SensorHub3[7:0] Third byte associated to external sensors DocID032891 Rev 1 137/153 

LSM6DSO32 Sensor hub register description 

> 153

# 15.4 SENSOR_HUB_4 (05h) 

Sensor hub output register (r) 

Fourth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from x = 0 to x = 3). 

# 15.5 SENSOR_HUB_5 (06h) 

Sensor hub output register (r) 

Fifth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from x = 0 to x = 3). 

# 15.6 SENSOR_HUB_6 (07h) 

Sensor hub output register (r) 

Sixth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from x = 0 to x = 3). 

Table 314. SENSOR_HUB_4 register 

Sensor Hub4_7 Sensor Hub4_6 Sensor Hub4_5 Sensor Hub4_4 Sensor Hub4_3 Sensor Hub4_2 Sensor Hub4_1 Sensor Hub4_0 

Table 315. SENSOR_HUB_4 register description 

SensorHub4[7:0] Fourth byte associated to external sensors 

Table 316. SENSOR_HUB_5 register 

Sensor Hub5_7 Sensor Hub5_6 Sensor Hub5_5 Sensor Hub5_4 Sensor Hub5_3 Sensor Hub5_2 Sensor Hub5_1 Sensor Hub5_0 

Table 317. SENSOR_HUB_5 register description 

SensorHub5[7:0] Fifth byte associated to external sensors 

Table 318. SENSOR_HUB_6 register 

Sensor Hub6_7 Sensor Hub6_6 Sensor Hub6_5 Sensor Hub6_4 Sensor Hub6_3 Sensor Hub6_2 Sensor Hub6_1 Sensor Hub6_0 

Table 319. SENSOR_HUB_6 register description 

SensorHub6[7:0] Sixth byte associated to external sensors Sensor hub register description LSM6DSO32 

138/153 DocID032891 Rev 1 

# 15.7 SENSOR_HUB_7 (08h) 

Sensor hub output register (r) 

Seventh byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

# 15.8 SENSOR_HUB_8 (09h) 

Sensor hub output register (r) 

Eighth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

# 15.9 SENSOR_HUB_9 (0Ah) 

Sensor hub output register (r) 

Ninth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

Table 320. SENSOR_HUB_7 register 

Sensor Hub7_7 Sensor Hub7_6 Sensor Hub7_5 Sensor Hub7_4 Sensor Hub7_3 Sensor Hub7_2 Sensor Hub7_1 Sensor Hub7_0 

Table 321. SENSOR_HUB_7 register description 

SensorHub7[7:0] Seventh byte associated to external sensors 

Table 322. SENSOR_HUB_8 register 

Sensor Hub8_7 Sensor Hub8_6 Sensor Hub8_5 Sensor Hub8_4 Sensor Hub8_3 Sensor Hub8_2 Sensor Hub8_1 Sensor Hub8_0 

Table 323. SENSOR_HUB_8 register description 

SensorHub8[7:0] Eighth byte associated to external sensors 

Table 324. SENSOR_HUB_9 register 

Sensor Hub9_7 Sensor Hub9_6 Sensor Hub9_5 Sensor Hub9_4 Sensor Hub9_3 Sensor Hub9_2 Sensor Hub9_1 Sensor Hub9_0 

Table 325. SENSOR_HUB_9 register description 

SensorHub9[7:0] Ninth byte associated to external sensors DocID032891 Rev 1 139/153 

LSM6DSO32 Sensor hub register description 

> 153

# 15.10 SENSOR_HUB_10 (0Bh) 

Sensor hub output register (r) 

Tenth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

# 15.11 SENSOR_HUB_11 (0Ch) 

Sensor hub output register (r) 

Eleventh byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

# 15.12 SENSOR_HUB_12 (0Dh) 

Sensor hub output register (r) 

Twelfth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

Table 326. SENSOR_HUB_10 register 

Sensor Hub10_7 Sensor Hub10_6 Sensor Hub10_5 Sensor Hub10_4 Sensor Hub10_3 Sensor Hub10_2 Sensor Hub10_1 Sensor Hub10_0 

Table 327. SENSOR_HUB_10 register description 

SensorHub10[7:0] Tenth byte associated to external sensors 

Table 328. SENSOR_HUB_11 register 

Sensor Hub11_7 Sensor Hub11_6 Sensor Hub11_5 Sensor Hub11_4 Sensor Hub11_3 Sensor Hub11_2 Sensor Hub11_1 Sensor Hub11_0 

Table 329. SENSOR_HUB_11 register description 

SensorHub11[7:0] Eleventh byte associated to external sensors 

Table 330. SENSOR_HUB_12 register 

Sensor Hub12_7 Sensor Hub12_6 Sensor Hub12_5 Sensor Hub12_4 Sensor Hub12_3 Sensor Hub12_2 Sensor Hub12_1 Sensor Hub12_0 

Table 331. SENSOR_HUB_12 register description 

SensorHub12[7:0] Twelfth byte associated to external sensors Sensor hub register description LSM6DSO32 

140/153 DocID032891 Rev 1 

# 15.13 SENSOR_HUB_13 (0Eh) 

Sensor hub output register (r) 

Thirteenth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

# 15.14 SENSOR_HUB_14 (0Fh) 

Sensor hub output register (r) 

Fourteenth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from x = 0 to x = 3). 

# 15.15 SENSOR_HUB_15 (10h) 

Sensor hub output register (r) 

Fifteenth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

Table 332. SENSOR_HUB_13 register 

Sensor Hub13_7 Sensor Hub13_6 Sensor Hub13_5 Sensor Hub13_4 Sensor Hub13_3 Sensor Hub13_2 Sensor Hub13_1 Sensor Hub13_0 

Table 333. SENSOR_HUB_13 register description 

SensorHub13[7:0] Thirteenth byte associated to external sensors 

Table 334. SENSOR_HUB_14 register 

Sensor Hub14_7 Sensor Hub14_6 Sensor Hub14_5 Sensor Hub14_4 Sensor Hub14_3 Sensor Hub14_2 Sensor Hub14_1 Sensor Hub14_0 

Table 335. SENSOR_HUB_14 register description 

SensorHub14[7:0] Fourteenth byte associated to external sensors 

Table 336. SENSOR_HUB_15 register 

Sensor Hub15_7 Sensor Hub15_6 Sensor Hub15_5 Sensor Hub15_4 Sensor Hub15_3 Sensor Hub15_2 Sensor Hub15_1 Sensor Hub15_0 

Table 337. SENSOR_HUB_15 register description 

SensorHub15[7:0] Fifteenth byte associated to external sensors DocID032891 Rev 1 141/153 

LSM6DSO32 Sensor hub register description 

> 153

# 15.16 SENSOR_HUB_16 (11h) 

Sensor hub output register (r) 

Sixteenth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from 

x = 0 to x = 3). 

# 15.17 SENSOR_HUB_17 (12h) 

Sensor hub output register (r) 

Seventeenth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from x = 0 to x = 3). 

# 15.18 SENSOR_HUB_18 (13h) 

Sensor hub output register (r) 

Eighteenth byte associated to external sensors. The content of the register is consistent with the SLAVEx_CONFIG number of read operation configurations (for external sensors from x = 0 to x = 3). 

Table 338. SENSOR_HUB_16 register 

Sensor Hub16_7 Sensor Hub16_6 Sensor Hub16_5 Sensor Hub16_4 Sensor Hub16_3 Sensor Hub16_2 Sensor Hub16_1 Sensor Hub16_0 

Table 339. SENSOR_HUB_16 register description 

SensorHub16[7:0] Sixteenth byte associated to external sensors 

Table 340. SENSOR_HUB_17 register 

Sensor Hub17_7 Sensor Hub17_6 Sensor Hub17_5 Sensor Hub17_4 Sensor Hub17_3 Sensor Hub17_2 Sensor Hub17_1 Sensor Hub17_0 

Table 341. SENSOR_HUB_17 register description 

SensorHub17[7:0] Seventeenth byte associated to external sensors 

Table 342. SENSOR_HUB_17 register 

Sensor Hub18_7 Sensor Hub18_6 Sensor Hub18_5 Sensor Hub18_4 Sensor Hub18_3 Sensor Hub18_2 Sensor Hub18_1 Sensor Hub18_0 

Table 343. SENSOR_HUB_17 register description 

SensorHub18[7:0] Eighteenth byte associated to external sensors Sensor hub register description LSM6DSO32 

142/153 DocID032891 Rev 1 

# 15.19 MASTER_CONFIG (14h) 

Master configuration register (r/w) 

# 15.20 SLV0_ADD (15h) 

I²C slave address of the first external sensor (Sensor 1) register (r/w) 

Table 344. MASTER_CONFIG register 

RST_ MASTER_ REGS WRITE_ ONCE START_ CONFIG PASS_ THROUGH _MODE SHUB_ PU_EN MASTER_ ON AUX_ SENS_ ON1 AUX_ SENS_ ON0 

Table 345. MASTER_CONFIG register description 

RST_MASTER_ REGS Reset Master logic and output registers. Must be set to ‘1’ and then set it to ‘0’. Default value: 0 WRITE_ONCE Slave 0 write operation is performed only at the first sensor hub cycle. Default value: 0 (0: write operation for each sensor hub cycle; 1: write operation only for the first sensor hub cycle) START_CONFIG Sensor hub trigger signal selection. Default value: 0 (0: sensor hub trigger signal is the accelerometer/gyro data-ready; 1: sensor hub trigger signal external from INT2 pin) PASS_THROUGH_ MODE I²C interface pass-through. Default value: 0 (0: pass-through disabled; 1: pass-through enabled, main I²C line is short-circuited with the auxiliary line) SHUB_PU_EN Master I²C pull-up enable. Default value: 0 (0: internal pull-up on auxiliary I²C line disabled; 1: internal pull-up on auxiliary I²C line enabled) MASTER_ON Sensor hub I²C master enable. Default: 0 (0: master I²C of sensor hub disabled; 1: master I²C of sensor hub enabled) AUX_SENS_ ON[1:0] Number of external sensors to be read by the sensor hub. (00: one sensor (default); 01: two sensors; 10: three sensors; 11: four sensors) 

Table 346. SLV0_ADD register 

slave0_ add6 slave0_ add5 slave0_ add4 slave0_ add3 slave0_ add2 slave0_ add1 slave0_ add0 rw_0 

Table 347. SLV_ADD register description 

slave0_add[6:0] I²C slave address of Sensor1 that can be read by the sensor hub. Default value: 0000000 rw_0 Read/write operation on Sensor 1. Default value: 0 (0: write operation; 1: read operation) DocID032891 Rev 1 143/153 

LSM6DSO32 Sensor hub register description 

> 153

# 15.21 SLV0_SUBADD (16h) 

Address of register on the first external sensor (Sensor 1) register (r/w) 

# 15.22 SLAVE0_CONFIG (17h) 

First external sensor (Sensor1) configuration and sensor hub settings register (r/w) 

Table 348. SLV0_SUBADD register 

slave0_ reg7 slave0_ reg6 slave0_ reg5 slave0_ reg4 slave0_ reg3 slave0_ reg2 slave0_ reg1 slave0_ reg0 

Table 349. SLV0_SUBADD register description 

slave0_reg[7:0] Address of register on Sensor1 that has to be read/written according to the rw_0 bit value in SLV0_ADD (15h) . Default value: 00000000 

Table 350. SLAVE0_CONFIG register 

SHUB_ ODR_1 SHUB_ ODR_0 0 (1) 1. This bit must be set to ‘0’ for the correct operation of the device. 0(1) BATCH_ EXT_SENS_ 0_EN Slave0_ numop2 Slave0_ numop1 Slave0_ numop0 

Table 351. SLAVE0_CONFIG register description 

SHUB_ODR_[1:0] Rate at which the master communicates. Default value: 00 (00: 104 Hz (or at the maximum ODR between the accelerometer and gyro if it is less than 104 Hz); 01: 52 Hz (or at the maximum ODR between the accelerometer and gyro if it is less than 52 Hz); 10: 26 Hz (or at the maximum ODR between the accelerometer and gyro if it is less than 26 Hz); 11: 13 Hz (or at the maximum ODR between the accelerometer and gyro if it is less than 13 Hz) BATCH_EXT_SENS_0_EN Enable FIFO batching data of first slave. Default value: 0 Slave0_numop[2:0] Number of read operations on Sensor 1. Default value: 000 Sensor hub register description LSM6DSO32 

144/153 DocID032891 Rev 1 

# 15.23 SLV1_ADD (18h) 

I² C slave address of the second external sensor (Sensor 2) register (r/w) 

# 15.24 SLV1_SUBADD (19h) 

Address of register on the second external sensor (Sensor 2) register (r/w) 

# 15.25 SLAVE1_CONFIG (1Ah) 

Second external sensor (Sensor 2) configuration register (r/w) 

Table 352. SLV1_ADD register 

Slave1_ add6 Slave1_ add5 Slave1_ add4 Slave1_ add3 Slave1_ add2 Slave1_ add1 Slave1_ add0 r_1 

Table 353. SLV1_ADD register description 

Slave1_add[6:0] I²C slave address of Sensor 2 that can be read by the sensor hub. Default value: 0000000 r_1 Read operation on Sensor 2 enable. Default value: 0 (0: read operation disabled; 1: read operation enabled) 

Table 354. SLV1_SUBADD register 

Slave1_ reg7 Slave1_ reg6 Slave1_ reg5 Slave1_ reg4 Slave1_ reg3 Slave1_ reg2 Slave1_ reg1 Slave1_ reg0 

Table 355. SLV1_SUBADD register description 

Slave1_reg[7:0] Address of register on Sensor 2 that has to be read/written according to the r_1 bit value in SLV1_ADD (18h) .

Table 356. SLAVE1_CONFIG register 

0(1) 1. This bit must be set to ‘0’ for the correct operation of the device. 0 (1) 0 (1) 0(1) BATCH_EXT _SENS_1 _EN Slave1_ numop2 Slave1_ numop1 Slave1_ numop0 

Table 357. SLAVE1_CONFIG register description 

BATCH_EXT_SENS_1_EN Enable FIFO batching data of second slave. Default value: 0 Slave1_numop[2:0] Number of read operations on Sensor 2. Default value: 000 DocID032891 Rev 1 145/153 

LSM6DSO32 Sensor hub register description 

> 153

# 15.26 SLV2_ADD (1Bh) 

I²C slave address of the third external sensor (Sensor 3) register (r/w) 

# 15.27 SLV2_SUBADD (1Ch) 

Address of register on the third external sensor (Sensor 3) register (r/w) 

# 15.28 SLAVE2_CONFIG (1Dh) 

Third external sensor (Sensor 3) configuration register (r/w) 

Enable FIFO batching data of third slave. Default value: 0 

Number of read operations on Sensor 3. Default value: 000 

Table 358. SLV2_ADD register 

Slave2_ add6 Slave2_ add5 Slave2_ add4 Slave2_ add3 Slave2_ add2 Slave2_ add1 Slave2_ add0 r_2 

Table 359. SLV2_ADD register description 

Slave2_add[6:0] I²C slave address of Sensor 3 that can be read by the sensor hub. r_2 Read operation on Sensor 3 enable. Default value: 0 (0: read operation disabled; 1: read operation enabled) 

Table 360. SLV2_SUBADD register 

Slave2_ reg7 Slave2_ reg6 Slave2_ reg5 Slave2_ reg4 Slave2_ reg3 Slave2_ reg2 Slave2_ reg1 Slave2_ reg0 

Table 361. SLV2_SUBADD register description 

Slave2_reg[7:0] Address of register on Sensor 3 that has to be read/written according to the r_2 bit value in SLV2_ADD (1Bh) .

Table 362. SLAVE2_CONFIG register 

0(1) 1. This bit must be set to ‘0’ for the correct operation of the device. 0 (1) 0(1) 0(1) BATCH_ EXT_SENS_ 2_EN Slave2_ numop2 Slave2_ numop1 Slave2_ numop0 

Table 363. SLAVE2_CONFIG register description 

BATCH_EXT_SENS_2_EN 

Slave2_numop[2:0] Sensor hub register description LSM6DSO32 

146/153 DocID032891 Rev 1 

# 15.29 SLV3_ADD (1Eh) 

I² C slave address of the fourth external sensor (Sensor 4) register (r/w) 

# 15.30 SLV3_SUBADD (1Fh) 

Address of register on the fourth external sensor (Sensor 4) register (r/w) 

# 15.31 SLAVE3_CONFIG (20h) 

Fourth external sensor (Sensor 4) configuration register (r/w) 

Table 364. SLV3_ADD register 

Slave3_ add6 Slave3_ add5 Slave3_ add4 Slave3_ add3 Slave3_ add2 Slave3_ add1 Slave3_ add0 r_3 

Table 365. SLV3_ADD register description 

Slave3_add[6:0] I²C slave address of Sensor 4 that can be read by the sensor hub. r_3 Read operation on Sensor 4 enable. Default value: 0 (0: read operation disabled; 1: read operation enabled) 

Table 366. SLV3_SUBADD register 

Slave3_ reg7 Slave3_ reg6 Slave3_ reg5 Slave3_ reg4 Slave3_ reg3 Slave3_ reg2 Slave3_ reg1 Slave3_ reg0 

Table 367. SLV3_SUBADD register description 

Slave3_reg[7:0] Address of register on Sensor 4 that has to be read according to the r_3 bit value in SLV3_ADD (1Eh) .

Table 368. SLAVE3_CONFIG register 

0(1) 1. This bit must be set to ‘0’ for the correct operation of the device. 0 (1) 0 (1) 0(1) BATCH_ EXT_SENS _3_EN Slave3_ numop2 Slave3_ numop1 Slave3_ numop0 

Table 369. SLAVE3_CONFIG register description 

BATCH_EXT_SENS_3_EN Enable FIFO batching data of fourth slave. Default value: 0 Slave3_numop[2:0] Number of read operations on Sensor 4. Default value: 000 DocID032891 Rev 1 147/153 

LSM6DSO32 Sensor hub register description 

> 153

# 15.32 DATAWRITE_SLV0 (21h) 

Data to be written into the slave device register (r/w) 

Data to be written into the slave 0 device according to the rw_0 bit in register 

SLV0_ADD (15h) .

Default value: 00000000 

# 15.33 STATUS_MASTER (22h) 

Sensor hub source register (r) 

Table 370. DATAWRITE_SLV0 register 

Slave0_ dataw7 Slave0_ dataw6 Slave0_ dataw5 Slave0_ dataw4 Slave0_ dataw3 Slave0_ dataw2 Slave0_ dataw1 Slave0_ dataw0 

Table 371. DATAWRITE_SLV0 register description 

Slave0_dataw[7:0] 

Table 372. STATUS_MASTER register 

WR_ ONCE_ DONE SLAVE3_ NACK SLAVE2_ NACK SLAVE1_ NACK SLAVE0_ NACK 0 0 SENS_HUB_ ENDOP 

Table 373. STATUS_MASTER register description 

WR_ONCE_DONE When the bit WRITE_ONCE in MASTER_CONFIG (14h) is configured as 1, this bit is set to 1 when the write operation on slave 0 has been performed and completed. Default value: 0 SLAVE3_NACK This bit is set to 1 if Not acknowledge occurs on slave 3 communication. Default value: 0 SLAVE2_NACK This bit is set to 1 if Not acknowledge occurs on slave 2 communication. Default value: 0 SLAVE1_NACK This bit is set to 1 if Not acknowledge occurs on slave 1 communication. Default value: 0 SLAVE0_NACK This bit is set to 1 if Not acknowledge occurs on slave 0 communication. Default value: 0 SENS_HUB_ENDOP Sensor hub communication status. Default value: 0 (0: sensor hub communication not concluded; 1: sensor hub communication concluded) Soldering information LSM6DSO32 

148/153 DocID032891 Rev 1 

# 16 Soldering information 

The LGA package is compliant with the ECOPACK, RoHS and "Green" standard. 

It is qualified for soldering heat resistance according to JEDEC J-STD-020. 

Land pattern and soldering recommendations are available at  www.st.com/mems .DocID032891 Rev 1 149/153 

LSM6DSO32 Package information 

> 153

# 17 Package information 

In order to meet environmental requirements, ST offers these devices in different grades of ECOPACK packages, depending on their level of environmental compliance. ECOPACK specifications, grade definitions and product status are available at: www.st.com. 

ECOPACK is an ST trademark. 

# 17.1 LGA-14L package information Figure 22. LGA-14L 2.5x3x0.86 mm package outline and mechanical data  

> :/+[   [  [    3LQLQGLFDWRU 3LQLQGLFDWRU

%277209,(: 7239,(:  'LPHQVLRQVDUHLQPLOOLPHWHUXQOHVVRWKHUZLVHVSHFLILHG *HQHUDOWROHUDQFHLVPPXQOHVVRWKHUZLVHVSHFLILHG 

287(5',0(16,216 ,7(0 ',0(16,21>PP@ 72/(5$1&(>PP@ 

@/>KWJQH/@:>KWGL:;$0@+>WKJLH+'0B Package information LSM6DSO32 

150/153 DocID032891 Rev 1 

# 17.2 LGA-14 packing information Figure 23. Carrier tape information for LGA-14 package Figure 24. LGA-14 package orientation in carrier tape DocID032891 Rev 1 151/153 

LSM6DSO32 Package information 

> 153

Figure 25. Reel information for carrier tape of LGA-14 package 

Table 374. Reel dimensions for carrier tape of LGA-14 package Reel dimensions (mm) 

A (max) 330 B (min) 1.5 C 13 ±0.25 D (min) 20.2 N (min) 60 G 12.4 +2/-0 T (max) 18.4  

> $'

%)XOOUDGLXV 7DSHVORW LQFRUHIRU WDSHVWDUW PPPLQZLGWK *PHDVXUHGDWKXE  &1PPPLQ $FFHVVKROHDW VORWORFDWLRQ 7Revision history LSM6DSO32 

152/153 DocID032891 Rev 1 

# 18 Revision history 

Table 375. Document revision history Date Revision Changes 

13-Mar-2020 1 Initial release DocID032891 Rev 1 153/153 

LSM6DSO32 

> 153

IMPORTANT NOTICE – PLEASE READ CAREFULLY 

STMicroelectronics NV and its subsidiaries (“ST”) reserve the right to make changes, corrections, enhancements, modifications, and improvements to ST products and/or to this document at any time without notice. Purchasers should obtain the latest relevant information on ST products before placing orders. ST products are sold pursuant to ST’s terms and conditions of sale in place at the time of order acknowledgement. Purchasers are solely responsible for the choice, selection, and use of ST products and ST assumes no liability for application assistance or the design of Purchasers’ products. No license, express or implied, to any intellectual property right is granted by ST herein. Resale of ST products with provisions different from the information set forth herein shall void any warranty granted by ST for such product. ST and the ST logo are trademarks of ST. For additional information about ST trademarks, please refer to www.st.com/trademarks . All other product or service names are the property of their respective owners. Information in this document supersedes and replaces information previously supplied in any prior versions of this document. © 2020 STMicroelectronics – All rights reserved
