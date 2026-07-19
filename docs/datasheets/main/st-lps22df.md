Title: Datasheet - LPS22DF - Low-power and high-precision MEMS nano pressure sensor: 260-1260 hPa absolute digital output barometer

URL Source: http://www.st.com/resource/en/datasheet/lps22df.pdf

Number of Pages: 53

Markdown Content:
Features 

• 260 to 1260 hPa absolute pressure range 

• Supply current down to 1.7 μA 

• Absolute pressure accuracy: 0.2 hPa 

• Robustness to soldering stress: 0.15 hPa 

• Low pressure sensor noise: 0.34 Pa 

• High-performance TCO: 0.45 Pa/°C 

• Embedded temperature compensation 

• 24-bit pressure data output 

• ODR from 1 Hz to 200 Hz 

• SPI, I²C, or MIPI I3C SM  interface 

• Supports 1.08 V digital interface 

• Embedded FIFO 

• Interrupt functions: data-ready, FIFO flags, pressure thresholds 

• Supply voltage: 1.7 to 3.6 V 

• High shock survivability: 22,000  g

• Small and thin package 

• ECOPACK  lead-free compliant 

## Applications 

• Altimeters and barometers for portable devices 

• GPS applications 

• Weather station equipment 

• Sport watches 

• e-cigarettes 

• Drones 

• Gas metering 

## Description 

The  LPS22DF  is an ultracompact, piezoresistive, absolute pressure sensor that functions as a digital output barometer. The  LPS22DF  provides lower power consumption, achieving lower pressure noise than its predecessor. 

The device comprises a sensing element and an IC interface that communicates over an I²C, MIPI I3C SM , or SPI interface from the sensing element to the application and supports a wide Vdd IO range for the digital interfaces as well. The sensing element, which detects absolute pressure, consists of a suspended membrane manufactured using a dedicated process developed by ST. 

The  LPS22DF  is available in a full-mold, holed LGA package (HLGA). It is guaranteed to operate over a temperature range extending from -40°C to +85°C. The package is holed to allow external pressure to reach the sensing element. 

Product status link 

LPS22DF 

Product summary 

Order code  LPS22DFTR 

Temperature 

range [°C]  -40 to +85 

Package  HLGA-10L 

(2.0 x 2.0 x 0.73 mm) 

Packing  Tape and reel 

Product resources 

AN5699  (device application note) 

TN0018  (design and soldering) 

# Low-power and high-precision MEMS nano pressure sensor: 260-1260 hPa 

# absolute digital output barometer 

# LPS22DF 

## Datasheet     

> DS13316 -Rev 3 -April 2025
> For further information, contact your local STMicroelectronics sales office. www.st.com

# 1 Block diagrams 

Figure 1.  Device architecture block diagram 

Sensing Element 

Temperature Sensor MUX Analog Front-End ADC Digital Logic I2CMIPI I3C SM 

SPI 

## Sensor Bias Voltage and Current Bias Clock and timing 

Figure 2.  Digital logic 

Analog Front-End ADC AVG Filter Pressure Compensation Temperature Compensation LP Filter FIFO Output Registers Output Registers 

> I2CMIPI I3C SM
> SPI

10

> EN_LPFP
> POUT
> TOUT

## LPS22DF 

Block diagrams 

DS13316 - Rev 3 page 2/53 2 Pin description 

Figure 3.  Pin connections (bottom view) 

1 235410 897 6

> Vdd_IO
> SCL/SPC

RES 

SDA/SDI/SDO 

SDO/SA0 

VDD 

GND 

GND_IO 

> INT_DRDY
> CS

Table 1.  Pin description 

Pin number  Name  Function 

1 Vdd_IO  Power supply for I/O pins 

2 SCL 

SPC 

I²C / MIPI I3C SM  serial clock (SCL) 

SPI serial port clock (SPC) 

3 RES  Connect to GND 

4

SDA 

SDI 

SDI/SDO 

I²C / MIPI I3C SM  serial data (SDA) 

4-wire SPI serial data input (SDI) 

3-wire serial data input/output (SDI/SDO) 

5 SDO 

SA0 

4-wire SPI serial data output (SDO) 

I²C least significant bit of the device address (SA0) 

MIPI I3C SM  least significant bit of the static address (SA0) 

6 CS 

Enables SPI 

I²C and MIPI I3C SM  / SPI mode selection 

(1: SPI idle mode / I²C and MIPI I3C SM  communication enabled; 

0: SPI communication mode / I²C and MIPI I3C SM  disabled) 

7 INT_DRDY  Interrupt or data-ready 

8 GND_IO  0 V supply 

9 GND  0 V supply 

10  VDD  Power supply 

## LPS22DF 

Pin description    

> DS13316 -Rev 3 page 3/53

# 3 Mechanical and electrical specifications 

## 3.1  Mechanical characteristics 

VDD = 1.8 V, T = 25°C, unless otherwise noted. 

Table 2.  Pressure and temperature sensor characteristics 

Symbol  Parameter  Test condition  Min.  Typ. (1)  Max.  Unit 

Pressure sensor characteristics 

PT op  Operating temperature range  -40  +85  °C 

Pop  Operating pressure range  260  1260  hPa 

Pbits  Pressure output data  24  bits 

Psens  Pressure sensitivity  4096  LSB/hPa 

PAccRel  Relative accuracy over pressure (2)  P = 800 - 1100 hPa, T = 25°C  ±0.01  hPa 

PAccT  Absolute accuracy over temperature  P = 260 ~ 1260 hPa, T = -20 ~ +85°C 

P = 660 ~ 1260 hPa, T = -20 ~ +65°C 

±0.45 

±0.2  hPa 

Pnoise  RMS pressure sensing noise (3)  with embedded filter and at T = 25°C  0.0034  hPa RMS 

ODR Pres  Pressure output data rate (4) 

1

4

10 

25 

50 

75 

100 

200 

Hz 

TCO  Temperature coefficient offset  P = 660 ~ 1160 hPa, T = -20 ~ +65°C  ±0.45  Pa/°C 

P_drift  Soldering drift  ±0.15  hPa 

P_short_drift  Short-term stability  24-hr, ODR = 10 Hz, AVG = 64  2 Pa 

P_long_drift  Long-term stability  1 year, based on HTOL  0.1  hPa 

Temperature sensor characteristics 

Top  Operating temperature range  -40  +85  °C 

Tsens  Temperature sensitivity  100  LSB/°C 

Tacc  Temperature absolute accuracy  T = 0 to 80°C  ±1.5  °C 

ODR T Output temperature data rate (4) 

1

4

10 

25 

50 

75 

100 

200 

Hz 

1.  Typical specifications are not guaranteed. 

2.  By design, the typ. value is defined based on characterization data with 10 hPa pressure interval. 

3.  Pressure noise RMS evaluated in a controlled environment, based on the average standard deviation of 50 measurements with AVG = 512, BW = ODR/9. 

4.  Output data rate is configured by ODR[3:0] in  CTRL_REG1 (10h) .

## LPS22DF 

Mechanical and electrical specifications 

DS13316 - Rev 3 page 4/53 3.2  Electrical characteristics 

VDD = 1.8 V, T = 25°C, unless otherwise noted. 

Table 3.  Electrical characteristics 

Symbol  Parameter  Test conditions  Min.  Typ. (1)  Max.  Unit 

VDD  Supply voltage  1.7  3.6  V

Vdd_IO  I/O supply voltage  1.08  3.6  V

Idd  Supply current 

@ODR 1 Hz 

AVGP = 4  1.7 

μA 

@ODR 1 Hz 

AVGP = 128  9.4 

IddPdn  Supply current in power-down mode  0.9  μA 

1.  Typical specifications are not guaranteed. 

Table 4.  DC characteristics 

Symbol  Parameter  Test conditions  Min.  Typ.  Max.  Unit 

DC input characteristics 

VIL  Low-level input voltage (Schmitt buffer)  Vdd_IO ≥ 1.8 V (typ)  - - 0.3 * Vdd_IO  V

Vdd_IO = 1.2 V (typ)  - - 0.2 * Vdd_IO 

VIH  High-level input voltage (Schmitt buffer)  Vdd_IO ≥ 1.8 V (typ)  0.7 * Vdd_IO  - - V

Vdd_IO = 1.2 V (typ)  0.8 * Vdd_IO  - -

DC output characteristics 

VOL  Low-level output voltage  - - 0.2  V

VOH  High-level output voltage  Vdd_IO - 0.2  - - V

## LPS22DF 

Mechanical and electrical specifications    

> DS13316 -Rev 3 page 5/53

## 3.3  Communication interface characteristics 

3.3.1  SPI - serial peripheral interface 

Subject to general operating conditions for Vdd and T OP .

Table 5.  SPI slave timing values 

Symbol  Parameter  Value (1) 

Unit 

Min  Max 

tc(SPC)  SPI clock cycle  100  ns 

fc(SPC)  SPI clock frequency  10 (2)  MHz 

tsu(CS)  CS setup time  6

ns 

th(CS)  CS hold time  8

tsu(SI)  SDI input setup time  5

th(SI)  SDI input hold time  15 

tv(SO)  SDO valid output time  50 

th(SO)  SDO output hold time  9

tdis(SO)  SDO output disable time  50 

1.  Values are evaluated at 10 MHz clock frequency for SPI with both 4 and 3 wires, based on characterization results, not tested in production. 

2.  Recommended to set max SPI clock 8 MHz to ≤50 Hz ODR. 

Figure 4.  SPI slave timing diagram 

Note:  Measurement points are done at 0.3·Vdd_IO and 0.7·Vdd_IO for both ports. 

## LPS22DF 

Mechanical and electrical specifications    

> DS13316 -Rev 3 page 6/53

3.3.2  I²C - inter-IC control interface 

Subject to general operating conditions for Vdd and T OP .

Table 6.  I²C slave timing values 

Symbol  Parameter  I²C fast mode (1) (2)  I²C fast mode plus (1) (2) 

Unit 

Min  Max  Min  Max 

f(SCL)  SCL clock frequency  0 400  0 1000  kHz 

tw(SCLL)  SCL clock low time  1.3  0.5 

μs 

tw(SCLH)  SCL clock high time  0.6  0.26 

tsu(SDA)  SDA setup time  100  50  ns 

th(SDA)  SDA data hold time  0 0.9  0

μs 

th(ST)  START/REPEATED START condition hold time  0.6  0.26 

tsu(SR)  REPEATED START condition setup time  0.6  0.26 

tsu(SP)  STOP condition setup time  0.6  0.26 

tw(SP:SR)  Bus free time between STOP and START condition  1.3  0.5 

Data valid time  0.9  0.45 

Data valid acknowledge time  0.9  0.45 

CB Capacitive load for each bus line  400  550  pF 

1.  Data based on standard I²C protocol requirement, not tested in production. 

2.  Data for I²C fast mode and I²C fast mode plus have been evaluated by characterization, not tested in production 

Figure 5.  I²C slave timing diagram 

SDA SCL 

tsu(SP) 

tw(SCLL) 

tsu(SDA) 

tsu(SR) 

th(ST) tw(SCLH) 

th(SDA)

tw(SP:SR)

START

REPEATED

START

STOP

ST ART

Note:  Measurement points are done at 0.3·Vdd_IO and 0.7·Vdd_IO for both ports. 

## LPS22DF 

Mechanical and electrical specifications 

DS13316 - Rev 3 page 7/53 3.4  Absolute maximum ratings 

Stress above those listed as absolute maximum ratings may cause permanent damage to the device. This is a stress rating only and functional operation of the device under these conditions is not implied. Exposure to maximum rating conditions for extended periods may affect device reliability. 

Table 7.  Absolute maximum ratings 

Symbol  Ratings  Maximum value  Unit 

Vdd  Supply voltage  -0.3 to +4.8  V

Vdd_IO  I/O pins supply voltage  -0.3 to +4.8  V

Vin  Input voltage on any control pin  -0.3 to Vdd_IO +0.3  V

P Overpressure  2 MPa 

TSTG  Storage temperature range  -40 to +125  °C 

ESD  Electrostatic discharge protection  2 (HBM)  kV 

Note:  Supply voltage on any pin should never exceed 4.8 V. 

This device is sensitive to mechanical shock, improper handling can cause permanent damage to the part. 

This device is sensitive to electrostatic discharge (ESD), improper handling can cause permanent damage to the part. 

## LPS22DF 

Mechanical and electrical specifications    

> DS13316 -Rev 3 page 8/53

# 4 Functionality 

The  LPS22DF  is a high-resolution, digital output pressure sensor packaged in an HLGA full-mold package. The complete device includes a sensing element based on a piezoresistive Wheatstone bridge approach, and an IC interface which communicates a digital signal from the sensing element to the application. 

## 4.1  Sensing element 

An ST proprietary process is used to obtain a silicon membrane for MEMS pressure sensors. This silicon membrane is surrounded by a silicon spring structure and it contributes to isolate the membrane from mechanical and thermal stress in applications. When pressure is applied, the membrane deflection induces an imbalance in the Wheatstone bridge piezoresistances whose output signal is converted by the IC interface. 

## 4.2  IC interface 

The complete measurement chain is composed of a low-noise amplifier which converts the resistance unbalance of the MEMS sensors (pressure and temperature) into an analog voltage using an analog-to-digital converter. 

The pressure and temperature data may be accessed through an I²C/MIPI I3C SM /SPI interface thus making the device particularly suitable for direct interfacing with a microcontroller. 

The  LPS22DF  features a Data-Ready signal which indicates when a new set of measured pressure and temperature data are available, thus simplifying data synchronization in the digital system that uses the device. 

## 4.3  Factory calibration 

The trimming values are stored inside the device in a nonvolatile structure. When the device is turned on, the trimming parameters are downloaded into the registers to be employed during the normal operation, which allows the device to be used without requiring any further calibration. 

## LPS22DF    

> Functionality
> DS13316 -Rev 3 page 9/53

## 4.4  Interpreting pressure readings 

The pressure data are stored in three registers:  PRESS_OUT_H (2Ah) , PRESS_OUT_L (29h) , and 

PRESS_OUT_XL (28h) . The value is expressed as a 24-bit signed number (in two’s complement). 

To obtain the pressure in hPa, take the complete 24-bit word and then divide by the sensitivity 4096 LSB/hPa. 

This same interpretation is applied to pressure readings when FIFO is enabled and the pressure data are stored in three registers:  FIFO_DATA_OUT_PRESS_XL (78h) , FIFO_DATA_OUT_PRESS_L (79h) , and 

FIFO_DATA_OUT_PRESS_H (7Ah) .

Figure 6.  Pressure readings 

(1)             

> Pressure Value =PRESS _OUT _H2Aℎ&PRESS _OUT _L29ℎ &PRESS _OUT _XL 28ℎ = 3 FF 58 Dℎ = 4191629 LSB signed decimal

(2) 

Pressure  ℎPa  = Pressure value  LSB 

Sensitivity = 4191629 LSB 

4096 LSB /ℎ Pa = 1023.3 ℎ Pa 

## LPS22DF 

Functionality    

> DS13316 -Rev 3 page 10/53

## 4.5  Interpreting temperature readings 

The temperature data are stored in two registers:  TEMP_OUT_H (2Ch)  and  TEMP_OUT_L (2Bh) .

The value is expressed as two’s complement. To obtain the temperature in °C, take the two’s complement of the complete 16-bit word and then divide by the sensitivity 100 LSB/°C. 

Figure 7.  Temperature readings 

## 0 0 0 0 1 0 0 1 1 1 0 0 0 1 0 0TEMP_OUT_H TEMP_OUT_L 

Temperature Value (LSB) = TEMP_OUT_H (2Ch) & TEMP_OUT_L (2Bh) = 09C4 = 2500 LSB (decimal signed) Temperature (°C) = Temperature Value (LSB) Sensitivity = 2500 LSB 100 LSB/°C = 25.00°C 

## LPS22DF    

> Functionality
> DS13316 -Rev 3 page 11/53

# 5 FIFO 

The  LPS22DF  embeds 128 slots of 24-bit data FIFO to store the pressure output values. This allows consistent power saving for the system, since the host processor does not need to continuously poll data from the sensor, but it can wake up only when needed and burst the significant data out from the FIFO. This buffer can work according to six different modes: 

• Bypass mode 

• FIFO mode 

• Continuous (dynamic-stream) mode 

• Continuous (dynamic-stream)-to-FIFO mode 

• Bypass-to-continuous (dynamic-stream) 

• Bypass-to-FIFO mode 

The FIFO buffer is enabled when a configuration different from all bits 0 are written in  FIFO_CTRL (14h)  and each mode is selected by the TRIG_MODES bit and F_MODE[1:0] bits in  FIFO_CTRL (14h) . Programmable FIFO threshold status, FIFO overrun events, and the number of unread samples stored are available in the 

FIFO_STATUS1 (25h)  and  FIFO_STATUS2 (26h)  registers and can be set to generate dedicated interrupts on the INT_DRDY pin using the  CTRL_REG3 (12h)  register. 

FIFO_STATUS2 (26h) (FIFO_WTM_IA) goes to 1 when the number of unread samples ( FIFO_STATUS1 (25h) 

(FSS[7:0]) is greater than or equal to WTM[6:0] in  FIFO_WTM (15h) . If  FIFO_WTM (15h) (WTM[6:0]) is equal to 0, 

FIFO_STATUS2 (26h) (FIFO_WTM_IA) stays at 0. 

FIFO_STATUS2 (26h) (FIFO_OVR_IA) is equal to 1 if a FIFO slot is overwritten. 

FIFO_STATUS1 (25h) (FSS[7:0]) contains stored data levels of unread samples; when FSS[7:0] is equal to 00000000, FIFO is empty; when FSS[7:0] is equal to 10000000, FIFO is full and the unread samples are 128. 

## LPS22DF    

> FIFO
> DS13316 -Rev 3 page 12/53

## 5.1  Bypass mode 

In bypass mode ( FIFO_CTRL (14h) (TRIG_MODES and F_MODE[1:0] = 000 or 100), the FIFO is not operational and it remains empty. 

Switching to bypass mode is also used to reset the FIFO. Passing through bypass mode is mandatory when switching between different FIFO buffer operating modes. 

As described in the next figure, for each channel only the first address is used. When new data is available, the older data is overwritten. 

Figure 8.  Bypass mode 

P1

P2

Pi

empty 

P0

P127 

## LPS22DF    

> FIFO
> DS13316 -Rev 3 page 13/53

## 5.2  FIFO mode 

In FIFO mode ( FIFO_CTRL (14h) (TRIG_MODES and F_MODE[1:0] = 001), data from the output 

PRESS_OUT_XL (28h) , PRESS_OUT_L (29h) , and  PRESS_OUT_H (2Ah)  are stored in the FIFO until it is full. 

To reset FIFO content, in order to select bypass mode the value 000 must be written in  FIFO_CTRL (14h) 

(TRIG_MODE and F_MODE[1:0]). After this reset command, it is possible to restart FIFO mode by writing the value 001 in  FIFO_CTRL (14h) (TRIG_MODE and F_MODE[1:0]). 

The FIFO buffer memorizes 128 levels of data, but the depth of the FIFO can be resized/reduced by setting the 

FIFO_CTRL (14h) (STOP_ON_WTM) bit. If the STOP_ON_WTM bit is set to 1, FIFO depth is limited to 

FIFO_WTM (15h) (WTM[6:0]) data. 

Figure 9.  FIFO mode 

P1

P2

Pi

P0

P127 

## LPS22DF    

> FIFO
> DS13316 -Rev 3 page 14/53

## 5.3  Continuous (dynamic-stream) mode 

In continuous (dynamic-stream) mode ( FIFO_CTRL (14h) (TRIG_MODES and F_MODE[1:0] = 011), after emptying the FIFO, the first new sample that arrives, becomes the first to be read in a subsequent read burst. In this way, the number of new data available in FIFO does not depend on the previous read. 

In continuous (dynamic-stream) mode  FIFO_STATUS1 (25h) (FSS[7:0]) is the number of new pressure samples available in the FIFO buffer. 

Continuous (dynamic-stream) is intended to be used to read  FIFO_STATUS1 (25h) (FSS[7:0]) samples when it is not possible to guarantee reading data within 1/ODR time period. 

Also, a FIFO threshold interrupt on the INT_DRDY pin through  CTRL_REG3 (12h) (INT_F_WTM) can be enabled in order to read data from the FIFO and leave free memory slots for incoming data. 

Figure 10.  Continuous (dynamic-stream) mode 

## LPS22DF    

> FIFO
> DS13316 -Rev 3 page 15/53

## 5.4  Bypass-to-FIFO mode 

In bypass-to-FIFO mode ( FIFO_CTRL (14h) (TRIG_MODES and F_MODE[1:0] = 101), FIFO behavior switches when the  INT_SOURCE (24h) (IA) bit rises for the first time. When the  INT_SOURCE (24h) (IA) bit is equal to 0, FIFO behaves like in bypass mode. Once the  INT_SOURCE (24h) (IA) bit rises to 1, FIFO behavior switches and keeps behaving like in FIFO mode. 

An interrupt generator has to be set to the desired configuration through  INTERRUPT_CFG (0Bh) .

Figure 11.  Bypass-to-FIFO mode 

P1

P2

Pi

empty 

P0

P127 

P1

P2

Pi P0

P127 

Bypass Mode FIFO Mode Trigger event 

## LPS22DF    

> FIFO
> DS13316 -Rev 3 page 16/53

## 5.5  Bypass-to-continuous (dynamic-stream) mode 

In bypass-to-continuous (dynamic-stream) mode ( FIFO_CTRL (14h) (TRIG_MODES and F_MODE[1:0] = 110), FIFO operates in bypass mode until it switches to continuous (dynamic-stream) mode behavior when 

INT_SOURCE (24h) (IA) rises to 1, then FIFO behavior keeps behaving like in continuous (dynamic-stream) mode. 

An interrupt generator has to be set to the desired configuration through  INTERRUPT_CFG (0Bh) .

Figure 12.  Bypass-to-continuous (dynamic-stream) mode 

P1

P2

Pi

empty 

P0

P127 

P1

P2

Pi P0

P127 

Bypass Mode Dynamic-Stream Mode Trigger event 

P126 

## LPS22DF 

FIFO    

> DS13316 -Rev 3 page 17/53

## 5.6  Continuous (dynamic-stream)-to-FIFO mode 

In continuous (dynamic-stream)-to-FIFO mode ( FIFO_CTRL (14h) (TRIG_MODES and F_MODE[1:0] = 111), data are stored in FIFO and FIFO operates in continuous (dynamic-stream) mode behavior until it switches to FIFO mode behavior when  INT_SOURCE (24h) (IA) rises to 1. 

An interrupt generator has to be set to the desired configuration through  INTERRUPT_CFG (0Bh) .

Figure 13.  Continuous (dynamic-stream)-to-FIFO mode 

P1

P2

Pi P0

P127 

P1

P2

Pi P0

P127 

Dynamic-Stream Mode FIFO Mode Trigger event 

P126 

## 5.7  Retrieving data from FIFO 

FIFO data is read through FIFO_DATA_OUT_PRESS (78h, 79h, and 7Ah). 

The read address is automatically updated by the device and it rolls back to 78h when register 7Ah is reached. In order to read all FIFO levels in a multiple byte read, 384 bytes (three output registers with 128 levels) must be read. 

## LPS22DF    

> FIFO
> DS13316 -Rev 3 page 18/53

# 6 Application hints 

Figure 14.  LPS22DF  electrical connections (top view) 

The device power supply must be provided through the VDD line; a power supply decoupling capacitor C1 (100 nF) must be placed as near as possible to the supply pins of the device. The C1 capacitor can be tied to VDD and Vdd_IO, but it is recommended to use 2 capacitors, one on each VDD and Vdd_IO line, in case VDD are Vdd_IO are separate. Depending on the application, an additional capacitor of 4.7 μF could be placed on the VDD line. 

The functionality of the device and the measured data outputs are selectable and accessible through the I²C, MIPI I3C SM , SPI interface. When using the I²C and MIPI I3C SM , CS must be tied to Vdd_IO. 

All the voltage and ground supplies must be present at the same time to have proper behavior of the IC (refer to 

Figure 14 ). It is possible to remove VDD while maintaining Vdd_IO without blocking the communication bus, in this condition the measurement chain is powered off. 

Note:  To guarantee proper power-off of the device, it is recommended to maintain the duration of the VDD line to GND for at least 10 ms. 

## LPS22DF    

> Application hints
> DS13316 -Rev 3 page 19/53

Figure 15.  LPS22DF  power-off sequence 

VDD Time 

 VDD Rising / Falling time : 10 μs ~ 100 ms 

 VDD must be lower than 0.7 V for at least 10 ms during power-off sequence for correct POR 

0.7 V 

Min.10 ms 

## 6.1  Soldering information 

The HLGA package is compliant with the  ECOPACK  standard and it is qualified for soldering heat resistance according to JEDEC J-STD-020. 

For land pattern and soldering recommendations, consult technical note  TN0018  available on  www.st.com .

## LPS22DF    

> Application hints
> DS13316 -Rev 3 page 20/53

# 7 Digital interfaces 

## 7.1  Serial interfaces 

The registers embedded in the  LPS22DF  may be accessed through either the I²C, MIPI I3C SM , or SPI serial interfaces. The latter may be software configured to operate either in 3-wire or 4-wire interface mode. 

The serial interfaces are mapped to the same pins. To select/exploit the I²C interface, the CS line must be tied high (that is, connected to Vdd_IO). 

Table 8.  Serial interface pin description 

Pin name  Pin description 

CS 

Enables SPI 

I²C and MIPI I3C SM  / SPI mode selection 

(1: SPI idle mode / I²C and MIPI I3C SM  communication enabled; 

0: SPI communication mode / I²C and MIPI I3C SM  disabled) 

SCL/SPC  I²C / MIPI I3C SM  serial clock (SCL) 

SPI serial port clock (SPC) 

SDA 

SDI 

SDI/SDO 

I²C / MIPI I3C SM  serial data (SDA) 

4-wire SPI serial data input (SDI) 

3-wire serial data input/output (SDI/SDO) 

SDO 

SAO 

4-wire SPI serial data output (SDO) 

I²C least significant bit of the device address (SA0) 

MIPI I3C SM  least significant bit of the static address (SA0) 

## 7.2  I²C serial interface (CS = high) 

The  LPS22DF  I²C is a bus slave. The I²C is employed to write data into registers whose content can also be read back. 

The relevant I²C terminology is given in the following table. 

Table 9.  I²C terminology 

Term  Description 

Transmitter  The device that sends data to the bus 

Receiver  The device that receives data from the bus 

Master  The device that initiates a transfer, generates clock signals, and terminates a transfer 

Slave  The device addressed by the master 

There are two signals associated with the I²C bus: the serial clock line (SCL) and the serial data line (SDA). The latter is a bidirectional line used for sending and receiving the data to/from the interface. Both lines have to be connected to Vdd_IO through pull-up resistors. 

The I²C interface is compliant with fast mode plus (1 MHz) I²C standards as well as with the normal mode. 

## LPS22DF 

Digital interfaces    

> DS13316 -Rev 3 page 21/53

7.2.1  I²C operation 

The transaction on the bus is started through a start (ST) signal. A start condition is defined as a high to low transition on the data line while the SCL line is held high. After the master has transmitted this, the bus is considered busy. The next data byte transmitted after the start condition contains the address of the slave in the first 7 bits and the eighth bit tells whether the master is receiving data from the slave or transmitting data to the slave. When an address is sent, each device in the system compares the first seven bits after a start condition with its address. If they match, the device considers itself addressed by the master. 

The 7-bit slave address (SAD) associated to the  LPS22DF  is 101110xb. The SDO/SA0 pin can be used to modify the less significant bit of the device address. If the SA0 pin is connected to the voltage supply, LSb is 1 (7-bit address 1011101b=5Dh), otherwise if the SA0 pin is connected to ground, the LSb value is 0 (7-bit address 1011100b=5Ch). This solution permits connecting and addressing two different  LPS22DF  devices to the same I²C lines. 

Data transfer with acknowledge is mandatory. The transmitter must release the SDA line during the acknowledge pulse. The receiver must then pull the data line low so that it remains stable low during the high period of the acknowledge clock pulse. A receiver that has been addressed is obliged to generate an acknowledge after each byte of data received. 

The I²C embedded inside the ASIC behaves like a slave device and the following protocol must be adhered to. After the start condition (ST) a slave address is sent, once a slave acknowledge has been returned (SAK), an 8-bit subaddress is transmitted (SUB). The IF_ADD_INC bit in  CTRL_REG2 (11h)  enables subaddress auto increment (IF_ADD_INC is 1 by default), so if IF_ADD_INC = 1 the SUB (subaddress) is automatically increased to allow multiple data read/write. 

The slave address is completed with a read/write bit. If the bit is 1 (read), a repeated start (SR) condition must be issued after the two subaddress bytes; if the bit is 0 (write) the master transmits to the slave with direction unchanged.  Table 10 explains how the SAD+read/write bit pattern is composed, listing all the possible configurations. 

Table 10.  SAD+read/write patterns                     

> Command SAD[6:1] SAD[0] = SA0 R/W SAD+R/W
> Read 101110 0110111001(B9h)
> Write 101110 0010111000(B8h)
> Read 101110 1110111011(BBh)
> Write 101110 1010111010(BAh)

Table 11.  Transfer when master is writing one byte to slave         

> Master ST SAD+ W SUB DATA SP
> Slave SAK SAK SAK

Table 12.  Transfer when master is writing multiple bytes to slave           

> Master ST SAD+ W SUB DATA DATA SP
> Slave SAK SAK SAK SAK

Table 13.  Transfer when master is receiving (reading) one byte of data from slave            

> Master ST SAD+ W SUB SR SAD+ R NMAK SP
> Slave SAK SAK SAK DATA

Table 14.  Transferwhen master is receiving (reading) multiple bytes of data from slave                

> Master ST SAD+ WSUB SR SAD+ RMAK MAK NMAK SP
> Slave SAK SAK SAK DATA DATA DATA

## LPS22DF    

> Digital interfaces
> DS13316 -Rev 3 page 22/53

Data are transmitted in byte format (DATA). Each data transfer contains 8 bits. The number of bytes transferred per transfer is unlimited. Data is transferred with the most significant bit (MSb) first. If a slave receiver does not acknowledge the slave address (that is, it is not able to receive because it is performing some real-time function), the data line must be kept high by the slave. The master can then abort the transfer. A low to high transition on the SDA line while the SCL line is high is defined as a stop condition. Each data transfer must be terminated by the generation of a stop (SP) condition. 

In the presented communication format MAK is master acknowledge and NMAK is no master acknowledge. 

## LPS22DF    

> Digital interfaces
> DS13316 -Rev 3 page 23/53

## 7.3  SPI bus interface (CS = low) 

The  LPS22DF  SPI is a bus slave. The SPI allows writing to and reading from the registers of the device. 

The serial interface interacts with the application using four wires:  CS , SPC , SDI , and  SDO .

Figure 16.  Read and write protocol 

CS 

SPC 

SDI 

SDO        

> RW
> AD5 AD4 AD3 AD2 AD1 AD0
> DI7 DI6 DI5 DI4 DI3 DI2 DI1 DI0
> DO7 DO6 DO5 DO4 DO3 DO2 DO1 DO0
> AD6

CS  enables the serial port and it is controlled by the SPI master. It goes low at the start of the transmission and returns to high at the end.  SPC  is the serial port clock and it is controlled by the SPI master. It is stopped high when  CS  is high (no transmission).  SDI  and  SDO  are respectively the serial port data input and output. Those lines are driven at the falling edge of  SPC  and should be captured at the rising edge of  SPC .

Both the read register and write register commands are completed in 16 clock pulses or multiples of 8 in the case of multiple read/write bytes. Bit duration is the time between the two falling edges of  SPC . The first bit (bit 0) starts at the first falling edge of  SPC  after the falling edge of  CS  while the last bit (bit 15, bit 23,...) starts at the last falling edge of SPC just before the rising edge of CS. 

bit 0 : R W bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0) from the device is read. In the latter case, the chip drives  SDO  at the start of bit 8. 

bit 1-7 : address AD(6:0). This is the address field of the indexed register. 

bit 8-15 : data DI(7:0) (write mode). This is the data that is written into the device (MSb first). bit 8-15: data DO(7:0) (read mode). This is the data that is read from the device (MSb first). In multiple read/write commands further blocks of 8 clock periods are added. When the IF_ADD_INC bit is 0, the address used to read/write data remains the same for every block. When the IF_ADD_INC bit is 1, the address used to read/write data is incremented at every block. 

The function and the behavior of  SDI  and  SDO  remain unchanged. 

## LPS22DF    

> Digital interfaces
> DS13316 -Rev 3 page 24/53

7.3.1  SPI read 

Figure 17.  SPI read protocol 

The SPI read command is performed with 16 clock pulses. The multiple byte read command is performed by adding blocks of 8 clock pulses to the previous one. 

bit 0 : READ bit. The value is 1. 

bit 1-7 : address AD(6:0). This is the address field of the indexed register. 

bit 8-15 : data DO(7:0) (read mode). This is the data that is read from the device (MSb first). 

bit 16-... : data DO(...-8). Further data in multiple byte reads. 

Figure 18.  Multiple byte SPI read protocol (2-byte example) 

## LPS22DF    

> Digital interfaces
> DS13316 -Rev 3 page 25/53

7.3.2  SPI write 

Figure 19.  SPI write protocol 

CS 

SPC 

SDI             

> RW DI7 DI6 DI5 DI4 DI3 DI2 DI1 DI0
> AD5 AD4 AD3 AD2 AD1 AD0 AD6

The SPI write command is performed with 16 clock pulses. The multiple byte write command is performed by adding blocks of 8 clock pulses to the previous one. 

bit 0 : WRITE bit. The value is 0. 

bit 1-7 : address AD(6:0). This is the address field of the indexed register. 

bit 8-15 : data DI(7:0) (write mode). This is the data that is written in the device (MSb first). 

bit 16-... : data DI(...-8). Further data in multiple byte writes. 

Figure 20.  Multiple byte SPI write protocol (2-byte example) 

CS 

SPC 

SDI                     

> RW
> AD5AD4AD 3AD2AD1AD0
> DI 7DI6DI 5DI4DI 3DI2DI 1DI0DI 15DI 14DI 13DI1 2DI 11DI10DI 9DI8
> AD6

7.3.3  SPI read in 3-wire mode 

A 3-wire mode is entered by setting the SIM bit to 1 (SPI serial interface mode selection) in  CTRL_REG1 (10h) .

Figure 21.  SPI read protocol in 3-wire mode 

CS 

SPC 

SDI/O            

> RW DO7 DO6 DO5 DO4 DO3 DO2 DO1 DO0
> AD5 AD4 AD3 AD2 AD1 AD0 MS AD6

The SPI read command is performed with 16 clock pulses: 

bit 0 : READ bit. The value is 1. 

bit 1-7 : address AD(6:0). This is the address field of the indexed register. 

bit 8-15 : data DO(7:0) (read mode). This is the data that is read from the device (MSb first). A multiple read command is also available in 3-wire mode. 

## 7.4  MIPI I3C SM  slave interface 

The  LPS22DF  interface includes an MIPI I3C SM  SDR only slave interface (compliant with release 1.1 of the specification) with MIPI I3C SM  SDR embedded features: 

• CCC command 

## LPS22DF 

Digital interfaces    

> DS13316 -Rev 3 page 26/53

• Direct CCC communication (SET and GET) 

• Broadcast CCC communication 

• Private communications 

• Private read and write for single byte 

• Multiple read and write 

• In-band interrupt request 

• Slave reset pattern 

• Group address 

• Full range VDD IO support 

• Asynchronous modes 0 and 1 

• Synchronous mode 

• Error detection and recovery methods (S0-S6) 

In order to disable the I3C block, I2C_I3C_DIS = 1 must be written in  IF_CTRL (0Eh) .

7.4.1  MIPI I3C SM  CCC supported commands 

The list of MIPI I3C SM  CCC commands supported by the device is detailed in the following table. 

Table 15.  MIPI I3C SM  CCC commands 

Command  Command code  Default  Description 

ENTDAA  0x07  DAA procedure 

SETDASA  0x87  Assign dynamic address using static address 0x6B/0x6A depending on SDO pin 

ENEC  0x80 / 0x00  Slave activity control (direct and broadcast) 

DISEC  0x81/ 0x01  Slave activity control (direct and broadcast) 

ENTAS0  0x82 / 0x02  Enter activity state (direct and broadcast) 

SETXTIME  0x98 / 0x28  Timing information exchange 

GETXTIME  0x99 

0x07 

0x00 

0x0C 

0x92 

Timing information exchange 

RSTDAA  0x06  Reset the assigned dynamic address (broadcast only) 

SETMWL  0x89 / 0x08  Define maximum write length during private write (direct and broadcast) 

SETMRL  0x8A / 0x09  Define maximum read length during private read (direct and broadcast) 

SETNEWDA  0x88  Change dynamic address 

GETMWL  0x8B 

0x00 

0x08 

(2 byte) 

Get maximum write length during private write 

GETMRL  0x8C 

0x00 

0x10 

0x05 

(3 byte) 

Get maximum read length during private read 

## LPS22DF 

Digital interfaces                          

> DS13316 -Rev 3 page 27/53 Command Command code Default Description
> GETPID 0x8D
> 0x02
> 0x08
> 0x00
> 0xB4
> 0x90
> 0x0B
> SDO = 1
> 0x02
> 0x08
> 0x00
> 0xB4
> 0x10
> 0x0B
> SDO = 0
> GETBCR 0x8E 0x07
> (1 byte) Bus characteristics register
> GETDCR 0x8F 0x62 default MIPI I3C SM device characteristics register
> GETSTATUS 0x90
> 0x00
> 0x00
> (2 byte)
> Status register
> GETMXDS 0x94 0x08
> 0x60 Return maximum write and read speed
> GETCAPS 0x95
> 0x00
> 0x11
> 0x18
> 0x00
> Provide information about device capabilities and supported extended features
> SETGRPA 0x9B Group address assignment command
> RSTGRPA 0x2C/0x9C Reset the group address
> RSTACT 0x9A/0x2A Configure slave reset action

7.4.2  Overview of antispike filter management 

The device acts as a standard I²C target as long as it has an I²C static address. The device is capable of detecting and disabling the I²C antispike filter after detecting the broadcast address (7'h7E/W). In order to guarantee proper behavior of the device, the I3C master must emit the first START, 7'h7E/W at open-drain speed using I²C fast mode plus reference timing. 

After detecting the broadcast address, the device can receive the I3C dynamic address following the I3C push-pull timing. If the device is not assigned a dynamic address, then the device continues to operate as an I²C device with no antispike filter. For the case in which the host decides to keep the device as I²C with antispike filter, there is a configuration required to keep the antispike filter active. This configuration is done by writing the ASF_ON bit to 1 in the  I3C_IF_CTRL_ADD (19h)  register. This configuration forces the antispike filter to always be turned on instead of being managed by the communication on the bus. 

## LPS22DF    

> Digital interfaces
> DS13316 -Rev 3 page 28/53

# 8 Register mapping 

The following table provides a quick overview of the 8-bit registers embedded in the device. 

Table 16.  Registers address map 

Name  Type 

Register address  Default 

Function and comment 

Hex  Hex 

Reserved  00 – 0A  - Reserved 

INTERRUPT_CFG  R/W  0B  00h  Interrupt register 

THS_P_L  R/W  0C  00h 

Pressure threshold registers 

THS_P_H  R/W  0D  00h 

IF_CTRL  R/W  0E  00h  Interface control register 

WHO_AM_I  R 0F  B4h  Who am I 

CTRL_REG1  R/W  10  00h 

Control registers 

CTRL_REG2  R/W  11  00h 

CTRL_REG3  R/W  12  01h 

CTRL_REG4  R/W  13  00h 

FIFO_CTRL  R/W  14  00h 

FIFO configuration registers 

FIFO_WTM  R/W  15  00h 

REF_P_L  R 16  00h 

Reference pressure registers 

REF_P_H  R 17  00h 

Reserved  18  - Reserved 

I3C_IF_CTRL  R/W  19  80h  Interface configuration register 

RPDS_L  R/W  1A  00h 

Pressure offset registers 

RPDS_H  R/W  1B  00h 

Reserved  1C-23  - Reserved 

INT_SOURCE  R 24  Output  Interrupt register 

FIFO_STATUS1  R 25  Output 

FIFO status registers 

FIFO_STATUS2  R 26  Output 

STATUS  R 27  Output  Status register 

PRESSURE_OUT_XL  R 28  Output 

Pressure output registers PRESSURE_OUT_L  R 29  Output 

PRESSURE_OUT_H  R 2A  Output 

TEMP_OUT_L  R 2B  Output 

Temperature output registers 

TEMP_OUT_H  R 2C  Output 

Reserved  2D - 77  - Reserved 

FIFO_DATA_OUT_PRESS_XL  R 78  Output 

FIFO pressure output registers FIFO_DATA_OUT_PRESS_L  R 79  Output 

FIFO_DATA_OUT_PRESS_H  R 7A  Output 

Reserved registers must not be changed. Writing to those registers may cause permanent damage to the device. 

To guarantee the proper behavior of the device, all register addresses not listed in the above table must not be accessed and the content stored in those registers must not be changed. 

The content of the registers that are loaded at boot should not be changed. They contain the factory calibration values. Their content is automatically restored when the device is powered up. 

## LPS22DF 

Register mapping 

DS13316 - Rev 3 page 29/53 9 Register description 

The device contains a set of registers which are used to control its behavior and to retrieve pressure and temperature data. The register address, made up of 7 bits, is used to identify them and to read/write the data through the serial interface. 

## 9.1  INTERRUPT_CFG (0Bh) 

Interrupt mode for pressure acquisition configuration (R/W) 

7 6 5 4 3 2 1 0

AUTOREFP  RESET_ARP  AUTOZERO  RESET_AZ  - LIR  PLE  PHE 

AUTOREFP  Enable AUTOREFP function. Default value: 0 

(0: normal mode; 1: AUTOREFP enabled) 

RESET_ARP  Reset AUTOREFP function. Default value: 0 

(0: normal mode; 1: reset AUTOREFP function) 

AUTOZERO  Enable AUTOZERO function. Default value: 0 

(0: normal mode; 1: AUTOZERO enabled) 

RESET_AZ  Reset AUTOZERO function. Default value: 0 

(0: normal mode; 1: reset AUTOZERO function) 

LIR  Latch interrupt request to the  INT_SOURCE (24h)  register. Default value: 0 

(0: interrupt request not latched; 1: interrupt request latched) 

PLE 

Enable interrupt generation on pressure low event. Default value: 0 

(0: disable interrupt request; 

1: enable interrupt request on pressure value lower than preset threshold) 

PHE 

Enable interrupt generation on pressure high event. Default value: 0 

(0: disable interrupt request; 

1: enable interrupt request on pressure value higher than preset threshold) 

Referring to  Figure 22 , the  LPS22DF  can be set by the user to support the interrupt function when P_DIFF_IN (defined below) is higher or lower than the threshold value stored in THS_P_L (0Ch) and THS_P_H (0Dh). 

It is enabled when either PHE bit or PLE bit (or both bits) is set to 1. Then, the differential pressure can be compared to a user-defined threshold stored in the 15-bit THS_P (0Ch and 0Dh) registers. 

The threshold pressure value defined by the user is a 15-bit unsigned value in a 16-bit register composed of 

THS_P_L (0Ch)  and  THS_P_H (0Dh) . The value is: 

THS_P (15-bit unsigned) = Desired Interrupt threshold (hPa) x 16 

The PHE and PLE bits in INTERRUPT_CFG (0Bh) enable the differential pressure interrupt generation on the positive or negative event respectively. 

The differential interrupt must be used with AUTOREFP or AUTOZERO mode. 

Figure 22.  "Threshold-based" interrupt event 

> P_DIFF_IN Negative
> Threshold positive value Threshold negative value
> Positive

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 30/53

To enable the  AUTOZERO  mode, the AUTOZERO bit must be set to 1 and then the measured pressure value is used as the reference and stored in the register REF_P ( REF_P_L (16h) , REF_P_H (17h) ). From this point on, the output pressure value ( PRESS_OUT_XL (28h) , PRESS_OUT_L (29h) , PRESS_OUT_H (2Ah) ) is updated with the difference between the measured pressure and REF_P. 

• P_DIFF_IN = measured pressure - REF_P 

• PRESS_OUT = measured pressure - REF_P 

After the first conversion, the AUTOZERO bit is automatically set back to 0. In order to return back to normal mode, the RESET_AZ bit in the INTERRUPT_CFG (0Bh) register has to be set to 1. This also resets the content of the REF_P registers to 0. 

AUTOREFP  mode allows using the pressure differential for the generation of the interrupt keeping the output pressure registers PRESS_OUT ( PRESS_OUT_XL (28h) , PRESS_OUT_L (29h) , PRESS_OUT_H (2Ah) ) without comparing REF_P. If the AUTOREFP bit is set to 1, the measured output pressure is used as the reference in the register REF_P ( REF_P_L (16h) , REF_P_H (17h) ) for interrupt generation with the following: 

• P_DIFF_IN = measured pressure - REF_P 

The output registers PRESS_OUT (28h, 29h, and 2Ah) are not changed by REF_P and shows as follows. 

• PRESS_OUT = measured pressure 

After the first conversion, the AUTOREFP bit is automatically set to 0. In order to return back to normal mode, the RESET_ARP bit has to be set to 1. 

## 9.2  THS_P_L (0Ch) 

User-defined threshold value for pressure interrupt event (least significant bits) (R/W)                

> 76543210
> THS7 THS6 THS5 THS4 THS3 THS2 THS1 THS0
> THS[7:0] This register contains the low part of threshold value for pressure interrupt generation. Default value: 00h

The threshold value for pressure interrupt generation is a 15-bit unsigned right-justified value composed of 

THS_P_H (0Dh)  and THS_P_L (0Ch).The value is expressed as: 

THS_P (15-bit unsigned) = Desired interrupt threshold (hPa) x 16 

To enable the interrupt event based on this user-defined threshold, the PHE bit or PLE bit (or both bits) in 

INTERRUPT_CFG (0Bh)  has to be enabled. 

## LPS22DF    

> Register description
> DS13316 -Rev 3 page 31/53

## 9.3  THS_P_H (0Dh) 

User-defined threshold value for pressure interrupt event (most significant bits) (R/W) 

7 6 5 4 3 2 1 0

- THS14  THS13  THS12  THS11  THS10  THS9  THS8 

THS[14:8]  This register contains the high part of threshold value for pressure interrupt generation. Refer to  THS_P_L (0Ch) .

Default value: 00h 

## 9.4  IF_CTRL (0Eh) 

Interface control register (R/W) 

7 6 5 4 3 2 1 0

INT_EN_I3C  I2C_I3C_DIS  SIM  SDA_PU_EN  SDO_PU_EN  INT_PD_DIS  CS_PU_DIS  -

INT_EN_I3C  Enable INT pin with MIPI I3C SM . If the INT_EN_I3C bit is set, the INT pin is polarized as OUT. Default value: 0 

(0: INT disabled with MIPI I3C SM ; 1: INT enabled with MIPI I3C SM )

I2C_I3C_DIS  Disable I²C and I3C digital interfaces. Default value: 0 

(0: enable I²C and I3C digital interfaces; 1: disable I²C and I3C digital interfaces) 

SIM  SPI serial interface mode selection. Default value: 0 

(0: 4-wire interface; 1: 3-wire interface) 

SDA_PU_EN  Enable pull-up on the SDA pin. Default value: 0 

(0: SDA pin pull-up disconnected; 1: SDA pin with pull-up) 

SDO_PU_EN  Enable pull-up on the SDO pin. Default value: 0 

(0: SDO pin pull-up disconnected; 1: SDO pin with pull-up) 

INT_PD_DIS  Disable pull-down on the INT pin. Default value: 0 

(0: INT pin with pull-down; 1: INT pin pull-down disconnected) 

CS_PU_DIS  Disable pull-up on the CS pin. Default value: 0 

(0: CS pin with pull-up; 1: CS pin pull-up disconnected) 

## 9.5  WHO_AM_I (0Fh) 

Device Who am I 

7 6 5 4 3 2 1 0

1 0 1 1 0 1 0 0

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 32/53

## 9.6  CTRL_REG1 (10h) 

Control register 1 (R/W) 

7 6 5 4 3 2 1 0

0 ODR3  ODR2  ODR1  ODR0  AVG2  AVG1  AVG0 

ODR[3:0]  Output data rate selection. Default value: 0000 

Refer to  Table 17 .

AVG[2:0]  Average selection. Default value: 000 

Refer to  Table 18 .

Table 17.  Output data rate bit configurations 

ODR[3:0]  ODR of pressure, temperature 

0000  Power-down / one-shot 

0001  1 Hz 

0010  4 Hz 

0011  10 Hz 

0100  25Hz 

0101  50 Hz 

0110  75 Hz 

0111  100 Hz 

1xxx  200 Hz 

Table 18.  Averaging selection 

AVG[2:0]  Averaging of pressure and temperature 

000  4

001  8

010  16 

011  32 

100  64 

101  128 

111  512 

The power consumption of the  LPS22DF  mainly depends on the selected ODR (output data rate) and on the selected resolution. The user can select the desired ODR and the oversampling frequency for pressure measurements in the CTRL_REG1 (10h) register. The ODR[3:0] bits are dedicated to the ODR selection, while the AVG[2:0] bits are used to configure the resolution. 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 33/53

The following table summarizes the supply current of all the ASIC resolution modes. 

Table 19.  Power consumption                                                                                 

> AVG
> One-shot mode Continuous mode – supply current (μA) vs. ODR
> Supply current (μA) @ 1 Hz ODR Max 1 Hz 4 Hz 10 Hz 25 Hz 50 Hz 75 Hz 100 Hz 200 Hz
> 512 32.2 25 32.8 126.8 314.4 783.8 ----
> 128 9.4 75 10 35.6 86.7 214.3 427 639.8 --
> 64 5.6 100 6.3 20.4 48.7 119.4 237.2 355 472.8 -
> 32 3.7 200 4.4 12.8 29.8 71.9 142.2 212.6 282.9 564.4
> 16 2.7 300 3.5 920.2 48.2 94.8 141.4 188 374
> 82400 2.7 612.6 29.1 56.5 84.2 111.5 221.7
> 41.7 500 2.5 510.2 23.2 44.7 66.2 87.8 174

The noise performance of  LPS22DF  is also defined as depending on the ODR and selected resolution and its performance is a trade-off between the power consumption and resolution. The noise performance is indicated in the following table. 

Table 20.  Noise performance                         

> AVG
> Pressure noise (P arms )
> ODR/2 (1) ODR/4 ODR/9
> 512 0.59 0.44 0.34
> 128 0.90 0.67 0.48
> 64 1.20 0.86 0.64
> 32 1.64 1.16 0.87
> 16 2.30 1.63 1.18
> 83.12 2.18 1.60
> 44.26 2.87 2.20
> 1. LPF1 filter is disabled.

When the ODR bits are set to 0000, the device is in power-down mode. When the device is in  power-down 

mode , almost all internal blocks of the device are switched off to minimize power consumption. The digital interface is still active to allow communication with the device. The content of the configuration registers is preserved and the output data registers are not updated, therefore keeping the last data sampled in memory before going into power-down mode. 

If the ONESHOT bit in  CTRL_REG2 (11h)  is set to 1,  one-shot mode  is triggered and a new acquisition starts when it is required. Enabling this mode is possible only if the device was previously in power-down mode (ODR bits set to 0000). Once the acquisition is completed and the output registers updated, the device automatically enters in power-down mode. ONESHOT bit self-clears itself. 

When the ODR bits are set to a value different than 0000, the device is in  continuous mode  and automatically acquires a set of data (pressure and temperature) at the frequency selected through the ODR[3:0] bits. 

## LPS22DF    

> Register description
> DS13316 -Rev 3 page 34/53

## 9.7  CTRL_REG2 (11h) 

Control register 2 (R/W) 

7 6 5 4 3 2 1 0

BOOT  0 LFPF_CFG  EN_LPFP  BDU  SWRESET  - ONESHOT 

BOOT  Reboot memory content. Default value: 0 

(0: normal mode; 1: reboot memory content) 

LFPF_CFG  Low-pass filter configuration. Default value: 0 

(0: ODR/4; 1: ODR/9) 

EN_LPFP  Enable low-pass filter on pressure data. Default value: 0 

(0: disable, 1: enable) 

BDU (1) 

Block data update. Default value: 0 

(0: continuous update; 

1: output registers not updated until MSB and LSB have been read) 

SWRESET 

Software reset. Default value: 0 

(0: normal mode; 1: software reset). 

The bit is self-cleared when the reset is completed. 

ONESHOT  Enable one-shot mode. Default value: 0 

(0: idle mode; 1: a new dataset is acquired) 

1.  To guarantee the correct behavior of the BDU feature,  PRESS_OUT_H (2Ah)  must be the last address read. 

The BOOT bit is used to refresh the content of the internal registers stored in the nonvolatile memory block. At device power-up, the content of the nonvolatile memory block is transferred to the internal registers related to the trimming functions to allow correct behavior of the device itself. If for any reason the content of the trimming registers is modified, it is sufficient to use this bit to restore the correct values. When the BOOT bit is set to 1, the content of the internal nonvolatile memory is copied into the corresponding internal registers and is used to calibrate the device. These values are factory trimmed and they are different for every device. They allow the correct behavior of the device and normally they should not be changed. At the end of the boot process, the BOOT bit is set again to 0 by hardware. The BOOT bit takes effect immediately after it is set to 1. 

The ONESHOT bit is used to start a new conversion when the ODR[3:0] bits in  CTRL_REG1 (10h)  are set to 0000. Writing a 1 to ONESHOT triggers a single measurement of pressure and temperature. Once the measurement is done, the ONESHOT bit self-clears, the new data are available in the output registers, and the 

STATUS (27h)  bits are updated. 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 35/53

## 9.8  CTRL_REG3 (12h) 

Control register 3 (R/W) 

7 6 5 4 3 2 1 0

0 0 0 0 INT_H_L  0 PP_OD  IF_ADD_INC 

INT_H_L  Select interrupt active-high, active-low. Default value: 0 

(0: active-high; 1: active-low) 

PP_OD  Push-pull/open-drain selection on interrupt pin. Default value: 0 

(0: push-pull; 1: open-drain) 

IF_ADD_INC  Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI). Default value: 1 

(0: disable, 1: enable) 

The INT_H_L bit selects an interrupt active-high/low value. 

The PP_OD bit selects push-pull/open-drain on the interrupt pin. 

The IF_ADD_INC bit enables the address to be automatically incremented during a multiple byte access with a serial interface (SPI or I²C). 

## 9.9  CTRL_REG4 (13h) 

Control register 4 (R/W) 

7 6 5 4 3 2 1 0

0 DRDY_PLS  DRDY  INT_EN  - INT_F_FULL  INT_F_WTM  INT_F_OVR 

DRDY_PLS (1)  Data-ready pulsed on INT pin. Default value: 0 

(0: disable; 1: enable data-ready pulsed on INT pin, pulse width around 5 μs) 

DRDY  Date-ready signal on INT pin. Default value: 0 

(0: disable; 1: enable) 

INT_EN  Interrupt signal on INT pin. Default value: 0 

(0: disable; 1: enable) 

INT_F_FULL  FIFO full flag on INT pin. Default value: 0 

(0: FIFO empty; 1: FIFO full with 128 unread samples) 

INT_F_WTM  FIFO threshold (watermark) status on INT pin. Default value: 0 

(0: FIFO is lower than WTM level; 1: FIFO is equal to or higher than WTM level) 

INT_F_OVR  FIFO overrun status on INT pin. Default value: 0 

(0: not overwritten; 1: at least one sample in the FIFO has been overwritten) 

1.  This bit is used together with the DRDY bit and it can be ignored if DRDY = 0. 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 36/53

Figure 23.  Interrupt events on INT pin 

> New data set is available FIFO Threshold (Watermark) FIFO Overrun FIFO Full Pressure higher than threshold Pressure lower than threshold PHE PLE INT_EN INT_F_FULL INT_F_OVR INT_F_WTM DRDY DRDY_PLS

INT Pin 

> CTRL_REG4 (13h) INTERRUPT_CFG (0Bh)

## 9.10  FIFO_CTRL (14h) 

FIFO control register (R/W)               

> 76543210
> 0000STOP_ON_WTM TRIG_MODES F_MODE1 F_MODE0

STOP_ON_WTM  Stop-on-FIFO watermark. Enables FIFO watermark level use. Default value: 0 

(0: disable; 1: enable) 

TRIG_MODES  Enables triggered FIFO modes. Default value: 0 

F_MODE[1:0]  Selects triggered FIFO modes. Default value: 00 

Refer to  Table 21 .

Table 21.  FIFO mode selection 

TRIG_MODES  F_MODE[1:]  Mode 

x 00  Bypass 

0 01  FIFO mode 

0 1x  Continuous (dynamic-stream) 

1 01  Bypass-to-FIFO 

1 10  Bypass-to-continuous (dynamic-stream) 

1 11  Continuous (dynamic-stream)-to-FIFO 

The STOP_ON_WTM bit enables the use of the FIFO watermark level: when the number of samples in FIFO is equal to the watermark level (set using the WTM[6:0] bits in  FIFO_WTM (15h) ) then FIFO is full. 

The TRIG_MODES bit enables the triggered FIFO modes. 

The F_MODE[1:0] bits select one of the FIFO modes, as described in  Table 21 .

Output pressure data are read through  FIFO_DATA_OUT_PRESS_XL (78h) , FIFO_DATA_OUT_PRESS_L (79h) ,and  FIFO_DATA_OUT_PRESS_H (7Ah) ; both single read and multiple read operations can be used. 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 37/53

## 9.11  FIFO_WTM (15h) 

FIFO threshold setting register (R/W) 

7 6 5 4 3 2 1 0

0 WTM6  WTM5  WTM4  WTM3  WTM2  WTM1  WTM0 

WTM[6:0]  FIFO threshold. Watermark level setting. Default value: 0000000 

## 9.12  REF_P_L (16h) 

Reference pressure LSB data (R) 

7 6 5 4 3 2 1 0

REFL7  REFL6  REFL5  REFL4  REFL3  REFL2  REFL1  REFL0 

REFL[7:0]  This register contains the low part of the reference pressure value. Default value: 00000000 

The reference pressure value is 16-bit data and it is composed of  REF_P_H (17h)  and REF_P_L (16h). The value is expressed as two's complement. 

The reference pressure value is stored and used when the AUTOZERO or AUTOREFP function is enabled. Refer to the  INTERRUPT_CFG (0Bh)  register description. 

## 9.13  REF_P_H (17h) 

Reference pressure MSB data (R) 

7 6 5 4 3 2 1 0

REFL15  REFL14  REFL13  REFL12  REFL11  REFL10  REFL9  REFL8 

REFL[15:8]  This register contains the high part of the reference pressure value. Default value: 00000000 

## 9.14  I3C_IF_CTRL_ADD (19h) 

Control register (R/W) 

7 6 5 4 3 2 1 0

1 0 ASF_ON  0 0 0 I3C_Bus_ 

Avb_Sel1 

I3C_Bus_ 

Avb_Sel0 

ASF_ON 

Enables antispike filters. Default value: 0 

(0: antispike filters are managed by protocol and turned off after the broadcast address; 

1: antispike filters on SCL and SDA lines are always enabled) 

I3C_Bus_Avb_Sel[1:0] 

These bits are used to select the bus available time when I3C IBI is used. Default value: 00 

(00: bus available time equal to 50 μsec; 

01: bus available time equal to 2 μsec; 

10: bus available time equal to 1 msec; 

11: bus available time equal to 25 msec) 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 38/53

## 9.15  RPDS_L (1Ah) 

Pressure offset (LSB data) 

7 6 5 4 3 2 1 0

RPDS7  RPDS6  RPDS5  RPDS4  RPDS3  RPDS2  RPDS1  RPDS0 

RPDS[7:0]  This register contains the low part of the pressure offset value. Default value: 00000000 

The pressure offset value is 16-bit data that can be used to implement one-point calibration (OPC) after soldering. This value is composed of  RPDS_H (1Bh)  and RPDS_L (1Ah). The value is expressed as two’s complement. 

The customer can perform a one-point calibration after soldering (recommended) and the offset coefficient can be stored for OPC in register RPDS (1Ah, 1Bh). These stored offset values are directly added to the compensated pressure data in the block diagram below. To give better flexibility to the user, the OPC value can be written twice in the same register map. For further details, refer to the application note. 

Figure 24.  One-point calibration 

## 9.16  RPDS_H (1Bh) 

Pressure offset (MSB data) 

7 6 5 4 3 2 1 0

RPDS15  RPDS14  RPDS13  RPDS12  RPDS11  RPDS10  RPDS9  RPDS8 

RPDS[15:8]  This register contains the high part of the pressure offset value. Default value: 00000000 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 39/53

## 9.17  INT_SOURCE (24h) 

Interrupt source (read only) register for differential pressure. A read at this address clears the INT_SOURCE register itself. 

7 6 5 4 3 2 1 0

BOOT_ON  0 0 0 0 IA  PL  PH 

BOOT_ON  Indication that the boot (reboot) phase is running. 

(0: boot phase not running; 1: boot phase is running) 

IA 

Interrupt active. 

(0: no interrupt has been generated; 

1: one or more interrupt events have been generated). 

PL 

Differential pressure low. 

(0: no interrupt has been generated; 

1: low differential pressure event has occurred). 

PH 

Differential pressure high. 

(0: no interrupt has been generated; 

1: high differential pressure event has occurred). 

## 9.18  FIFO_STATUS1 (25h) 

FIFO status register (read only) 

7 6 5 4 3 2 1 0

FSS7  FSS6  FSS5  FSS4  FSS3  FSS2  FSS1  FSS0 

FSS[7:0]  FIFO stored data level, number of unread samples stored in FIFO. 

(00000000: FIFO empty; 10000000: FIFO full, 128 unread samples) 

## 9.19  FIFO_STATUS2 (26h) 

FIFO status register (read only) 

7 6 5 4 3 2 1 0

FIFO_WTM_IA  FIFO_OVR_IA  FIFO_FULL_IA  - - - - -

FIFO_WTM_IA 

FIFO threshold (watermark) status. Default value: 0 

(0: FIFO filling is lower than threshold level; 

1: FIFO filling is equal or higher than threshold level). 

FIFO_OVR_IA 

FIFO overrun status. Default value: 0 

(0: FIFO is not completely full; 

1: FIFO is full and at least one sample in the FIFO has been overwritten). 

FIFO_FULL_IA 

FIFO full status. Default value: 0 

(0: FIFO is not completely filled; 

1: FIFO is completely filled, no samples overwritten) 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 40/53

## 9.20  STATUS (27h) 

Status register (read only) 

7 6 5 4 3 2 1 0

- - T_OR  P_OR  - - T_DA  P_DA 

T_OR 

Temperature data overrun. 

(0: no overrun has occurred; 

1: a new data for temperature has overwritten the previous data) 

P_OR 

Pressure data overrun. 

(0: no overrun has occurred; 

1: new data for pressure has overwritten the previous data) 

T_DA 

Temperature data available. 

(0: new data for temperature is not yet available; 

1: new temperature data is generated) 

P_DA 

Pressure data available. 

(0: new data for pressure is not yet available; 

1: new pressure data is generated) 

This register is updated every ODR cycle. 

## 9.21  PRESS_OUT_XL (28h) 

Pressure output value LSB data (read only) 

7 6 5 4 3 2 1 0

POUT7  POUT6  POUT5  POUT4  POUT3  POUT2  POUT1  POUT0 

POUT[7:0]  This register contains the low part of the pressure output value. 

The pressure output value is a 24-bit data that contains the measured pressure. It is composed of 

PRESS_OUT_H (2Ah) , PRESS_OUT_L (29h)  and PRESS_OUT_XL (28h). The value is expressed as two's complement. 

The output pressure register  PRESS_OUT  is provided as the difference between the measured pressure and the content of the register RPDS (1Ah, 1Bh). 

Refer to  Section 4.4:  Interpreting pressure readings  for additional information. 

## 9.22  PRESS_OUT_L (29h) 

Pressure output value middle data (read only) 

7 6 5 4 3 2 1 0

POUT15  POUT14  POUT13  POUT12  POUT11  POUT10  POUT9  POUT8 

POUT[15:8]  This register contains the middle part of the pressure output value. Refer to  PRESS_OUT_XL (28h) .

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 41/53

## 9.23  PRESS_OUT_H (2Ah) 

Pressure output value MSB data (read only) 

7 6 5 4 3 2 1 0

POUT23  POUT22  POUT21  POUT20  POUT19  POUT18  POUT17  POUT16 

POUT[23:16]  This register contains the high part of the pressure output value. Refer to  PRESS_OUT_XL (28h) .

## 9.24  TEMP_OUT_L (2Bh) 

Temperature output value LSB data (read only) 

7 6 5 4 3 2 1 0

TOUT7  TOUT6  TOUT5  TOUT4  TOUT3  TOUT2  TOUT1  TOUT0 

TOUT[7:0]  This register contains the low part of the temperature output value. 

The temperature output value is 16-bit data that contains the measured temperature. It is composed of 

TEMP_OUT_H (2Ch) , and TEMP_OUT_L (2Bh). The value is expressed as two’s complement. 

This register contains the temperature value and the resolution is: 1LSB = 0.01°C. 

## 9.25  TEMP_OUT_H (2Ch) 

Temperature output value MSB data (read only) 

7 6 5 4 3 2 1 0

TOUT15  TOUT14  TOUT13  TOUT12  TOUT11  TOUT10  TOUT9  TOUT8 

TOUT[15:8]  This register contains the high part of the temperature output value. 

## 9.26  FIFO_DATA_OUT_PRESS_XL (78h) 

FIFO pressure output LSB data (read only) 

7 6 5 4 3 2 1 0

FIFO_P7  FIFO_P6  FIFO_P5  FIFO_P4  FIFO_P3  FIFO_P2  FIFO_P1  FIFO_P0 

FIFO_P[7:0]  Pressure LSB data in FIFO buffer 

## 9.27  FIFO_DATA_OUT_PRESS_L (79h) 

FIFO pressure output middle data (read only) 

7 6 5 4 3 2 1 0

FIFO_P15  FIFO_P14  FIFO_P13  FIFO_P12  FIFO_P11  FIFO_P10  FIFO_P9  FIFO_P8 

FIFO_P[15:8]  Pressure middle data in FIFO buffer 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 42/53

## 9.28  FIFO_DATA_OUT_PRESS_H (7Ah) 

FIFO pressure output MSB data (read only) 

7 6 5 4 3 2 1 0

FIFO_P23  FIFO_P22  FIFO_P21  FIFO_P20  FIFO_P19  FIFO_P18  FIFO_P17  FIFO_P16 

FIFO_P[23:16]  Pressure MSB data in FIFO buffer 

## LPS22DF 

Register description    

> DS13316 -Rev 3 page 43/53

# 10  Package information 

To meet environmental requirements, ST offers these devices in different grades of  ECOPACK  packages, depending on their level of environmental compliance. ECOPACK specifications, grade definitions, and product status are available at:  www.st.com . ECOPACK is an ST trademark. 

## 10.1  HLGA-10L package information 

Figure 25.  HLGA-10L (2.0 x 2.0 x 0.73 mm typ.) package outline and mechanical dimensions 

L 

> W
> Pin 1

H

> 0.5  2x (0.1)

2x (0.1) 

> Sensing area

1

0.5 

> Pin 1

(10x) 0.25 ±0.05 

> (10x) 0.275 ±0.05

A

B

C

0.15 C A B

0.05 C

0.31 

> 0.24

0.91 

> 0.91

Dimensions are in millimeter unless otherwise speci fied 

General Tolerance is +/-0.1mm unless otherwise spec ified 

OUTER DIMENSIONS 

ITEM DIMENSION [mm] TOLERANCE [mm] 

1.0±2]L[htgneL

1.0±2]W[htdiW

/xam8.0]H[thgieH

DM00386636_1 

## LPS22DF 

Package information    

> DS13316 -Rev 3 page 44/53

## 10.2  HLGA-10L packing information 

Figure 26.  Carrier tape information for HLGA-10L package 

1.00 MIN 12.00 +- 0.3 0.1 1.50 + 0.1 0.0 2.00±0.05 SEE NOTE 2 4.00 SEE NOTE 1 1.75±0.10 4.00 5.50±0.05 SEE NOTE 2 

BBAA

SCALE 1:1 

Ko 0.30±0.05 R 0.20 MAX Bo 

SECTION B-B 

Ao 

SECTION A-A 

R0.25 

0.13 

0.13 

DETAIL D 

SCALE 6:1 

SCALE 12 : 1 DIM ±

Ao 2.20 0.05 

Bo 2.20 0.05 

Ko 1.00 0.10 

NOTES: 

10 SPROCKET HOLE PITCH CUMULATIVE TOLERANCE ±0.2 1. POCKET POSITION RELATIVE TO SPROCKET HOLE MEASURED AS TRUE POSITION OF POCKET, NOT 2. POCKET HOLE. 

Ao AND Bo ARE MEASURED ON A PLANE AT A DISTANCE "R" ABOVE THE BOTTOM OF THE POCKET. 3. 

Figure 27.  HLGA-10L package orientation in carrier tape 

## LPS22DF 

Package information 

DS13316 - Rev 3 page 45/53 Figure 28.  Reel information for carrier tape of HLGA-10L package 

A D

B 

> Full radius

Tape slot in core for tape start 2.5mm min. width G measured at hub CN40mm min. Access hole at slot location T

Table 22.  Reel dimensions for carrier tape of HLGA-10L package 

Reel dimensions (mm) 

A (max)  330 

B (min)  1.5 

C 13 ±0.25 

D (min)  20.2 

N (min)  60 

G 12.4 +2/-0 

T (max)  18.4 

## LPS22DF 

Package information    

> DS13316 -Rev 3 page 46/53

## Revision history 

Table 23.  Document revision history 

Date  Version  Changes 

30-Sep-2021  1 Initial release 

8-Jun-2023  2 Updated Features 

Updated Table 2. Pressure and temperature sensor characteristics 

30-Apr-2025  3 Updated  Table 4.  DC characteristics 

## LPS22DF    

> DS13316 -Rev 3 page 47/53

# Contents 

## 1 Block diagrams  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 2

## 2 Pin description  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 3

## 3 Mechanical and electrical specifications  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  4

3.1  Mechanical characteristics  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  4

3.2  Electrical characteristics . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  5

3.3  Communication interface characteristics  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  6      

> 3.3.1 SPI - serial peripheral interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 6
> 3.3.2 I²C - inter-IC control interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 7

3.4  Absolute maximum ratings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  8

## 4 Functionality  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 9

4.1  Sensing element  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  9

4.2  IC interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  9

4.3  Factory calibration . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  9

4.4  Interpreting pressure readings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  10 

4.5  Interpreting temperature readings  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  11 

## 5 FIFO . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 12 

5.1  Bypass mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  13 

5.2  FIFO mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  14 

5.3  Continuous (dynamic-stream) mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  15 

5.4  Bypass-to-FIFO mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  16 

5.5  Bypass-to-continuous (dynamic-stream) mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  17 

5.6  Continuous (dynamic-stream)-to-FIFO mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  18 

5.7  Retrieving data from FIFO  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  18 

## 6 Application hints  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 19 

6.1  Soldering information  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  20 

## 7 Digital interfaces  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 21 

7.1  Serial interfaces . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  21 

7.2  I²C serial interface (CS = high)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  21    

> 7.2.1 I²C operation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 22

7.3  SPI bus interface (CS = low)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  24          

> 7.3.1 SPI read . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 25
> 7.3.2 SPI write . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 26
> 7.3.3 SPI read in 3-wire mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 26

7.4  MIPI I3C SM  slave interface . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  26 

## LPS22DF          

> Contents
> DS13316 -Rev 3 page 48/53 7.4.1 MIPI I3C SM CCC supported commands . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 27
> 7.4.2 Overview of antispike filter management . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 28

## 8 Register mapping . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 29 

## 9 Register description  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 30 

9.1  INTERRUPT_CFG (0Bh)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  30 

9.2  THS_P_L (0Ch)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  31 

9.3  THS_P_H (0Dh)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  32 

9.4  IF_CTRL (0Eh)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  32 

9.5  WHO_AM_I (0Fh)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  32 

9.6  CTRL_REG1 (10h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  33 

9.7  CTRL_REG2 (11h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  35 

9.8  CTRL_REG3 (12h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  36 

9.9  CTRL_REG4 (13h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  36 

9.10  FIFO_CTRL (14h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  37 

9.11  FIFO_WTM (15h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  38 

9.12  REF_P_L (16h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  38 

9.13  REF_P_H (17h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  38 

9.14  I3C_IF_CTRL_ADD (19h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  38 

9.15  RPDS_L (1Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  39 

9.16  RPDS_H (1Bh)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  39 

9.17  INT_SOURCE (24h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  40 

9.18  FIFO_STATUS1 (25h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  40 

9.19  FIFO_STATUS2 (26h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  40 

9.20  STATUS (27h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  41 

9.21  PRESS_OUT_XL (28h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  41 

9.22  PRESS_OUT_L (29h) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  41 

9.23  PRESS_OUT_H (2Ah)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  42 

9.24  TEMP_OUT_L (2Bh)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  42 

9.25  TEMP_OUT_H (2Ch)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  42 

9.26  FIFO_DATA_OUT_PRESS_XL (78h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  42 

9.27  FIFO_DATA_OUT_PRESS_L (79h)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  42 

9.28  FIFO_DATA_OUT_PRESS_H (7Ah) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  43 

## 10  Package information . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 44 

10.1  HLGA-10L package information  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  44 

10.2  HLGA-10L packing information  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  45 

## LPS22DF    

> Contents
> DS13316 -Rev 3 page 49/53

## Revision history  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 47 

## List of tables  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 51 

## List of figures . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 52 

## LPS22DF    

> Contents
> DS13316 -Rev 3 page 50/53

# List of tables 

Table 1.  Pin description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  3

Table 2.  Pressure and temperature sensor characteristics  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  4

Table 3.  Electrical characteristics  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  5

Table 4.  DC characteristics  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  5

Table 5.  SPI slave timing values . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  6

Table 6.  I²C slave timing values  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  7

Table 7.  Absolute maximum ratings  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  8

Table 8.  Serial interface pin description  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  21 

Table 9.  I²C terminology  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  21 

Table 10.  SAD+read/write patterns  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  22 

Table 11.  Transfer when master is writing one byte to slave . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  22 

Table 12.  Transfer when master is writing multiple bytes to slave  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  22 

Table 13.  Transfer when master is receiving (reading) one byte of data from slave  . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  22 

Table 14.  Transferwhen master is receiving (reading) multiple bytes of data from slave  . . . . . . . . . . . . . . . . . . . . . . . . . .  22 

Table 15.  MIPI I3C SM  CCC commands  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  27 

Table 16.  Registers address map  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  29 

Table 17.  Output data rate bit configurations  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  33 

Table 18.  Averaging selection  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  33 

Table 19.  Power consumption  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  34 

Table 20.  Noise performance . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  34 

Table 21.  FIFO mode selection  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  37 

Table 22.  Reel dimensions for carrier tape of HLGA-10L package  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  46 

Table 23.  Document revision history  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  47 

## LPS22DF    

> List of tables
> DS13316 -Rev 3 page 51/53

# List of figures 

Figure 1.  Device architecture block diagram . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  2

Figure 2.  Digital logic  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  2

Figure 3.  Pin connections (bottom view)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  3

Figure 4.  SPI slave timing diagram  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  6

Figure 5.  I²C slave timing diagram  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  7

Figure 6.  Pressure readings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  10 

Figure 7.  Temperature readings  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  11 

Figure 8.  Bypass mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  13 

Figure 9.  FIFO mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  14 

Figure 10.  Continuous (dynamic-stream) mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  15 

Figure 11.  Bypass-to-FIFO mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  16 

Figure 12.  Bypass-to-continuous (dynamic-stream) mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  17 

Figure 13.  Continuous (dynamic-stream)-to-FIFO mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  18 

Figure 14.  LPS22DF  electrical connections (top view)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  19 

Figure 15.  LPS22DF  power-off sequence  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  20 

Figure 16.  Read and write protocol  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  24 

Figure 17.  SPI read protocol  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  25 

Figure 18.  Multiple byte SPI read protocol (2-byte example) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  25 

Figure 19.  SPI write protocol  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  26 

Figure 20.  Multiple byte SPI write protocol (2-byte example)  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  26 

Figure 21.  SPI read protocol in 3-wire mode  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  26 

Figure 22.  "Threshold-based" interrupt event  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  30 

Figure 23.  Interrupt events on INT pin  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  37 

Figure 24.  One-point calibration  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  39 

Figure 25.  HLGA-10L (2.0 x 2.0 x 0.73 mm typ.) package outline and mechanical dimensions  . . . . . . . . . . . . . . . . . . . .  44 

Figure 26.  Carrier tape information for HLGA-10L package  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  45 

Figure 27.  HLGA-10L package orientation in carrier tape . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  45 

Figure 28.  Reel information for carrier tape of HLGA-10L package  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .  46 

## LPS22DF    

> List of figures
> DS13316 -Rev 3 page 52/53

IMPORTANT NOTICE – READ CAREFULLY 

STMicroelectronics NV and its subsidiaries (“ST”) reserve the right to make changes, corrections, enhancements, modifications, and improvements to ST products and/or to this document at any time without notice. Purchasers should obtain the latest relevant information on ST products before placing orders. ST products are sold pursuant to ST’s terms and conditions of sale in place at the time of order acknowledgment. 

Purchasers are solely responsible for the choice, selection, and use of ST products and ST assumes no liability for application assistance or the design of purchasers’ products. 

No license, express or implied, to any intellectual property right is granted by ST herein. 

Resale of ST products with provisions different from the information set forth herein shall void any warranty granted by ST for such product. 

ST and the ST logo are trademarks of ST. For additional information about ST trademarks, refer to  www.st.com/trademarks . All other product or service names are the property of their respective owners. 

Information in this document supersedes and replaces information previously supplied in any prior versions of this document. 

© 2025 STMicroelectronics – All rights reserved 

## LPS22DF 

DS13316 - Rev 3 page 53/53
