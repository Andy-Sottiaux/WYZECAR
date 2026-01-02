================================================================================
J6 — I2C, UART & ENET MDIO HEADER
20-pin, 2x10, 2.54mm
================================================================================

Pin 1  : BT_UART4_TX        | IO | UART4 Transmit
Pin 2  : FTDI_RTSN          | O  | FTDI RTS, Active Low
Pin 3  : BT_UART4_CTS_B     | IO | UART4 CTS
Pin 4  : FTDI_RXI           | IO | UART2 Transmit (FTDI layout)
Pin 5  : BT_UART4_RX        | IO | UART4 Receive
Pin 6  : FTDI_TXO           | IO | UART2 Receive (FTDI layout)
Pin 7  : BT_UART4_RTS_B     | IO | UART4 RTS
Pin 8  : NC
Pin 9  : BASE_PER_3V3       | P  | Base board 3.3V
Pin 10 : FTDI_CTSN          | IO | FTDI CTS, Active Low
Pin 11 : UART3_RXD          | IO | UART3 Receive
Pin 12 : GND                | P  | Digital Ground
Pin 13 : UART3_TXD          | IO | UART3 Transmit
Pin 14 : ENET_MDIO          | IO | Ethernet MDIO Data
Pin 15 : GND                | P  | Digital Ground
Pin 16 : ENET_MDC           | O  | Ethernet MDIO Clock
Pin 17 : I2C4_SCL           | IO | I2C #4 Clock
Pin 18 : I2C3_SCL           | IO | I2C #3 Clock
Pin 19 : I2C4_SDA           | IO | I2C #4 Data
Pin 20 : I2C3_SDA           | IO | I2C #3 Data

Notes:
- UART4 used on SOMs with WBD/WBE assemblies
- MDIO runs at 2.5V on SOMs with EC assembly
- UART2 follows FTDI pin convention
- I2C3 has 10k pull-ups on SOM


================================================================================
J7 — SAI2 / SAI5 / CAN-FD HEADER
20-pin, 2x10, 2.54mm
================================================================================

Pin 1  : CAN_H              | IO | CAN Bus High
Pin 2  : CAN_L              | IO | CAN Bus Low
Pin 3  : BASE_PER_1V8       | P  | Base board 1.8V
Pin 4  : GND                | P  | Digital Ground
Pin 5  : SAI2_RXC           | IO | SAI2 Receive Bit Clock
Pin 6  : PMIC_STBY_REQ      | O  | PMIC Standby Request (Hi-Z if unused)
Pin 7  : SAI2_RXFS          | IO | SAI2 Receive Frame Sync
Pin 8  : SAI5_RXC           | IO | SAI5 Receive Bit Clock
Pin 9  : SAI2_RXD0          | IO | SAI2 Receive Data 0
Pin 10 : SAI5_RXFS          | IO | SAI5 Receive Frame Sync
Pin 11 : SAI2_TXC           | IO | SAI2 Transmit Bit Clock
Pin 12 : SAI5_RXD0          | IO | SAI5 Receive Data 0
Pin 13 : SAI2_TXFS          | IO | SAI2 Transmit Frame Sync
Pin 14 : SAI5_RXD1          | IO | SAI5 Receive Data 1
Pin 15 : SAI2_TXD0          | IO | SAI2 Transmit Data 0
Pin 16 : SAI5_RXD2          | IO | SAI5 Receive Data 2
Pin 17 : SAI2_MCLK          | IO | SAI2 Master Clock
Pin 18 : SAI5_RXD3          | IO | SAI5 Receive Data 3
Pin 19 : SOM_VDD_PHY_1V8    | P  | Programmable PMIC LDO (see notes)
Pin 20 : SAI5_MCLK          | IO | SAI5 Master Clock

Notes:
- Signal routing varies by SOM
- Some SAI lines shared with CAN-FD on MX8M-PLUS
- SOM_VDD_PHY_1V8 is HDMI PHY power on MX8M-PLUS


================================================================================
J8 — GPIO & SPDIF HEADER
10-pin, 2x5, 2.54mm
================================================================================

Pin 1  : BASE_PER_3V3       | P  | Base board 3.3V
Pin 2  : GPIO1_IO11         | IO | GPIO
Pin 3  : SPDIF_RX           | IO | SPDIF Receive Data
Pin 4  : GPIO1_IO12         | IO | GPIO
Pin 5  : SPDIF_EXT_CLK      | IO | SPDIF External Clock
Pin 6  : GPIO1_IO08         | IO | GPIO
Pin 7  : SPDIF_TX           | IO | SPDIF Transmit Data
Pin 8  : GPIO1_IO15         | IO | GPIO (RTC_IRQn on some SOMs)
Pin 9  : GND                | P  | Digital Ground
Pin 10 : GPIO1_IO06         | IO | GPIO

Notes:
- SPDIF_RX and SPDIF_TX share routing with CAN-FD via resistors
- GPIO1_IO15 connects to RTC interrupt on some SOMs
