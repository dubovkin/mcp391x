#ifndef _MCP39XX_H
#define _MCP39XX_H

#define ADC_CHn_DATA_LENGTH(n) ((n * 6) / 8)
#define ADC_CHANELL 6

#define MCP391x_READ_REG(n)  (BIT(6) | ((n) << 1) | BIT(0))
#define MCP391x_WRITE_REG(n) (BIT(6) | ((n) << 1));

/* STATUSCOM REGISTER */
#define MCP391x_STATUSCOM_WIDTH_DATA_24 BIT(16)
#define MCP391x_STATUSCOM_WIDTH_DATA_32 BIT(17)
#define MCP391x_STATUSCOM_WIDTH_DATA_32_SIGN BIT(17) | BIT(16)
#define MCP391x_STATUSCOM_WIDTH_CRC_16  BIT(18)
#define MCP391x_STATUSCOM_DR_LINK       BIT(19)
#define MCP391x_STATUSCOM_DR_HIZ_HIGH   BIT(20)
#define MCP391x_STATUSCOM_DR_HIZ_LOW    ~BIT(20)
#define MCP391x_STATUSCOM_READ_GROUPS   BIT(22)
#define MCP391x_STATUSCOM_READ_TYPES    BIT(23)

/* CONFIG0 REGISTER */
#define MCP391x_CONFIG0_DIFER(n)  ((n) << 20)
#define MCP391x_CONFIG0_BOOST(n)  ((n) << 18)
#define MCP391x_CONFIG0_AMCLK(n)  BIT(16)
#define MCP391x_CONFIG0_OSR_256   (3 << 13)
#define MCP391x_CONFIG0_OSR_512   (4 << 13)
#define MCP391x_CONFIG0_OSR_1024  (5 << 13)
#define MCP391x_CONFIG0_VREFCAL   (5 << 4)

/* CONFIG1 REGISTER */
#define MCP391x_CONFIG1_VREFEXT_EN    (0 << 7)
#define MCP391x_CONFIG1_CLKEXT_EN     (0 << 6)
#define MCP391x_CONFIG1_SHUTDOWN_CH0  BIT(8)
#define MCP391x_CONFIG1_SHUTDOWN_CH1  BIT(9)
#define MCP391x_CONFIG1_SHUTDOWN_CH2  BIT(10)
#define MCP391x_CONFIG1_SHUTDOWN_CH3  BIT(11)
#define MCP391x_CONFIG1_SHUTDOWN_CH4  BIT(12)
#define MCP391x_CONFIG1_SHUTDOWN_CH5  BIT(13)

/* GAIN REGISTER */
#define MCP39xx_GAIN_PGA_CH0(n) ((n) << 0)
#define MCP39xx_GAIN_PGA_CH1(n) ((n) << 3)
#define MCP39xx_GAIN_PGA_CH2(n) ((n) << 6)
#define MCP39xx_GAIN_PGA_CH3(n) ((n) << 9)
#define MCP39xx_GAIN_PGA_CH4(n) ((n) << 12)
#define MCP39xx_GAIN_PGA_CH5(n) ((n) << 15)

#define MCP39xx_LOCK_CRC_LOCK(n)   ((n) << 16))
#define MCP39xx_LOCK_CRC_CRCREG(n) ((n) << 0)) 

/* MCP3913 REGISTER MAP */
#define MCP39xx_CH_0        0x0
#define MCP39xx_CH_1        0x1
#define MCP39xx_CH_2        0x2
#define MCP39xx_CH_3        0x3
#define MCP39xx_CH_4        0x4
#define MCP39xx_CH_5        0x5
#define MCP39xx_CH_6        0x6
#define MCP39xx_CH_7        0x7
#define MCP39xx_MOD         0x8
#define MCP39xx_PHASE0      0x9
#define MCP39xx_PHASE1      0xA
#define MCP39xx_GAIN        0xB
#define MCP39xx_STATUSCOM   0xC
#define MCP39xx_CONFIG0     0xD
#define MCP39xx_CONFIG1     0xE
#define MCP39xx_OFFCAL_CH0  0xF
#define MCP39xx_GAINCAL_CH0 0x10
#define MCP39xx_OFFCAL_CH1  0x11
#define MCP39xx_GAINCAL_CH1 0x12
#define MCP39xx_OFFCAL_CH2  0x13
#define MCP39xx_GAINCAL_CH2 0x14
#define MCP39xx_OFFCAL_CH3  0x15
#define MCP39xx_GAINCAL_CH3 0x16
#define MCP39xx_OFFCAL_CH4  0x17
#define MCP39xx_GAINCAL_CH4 0x18
#define MCP39xx_OFFCAL_CH5  0x19
#define MCP39xx_GAINCAL_CH5 0x1A
#define MCP39xx_OFFCAL_CH6  0x1B
#define MCP39xx_GAINCAL_CH6 0x1C
#define MCP39xx_OFFCAL_CH7  0x1D
#define MCP39xx_GAINCAL_CH7 0x1E
#define MCP39xx_LOCK_CRC    0x1F

struct mcp391x_state {
	const struct mcp391x_chip_info	*chip_info;
	struct spi_device	            *spi;
	struct iio_trigger	            *trig;
	struct regulator		        *reg;
	u8				                tx[4] ____cacheline_aligned;
	u8				                rx[4];
	u32				                data[ADC_CHANELL];
	struct mutex                    lock;
	u8							    clk_ext;
	u8				                *buffer;
	u8 buf[144];
};

#endif
