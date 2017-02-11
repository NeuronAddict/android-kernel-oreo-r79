/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of Intel nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Intel MonetteHill PVT Sensor Thermal driver
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <linux/timer.h>
#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-scu.h>

#define MNH_BAD_ADDR ((void *)0xFFFFFFFF)
#define MNH_NUM_PVT_SENSORS 4

/* Thermal characterization data */
#define API_TRIM_CODE		0xF
#define API_PRECISION_CODE	0x0

/* Thermal efuse data */
#define NUM_BITS_EFUSE	10
#define API_BITS_SLOPE	5
#define API_BITS_OFFSET	(NUM_BITS_EFUSE-API_BITS_SLOPE)

/* Temperature calculation algorithm parameters */
#define API_POLY_N4 (-16743)  /* -1.6743e-11 : 15bits excluding sign */
#define API_POLY_N3 (+81542)  /* +8.1542e-08 : 17bits excluding sign */
#define API_POLY_N2 (-18201)  /* -1.8201e-04 : 15bits excluding sign */
#define API_POLY_N1 (+31020)  /* +3.1020e-01 : 15bits excluding sign */
#define API_POLY_N0 (-48380)  /* -4.8380e+01 : 16bits excluding sign */

#define N3_E15_MULTIPLIER 1000 /* 1e3*/
#define N2_E15_MULTIPLIER 10000000 /* 1e7 */
#define N1_E15_MULTIPLIER 10000000000 /* 1e10 */
#define N0_E15_MULTIPLIER 1000000000000 /* 1e12 */

#define API_RES_SLOPE  1  /* 0.000010 */
#define API_RES_OFFSET 1  /* 0.001000 */

#define RES_SLOPE_DIVIDER 100000 /* 1e5 */
#define RES_OFFSET_E15_MULTIPLIER 1000000000000 /* 1e12 */

#define API_BITS_SLOPE_MASK    0x1F
#define API_BITS_OFFSET_MASK   0x1F

#define N12_DIVIDER  1000000000000 /* 1e12 */
#define N12_ROUNDING_NUM 500000000000 /* 5e11 */

/* PVT Data conversion wait time calculation
 * PVT_FREQ: 1.15MHZ
 * PVT_WAIT_CYCLE: 376 + marginal cycles
 * PVT_WAIT_TIME: ((1/PVT_FREQ)*PVT_WAIT_CYCLE*PRECISION)
 * Average samples: (1<<(PRECISION*2))
 * PVT data conversion time:
 *   temp/vol mode: 376 PVT_SENSOR_CLK cycles
 *   process mode: 4 PVT_SENSOR_CLK cycles
 */
#define PVT_FREQ		1150
#define PVT_FREQ_UNIT   1000000
#define PVT_CYCLE		(PVT_FREQ_UNIT/PVT_FREQ)
#define PVT_WAIT_CYCLE	390
#define PVT_WAIT_MS		(((PVT_CYCLE*PVT_WAIT_CYCLE* \
	(1<<(API_PRECISION_CODE*2)))/PVT_FREQ_UNIT)+1)

/* PVT debug messages definition */
#define MNH_THERM_DBG 1
#if MNH_THERM_DBG
#define mnh_debug pr_err
#else
#define mnh_debug pr_debug
#endif

struct mnh_thermal_sensor {
	struct mnh_thermal_device *dev;
	struct thermal_zone_device *tzd;
	uint32_t id;
	uint32_t alarm_temp;
};

struct mnh_thermal_device {
	struct reset_control *reset;
	void __iomem *regs;
	uint32_t emulation;
	uint32_t wait_time_ms;
	uint32_t init_done;
	struct mnh_thermal_sensor *sensors[MNH_NUM_PVT_SENSORS];
};

/* PVT Sensor operation mode */
enum mnh_thermal_op_mode {
	THERMAL_OPMODE_TEMP, /* Temperature evaluation */
	THERMAL_OPMODE_VOL,  /* Voltage evaluation */
	THERMAL_OPMODE_LVT,  /* Process LVT evaluation */
	THERMAL_OPMODE_HVT,  /* Process HVT evaluation */
	THERMAL_OPMODE_SVT,  /* Process SVT evaluation */
};

struct mnh_thermal_op_param {
	int vsample;
	int psample;
};

static struct mnh_thermal_op_param mnh_op_mode_table[] = {
	{0, 0}, /* temperature: vsample b0, psample b00 */
	{1, 0}, /* voltage: vsample b1, psample b00 */
	{0, 1}, /* LVT: vsample b0, psample b01 */
	{0, 2}, /* HVT: vsample b0, psample b10 */
	{0, 3}  /* SVT: vsample b0, psample b11 */
};

static uint32_t read_emulation_setting(void)
{
	uint32_t val;
	struct device_node *node =
		of_find_node_by_name(NULL, "chosen");

	if (node && !of_property_read_u32(node, "emulation", &val))
		return val;
	else
		return 0;
}

/*
 * Calculate slope and offset value from EFUSE DTS values
 *
 * Slope = signedBin2DeC( Bits[9-m] ) * API_Res_slope
 * Offset = signedBin2DeC( Bits[(m-1)-0] ) * API_Res_offset
 * m : API_BITS_SLOPE
 */
static void read_efuse_trim(struct mnh_thermal_device *dev)
{
	struct device_node *np, *child;
	uint32_t i=0, val;
	int slope, offset;

	/* Read thermal zone configuration */
	np = of_find_node_by_name(NULL, "thermal-zones");
	if (!np)
		return; /* Not able to find */

	for_each_available_child_of_node(np, child) {
		if (!of_property_read_u32(child, "dts_trim", &val)){
			/* Make sure code is 10bit */
			val = val & 0x3ff;

			/* Calculate slope and offset */
			slope =  ((unsigned)val >> API_BITS_OFFSET) &
				API_BITS_SLOPE_MASK;
			if (slope >= 16) slope = slope - 32;

			offset = val & API_BITS_OFFSET_MASK;
			if (offset >= 16) offset = offset - 32;

			mnh_debug("pvt%d efuse:0x%x, s:%d, o:%d\n",
				i, val, slope, offset);

			dev->sensors[i]->tzd->tzp->slope = slope;
			dev->sensors[i]->tzd->tzp->offset = offset;
		}
		i++;
	}

	return;
}

/*
 * Caculate PVT_DATA output to millicelsius.
 *
 * Step #1 : Calculate Code to DegC using ideal equation.
 * DegC= API_Poly_N4 * Code^4 + API_Poly_N3 * Code^3 + API_Poly_N2 * Code^2
 *       + API_Poly_N1 * Code^1 + API_Poly_N0
 * Step #2 : Calculate Error using slope and offset.
 * Error= Slope * DegC + Offset
 * Step #3 : Calculate Final_temp by removing the above error from DegC
 * Final_Temp =  DegC - Error
 *
 * Note: kernel doesn't support floaing point calculation, so convert all the
 * predefined floating number to 64bit number. All variables and predefined
 * constants with an underscore "_" carry an implicit E-15 factor.
 */
static int calculate_temp(int code, int slope, int offset)
{
	long n1_, n2_, n3_, n4_, n0_, n01234_, final_temp_, error_;
	int temp_ms;


	mnh_debug("temp code:%d, slope:%d, offset:%d",
		code, slope, offset);

	/* Make sure code is 10bit */
	code = code & 0x3FF;

	/* Calculate DegC
	 * n4: 55bits
	 * n3: 57bits
	 * n2: 55bits
	 * n1: 56bits
	 * n0: 59bits
	 */
	n4_ = (long)code*code*code*code * API_POLY_N4;
	n3_ = (long)code*code*code * API_POLY_N3 * N3_E15_MULTIPLIER;
	n2_ = (long)code*code * API_POLY_N2 * N2_E15_MULTIPLIER;
	n1_ = (long)code * API_POLY_N1 * N1_E15_MULTIPLIER;
	n0_ = API_POLY_N0 * N0_E15_MULTIPLIER;
	n01234_ = n4_ + n3_ + n2_ + n1_ + n0_;

	/* n01234_ is 59bit, max slope is 5bit, which may overflow
	 * when multiplied without first down-scaling.
	 */
	error_ = n01234_/ RES_SLOPE_DIVIDER * slope * API_RES_SLOPE
		+ (long)offset * RES_OFFSET_E15_MULTIPLIER * API_RES_OFFSET;

	final_temp_ = n01234_ - error_;
	temp_ms = (final_temp_ + N12_ROUNDING_NUM) / N12_DIVIDER;
	mnh_debug("temp n01234:%ld, err:%ld\n", n01234_, error_);

	return temp_ms;
}

/* Read raw temperature data from sensors
 * Enable 1.2MHz Clock to PVT Sensor
 * Wait for 2 msecs for the clock to PVT sensor to start ticking
 * Program VSAMPLE/PSAMPLE
 * Enable PVT sensor access
 * Wait for conversion cycle time or PVT_DATA.DATAVALID or
 * wait for SCU interrupt
 * Read PVT_DATA[x].DATA_OUT[9:0]
 * Disable PVT sensor access
 * Reset PVT_DATA.DATAVALID
 * Disable 1.2MHz clock to PVT sensor
*/
static int mnh_thermal_get_data(void *data, int *data_out)
{

	struct mnh_thermal_sensor *sensor = (struct mnh_thermal_sensor*)data;
	u32 val = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	u32 op_mode, psample, vsample;

	if(sensor->dev->emulation != 0){ /* Not silicon */

		/* For emulation, below data_out values are sampled from
		 * temperature translation table of PVT sensor datasheet
		 */
		switch(sensor->id){
			case 0:
				val = 254;
				break;
			case 1:
				val = 276;
				break;
			case 2:
				val = 677;
				break;
			case 3:
				val = 813;
				break;
			default:
				val = 849;
				break;
		}
	} else { /* silicon */
		op_mode = sensor->tzd->tzp->op_mode;
		psample = mnh_op_mode_table[op_mode].psample;
		vsample = mnh_op_mode_table[op_mode].vsample;
		mnh_debug("pvt opmode:%d, vsample:%d, psample:%d\n",
			op_mode, vsample, psample);

		/* Program VSAMPLE/PSAMPLE for temperature evaulation */
		HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id,
			PSAMPLE, psample);
		HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id,
			VSAMPLE, vsample);

		/* Enable PVT sensor access */
		HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id,
			ENA, 1);

		/* Wait until PVT data conversion is finished in
		 * temperature and voltage mode.
		 * Conversion time: 376 PVT_SENSOR_CLK cycles
		 */
		do {
			val = HW_INxf(sensor->dev->regs, SCU, PVT_DATA,
				sensor->id, DATAVALID);
			if(val == 1)
				break;
			mdelay(sensor->dev->wait_time_ms);
		} while (time_before(jiffies, timeout));

		/* Read DATA_OUT from sensor */
		val = HW_INxf(sensor->dev->regs, SCU, PVT_DATA,
			sensor->id, DATA);

		/* Disable PVT sensor access */
		HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id,
			ENA, 0);

		/* Reset DATAVALID bit */
		HW_OUTxf(sensor->dev->regs, SCU, PVT_DATA, sensor->id,
			DATAVALID, 1);
	}

	*data_out = val;

	return 0;
}

/*
 * This will read raw_temp_code and convert to DecC temperature.
 */
static int mnh_thermal_get_temp(void *data, int *temp_out)
{
	struct mnh_thermal_sensor *sensor = (struct mnh_thermal_sensor*)data;
	int temp_raw = 0;

	if (!sensor->dev->init_done)
		return 0;

	/* Apply 5 bit trim and 2 bit precision to PVT sensor register */
	HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id,
	       TRIM, API_TRIM_CODE);
	HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id,
	       PRECISION, API_PRECISION_CODE);

	/* Get raw temp code */
	sensor->tzd->tzp->op_mode = THERMAL_OPMODE_TEMP;
	mnh_thermal_get_data(data, &temp_raw);

	/* Convert raw code to milli DecC */
	*temp_out = calculate_temp(temp_raw, sensor->tzd->tzp->slope,
			sensor->tzd->tzp->offset);

	return 0;
}

static const struct thermal_zone_of_device_ops mnh_of_thermal_ops = {
	.get_temp = mnh_thermal_get_temp,
	.get_data_out = mnh_thermal_get_data,
};


static int mnh_thermal_probe(struct platform_device *pdev)
{
	struct mnh_thermal_device *mnh_dev;
	struct resource *res;
	unsigned int i;
	int err;

	mnh_dev = devm_kzalloc(&pdev->dev, sizeof(*mnh_dev),
			GFP_KERNEL);
	if (!mnh_dev)
		return -ENOMEM;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("cannot get platform resources\n");
		return -ENOENT;
	}

	mnh_dev->regs = ioremap_nocache(res->start, resource_size(res));
	if (!mnh_dev->regs) {
		pr_err("unable to remap resources\n");
		return -ENOMEM;
	}

	mnh_dev->init_done = 0;

	/* Enable 1.2MHz clock to PVT sensor and wait for 2usecs
	 * until the clock is ticking
	 */
	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PVT_CLKEN, 1);
	udelay(2);

	/* Read target settings */
	mnh_dev->emulation = read_emulation_setting();

	/* Calculate PVT data read wait time */
	mnh_dev->wait_time_ms = PVT_WAIT_MS;
	mnh_debug("pvt wait time: %d\n", mnh_dev->wait_time_ms);

	/* Initialize thermctl sensors */
	for (i = 0; i < ARRAY_SIZE(mnh_dev->sensors); ++i) {
		struct mnh_thermal_sensor *sensor =
			devm_kzalloc(&pdev->dev, sizeof(*sensor),
			GFP_KERNEL);

		sensor->dev = mnh_dev;
		sensor->id = i;
		sensor->tzd = thermal_zone_of_sensor_register(&pdev->dev, i,
				sensor, &mnh_of_thermal_ops);

		if (IS_ERR(sensor->tzd)) {
			err = PTR_ERR(sensor->tzd);
			pr_err("failed to register sensor: %d\n", err);
			goto unregister_sensors;
		}

		mnh_dev->sensors[i] = sensor;
	}

	mnh_dev->init_done = 1;

	dev_info(&pdev->dev, "init done\n");

	/* TODO: configure sensors with characterized TRIM data
	 * and dts trimmed config data from efuse.
	 */
	read_efuse_trim(mnh_dev);

	/* TODO: Enable Thermal alarm */

	platform_set_drvdata(pdev, mnh_dev);

	return 0;

unregister_sensors:
	while (i--)
		thermal_zone_of_sensor_unregister(&pdev->dev,
			mnh_dev->sensors[i]->tzd);


	return err;
}


static int mnh_thermal_remove(struct platform_device *pdev)
{
	struct mnh_thermal_device *mnh_dev = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mnh_dev->sensors); ++i) {
		thermal_zone_of_sensor_unregister(&pdev->dev,
			mnh_dev->sensors[i]->tzd);
	}

	/* Disable the clock */
	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PVT_CLKEN, 0);

	iounmap(&mnh_dev->regs);

	return 0;
}

/*
 * of_device_id structure
 */
static const struct of_device_id mnh_thermal_of_match[] = {
	{ .compatible = "intel, mnh_thermal" },
	{ }
};

MODULE_DEVICE_TABLE(of, mnh_thermal_of_match);

/*
 * Platform driver structure
 */
static struct platform_driver mnh_thermal_driver = {
	.probe = mnh_thermal_probe,
	.remove = mnh_thermal_remove,
	.driver = {
		.name = "intel, mnh_thermal",
		.owner = THIS_MODULE,
		.of_match_table = mnh_thermal_of_match,
	},
};
module_platform_driver(mnh_thermal_driver);

MODULE_DESCRIPTION("MonetteHill Thermal Driver");
MODULE_LICENSE("GPL");
