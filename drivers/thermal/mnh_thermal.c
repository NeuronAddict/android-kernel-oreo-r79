/*
 * mnh_thermal.c - MonhetteHill PVT Sensor Thermal driver
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program;
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Jimin Ha (jiminha@intel.com)
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

#define MNH_BAD_ADDR  ((void *)0xFFFFFFFF)
#define MNH_NUM_PVT_SENSORS 4

#define API_TRIM_CODE          0xF
#define API_PRECISION_CODE     0x0
#define NO_BITS_EFUSE          10
#define API_BITS_SLOPE         5
#define API_BITS_OFFSET        (NO_BITS_EFUSE-API_BITS_SLOPE)

#define API_POLY_N4 -16743  // -1.6743e-11 : 15bits excluding sign
#define API_POLY_N3 +81542  // +8.1542e-08 : 17bits excluding sign
#define API_POLY_N2 -18201  // -1.8201e-04 : 15bits excluding sign
#define API_POLY_N1 +31020  // +3.1020e-01 : 15bits excluding sign
#define API_POLY_N0 -48380  // -4.8380e+01 : 16bits excluding sign

#define N3_E15_MULTIPLIER 1000 /* 1e3*/
#define N2_E15_MULTIPLIER 10000000 /* 1e7 */
#define N1_E15_MULTIPLIER 10000000000 /* 1e10 */
#define N0_E15_MULTIPLIER 1000000000000 /* 1e12 */

#define API_RES_SLOPE  1  /* 0.000010 */
#define API_RES_OFFSET 1  /* 0.001000 */

#define RES_SLOPE_DIVIDER 100000 /* 1e5 */
#define RES_OFFSET_E15_MULTIPLIER 1000000000000/* 1e12 */

#define API_BITS_SLOPE_MASK    0x1F
#define API_BITS_OFFSET_MASK   0x1F

#define N15_DIVIDER  1000000000000000 /* 1e15 */
#define N15_ROUNDING_NUM 500000000000000 /* 5e14 */


struct mnh_thermal_sensor {
	struct mnh_thermal_device *dev;
	struct thermal_zone_device *tzd;
	uint32_t id;
	uint32_t alarm_temp;
	int slope;
	int offset;
};

struct mnh_thermal_device {
	struct reset_control *reset;
	void __iomem *regs;
	uint32_t emulation;
	struct mnh_thermal_sensor *sensors[MNH_NUM_PVT_SENSORS];
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

			dev_dbg(&dev->sensors[i]->tzd->device,
				"sensor[%d]: efuse[0x%x] - s:%d,o:%d\n",
				i, val, slope, offset);

			dev->sensors[i]->slope = slope;
			dev->sensors[i]->offset = offset;

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
	int T, Final_Temp;


	pr_debug("calculate_temp: code:%d, slope:%d, offset:%d",
		code, slope, offset);

	/* Make sure code is 10bit */
	code = code & 0x3FF;

	/* Calculate raw code to DegC */
	n4_ = (long)code*code*code*code * API_POLY_N4;			// 55bits
	n3_ = (long)code*code*code * API_POLY_N3 * N3_E15_MULTIPLIER;	// 57bits
	n2_ = (long)code*code * API_POLY_N2 * N2_E15_MULTIPLIER;	// 55bits
	n1_ = (long)code * API_POLY_N1 * N1_E15_MULTIPLIER; 		// 56bits
	n0_ = API_POLY_N0 * N0_E15_MULTIPLIER;				// 56bits
	n01234_ = n4_ + n3_ + n2_ + n1_ + n0_;				// 59bits
	pr_debug("n01234_:%ld\n", n01234_);

	T = (n01234_ + N15_ROUNDING_NUM) / N15_DIVIDER;
	pr_debug("T = %d\n", T);

	/* n01234_ is 59bit, max slope is 5bit, which may overflow
	 * when multiplied without first down-scaling.
	 */
	error_ = n01234_/ RES_SLOPE_DIVIDER * slope * API_RES_SLOPE
		+ (long)offset * RES_OFFSET_E15_MULTIPLIER * API_RES_OFFSET;
	pr_debug("error = %ld\n", error_);

	final_temp_ = n01234_ - error_;
	Final_Temp = (final_temp_ + N15_ROUNDING_NUM) / N15_DIVIDER;
	pr_debug("Final_Temp = %d\n", Final_Temp);

	return Final_Temp;
}

/* The sequence of PVT sensing :
 * 1. Enable 1.2MHz Clock to PVT Sensor(PVT_CLKEN = 1)
 * 2. Wait for 2 msecs for the clock to PVT sensor to start ticking
 * 3. Program VSAMPLE/PSAMPLE
 * 4. Set ENA register to 1
 * 5. Wait for conversion cycle time or poll for PVT_DATA[x].DATAVALID to be 1 or
 * wait for SCU interrupt
 * 6. Read PVT_DATA[x].DATA_OUT[9:0]
 * 7. Reset ENA to 0
 * 8. Disable 1.2MHz clock to PVT sensor(PVT_CLKEN = 0)
*/
static void get_raw_temp_code(void *data, int *out_temp)
{

	struct mnh_thermal_sensor *sensor = (struct mnh_thermal_sensor*)data;
	u32 val = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	if(sensor->dev->emulation != 0){ /* Not silicon */

		/* For emulation, take raw out value as below
		 * data_out values are from temperature translation table from
		 * PVT sensor datasheet
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
		/* Program VSAMPLE/PSAMPLE for temperature evaulation */
		HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id, PSAMPLE, 0);
		HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id, VSAMPLE, 0);

		/* Set ENA register to 1 */
		HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id, ENA, 1);

		/* Wait for conversion cycle time or poll for PVT_DATA[x]. DATAVALID to
		 * be 1 or wait for SCU interrupt if enabled for PVT_SENSx_DV
		 * interrupt reason
		 * Conversion cycle time should be 376 PVT_SENSOR_CLK cycles
		 */
		do {
			val = HW_INxf(sensor->dev->regs, SCU, PVT_DATA,
				sensor->id, DATAVALID);
			if(val == 1)
				break;
			msleep(20);
		} while (time_before(jiffies, timeout));

		/* Read PVT_DATA[x].DATA_OUT status register to read temperature */
		val = HW_INxf(sensor->dev->regs, SCU, PVT_DATA, sensor->id, DATA);

		/* Reset ENA register to 0 */
		HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id, ENA, 0);
	}

	*out_temp = val;

	return;
}

/*
 * This will read raw_temp_code and convert to DecC temperature.
 */
static int mnh_thermal_get_temp(void *data, int *out_temp)
{
	struct mnh_thermal_sensor *sensor = (struct mnh_thermal_sensor*)data;
	int raw_temp = 0;
	int deg_temp = 0;


	/* Apply 5 bit trim and 2 bit precision to PVT sensor register */
	HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id,
	       TRIM, API_TRIM_CODE);
	HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id,
	       PRECISION, API_PRECISION_CODE);

	/* Get raw temp code */
	get_raw_temp_code(data, &raw_temp);

	/* Convert raw code to DecC */
	*out_temp = calculate_temp(raw_temp, sensor->slope, sensor->offset);

	return 0;

}


static const struct thermal_zone_of_device_ops mnh_of_thermal_ops = {
	.get_temp = mnh_thermal_get_temp,
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
		dev_err(&pdev->dev, "cannot get platform resources\n");
		return -ENOENT;
	}

	mnh_dev->regs = ioremap_nocache(res->start, resource_size(res));
	if (!mnh_dev->regs) {
		dev_err(&pdev->dev, "unable to remap resources\n");
		return -ENOMEM;
	}

	/* Read target settings */
	mnh_dev->emulation = read_emulation_setting();
	dev_dbg(&pdev->dev, "emulation : %d\n", mnh_dev->emulation);

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
			dev_err(&pdev->dev, "failed to register sensor: %d\n",
				err);
			goto unregister_sensors;
		}

		mnh_dev->sensors[i] = sensor;

		dev_info(&pdev->dev, "mnh_thermal - zone register:%d\n",i);
	}

	/* TBD : Initialize the sensor with TRIM value by readin EFUSE
	 * (Not available yet in FPGA, will be available in silicon)
	 */
	read_efuse_trim(mnh_dev);

	/* Enable 1.2MHz clock to PVT sensor and wait for 2msecs for the clock
	 * to PVT sensor to start ticking
	 */
	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PVT_CLKEN, 1);
	udelay(2);

	/* Enable THERMAL TRIP */

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

MODULE_DESCRIPTION("Monette Hill Thermal Driver");
MODULE_LICENSE("GPL");
