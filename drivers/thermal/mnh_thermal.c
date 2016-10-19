
#include <linux/bitops.h>
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
#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-scu.h>

#define MNH_BAD_ADDR  ((void *)0xFFFFFFFF)
#define MNH_NUM_PVT_SENSORS 4

struct mnh_thermal_sensor {
	struct mnh_thermal_device *dev;
	struct thermal_zone_device *tzd;
	uint32_t id;
	uint32_t alarm_temp;
};

struct mnh_thermal_device {
	struct reset_control *reset;
	void __iomem *regs;
	struct mnh_thermal_sensor *sensors[MNH_NUM_PVT_SENSORS];
};


/*
 * Caculate PVT_DATA output to millicelsius.
 */
static int caculate_temp(u16 val)
{
	long t;

	/* This will be updated once Silicon is avaialble */
	t = val;

	return t;
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
static int mnh_thermal_get_temp(void *data, int *out_temp)
{
	struct mnh_thermal_sensor *sensor = (struct mnh_thermal_sensor*)data;
	u32 val = 0;

	/* Program VSAMPLE/PSAMPLE for temperature evaulation */
	HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id, PSAMPLE, 0);
	HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id, VSAMPLE, 0);

	/* Set ENA register to 1 */
	HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id, ENA, 1);

	/* Wait for conversion cycle time or poll for PVT_DATA[x]. DATAVALID to
	 * be 1 or wait for SCU interrupt if enabled for PVT_SENSx_DV
	 * interrupt reason
	 */
	while(1){
		val = HW_INxf(sensor->dev->regs, SCU, PVT_DATA,
			sensor->id, DATAVALID);
		if(val == 1)
			break;
	}

	/* Read PVT_DATA[x].DATA_OUT status register to read temperature */
	val = HW_INxf(sensor->dev->regs, SCU, PVT_DATA, sensor->id, DATA);

	/* Reset ENA register to 0 */
	HW_OUTxf(sensor->dev->regs, SCU, PVT_CONTROL, sensor->id, ENA, 0);

	*out_temp = caculate_temp(val);

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

	dev_err(&pdev->dev, "mnh_thermal_probe\n");

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
		dev_err(&pdev->dev, "mnh_thermal - zone register:%d\n",i);
	}


	/* TBD : Initialize the sensor with TRIM value by readin EFUSE
	 * (Not available yet in FPGA, will be available in silicon)
	 */

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
