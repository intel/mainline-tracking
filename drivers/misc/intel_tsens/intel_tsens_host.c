// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel tsens I2C thermal Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hddl_device.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/intel_tsens_host.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <uapi/linux/stat.h>

static int intel_tsens_get_temp(struct thermal_zone_device *zone,
				int *temp)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;
	struct i2c_client *i2c_c;

	if (strstr(zone->type, "smb"))
		i2c_c = tsens->i2c_smbus;
	else
		i2c_c = tsens->i2c_xlk;

	*temp = i2c_smbus_read_word_data(i2c_c,
					 tsens->t_data->sensor_type);
	return 0;
}

static int intel_tsens_thermal_get_trip_type(struct thermal_zone_device *zone,
					     int trip,
					     enum thermal_trip_type *type)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;

	*type = tsens->trip_info[trip]->trip_type;
	return 0;
}

static int intel_tsens_thermal_get_trip_temp(struct thermal_zone_device *zone,
					     int trip, int *temp)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;

	*temp = tsens->trip_info[trip]->temp;
	return 0;
}

static int intel_tsens_thermal_notify(struct thermal_zone_device *zone,
				      int trip, enum thermal_trip_type type)
{
	int ret = 0;

	switch (type) {
	case THERMAL_TRIP_ACTIVE:
		pr_warn("Thermal reached to active temperature\n");
		ret = 1;
		break;
	case THERMAL_TRIP_CRITICAL:
		pr_warn("Thermal reached to critical temperature\n");
		ret = 1;
		break;
	default:
		pr_warn("Thermal not reached to active temperature\n");
		break;
	}
	return ret;
}

static int intel_tsens_bind(struct thermal_zone_device *tz,
			    struct thermal_cooling_device *cdev)
{
	int ret;

	/*Check here thermal device zone name and*/
	/*cdev name to match, then call the bind device */
	if (strncmp("intel_tsens_thermal", cdev->type,
		    THERMAL_NAME_LENGTH) == 0) {
		ret = thermal_zone_bind_cooling_device
				(tz,
				THERMAL_TRIP_ACTIVE,
				cdev,
				THERMAL_NO_LIMIT,
				THERMAL_NO_LIMIT,
				THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			dev_err(&tz->device,
				"binding zone %s with cdev %s failed:%d\n",
				tz->type, cdev->type, ret);
			return ret;
		}
	}
	return 0;
}

static int intel_tsens_unbind(struct thermal_zone_device *tz,
			      struct thermal_cooling_device *cdev)
{
	int ret;

	if (strncmp("intel_tsens_thermal", cdev->type,
		    THERMAL_NAME_LENGTH) == 0) {
		ret = thermal_zone_unbind_cooling_device(tz, 0, cdev);
		if (ret) {
			dev_err(&tz->device,
				"unbinding zone %s with cdev %s failed:%d\n",
				tz->type, cdev->type, ret);
			return ret;
		}
	}
	return 0;
}

static struct thermal_zone_device_ops tsens_thermal_ops = {
	.bind = intel_tsens_bind,
	.unbind = intel_tsens_unbind,
	.get_temp = intel_tsens_get_temp,
	.get_trip_type	= intel_tsens_thermal_get_trip_type,
	.get_trip_temp	= intel_tsens_thermal_get_trip_temp,
	.notify		= intel_tsens_thermal_notify,
};

static int intel_tsens_add_tz(struct intel_tsens_host *tsens,
			      struct thermal_zone_device **tz,
			      const char *name,
			      struct device *dev,
			      int i)
{
	int ret;

	*tz =  thermal_zone_device_register(name,
					    tsens->t_data->n_trips,
					    0, tsens,
					    &tsens_thermal_ops,
					    NULL,
					    tsens->t_data->passive_delay,
					    tsens->t_data->polling_delay);
	if (IS_ERR(*tz)) {
		ret = PTR_ERR(*tz);
		dev_err(dev,
			"failed to register thermal zone device %s\n",
			tsens->t_data->name);
		return ret;
	}
	return 0;
}

static void intel_tsens_remove_tz(struct intel_hddl_clients *d)
{
	int i;

	for (i = 0; i < d->nsens; i++) {
		struct intel_tsens_host *tsens = d->tsens[i];

		if (tsens->tz_smbus) {
			thermal_zone_device_unregister(tsens->tz_smbus);
			tsens->tz_smbus = NULL;
		}
		if (tsens->tz_xlk) {
			thermal_zone_device_unregister(tsens->tz_xlk);
			tsens->tz_xlk = NULL;
		}
	}
}

static int intel_tsens_tj_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct intel_hddl_clients *d = client->dev.platform_data;
	u32 device_id = tsens_get_device_id(d);
	char *i2c_str;
	int ret, i;

	if (strstr(client->adapter->name, "SMBus I801")) {
		i2c_str = "smb";
		for (i = 0; i < d->nsens; i++) {
			struct intel_tsens_host *tsens = d->tsens[i];

			tsens->sensor_name_smbus =
				kasprintf(GFP_KERNEL,
					  "%s_%s-%x",
					  tsens->t_data->name,
					  i2c_str, device_id);
			tsens->i2c_smbus = client;
			ret = intel_tsens_add_tz(tsens,
						 &tsens->tz_smbus,
						 tsens->sensor_name_smbus,
						 &client->dev,
						 i);
			if (ret) {
				dev_err(&client->dev,
					"thermal zone configuration failed\n");
				intel_tsens_remove_tz(d);
				return ret;
			}
		}
	} else {
		i2c_str = "xlk";
		for (i = 0; i < d->nsens; i++) {
			struct intel_tsens_host *tsens = d->tsens[i];

			tsens->sensor_name_xlk =
				kasprintf(GFP_KERNEL,
					  "%s_%s-%x",
					  tsens->t_data->name,
					  i2c_str, device_id);
			tsens->i2c_xlk = client;
			ret = intel_tsens_add_tz(tsens,
						 &tsens->tz_xlk,
						 tsens->sensor_name_xlk,
						 &client->dev,
						 i);
			if (ret) {
				dev_err(&client->dev,
					"thermal zone configuration failed\n");
				intel_tsens_remove_tz(d);
				return ret;
			}
		}
	}

	i2c_set_clientdata(client, d);

	return 0;
}

static int intel_tsens_tj_exit(struct i2c_client *client)
{
	struct intel_hddl_clients *d = client->dev.platform_data;

	if (!d) {
		dev_err(&client->dev,
			"Unable to get private data\n");
		return -EINVAL;
	}
	intel_tsens_remove_tz(d);
	return 0;
}

static const struct i2c_device_id i2c_intel_tsens_id[] = {
	{ "intel_tsens", (kernel_ulong_t)NULL },
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_intel_tsens_id);

static struct i2c_driver i2c_intel_tsens_driver = {
	.driver = {
		.name = "intel_tsens",
	},
	.probe = intel_tsens_tj_probe,
	.remove = intel_tsens_tj_exit,
	.id_table = i2c_intel_tsens_id,
};
module_i2c_driver(i2c_intel_tsens_driver);

MODULE_DESCRIPTION("Intel tsens host Device driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
