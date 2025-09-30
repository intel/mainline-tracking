// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016-2025 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <media/ipu-acpi.h>
#include <media/ipu-acpi-pdata.h>

#define MIN_SENSOR_I2C 1
#define MIN_SERDES_I2C 3
#define SUFFIX_BASE 97

struct ipu7_isys_subdev_pdata acpi_subdev_pdata = {
	.subdevs = (struct ipu7_isys_subdev_info *[]) {
		NULL,
	}
};

struct serdes_local serdes_info;

/*
 * The dev_id was hard code in platform data, as i2c bus number
 * may change dynamiclly, we need to update this bus id
 * accordingly.
 *
 * @adapter_id: hardware i2c adapter id, this was fixed in platform data
 * return: i2c bus id registered in system
 */
static int get_i2c_bus_id(int adapter_id, char *adapter_bdf, int bdf_len)
{
	struct i2c_adapter *adapter;
	char name[32];
	int i = 0;

	if (adapter_bdf) {
		while ((adapter = i2c_get_adapter(i)) != NULL) {
			struct device *parent = adapter->dev.parent;
			struct device *pp = parent->parent;

			if (pp && !strncmp(adapter_bdf, dev_name(pp), bdf_len))
				return i;
			i++;
		}
	}

	i = 0;
	snprintf(name, sizeof(name), "i2c_designware.%d", adapter_id);
	while ((adapter = i2c_get_adapter(i)) != NULL) {
		struct device *parent = adapter->dev.parent;

		if (parent && !strncmp(name, dev_name(parent), sizeof(name)))
			return i;
		i++;
	}

	/* Not found, should never happen! */
	WARN_ON_ONCE(1);
	return -1;
}

/*
 * update i2c bus here to avoid deadlock between i2c_for_each_dev
 * and i2c_get_adapter
 */
static void update_i2c_bus_id(void)
{
	struct ipu7_isys_subdev_info **subdevs = acpi_subdev_pdata.subdevs;
	for (int i = 0; subdevs[i] != NULL; i++) {
		subdevs[i]->i2c.i2c_adapter_id =
			get_i2c_bus_id(subdevs[i]->i2c.i2c_adapter_id,
				       subdevs[i]->i2c.i2c_adapter_bdf,
				       sizeof(subdevs[i]->i2c.i2c_adapter_bdf));
	}
}

struct ipu7_isys_subdev_pdata *get_acpi_subdev_pdata(void)
{
	struct ipu7_isys_subdev_pdata *ptr;

	update_i2c_bus_id();
	ptr = &acpi_subdev_pdata;
	return ptr;
}
EXPORT_SYMBOL(get_acpi_subdev_pdata);

static void print_serdes_sdinfo(struct serdes_subdev_info *sdinfo)
{
	int i;
	struct serdes_module_pdata *sd_mpdata = sdinfo->board_info.platform_data;

	if (!sd_mpdata) {
		pr_err("Empty serdes module pdata");
		return;
	}

	pr_debug("\t\trx_port \t\t= %d", sdinfo->rx_port);
	pr_debug("\t\tphy_i2c_addr \t\t= 0x%x", sdinfo->phy_i2c_addr);
	pr_debug("\t\tser_alias \t\t= 0x%x", sdinfo->ser_alias);
	pr_debug("\t\tser_phys_addr \t\t= 0x%x", sdinfo->ser_phys_addr);
	pr_debug("\t\tsuffix \t\t\t= %s", sdinfo->suffix);
	pr_debug("\t\tboard_info.type \t= %s", sdinfo->board_info.type);
	pr_debug("\t\tboard_info.addr \t= 0x%x", sdinfo->board_info.addr);

	pr_debug("serdes board_info.platform_data");
	pr_debug("\t\tlanes \t\t\t= %d", sd_mpdata->lanes);
	pr_debug("\t\tmodule_name \t\t= %s", sd_mpdata->module_name);
	pr_debug("\t\tfsin \t\t\t= %d", sd_mpdata->fsin);

	if (serdes_info.gpio_powerup_seq > 0)
		for (i = 0; i < serdes_info.gpio_powerup_seq; i++)
			pr_debug("\t\t gpio_powerup_seq[%d] \t= %d", i,
				(int)sd_mpdata->gpio_powerup_seq[i]);
}

static void print_serdes_subdev(struct ipu7_isys_subdev_info *sd)
{
	struct serdes_platform_data *sd_pdata = sd->i2c.board_info.platform_data;
	int i;
	struct serdes_subdev_info *sd_sdinfo;
	struct serdes_module_pdata *sd_mpdata;

	if (!sd_pdata) {
		pr_err("Empty serdes subdev pdata");
		return;
	}

	pr_debug("IPU ACPI: %s", __func__);
	pr_debug("sd_csi2");
	pr_debug("\t\tnlanes \t\t\t= %d", sd->csi2->nlanes);
	pr_debug("\t\tport \t\t\t= %d", sd->csi2->port);
	pr_debug("\t\ttype \t\t\t= %d", sd->csi2->bus_type);

	pr_debug("sd->i2c");
	pr_debug("\t\ti2c_adapter_bdf \t= %s", sd->i2c.i2c_adapter_bdf);
	pr_debug("\t\tboard_info.type \t= %s", sd->i2c.board_info.type);
	pr_debug("\t\tboard_info.addr \t= 0x%x", sd->i2c.board_info.addr);

	pr_debug("sd->i2c.board_info.platform_data");
	pr_debug("\t\treset_gpio \t\t= %d", sd_pdata->reset_gpio);
	pr_debug("\t\tFPD_gpio \t\t= %d", sd_pdata->FPD_gpio);
	pr_debug("\t\tsuffix \t\t\t= %c", sd_pdata->suffix);

	pr_debug("\t\tlink_freq_mbps \t\t= %d", sd_pdata->link_freq_mbps);
	pr_debug("\t\tdeser_nlanes \t\t= %d", sd_pdata->deser_nlanes);
	pr_debug("\t\tser_nlanes \t\t= %d", sd_pdata->ser_nlanes);

	for (i = 0; i < serdes_info.rx_port; i++) {
		sd_sdinfo = &sd_pdata->subdev_info[i];
		sd_mpdata = sd_sdinfo->board_info.platform_data;

		if (!sd_mpdata)
			continue;

		pr_debug("serdes subdev_info[%d]", i);
		print_serdes_sdinfo(sd_sdinfo);
	}

}

static void print_subdev(struct ipu7_isys_subdev_info *sd)
{
	struct sensor_platform_data *spdata = sd->i2c.board_info.platform_data;
	int i;

	if (!spdata) {
		pr_err("IPU ACPI: Empty sensor subdev");
		return;
	}

	pr_debug("IPU ACPI: %s", __func__);
	pr_debug("sd->csi2");
	pr_debug("\t\tnlanes \t\t\t= %d", sd->csi2->nlanes);
	pr_debug("\t\tport \t\t\t= %d", sd->csi2->port);
	pr_debug("\t\ttype \t\t\t= %d", sd->csi2->bus_type);

	pr_debug("sd->i2c");
	pr_debug("\t\ti2c_adapter_bdf \t= %s", sd->i2c.i2c_adapter_bdf);
	pr_debug("\t\tboard_info.type \t= %s", sd->i2c.board_info.type);
	pr_debug("\t\tboard_info.addr \t= 0x%x", sd->i2c.board_info.addr);

	pr_debug("sd->i2c.platform_data");
	pr_debug("\t\tport \t\t\t= %d", spdata->port);
	pr_debug("\t\tlanes \t\t\t= %d", spdata->lanes);
	pr_debug("\t\ti2c_slave_address \t= 0x%x", spdata->i2c_slave_address);
	pr_debug("\t\tirq_pin \t\t= %d", spdata->irq_pin);
	pr_debug("\t\tirq_pin_name \t\t= %s", spdata->irq_pin_name);
	pr_debug("\t\tsuffix \t\t\t= %c", spdata->suffix);
	pr_debug("\t\treset_pin \t\t= %d", spdata->reset_pin);
	pr_debug("\t\tdetect_pin \t\t= %d", spdata->detect_pin);

	for (i = 0; i < IPU7_SPDATA_GPIO_NUM; i++)
		pr_debug("\t\tgpios[%d] \t\t= %d", i, spdata->gpios[i]);
}

static void set_common_gpio(struct control_logic_data *ctl_data,
		     struct sensor_platform_data **pdata)
{
	int i;

	/* TODO: consider remove specific naming such as irq_pin, and use gpios[] */
	(*pdata)->irq_pin = -1;
	(*pdata)->reset_pin = -1;
	(*pdata)->detect_pin = -1;

	(*pdata)->gpios[0] = -1;
	(*pdata)->gpios[1] = 0;
	(*pdata)->gpios[2] = 0;
	(*pdata)->gpios[3] = 0;

	/* all sensors should have RESET GPIO */
	if (ctl_data->completed && ctl_data->gpio_num > 0)
		for (i = 0; i < ctl_data->gpio_num; i++)
			if (ctl_data->gpio[i].func != GPIO_RESET)
				dev_err(ctl_data->dev,
					"IPU ACPI: Invalid GPIO func: %d\n",
					ctl_data->gpio[i].func);
}

static int set_csi2(struct ipu7_isys_subdev_info **sensor_sd,
		    unsigned int lanes, unsigned int port,
		    unsigned int bus_type)
{
	struct ipu7_isys_csi2_config *csi2_config;

	csi2_config = kzalloc(sizeof(*csi2_config), GFP_KERNEL);
	if (!csi2_config)
		return -ENOMEM;

	csi2_config->nlanes = lanes;
	csi2_config->port = port;
	if (bus_type == PHY_MODE_DPHY)
		csi2_config->bus_type = V4L2_MBUS_CSI2_DPHY;
	else if (bus_type == PHY_MODE_CPHY)
		csi2_config->bus_type = V4L2_MBUS_CSI2_CPHY;
	else
		csi2_config->bus_type = V4L2_MBUS_UNKNOWN;

	(*sensor_sd)->csi2 = csi2_config;

	return 0;
}

static void set_i2c(struct ipu7_isys_subdev_info **sensor_sd,
		struct device *dev,
		const char *sensor_name,
		unsigned int addr,
		const char *i2c_adapter_bdf)
{
	dev_info(dev, "IPU ACPI: kernel I2C BDF: %s", i2c_adapter_bdf);
	(*sensor_sd)->i2c.board_info.addr = addr;
	strscpy((*sensor_sd)->i2c.board_info.type, sensor_name, I2C_NAME_SIZE);
	strscpy((*sensor_sd)->i2c.i2c_adapter_bdf, i2c_adapter_bdf,
		sizeof((*sensor_sd)->i2c.i2c_adapter_bdf));
}

static void set_serdes_sd_pdata(struct serdes_module_pdata **module_pdata,
				const char *sensor_name, const char *hid_name,
				unsigned int lanes)
{
	/* general */
	(*module_pdata)->lanes = lanes;
	strscpy((*module_pdata)->module_name, sensor_name, I2C_NAME_SIZE);
}

#define PORT_NR 8

static int set_serdes_subdev(struct ipu7_isys_subdev_info **serdes_sd,
		struct device *dev,
		struct serdes_platform_data **pdata,
		const char *sensor_name,
		const char *hid_name,
		unsigned int lanes,
		unsigned int addr,
		unsigned int subdev_num)
{
	int i;
	struct serdes_module_pdata *module_pdata[PORT_NR];
	struct serdes_subdev_info *serdes_sdinfo;
	size_t subdev_size = subdev_num * sizeof(*serdes_sdinfo);
	unsigned int port = (*pdata)->suffix - SUFFIX_BASE;

	serdes_sdinfo = kzalloc(subdev_size, GFP_KERNEL);
	if (!serdes_sdinfo)
		return -ENOMEM;

	for (i = 0; i < subdev_num; i++) {
		module_pdata[i] = kzalloc(sizeof(*module_pdata[i]), GFP_KERNEL);
		if (!module_pdata[i]) {
			kfree(serdes_sdinfo);
			return -ENOMEM;
		}

		set_serdes_sd_pdata(&module_pdata[i], sensor_name, hid_name, lanes);

		/* board info */
		strscpy(serdes_sdinfo[i].board_info.type, sensor_name, I2C_NAME_SIZE);
		serdes_sdinfo[i].board_info.addr = serdes_info.sensor_map_addr + i;
		serdes_sdinfo[i].board_info.platform_data = module_pdata[i];

		/* serdes_subdev_info */
		serdes_sdinfo[i].rx_port = i;
		serdes_sdinfo[i].ser_alias = serdes_info.ser_map_addr + i;

		serdes_sdinfo[i].phy_i2c_addr = serdes_info.phy_i2c_addr;
		snprintf(serdes_sdinfo[i].suffix, sizeof(serdes_sdinfo[i].suffix), "%c-%d",
			 SUFFIX_BASE + i, port);
#if IS_ENABLED(CONFIG_VIDEO_ISX031)
		serdes_sdinfo[i].ser_phys_addr = 0x40;
		serdes_sdinfo[i].sensor_dt = 0x1e;
#endif
	}

	(*pdata)->subdev_info = serdes_sdinfo;
	(*pdata)->subdev_num = subdev_num;

	return 0;
}

static int set_pdata(struct ipu7_isys_subdev_info **sensor_sd,
		struct device *dev,
		const char *sensor_name,
		const char *hid_name,
		struct control_logic_data *ctl_data,
		unsigned int port,
		unsigned int lanes,
		unsigned int addr,
		unsigned int subdev_num,
		unsigned int deser_lanes,
		bool is_dummy,
		enum connection_type connect,
		int link_freq,
		int des_port)
{
	if (connect == TYPE_DIRECT) {
		struct sensor_platform_data *pdata;

		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		pr_debug("IPU ACPI: %s - Direct connection", __func__);
		/* use ascii */
		/* port for start from 0 */
		if (port >= 0) {
			pdata->suffix = port + SUFFIX_BASE;
			pr_info("IPU ACPI: create %s on port %d",
				sensor_name, port);
		} else
			dev_err(dev, "INVALID MIPI PORT");

		pdata->port = port;
		pdata->lanes = lanes;
		pdata->i2c_slave_address = addr;

		/* gpio */
		set_common_gpio(ctl_data, &pdata);

		(*sensor_sd)->i2c.board_info.platform_data = pdata;
	} else if (connect == TYPE_SERDES) {
		struct serdes_platform_data *pdata;

		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		pr_debug("IPU ACPI: %s - Serdes connection", __func__);
		/* use ascii */
		if (port >= 0) {
			pdata->suffix = port + SUFFIX_BASE;
			pr_info("IPU ACPI: create %s on mipi port %d",
				sensor_name, port);
		} else
			pr_err("IPU ACPI: Invalid MIPI Port : %d", port);

		pdata->link_freq_mbps = link_freq;
		pdata->bus_type = (*sensor_sd)->csi2->bus_type;
		pdata->deser_nlanes = deser_lanes;
		pdata->ser_nlanes = lanes;
		pdata->des_port = des_port;
		strscpy(pdata->ser_name, (*sensor_sd)->i2c.board_info.type, I2C_NAME_SIZE);
		set_serdes_subdev(sensor_sd, dev, &pdata, sensor_name, hid_name, lanes, addr, subdev_num);

		(*sensor_sd)->i2c.board_info.platform_data = pdata;
		pdata->deser_board_info = &(*sensor_sd)->i2c.board_info;
	}

	return 0;
}

static void set_serdes_info(struct device *dev, const char *sensor_name,
			    const char *serdes_name,
			    struct sensor_bios_data *cam_data,
			    int sensor_physical_addr)
{
	int i;

	serdes_info.deser_num = 0;
	/* pprunit as num of sensor connected to deserializer */
	serdes_info.rx_port = cam_data->pprunit;

	/* i2c devices */
	serdes_info.i2c_num = cam_data->i2c_num;

	i = 1;
	/* serializer mapped addr */
	serdes_info.ser_map_addr = cam_data->i2c[i++].addr;
	/* sensor mapped addr */
	serdes_info.sensor_map_addr = cam_data->i2c[i++].addr;

		serdes_info.gpio_powerup_seq = 0;

	serdes_info.phy_i2c_addr = sensor_physical_addr;
}

static int populate_sensor_pdata(struct device *dev,
			struct ipu7_isys_subdev_info **sensor_sd,
			struct sensor_bios_data *cam_data,
			struct control_logic_data *ctl_data,
			enum connection_type connect,
			const char *sensor_name,
			const char *serdes_name,
			const char *hid_name,
			int sensor_physical_addr,
			int link_freq)
{
	struct ipu7_isys_subdev_pdata *ptr_acpi_subdev_pdata = &acpi_subdev_pdata;
	int i = 0;
	int ret;

	if (connect == TYPE_DIRECT) {
		/* sensor csi2 info */
		ret = set_csi2(sensor_sd, cam_data->lanes, cam_data->link, cam_data->bus_type);
		if (ret)
			return ret;

		/* sensor i2c info */
		if (cam_data->i2c_num == MIN_SENSOR_I2C) {
			pr_debug("IPU ACPI: num of I2C device for Direct connection: %lld is Correct.",
				cam_data->i2c_num);
			set_i2c(sensor_sd, dev, sensor_name, cam_data->i2c[0].addr, cam_data->i2c[0].bdf);
		} else {
			pr_err("IPU ACPI: num of I2C device for Direct connection : %lld is Incorrect",
				cam_data->i2c_num);
			return -1;
		}
		/* Others use DISCRETE Control Logic */
		if (ctl_data->type != CL_DISCRETE) {
			dev_err(dev, "IPU ACPI: Control Logic Type\n");
			dev_err(dev, "for %s: %d is Incorrect\n",
				sensor_name, ctl_data->type);
			return -EINVAL;
		}
	} else if (connect == TYPE_SERDES) {
		/* serdes csi2 info. pprval as deserializer lane */
		ret = set_csi2(sensor_sd, cam_data->pprval, cam_data->link, cam_data->bus_type);
		if (ret)
			return ret;

		/* Use DISCRETE Control Logic or No Control Logic for serdes */
		if (ctl_data->type != CL_DISCRETE && ctl_data->type != CL_EMPTY) {
			pr_err("IPU ACPI: Control Logic Type for serdes: %d is Incorrect",
				ctl_data->type);
			return -1;
		}

		/* serdes i2c info */
		if (cam_data->i2c_num >= MIN_SERDES_I2C) {
			pr_debug("IPU ACPI: num of I2C device for Serdes connection: %lld is Correct",
				cam_data->i2c_num);
			set_i2c(sensor_sd, dev, serdes_name, cam_data->i2c[0].addr, cam_data->i2c[0].bdf);
		} else {
			pr_err("IPU ACPI: num of I2C device for Serdes connection: %lld is Incorrect",
				cam_data->i2c_num);
			return -1;
		}

		/* local serdes info */
		set_serdes_info(dev, sensor_name, serdes_name, cam_data, sensor_physical_addr);
	}

	/* Use last I2C device */
	ret = set_pdata(sensor_sd, dev, sensor_name, hid_name, ctl_data, cam_data->link,
		cam_data->lanes, cam_data->i2c[cam_data->i2c_num - 1].addr,
		cam_data->pprunit, cam_data->pprval, false, connect, link_freq, cam_data->degree);
	if (ret)
		return ret;

	/* update local ipu7_isys_subdev_pdata */
	while (i <= MAX_ACPI_SENSOR_NUM) {
		if (!ptr_acpi_subdev_pdata->subdevs[i]) {
			ptr_acpi_subdev_pdata->subdevs[i] = *sensor_sd;
			ptr_acpi_subdev_pdata->subdevs[i+1] = NULL;
			break;
		}
		i++;
	}

	/* print new subdev */
	if (connect == TYPE_DIRECT) {
		pr_debug("New sensor subdev\n");
		print_subdev(*sensor_sd);
	} else {
		pr_debug("New serdes subdev\n");
		print_serdes_subdev(*sensor_sd);
	}

	/* update total num of sensor connected */
	if (connect == TYPE_SERDES)
		serdes_info.deser_num++;

	return 0;
}

int get_sensor_pdata(struct device *dev,
			struct ipu_camera_module_data *data,
			void *priv, size_t size,
			enum connection_type connect, const char *sensor_name,
			const char *serdes_name, const char *hid_name,
			int sensor_physical_addr, int link_freq)
{
	struct sensor_bios_data *cam_data;
	struct control_logic_data *ctl_data;
	struct ipu7_isys_subdev_info *sensor_sd;
	int rval;

	cam_data = kzalloc(sizeof(*cam_data), GFP_KERNEL);
	if (!cam_data)
		return -ENOMEM;
	cam_data->dev = dev;

	ctl_data = kzalloc(sizeof(*ctl_data), GFP_KERNEL);
	if (!ctl_data) {
		kfree(cam_data);
		return -ENOMEM;
	}
	ctl_data->dev = dev;

	sensor_sd = kzalloc(sizeof(*sensor_sd), GFP_KERNEL);
	if (!sensor_sd) {
		kfree(cam_data);
		kfree(ctl_data);
		return -ENOMEM;
	}

	/* camera info */
	rval = ipu_acpi_get_cam_data(dev, cam_data);
	if (rval) {
		kfree(sensor_sd);
		kfree(cam_data);
		kfree(ctl_data);
		return rval;
	}

	/* control logic info */
	rval = ipu_acpi_get_dep_data(dev, ctl_data);
	if (rval) {
		kfree(sensor_sd);
		kfree(cam_data);
		kfree(ctl_data);
		return rval;
	}

	/* populate pdata */
	rval = populate_sensor_pdata(dev, &sensor_sd, cam_data, ctl_data,
				     connect, sensor_name, serdes_name, hid_name,
				     sensor_physical_addr, link_freq);
	if (rval) {
		kfree(sensor_sd);
		kfree(cam_data);
		kfree(ctl_data);
		return rval;
	}

	dev->platform_data = sensor_sd;

	kfree(cam_data);
	kfree(ctl_data);
	return rval;
}
EXPORT_SYMBOL(get_sensor_pdata);

MODULE_AUTHOR("Khai Wen, Ng <khai.wen.ng@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IPU ACPI support");
