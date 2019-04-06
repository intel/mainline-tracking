// SPDX-License-Identifier: GPL-2.0-only
/*
 *  skl-debug.c - Debugfs for skl driver
 *
 *  Copyright (C) 2016-17 Intel Corp
 */

#include <linux/sched/signal.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <uapi/sound/skl-tplg-interface.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include "skl.h"
#include "skl-sst-dsp.h"
#include "skl-sst-ipc.h"
#include "skl-topology.h"
#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "skl-nhlt.h"

#define MOD_BUF		PAGE_SIZE
#define FW_REG_BUF	PAGE_SIZE
#define FW_REG_SIZE	0x60
#define MAX_SSP 	6

struct nhlt_blob {
	size_t size;
	struct nhlt_specific_cfg *cfg;
};

struct skl_pipe_event_data {
	long event_time;
	int event_type;
};

struct skl_debug {
	struct skl_dev *skl;
	struct device *dev;

	struct dentry *fs;
	struct dentry *ipc;
	struct dentry *modules;
	struct dentry *nhlt;
	u8 fw_read_buff[FW_REG_BUF];
	struct nhlt_blob ssp_blob[2*MAX_SSP];
	struct nhlt_blob dmic_blob;
	struct skl_pipe_event_data data;
};

/**
 * strsplit_u32 - Split string into sequence of u32 tokens
 * @buf:	String to split into tokens.
 * @delim:	String containing delimiter characters.
 * @tkns:	Returned u32 sequence pointer.
 * @num_tkns:	Returned number of tokens obtained.
 */
static int
strsplit_u32(char **buf, const char *delim, u32 **tkns, size_t *num_tkns)
{
	char *s;
	u32 *data, *tmp;
	size_t count = 0;
	size_t max_count = 32;
	int ret = 0;

	*tkns = NULL;
	*num_tkns = 0;
	data = kcalloc(max_count, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	while ((s = strsep(buf, delim)) != NULL) {
		ret = kstrtouint(s, 0, (data + count));
		if (ret)
			goto exit;
		if (++count >= max_count) {
			max_count *= 2;
			tmp = kcalloc(max_count, sizeof(*data), GFP_KERNEL);
			if (!tmp) {
				ret = -ENOMEM;
				goto exit;
			}

			memcpy(tmp, data, count * sizeof(*data));
			kfree(data);
			data = tmp;
		}
	}

	if (!count)
		goto exit;
	*tkns = kcalloc(count, sizeof(*data), GFP_KERNEL);
	if (*tkns == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memcpy(*tkns, data, count * sizeof(*data));
	*num_tkns = count;

exit:
	kfree(data);
	return ret;
}

static ssize_t skl_print_pins(struct skl_module_pin *m_pin, char *buf,
				int max_pin, ssize_t size, bool direction)
{
	int i;
	ssize_t ret = 0;

	for (i = 0; i < max_pin; i++)
		ret += snprintf(buf + size, MOD_BUF - size,
				"%s %d\n\tModule %d\n\tInstance %d\n\t"
				"In-used %s\n\tType %s\n"
				"\tState %d\n\tIndex %d\n",
				direction ? "Input Pin:" : "Output Pin:",
				i, m_pin[i].id.module_id,
				m_pin[i].id.instance_id,
				m_pin[i].in_use ? "Used" : "Unused",
				m_pin[i].is_dynamic ? "Dynamic" : "Static",
				m_pin[i].pin_state, i);
	return ret;
}

static ssize_t skl_print_fmt(struct skl_module_fmt *fmt, char *buf,
					ssize_t size, bool direction)
{
	return snprintf(buf + size, MOD_BUF - size,
			"%s\n\tCh %d\n\tFreq %d\n\tBit depth %d\n\t"
			"Valid bit depth %d\n\tCh config %#x\n\tInterleaving %d\n\t"
			"Sample Type %d\n\tCh Map %#x\n",
			direction ? "Input Format:" : "Output Format:",
			fmt->channels, fmt->s_freq, fmt->bit_depth,
			fmt->valid_bit_depth, fmt->ch_cfg,
			fmt->interleaving_style, fmt->sample_type,
			fmt->ch_map);
}

static ssize_t module_read(struct file *file, char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct skl_module_cfg *mconfig = file->private_data;
	struct skl_module *module = mconfig->module;
	struct skl_module_res *res = &module->resources[mconfig->res_idx];
	char *buf;
	ssize_t ret;

	buf = kzalloc(MOD_BUF, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = snprintf(buf, MOD_BUF, "Module:\n\tUUID %pUL\n\tModule id %d\n"
			"\tInstance id %d\n\tPvt_id %d\n", mconfig->guid,
			mconfig->id.module_id, mconfig->id.instance_id,
			mconfig->id.pvt_id);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Resources:\n\tCPC %#x\n\tIBS %#x\n\tOBS %#x\t\n",
			res->cpc, res->ibs, res->obs);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Module data:\n\tCore %d\n\tIn queue %d\n\t"
			"Out queue %d\n\tType %s\n",
			mconfig->core_id, mconfig->max_in_queue,
			mconfig->max_out_queue,
			mconfig->is_loadable ? "loadable" : "inbuilt");

	ret += skl_print_fmt(mconfig->in_fmt, buf, ret, true);
	ret += skl_print_fmt(mconfig->out_fmt, buf, ret, false);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Fixup:\n\tParams %#x\n\tConverter %#x\n",
			mconfig->params_fixup, mconfig->converter);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Module Gateway:\n\tType %#x\n\tVbus %#x\n\tHW conn %#x\n\tSlot %#x\n",
			mconfig->dev_type, mconfig->vbus_id,
			mconfig->hw_conn_type, mconfig->time_slot);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Pipeline:\n\tID %d\n\tPriority %d\n\tConn Type %d\n\t"
			"Pages %#x\n", mconfig->pipe->ppl_id,
			mconfig->pipe->pipe_priority, mconfig->pipe->conn_type,
			mconfig->pipe->memory_pages);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tParams:\n\t\tHost DMA %d\n\t\tLink DMA %d\n",
			mconfig->pipe->p_params->host_dma_id,
			mconfig->pipe->p_params->link_dma_id);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tPCM params:\n\t\tCh %d\n\t\tFreq %d\n\t\tFormat %d\n",
			mconfig->pipe->p_params->ch,
			mconfig->pipe->p_params->s_freq,
			mconfig->pipe->p_params->s_fmt);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tLink %#x\n\tStream %#x\n",
			mconfig->pipe->p_params->linktype,
			mconfig->pipe->p_params->stream);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tState %d\n\tPassthru %s\n",
			mconfig->pipe->state,
			mconfig->pipe->passthru ? "true" : "false");

	ret += skl_print_pins(mconfig->m_in_pin, buf,
			mconfig->max_in_queue, ret, true);
	ret += skl_print_pins(mconfig->m_out_pin, buf,
			mconfig->max_out_queue, ret, false);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Other:\n\tDomain %d\n\tHomogeneous Input %s\n\t"
			"Homogeneous Output %s\n\tIn Queue Mask %d\n\t"
			"Out Queue Mask %d\n\tDMA ID %d\n\tMem Pages %d\n\t"
			"Module Type %d\n\tModule State %d\n",
			mconfig->domain,
			mconfig->homogenous_inputs ? "true" : "false",
			mconfig->homogenous_outputs ? "true" : "false",
			mconfig->in_queue_mask, mconfig->out_queue_mask,
			mconfig->dma_id, mconfig->mem_pages, mconfig->m_state,
			mconfig->m_type);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);
	return ret;
}

static const struct file_operations mcfg_fops = {
	.open = simple_open,
	.read = module_read,
	.llseek = default_llseek,
};

void skl_debug_init_module(struct skl_debug *d,
			struct snd_soc_dapm_widget *w,
			struct skl_module_cfg *mconfig)
{
	debugfs_create_file(w->name, 0444, d->modules, mconfig,
			    &mcfg_fops);
}

static ssize_t fw_softreg_read(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	struct sst_dsp *sst = d->skl->dsp;
	size_t w0_stat_sz = SKL_FW_REGS_SIZE;
	void __iomem *in_base = sst->mailbox.in_base;
	void __iomem *fw_reg_addr;
	unsigned int offset;
	char *tmp;
	ssize_t ret = 0;

	tmp = kzalloc(FW_REG_BUF, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	fw_reg_addr = in_base - w0_stat_sz;
	memset(d->fw_read_buff, 0, FW_REG_BUF);

	if (w0_stat_sz > 0)
		__ioread32_copy(d->fw_read_buff, fw_reg_addr, w0_stat_sz >> 2);

	for (offset = 0; offset < FW_REG_SIZE; offset += 16) {
		ret += snprintf(tmp + ret, FW_REG_BUF - ret, "%#.4x: ", offset);
		hex_dump_to_buffer(d->fw_read_buff + offset, 16, 16, 4,
				   tmp + ret, FW_REG_BUF - ret, 0);
		ret += strlen(tmp + ret);

		/* print newline for each offset */
		if (FW_REG_BUF - ret > 0)
			tmp[ret++] = '\n';
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, tmp, ret);
	kfree(tmp);

	return ret;
}

static const struct file_operations soft_regs_ctrl_fops = {
	.open = simple_open,
	.read = fw_softreg_read,
	.llseek = default_llseek,
};

static ssize_t injection_dma_read(struct file *file,
		char __user *to, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	struct skl_probe_dma *dma;
	size_t num_dma, len = 0;
	char *buf;
	int i, ret;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = skl_probe_get_dma(d->skl, &dma, &num_dma);
	if (ret < 0)
		goto exit;

	for (i = 0; i < num_dma; i++) {
		ret = snprintf(buf + len, PAGE_SIZE - len,
			"Node id: %#x  DMA buffer size: %d\n",
			dma[i].node_id.val, dma[i].dma_buffer_size);
		if (ret < 0)
			goto free_dma;
		len += ret;
	}

	ret = simple_read_from_buffer(to, count, ppos, buf, len);
free_dma:
	kfree(dma);
exit:
	kfree(buf);
	return ret;
}

static const struct file_operations injection_dma_fops = {
	.open = simple_open,
	.read = injection_dma_read,
	.llseek = default_llseek,
};

static ssize_t ppoints_read(struct file *file,
		char __user *to, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	struct skl_probe_point_desc *desc;
	size_t num_desc, len = 0;
	char *buf;
	int i, ret;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = skl_probe_get_points(d->skl, &desc, &num_desc);
	if (ret < 0)
		goto exit;

	for (i = 0; i < num_desc; i++) {
		ret = snprintf(buf + len, PAGE_SIZE - len,
			"Id: %#010x  Purpose: %d  Node id: %#x\n",
			desc[i].id.value, desc[i].purpose, desc[i].node_id.val);
		if (ret < 0)
			goto free_desc;
		len += ret;
	}

	ret = simple_read_from_buffer(to, count, ppos, buf, len);
free_desc:
	kfree(desc);
exit:
	kfree(buf);
	return ret;
}

static ssize_t ppoints_write(struct file *file,
		const char __user *from, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	struct skl_probe_point_desc *desc;
	char *buf;
	u32 *tkns;
	size_t num_tkns;
	int ret;

	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, ppos, from, count);
	if (ret != count) {
		ret = ret >= 0 ? -EIO : ret;
		goto exit;
	}

	buf[count] = '\0';
	ret = strsplit_u32((char **)&buf, ",", &tkns, &num_tkns);
	if (ret < 0)
		goto exit;
	num_tkns *= sizeof(*tkns);
	if (!num_tkns || (num_tkns % sizeof(*desc))) {
		ret = -EINVAL;
		goto free_tkns;
	}

	desc = (struct skl_probe_point_desc *)tkns;
	ret = skl_probe_points_connect(d->skl, desc,
					num_tkns / sizeof(*desc));
	if (ret < 0)
		goto free_tkns;

	ret = count;
free_tkns:
	kfree(tkns);
exit:
	kfree(buf);
	return ret;
}

static const struct file_operations ppoints_fops = {
	.open = simple_open,
	.read = ppoints_read,
	.write = ppoints_write,
	.llseek = default_llseek,
};

static ssize_t ppoints_discnt_write(struct file *file,
		const char __user *from, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	union skl_probe_point_id *id;
	char *buf;
	u32 *tkns;
	size_t num_tkns;
	int ret;

	buf = kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = simple_write_to_buffer(buf, count, ppos, from, count);
	if (ret != count) {
		ret = ret >= 0 ? -EIO : ret;
		goto exit;
	}

	buf[count] = '\0';
	ret = strsplit_u32((char **)&buf, ",", &tkns, &num_tkns);
	if (ret < 0)
		goto exit;
	num_tkns *= sizeof(*tkns);
	if (!num_tkns || (num_tkns % sizeof(*id))) {
		ret = -EINVAL;
		goto free_tkns;
	}

	id = (union skl_probe_point_id *)tkns;
	ret = skl_probe_points_disconnect(d->skl, id,
					num_tkns / sizeof(*id));
	if (ret < 0)
		goto free_tkns;

	ret = count;
free_tkns:
	kfree(tkns);
exit:
	kfree(buf);
	return ret;
}

static const struct file_operations ppoints_discnt_fops = {
	.open = simple_open,
	.write = ppoints_discnt_write,
	.llseek = default_llseek,
};

static int trace_open(struct inode *inode, struct file *file)
{
	struct skl_debug *d = inode->i_private;
	struct skl_dev *skl = d->skl;
	int ret;

	ret = kfifo_alloc(&skl->trace_fifo, PAGE_SIZE, GFP_KERNEL);
	if (ret < 0)
		return ret;

	pm_runtime_get_sync(skl->dev);

	ret = skl_system_time_set(&skl->ipc);
	if (ret < 0)
		goto err;

	file->private_data = d;
	return 0;

err:
	kfifo_free(&skl->trace_fifo);
	pm_runtime_mark_last_busy(skl->dev);
	pm_runtime_put_autosuspend(skl->dev);

	return ret;
}

static ssize_t trace_read(struct file *file,
		char __user *to, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	struct skl_dev *skl = d->skl;
	struct kfifo *fifo = &skl->trace_fifo;
	unsigned int copied;

	count = kfifo_len(fifo);
	if (!count) {
		DEFINE_WAIT(wait);

		prepare_to_wait(&skl->trace_waitq, &wait, TASK_INTERRUPTIBLE);
		if (!signal_pending(current))
			schedule();
		finish_wait(&skl->trace_waitq, &wait);

		count = kfifo_len(fifo);
	}

	if (kfifo_to_user(fifo, to, count, &copied))
		return -EFAULT;
	*ppos += count;
	return count;
}

static int trace_release(struct inode *inode, struct file *file)
{
	struct skl_debug *d = file->private_data;
	struct skl_dev *skl = d->skl;

	kfifo_free(&skl->trace_fifo);
	pm_runtime_mark_last_busy(skl->dev);
	pm_runtime_put_autosuspend(skl->dev);

	return 0;
}

static const struct file_operations trace_fops = {
	.open = trace_open,
	.read = trace_read,
	.llseek = default_llseek,
	.release = trace_release,
};

static int skl_debugfs_init_ipc(struct skl_debug *d)
{
	if (!debugfs_create_file("injection_dma", 0444,
			d->ipc, d, &injection_dma_fops))
		return -EIO;
	if (!debugfs_create_file("probe_points", 0644,
			d->ipc, d, &ppoints_fops))
		return -EIO;
	if (!debugfs_create_file("probe_points_disconnect", 0200,
			d->ipc, d, &ppoints_discnt_fops))
		return -EIO;
	if (!debugfs_create_file("trace", 0444,
			d->ipc, d, &trace_fops))
		return -EIO;

	return 0;
}

struct nhlt_specific_cfg
*skl_nhlt_get_debugfs_blob(struct skl_debug *d, u8 link_type, u32 instance,
		u8 stream)
{
	switch (link_type) {
	case NHLT_LINK_DMIC:
		return d->dmic_blob.cfg;

	case NHLT_LINK_SSP:
		if (instance >= MAX_SSP)
			return NULL;

		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			return d->ssp_blob[instance].cfg;
		else
			return d->ssp_blob[MAX_SSP + instance].cfg;

	default:
		break;
	}

	dev_err(d->dev, "NHLT debugfs query failed\n");
	return NULL;
}

static ssize_t nhlt_read(struct file *file, char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct nhlt_blob *blob = file->private_data;

	if (!blob->cfg)
		return -EIO;

	return simple_read_from_buffer(user_buf, count, ppos,
			blob->cfg, blob->size);
}

static ssize_t nhlt_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct nhlt_blob *blob = file->private_data;
	struct nhlt_specific_cfg *new_cfg;
	ssize_t written;
	size_t size = blob->size;

	if (!blob->cfg) {
		/* allocate mem for blob */
		blob->cfg = kzalloc(count, GFP_KERNEL);
		if (!blob->cfg)
			return -ENOMEM;
		size = count;
	} else if (blob->size < count) {
		/* size if different, so relloc */
		new_cfg = krealloc(blob->cfg, count, GFP_KERNEL);
		if (!new_cfg)
			return -ENOMEM;
		size = count;
		blob->cfg = new_cfg;
	}

	written = simple_write_to_buffer(blob->cfg, size, ppos,
						user_buf, count);
	blob->size = written;

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);

	print_hex_dump(KERN_DEBUG, "Debugfs Blob:", DUMP_PREFIX_OFFSET, 8, 4,
			blob->cfg, blob->size, false);

	return written;
}

static const struct file_operations nhlt_fops = {
	.open = simple_open,
	.read = nhlt_read,
	.write = nhlt_write,
	.llseek = default_llseek,
};

static void skl_exit_nhlt(struct skl_debug *d)
{
	int i;

	/* free blob memory, if allocated */
	for (i = 0; i < MAX_SSP; i++)
		kfree(d->ssp_blob[MAX_SSP + i].cfg);
}

static ssize_t nhlt_control_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char *state;

	state = d->skl->nhlt_override ? "enable\n" : "disable\n";
	return simple_read_from_buffer(user_buf, count, ppos,
			state, strlen(state));
}

static ssize_t nhlt_control_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char buf[16];
	int len = min(count, (sizeof(buf) - 1));


	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;
	buf[len] = 0;

	if (!strncmp(buf, "enable\n", len)) {
		d->skl->nhlt_override = true;
	} else if (!strncmp(buf, "disable\n", len)) {
		d->skl->nhlt_override = false;
		skl_exit_nhlt(d);
	} else {
		return -EINVAL;
	}

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);

	return len;
}

static const struct file_operations ssp_cntrl_nhlt_fops = {
	.open = simple_open,
	.read = nhlt_control_read,
	.write = nhlt_control_write,
	.llseek = default_llseek,
};

static int skl_init_nhlt(struct skl_debug *d)
{
	int i;
	char name[12];

	if (!debugfs_create_file("control",
				0644, d->nhlt,
				d, &ssp_cntrl_nhlt_fops)) {
		dev_err(d->dev, "nhlt control debugfs init failed\n");
		return -EIO;
	}

	for (i = 0; i < MAX_SSP; i++) {
		snprintf(name, (sizeof(name)-1), "ssp%dp", i);
		if (!debugfs_create_file(name,
					0644, d->nhlt,
					&d->ssp_blob[i], &nhlt_fops))
			dev_err(d->dev, "%s: debugfs init failed\n", name);
		snprintf(name, (sizeof(name)-1), "ssp%dc", i);
		if (!debugfs_create_file(name,
					0644, d->nhlt,
					&d->ssp_blob[MAX_SSP + i], &nhlt_fops))
			dev_err(d->dev, "%s: debugfs init failed\n", name);
	}

	if (!debugfs_create_file("dmic", 0644,
				d->nhlt, &d->dmic_blob,
				&nhlt_fops))
		dev_err(d->dev, "%s: debugfs init failed\n", name);

	return 0;
}

static ssize_t core_power_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	struct skl_dev *skl_ctx = d->skl;
	struct sst_dsp *ctx = skl_ctx->dsp;
	char buf[16];
	int len = min(count, (sizeof(buf) - 1));
	unsigned int core_id;
	char *ptr;
	int wake;
	int err;


	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;
	buf[len] = 0;

	/*
	 * The buffer content should be "wake n" or "sleep n",
	 * where n is the core id
	 */
	ptr = strnstr(buf, "wake", len);
	if (ptr) {
		ptr = ptr + 5;
		wake = 1;
	} else {
		ptr = strnstr(buf, "sleep", len);
		if (ptr) {
			ptr = ptr + 6;
			wake = 0;
		} else
			return -EINVAL;
	}

	err = kstrtouint(ptr, 10, &core_id);
	if (err) {
		dev_err(d->dev, "%s: Debugfs kstrtouint returned error = %d\n",
				__func__, err);
		return err;
	}

	dev_info(d->dev, "Debugfs: %s %d\n", wake ? "wake" : "sleep", core_id);

	if (wake) {
		if (core_id == SKL_DSP_CORE0_ID)
			pm_runtime_get_sync(d->dev);
		else
			skl_dsp_get_core(ctx, core_id);
	} else {
		if (core_id == SKL_DSP_CORE0_ID)
			pm_runtime_put_sync(d->dev);
		else
			skl_dsp_put_core(ctx, core_id);
	}

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);

	return len;
}
static const struct file_operations core_power_fops = {
	.open = simple_open,
	.write = core_power_write,
	.llseek = default_llseek,
};

void skl_dbg_event(struct skl_dev *ctx, int type)
{
	int retval;
	struct timespec64 pipe_event_ts;
	struct skl_dev *skl = get_skl_ctx(ctx->dev);
	struct kobject *kobj;

	kobj = &skl->component->dev->kobj;

	if (type == SKL_PIPE_CREATED)
		/* pipe creation event */
		retval = kobject_uevent(kobj, KOBJ_ADD);
	else if (type == SKL_PIPE_INVALID)
		/* pipe deletion event */
		retval = kobject_uevent(kobj, KOBJ_REMOVE);
	else
		return;

	if (retval < 0) {
		dev_err(ctx->dev,
			"pipeline uevent failed, ret = %d\n", retval);
		return;
	}

	ktime_get_real_ts64(&pipe_event_ts);

	skl->debugfs->data.event_time = pipe_event_ts.tv_nsec/1000;
	skl->debugfs->data.event_type = type;
}

static ssize_t skl_dbg_event_read(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char buf[32];
	char pipe_state[24];
	int retval;

	if (d->data.event_type)
		strcpy(pipe_state, "SKL_PIPE_CREATED");
	else
		strcpy(pipe_state, "SKL_PIPE_INVALID");

	retval = snprintf(buf, sizeof(buf), "%s - %ld\n",
			pipe_state, d->data.event_time);

	return simple_read_from_buffer(user_buf, count, ppos, buf, retval);
}

static const struct file_operations skl_dbg_event_fops = {
	.open = simple_open,
	.read = skl_dbg_event_read,
	.llseek = default_llseek,
};

static int skl_init_dbg_event(struct skl_debug *d)
{
	if (!debugfs_create_file("dbg_event", 0644, d->fs, d,
				&skl_dbg_event_fops)) {
		dev_err(d->dev, "dbg_event debugfs file creation failed\n");
		return -EIO;
	}

	return 0;
}

struct skl_debug *skl_debugfs_init(struct skl_dev *skl)
{
	struct skl_debug *d;
	int ret;

	d = devm_kzalloc(&skl->pci->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return NULL;

	/* create the debugfs dir with platform component's debugfs as parent */
	d->fs = debugfs_create_dir("dsp", skl->component->debugfs_root);

	d->skl = skl;
	d->dev = &skl->pci->dev;

	d->ipc = debugfs_create_dir("ipc", d->fs);
	if (IS_ERR_OR_NULL(d->ipc))
		return NULL;
	if (skl_debugfs_init_ipc(d))
		return NULL;

	/* now create the module dir */
	d->modules = debugfs_create_dir("modules", d->fs);

	debugfs_create_file("fw_soft_regs_rd", 0444, d->fs, d,
			    &soft_regs_ctrl_fops);

	if (!debugfs_create_file("core_power", 0644, d->fs, d,
			 &core_power_fops)) {
		dev_err(d->dev, "core power debugfs init failed\n");
		return NULL;
	}

	/* now create the NHLT dir */
	d->nhlt =  debugfs_create_dir("nhlt", d->fs);
	if (IS_ERR(d->nhlt) || !d->nhlt) {
		dev_err(&skl->pci->dev, "nhlt debugfs create failed\n");
		return NULL;
	}

	skl_init_nhlt(d);

	ret = skl_init_dbg_event(d);
	if (ret < 0) {
		dev_err(&skl->pci->dev,
			"dbg_event debugfs init failed, ret = %d\n", ret);
		return NULL;
	}

	return d;
}

void skl_debugfs_exit(struct skl_dev *skl)
{
	struct skl_debug *d = skl->debugfs;

	debugfs_remove_recursive(d->fs);

	d = NULL;
}
