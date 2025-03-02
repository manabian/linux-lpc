/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Lee Jones <lee.jones@linaro.org> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/export.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/spinlock.h>
#include <linux/sys_soc.h>
#include <linux/err.h>
#include <linux/glob.h>

static DEFINE_IDA(soc_ida);

static ssize_t soc_info_get(struct device *dev,
			    struct device_attribute *attr,
			    char *buf);

struct soc_device {
	struct device dev;
	struct soc_device_attribute *attr;
	int soc_dev_num;
};

static struct bus_type soc_bus_type = {
	.name  = "soc",
};

static DEVICE_ATTR(machine,  S_IRUGO, soc_info_get,  NULL);
static DEVICE_ATTR(family,   S_IRUGO, soc_info_get,  NULL);
static DEVICE_ATTR(soc_id,   S_IRUGO, soc_info_get,  NULL);
static DEVICE_ATTR(revision, S_IRUGO, soc_info_get,  NULL);

struct device *soc_device_to_device(struct soc_device *soc_dev)
{
	return &soc_dev->dev;
}

static umode_t soc_attribute_mode(struct kobject *kobj,
				struct attribute *attr,
				int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct soc_device *soc_dev = container_of(dev, struct soc_device, dev);

	if ((attr == &dev_attr_machine.attr)
	    && (soc_dev->attr->machine != NULL))
		return attr->mode;
	if ((attr == &dev_attr_family.attr)
	    && (soc_dev->attr->family != NULL))
		return attr->mode;
	if ((attr == &dev_attr_revision.attr)
	    && (soc_dev->attr->revision != NULL))
		return attr->mode;
	if ((attr == &dev_attr_soc_id.attr)
	    && (soc_dev->attr->soc_id != NULL))
		return attr->mode;

	/* Unknown or unfilled attribute. */
	return 0;
}

static ssize_t soc_info_get(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct soc_device *soc_dev = container_of(dev, struct soc_device, dev);

	if (attr == &dev_attr_machine)
		return sprintf(buf, "%s\n", soc_dev->attr->machine);
	if (attr == &dev_attr_family)
		return sprintf(buf, "%s\n", soc_dev->attr->family);
	if (attr == &dev_attr_revision)
		return sprintf(buf, "%s\n", soc_dev->attr->revision);
	if (attr == &dev_attr_soc_id)
		return sprintf(buf, "%s\n", soc_dev->attr->soc_id);

	return -EINVAL;

}

static struct attribute *soc_attr[] = {
	&dev_attr_machine.attr,
	&dev_attr_family.attr,
	&dev_attr_soc_id.attr,
	&dev_attr_revision.attr,
	NULL,
};

static const struct attribute_group soc_attr_group = {
	.attrs = soc_attr,
	.is_visible = soc_attribute_mode,
};

static const struct attribute_group *soc_attr_groups[] = {
	&soc_attr_group,
	NULL,
};

static void soc_release(struct device *dev)
{
	struct soc_device *soc_dev = container_of(dev, struct soc_device, dev);

	kfree(soc_dev);
}

struct soc_device *soc_device_register(struct soc_device_attribute *soc_dev_attr)
{
	struct soc_device *soc_dev;
	int ret;

	if (!soc_bus_type.p) {
		ret = bus_register(&soc_bus_type);
		if (ret)
			goto out1;
	}

	soc_dev = kzalloc(sizeof(*soc_dev), GFP_KERNEL);
	if (!soc_dev) {
		ret = -ENOMEM;
		goto out1;
	}

	/* Fetch a unique (reclaimable) SOC ID. */
	ret = ida_simple_get(&soc_ida, 0, 0, GFP_KERNEL);
	if (ret < 0)
		goto out2;
	soc_dev->soc_dev_num = ret;

	soc_dev->attr = soc_dev_attr;
	soc_dev->dev.bus = &soc_bus_type;
	soc_dev->dev.groups = soc_attr_groups;
	soc_dev->dev.release = soc_release;

	dev_set_name(&soc_dev->dev, "soc%d", soc_dev->soc_dev_num);

	ret = device_register(&soc_dev->dev);
	if (ret)
		goto out3;

	return soc_dev;

out3:
	ida_simple_remove(&soc_ida, soc_dev->soc_dev_num);
out2:
	kfree(soc_dev);
out1:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(soc_device_register);

/* Ensure soc_dev->attr is freed prior to calling soc_device_unregister. */
void soc_device_unregister(struct soc_device *soc_dev)
{
	ida_simple_remove(&soc_ida, soc_dev->soc_dev_num);

	device_unregister(&soc_dev->dev);
}
EXPORT_SYMBOL_GPL(soc_device_unregister);

static int __init soc_bus_register(void)
{
	if (soc_bus_type.p)
		return 0;

	return bus_register(&soc_bus_type);
}
core_initcall(soc_bus_register);

static int soc_device_match_one(struct device *dev, void *arg)
{
	struct soc_device *soc_dev = container_of(dev, struct soc_device, dev);
	const struct soc_device_attribute *match = arg;

	if (match->machine &&
	    (!soc_dev->attr->machine ||
	     !glob_match(match->machine, soc_dev->attr->machine)))
		return 0;

	if (match->family &&
	    (!soc_dev->attr->family ||
	     !glob_match(match->family, soc_dev->attr->family)))
		return 0;

	if (match->revision &&
	    (!soc_dev->attr->revision ||
	     !glob_match(match->revision, soc_dev->attr->revision)))
		return 0;

	if (match->soc_id &&
	    (!soc_dev->attr->soc_id ||
	     !glob_match(match->soc_id, soc_dev->attr->soc_id)))
		return 0;

	return 1;
}

/*
 * soc_device_match - identify the SoC in the machine
 * @matches: zero-terminated array of possible matches
 *
 * returns the first matching entry of the argument array, or NULL
 * if none of them match.
 *
 * This function is meant as a helper in place of of_match_node()
 * in cases where either no device tree is available or the information
 * in a device node is insufficient to identify a particular variant
 * by its compatible strings or other properties. For new devices,
 * the DT binding should always provide unique compatible strings
 * that allow the use of of_match_node() instead.
 *
 * The calling function can use the .data entry of the
 * soc_device_attribute to pass a structure or function pointer for
 * each entry.
 */
const struct soc_device_attribute *soc_device_match(
	const struct soc_device_attribute *matches)
{
	int ret = 0;

	if (!matches)
		return NULL;

	while (!ret) {
		if (!(matches->machine || matches->family ||
		      matches->revision || matches->soc_id))
			break;
		ret = bus_for_each_dev(&soc_bus_type, NULL, (void *)matches,
				       soc_device_match_one);
		if (!ret)
			matches++;
		else
			return matches;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(soc_device_match);
