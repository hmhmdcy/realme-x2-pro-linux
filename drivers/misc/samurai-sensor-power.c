// SPDX-License-Identifier: GPL-2.0-only
/* Board-local SSC supply sequencer tied to the SLPI remoteproc lifetime. */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/remoteproc.h>
#include "samurai-sensor-power-seq.h"

struct samurai_sensor_power {
    struct device *dev;
    struct regulator *rails[2];
    struct rproc *slpi;
    struct rproc_subdev subdev;
    struct mutex lock;
    struct samurai_power_state state;
};

static int rail_enable(void *ctx, enum samurai_sensor_rail rail)
{
    struct samurai_sensor_power *p = ctx;
    int ret;

    /* Official board19781 SSC: L7C 2.856..3.104 V, then L8C 1.8 V. */
    ret = regulator_set_voltage(p->rails[rail], rail == SAMURAI_VDD ? 2856000 : 1800000,
                                rail == SAMURAI_VDD ? 3104000 : 1800000);
    if (ret)
        return ret;
    ret = regulator_set_load(p->rails[rail], 200000);
    if (ret)
        return ret;
    ret = regulator_enable(p->rails[rail]);
    if (ret) {
        int cleanup = regulator_set_load(p->rails[rail], 0);

        if (cleanup)
            dev_warn(p->dev, "rail %u load rollback failed: %d\n", rail, cleanup);
    }
    return ret;
}

static int rail_disable(void *ctx, enum samurai_sensor_rail rail)
{
    struct samurai_sensor_power *p = ctx;
    int ret = regulator_disable(p->rails[rail]);

    if (ret) {
        dev_err(p->dev, "rail %u disable failed, retaining reference: %d\n", rail, ret);
    } else {
        int cleanup = regulator_set_load(p->rails[rail], 0);

        if (cleanup)
            dev_warn(p->dev, "rail %u load clear failed after disable: %d\n", rail, cleanup);
    }
    return ret;
}

static void rail_delay(void *ctx) { msleep(10); }

static const struct samurai_power_ops power_ops = {
    .enable = rail_enable,
    .disable = rail_disable,
    .delay = rail_delay,
};

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct samurai_sensor_power *p = dev_get_drvdata(dev);
    ssize_t len;

    mutex_lock(&p->lock);
    len = sysfs_emit(buf, "vdd=%u vddio=%u\n", p->state.vdd_on, p->state.io_on);
    mutex_unlock(&p->lock);
    return len;
}
static DEVICE_ATTR_RO(state);
static struct attribute *samurai_power_attrs[] = { &dev_attr_state.attr, NULL };
static const struct attribute_group samurai_power_group = { .attrs = samurai_power_attrs };

static int samurai_sensor_power_prepare(struct rproc_subdev *subdev)
{
    struct samurai_sensor_power *p =
        container_of(subdev, struct samurai_sensor_power, subdev);
    int ret;

    mutex_lock(&p->lock);
    ret = samurai_power_on(&p->state, &power_ops, p);
    if (!ret)
        dev_info(p->dev, "sensor supplies enabled for %s\n", p->slpi->name);
    mutex_unlock(&p->lock);
    return ret;
}

static void samurai_sensor_power_stop(struct rproc_subdev *subdev, bool crashed)
{
    struct samurai_sensor_power *p =
        container_of(subdev, struct samurai_sensor_power, subdev);

    if (crashed)
        dev_warn(p->dev, "%s stopped after crash; supplies remain on until unprepare\n",
                 p->slpi->name);
}

static void samurai_sensor_power_unprepare(struct rproc_subdev *subdev)
{
    struct samurai_sensor_power *p =
        container_of(subdev, struct samurai_sensor_power, subdev);
    int ret;

    mutex_lock(&p->lock);
    ret = samurai_power_off(&p->state, &power_ops, p);
    if (ret)
        dev_err(p->dev, "sensor supply disable failed: %d\n", ret);
    else
        dev_info(p->dev, "sensor supplies disabled after %s stop\n", p->slpi->name);
    mutex_unlock(&p->lock);
}

static void put_slpi(void *ctx) { rproc_put(ctx); }

static void remove_subdev(void *ctx)
{
    struct samurai_sensor_power *p = ctx;

    rproc_remove_subdev(p->slpi, &p->subdev);
}

static int samurai_sensor_power_probe(struct platform_device *pdev)
{
    struct samurai_sensor_power *p;
    u32 phandle;
    int ret;

    if (!of_machine_is_compatible("realme,samurai"))
        return -ENODEV;
    ret = of_property_read_u32(pdev->dev.of_node, "qcom,rproc", &phandle);
    if (ret)
        ret = of_property_read_u32(pdev->dev.of_node, "remoteproc",
                                   &phandle);
    if (ret)
        return ret;
    p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
    if (!p)
        return -ENOMEM;
    p->dev = &pdev->dev;
    mutex_init(&p->lock);
    p->slpi = rproc_get_by_phandle(phandle);
    if (!p->slpi)
        return -EPROBE_DEFER;
    ret = devm_add_action_or_reset(&pdev->dev, put_slpi, p->slpi);
    if (ret)
        return ret;
    p->rails[SAMURAI_VDD] = devm_regulator_get(&pdev->dev, "vdd");
    if (IS_ERR(p->rails[SAMURAI_VDD]))
        return PTR_ERR(p->rails[SAMURAI_VDD]);
    p->rails[SAMURAI_VDDIO] = devm_regulator_get(&pdev->dev, "vddio");
    if (IS_ERR(p->rails[SAMURAI_VDDIO]))
        return PTR_ERR(p->rails[SAMURAI_VDDIO]);
    p->subdev.prepare = samurai_sensor_power_prepare;
    p->subdev.stop = samurai_sensor_power_stop;
    p->subdev.unprepare = samurai_sensor_power_unprepare;
    rproc_add_subdev(p->slpi, &p->subdev);
    ret = devm_add_action_or_reset(&pdev->dev, remove_subdev, p);
    if (ret)
        return ret;
    platform_set_drvdata(pdev, p);
    ret = devm_device_add_group(&pdev->dev, &samurai_power_group);
    if (ret)
        return ret;
    dev_info(&pdev->dev, "SSC supplies bound to %s lifecycle\n", p->slpi->name);
    return 0;
}

static void samurai_sensor_power_shutdown(struct platform_device *pdev)
{
    struct samurai_sensor_power *p = platform_get_drvdata(pdev);
    int ret;

    mutex_lock(&p->lock);
    if (p->slpi->state == RPROC_OFFLINE) {
        ret = samurai_power_off(&p->state, &power_ops, p);
        if (ret)
            dev_warn(p->dev, "shutdown supply disable failed: %d\n", ret);
    } else if (p->state.vdd_on || p->state.io_on) {
        dev_warn(p->dev, "SLPI active at shutdown; keeping supplies until reset\n");
    }
    mutex_unlock(&p->lock);
}

static const struct of_device_id samurai_sensor_power_match[] = {
    { .compatible = "realme,samurai-sensor-power" }, {}
};
static struct platform_driver samurai_sensor_power_driver = {
    .probe = samurai_sensor_power_probe,
    .shutdown = samurai_sensor_power_shutdown,
    .driver = {
        .name = "samurai-sensor-power",
        .of_match_table = samurai_sensor_power_match,
        .suppress_bind_attrs = true,
    },
};
module_platform_driver(samurai_sensor_power_driver);
MODULE_DESCRIPTION("Realme X2 Pro SLPI sensor supply sequencer");
MODULE_LICENSE("GPL");
