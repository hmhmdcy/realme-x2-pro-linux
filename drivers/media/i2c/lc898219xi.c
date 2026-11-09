// SPDX-License-Identifier: GPL-2.0-only
/* ON Semiconductor LC898219XI lens controller. */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define LC898219XI_MAX_FOCUS	1023
#define LC898219XI_POSITION	0x84
#define LC898219XI_INITIALIZE	0xe0
#define LC898219XI_OFF_US		100000
#define LC898219XI_START_US	5000

struct lc898219xi {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;
	struct mutex lock;
	struct regulator *vdd;
	ktime_t last_off;
	u16 target;
	bool target_valid;
	bool supply_enabled;
};

static inline struct lc898219xi *to_lc898219xi(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lc898219xi, sd);
}

static int lc898219xi_write(struct lc898219xi *lens, const u8 *data, int len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lens->sd);
	int ret = i2c_master_send(client, data, len);

	return ret == len ? 0 : ret < 0 ? ret : -EIO;
}

static int lc898219xi_set_position(struct lc898219xi *lens, u16 position)
{
	u8 data[] = { LC898219XI_POSITION, position >> 8, position & 0xff };

	return lc898219xi_write(lens, data, sizeof(data));
}

static int lc898219xi_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898219xi *lens = container_of(ctrl->handler, struct lc898219xi, ctrls);
	struct device *dev = lens->sd.dev;
	int active, ret = 0;

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return -EINVAL;

	active = pm_runtime_get_if_in_use(dev);
	if (active < 0)
		return active;
	if (active) {
		ret = lc898219xi_set_position(lens, ctrl->val);
		pm_runtime_put(dev);
	}
	if (!ret) {
		lens->target = ctrl->val;
		lens->target_valid = true;
	}

	return ret;
}

static const struct v4l2_ctrl_ops lc898219xi_ctrl_ops = {
	.s_ctrl = lc898219xi_set_ctrl,
};

/* Caller holds lock. A failed disable retains the regulator reference. */
static int lc898219xi_power_off(struct lc898219xi *lens)
{
	int ret;

	if (!lens->supply_enabled)
		return 0;
	ret = regulator_disable(lens->vdd);
	if (ret)
		return ret;
	lens->supply_enabled = false;
	lens->last_off = ktime_get();

	return 0;
}

static int lc898219xi_runtime_resume(struct device *dev)
{
	struct lc898219xi *lens = to_lc898219xi(dev_get_drvdata(dev));
	const u8 initialize[] = { LC898219XI_INITIALIZE, 1 };
	s64 elapsed;
	int ret, cleanup;

	mutex_lock(&lens->lock);
	/* Recover an earlier failed rollback without accumulating references. */
	ret = lc898219xi_power_off(lens);
	if (ret)
		goto unlock;
	elapsed = ktime_us_delta(ktime_get(), lens->last_off);
	if (elapsed < LC898219XI_OFF_US)
		usleep_range(LC898219XI_OFF_US - elapsed,
			     LC898219XI_OFF_US - elapsed + 1000);
	ret = regulator_enable(lens->vdd);
	if (ret)
		goto unlock;
	lens->supply_enabled = true;
	usleep_range(LC898219XI_START_US, LC898219XI_START_US + 1000);
	ret = lc898219xi_write(lens, initialize, sizeof(initialize));
	if (ret)
		goto disable;
	/*
	 * OEM initialization is a write, with a minimum one microsecond wait.
	 * Its readback does not echo 1 on the tested module; do not poll it.
	 */
	usleep_range(1000, 1100);
	if (lens->target_valid) {
		ret = lc898219xi_set_position(lens, lens->target);
		if (ret)
			goto disable;
	}
	goto unlock;

disable:
	cleanup = lc898219xi_power_off(lens);
	if (cleanup)
		dev_err(dev, "failed to release VDD after initialization error: %d\n",
			cleanup);
unlock:
	mutex_unlock(&lens->lock);
	return ret;
}

static int lc898219xi_runtime_suspend(struct device *dev)
{
	struct lc898219xi *lens = to_lc898219xi(dev_get_drvdata(dev));
	int ret;

	mutex_lock(&lens->lock);
	ret = lc898219xi_power_off(lens);
	mutex_unlock(&lens->lock);

	return ret;
}

static int lc898219xi_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int lc898219xi_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);
	return 0;
}

static const struct v4l2_subdev_internal_ops lc898219xi_internal_ops = {
	.open = lc898219xi_open,
	.close = lc898219xi_close,
};

static const struct v4l2_subdev_ops lc898219xi_ops = { };

static int lc898219xi_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lc898219xi *lens;
	struct v4l2_ctrl *focus;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	lens = devm_kzalloc(dev, sizeof(*lens), GFP_KERNEL);
	if (!lens)
		return -ENOMEM;
	lens->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(lens->vdd))
		return dev_err_probe(dev, PTR_ERR(lens->vdd), "failed to get VDD\n");
	mutex_init(&lens->lock);
	lens->last_off = ktime_get();
	v4l2_i2c_subdev_init(&lens->sd, client, &lc898219xi_ops);
	lens->sd.internal_ops = &lc898219xi_internal_ops;
	lens->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	lens->sd.entity.function = MEDIA_ENT_F_LENS;
	v4l2_ctrl_handler_init(&lens->ctrls, 1);
	lens->ctrls.lock = &lens->lock;
	focus = v4l2_ctrl_new_std(&lens->ctrls, &lc898219xi_ctrl_ops,
				V4L2_CID_FOCUS_ABSOLUTE, 0,
				LC898219XI_MAX_FOCUS, 1, 0);
	/*
	 * Do not select an uncalibrated initial target merely by opening a node.
	 * Explicit writes of the cached default must still reach the device.
	 */
	if (focus)
		focus->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ret = lens->ctrls.error;
	if (ret)
		goto free_ctrls;
	lens->sd.ctrl_handler = &lens->ctrls;
	ret = media_entity_pads_init(&lens->sd.entity, 0, NULL);
	if (ret)
		goto free_ctrls;
	pm_runtime_enable(dev);
	ret = v4l2_async_register_subdev(&lens->sd);
	if (ret)
		goto disable_pm;
	return 0;

disable_pm:
	pm_runtime_disable(dev);
	media_entity_cleanup(&lens->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&lens->ctrls);
	mutex_destroy(&lens->lock);
	return ret;
}

static void lc898219xi_remove(struct i2c_client *client)
{
	struct lc898219xi *lens = to_lc898219xi(i2c_get_clientdata(client));
	int ret;

	v4l2_async_unregister_subdev(&lens->sd);
	pm_runtime_disable(&client->dev);
	mutex_lock(&lens->lock);
	ret = lc898219xi_power_off(lens);
	mutex_unlock(&lens->lock);
	if (ret)
		dev_err(&client->dev, "failed to release VDD on removal: %d\n", ret);
	if (!ret)
		pm_runtime_set_suspended(&client->dev);
	media_entity_cleanup(&lens->sd.entity);
	v4l2_ctrl_handler_free(&lens->ctrls);
	mutex_destroy(&lens->lock);
}

static const struct of_device_id lc898219xi_of_match[] = {
	{ .compatible = "onnn,lc898219xi" },
	{ }
};
MODULE_DEVICE_TABLE(of, lc898219xi_of_match);

static const struct dev_pm_ops lc898219xi_pm_ops = {
	SET_RUNTIME_PM_OPS(lc898219xi_runtime_suspend,
			   lc898219xi_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static struct i2c_driver lc898219xi_driver = {
	.driver = {
		.name = "lc898219xi",
		.of_match_table = lc898219xi_of_match,
		.pm = &lc898219xi_pm_ops,
	},
	.probe = lc898219xi_probe,
	.remove = lc898219xi_remove,
};
module_i2c_driver(lc898219xi_driver);

MODULE_DESCRIPTION("ON Semiconductor LC898219XI lens controller");
MODULE_LICENSE("GPL");
