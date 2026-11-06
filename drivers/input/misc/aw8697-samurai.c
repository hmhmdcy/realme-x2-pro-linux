// SPDX-License-Identifier: GPL-2.0-only
/* RMX1931/19781 AW8697 bring-up: continuous mode, no waveform/NV writes.
 * Register meanings and 170 Hz/52 drive ceiling come from the published
 * Realme 19781 driver. AW86927 is a different register-map generation.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define AW_ID 0x00
#define AW_SYSST 0x01
#define AW_SYSINTM 0x03
#define AW_SYSCTRL 0x04
#define AW_GO 0x05
#define AW_STATE 0x46
#define AW_FAULT (BIT(2) | BIT(1)) /* Overcurrent and overtemperature. */
#define AW_MAX_MS 1000

struct samurai_aw {
	struct device *dev;
	struct regmap *map;
	struct gpio_desc *reset;
	struct delayed_work work;
	struct mutex io_lock;
	spinlock_t request_lock;
	u16 requested;
	unsigned long requested_at;
	u32 sequence;
	bool suspended;
	bool faulted;
	bool active;
	bool identified;
	u16 applied;
	int last_error;
	u32 starts;
	u32 stops;
	u32 watchdog_stops;
};

static const struct regmap_config samurai_aw_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7f,
	.cache_type = REGCACHE_NONE,
};

/* Called only with io_lock, or before publishing the input device. */
static int samurai_aw_stop(struct samurai_aw *aw)
{
	unsigned int state;
	int ret, standby;

	if (!aw->identified)
		return 0;
	ret = regmap_update_bits(aw->map, AW_GO, BIT(0), 0);
	if (!ret)
		ret = regmap_read_poll_timeout(aw->map, AW_STATE, state,
					       !(state & 0xf), 2000, 120000);
	standby = regmap_update_bits(aw->map, AW_SYSCTRL, BIT(0), BIT(0));
	if (aw->active)
		aw->stops++;
	aw->active = false;
	aw->applied = 0;
	return ret ? ret : standby;
}

static void samurai_aw_fault(struct samurai_aw *aw, int error)
{
	samurai_aw_stop(aw);
	gpiod_set_value_cansleep(aw->reset, 1);
	aw->faulted = true;
	aw->last_error = error;
	dev_err(aw->dev, "haptics stopped; reset held after error %d\n", error);
}

static int samurai_aw_init_hw(struct samurai_aw *aw)
{
	unsigned int id;
	int ret;
	static const struct reg_sequence defaults[] = {
		{ AW_SYSINTM, 0xff }, /* No IRQ handler in this bounded candidate. */
		{ 0x31, 0x30 }, { 0x32, 0xeb }, { 0x33, 0xd4 },
		{ 0x4d, 0x12 }, { 0x5d, 0x68 },
	};

	gpiod_set_value_cansleep(aw->reset, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(aw->reset, 0);
	usleep_range(8000, 9000);
	ret = regmap_read(aw->map, AW_ID, &id);
	if (ret)
		return ret;
	if (id != 0x97)
		return -ENODEV;
	aw->identified = true;
	ret = regmap_write(aw->map, AW_ID, 0xaa);
	if (ret)
		return ret;
	usleep_range(2000, 3000);
	ret = samurai_aw_stop(aw);
	if (ret)
		return ret;
	ret = regmap_multi_reg_write(aw->map, defaults, ARRAY_SIZE(defaults));
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->map, 0x2e, GENMASK(6, 5), 2 << 5);
	if (ret)
		return ret;
	/* Continuous playback uses bypass. Preserve reset protection thresholds. */
	ret = regmap_update_bits(aw->map, 0x47, BIT(2), 0);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->map, 0x66, BIT(6), BIT(6));
	if (!ret) {
		aw->faulted = false;
		aw->last_error = 0;
	}
	return ret;
}

static int samurai_aw_start(struct samurai_aw *aw, u16 strength)
{
	u16 preset = 1000000000U / (1700U * 260U);
	u8 level = DIV_ROUND_UP((u32)strength * 52, U16_MAX);
	u8 overdrive = DIV_ROUND_UP((u32)strength * 125, U16_MAX);
	struct reg_sequence profile[] = {
		{ 0x48, 0x89 }, /* zero-cross, closed-loop, brake, GO controlled */
		{ 0x49, preset >> 8 }, { 0x4a, preset & 0xff },
		{ 0x4b, 0 }, { 0x4c, 0x9a }, { 0x4d, 0x12 },
		{ 0x72, 0x0f }, { 0x73, 0xf1 },
		{ 0x74, 0x10 }, { 0x75, 0x08 },
		{ 0x76, 0x03 }, { 0x77, 0xf8 },
		{ 0x7a, 0x23 }, { 0x7b, level }, { 0x7c, overdrive },
	};
	int ret;

	ret = samurai_aw_stop(aw);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->map, AW_SYSCTRL, GENMASK(3, 0), 2 << 2);
	if (ret)
		return ret;
	usleep_range(2000, 3000);
	ret = regmap_update_bits(aw->map, 0x2b, GENMASK(7, 5), 0xe0);
	if (ret)
		return ret;
	ret = regmap_multi_reg_write(aw->map, profile, ARRAY_SIZE(profile));
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->map, 0x78, GENMASK(3, 0), 3);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->map, AW_GO, BIT(0), BIT(0));
	if (!ret) {
		aw->active = true;
		aw->applied = strength;
		aw->starts++;
	}
	return ret;
}

static void samurai_aw_work(struct work_struct *work)
{
	struct samurai_aw *aw = container_of(to_delayed_work(work), struct samurai_aw, work);
	unsigned long flags, deadline;
	unsigned int status;
	u32 sequence;
	u16 strength;
	bool suspended, reschedule;
	int ret = 0;

	spin_lock_irqsave(&aw->request_lock, flags);
	strength = aw->requested;
	deadline = aw->requested_at + msecs_to_jiffies(AW_MAX_MS);
	sequence = aw->sequence;
	suspended = aw->suspended;
	spin_unlock_irqrestore(&aw->request_lock, flags);
	mutex_lock(&aw->io_lock);
	if (aw->faulted || suspended)
		goto out;
	if (!strength || time_after_eq(jiffies, deadline)) {
		if (strength && aw->active)
			aw->watchdog_stops++;
		ret = samurai_aw_stop(aw);
	} else {
		ret = regmap_read(aw->map, AW_SYSST, &status);
		if (!ret && (status & AW_FAULT))
			ret = -EIO;
		if (!ret && (!aw->active || aw->applied != strength))
			ret = samurai_aw_start(aw, strength);
	}
	if (ret)
		samurai_aw_fault(aw, ret);
out:
	reschedule = aw->active && !aw->faulted && !suspended;
	mutex_unlock(&aw->io_lock);
	/* A new request during I2C work must win over the periodic poll. */
	spin_lock_irqsave(&aw->request_lock, flags);
	if (aw->sequence != sequence)
		mod_delayed_work(system_wq, &aw->work, 0);
	else if (reschedule)
		mod_delayed_work(system_wq, &aw->work, msecs_to_jiffies(20));
	spin_unlock_irqrestore(&aw->request_lock, flags);
}

static int samurai_aw_play(struct input_dev *input, void *data, struct ff_effect *effect)
{
	struct samurai_aw *aw = data;
	unsigned long flags;

	spin_lock_irqsave(&aw->request_lock, flags);
	aw->requested = max(effect->u.rumble.strong_magnitude, effect->u.rumble.weak_magnitude);
	aw->requested_at = jiffies;
	aw->sequence++;
	if (!aw->suspended)
		mod_delayed_work(system_wq, &aw->work, 0);
	spin_unlock_irqrestore(&aw->request_lock, flags);
	return 0;
}

static void samurai_aw_quiesce(struct samurai_aw *aw)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&aw->request_lock, flags);
	aw->suspended = true;
	aw->requested = 0;
	aw->sequence++;
	spin_unlock_irqrestore(&aw->request_lock, flags);
	cancel_delayed_work_sync(&aw->work);
	mutex_lock(&aw->io_lock);
	ret = samurai_aw_stop(aw);
	if (ret)
		samurai_aw_fault(aw, ret);
	mutex_unlock(&aw->io_lock);
}

static int samurai_aw_open(struct input_dev *input)
{
	struct samurai_aw *aw = input_get_drvdata(input);
	unsigned long flags;
	int ret = 0;

	mutex_lock(&aw->io_lock);
	if (aw->faulted)
		ret = aw->last_error;
	mutex_unlock(&aw->io_lock);
	if (ret)
		return ret;
	spin_lock_irqsave(&aw->request_lock, flags);
	aw->suspended = false;
	spin_unlock_irqrestore(&aw->request_lock, flags);
	return 0;
}

static void samurai_aw_close(struct input_dev *input)
{
	samurai_aw_quiesce(input_get_drvdata(input));
}

static void samurai_aw_release(void *data)
{
	struct samurai_aw *aw = data;

	samurai_aw_quiesce(aw);
	gpiod_set_value_cansleep(aw->reset, 1);
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct samurai_aw *aw = dev_get_drvdata(dev);
	unsigned int id, go, state, sysctrl, drive;
	int ret;

	mutex_lock(&aw->io_lock);
	ret = regmap_read(aw->map, AW_ID, &id);
	if (!ret)
		ret = regmap_read(aw->map, AW_GO, &go);
	if (!ret)
		ret = regmap_read(aw->map, AW_STATE, &state);
	if (!ret)
		ret = regmap_read(aw->map, AW_SYSCTRL, &sysctrl);
	if (!ret)
		ret = regmap_read(aw->map, 0x7b, &drive);
	if (!ret)
		ret = sysfs_emit(buf, "id=%02x go=%02x state=%02x sysctrl=%02x drive=%u active=%u starts=%u stops=%u watchdog_stops=%u error=%d\n",
			id, go, state, sysctrl, drive, aw->active, aw->starts,
			aw->stops, aw->watchdog_stops, aw->last_error);
	mutex_unlock(&aw->io_lock);
	return ret;
}
static DEVICE_ATTR_RO(status);
static struct attribute *samurai_aw_attrs[] = {
	&dev_attr_status.attr,
	NULL,
};
static const struct attribute_group samurai_aw_group = {
	.attrs = samurai_aw_attrs,
};

static int samurai_aw_probe(struct i2c_client *client)
{
	struct samurai_aw *aw;
	struct input_dev *input;
	int ret;

	aw = devm_kzalloc(&client->dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	aw->dev = &client->dev;
	aw->map = devm_regmap_init_i2c(client, &samurai_aw_regmap);
	if (IS_ERR(aw->map))
		return PTR_ERR(aw->map);
	aw->reset = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset))
		return dev_err_probe(&client->dev, PTR_ERR(aw->reset), "reset GPIO\n");
	mutex_init(&aw->io_lock);
	spin_lock_init(&aw->request_lock);
	INIT_DELAYED_WORK(&aw->work, samurai_aw_work);
	aw->suspended = true;
	i2c_set_clientdata(client, aw);
	ret = devm_add_action_or_reset(&client->dev, samurai_aw_release, aw);
	if (ret)
		return ret;
	ret = samurai_aw_init_hw(aw);
	if (ret)
		return dev_err_probe(&client->dev, ret, "AW8697 identity/standby\n");
	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return -ENOMEM;
	input->name = "Realme X2 Pro AW8697 haptics";
	input->id.bustype = BUS_I2C;
	input->open = samurai_aw_open;
	input->close = samurai_aw_close;
	input_set_drvdata(input, aw);
	input_set_capability(input, EV_FF, FF_RUMBLE);
	ret = input_ff_create_memless(input, aw, samurai_aw_play);
	if (ret)
		return ret;
	ret = input_register_device(input);
	if (ret)
		return ret;
	ret = devm_device_add_group(&client->dev, &samurai_aw_group);
	if (ret)
		return ret;
	dev_info(&client->dev, "AW8697 id=97 standby; 170Hz drive ceiling=52 pulse limit=%ums\n", AW_MAX_MS);
	return 0;
}

static const struct of_device_id samurai_aw_match[] = {
	{ .compatible = "realme,samurai-aw8697" },
	{ }
};
MODULE_DEVICE_TABLE(of, samurai_aw_match);

static int samurai_aw_suspend(struct device *dev)
{
	struct samurai_aw *aw = dev_get_drvdata(dev);

	samurai_aw_quiesce(aw);
	gpiod_set_value_cansleep(aw->reset, 1);
	return 0;
}

static int samurai_aw_resume(struct device *dev)
{
	struct samurai_aw *aw = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	mutex_lock(&aw->io_lock);
	ret = samurai_aw_init_hw(aw);
	if (ret)
		samurai_aw_fault(aw, ret);
	mutex_unlock(&aw->io_lock);
	if (ret)
		return ret;
	spin_lock_irqsave(&aw->request_lock, flags);
	aw->requested = 0;
	aw->suspended = false;
	spin_unlock_irqrestore(&aw->request_lock, flags);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(samurai_aw_pm, samurai_aw_suspend, samurai_aw_resume);

static void samurai_aw_shutdown(struct i2c_client *client)
{
	samurai_aw_release(i2c_get_clientdata(client));
}

static struct i2c_driver samurai_aw_driver = {
	.driver = { .name = "aw8697-samurai", .of_match_table = samurai_aw_match,
		    .pm = pm_sleep_ptr(&samurai_aw_pm) },
	.probe = samurai_aw_probe,
	.shutdown = samurai_aw_shutdown,
};
module_i2c_driver(samurai_aw_driver);
MODULE_DESCRIPTION("RMX1931 bounded AW8697 continuous-mode force feedback");
MODULE_LICENSE("GPL");
