// SPDX-License-Identifier: GPL-2.0-only
/* MP2650 input/status support. Control is not enabled until the board's
 * protection chain and input budget are verified. This driver never writes
 * configuration, enables ADC, services the watchdog or reads fault REG14.
 * MPS Rev1.0 REG13 ACOK is active high; the OEM header reverses its constants.
 */
#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>

#define MP2650_STATUS 0x13
#define MP2650_VIN_LOW 0x1c
#define MP2650_IIN_LOW 0x1e
#define MP2650_ACOK BIT(1)
#define MP2650_CHG_STAT GENMASK(3, 2)

struct mp2650 {
	struct i2c_client *client;
	struct mutex lock;
};

static int mp2650_read(struct mp2650 *mp, unsigned int reg)
{
	if (reg != MP2650_STATUS && reg != MP2650_VIN_LOW &&
	    reg != MP2650_VIN_LOW + 1 && reg != MP2650_IIN_LOW &&
	    reg != MP2650_IIN_LOW + 1)
		return -EINVAL;

	return i2c_smbus_read_byte_data(mp->client, reg);
}

/* Keep the OEM single-byte protocol. Accept only a stable high byte across
 * the low-byte read. Up to three attempts accommodate live ADC changes.
 * Bus errors stop immediately; unavailable ADC data must not drop the whole
 * power_supply uevent, so consistency failure returns ENODATA.
 */
static int mp2650_adc(struct mp2650 *mp, unsigned int low_reg, int *value)
{
	int high, low, check, attempt;

	for (attempt = 0; attempt < 3; attempt++) {
		high = mp2650_read(mp, low_reg + 1);
		if (high < 0)
			return high;
		low = mp2650_read(mp, low_reg);
		if (low < 0)
			return low;
		check = mp2650_read(mp, low_reg + 1);
		if (check < 0)
			return check;
		if (high == check) {
			*value = (high << 2) | (low >> 6);
			return 0;
		}
	}
	return -ENODATA;
}

static int mp2650_get_property(struct power_supply *psy,
			       enum power_supply_property property,
			       union power_supply_propval *value)
{
	struct mp2650 *mp = power_supply_get_drvdata(psy);
	int status, after, code, ret = 0;

	mutex_lock(&mp->lock);
	status = mp2650_read(mp, MP2650_STATUS);
	if (status < 0) {
		ret = status;
		goto out;
	}
	switch (property) {
	case POWER_SUPPLY_PROP_ONLINE:
		value->intval = !!(status & MP2650_ACOK);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!(status & MP2650_ACOK)) {
			value->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		}
		switch (FIELD_GET(MP2650_CHG_STAT, status)) {
		case 1:
		case 2:
			value->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 3:
			value->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			value->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* With no valid input the ADC can be disabled in battery-only
		 * mode. Do not enable it or turn an unavailable value into zero.
		 */
		if (!(status & MP2650_ACOK)) {
			ret = -ENODATA;
			break;
		}
		ret = mp2650_adc(mp, property == POWER_SUPPLY_PROP_VOLTAGE_NOW ?
				 MP2650_VIN_LOW : MP2650_IIN_LOW, &code);
		if (ret)
			break;
		after = mp2650_read(mp, MP2650_STATUS);
		if (after < 0) {
			ret = after;
			break;
		}
		if (!(after & MP2650_ACOK) ||
		    ((after ^ status) & MP2650_CHG_STAT)) {
			ret = -ENODATA;
			break;
		}
		/* Standard power_supply units: uV/uA, input-side measurements. */
		value->intval = code * (property == POWER_SUPPLY_PROP_VOLTAGE_NOW ?
				      25000 : 6250);
		break;
	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&mp->lock);
	return ret;
}

static enum power_supply_property mp2650_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static const struct power_supply_desc mp2650_supply = {
	.name = "mp2650-input",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mp2650_properties,
	.num_properties = ARRAY_SIZE(mp2650_properties),
	.get_property = mp2650_get_property,
};

static int mp2650_probe(struct i2c_client *client)
{
	struct power_supply_config config = {};
	struct power_supply *psy;
	struct mp2650 *mp;
	int status;

	if (client->addr != 0x5c)
		return -EINVAL;
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA))
		return -EOPNOTSUPP;
	mp = devm_kzalloc(&client->dev, sizeof(*mp), GFP_KERNEL);
	if (!mp)
		return -ENOMEM;
	mp->client = client;
	mutex_init(&mp->lock);
	status = mp2650_read(mp, MP2650_STATUS);
	if (status < 0)
		return status;
	config.drv_data = mp;
	config.fwnode = dev_fwnode(&client->dev);
	psy = devm_power_supply_register(&client->dev, &mp2650_supply, &config);
	return PTR_ERR_OR_ZERO(psy);
}

static const struct of_device_id mp2650_of_match[] = {
	{ .compatible = "mps,mp2650" },
	{}
};
MODULE_DEVICE_TABLE(of, mp2650_of_match);

static const struct i2c_device_id mp2650_i2c_ids[] = {
	{ "mp2650" },
	{}
};
MODULE_DEVICE_TABLE(i2c, mp2650_i2c_ids);

static struct i2c_driver mp2650_driver = {
	.driver = {
		.name = "mp2650",
		.of_match_table = mp2650_of_match,
	},
	.probe = mp2650_probe,
	.id_table = mp2650_i2c_ids,
};
module_i2c_driver(mp2650_driver);

MODULE_DESCRIPTION("MP2650 input and charger status support");
MODULE_LICENSE("GPL");
