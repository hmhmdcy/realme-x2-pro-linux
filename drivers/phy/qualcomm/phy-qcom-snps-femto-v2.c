// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include "phy-qcom-snps-femto-v2-window.h"

#define USB2_PHY_USB_PHY_UTMI_CTRL0		(0x3c)
#define SLEEPM					BIT(0)
#define OPMODE_MASK				GENMASK(4, 3)
#define OPMODE_NORMAL				(0x00)
#define OPMODE_NONDRIVING			BIT(3)
#define TERMSEL					BIT(5)

#define USB2_PHY_USB_PHY_UTMI_CTRL1		(0x40)
#define XCVRSEL					BIT(0)

#define USB2_PHY_USB_PHY_UTMI_CTRL5		(0x50)
#define POR					BIT(1)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON0	(0x54)
#define SIDDQ					BIT(2)
#define RETENABLEN				BIT(3)
#define FSEL_MASK				GENMASK(6, 4)
#define FSEL_DEFAULT				(0x3 << 4)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON1	(0x58)
#define VBUSVLDEXTSEL0				BIT(4)
#define PLLBTUNE				BIT(5)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON2	(0x5c)
#define VREGBYPASS				BIT(0)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL1		(0x60)
#define VBUSVLDEXT0				BIT(0)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL2		(0x64)
#define USB2_AUTO_RESUME			BIT(0)
#define USB2_SUSPEND_N				BIT(2)
#define USB2_SUSPEND_N_SEL			BIT(3)

#define USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X0		(0x6c)
#define USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X1		(0x70)
#define USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X2		(0x74)
#define USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X3		(0x78)
#define PARAM_OVRD_MASK				0xFF

#define USB2_PHY_USB_PHY_CFG0			(0x94)
#define UTMI_PHY_DATAPATH_CTRL_OVERRIDE_EN	BIT(0)
#define UTMI_PHY_CMN_CTRL_OVERRIDE_EN		BIT(1)

#define USB2_PHY_USB_PHY_REFCLK_CTRL		(0xa0)
#define REFCLK_SEL_MASK				GENMASK(1, 0)
#define REFCLK_SEL_DEFAULT			(0x2 << 0)

#define HS_DISCONNECT_MASK			GENMASK(2, 0)
#define SQUELCH_DETECTOR_MASK			GENMASK(7, 5)

#define HS_AMPLITUDE_MASK			GENMASK(3, 0)
#define PREEMPHASIS_DURATION_MASK		BIT(5)
#define PREEMPHASIS_AMPLITUDE_MASK		GENMASK(7, 6)

#define HS_RISE_FALL_MASK			GENMASK(1, 0)
#define HS_CROSSOVER_VOLTAGE_MASK		GENMASK(3, 2)
#define HS_OUTPUT_IMPEDANCE_MASK		GENMASK(5, 4)

#define LS_FS_OUTPUT_IMPEDANCE_MASK		GENMASK(3, 0)

static const char * const qcom_snps_hsphy_vreg_names[] = {
	"vdda-pll", "vdda33", "vdda18",
};

#define SNPS_HS_NUM_VREGS		ARRAY_SIZE(qcom_snps_hsphy_vreg_names)

struct override_param {
	s32	value;
	u8	reg_val;
};

struct override_param_map {
	const char *prop_name;
	const struct override_param *param_table;
	u8 table_size;
	u8 reg_offset;
	u8 param_mask;
};

struct phy_override_seq {
	bool	need_update;
	u8	offset;
	u8	value;
	u8	mask;
};

#define NUM_HSPHY_TUNING_PARAMS	(9)

struct qcom_hsphy_window_provider;

/**
 * struct qcom_snps_hsphy - snps hs phy attributes
 *
 * @dev: device structure
 *
 * @phy: generic phy
 * @base: iomapped memory space for snps hs phy
 *
 * @num_clks: number of clocks
 * @clks: array of clocks
 * @phy_reset: phy reset control
 * @vregs: regulator supplies bulk data
 * @phy_initialized: if PHY has been initialized correctly
 * @window: owned masked-state receipt, protected by generic PHY mutex
 * @window_generation: never-zero per-provider cookie generation
 * @window_pm_ref: active-only provider runtime reference retained by window
 * @window_withdrawing: provider removal/shutdown has revoked new operations
 * @window_provider: separately referenced publication and withdrawal state
 * @mode: contains the current mode the PHY is in
 * @update_seq_cfg: tuning parameters for phy init
 */
struct qcom_snps_hsphy {
	struct device *dev;

	struct phy *phy;
	void __iomem *base;

	int num_clks;
	struct clk_bulk_data *clks;
	struct reset_control *phy_reset;
	struct regulator_bulk_data vregs[SNPS_HS_NUM_VREGS];

	bool phy_initialized;
	struct qcom_snps_hsphy_window_result window;
	u64 window_generation;
	bool window_pm_ref;
	bool window_withdrawing;
	struct qcom_hsphy_window_provider *window_provider;
	enum phy_mode mode;
	struct phy_override_seq update_seq_cfg[NUM_HSPHY_TUNING_PARAMS];
};

static int qcom_snps_hsphy_clk_init(struct qcom_snps_hsphy *hsphy)
{
	struct device *dev = hsphy->dev;

	hsphy->num_clks = 2;
	hsphy->clks = devm_kcalloc(dev, hsphy->num_clks, sizeof(*hsphy->clks), GFP_KERNEL);
	if (!hsphy->clks)
		return -ENOMEM;

	/*
	 * TODO: Currently no device tree instantiation of the PHY is using the clock.
	 * This needs to be fixed in order for this code to be able to use devm_clk_bulk_get().
	 */
	hsphy->clks[0].id = "cfg_ahb";
	hsphy->clks[0].clk = devm_clk_get_optional(dev, "cfg_ahb");
	if (IS_ERR(hsphy->clks[0].clk))
		return dev_err_probe(dev, PTR_ERR(hsphy->clks[0].clk),
				     "failed to get cfg_ahb clk\n");

	hsphy->clks[1].id = "ref";
	hsphy->clks[1].clk = devm_clk_get(dev, "ref");
	if (IS_ERR(hsphy->clks[1].clk))
		return dev_err_probe(dev, PTR_ERR(hsphy->clks[1].clk),
				     "failed to get ref clk\n");

	return 0;
}

static inline void qcom_snps_hsphy_write_mask(void __iomem *base, u32 offset,
						u32 mask, u32 val)
{
	u32 reg;

	reg = readl_relaxed(base + offset);
	reg &= ~mask;
	reg |= val & mask;
	writel_relaxed(reg, base + offset);

	/* Ensure above write is completed */
	readl_relaxed(base + offset);
}

static const struct phy_ops qcom_snps_hsphy_gen_ops;

static bool qcom_snps_hsphy_window_busy(struct qcom_snps_hsphy *hsphy)
{
	return hsphy->window.state != QCOM_HSPHY_WINDOW_IDLE;
}

static int qcom_snps_hsphy_window_write(struct qcom_snps_hsphy *hsphy,
				      u32 offset, u32 mask, u32 value)
{
	u32 actual;

	lockdep_assert_held(&hsphy->phy->mutex);
	actual = readl_relaxed(hsphy->base + offset);
	writel_relaxed((actual & ~mask) | (value & mask), hsphy->base + offset);
	actual = readl_relaxed(hsphy->base + offset);
	return (actual & mask) == (value & mask) ? 0 : -EIO;
}

/* Same three masked operations and entry order as the pinned OEM DPDM
 * regulator. Power, clocks and reset are deliberately not borrowed from its
 * enable path: only an initialized, already-active PHY can be claimed. */
static const u32 qcom_snps_hsphy_window_offsets[] = {
	USB2_PHY_USB_PHY_HS_PHY_CTRL2,
	USB2_PHY_USB_PHY_UTMI_CTRL0,
	USB2_PHY_USB_PHY_CFG0,
};
static const u32 qcom_snps_hsphy_window_masks[] = {
	USB2_SUSPEND_N_SEL | USB2_SUSPEND_N,
	OPMODE_MASK,
	UTMI_PHY_DATAPATH_CTRL_OVERRIDE_EN,
};
static const u32 qcom_snps_hsphy_window_values[] = {
	USB2_SUSPEND_N_SEL | USB2_SUSPEND_N,
	OPMODE_NONDRIVING,
	UTMI_PHY_DATAPATH_CTRL_OVERRIDE_EN,
};

/**
 * qcom_snps_hsphy_window_bound_begin - borrow this active PHY's masked UTMI state
 * @phy: bound femto-v2 PHY protected by the caller's DWC active HALT window
 * @result: cookie, exact old values, and per-step readback receipt
 *
 * Process context only; caller holds no PHY/DWC/EUD component lock. A failed
 * enter with a cookie remains owned, with its PM reference and old values.
 * It is not a high-Z success and requires explicit end after detection stops.
 */
static int qcom_snps_hsphy_window_bound_begin(struct phy *phy,
		struct qcom_snps_hsphy_window_result *result)
{
	struct qcom_snps_hsphy_window_result *window;
	struct qcom_snps_hsphy *hsphy;
	int i, ret;

	if (!result)
		return -EINVAL;
	memset(result, 0, sizeof(*result));
	if (!phy || phy->ops != &qcom_snps_hsphy_gen_ops)
		return -ENODEV;
	hsphy = phy_get_drvdata(phy);
	/* Never resume to gain a detection window; PM operations are outside
	 * the generic PHY mutex, including all failure/ref-release paths. */
	ret = pm_runtime_get_if_active(hsphy->dev);
	if (ret <= 0)
		return ret < 0 ? ret : -EAGAIN;
	mutex_lock(&phy->mutex);
	window = &hsphy->window;
	if (qcom_snps_hsphy_window_busy(hsphy))
		ret = -EBUSY;
	else if (!hsphy->phy_initialized || phy->init_count <= 0)
		ret = -EAGAIN;
	else if (hsphy->mode != PHY_MODE_USB_DEVICE)
		ret = -EOPNOTSUPP;
	else {
		memset(window, 0, sizeof(*window));
		if (!++hsphy->window_generation)
			++hsphy->window_generation;
		window->cookie = hsphy->window_generation;
		window->state = QCOM_HSPHY_WINDOW_CLAIMED;
		hsphy->window_pm_ref = true;
		/* Capture all originals before any write, including when entering
		 * later fails part-way. Never synthesize normal/default values. */
		for (i = 0; i < ARRAY_SIZE(qcom_snps_hsphy_window_offsets); i++)
			window->saved[i] = readl_relaxed(hsphy->base +
				qcom_snps_hsphy_window_offsets[i]) & qcom_snps_hsphy_window_masks[i];
		for (i = 0; i < ARRAY_SIZE(qcom_snps_hsphy_window_offsets); i++) {
			window->attempted |= BIT(i);
			ret = qcom_snps_hsphy_window_write(hsphy,
				qcom_snps_hsphy_window_offsets[i],
				qcom_snps_hsphy_window_masks[i], qcom_snps_hsphy_window_values[i]);
			if (ret)
				break;
			window->verified |= BIT(i);
		}
		window->enter_error = ret;
		window->high_z = !ret;
		window->state = ret ? QCOM_HSPHY_WINDOW_FAILED : QCOM_HSPHY_WINDOW_HIGH_Z;
		*result = *window;
		mutex_unlock(&phy->mutex);
		return ret;
	}
	mutex_unlock(&phy->mutex);
	pm_runtime_put_noidle(hsphy->dev);
	return ret;
}

/**
 * qcom_snps_hsphy_window_bound_end - restore exactly the borrowed masked state
 * @phy: same bound PHY, still under caller's active DWC/lifetime protection
 * @cookie: nonzero begin cookie, including a failed enter's cookie
 * @result: old values and independent enter/restore error receipt
 *
 * Reverse entry order; each write is read back. Failure preserves ownership
 * and PM reference for explicit retry; it never reconnects DWC/EUD. Success
 * means these masked values were restored, not that USB enumerated.
 */
static int qcom_snps_hsphy_window_bound_end(struct phy *phy, u64 cookie,
		struct qcom_snps_hsphy_window_result *result)
{
	struct qcom_snps_hsphy_window_result *window;
	struct qcom_snps_hsphy *hsphy;
	int i, ret = 0;

	if (!result)
		return -EINVAL;
	memset(result, 0, sizeof(*result));
	if (!phy || phy->ops != &qcom_snps_hsphy_gen_ops)
		return -ENODEV;
	hsphy = phy_get_drvdata(phy);
	mutex_lock(&phy->mutex);
	window = &hsphy->window;
	if (!cookie || cookie != window->cookie || !qcom_snps_hsphy_window_busy(hsphy)) {
		ret = -ESTALE;
		goto unlock;
	}
	/* exit/set_mode/runtime callbacks cannot invalidate this snapshot. */
	if (!hsphy->phy_initialized || phy->init_count <= 0 || !hsphy->window_pm_ref) {
		ret = -ESHUTDOWN;
		goto failed;
	}
	window->high_z = false;
	window->restored_mask = 0;
	for (i = ARRAY_SIZE(qcom_snps_hsphy_window_offsets) - 1; i >= 0; i--) {
		ret = qcom_snps_hsphy_window_write(hsphy,
			qcom_snps_hsphy_window_offsets[i],
			qcom_snps_hsphy_window_masks[i], window->saved[i]);
		if (ret)
			goto failed;
		window->restored_mask |= BIT(i);
	}
	window->restored = true;
	window->restore_error = 0;
	window->state = QCOM_HSPHY_WINDOW_IDLE;
	hsphy->window_pm_ref = false;
	*result = *window;
	mutex_unlock(&phy->mutex);
	pm_runtime_mark_last_busy(hsphy->dev);
	pm_runtime_put_autosuspend(hsphy->dev);
	return 0;
failed:
	window->high_z = false;
	window->restore_error = ret;
	window->state = QCOM_HSPHY_WINDOW_FAILED;
	*result = *window;
unlock:
	mutex_unlock(&phy->mutex);
	return ret;
}

/* Registry protects publication only. Provider lock protects bound storage;
 * no client callback runs under either it or the generic PHY mutex. */
struct qcom_hsphy_window_provider;
struct qcom_snps_hsphy_window_client {
	struct kref ref;
	struct qcom_hsphy_window_provider *provider;
	struct qcom_snps_hsphy_window_client_ops ops;
	void *context;
};
struct qcom_hsphy_window_provider {
	struct list_head node;
	struct kref ref;
	struct mutex lock;
	struct completion withdrawn;
	struct phy *key; /* compare only under registry; never dereference */
	struct qcom_snps_hsphy *bound;
	struct qcom_snps_hsphy_window_client *client;
	struct qcom_snps_hsphy_window_result receipt;
	bool withdrawing, dead;
	int withdraw_error;
};
static LIST_HEAD(qcom_hsphy_window_providers);
static DEFINE_MUTEX(qcom_hsphy_window_registry);

static void qcom_hsphy_window_provider_free(struct kref *ref)
{
	struct qcom_hsphy_window_provider *p = container_of(ref,
		struct qcom_hsphy_window_provider, ref);
	mutex_destroy(&p->lock);
	kfree(p);
}

static void qcom_hsphy_window_client_free(struct kref *ref)
{
	struct qcom_snps_hsphy_window_client *c = container_of(ref,
		struct qcom_snps_hsphy_window_client, ref);
	/* Context release can destroy the owner; always outside component locks. */
	c->ops.put(c->context);
	kref_put(&c->provider->ref, qcom_hsphy_window_provider_free);
	kfree(c);
}

int qcom_snps_hsphy_window_client_register(struct phy *phy,
		const struct qcom_snps_hsphy_window_client_ops *ops, void *context,
		struct qcom_snps_hsphy_window_client **client)
{
	struct qcom_snps_hsphy_window_client *c;
	struct qcom_hsphy_window_provider *p;
	int ret = -ENODEV;

	if (!client)
		return -EINVAL;
	*client = NULL;
	if (!phy || !ops || !ops->get_unless_zero || !ops->put || !ops->cancel_join)
		return -EINVAL;
	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	/* Pins callback code/data until final client or withdrawal callback put. */
	if (!ops->get_unless_zero(context)) {
		kfree(c);
		return -ESHUTDOWN;
	}
	kref_init(&c->ref);
	c->ops = *ops;
	c->context = context;
	mutex_lock(&qcom_hsphy_window_registry);
	list_for_each_entry(p, &qcom_hsphy_window_providers, node) {
		if (p->key != phy)
			continue;
		mutex_lock(&p->lock);
		if (p->withdrawing || p->dead)
			ret = -ESHUTDOWN;
		else if (p->client)
			ret = -EBUSY;
		else {
			kref_get(&p->ref);
			c->provider = p;
			p->client = c;
			*client = c;
			ret = 0;
		}
		mutex_unlock(&p->lock);
		break;
	}
	mutex_unlock(&qcom_hsphy_window_registry);
	if (ret) {
		ops->put(context);
		kfree(c);
	}
	return ret;
}

int qcom_snps_hsphy_window_client_unregister(struct qcom_snps_hsphy_window_client *c)
{
	struct qcom_hsphy_window_provider *p;
	bool busy = false;

	if (!c)
		return -EINVAL;
	p = c->provider;
	mutex_lock(&p->lock);
	if (p->bound) {
		mutex_lock(&p->bound->phy->mutex);
		busy = qcom_snps_hsphy_window_busy(p->bound);
		mutex_unlock(&p->bound->phy->mutex);
	}
	if (!busy)
		p->client = NULL;
	mutex_unlock(&p->lock);
	if (busy)
		return -EBUSY; /* preserve context/handle until restored or revoked */
	kref_put(&c->ref, qcom_hsphy_window_client_free);
	return 0;
}

int qcom_snps_hsphy_window_begin(struct qcom_snps_hsphy_window_client *c,
		struct qcom_snps_hsphy_window_result *result)
{
	struct qcom_hsphy_window_provider *p;
	int ret;

	if (!c || !result)
		return -EINVAL;
	memset(result, 0, sizeof(*result));
	p = c->provider;
	mutex_lock(&p->lock);
	if (p->withdrawing || !p->bound)
		ret = -ESHUTDOWN;
	else
		ret = qcom_snps_hsphy_window_bound_begin(p->bound->phy, result);
	mutex_unlock(&p->lock);
	return ret;
}

int qcom_snps_hsphy_window_end(struct qcom_snps_hsphy_window_client *c, u64 cookie,
		struct qcom_snps_hsphy_window_result *result)
{
	struct qcom_hsphy_window_provider *p;
	int ret;

	if (!c || !result)
		return -EINVAL;
	memset(result, 0, sizeof(*result));
	p = c->provider;
	mutex_lock(&p->lock);
	/* The withdrawal callback may restore while storage is still bound.
	 * Once tombstoned, no stale handle may touch PHY/PM/devres again. */
	if (!p->bound) {
		*result = p->receipt;
		ret = -ESHUTDOWN;
	} else {
		ret = qcom_snps_hsphy_window_bound_end(p->bound->phy, cookie, result);
	}
	mutex_unlock(&p->lock);
	return ret;
}

int qcom_snps_hsphy_window_status(struct qcom_snps_hsphy_window_client *c,
		struct qcom_snps_hsphy_window_result *result)
{
	struct qcom_hsphy_window_provider *p;

	if (!c || !result)
		return -EINVAL;
	p = c->provider;
	mutex_lock(&p->lock);
	if (p->bound) {
		mutex_lock(&p->bound->phy->mutex);
		*result = p->bound->window;
		mutex_unlock(&p->bound->phy->mutex);
		result->revoked = p->withdrawing;
	} else {
		*result = p->receipt;
	}
	mutex_unlock(&p->lock);
	return 0;
}

static int qcom_hsphy_window_withdraw(struct qcom_hsphy_window_provider *p)
{
	struct qcom_snps_hsphy_window_client *c;
	struct qcom_snps_hsphy *hsphy;
	bool put_pm = false;
	int error = 0;

	/* Each concurrent withdrawal keeps its completion/receipt storage alive,
	 * even if devres drops the publication reference before a waiter wakes. */
	kref_get(&p->ref);
	mutex_lock(&qcom_hsphy_window_registry);
	mutex_lock(&p->lock);
	if (p->withdrawing) {
		mutex_unlock(&p->lock);
		mutex_unlock(&qcom_hsphy_window_registry);
		wait_for_completion(&p->withdrawn);
		error = p->withdraw_error;
		kref_put(&p->ref, qcom_hsphy_window_provider_free);
		return error;
	}
	p->withdrawing = true;
	list_del_init(&p->node);
	hsphy = p->bound;
	mutex_lock(&hsphy->phy->mutex);
	hsphy->window_withdrawing = true;
	mutex_unlock(&hsphy->phy->mutex);
	c = p->client;
	if (c)
		kref_get(&c->ref);
	mutex_unlock(&p->lock);
	mutex_unlock(&qcom_hsphy_window_registry);
	if (c)
		error = c->ops.cancel_join(c->context);
	mutex_lock(&p->lock);
	mutex_lock(&hsphy->phy->mutex);
	p->receipt = hsphy->window;
	if (qcom_snps_hsphy_window_busy(hsphy)) {
		if (!error)
			error = p->receipt.restore_error ? p->receipt.restore_error : -ESHUTDOWN;
		p->receipt.high_z = false;
		p->receipt.restored = false;
		/* Forced removal cannot retain permission to retry on freed MMIO.
		 * No second restore attempt or reconnect is invented here. */
		put_pm = hsphy->window_pm_ref;
		hsphy->window_pm_ref = false;
	}
	p->receipt.revoked = true;
	p->receipt.withdraw_error = error;
	p->withdraw_error = error;
	p->bound = NULL;
	p->dead = true;
	mutex_unlock(&hsphy->phy->mutex);
	mutex_unlock(&p->lock);
	if (put_pm)
		pm_runtime_put_noidle(hsphy->dev);
	if (c)
		kref_put(&c->ref, qcom_hsphy_window_client_free);
	complete_all(&p->withdrawn);
	kref_put(&p->ref, qcom_hsphy_window_provider_free);
	return error;
}

static void qcom_hsphy_window_release(void *data)
{
	struct qcom_hsphy_window_provider *p = data;

	qcom_hsphy_window_withdraw(p);
	kref_put(&p->ref, qcom_hsphy_window_provider_free);
}

static int qcom_hsphy_window_publish(struct qcom_snps_hsphy *hsphy)
{
	struct qcom_hsphy_window_provider *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	kref_init(&p->ref);
	mutex_init(&p->lock);
	init_completion(&p->withdrawn);
	INIT_LIST_HEAD(&p->node);
	p->key = hsphy->phy;
	p->bound = hsphy;
	hsphy->window_provider = p;
	mutex_lock(&qcom_hsphy_window_registry);
	list_add_tail(&p->node, &qcom_hsphy_window_providers);
	mutex_unlock(&qcom_hsphy_window_registry);
	/* Installed after generic PHY/provider/PM devres: withdrawal runs first
	 * on probe rollback as well as ordinary unbind. */
	return devm_add_action_or_reset(hsphy->dev, qcom_hsphy_window_release, p);
}

static void qcom_snps_hsphy_remove(struct platform_device *pdev)
{
	struct qcom_snps_hsphy *hsphy = dev_get_drvdata(&pdev->dev);
	int ret = qcom_hsphy_window_withdraw(hsphy->window_provider);

	if (ret)
		dev_warn(&pdev->dev, "PHY window revoked without successful cleanup: %d\n", ret);
}

static void qcom_snps_hsphy_shutdown(struct platform_device *pdev)
{
	qcom_snps_hsphy_remove(pdev);
}


static int qcom_snps_hsphy_suspend(struct qcom_snps_hsphy *hsphy)
{
	dev_dbg(&hsphy->phy->dev, "Suspend QCOM SNPS PHY\n");

	if (hsphy->mode == PHY_MODE_USB_HOST) {
		/* Enable auto-resume to meet remote wakeup timing */
		qcom_snps_hsphy_write_mask(hsphy->base,
					   USB2_PHY_USB_PHY_HS_PHY_CTRL2,
					   USB2_AUTO_RESUME,
					   USB2_AUTO_RESUME);
		usleep_range(500, 1000);
		qcom_snps_hsphy_write_mask(hsphy->base,
					   USB2_PHY_USB_PHY_HS_PHY_CTRL2,
					   0, USB2_AUTO_RESUME);
	}

	return 0;
}

static int qcom_snps_hsphy_resume(struct qcom_snps_hsphy *hsphy)
{
	dev_dbg(&hsphy->phy->dev, "Resume QCOM SNPS PHY, mode\n");

	return 0;
}

static int __maybe_unused qcom_snps_hsphy_runtime_suspend(struct device *dev)
{
	struct qcom_snps_hsphy *hsphy = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&hsphy->phy->mutex);
	if (hsphy->window_withdrawing)
		ret = -ESHUTDOWN;
	else
	if (qcom_snps_hsphy_window_busy(hsphy))
		ret = -EBUSY;
	else if (hsphy->phy_initialized)
		ret = qcom_snps_hsphy_suspend(hsphy);
	mutex_unlock(&hsphy->phy->mutex);
	return ret;
}

static int __maybe_unused qcom_snps_hsphy_runtime_resume(struct device *dev)
{
	struct qcom_snps_hsphy *hsphy = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&hsphy->phy->mutex);
	if (hsphy->window_withdrawing)
		ret = -ESHUTDOWN;
	else
	if (qcom_snps_hsphy_window_busy(hsphy))
		ret = -EBUSY;
	else if (hsphy->phy_initialized)
		ret = qcom_snps_hsphy_resume(hsphy);
	mutex_unlock(&hsphy->phy->mutex);
	return ret;
}

static int qcom_snps_hsphy_set_mode(struct phy *phy, enum phy_mode mode,
				    int submode)
{
	struct qcom_snps_hsphy *hsphy = phy_get_drvdata(phy);

	if (hsphy->window_withdrawing)
		return -ESHUTDOWN;

	if (qcom_snps_hsphy_window_busy(hsphy))
		return -EBUSY;

	hsphy->mode = mode;
	return 0;
}

static const struct override_param hs_disconnect_sc7280[] = {
	{ -272, 0 },
	{ 0, 1 },
	{ 317, 2 },
	{ 630, 3 },
	{ 973, 4 },
	{ 1332, 5 },
	{ 1743, 6 },
	{ 2156, 7 },
};

static const struct override_param squelch_det_threshold_sc7280[] = {
	{ -2090, 7 },
	{ -1560, 6 },
	{ -1030, 5 },
	{ -530, 4 },
	{ 0, 3 },
	{ 530, 2 },
	{ 1060, 1 },
	{ 1590, 0 },
};

static const struct override_param hs_amplitude_sc7280[] = {
	{ -660, 0 },
	{ -440, 1 },
	{ -220, 2 },
	{ 0, 3 },
	{ 230, 4 },
	{ 440, 5 },
	{ 650, 6 },
	{ 890, 7 },
	{ 1110, 8 },
	{ 1330, 9 },
	{ 1560, 10 },
	{ 1780, 11 },
	{ 2000, 12 },
	{ 2220, 13 },
	{ 2430, 14 },
	{ 2670, 15 },
};

static const struct override_param preemphasis_duration_sc7280[] = {
	{ 10000, 1 },
	{ 20000, 0 },
};

static const struct override_param preemphasis_amplitude_sc7280[] = {
	{ 10000, 1 },
	{ 20000, 2 },
	{ 30000, 3 },
	{ 40000, 0 },
};

static const struct override_param hs_rise_fall_time_sc7280[] = {
	{ -4100, 3 },
	{ 0, 2 },
	{ 2810, 1 },
	{ 5430, 0 },
};

static const struct override_param hs_crossover_voltage_sc7280[] = {
	{ -31000, 1 },
	{ 0, 3 },
	{ 28000, 2 },
};

static const struct override_param hs_output_impedance_sc7280[] = {
	{ -2300000, 3 },
	{ 0, 2 },
	{ 2600000, 1 },
	{ 6100000, 0 },
};

static const struct override_param ls_fs_output_impedance_sc7280[] = {
	{ -1053, 15 },
	{ -557, 7 },
	{ 0, 3 },
	{ 612, 1 },
	{ 1310, 0 },
};

static const struct override_param_map sc7280_snps_7nm_phy[] = {
	{
		"qcom,hs-disconnect-bp",
		hs_disconnect_sc7280,
		ARRAY_SIZE(hs_disconnect_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X0,
		HS_DISCONNECT_MASK
	},
	{
		"qcom,squelch-detector-bp",
		squelch_det_threshold_sc7280,
		ARRAY_SIZE(squelch_det_threshold_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X0,
		SQUELCH_DETECTOR_MASK
	},
	{
		"qcom,hs-amplitude-bp",
		hs_amplitude_sc7280,
		ARRAY_SIZE(hs_amplitude_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X1,
		HS_AMPLITUDE_MASK
	},
	{
		"qcom,pre-emphasis-duration-bp",
		preemphasis_duration_sc7280,
		ARRAY_SIZE(preemphasis_duration_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X1,
		PREEMPHASIS_DURATION_MASK,
	},
	{
		"qcom,pre-emphasis-amplitude-bp",
		preemphasis_amplitude_sc7280,
		ARRAY_SIZE(preemphasis_amplitude_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X1,
		PREEMPHASIS_AMPLITUDE_MASK,
	},
	{
		"qcom,hs-rise-fall-time-bp",
		hs_rise_fall_time_sc7280,
		ARRAY_SIZE(hs_rise_fall_time_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X2,
		HS_RISE_FALL_MASK
	},
	{
		"qcom,hs-crossover-voltage-microvolt",
		hs_crossover_voltage_sc7280,
		ARRAY_SIZE(hs_crossover_voltage_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X2,
		HS_CROSSOVER_VOLTAGE_MASK
	},
	{
		"qcom,hs-output-impedance-micro-ohms",
		hs_output_impedance_sc7280,
		ARRAY_SIZE(hs_output_impedance_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X2,
		HS_OUTPUT_IMPEDANCE_MASK,
	},
	{
		"qcom,ls-fs-output-impedance-bp",
		ls_fs_output_impedance_sc7280,
		ARRAY_SIZE(ls_fs_output_impedance_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X3,
		LS_FS_OUTPUT_IMPEDANCE_MASK,
	},
	{},
};

static int qcom_snps_hsphy_init(struct phy *phy)
{
	struct qcom_snps_hsphy *hsphy = phy_get_drvdata(phy);
	int ret, i;

	if (hsphy->window_withdrawing)
		return -ESHUTDOWN;

	if (qcom_snps_hsphy_window_busy(hsphy))
		return -EBUSY;

	dev_vdbg(&phy->dev, "%s(): Initializing SNPS HS phy\n", __func__);

	ret = regulator_bulk_enable(ARRAY_SIZE(hsphy->vregs), hsphy->vregs);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(hsphy->num_clks, hsphy->clks);
	if (ret) {
		dev_err(&phy->dev, "failed to enable clocks, %d\n", ret);
		goto poweroff_phy;
	}

	ret = reset_control_assert(hsphy->phy_reset);
	if (ret) {
		dev_err(&phy->dev, "failed to assert phy_reset, %d\n", ret);
		goto disable_clks;
	}

	usleep_range(100, 150);

	ret = reset_control_deassert(hsphy->phy_reset);
	if (ret) {
		dev_err(&phy->dev, "failed to de-assert phy_reset, %d\n", ret);
		goto disable_clks;
	}

	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_CFG0,
					UTMI_PHY_CMN_CTRL_OVERRIDE_EN,
					UTMI_PHY_CMN_CTRL_OVERRIDE_EN);
	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_UTMI_CTRL5,
							POR, POR);
	qcom_snps_hsphy_write_mask(hsphy->base,
					USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON0,
					FSEL_MASK, 0);
	qcom_snps_hsphy_write_mask(hsphy->base,
					USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON1,
					PLLBTUNE, PLLBTUNE);
	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_REFCLK_CTRL,
					REFCLK_SEL_DEFAULT, REFCLK_SEL_MASK);
	qcom_snps_hsphy_write_mask(hsphy->base,
					USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON1,
					VBUSVLDEXTSEL0, VBUSVLDEXTSEL0);
	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_HS_PHY_CTRL1,
					VBUSVLDEXT0, VBUSVLDEXT0);

	for (i = 0; i < ARRAY_SIZE(hsphy->update_seq_cfg); i++) {
		if (hsphy->update_seq_cfg[i].need_update)
			qcom_snps_hsphy_write_mask(hsphy->base,
					hsphy->update_seq_cfg[i].offset,
					hsphy->update_seq_cfg[i].mask,
					hsphy->update_seq_cfg[i].value);
	}

	qcom_snps_hsphy_write_mask(hsphy->base,
					USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON2,
					VREGBYPASS, VREGBYPASS);

	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_HS_PHY_CTRL2,
					USB2_SUSPEND_N_SEL | USB2_SUSPEND_N,
					USB2_SUSPEND_N_SEL | USB2_SUSPEND_N);

	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_UTMI_CTRL0,
					SLEEPM, SLEEPM);

	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON0,
				   SIDDQ, 0);

	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_UTMI_CTRL5,
					POR, 0);

	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_HS_PHY_CTRL2,
					USB2_SUSPEND_N_SEL, 0);

	qcom_snps_hsphy_write_mask(hsphy->base, USB2_PHY_USB_PHY_CFG0,
					UTMI_PHY_CMN_CTRL_OVERRIDE_EN, 0);

	hsphy->phy_initialized = true;

	return 0;

disable_clks:
	clk_bulk_disable_unprepare(hsphy->num_clks, hsphy->clks);
poweroff_phy:
	regulator_bulk_disable(ARRAY_SIZE(hsphy->vregs), hsphy->vregs);

	return ret;
}

static int qcom_snps_hsphy_exit(struct phy *phy)
{
	struct qcom_snps_hsphy *hsphy = phy_get_drvdata(phy);

	if (hsphy->window_withdrawing)
		return -ESHUTDOWN;

	if (qcom_snps_hsphy_window_busy(hsphy))
		return -EBUSY;

	reset_control_assert(hsphy->phy_reset);
	clk_bulk_disable_unprepare(hsphy->num_clks, hsphy->clks);
	regulator_bulk_disable(ARRAY_SIZE(hsphy->vregs), hsphy->vregs);
	hsphy->phy_initialized = false;

	return 0;
}

static const struct phy_ops qcom_snps_hsphy_gen_ops = {
	.init		= qcom_snps_hsphy_init,
	.exit		= qcom_snps_hsphy_exit,
	.set_mode	= qcom_snps_hsphy_set_mode,
	.owner		= THIS_MODULE,
};

static const struct of_device_id qcom_snps_hsphy_of_match_table[] = {
	{ .compatible	= "qcom,sm8150-usb-hs-phy", },
	{ .compatible	= "qcom,usb-snps-hs-5nm-phy", },
	{
		.compatible	= "qcom,usb-snps-hs-7nm-phy",
		.data		= &sc7280_snps_7nm_phy,
	},
	{ .compatible	= "qcom,usb-snps-femto-v2-phy",	},
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_snps_hsphy_of_match_table);

static const struct dev_pm_ops qcom_snps_hsphy_pm_ops = {
	SET_RUNTIME_PM_OPS(qcom_snps_hsphy_runtime_suspend,
			   qcom_snps_hsphy_runtime_resume, NULL)
};

static void qcom_snps_hsphy_override_param_update_val(
			const struct override_param_map map,
			s32 dt_val, struct phy_override_seq *seq_entry)
{
	int i;

	/*
	 * Param table for each param is in increasing order
	 * of dt values. We need to iterate over the list to
	 * select the entry that matches the dt value and pick
	 * up the corresponding register value.
	 */
	for (i = 0; i < map.table_size - 1; i++) {
		if (map.param_table[i].value == dt_val)
			break;
	}

	seq_entry->need_update = true;
	seq_entry->offset = map.reg_offset;
	seq_entry->mask = map.param_mask;
	seq_entry->value = map.param_table[i].reg_val << __ffs(map.param_mask);
}

static void qcom_snps_hsphy_read_override_param_seq(struct device *dev)
{
	struct device_node *node = dev->of_node;
	s32 val;
	int ret, i;
	struct qcom_snps_hsphy *hsphy;
	const struct override_param_map *cfg = of_device_get_match_data(dev);

	if (!cfg)
		return;

	hsphy = dev_get_drvdata(dev);

	for (i = 0; cfg[i].prop_name != NULL; i++) {
		ret = of_property_read_s32(node, cfg[i].prop_name, &val);
		if (ret)
			continue;

		qcom_snps_hsphy_override_param_update_val(cfg[i], val,
					&hsphy->update_seq_cfg[i]);
		dev_dbg(&hsphy->phy->dev, "Read param: %s dt_val: %d reg_val: 0x%x\n",
			cfg[i].prop_name, val, hsphy->update_seq_cfg[i].value);

	}
}

static int qcom_snps_hsphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_snps_hsphy *hsphy;
	struct phy_provider *phy_provider;
	struct phy *generic_phy;
	int ret, i;
	int num;

	hsphy = devm_kzalloc(dev, sizeof(*hsphy), GFP_KERNEL);
	if (!hsphy)
		return -ENOMEM;

	hsphy->dev = dev;

	hsphy->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hsphy->base))
		return PTR_ERR(hsphy->base);

	ret = qcom_snps_hsphy_clk_init(hsphy);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize clocks\n");

	hsphy->phy_reset = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(hsphy->phy_reset)) {
		dev_err(dev, "failed to get phy core reset\n");
		return PTR_ERR(hsphy->phy_reset);
	}

	num = ARRAY_SIZE(hsphy->vregs);
	for (i = 0; i < num; i++)
		hsphy->vregs[i].supply = qcom_snps_hsphy_vreg_names[i];

	ret = devm_regulator_bulk_get(dev, num, hsphy->vregs);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get regulator supplies\n");

	/*
	 * Enable runtime PM before creating the PHY, phy_create() only enables
	 * it on the PHY device if already enabled on the parent. Hold a usage
	 * reference so callbacks cannot run before the PHY is ready.
	 */
	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret) {
		pm_runtime_put_noidle(dev);
		return ret;
	}
	/*
	 * Prevent runtime pm from being ON by default. Users can enable
	 * it using power/control in sysfs.
	 */
	pm_runtime_forbid(dev);

	generic_phy = devm_phy_create(dev, NULL, &qcom_snps_hsphy_gen_ops);
	if (IS_ERR(generic_phy)) {
		ret = PTR_ERR(generic_phy);
		dev_err(dev, "failed to create phy, %d\n", ret);
		pm_runtime_put_noidle(dev);
		return ret;
	}
	hsphy->phy = generic_phy;

	dev_set_drvdata(dev, hsphy);
	phy_set_drvdata(generic_phy, hsphy);
	qcom_snps_hsphy_read_override_param_seq(dev);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		pm_runtime_put_noidle(dev);
		return PTR_ERR(phy_provider);
	}

	ret = qcom_hsphy_window_publish(hsphy);
	if (ret) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	dev_dbg(dev, "Registered Qcom-SNPS HS phy\n");
	pm_runtime_put(dev);

	return 0;
}

static struct platform_driver qcom_snps_hsphy_driver = {
	.probe		= qcom_snps_hsphy_probe,
	.remove		= qcom_snps_hsphy_remove,
	.shutdown	= qcom_snps_hsphy_shutdown,
	.driver = {
		.name	= "qcom-snps-hs-femto-v2-phy",
		.pm = &qcom_snps_hsphy_pm_ops,
		.of_match_table = qcom_snps_hsphy_of_match_table,
	},
};

module_platform_driver(qcom_snps_hsphy_driver);

MODULE_DESCRIPTION("Qualcomm SNPS FEMTO USB HS PHY V2 driver");
MODULE_LICENSE("GPL v2");
