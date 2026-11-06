/* SPDX-License-Identifier: GPL-2.0 */
#ifndef QCOM_SNPS_FEMTO_V2_WINDOW_H
#define QCOM_SNPS_FEMTO_V2_WINDOW_H
#include <linux/types.h>
struct phy;

/* Internal same-Image interface. Registration pins the real owner context.
 * The caller joins all of its API users before unregister. EBUSY retains the
 * handle/context until exact restoration or completed provider revocation.
 * Withdrawal calls cancel_join without component locks; the callback must
 * stop/join detection and may call end/unregister. A failed end must never
 * reconnect DWC/EUD. status remains readable after provider devres is gone.
 * No handle authorizes route, PD, DWC HALT or a PMIC detection transfer. */
enum qcom_snps_hsphy_window_state {
	QCOM_HSPHY_WINDOW_IDLE,
	QCOM_HSPHY_WINDOW_CLAIMED,
	QCOM_HSPHY_WINDOW_HIGH_Z,
	QCOM_HSPHY_WINDOW_FAILED,
};
struct qcom_snps_hsphy_window_result {
	u64 cookie;
	enum qcom_snps_hsphy_window_state state;
	u32 saved[3]; /* masked old CTRL2, UTMI_CTRL0, CFG0, in entry order */
	u8 attempted, verified, restored_mask;
	bool high_z, restored, revoked;
	int enter_error, restore_error, withdraw_error;
};
struct qcom_snps_hsphy_window_client;
struct qcom_snps_hsphy_window_client_ops {
 bool (*get_unless_zero)(void *context);
 void (*put)(void *context);
 int (*cancel_join)(void *context);
};
int qcom_snps_hsphy_window_client_register(struct phy *phy,
 const struct qcom_snps_hsphy_window_client_ops *ops, void *context,
 struct qcom_snps_hsphy_window_client **client);
int qcom_snps_hsphy_window_client_unregister(struct qcom_snps_hsphy_window_client *client);
int qcom_snps_hsphy_window_begin(struct qcom_snps_hsphy_window_client *client,
 struct qcom_snps_hsphy_window_result *result);
int qcom_snps_hsphy_window_end(struct qcom_snps_hsphy_window_client *client, u64 cookie,
 struct qcom_snps_hsphy_window_result *result);
int qcom_snps_hsphy_window_status(struct qcom_snps_hsphy_window_client *client,
 struct qcom_snps_hsphy_window_result *result);
#endif
