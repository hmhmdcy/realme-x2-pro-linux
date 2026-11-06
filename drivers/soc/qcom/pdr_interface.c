// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 The Linux Foundation. All rights reserved.
 */

#include <linux/cleanup.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "pdr_internal.h"

struct pdr_service {
	char service_name[SERVREG_NAME_LENGTH + 1];
	char service_path[SERVREG_NAME_LENGTH + 1];

	struct sockaddr_qrtr addr;

	unsigned int instance;
	unsigned int service;
	u8 service_data_valid;
	u32 service_data;
	int state;

	bool need_notifier_register;
	bool need_notifier_remove;
	bool need_locator_lookup;
	bool service_connected;
	bool lookup_failed;
	u64 server_generation;
	u64 state_sequence;
	u64 locator_attempt;
	struct qmi_txn *register_txn;

	struct list_head node;
};

struct pdr_handle {
	struct qmi_handle locator_hdl;
	struct qmi_handle notifier_hdl;

	struct sockaddr_qrtr locator_addr;

	struct list_head lookups;
	struct list_head retired;
	struct list_head indack_list;

	/* control access to pdr lookup/indack lists */
	struct mutex list_lock;

	/* serialize pd status invocation */
	struct mutex status_lock;

	/* control access to the locator state */
	struct mutex lock;

	bool locator_init_complete;
	bool stopping;
	u64 locator_generation;
	u64 locator_run;

	struct work_struct locator_work;
	struct work_struct notifier_work;
	struct work_struct indack_work;

	struct workqueue_struct *notifier_wq;
	struct workqueue_struct *indack_wq;

	void (*status)(int state, char *service_path, void *priv);
	void *priv;
};

struct pdr_list_node {
	enum servreg_service_state curr_state;
	u16 transaction_id;
	struct pdr_service *pds;
	struct sockaddr_qrtr addr;
	u64 server_generation;
	struct list_head node;
};

/* list_lock keeps the stack transaction alive until its owner detaches it. */
static void pdr_cancel_registration(struct pdr_handle *pdr, struct pdr_service *pds)
{
	struct qmi_txn *txn = pds->register_txn;

	lockdep_assert_held(&pdr->list_lock);
	if (!txn)
		return;
	mutex_lock(&txn->lock);
	txn->result = -ENETRESET;
	complete(&txn->completion);
	mutex_unlock(&txn->lock);
	pds->register_txn = NULL;
}

static int pdr_locator_new_server(struct qmi_handle *qmi,
				  struct qmi_service *svc)
{
	struct pdr_handle *pdr = container_of(qmi, struct pdr_handle, locator_hdl);

	mutex_lock(&pdr->lock);
	if (READ_ONCE(pdr->stopping)) {
		mutex_unlock(&pdr->lock);
		return -ESHUTDOWN;
	}
	pdr->locator_addr.sq_family = AF_QIPCRTR;
	pdr->locator_addr.sq_node = svc->node;
	pdr->locator_addr.sq_port = svc->port;
	pdr->locator_generation++;
	pdr->locator_init_complete = true;
	schedule_work(&pdr->locator_work);
	mutex_unlock(&pdr->lock);

	return 0;
}

static void pdr_locator_del_server(struct qmi_handle *qmi,
				   struct qmi_service *svc)
{
	struct pdr_handle *pdr = container_of(qmi, struct pdr_handle, locator_hdl);

	mutex_lock(&pdr->lock);
	if (!READ_ONCE(pdr->stopping) && pdr->locator_init_complete &&
	    pdr->locator_addr.sq_node == svc->node &&
	    pdr->locator_addr.sq_port == svc->port) {
		pdr->locator_init_complete = false;
		pdr->locator_generation++;
		pdr->locator_addr.sq_node = 0;
		pdr->locator_addr.sq_port = 0;
	}
	mutex_unlock(&pdr->lock);
}

static const struct qmi_ops pdr_locator_ops = {
	.new_server = pdr_locator_new_server,
	.del_server = pdr_locator_del_server,
};

static int pdr_register_listener(struct pdr_handle *pdr, struct pdr_service *pds,
				 struct sockaddr_qrtr *addr, u64 generation,
				 bool enable, int *state)
{
	const char *service_path = pds->service_path;
	struct servreg_register_listener_resp resp = { };
	struct servreg_register_listener_req req;
	struct qmi_txn txn;
	int ret;

	ret = qmi_txn_init(&pdr->notifier_hdl, &txn,
			   servreg_register_listener_resp_ei, &resp);
	if (ret < 0)
		return ret;

	mutex_lock(&pdr->list_lock);
	if (pdr->stopping || pds->lookup_failed || !pds->service_connected ||
	    pds->server_generation != generation) {
		mutex_unlock(&pdr->list_lock);
		qmi_txn_cancel(&txn);
		return -ENETRESET;
	}
	pds->register_txn = &txn;
	mutex_unlock(&pdr->list_lock);

	req.enable = enable;
	strscpy(req.service_path, service_path, sizeof(req.service_path));
	ret = qmi_send_request(&pdr->notifier_hdl, addr, &txn,
			       SERVREG_REGISTER_LISTENER_REQ,
			       SERVREG_REGISTER_LISTENER_REQ_LEN,
			       servreg_register_listener_req_ei, &req);
	if (ret < 0) {
		mutex_lock(&pdr->list_lock);
		if (pds->register_txn == &txn)
			pds->register_txn = NULL;
		mutex_unlock(&pdr->list_lock);
		qmi_txn_cancel(&txn);
		return ret;
	}

	ret = qmi_txn_wait(&txn, 5 * HZ);
	mutex_lock(&pdr->list_lock);
	if (pds->register_txn == &txn)
		pds->register_txn = NULL;
	mutex_unlock(&pdr->list_lock);
	/* Server loss/release explicitly completed the transaction, not a timeout. */
	if (ret == -ENETRESET)
		return ret;
	if (ret < 0) {
		pr_err("PDR: %s register listener txn wait failed: %d\n",
		       service_path, ret);
		return ret;
	}
	if (resp.resp.result != QMI_RESULT_SUCCESS_V01) {
		pr_err("PDR: %s register listener failed: 0x%x\n",
		       service_path, resp.resp.error);
		return -EREMOTEIO;
	}
	if (!resp.curr_state_valid) {
		pr_err("PDR: %s register listener response has no state\n", service_path);
		return -EPROTO;
	}
	*state = resp.curr_state;
	return 0;
}

/* status_lock orders callbacks; list_lock is never held across client code. */
static bool pdr_notify_pending_down(struct pdr_handle *pdr, struct pdr_service *pds)
{
	bool notify;

	lockdep_assert_held(&pdr->status_lock);
	mutex_lock(&pdr->list_lock);
	notify = !pdr->stopping && !pds->lookup_failed && pds->need_notifier_remove;
	if (notify) {
		pds->need_notifier_remove = false;
		pds->state = SERVREG_SERVICE_STATE_DOWN;
	}
	mutex_unlock(&pdr->list_lock);
	if (notify)
		pdr->status(SERVREG_SERVICE_STATE_DOWN, pds->service_path, pdr->priv);
	return notify;
}

static void pdr_publish_registration(struct pdr_handle *pdr, struct pdr_service *pds,
				     u64 generation, u64 sequence, int state)
{
	bool notify;

	mutex_lock(&pdr->status_lock);
	/* A coalesced DEL/NEW must release the old client's resources first. */
	while (pdr_notify_pending_down(pdr, pds))
		;
	mutex_lock(&pdr->list_lock);
	notify = !pdr->stopping && !pds->lookup_failed && pds->service_connected &&
		 pds->server_generation == generation && pds->state_sequence == sequence;
	if (notify)
		pds->state = state;
	mutex_unlock(&pdr->list_lock);
	if (notify)
		pdr->status(state, pds->service_path, pdr->priv);
	mutex_unlock(&pdr->status_lock);
}

static void pdr_notifier_work(struct work_struct *work)
{
	struct pdr_handle *pdr = container_of(work, struct pdr_handle, notifier_work);
	struct pdr_service *pds, *iter;
	struct sockaddr_qrtr addr;
	u64 generation, sequence;
	bool remove;
	int ret, state;

	for (;;) {
		pds = NULL;
		mutex_lock(&pdr->status_lock);
		mutex_lock(&pdr->list_lock);
		if (!pdr->stopping) {
			list_for_each_entry(iter, &pdr->lookups, node) {
				if (iter->need_notifier_remove ||
				    (iter->service_connected && iter->need_notifier_register)) {
					pds = iter;
					break;
				}
			}
		}
		if (!pds) {
			mutex_unlock(&pdr->list_lock);
			mutex_unlock(&pdr->status_lock);
			return;
		}
		remove = pds->need_notifier_remove;
		if (remove) {
			pds->need_notifier_remove = false;
			pds->state = SERVREG_SERVICE_STATE_DOWN;
		} else {
			pds->need_notifier_register = false;
			addr = pds->addr;
			generation = pds->server_generation;
			sequence = pds->state_sequence;
		}
		mutex_unlock(&pdr->list_lock);
		if (remove)
			pdr->status(SERVREG_SERVICE_STATE_DOWN, pds->service_path, pdr->priv);
		mutex_unlock(&pdr->status_lock);
		if (remove)
			continue;

		/* RX control/indication callbacks must run while this transaction waits. */
		ret = pdr_register_listener(pdr, pds, &addr, generation, true, &state);
		if (ret < 0)
			state = SERVREG_SERVICE_STATE_DOWN;
		pdr_publish_registration(pdr, pds, generation, sequence, state);
	}
}

static int pdr_notifier_new_server(struct qmi_handle *qmi,
				   struct qmi_service *svc)
{
	struct pdr_handle *pdr = container_of(qmi, struct pdr_handle, notifier_hdl);
	struct pdr_service *pds;

	mutex_lock(&pdr->list_lock);
	if (pdr->stopping) {
		mutex_unlock(&pdr->list_lock);
		return -ESHUTDOWN;
	}
	list_for_each_entry(pds, &pdr->lookups, node) {
		if (pds->service != svc->service || pds->instance != svc->instance)
			continue;
		if (pds->service_connected && pds->addr.sq_node == svc->node &&
		    pds->addr.sq_port == svc->port)
			continue;
		if (pds->service_connected)
			pds->need_notifier_remove = true;
		pdr_cancel_registration(pdr, pds);
		pds->server_generation++;
		pds->state_sequence++;
		pds->service_connected = true;
		pds->need_notifier_register = true;
		pds->addr.sq_family = AF_QIPCRTR;
		pds->addr.sq_node = svc->node;
		pds->addr.sq_port = svc->port;
		queue_work(pdr->notifier_wq, &pdr->notifier_work);
		if (pds->need_notifier_remove)
			queue_work(pdr->indack_wq, &pdr->indack_work);
	}
	mutex_unlock(&pdr->list_lock);
	return 0;
}

static void pdr_notifier_del_server(struct qmi_handle *qmi,
				    struct qmi_service *svc)
{
	struct pdr_handle *pdr = container_of(qmi, struct pdr_handle, notifier_hdl);
	struct pdr_service *pds;

	mutex_lock(&pdr->list_lock);
	if (!pdr->stopping) {
		list_for_each_entry(pds, &pdr->lookups, node) {
			if (pds->service != svc->service || pds->instance != svc->instance ||
			    !pds->service_connected || pds->addr.sq_node != svc->node ||
			    pds->addr.sq_port != svc->port)
				continue;
			pdr_cancel_registration(pdr, pds);
			pds->server_generation++;
			pds->state_sequence++;
			pds->service_connected = false;
			pds->need_notifier_register = false;
			pds->need_notifier_remove = true;
			pds->addr.sq_node = 0;
			pds->addr.sq_port = 0;
			queue_work(pdr->notifier_wq, &pdr->notifier_work);
			queue_work(pdr->indack_wq, &pdr->indack_work);
		}
	}
	mutex_unlock(&pdr->list_lock);
}

static const struct qmi_ops pdr_notifier_ops = {
	.new_server = pdr_notifier_new_server,
	.del_server = pdr_notifier_del_server,
};

static int pdr_send_indack_msg(struct pdr_handle *pdr, struct pdr_service *pds,
			       struct sockaddr_qrtr *addr, u16 tid)
{
	struct servreg_set_ack_resp resp;
	struct servreg_set_ack_req req;
	struct qmi_txn txn;
	int ret;

	ret = qmi_txn_init(&pdr->notifier_hdl, &txn, servreg_set_ack_resp_ei,
			   &resp);
	if (ret < 0)
		return ret;

	req.transaction_id = tid;
	strscpy(req.service_path, pds->service_path, sizeof(req.service_path));

	ret = qmi_send_request(&pdr->notifier_hdl, addr,
			       &txn, SERVREG_SET_ACK_REQ,
			       SERVREG_SET_ACK_REQ_LEN,
			       servreg_set_ack_req_ei,
			       &req);

	/* Skip waiting for response */
	qmi_txn_cancel(&txn);
	return ret;
}

static void pdr_indack_work(struct work_struct *work)
{
	struct pdr_handle *pdr = container_of(work, struct pdr_handle, indack_work);
	struct pdr_list_node *ind;
	struct pdr_service *pds, *iter;
	bool notify;

	for (;;) {
		pds = NULL;
		mutex_lock(&pdr->status_lock);
		mutex_lock(&pdr->list_lock);
		if (pdr->stopping)
			goto done;
		/* Deliver server loss even while notifier_work is waiting for QMI. */
		list_for_each_entry(iter, &pdr->lookups, node) {
			if (iter->need_notifier_remove) {
				pds = iter;
				break;
			}
		}
		if (pds) {
			mutex_unlock(&pdr->list_lock);
			pdr_notify_pending_down(pdr, pds);
			mutex_unlock(&pdr->status_lock);
			continue;
		}
		if (list_empty(&pdr->indack_list))
			goto done;
		ind = list_first_entry(&pdr->indack_list, struct pdr_list_node, node);
		list_del(&ind->node);
		pds = ind->pds;
		notify = !pds->lookup_failed && pds->service_connected &&
			 pds->server_generation == ind->server_generation;
		if (notify)
			pds->state = ind->curr_state;
		mutex_unlock(&pdr->list_lock);
		if (notify)
			pdr->status(ind->curr_state, pds->service_path, pdr->priv);
		mutex_unlock(&pdr->status_lock);

		/* Ack only the original peer, after the client has released resources. */
		if (!READ_ONCE(pdr->stopping))
			pdr_send_indack_msg(pdr, pds, &ind->addr, ind->transaction_id);
		kfree(ind);
	}
done:
	mutex_unlock(&pdr->list_lock);
	mutex_unlock(&pdr->status_lock);
}

static void pdr_indication_cb(struct qmi_handle *qmi, struct sockaddr_qrtr *sq,
			      struct qmi_txn *txn, const void *data)
{
	struct pdr_handle *pdr = container_of(qmi, struct pdr_handle, notifier_hdl);
	const struct servreg_state_updated_ind *ind_msg = data;
	struct pdr_list_node *ind;
	struct pdr_service *pds;

	if (!sq || !ind_msg || !ind_msg->service_path[0] ||
	    strnlen(ind_msg->service_path, sizeof(ind_msg->service_path)) > SERVREG_NAME_LENGTH)
		return;
	ind = kzalloc_obj(*ind);
	if (!ind)
		return;

	mutex_lock(&pdr->list_lock);
	if (!pdr->stopping) {
		list_for_each_entry(pds, &pdr->lookups, node) {
			if (strcmp(pds->service_path, ind_msg->service_path) ||
			    !pds->service_connected || pds->addr.sq_node != sq->sq_node ||
			    pds->addr.sq_port != sq->sq_port)
				continue;
			pds->state_sequence++;
			ind->transaction_id = ind_msg->transaction_id;
			ind->curr_state = ind_msg->curr_state;
			ind->pds = pds;
			ind->addr = *sq;
			ind->server_generation = pds->server_generation;
			list_add_tail(&ind->node, &pdr->indack_list);
			queue_work(pdr->indack_wq, &pdr->indack_work);
			mutex_unlock(&pdr->list_lock);
			pr_info("PDR: Indication received from %s, state: 0x%x, trans-id: %d\n",
				ind_msg->service_path, ind_msg->curr_state,
				ind_msg->transaction_id);
			return;
		}
	}
	mutex_unlock(&pdr->list_lock);
	kfree(ind);
}

static const struct qmi_msg_handler qmi_indication_handler[] = {
	{
		.type = QMI_INDICATION,
		.msg_id = SERVREG_STATE_UPDATED_IND_ID,
		.ei = servreg_state_updated_ind_ei,
		.decoded_size = sizeof(struct servreg_state_updated_ind),
		.fn = pdr_indication_cb,
	},
	{}
};

static int pdr_get_domain_list(struct servreg_get_domain_list_req *req,
			       struct servreg_get_domain_list_resp *resp,
			       struct pdr_handle *pdr, struct sockaddr_qrtr *addr)
{
	struct qmi_txn txn;
	int ret;

	ret = qmi_txn_init(&pdr->locator_hdl, &txn,
			   servreg_get_domain_list_resp_ei, resp);
	if (ret < 0)
		return ret;

	ret = qmi_send_request(&pdr->locator_hdl,
			       addr,
			       &txn, SERVREG_GET_DOMAIN_LIST_REQ,
			       SERVREG_GET_DOMAIN_LIST_REQ_MAX_LEN,
			       servreg_get_domain_list_req_ei,
			       req);
	if (ret < 0) {
		qmi_txn_cancel(&txn);
		return ret;
	}

	ret = qmi_txn_wait(&txn, 5 * HZ);
	if (ret < 0) {
		pr_err("PDR: %s get domain list txn wait failed: %d\n",
		       req->service_name, ret);
		return ret;
	}

	if (resp->resp.result != QMI_RESULT_SUCCESS_V01) {
		pr_err("PDR: %s get domain list failed: 0x%x\n",
		       req->service_name, resp->resp.error);
		return -EREMOTEIO;
	}

	return 0;
}

static int pdr_locate_service(struct pdr_handle *pdr, struct pdr_service *pds,
			      struct sockaddr_qrtr *addr)
{
	struct servreg_get_domain_list_req req;
	struct servreg_location_entry *entry;
	int domains_read = 0;
	int ret, i;

	struct servreg_get_domain_list_resp *resp __free(kfree) = kzalloc_obj(*resp);
	if (!resp)
		return -ENOMEM;

	/* Prepare req message */
	strscpy(req.service_name, pds->service_name, sizeof(req.service_name));
	req.domain_offset_valid = true;
	req.domain_offset = 0;

	do {
		req.domain_offset = domains_read;
		ret = pdr_get_domain_list(&req, resp, pdr, addr);
		if (ret < 0)
			return ret;

		for (i = 0; i < resp->domain_list_len; i++) {
			entry = &resp->domain_list[i];

			if (strnlen(entry->name, sizeof(entry->name)) == sizeof(entry->name))
				continue;

			if (!strcmp(entry->name, pds->service_path)) {
				pds->service_data_valid = entry->service_data_valid;
				pds->service_data = entry->service_data;
				pds->instance = entry->instance;
				return 0;
			}
		}

		/* Update ret to indicate that the service is not yet found */
		ret = -ENXIO;

		/* Always read total_domains from the response msg */
		if (resp->domain_list_len > resp->total_domains)
			resp->domain_list_len = resp->total_domains;

		domains_read += resp->domain_list_len;
	} while (domains_read < resp->total_domains);

	return ret;
}

static void pdr_notify_lookup_failure(struct pdr_handle *pdr,
				      struct pdr_service *pds, int err)
{
	pr_err("PDR: service lookup for %s failed: %d\n", pds->service_name, err);
	if (err == -ENXIO)
		return;

	mutex_lock(&pdr->status_lock);
	mutex_lock(&pdr->list_lock);
	if (pdr->stopping || pds->lookup_failed) {
		mutex_unlock(&pdr->list_lock);
		mutex_unlock(&pdr->status_lock);
		return;
	}
	pdr_cancel_registration(pdr, pds);
	pds->lookup_failed = true;
	pds->need_locator_lookup = false;
	pds->need_notifier_register = false;
	pds->need_notifier_remove = false;
	pds->state = SERVREG_LOCATOR_ERR;
	/* Workers and queued indications retain this object until handle release. */
	list_move_tail(&pds->node, &pdr->retired);
	mutex_unlock(&pdr->list_lock);
	pdr->status(SERVREG_LOCATOR_ERR, pds->service_path, pdr->priv);
	mutex_unlock(&pdr->status_lock);
}

static void pdr_locator_work(struct work_struct *work)
{
	struct pdr_handle *pdr = container_of(work, struct pdr_handle, locator_work);
	struct pdr_service snapshot, *pds, *iter;
	struct sockaddr_qrtr addr;
	u64 generation, run;
	int ret;

	mutex_lock(&pdr->list_lock);
	run = ++pdr->locator_run;
	mutex_unlock(&pdr->list_lock);
	for (;;) {
		mutex_lock(&pdr->lock);
		if (!pdr->locator_init_complete || READ_ONCE(pdr->stopping)) {
			mutex_unlock(&pdr->lock);
			return;
		}
		addr = pdr->locator_addr;
		generation = pdr->locator_generation;
		mutex_unlock(&pdr->lock);

		pds = NULL;
		mutex_lock(&pdr->list_lock);
		if (!pdr->stopping) {
			list_for_each_entry(iter, &pdr->lookups, node) {
				if (iter->need_locator_lookup && iter->locator_attempt != run) {
					pds = iter;
					pds->locator_attempt = run;
					snapshot = *pds;
					break;
				}
			}
		}
		mutex_unlock(&pdr->list_lock);
		if (!pds)
			return;

		ret = pdr_locate_service(pdr, &snapshot, &addr);
		mutex_lock(&pdr->lock);
		if (!pdr->locator_init_complete || pdr->locator_generation != generation ||
		    READ_ONCE(pdr->stopping)) {
			mutex_unlock(&pdr->lock);
			continue;
		}
		if (!ret) {
			mutex_lock(&pdr->list_lock);
			pds->service_data_valid = snapshot.service_data_valid;
			pds->service_data = snapshot.service_data;
			pds->instance = snapshot.instance;
			pds->need_locator_lookup = false;
			mutex_unlock(&pdr->list_lock);
		}
		mutex_unlock(&pdr->lock);
		if (ret < 0) {
			pdr_notify_lookup_failure(pdr, pds, ret);
			continue;
		}
		ret = qmi_add_lookup(&pdr->notifier_hdl, pds->service, 1, snapshot.instance);
		if (ret < 0)
			pdr_notify_lookup_failure(pdr, pds, ret);
	}
}

/**
 * pdr_add_lookup() - register a tracking request for a PD
 * @pdr:		PDR client handle
 * @service_name:	service name of the tracking request
 * @service_path:	service path of the tracking request
 *
 * Registering a pdr lookup allows for tracking the life cycle of the PD.
 *
 * Return: pdr_service object on success, ERR_PTR on failure. -EALREADY is
 * returned if a lookup is already in progress for the given service path.
 */
struct pdr_service *pdr_add_lookup(struct pdr_handle *pdr,
				   const char *service_name,
				   const char *service_path)
{
	struct pdr_service *tmp;

	if (IS_ERR_OR_NULL(pdr))
		return ERR_PTR(-EINVAL);

	if (!service_name || strlen(service_name) > SERVREG_NAME_LENGTH ||
	    !service_path || strlen(service_path) > SERVREG_NAME_LENGTH)
		return ERR_PTR(-EINVAL);

	struct pdr_service *pds __free(kfree) = kzalloc_obj(*pds);
	if (!pds)
		return ERR_PTR(-ENOMEM);

	pds->service = QMI_SERVICE_ID_SERVREG_NOTIF;
	strscpy(pds->service_name, service_name, sizeof(pds->service_name));
	strscpy(pds->service_path, service_path, sizeof(pds->service_path));
	pds->need_locator_lookup = true;

	mutex_lock(&pdr->list_lock);
	if (pdr->stopping) {
		mutex_unlock(&pdr->list_lock);
		return ERR_PTR(-ESHUTDOWN);
	}
	list_for_each_entry(tmp, &pdr->lookups, node) {
		if (strcmp(tmp->service_path, service_path))
			continue;

		mutex_unlock(&pdr->list_lock);
		return ERR_PTR(-EALREADY);
	}

	list_add(&pds->node, &pdr->lookups);
	schedule_work(&pdr->locator_work);
	mutex_unlock(&pdr->list_lock);

	return_ptr(pds);
}
EXPORT_SYMBOL_GPL(pdr_add_lookup);

/**
 * pdr_restart_pd() - restart PD
 * @pdr:	PDR client handle
 * @pds:	PD service handle
 *
 * Restarts the PD tracked by the PDR client handle for a given service path.
 *
 * Return: 0 on success, negative errno on failure.
 */
int pdr_restart_pd(struct pdr_handle *pdr, struct pdr_service *pds)
{
	struct servreg_restart_pd_resp resp;
	struct servreg_restart_pd_req req = { 0 };
	struct sockaddr_qrtr addr;
	struct pdr_service *tmp;
	struct qmi_txn txn;
	int ret;

	if (IS_ERR_OR_NULL(pdr) || IS_ERR_OR_NULL(pds))
		return -EINVAL;

	mutex_lock(&pdr->list_lock);
	if (pdr->stopping) {
		mutex_unlock(&pdr->list_lock);
		return -ESHUTDOWN;
	}
	list_for_each_entry(tmp, &pdr->lookups, node) {
		if (tmp != pds)
			continue;

		if (!pds->service_connected)
			break;

		/* Prepare req message */
		strscpy(req.service_path, pds->service_path, sizeof(req.service_path));
		addr = pds->addr;
		break;
	}
	mutex_unlock(&pdr->list_lock);

	if (!req.service_path[0])
		return -EINVAL;

	ret = qmi_txn_init(&pdr->notifier_hdl, &txn,
			   servreg_restart_pd_resp_ei,
			   &resp);
	if (ret < 0)
		return ret;

	ret = qmi_send_request(&pdr->notifier_hdl, &addr,
			       &txn, SERVREG_RESTART_PD_REQ,
			       SERVREG_RESTART_PD_REQ_MAX_LEN,
			       servreg_restart_pd_req_ei, &req);
	if (ret < 0) {
		qmi_txn_cancel(&txn);
		return ret;
	}

	ret = qmi_txn_wait(&txn, 5 * HZ);
	if (ret < 0) {
		pr_err("PDR: %s PD restart txn wait failed: %d\n",
		       req.service_path, ret);
		return ret;
	}

	/* Check response if PDR is disabled */
	if (resp.resp.result == QMI_RESULT_FAILURE_V01 &&
	    resp.resp.error == QMI_ERR_DISABLED_V01) {
		pr_err("PDR: %s PD restart is disabled: 0x%x\n",
		       req.service_path, resp.resp.error);
		return -EOPNOTSUPP;
	}

	/* Check the response for other error case*/
	if (resp.resp.result != QMI_RESULT_SUCCESS_V01) {
		pr_err("PDR: %s request for PD restart failed: 0x%x\n",
		       req.service_path, resp.resp.error);
		return -EREMOTEIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pdr_restart_pd);

/**
 * pdr_handle_alloc() - initialize the PDR client handle
 * @status:	function to be called on PD state change
 * @priv:	handle for client's use
 *
 * Initializes the PDR client handle to allow for tracking/restart of PDs.
 *
 * Return: pdr_handle object on success, ERR_PTR on failure.
 */
struct pdr_handle *pdr_handle_alloc(void (*status)(int state,
						   char *service_path,
						   void *priv), void *priv)
{
	int ret;

	if (!status)
		return ERR_PTR(-EINVAL);

	struct pdr_handle *pdr __free(kfree) = kzalloc_obj(*pdr);
	if (!pdr)
		return ERR_PTR(-ENOMEM);

	pdr->status = status;
	pdr->priv = priv;

	mutex_init(&pdr->status_lock);
	mutex_init(&pdr->list_lock);
	mutex_init(&pdr->lock);

	INIT_LIST_HEAD(&pdr->lookups);
	INIT_LIST_HEAD(&pdr->retired);
	INIT_LIST_HEAD(&pdr->indack_list);

	INIT_WORK(&pdr->locator_work, pdr_locator_work);
	INIT_WORK(&pdr->notifier_work, pdr_notifier_work);
	INIT_WORK(&pdr->indack_work, pdr_indack_work);

	pdr->notifier_wq = create_singlethread_workqueue("pdr_notifier_wq");
	if (!pdr->notifier_wq)
		return ERR_PTR(-ENOMEM);

	pdr->indack_wq = alloc_ordered_workqueue("pdr_indack_wq", WQ_HIGHPRI);
	if (!pdr->indack_wq) {
		ret = -ENOMEM;
		goto destroy_notifier;
	}

	ret = qmi_handle_init(&pdr->locator_hdl,
			      SERVREG_GET_DOMAIN_LIST_RESP_MAX_LEN,
			      &pdr_locator_ops, NULL);
	if (ret < 0)
		goto destroy_indack;

	ret = qmi_add_lookup(&pdr->locator_hdl, QMI_SERVICE_ID_SERVREG_LOC, 1, 1);
	if (ret < 0)
		goto release_qmi_handle;

	ret = qmi_handle_init(&pdr->notifier_hdl,
			      SERVREG_STATE_UPDATED_IND_MAX_LEN,
			      &pdr_notifier_ops,
			      qmi_indication_handler);
	if (ret < 0)
		goto release_qmi_handle;

	return_ptr(pdr);

release_qmi_handle:
	WRITE_ONCE(pdr->stopping, true);
	mutex_lock(&pdr->lock);
	pdr->locator_init_complete = false;
	mutex_unlock(&pdr->lock);
	cancel_work_sync(&pdr->locator_work);
	qmi_handle_release(&pdr->locator_hdl);
destroy_indack:
	destroy_workqueue(pdr->indack_wq);
destroy_notifier:
	destroy_workqueue(pdr->notifier_wq);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(pdr_handle_alloc);

/**
 * pdr_handle_release() - release the PDR client handle
 * @pdr:	PDR client handle
 *
 * Cleans up pending tracking requests and releases the underlying qmi handles.
 */
void pdr_handle_release(struct pdr_handle *pdr)
{
	struct pdr_service *pds, *tmp;
	struct pdr_list_node *ind, *next;

	if (IS_ERR_OR_NULL(pdr))
		return;

	mutex_lock(&pdr->list_lock);
	WRITE_ONCE(pdr->stopping, true);
	list_for_each_entry(pds, &pdr->lookups, node)
		pdr_cancel_registration(pdr, pds);
	list_for_each_entry(pds, &pdr->retired, node)
		pdr_cancel_registration(pdr, pds);
	mutex_unlock(&pdr->list_lock);
	/* Pair with locator callbacks so no callback can queue after cancellation. */
	mutex_lock(&pdr->lock);
	pdr->locator_init_complete = false;
	pdr->locator_generation++;
	mutex_unlock(&pdr->lock);

	/* Keep QMI RX alive until every in-flight transaction has finished. */
	cancel_work_sync(&pdr->locator_work);
	cancel_work_sync(&pdr->notifier_work);
	cancel_work_sync(&pdr->indack_work);
	qmi_handle_release(&pdr->locator_hdl);
	qmi_handle_release(&pdr->notifier_hdl);
	destroy_workqueue(pdr->notifier_wq);
	destroy_workqueue(pdr->indack_wq);

	list_for_each_entry_safe(ind, next, &pdr->indack_list, node) {
		list_del(&ind->node);
		kfree(ind);
	}
	list_for_each_entry_safe(pds, tmp, &pdr->lookups, node) {
		list_del(&pds->node);
		kfree(pds);
	}
	list_for_each_entry_safe(pds, tmp, &pdr->retired, node) {
		list_del(&pds->node);
		kfree(pds);
	}
	kfree(pdr);
}
EXPORT_SYMBOL_GPL(pdr_handle_release);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm Protection Domain Restart helpers");
