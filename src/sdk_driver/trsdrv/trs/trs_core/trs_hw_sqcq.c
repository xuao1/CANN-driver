/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2025-2025. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <uapi/linux/sched/types.h>
#include "ka_task_pub.h"
#include "ka_barrier_pub.h"
#include "ka_kernel_def_pub.h"
#include "ka_common_pub.h"
#include "ka_system_pub.h"
#include "ka_errno_pub.h"

#include "securec.h"

#include "pbl_uda.h"
#include "trs_mailbox_def.h"
#include "trs_chan.h"
#include "trs_ts_inst.h"
#include "trs_sqcq_map.h"
#include "trs_proc_fs.h"
#include "trs_res_mng.h"
#include "trs_shr_sqcq.h"
#include "trs_hw_sqcq.h"

static bool trs_is_sqcq_pair_mode(struct trs_core_ts_inst *ts_inst)
{
    return trs_is_stars_inst(ts_inst);
}

static inline u32 trs_hw_get_sq_head(struct trs_sq_ctx *sq_ctx)
{
    return *(u32 *)sq_ctx->head.kva;
}

static inline void trs_hw_set_sq_head(struct trs_sq_ctx *sq_ctx, u32 head)
{
     ka_smp_mb();

    *(u32 *)sq_ctx->head.kva = head;
}

static inline u32 trs_hw_get_sq_tail(struct trs_sq_ctx *sq_ctx)
{
    return *(u32 *)sq_ctx->tail.kva;
}

static void trs_trace_hw_cqe_fill(struct trs_id_inst *inst, struct trs_chan_cq_trace *cq_trace, void *cqe)
{
    struct trs_core_ts_inst *ts_inst = trs_core_ts_inst_get(inst);
    if (ts_inst != NULL) {
        if (ts_inst->ops.trace_cqe_fill != NULL) {
            ts_inst->ops.trace_cqe_fill(inst, cq_trace, cqe);
        }
        trs_core_ts_inst_put(ts_inst);
    }
}

static void trs_trace_hw_sqe_fill(struct trs_id_inst *inst, struct trs_chan_sq_trace *sq_trace, void *sqe)
{
    struct trs_core_ts_inst *ts_inst = trs_core_ts_inst_get(inst);
    if (ts_inst != NULL) {
        if (ts_inst->ops.trace_sqe_fill != NULL) {
            ts_inst->ops.trace_sqe_fill(inst, sq_trace, sqe);
        }
        trs_core_ts_inst_put(ts_inst);
    }
}

int trs_hw_sq_send_task(struct trs_sq_ctx *sq_ctx, enum trs_sq_send_sched_type sched_mode)
{
    struct trs_chan_send_para para = {.timeout = 1};
    struct trs_id_inst *inst = &sq_ctx->inst;
    unsigned long cur_jiffies = ka_jiffies;
    u32 head, tail, num = 0;

    // pr_info("trs_hw_sq_send_task: sq_ctx=%pK, sched_mode=%d\n", sq_ctx, sched_mode);
    // pr_info("trs_hw_sq_send_task: status=%d, head.kva=%pK, tail.kva=%pK, sq_depth=%u, sqe_size=%u, chan_id=%d\n",
    //     sq_ctx->status, sq_ctx->head.kva, sq_ctx->tail.kva, sq_ctx->sq_depth, sq_ctx->sqe_size, sq_ctx->chan_id);

    if ((sq_ctx->status == 0) || (sq_ctx->head.kva == NULL)) {
        return 0;
    }

    if (ka_task_mutex_trylock(&sq_ctx->mutex) == 0) {
        return 0;
    }

    if ((sq_ctx->status == 0) || (sq_ctx->head.kva == NULL)) {
        ka_task_mutex_unlock(&sq_ctx->mutex);
        return 0;
    }

    while ((head = trs_hw_get_sq_head(sq_ctx)) != (tail = trs_hw_get_sq_tail(sq_ctx))) {
        int ret;

        if ((head >= sq_ctx->sq_depth) || (tail >= sq_ctx->sq_depth)) {
            trs_err("Invalid head tail. (devid=%u; tsid=%u; sqId=%u; sq_depth=%u; head=%u; tail=%u)\n",
                inst->devid, inst->tsid, sq_ctx->sqid, sq_ctx->sq_depth, head, tail);
            break;
        }

        para.sqe = (u8 *)sq_ctx->que_mem.kva + head * sq_ctx->sqe_size;
        para.sqe_num = 1; /* make sure stars run and task submit at the same time */
        // pr_info("sq_send: devid=%u tsid=%u sqId=%u head=%u tail=%u sqe=[%*ph]\n",
        //     inst->devid, inst->tsid, sq_ctx->sqid, head, tail,
        //     (int)min_t(u32, sq_ctx->sqe_size, 64), para.sqe);
        ret = hal_kernel_trs_chan_send(inst, sq_ctx->chan_id, &para);
        if (ret != 0) {
            sq_ctx->send_fail++;
            trs_debug("Send failed. (devid=%u; tsid=%u; sqId=%u; sq_depth=%u; head=%u; tail=%u; ret=%d)\n",
                inst->devid, inst->tsid, sq_ctx->sqid, sq_ctx->sq_depth, head, tail, ret);
            break;
        }

        trs_hw_set_sq_head(sq_ctx, (head + para.sqe_num) % sq_ctx->sq_depth);
        num += para.sqe_num;
        trs_try_resched(&cur_jiffies, 4800); /* timeout is 4800 ms */
        if ((sched_mode == SQ_SEND_FAIR_MODE) && (num >= sq_ctx->sq_fair_send_threshold)) {
            break;
        }
    }

    ka_task_mutex_unlock(&sq_ctx->mutex);

    if (num > 1) {
        trs_debug("Tx burst. (devid=%u; tsid=%u; sqId=%u; num=%u; tail=%u)\n",
            inst->devid, inst->tsid, sq_ctx->sqid, num, tail);
    }
    return num;
}

static int trs_sq_send_proc(struct trs_core_ts_inst *ts_inst, enum trs_sq_send_sched_type sched_mode)
{
    struct trs_res_mng *res_mng = &ts_inst->res_mng[TRS_HW_SQ];
    u32 i, sq_max_id = trs_res_get_max_id(ts_inst, TRS_HW_SQ);
    unsigned long cur_jiffies = ka_jiffies;
    int num = 0;

    /* fast send */
    if ((sched_mode == SQ_SEND_NON_FAIR_MODE) && (ts_inst->ops.get_trigger_sqid) != NULL) {
        u32 sqid;
        int ret = ts_inst->ops.get_trigger_sqid(&ts_inst->inst, &sqid);
        if ((ret == 0) && (sqid < sq_max_id)) {
            if (res_mng->ids[sqid].ref > 0) {
                num += trs_hw_sq_send_task(&ts_inst->sq_ctx[sqid], sched_mode);
            }
        }
    }

    /* check all sq */
    for (i = 0; i < sq_max_id; i++) {
        if (res_mng->ids[i].ref > 0) {
            num += trs_hw_sq_send_task(&ts_inst->sq_ctx[i], sched_mode);
        }
        trs_try_resched(&cur_jiffies, 4800); /* timeout is 4800 ms */
    }

    return num;
}

static void trs_set_thread_priority(struct trs_id_inst *inst)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0)
    ka_sched_param_t sp;

    sp.sched_priority = 1;
    if (ka_task_sched_setscheduler(ka_task_get_current(), SCHED_FIFO, &sp) != 0) {
        trs_err("Set priority fail. (devid=%u; tsid=%u)\n", inst->devid, inst->tsid);
    }
#else
    ka_task_sched_set_fifo_low(ka_task_get_current());
#endif
}

static int trs_sq_send_thread(void *arg)
{
    struct trs_core_ts_inst *ts_inst = (struct trs_core_ts_inst *)arg;
    u32 sq_task_sched_time = 4000; /* 4000 us */
    ts_inst->sq_task_flag = 0;

    trs_set_thread_priority(&ts_inst->inst);

    while (!ka_task_kthread_should_stop()) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
        ka_system_usleep_range(sq_task_sched_time - 100, sq_task_sched_time); /* 100 us */
#else
        usleep_range_state(sq_task_sched_time - 100, sq_task_sched_time, TASK_INTERRUPTIBLE); /* 100 us */
#endif

        if (ts_inst->sq_task_flag == 0) {
            continue;
        }

        if (trs_sq_send_proc(ts_inst, SQ_SEND_NON_FAIR_MODE) == 0) {
            sq_task_sched_time = 4000; /* 4000 us */
        } else {
            sq_task_sched_time = 1000; /* 1000 us */
        }
    }

    return 0;
}

static bool trs_sq_work_is_need_retry(struct trs_core_ts_inst *ts_inst, u32 num)
{
    if (work_pending(&ts_inst->sq_trigger_work)) {
        return false;
    }

#ifndef EMU_ST
    if (num != 0) {
        ts_inst->sq_work_retry_time = 3;    /* 3:retry time */
        return true;
    }

    if ((num == 0) && (ts_inst->sq_work_retry_time > 0)) {
        ts_inst->sq_work_retry_time--;
        return true;
    }

    return false;
#endif
}

static void trs_sq_trigger_work(struct work_struct *p_work)
{
    struct trs_core_ts_inst *ts_inst = ka_container_of(p_work, struct trs_core_ts_inst, sq_trigger_work);
    u32 num;

    num = (u32)trs_sq_send_proc(ts_inst, SQ_SEND_NON_FAIR_MODE);
    if (trs_sq_work_is_need_retry(ts_inst, num)) {
#ifndef EMU_ST
        (void)ka_task_queue_work(ts_inst->work_queue, &ts_inst->sq_trigger_work);
#endif
    }
}

static void trs_sq_send_fair_work(struct work_struct *p_work)
{
    struct trs_core_ts_inst *ts_inst = ka_container_of(p_work, struct trs_core_ts_inst, sq_fair_work);

    if (trs_sq_send_proc(ts_inst, SQ_SEND_FAIR_MODE) != 0) {
        (void)ka_task_queue_work(ts_inst->fair_work_queue, &ts_inst->sq_fair_work);
    }
}

static ka_irqreturn_t trs_sq_trigger_irq_proc(int irq, void *para)
{
    struct trs_core_ts_inst *ts_inst = (struct trs_core_ts_inst *)para;

    ts_inst->sq_task_flag = 1;

    (void)ka_task_queue_work(ts_inst->work_queue, &ts_inst->sq_trigger_work);

    if (ts_inst->location != UDA_LOCAL) {
        (void)ka_task_queue_work(ts_inst->fair_work_queue, &ts_inst->sq_fair_work);
    }

    return IRQ_HANDLED;
}

static void trs_sq_set_cqe_status(struct trs_core_ts_inst *ts_inst, u32 sqid)
{
    struct trs_sq_ctx *sq_ctx = &ts_inst->sq_ctx[sqid];
    struct trs_sq_shr_info *shr_info = NULL;

    ka_task_spin_lock_bh(&sq_ctx->shr_info_lock);
    shr_info = (struct trs_sq_shr_info *)sq_ctx->shr_info.kva;
    if (shr_info != NULL) {
        shr_info->cqe_status = 1;
    }
    ka_task_spin_unlock_bh(&sq_ctx->shr_info_lock);

    sq_ctx->cqe_status = 1;
}

static void trs_sq_get_and_clr_cqe_status(struct trs_core_ts_inst *ts_inst, u32 sqid, u32 *cqe_status)
{
    struct trs_sq_ctx *sq_ctx = &ts_inst->sq_ctx[sqid];

    *cqe_status = sq_ctx->cqe_status;
    if (*cqe_status == 1) {
        sq_ctx->cqe_status = 0;
    }
}

static int trs_hw_get_sqid_head_form_cqe(struct trs_core_ts_inst *ts_inst, void *cqe, u32 *sqid, u32 *head)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    int ret;

    ret = ts_inst->ops.get_sq_id_head_from_hw_cqe(inst, cqe, sqid, head);
    if (ret != 0) {
        trs_err("Get sq id head failed. (devid=%u; tsid=%u; ret=%d)\n", inst->devid, inst->tsid, ret);
        return ret;
    }

    if (*sqid >= trs_res_get_max_id(ts_inst, TRS_HW_SQ)) {
        trs_err("Invalid sqid. (devid=%u; tsid=%u; sqid=%u)\n", inst->devid, inst->tsid, *sqid);
        return -EINVAL;
    }

    return 0;
}

static int trs_hw_get_logic_cq_from_cqe(struct trs_core_ts_inst *ts_inst, void *cqe)
{
    u32 streamid;
    int ret;

    if (ts_inst->ops.get_stream_id_from_hw_cqe == NULL) {
        return -1;
    }

    ret = ts_inst->ops.get_stream_id_from_hw_cqe(&ts_inst->inst, cqe, &streamid);
    if (ret != 0) {
        trs_err("Get stream id. (devid=%u; tsid=%u; ret=%d)\n", ts_inst->inst.devid, ts_inst->inst.tsid, ret);
        return -1;
    }

    return trs_get_stream_logic_cq(ts_inst, streamid);
}

static int trs_hw_cq_recv_proc(struct trs_core_ts_inst *ts_inst, u32 cqid, u32 logic_cq, void *cqe)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    struct trs_logic_cqe logic_cqe;
    int ret;

    if (ts_inst->ops.hw_cqe_to_logic_cqe == NULL) {
        return -EINVAL;
    }

    ret = ts_inst->ops.hw_cqe_to_logic_cqe(inst, cqe, (void *)&logic_cqe);
    if (ret != 0) {
        trs_err("trans failed. (devid=%u; tsid=%u; logic_cq=%u)\n", inst->devid, inst->tsid, logic_cq);
        return ret;
    }

    if (ts_inst->ops.is_drop_cqe != NULL) {
        if (ts_inst->ops.is_drop_cqe(inst, &logic_cqe)) {
            ts_inst->cq_ctx[cqid].stat.rx_drop++;
            return -EINVAL;
        }
    }

    ret = trs_logic_cq_enque(ts_inst, logic_cq, logic_cqe.stream_id, logic_cqe.task_id, (void *)&logic_cqe);
    if (ret != 0) {
        ts_inst->cq_ctx[cqid].stat.rx_enque_fail++;
        return ret;
    }
    ts_inst->cq_ctx[cqid].stat.rx_enque++;
    return 0;
}

static int trs_hw_cq_recv(struct trs_id_inst *inst, u32 cqid, void *cqe)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    u32 sqid, sq_head;
    int recv_result = CQ_RECV_FINISH;
    int logic_cqid, ret = 0;

    ts_inst = trs_core_ts_inst_get(inst);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u; cqid=%u)\n", inst->devid, inst->tsid, cqid);
        return CQ_RECV_FINISH;
    }

    ts_inst->cq_ctx[cqid].stat.rx++;
    logic_cqid = trs_hw_get_logic_cq_from_cqe(ts_inst, cqe);
    if (logic_cqid >= 0) {
        ret = trs_hw_cq_recv_proc(ts_inst, cqid, (u32)logic_cqid, cqe);
    } else {
        recv_result = CQ_RECV_CONTINUE;
    }

    if (trs_hw_get_sqid_head_form_cqe(ts_inst, cqe, &sqid, &sq_head) == 0) {
        if (ret == 0) {
            trs_sq_set_cqe_status(ts_inst, sqid);
        }

        if (!trs_is_sqcq_pair_mode(ts_inst)) {
            (void)trs_chan_ctrl(inst, ts_inst->sq_ctx[sqid].chan_id, CHAN_CTRL_CMD_SQ_HEAD_UPDATE, sq_head);
        }
    }

    trs_core_ts_inst_put(ts_inst);

    return recv_result;
}

static int trs_chan_cq_full_proc(struct trs_id_inst *inst, int chan_id)
{
    u8 cqe[128];    /* max cqe size is 128B */
    struct trs_chan_recv_para recv_para = {
        .cqe = cqe,
        .cqe_num = 1,
        .recv_cqe_num = 0,
        .timeout = 1000
    };
    int ret;

    ret = trs_chan_ctrl(inst, chan_id, CHAN_CTRL_CMD_SQ_STATUS_SET, 0);
    if (ret != 0) {
        trs_err("Set sq status failed. (devid=%u; tsid=%u; chan_id=%d; ret=%d)\n",
            inst->devid, inst->tsid, chan_id, ret);
        return ret;
    }

    ret = hal_kernel_trs_chan_recv(inst, chan_id, &recv_para);
    if (ret != 0) {
        trs_err("Fetch cq and update cq head failed. (devid=%u; tsid=%u; chan_id=%d; ret=%d)\n",
            inst->devid, inst->tsid, chan_id, ret);
        return ret;
    }

    return 0;
}

typedef int (*trs_hw_sqcq_abnormal_handle)(struct trs_id_inst *inst, int chan_id);
static const trs_hw_sqcq_abnormal_handle hw_sqcq_abnormal_handle[ABNORMAL_ERR_TYPE_MAX] = {
    [ABNORMAL_ERR_TYPE_CQ_FULL] = trs_chan_cq_full_proc
};

static int trs_hw_sqcq_abnormal_proc(struct trs_id_inst *inst, int chan_id, u8 err_type)
{
    if (err_type >= ABNORMAL_ERR_TYPE_MAX) {
        return -EINVAL;
    }

    if (hw_sqcq_abnormal_handle[err_type] != NULL) {
        return hw_sqcq_abnormal_handle[err_type](inst, chan_id);
    }

    return 0;
}

static int trs_hw_alloc_chan(struct trs_proc_ctx *proc_ctx, struct trs_id_inst *inst,
    struct halSqCqInputInfo *para, int *chan_id)
{
    struct trs_chan_para chan_para = {0};
    int ret;

    trs_debug("Alloc chan. (devid=%u; tsid=%u; flag=0x%x; sqeSize=%u; sqeDepth=%u; cqeSize=%u; cqeDepth=%u; "
        "info[0]=%u)\n", inst->devid, inst->tsid, para->flag, para->sqeSize, para->sqeDepth, para->cqeSize,
        para->cqeDepth, para->info[0]);
    chan_para.types.type = CHAN_TYPE_HW;
    if ((para->flag & TSDRV_FLAG_ONLY_SQCQ_ID) != 0) {
        chan_para.types.sub_type = CHAN_SUB_TYPE_HW_TS;
    } else if ((para->ext_info != 0) && (para->ext_info_len > 0)) {
        chan_para.types.sub_type = ((struct trs_ext_info_header *)(para->ext_info))->type;
        if (chan_para.types.sub_type >= CHAN_SUB_TYPE_MAX) {
            trs_err("Invalid sub_type. (devid=%u; sub_type=%u)\n", inst->devid, chan_para.types.sub_type);
            return -EINVAL;
        }
    } else {
        chan_para.types.sub_type = CHAN_SUB_TYPE_HW_RTS;
    }

    chan_para.ssid = proc_ctx->cp_ssid;
    chan_para.flag = 0;

    if (((para->flag & TSDRV_FLAG_REUSE_SQ) == 0) || ((para->flag & TSDRV_FLAG_SPECIFIED_SQ_ID) != 0)) {
        chan_para.sq_para.sqe_size = para->sqeSize;
        chan_para.sq_para.sq_depth = para->sqeDepth;
        chan_para.ops.trace_sqe_fill = trs_trace_hw_sqe_fill;
        chan_para.flag |= (0x1 << CHAN_FLAG_ALLOC_SQ_BIT);
        if ((para->flag & TSDRV_FLAG_SPECIFIED_SQ_ID) != 0) {
            chan_para.sqid = para->sqId;
            chan_para.flag |= (0x1 << CHAN_FLAG_SPECIFIED_SQ_ID_BIT);
        }
    }

    if (((para->flag & TSDRV_FLAG_REUSE_CQ) == 0) || ((para->flag & TSDRV_FLAG_SPECIFIED_CQ_ID) != 0)) {
        chan_para.cq_para.cqe_size = para->cqeSize;
        chan_para.cq_para.cq_depth = para->cqeDepth;
        chan_para.ops.cqe_is_valid = NULL;
        chan_para.ops.trace_cqe_fill = trs_trace_hw_cqe_fill;
        chan_para.ops.get_sq_head_in_cqe = NULL;
        chan_para.ops.cq_recv = trs_hw_cq_recv;
        chan_para.ops.abnormal_proc = trs_hw_sqcq_abnormal_proc;
        chan_para.flag |= (0x1 << CHAN_FLAG_ALLOC_CQ_BIT);
        if ((para->flag & TSDRV_FLAG_SPECIFIED_CQ_ID) != 0) {
            chan_para.cqid = para->cqId;
            chan_para.flag |= (0x1 << CHAN_FLAG_SPECIFIED_CQ_ID_BIT);
        }
    }

    if ((para->flag & TSDRV_FLAG_RANGE_ID) != 0) {
#ifndef EMU_ST
        chan_para.flag |= (0x1 << CHAN_FLAG_RANGE_SQ_ID_BIT) | (0x1 << CHAN_FLAG_RANGE_CQ_ID_BIT) |
            (0x1 << CHAN_FLAG_SPECIFIED_SQ_ID_BIT) | (0x1 << CHAN_FLAG_SPECIFIED_CQ_ID_BIT);
#endif
    }

    if (((chan_para.flag & (0x1 << CHAN_FLAG_ALLOC_SQ_BIT)) != 0) &&
        ((chan_para.flag & (0x1 << CHAN_FLAG_ALLOC_CQ_BIT)) != 0)) {
        chan_para.flag |= (0x1 << CHAN_FLAG_NOTICE_TS_BIT) | (0x1 << CHAN_FLAG_AUTO_UPDATE_SQ_HEAD_BIT);
    }

    if ((para->flag & TSDRV_FLAG_RTS_RSV_SQCQ_ID) != 0) {
        chan_para.flag |= 0x1 << CHAN_FLAG_RTS_RSV_SQ_ID_BIT;
        chan_para.flag |= 0x1 << CHAN_FLAG_RTS_RSV_CQ_ID_BIT;
    }

    if ((para->flag & TSDRV_FLAG_ONLY_SQCQ_ID) != 0) {
        chan_para.flag |= (0x1 << CHAN_FLAG_NO_SQ_MEM_BIT);
        chan_para.flag |= (0x1 << CHAN_FLAG_NO_CQ_MEM_BIT);
    }

    if ((para->flag & TSDRV_FLAG_NO_SQ_MEM) != 0) {
        chan_para.flag |= (0x1 << CHAN_FLAG_NO_SQ_MEM_BIT);
    }

    if ((para->flag & TSDRV_FLAG_NO_CQ_MEM) != 0) {
        chan_para.flag |= (0x1 << CHAN_FLAG_NO_CQ_MEM_BIT);
    }

    if ((para->flag & TSDRV_FLAG_SPECIFIED_SQ_MEM) != 0) {
        struct trs_alloc_para *alloc_para = get_alloc_para_addr(para);
        struct trs_uio_info *uio_info = alloc_para->uio_info;
        chan_para.sq_para.sq_que_uva = (void *)(uintptr_t)(uio_info->sq_que_addr);
        chan_para.flag |= (0x1U << CHAN_FLAG_SPECIFIED_SQ_MEM_BIT);
    }

    ret = memcpy_s(chan_para.msg, sizeof(chan_para.msg), para->info, sizeof(para->info));
    if (ret != 0) {
        trs_err("Memcopy failed. (dest_len=%lx; src_len=%lx)\n", sizeof(chan_para.msg), sizeof(para->info));
        return ret;
    }
    chan_para.ext_msg = para->ext_info;
    chan_para.ext_msg_len = para->ext_info_len;

    if (((para->flag & TSDRV_FLAG_REMOTE_ID) != 0) || ((para->flag & TSDRV_FLAG_AGENT_ID) != 0)) {
        int master_pid;
        if ((para->flag & TSDRV_FLAG_REMOTE_ID) != 0) {
            chan_para.flag |= (0x1 << CHAN_FLAG_REMOTE_ID_BIT);
            chan_para.flag |= (0x1 << CHAN_FLAG_USE_MASTER_PID_BIT);
        } else {
            chan_para.flag |= (0x1 << CHAN_FLAG_AGENT_ID_BIT);
        }
        ret = trs_core_get_host_pid(ka_task_get_current_tgid(), &master_pid);
        if (ret != 0) {
            return ret;
        }
        chan_para.msg[SQCQ_INFO_LENGTH - 1] = (u32)master_pid;
    }

    return hal_kernel_trs_chan_create(inst, &chan_para, chan_id);
}

static int trs_pair_mode_alloc_para_check(struct trs_id_inst *inst, struct halSqCqInputInfo *para)
{
    if (((para->flag & TSDRV_FLAG_REUSE_SQ) != 0) || ((para->flag & TSDRV_FLAG_REUSE_CQ) != 0)) {
        trs_err("Not support reuse, pair. (devid=%u; tsid=%u; flag=0x%x)\n", inst->devid, inst->tsid, para->flag);
        return -EINVAL;
    }

    if (para->sqeSize != 64) {  /* rtsq sqe size must be 64 */
        trs_err("Invalid sqe size. (devid=%u; tsid=%u; sqeSize=%u)\n",
            inst->devid, inst->tsid, para->sqeSize);
        return -EINVAL;
    }

    return 0;
}

static int trs_hw_sqcq_alloc_pair(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqInputInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    struct trs_sqcq_reg_map_para reg_map_para;
    struct trs_chan_sq_info sq_info;
    struct trs_chan_cq_info cq_info;
    u32 stream_id = para->info[0];
    int chan_id, ret;

    ret = trs_pair_mode_alloc_para_check(inst, para);
    if (ret != 0) {
        return ret;
    }

    if (trs_is_proc_res_limited(proc_ctx, ts_inst, TRS_HW_SQ)) {
        return -ENOSPC;
    }

    ret = trs_hw_alloc_chan(proc_ctx, &ts_inst->inst, para, &chan_id);
    if (ret != 0) {
        if (trs_get_proc_id_dfx_times(proc_ctx, inst->tsid, TRS_HW_SQ) <= TRS_PROC_DFX_TIMES_MAX) {
            trs_warn("Alloc chan failed. (devid=%u; tsid=%u; type=%d; cur_sq_num=%u; cur_cq_num=%u; ret=%d)\n",
                inst->devid, inst->tsid, para->type, trs_get_proc_res_num(proc_ctx, inst->tsid, TRS_HW_SQ),
                trs_get_proc_res_num(proc_ctx, inst->tsid, TRS_HW_CQ), ret);
        }
        return ret;
    }

    (void)trs_chan_get_sq_info(inst, chan_id, &sq_info);
    (void)trs_chan_get_cq_info(inst, chan_id, &cq_info);

    ret = trs_proc_add_res_ex(proc_ctx, ts_inst, TRS_HW_SQ, sq_info.sqid, ((para->flag & TSDRV_FLAG_AGENT_ID) != 0));
    if (ret != 0) {
        goto destroy_chan;
    }

    ret = trs_proc_add_res_ex(proc_ctx, ts_inst, TRS_HW_CQ, cq_info.cqid, ((para->flag & TSDRV_FLAG_AGENT_ID) != 0));
    if (ret != 0) {
        goto del_sq_res;
    }

    para->sqId = sq_info.sqid;
    para->cqId = cq_info.cqid;

    trs_sq_ctx_init(inst, &ts_inst->sq_ctx[sq_info.sqid], para, stream_id, chan_id);
    trs_cq_ctx_init(&ts_inst->cq_ctx[cq_info.cqid], U32_MAX, chan_id);

    reg_map_para.stream_id = stream_id;
    reg_map_para.sqid = sq_info.sqid;
    reg_map_para.cqid = cq_info.cqid;
    reg_map_para.host_pid = proc_ctx->pid;
    trs_sqcq_reg_map(ts_inst, &reg_map_para);
    /* If the map fails, the UIO is not used. Therefore, the return value of the map is ignored. */
    (void)trs_sq_remap(proc_ctx, ts_inst, para, &ts_inst->sq_ctx[sq_info.sqid], &sq_info);
    trs_set_sq_status(&ts_inst->sq_ctx[sq_info.sqid], 1);

    trs_debug("Alloc pair sqcq. (devid=%u; tsid=%u; flag=0x%x; sqId=%u; cqId=%u)\n",
        inst->devid, inst->tsid, para->flag, para->sqId, para->cqId);
    return 0;

del_sq_res:
    (void)trs_proc_del_res(proc_ctx, ts_inst, TRS_HW_SQ, sq_info.sqid);

destroy_chan:
    hal_kernel_trs_chan_destroy(inst, chan_id);
    trs_err("Sqcq Alloc failed. (devid=%u; tsid=%u)\n", inst->devid, inst->tsid);
    return ret;
}

int trs_get_stream_with_sq(struct trs_id_inst *inst, u32 sqid, u32 *stream_id)
{
    struct trs_core_ts_inst *ts_inst = trs_core_ts_inst_get(inst);

    if (ts_inst == NULL) {
        trs_err("Not init. (devid=%u; sqid=%u)\n", inst->devid, sqid);
        return -EINVAL;
    }

    *stream_id = ts_inst->sq_ctx[sqid].stream_id;

    trs_core_ts_inst_put(ts_inst);
    return 0;
}
KA_EXPORT_SYMBOL_GPL(trs_get_stream_with_sq);

static int pair_mode_free_para_check(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqFreeInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    u32 stream_id;

    if ((para->flag & TSDRV_FLAG_REUSE_CQ) != 0) {
        trs_err("Not support only free sq. (devid=%u; tsid=%u; flag=%x)\n", inst->devid, inst->tsid, para->flag);
        return -EINVAL;
    }

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, para->sqId)) {
        trs_err("Not proc sq. (devid=%u; tsid=%u; sq=%u; cq=%u)\n", inst->devid, inst->tsid, para->sqId, para->cqId);
        return -EINVAL;
    }

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_CQ, para->cqId)) {
        trs_err("Not proc cq. (devid=%u; tsid=%u; sq=%u; cq=%u)\n", inst->devid, inst->tsid, para->sqId, para->cqId);
        return -EINVAL;
    }

    if (ts_inst->sq_ctx[para->sqId].chan_id != ts_inst->cq_ctx[para->cqId].chan_id) {
        trs_err("Not pair sqcq. (devid=%u; tsid=%u; sqId=%u; cqId=%u)\n",
            inst->devid, inst->tsid, para->sqId, para->cqId);
        return -EINVAL;
    }

    stream_id = ts_inst->sq_ctx[para->sqId].stream_id;
    if ((stream_id != KA_U32_MAX) && (ts_inst->stream_ctx[stream_id].stream_base_addr != 0)) {
        trs_err("Not unbind. (devid=%u; sqid=%u; stream_id=%u)\n", inst->devid, para->sqId, stream_id);
        return -EINVAL;
    }

    return 0;
}

static void _trs_hw_sqcq_free_pair(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    u32 flag, u32 sqId, u32 cqId)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    int chan_id;

    trs_debug("Free pair sqcq. (devid=%u; tsid=%u; flag=0x%x; sqId=%u; cqId=%u)\n",
        inst->devid, inst->tsid, flag, sqId, cqId);

    chan_id = ts_inst->sq_ctx[sqId].chan_id;

    trs_sq_ctx_uninit(&ts_inst->sq_ctx[sqId]);
    trs_cq_ctx_uninit(&ts_inst->cq_ctx[cqId]);
    if (trs_cq_is_need_show(&ts_inst->cq_ctx[cqId])) {
        trs_hw_cq_show(ts_inst, cqId);
    }
    (void)trs_proc_del_res(proc_ctx, ts_inst, TRS_HW_SQ, sqId);
    (void)trs_proc_del_res(proc_ctx, ts_inst, TRS_HW_CQ, cqId);
    hal_kernel_trs_chan_destroy_ex(inst, flag, chan_id);
}

static void trs_fill_notice_ts_msg(struct trs_proc_ctx *proc_ctx, u16 cmd_type,
    struct trs_chan_sq_info *sq_info, struct trs_chan_cq_info *cq_info, struct trs_normal_cqsq_mailbox *msg)
{
    trs_mbox_init_header(&msg->header, cmd_type);

    msg->sq_index = sq_info->sqid;
    msg->sqesize = sq_info->sq_para.sqe_size;
    msg->sqdepth = sq_info->sq_para.sq_depth;
    msg->sq_addr = sq_info->sq_phy_addr;

    msg->cq0_index = cq_info->cqid;
    msg->cqesize = cq_info->cq_para.cqe_size;
    msg->cqdepth = cq_info->cq_para.cq_depth;
    msg->cq0_addr = cq_info->cq_phy_addr;

    msg->pid = proc_ctx->pid;
    msg->cq_irq = (u16)cq_info->irq;
    msg->ssid = proc_ctx->cp_ssid;

    /* adapt fill: app_type, fid, sq_cq_side */
}

static int trs_hw_sqcq_status_query_notice_ts(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqFreeInfo *para)
{
    struct trs_normal_cqsq_mailbox msg = {0};
    struct trs_chan_sq_info sq_info;
    struct trs_chan_cq_info cq_info;
    int chan_id = ts_inst->sq_ctx[para->sqId].chan_id;
    int ret;

    (void)trs_chan_get_sq_info(&ts_inst->inst, chan_id, &sq_info);

    chan_id = ts_inst->cq_ctx[para->cqId].chan_id;
    (void)trs_chan_get_cq_info(&ts_inst->inst, chan_id, &cq_info);

    msg.info[0] = ts_inst->sq_ctx[sq_info.sqid].stream_id;
    trs_fill_notice_ts_msg(proc_ctx, TRS_MBOX_QUERY_SQ_STATUS, &sq_info, &cq_info, &msg);
    ret = ts_inst->ops.notice_ts(&ts_inst->inst, (u8 *)&msg, sizeof(msg));
    if (ret != 0) {
        trs_err("Error notice ts. (ret=%d)\n", ret);
        return ret;
    }

    return msg.header.result;
}

static int trs_hw_sqcq_free_pair(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqFreeInfo *para)
{
    struct trs_sqcq_reg_map_para reg_map_para;
    struct trs_sq_ctx *sq_ctx = NULL;
    u32 chan_flag = 0;
    int ret;

    ret = pair_mode_free_para_check(proc_ctx, ts_inst, para);
    if (ret != 0) {
        return ret;
    }

    ret = trs_hw_sqcq_status_query_notice_ts(proc_ctx, ts_inst, para);
    if (ret != 0) {
        trs_warn("Sq is using, can't free.(ret=%d)\n", ret);
        return ret;
    }

    sq_ctx = &ts_inst->sq_ctx[para->sqId];
    trs_hw_sq_show(ts_inst, para->sqId);
    trs_set_sq_status(sq_ctx, 0);

    reg_map_para.stream_id = sq_ctx->stream_id;
    reg_map_para.sqid = para->sqId;
    reg_map_para.host_pid = proc_ctx->pid;
    trs_sqcq_reg_unmap(ts_inst, &reg_map_para);
    trs_shr_sq_unmap(proc_ctx, ts_inst, sq_ctx);
    trs_sq_unmap(proc_ctx, ts_inst, sq_ctx);

    if (para->flag & TSDRV_FLAG_RSV_SQ_ID) {
        chan_flag |= (0x1 << CHAN_FLAG_RESERVED_SQ_ID_BIT);
    }
    if (para->flag & TSDRV_FLAG_RSV_CQ_ID) {
        chan_flag |= (0x1 << CHAN_FLAG_RESERVED_CQ_ID_BIT);
    }

    _trs_hw_sqcq_free_pair(proc_ctx, ts_inst, chan_flag, para->sqId, para->cqId);
    return 0;
}

static int non_pair_mode_alloc_para_check(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct halSqCqInputInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;

    if (((para->flag & TSDRV_FLAG_REUSE_SQ) != 0) && ((para->flag & TSDRV_FLAG_REUSE_CQ) == 0)) {
        trs_err("Not support only reuse sq. (devid=%u; tsid=%u; flag=%x)\n", inst->devid, inst->tsid, para->flag);
        return -EINVAL;
    }

    if ((para->flag & TSDRV_FLAG_REUSE_SQ) != 0) {
        if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, para->sqId)) {
            trs_err("Not proc owner sq. (devid=%u; tsid=%u; sqId=%u)\n", inst->devid, inst->tsid, para->sqId);
            return -EINVAL;
        }
    }

    if ((para->flag & TSDRV_FLAG_REUSE_CQ) != 0) {
        if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_CQ, para->cqId)) {
            trs_err("Not proc owner cq. (devid=%u; tsid=%u; cqId=%u)\n", inst->devid, inst->tsid, para->cqId);
            return -EINVAL;
        }
    }

    return 0;
}

static int trs_hw_sqcq_alloc_sq_chan(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqInputInfo *para, struct trs_chan_sq_info *sq_info, int *chan_id)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    int ret;

    if (trs_is_proc_res_limited(proc_ctx, ts_inst, TRS_HW_SQ)) {
        return -ENOSPC;
    }

    para->flag = TSDRV_FLAG_REUSE_CQ;
    ret = trs_hw_alloc_chan(proc_ctx, &ts_inst->inst, para, chan_id);
    if (ret != 0) {
        trs_err("Alloc sq chan failed. (devid=%u; tsid=%u; type=%d)\n", inst->devid, inst->tsid, (int)para->type);
        return ret;
    }

    (void)trs_chan_get_sq_info(inst, *chan_id, sq_info);

    ret = trs_proc_add_res(proc_ctx, ts_inst, TRS_HW_SQ, sq_info->sqid);
    if (ret != 0) {
        hal_kernel_trs_chan_destroy(inst, *chan_id);
        return ret;
    }

    return 0;
}

static void trs_hw_sqcq_free_sq_chan(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, u32 sqid)
{
    (void)trs_proc_del_res(proc_ctx, ts_inst, TRS_HW_SQ, sqid);
    hal_kernel_trs_chan_destroy(&ts_inst->inst, ts_inst->sq_ctx[sqid].chan_id);
    trs_sq_ctx_uninit(&ts_inst->sq_ctx[sqid]);
}

static int trs_hw_sqcq_alloc_cq_chan(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqInputInfo *para, struct trs_chan_cq_info *cq_info, int *chan_id)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    int ret;

    para->flag = TSDRV_FLAG_REUSE_SQ;
    ret = trs_hw_alloc_chan(proc_ctx, &ts_inst->inst, para, chan_id);
    if (ret != 0) {
        trs_err("Alloc cq chan failed. (devid=%u; tsid=%u; type=%d)\n", inst->devid, inst->tsid, para->type);
        return ret;
    }

    (void)trs_chan_get_cq_info(inst, *chan_id, cq_info);

    ret = trs_proc_add_res(proc_ctx, ts_inst, TRS_HW_CQ, cq_info->cqid);
    if (ret != 0) {
        hal_kernel_trs_chan_destroy(inst, *chan_id);
        return ret;
    }

    return 0;
}

static void trs_hw_sqcq_free_cq_chan(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, u32 cqid)
{
    (void)trs_proc_del_res(proc_ctx, ts_inst, TRS_HW_CQ, cqid);
    hal_kernel_trs_chan_destroy(&ts_inst->inst, ts_inst->cq_ctx[cqid].chan_id);
    trs_cq_ctx_uninit(&ts_inst->cq_ctx[cqid]);
}

static int trs_hw_sqcq_alloc_notice_ts(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqInputInfo *para, struct trs_chan_sq_info *sq_info, struct trs_chan_cq_info *cq_info)
{
    struct trs_normal_cqsq_mailbox msg;
    int ret;

    ret = memcpy_s(msg.info, sizeof(msg.info), para->info, sizeof(para->info));
    if (ret != 0) {
        trs_err("Memcopy failed. (dest_len=%lx; src_len=%lx)\n", sizeof(msg.info), sizeof(para->info));
        return ret;
    }

    trs_fill_notice_ts_msg(proc_ctx, TRS_MBOX_CREATE_CQSQ_CALC, sq_info, cq_info, &msg);

    return trs_core_notice_ts(ts_inst, (u8 *)&msg, sizeof(msg));
}

static int trs_hw_sqcq_alloc_non_pair(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqInputInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    struct trs_sqcq_reg_map_para reg_map_para;
    struct trs_chan_sq_info sq_info;
    struct trs_chan_cq_info cq_info;
    u32 flag = para->flag;
    u32 stream_id = para->info[0];
    int sq_chan_id, cq_chan_id, ret;

    ret = non_pair_mode_alloc_para_check(proc_ctx, ts_inst, para);
    if (ret != 0) {
        return ret;
    }

    if ((flag & TSDRV_FLAG_REUSE_SQ) == 0) {
        ret = trs_hw_sqcq_alloc_sq_chan(proc_ctx, ts_inst, para, &sq_info, &sq_chan_id);
        if (ret != 0) {
            return ret;
        }
        para->sqId = sq_info.sqid;
    } else {
        sq_info.sqid = para->sqId;
        sq_info.sq_phy_addr = 0;
        sq_info.tail_addr = 0;
    }

    if ((flag & TSDRV_FLAG_REUSE_CQ) == 0) {
        ret = trs_hw_sqcq_alloc_cq_chan(proc_ctx, ts_inst, para, &cq_info, &cq_chan_id);
        if (ret != 0) {
            if (sq_info.sq_phy_addr != 0) {
                trs_hw_sqcq_free_sq_chan(proc_ctx, ts_inst, sq_info.sqid);
            }
            return ret;
        }
        para->cqId = cq_info.cqid;
    } else {
        cq_info.cqid = para->cqId;
        cq_info.cq_phy_addr = 0;
        cq_info.irq = 0;
    }

    ts_inst->sq_ctx[sq_info.sqid].cqid = para->cqId;

    para->flag = flag;

    ret = trs_hw_sqcq_alloc_notice_ts(proc_ctx, ts_inst, para, &sq_info, &cq_info);
    if (ret != 0) {
        if ((flag & TSDRV_FLAG_REUSE_SQ) == 0) {
            trs_hw_sqcq_free_sq_chan(proc_ctx, ts_inst, sq_info.sqid);
        }
        if ((flag & TSDRV_FLAG_REUSE_CQ) == 0) {
            trs_hw_sqcq_free_cq_chan(proc_ctx, ts_inst, cq_info.cqid);
        }
        trs_err("Notice ts failed. (devid=%u; tsid=%u; ret=%d)\n", inst->devid, inst->tsid, ret);
        return ret;
    }

    if (sq_info.sq_phy_addr != 0) {
        trs_sq_ctx_init(inst, &ts_inst->sq_ctx[sq_info.sqid], para, stream_id, sq_chan_id);
    }
    if (cq_info.cq_phy_addr != 0) {
        trs_cq_ctx_init(&ts_inst->cq_ctx[cq_info.cqid], U32_MAX, cq_chan_id);
    }
    reg_map_para.sqid = sq_info.sqid;
    reg_map_para.cqid = cq_info.cqid;
    reg_map_para.host_pid = proc_ctx->pid;
    reg_map_para.stream_id = stream_id;
    trs_sqcq_reg_map(ts_inst, &reg_map_para);

    if (sq_info.sq_phy_addr != 0) {
        /* If the map fails, the UIO is not used. Therefore, the return value of the map is ignored. */
        (void)trs_sq_remap(proc_ctx, ts_inst, para, &ts_inst->sq_ctx[sq_info.sqid], &sq_info);
        trs_set_sq_status(&ts_inst->sq_ctx[sq_info.sqid], 1);
    }

    trs_info("Alloc non pair sqcq. (devid=%u; tsid=%u; sqId=%u; cqId=%u; flag=%x)\n",
        inst->devid, inst->tsid, para->sqId, para->cqId, para->flag);

    return 0;
}

static int non_pair_mode_free_para_check(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqFreeInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, para->sqId)) {
        trs_err("Not proc owner sq. (devid=%u; tsid=%u; sqId=%u)\n", inst->devid, inst->tsid, para->sqId);
        return -EINVAL;
    }

    if ((para->flag & TSDRV_FLAG_REUSE_CQ) == 0) {
        if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_CQ, para->cqId)) {
            trs_err("Not proc owner cq. (devid=%u; tsid=%u; cqId=%u)\n", inst->devid, inst->tsid, para->cqId);
            return -EINVAL;
        }
    }

    return 0;
}

static int trs_hw_sqcq_free_notice_ts(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqFreeInfo *para)
{
    struct trs_normal_cqsq_mailbox msg;
    struct trs_chan_sq_info sq_info;
    struct trs_chan_cq_info cq_info;
    int chan_id = ts_inst->sq_ctx[para->sqId].chan_id;

    (void)trs_chan_get_sq_info(&ts_inst->inst, chan_id, &sq_info);

    if ((para->flag & TSDRV_FLAG_REUSE_CQ) == 0) {
        chan_id = ts_inst->cq_ctx[para->cqId].chan_id;
        (void)trs_chan_get_cq_info(&ts_inst->inst, chan_id, &cq_info);
    } else {
        cq_info.cqid = TRS_MBOX_INVALID_INDEX;
    }

    msg.info[0] = ts_inst->sq_ctx[sq_info.sqid].stream_id;
    trs_fill_notice_ts_msg(proc_ctx, TRS_MBOX_RELEASE_CQSQ_CALC, &sq_info, &cq_info, &msg);

    return trs_core_notice_ts(ts_inst, (u8 *)&msg, sizeof(msg));
}

static int trs_hw_sqcq_free_non_pair(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqFreeInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    int ret;

    ret = non_pair_mode_free_para_check(proc_ctx, ts_inst, para);
    if (ret != 0) {
        return ret;
    }

    ret = trs_hw_sqcq_free_notice_ts(proc_ctx, ts_inst, para);
    if (ret != 0) {
        trs_info("Notice ts warn. (devid=%u; tsid=%u; sqId=%u; cqId=%u; flag=%x)\n",
            inst->devid, inst->tsid, para->sqId, para->cqId, para->flag);
        return ret;
    }

    trs_info("Free non pair sqcq. (devid=%u; tsid=%u; sqId=%u; cqId=%u; flag=%x)\n",
        inst->devid, inst->tsid, para->sqId, para->cqId, para->flag);

    trs_set_sq_status(&ts_inst->sq_ctx[para->sqId], 0);
    trs_sq_unmap(proc_ctx, ts_inst, &ts_inst->sq_ctx[para->sqId]);
    trs_hw_sqcq_free_sq_chan(proc_ctx, ts_inst, para->sqId);

    if ((para->flag & TSDRV_FLAG_REUSE_CQ) == 0) {
        trs_hw_sqcq_free_cq_chan(proc_ctx, ts_inst, para->cqId);
    }

    return 0;
}

#define TRS_SQCQ_EXT_INFO_MAX_LEN 256
int trs_hw_sqcq_alloc(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, struct halSqCqInputInfo *para)
{
    u8 ext_info[TRS_SQCQ_EXT_INFO_MAX_LEN];
    struct trs_ext_info_header *header = NULL;
    u8 *user_ext_info = (u8 *)para->ext_info;
    u32 stream_id = para->info[0];
    int ret;

    if (para->ext_info != NULL) {
        if ((para->ext_info_len == 0) || (para->ext_info_len > TRS_SQCQ_EXT_INFO_MAX_LEN)) {
            trs_err("Invalid value. (ext_info_len=%u)\n", para->ext_info_len);
            return -EINVAL;
        }
        ret = ka_base_copy_from_user(ext_info, (u8 __user *)para->ext_info, para->ext_info_len);
        if (ret != 0) {
            trs_err("Failed to copy ext_info from user. (len=%u)\n", para->ext_info_len);
            return ret;
        }
        para->ext_info = ext_info;
    }

    header = para->ext_info;
    if (((header == NULL) || ((header != NULL) && (header->type == CHAN_SUB_TYPE_HW_RTS))) && (stream_id != U32_MAX)) {
        if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_STREAM, stream_id)) {
            int ret = -EINVAL;
            if ((para->flag & TSDRV_FLAG_AGENT_ID) != 0) {
                if (ts_inst->ops.res_id_check != NULL) {
                    ret = ts_inst->ops.res_id_check(&ts_inst->inst, TRS_STREAM, stream_id);
                }
            }
            if (ret != 0) {
                trs_err("No stream. (ret=%d; devid=%u; tsid=%u; stream_id=%u)\n",
                    ret, ts_inst->inst.devid, ts_inst->inst.tsid, stream_id);
                return ret;
            }
        }
    }

    if (trs_is_sqcq_pair_mode(ts_inst)) {
        ret = trs_hw_sqcq_alloc_pair(proc_ctx, ts_inst, para);
    } else {
        ret = trs_hw_sqcq_alloc_non_pair(proc_ctx, ts_inst, para);
    }
    para->ext_info = user_ext_info;
    return ret;
}

int trs_hw_sqcq_free(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, struct halSqCqFreeInfo *para)
{
    if (trs_is_sqcq_pair_mode(ts_inst)) {
        return trs_hw_sqcq_free_pair(proc_ctx, ts_inst, para);
    } else {
        return trs_hw_sqcq_free_non_pair(proc_ctx, ts_inst, para);
    }
}

#ifdef CFG_FEATURE_SUPPORT_STREAM_TASK
static int trs_sq_switch_stream_notice_ts(struct trs_core_ts_inst *ts_inst, u32 sqid, u32 stream_id,
    u64 bar_addr, u32 sq_depth)
{
    struct trs_sq_switch_stream_info msg;
    u64 paddr = bar_addr;
    int ret;

    if ((bar_addr != 0) && (ts_inst->ops.get_connect_protocol(&ts_inst->inst) == TRS_CONNECT_PROTOCOL_PCIE)) {
        if (ts_inst->ops.mem_update != NULL) {
            ret = ts_inst->ops.mem_update(&ts_inst->inst, bar_addr, &paddr, 1);
            if (ret != 0) {
                trs_err("Failed to get pa. (ret=%d; devid=%u)\n", ret, ts_inst->inst.devid);
                return ret;
            }
        }
    }

    trs_mbox_init_header(&msg.header, TRS_MBOX_SQ_SWITCH_STREAM);
    msg.cnt = 1;
    msg.nodes[0].sq_id = sqid;
    msg.nodes[0].stream_id = stream_id;
    msg.nodes[0].sq_addr = paddr;
    msg.nodes[0].sq_depth = sq_depth;
    trs_debug("(devid=%u; sqid=%u; stream_id=%u; sq_depth=%u)\n", ts_inst->inst.devid, sqid, stream_id, sq_depth);

    ret = ts_inst->ops.notice_ts(&ts_inst->inst, (u8 *)&msg, sizeof(struct trs_sq_switch_stream_info));
    if ((ret != 0) || (msg.header.result != 0)) {
        trs_err("Notice ts failed. (ret=%d; result=%u; devid=%u; stream_id=%u; sqid=%u)\n", ret, msg.header.result,
            ts_inst->inst.devid, stream_id, sqid);
        return (ret != 0) ? ret : -EINVAL;
    }
    return 0;
}

int trs_sq_switch_stream(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct sq_switch_stream_info *info)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    u64 stream_pa_addr = 0;
    u32 old_stream_id;
    int ret;

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, info->sq_id)) {
        trs_err("Not proc sq. (devid=%u; tsid=%u; sqid=%u)\n", inst->devid, inst->tsid, info->sq_id);
        return -EINVAL;
    }

    if (info->sq_depth > 32768) { /* 32768: support sq max depth, 32K */
        trs_err("Invalid sq depth. (sq_depth=%u)\n", info->sq_depth);
        return -EINVAL;
    }

    ret = trs_chan_update_sq_depth(&ts_inst->inst, ts_inst->sq_ctx[info->sq_id].chan_id, info->sq_depth);
    if (ret != 0) {
        return ret;
    }

    if (info->stream_id != U32_MAX) {
        if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_STREAM, info->stream_id)) {
            trs_err("Not proc sq. (devid=%u; tsid=%u; stream_id=%u)\n", inst->devid, inst->tsid, info->stream_id);
            return -EINVAL;
        }

        if (ts_inst->stream_ctx[info->stream_id].stream_base_addr == 0) {
            trs_err("Not fill stream memory. (devid=%u; tsid=%u; stream_id=%u)\n",
                inst->devid, inst->tsid, info->stream_id);
            return -EINVAL;
        }
        stream_pa_addr = ts_inst->stream_ctx[info->stream_id].pa_list[0] +
            ts_inst->stream_ctx[info->stream_id].stream_base_addr - ts_inst->stream_ctx[info->stream_id].stream_uva;
    }

    ret = trs_sq_switch_stream_notice_ts(ts_inst, info->sq_id, info->stream_id, stream_pa_addr, info->sq_depth);
    if (ret != 0) {
        trs_err("Notice ts failed. (ret=%d; devid=%u; tsid=%u; stream_id=%u; sqid=%u)\n",
            ret, inst->devid, inst->tsid, info->stream_id, info->sq_id);
        return ret;
    }

    old_stream_id = ts_inst->sq_ctx[info->sq_id].stream_id;
    if (old_stream_id < trs_res_get_max_id(ts_inst, TRS_STREAM)) {
        ts_inst->stream_ctx[old_stream_id].sq = -1;
    }
    ts_inst->sq_ctx[info->sq_id].stream_id = info->stream_id;

    if (info->stream_id != U32_MAX) {
        ts_inst->stream_ctx[info->stream_id].sq = info->sq_id;
        trs_stream_set_bind_sqcq(ts_inst, info->stream_id, info->sq_id, ts_inst->sq_ctx[info->sq_id].cqid,
            proc_ctx->pid);
    }

    trs_debug("Sq switches stream success. (devid=%u; tsid=%u; stream_id=%u; sqid=%u)\n",
        inst->devid, inst->tsid, info->stream_id, info->sq_id);

    return 0;
}
#endif

int trs_hw_sqcq_get(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, struct halSqCqInputInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    struct trs_chan_sq_info sq_info = {0};
    struct trs_chan_cq_info cq_info = {0};
    struct trs_stream_ctx stm_ctx = {0};
    u32 chan_id, stream_id;
    int sqid, cqid;
    int ret;

    stream_id = para->info[0];
    ret = trs_get_stream_ctx(proc_ctx, ts_inst, stream_id, &stm_ctx);
    if (ret != 0) {
        return ret;
    }

    sqid = stm_ctx.sq;
    if ((sqid < 0) || ((u32)(sqid) >= trs_res_get_max_id(ts_inst, TRS_HW_SQ))) {
        trs_err("Stream no sq. (devid=%u; tsid=%u; stream_id=%u; sqId=%d)\n", inst->devid, inst->tsid, stream_id, sqid);
        return -EINVAL;
    }

    cqid = stm_ctx.cq;
    if ((cqid < 0) || ((u32)(cqid) >= trs_res_get_max_id(ts_inst, TRS_HW_CQ))) {
        trs_err("Stream no cq. (devid=%u; tsid=%u; stream_id=%u; cqId=%d)\n", inst->devid, inst->tsid, stream_id, sqid);
        return -EINVAL;
    }

    chan_id = ts_inst->sq_ctx[sqid].chan_id;
    (void)trs_chan_get_sq_info(&ts_inst->inst, chan_id, &sq_info);
    (void)trs_chan_get_cq_info(&ts_inst->inst, chan_id, &cq_info);

    para->sqId = sqid;
    para->cqId = cqid;

#ifndef EMU_ST
    para->sqeDepth = sq_info.sq_para.sq_depth;
    para->sqeSize = sq_info.sq_para.sqe_size;
    para->cqeDepth = cq_info.cq_para.cq_depth;
    para->cqeSize = cq_info.cq_para.cqe_size;
#else
    para->sqeDepth = 1024;  /* 1024 is sq depth for ut */
    para->sqeSize = 64;     /* 64 is sqe size for ut */
    para->cqeDepth = 512;   /* 512 is cq depth for ut */
    para->cqeSize = 32;     /* 32 is cqe size for ut */
#endif

    trs_debug("Share sqcq info. (devid=%u; streamid=%u; sqid=%u; sq_depth=%u; sqe_size=%u; cqid=%u; "
        "cq_depth=%u; cqe_size=%u; logic_cqid=%d)\n", proc_ctx->devid, stream_id, para->sqId, para->sqeDepth, para->sqeSize,
        para->cqId, para->cqeDepth, para->cqeSize, para->info[1]);

    return 0;
}

static int trs_sqcq_restore_para_check(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct halSqCqInputInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;

    if ((para->flag & TSDRV_FLAG_REUSE_CQ) != 0) {
        trs_err("Not support. (devid=%u; tsid=%u; flag=%x)\n", inst->devid, inst->tsid, para->flag);
        return -EINVAL;
    }

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, para->sqId)) {
        trs_err("Not proc sq. (devid=%u; tsid=%u; sq=%u; cq=%u)\n", inst->devid, inst->tsid, para->sqId, para->cqId);
        return -EINVAL;
    }

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_CQ, para->cqId)) {
        trs_err("Not proc cq. (devid=%u; tsid=%u; sq=%u; cq=%u)\n", inst->devid, inst->tsid, para->sqId, para->cqId);
        return -EINVAL;
    }

    if (ts_inst->sq_ctx[para->sqId].chan_id != ts_inst->cq_ctx[para->cqId].chan_id) {
#ifndef EMU_ST
        trs_err("Not pair sqcq. (devid=%u; tsid=%u; sqId=%u; cqId=%u)\n",
            inst->devid, inst->tsid, para->sqId, para->cqId);
        return -EINVAL;
#endif
    }

    return 0;
}

int trs_hw_sqcq_restore(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct halSqCqInputInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    struct trs_chan_sq_info sq_info;
    int chan_id, ret;

    ret = trs_sqcq_restore_para_check(proc_ctx, ts_inst, para);
    if (ret != 0) {
        return ret;
    }

    chan_id = ts_inst->sq_ctx[para->sqId].chan_id;
    (void)trs_chan_get_sq_info(inst, chan_id, &sq_info);

    ret = trs_shr_sq_remap(proc_ctx, ts_inst, para, &ts_inst->sq_ctx[sq_info.sqid], &sq_info);
    if (ret != 0) {
        trs_err("Share sq map failed. (devid=%u; sqid=%u)\n", inst->devid, sq_info.sqid);
        return ret;
    }

    trs_debug("Restore sqcq. (devid=%u; tsid=%u; flag=0x%x; sqId=%u; cqId=%u)\n",
        inst->devid, inst->tsid, para->flag, para->sqId, para->cqId);

    return 0;
}

void trs_proc_diable_sq_status(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    int res_type, u32 res_id)
{
    struct trs_sq_ctx *sq_ctx = &ts_inst->sq_ctx[res_id];
    struct trs_id_inst *inst = &ts_inst->inst;

    trs_set_sq_status(sq_ctx, 0);
    trs_debug("Disable sq success. (devid=%u; tsid=%u; pid=%u; sqid=%u)\n", inst->devid, inst->tsid,
        proc_ctx->pid, res_id);
}

static void trs_proc_enable_sq_status(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    int res_type, u32 res_id)
{
    struct trs_sq_ctx *sq_ctx = &ts_inst->sq_ctx[res_id];
    struct trs_id_inst *inst = &ts_inst->inst;

    trs_set_sq_status(sq_ctx, 1);
    trs_debug("Enable sq success. (devid=%u; tsid=%u; pid=%u; sqid=%u)\n", inst->devid, inst->tsid,
        proc_ctx->pid, res_id);
}

static int trs_sq_task_stop(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, u32 sqid)
{
    trs_proc_diable_sq_status(proc_ctx, ts_inst, TRS_HW_SQ, sqid);
    return 0;
}

static int trs_sq_task_resume(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, u32 sqid)
{
    trs_proc_enable_sq_status(proc_ctx, ts_inst, TRS_HW_SQ, sqid);
    return 0;
}

static int trs_hw_sqcq_config_of_proc(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    uint32_t type, int prop)
{
    u32 i;
    int ret = 0;

    for (i = 0; i < ts_inst->res_mng[type].max_id; i++) {
        struct trs_res_ids *id = &ts_inst->res_mng[type].ids[i];
        if ((id->ref != 0) && (proc_ctx->pid == id->pid) &&
            (proc_ctx->task_id == id->task_id) && (id->status == RES_STATUS_NORMAL)) {
            if ((type == TRS_HW_SQ) && (prop == DRV_SQCQ_PROP_SQ_PAUSE)) {
                ret |= trs_sq_task_stop(proc_ctx, ts_inst, i);
            }
            if ((type == TRS_HW_SQ) && (prop == DRV_SQCQ_PROP_SQ_RESUME)) {
                ret |= trs_sq_task_resume(proc_ctx, ts_inst, i);
            }
            if ((type == TRS_HW_CQ) && (prop == DRV_SQCQ_PROP_SQ_PAUSE)) {
                int chan_id = ts_inst->cq_ctx[i].chan_id;
                ret |= trs_chan_ctrl(&ts_inst->inst, chan_id, CHAN_CTRL_CMD_CQ_PAUSE, 0);
            }
            if ((type == TRS_HW_CQ) && (prop == DRV_SQCQ_PROP_SQ_RESUME)) {
                int chan_id = ts_inst->cq_ctx[i].chan_id;
                ret |= trs_chan_ctrl(&ts_inst->inst, chan_id, CHAN_CTRL_CMD_CQ_RESUME, 0);
            }
        }
    }

    return ret;
}

static int trs_config_sq_tail_check(struct trs_core_ts_inst *ts_inst, struct halSqCqConfigInfo *para)
{
    u32 stream_id = ts_inst->sq_ctx[para->sqId].stream_id;
    if (stream_id >= trs_res_get_max_id(ts_inst, TRS_STREAM)) {
        trs_err("Stream id is invalid. (devid=%u; sqid=%u; stream_id=%u)\n",
            ts_inst->inst.devid, para->sqId, stream_id);
        return -EINVAL;
    }
    if ((ts_inst->stream_ctx[stream_id].pa_list != NULL) &&
        (para->value[0] > ts_inst->stream_ctx[stream_id].tail)) {
        trs_err("Invalid tail. (tail=%u; value=%u)\n", ts_inst->stream_ctx[stream_id].tail, para->value[0]);
        return -EINVAL;
    }
    trs_debug("Config tail success. (devid=%u; sqid=%u; tail=%u; value=%u)\n",
        ts_inst->inst.devid, para->sqId, ts_inst->stream_ctx[stream_id].tail, para->value[0]);
    return 0;
}

int trs_hw_sqcq_config(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, struct halSqCqConfigInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    u32 sqid = para->sqId;
    u32 cqid = para->cqId;
    int chan_id, ret;

    if ((sqid == U32_MAX) && (cqid == U32_MAX) &&
        ((para->prop == DRV_SQCQ_PROP_SQ_PAUSE) || (para->prop == DRV_SQCQ_PROP_SQ_RESUME))) {
        ret = trs_hw_sqcq_config_of_proc(proc_ctx, ts_inst, TRS_HW_SQ, para->prop);
        ret |= trs_hw_sqcq_config_of_proc(proc_ctx, ts_inst, TRS_HW_CQ, para->prop); /* cq need to pause in UB scene */
        if (ret != 0) {
            trs_err("Proc sqcq config failed. (devid=%u; pid=%u; prop=%d; ret=%d)\n",
                inst->devid, proc_ctx->pid, para->prop, ret);
            return ret;
        }
        trs_debug("Proc sqcq config success. (devid=%u; pid=%u; prop=%d)\n", inst->devid, proc_ctx->pid, para->prop);
        return ret;
    }

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, sqid)) {
        trs_err("Proc no sq. (devid=%u; tsid=%u; sqId=%u)\n", ts_inst->inst.devid, ts_inst->inst.tsid, sqid);
        return -EINVAL;
    }

    chan_id = ts_inst->sq_ctx[sqid].chan_id;
    switch (para->prop) {
        case DRV_SQCQ_PROP_SQ_STATUS:
            ret = trs_chan_ctrl(inst, chan_id, CHAN_CTRL_CMD_SQ_STATUS_SET, para->value[0]);
            break;
        case DRV_SQCQ_PROP_SQ_HEAD:
            ret = trs_chan_ctrl(inst, chan_id, CHAN_CTRL_CMD_SQ_HEAD_SET, para->value[0]);
            break;
        case DRV_SQCQ_PROP_SQ_TAIL:
            ret = trs_config_sq_tail_check(ts_inst, para);
            if (ret != 0) {
                return ret;
            }
            ret = trs_chan_ctrl(inst, chan_id, CHAN_CTRL_CMD_SQ_TAIL_SET, para->value[0]);
            break;
        case DRV_SQCQ_PROP_SQ_DISABLE_TO_ENABLE:
            ret = trs_chan_ctrl(inst, chan_id, CHAN_CTRL_CMD_SQ_DISABLE_TO_ENABLE, para->value[0]);
            break;
        case DRV_SQCQ_PROP_SQ_PAUSE:
            ret = trs_sq_task_stop(proc_ctx, ts_inst, sqid);
            ret |= trs_chan_ctrl(inst, chan_id, CHAN_CTRL_CMD_CQ_PAUSE, 0); /* cq also need to pause in UB scene */
            break;
        case DRV_SQCQ_PROP_SQ_RESUME:
            ret |= trs_sq_task_resume(proc_ctx, ts_inst, sqid);
            ret = trs_chan_ctrl(inst, chan_id, CHAN_CTRL_CMD_CQ_RESUME, 0); /* cq also need to pause in UB scene */
            break;
        case DRV_SQCQ_PROP_SQCQ_RESET:
            ret = trs_chan_ctrl(inst, chan_id, CHAN_CTRL_CMD_SQCQ_RESET, 0);
            break;
        default:
            trs_err("Invalid prop. (devid=%u; tsid=%u, prop=%u)\n", inst->devid, inst->tsid, para->prop);
            return -EINVAL;
    }

    if (ret != 0) {
        trs_err("Cfg failed. (devid=%u; tsid=%u; sqId=%u; cqId=%u; prop=%d)\n",
            inst->devid, inst->tsid, sqid, cqid, para->prop);
    }

    return ret;
}

int trs_set_sq_reg_vaddr(struct trs_id_inst *inst, u32 sqid, u64 va, size_t size)
{
    struct trs_core_ts_inst *ts_inst = NULL;

    if (inst == NULL) {
        trs_err("Null ptr. (sqid=%u)\n", sqid);
        return -EINVAL;
    }
    ts_inst = trs_core_ts_inst_get(inst);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", inst->devid, inst->tsid);
        return -EINVAL;
    }
    if (sqid >= trs_res_get_max_id(ts_inst, TRS_HW_SQ)) {
        trs_core_ts_inst_put(ts_inst);
        trs_err("Invalid para. (devid=%u; tsid=%u; sqid=%u)\n",
            inst->devid, inst->tsid, sqid);
        return -EINVAL;
    }
    ts_inst->sq_ctx[sqid].reg_mem.uva = va;
    ts_inst->sq_ctx[sqid].reg_mem.len = size;
    trs_core_ts_inst_put(ts_inst);
    return 0;
}
KA_EXPORT_SYMBOL_GPL(trs_set_sq_reg_vaddr);

int trs_get_sq_reg_vaddr(struct trs_id_inst *inst, u32 sqid, u64 *va, size_t *size)
{
    struct trs_core_ts_inst *ts_inst = NULL;

    if (inst == NULL) {
        trs_err("Null ptr. (sqid=%u)\n", sqid);
        return -EINVAL;
    }
    ts_inst = trs_core_ts_inst_get(inst);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", inst->devid, inst->tsid);
        return -EINVAL;
    }
    if (sqid >= trs_res_get_max_id(ts_inst, TRS_HW_SQ)) {
        trs_core_ts_inst_put(ts_inst);
        trs_err("Invalid para. (devid=%u; tsid=%u; sqid=%u)\n", inst->devid, inst->tsid, sqid);
        return -EINVAL;
    }

    if (ts_inst->sq_ctx[sqid].reg_mem.uva == 0) {
        trs_core_ts_inst_put(ts_inst);
        return -EOPNOTSUPP;
    }

    if (va != NULL) {
        *va = ts_inst->sq_ctx[sqid].reg_mem.uva;
    }
    if (size != NULL) {
        *size = ts_inst->sq_ctx[sqid].reg_mem.len;
    }
    trs_core_ts_inst_put(ts_inst);
    return 0;
}
KA_EXPORT_SYMBOL_GPL(trs_get_sq_reg_vaddr);

static int trs_query_sq_reg_vaddr(struct trs_id_inst *inst, u32 sqid, struct halSqCqQueryInfo *para)
{
    size_t size;
    int ret;
    u64 va;

    ret = trs_get_sq_reg_vaddr(inst, sqid, &va, &size);
    if (ret == 0) {
        u32 mask = sizeof(uint32_t) * BITS_PER_BYTE;
        para->value[0] = (uint32_t)(va >> mask);
        para->value[1] = (uint32_t)(va & ((1ULL << mask) - 1));
        para->value[2] = (uint32_t)size; /* 2 return Sq reg size */
    }
    return ret;
}

int trs_sqcq_query(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, struct halSqCqQueryInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    u32 sqid = para->sqId;
    int chan_id, ret;

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, sqid)) {
        trs_err("Proc no sq. (devid=%u; tsid=%u; sqId=%u)\n", ts_inst->inst.devid, ts_inst->inst.tsid, sqid);
        return -EINVAL;
    }

    chan_id = ts_inst->sq_ctx[sqid].chan_id;
    switch (para->prop) {
        case DRV_SQCQ_PROP_SQ_STATUS:
            ret = trs_chan_query(inst, chan_id, CHAN_QUERY_CMD_SQ_STATUS, &para->value[0]);
            break;
        case DRV_SQCQ_PROP_SQ_HEAD:
            ret = trs_chan_query(inst, chan_id, CHAN_QUERY_CMD_SQ_HEAD, &para->value[0]);
            break;
        case DRV_SQCQ_PROP_SQ_TAIL:
            ret = trs_chan_query(inst, chan_id, CHAN_QUERY_CMD_SQ_TAIL, &para->value[0]);
            break;
        case DRV_SQCQ_PROP_SQ_CQE_STATUS:
            trs_sq_get_and_clr_cqe_status(ts_inst, sqid, &para->value[0]);
            ret = 0;
            break;
        case DRV_SQCQ_PROP_SQ_REG_BASE:
            ret = trs_query_sq_reg_vaddr(inst, sqid, para);
            break;
        default:
            trs_err("Invalid prop. (devid=%u; tsid=%u, prop=%u)\n", inst->devid, inst->tsid, para->prop);
            return -EINVAL;
    }

    if (ret != 0) {
        trs_err("Query failed. (devid=%u; tsid=%u; sqId=%u; prop=%d; ret=%d)\n",
            inst->devid, inst->tsid, sqid, para->prop, ret);
    }

    return ret;
}

int trs_hw_sqcq_send(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, struct halTaskSendInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    struct trs_chan_send_para send_para;
    int ret;

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, para->sqId)) {
        trs_err("Not proc owner sq. (devid=%u; tsid=%u; sqId=%u)\n", inst->devid, inst->tsid, para->sqId);
        return -EINVAL;
    }

    send_para.sqe = para->sqe_addr;
    send_para.sqe_num = para->sqe_num;
    send_para.timeout = para->timeout;

    if (ts_inst->sq_ctx[para->sqId].status == 0) {
        return -ESHUTDOWN;
    }

    ret = trs_chan_send(inst, ts_inst->sq_ctx[para->sqId].chan_id, &send_para);
    para->pos = send_para.first_pos;
    return ret;
}

int trs_hw_sqcq_recv(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, struct halReportRecvInfo *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    struct trs_chan_recv_para recv_para;
    int ret;

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_CQ, para->cqId)) {
        trs_err("Not proc owner cq. (devid=%u; tsid=%u; cqId=%u)\n", inst->devid, inst->tsid, para->cqId);
        return -EINVAL;
    }

    recv_para.cqe = para->cqe_addr;
    recv_para.cqe_num = para->cqe_num;
    recv_para.timeout = para->timeout;

    ret = trs_chan_recv(inst, ts_inst->cq_ctx[para->cqId].chan_id, &recv_para);
    if (ret != 0) {
        return ret;
    }

    para->report_cqe_num = recv_para.recv_cqe_num;

    return 0;
}

void trs_hw_sqcq_recycle(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst, int res_type, u32 res_id)
{
    struct trs_id_inst *inst = &ts_inst->inst;

    if (res_type == TRS_HW_SQ) {
        struct trs_sq_ctx *sq_ctx = &ts_inst->sq_ctx[res_id];
        trs_shr_sq_unmap(proc_ctx, ts_inst, sq_ctx);
        trs_sq_ctx_mem_free(sq_ctx);
        trs_sq_clear_map_info(sq_ctx);
        if (proc_ctx->force_recycle) {
            (void)trs_chan_ctrl(&ts_inst->inst, sq_ctx->chan_id, CHAN_CTRL_CMD_NOT_NOTICE_TS, 0);
        }
        if (trs_is_sqcq_pair_mode(ts_inst)) {
            _trs_hw_sqcq_free_pair(proc_ctx, ts_inst, 0, res_id, sq_ctx->cqid);
        } else {
            trs_hw_sqcq_free_sq_chan(proc_ctx, ts_inst, res_id);
        }
    } else {
        if (trs_is_sqcq_pair_mode(ts_inst)) {
            trs_err("Unexpected. (devid=%u; tsid=%u; cqId=%u)\n", inst->devid, inst->tsid, res_id);
        } else {
            if (proc_ctx->force_recycle) {
                (void)trs_chan_ctrl(&ts_inst->inst, ts_inst->cq_ctx[res_id].chan_id, CHAN_CTRL_CMD_NOT_NOTICE_TS, 0);
            }
            trs_hw_sqcq_free_cq_chan(proc_ctx, ts_inst, res_id);
        }
    }
}

int trs_hw_sqcq_dma_desc_create(struct trs_proc_ctx *proc_ctx, struct trs_core_ts_inst *ts_inst,
    struct trs_cmd_dma_desc *para)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    u32 sqid = para->sq_id;
    int chan_id, ret;

    if (!trs_proc_has_res(proc_ctx, ts_inst, TRS_HW_SQ, sqid)) {
        trs_err("Proc no sq. (devid=%u; tsid=%u; sqId=%u)\n", ts_inst->inst.devid, ts_inst->inst.tsid, sqid);
        return -EINVAL;
    }

    chan_id = ts_inst->sq_ctx[sqid].chan_id;
    ret = trs_chan_dma_desc_create(inst, chan_id, (struct trs_chan_dma_desc *)para);
    if (ret != 0) {
        trs_err("Sqcq dma desc create failed. (devid=%u; tsid=%u; sqid=%u; ret=%d)\n",
            proc_ctx->devid, para->tsid, para->sq_id, ret);
        return -EINVAL;
    }

    return 0;
}

int trs_hw_sq_send_thread_create(struct trs_core_ts_inst *ts_inst)
{
    struct trs_id_inst *inst = &ts_inst->inst;

    ts_inst->sq_task = kthread_create(trs_sq_send_thread, (void *)ts_inst, "dev%u_sq_task", inst->devid);
    if (KA_IS_ERR_OR_NULL(ts_inst->sq_task)) {
        trs_err("Failed to start hw sq send thread. (devid=%u; tsid=%u)\n", inst->devid, inst->tsid);
        return -1;
    }

    if (ts_inst->ops.set_thread_affinity != NULL) {
        ts_inst->ops.set_thread_affinity(inst, ts_inst->sq_task);
    }

    ka_task_wake_up_process(ts_inst->sq_task);

    return 0;
}

void trs_hw_sq_send_thread_destroy(struct trs_core_ts_inst *ts_inst)
{
    if (!KA_IS_ERR_OR_NULL(ts_inst->sq_task)) {
        (void)ka_task_kthread_stop(ts_inst->sq_task);
        ts_inst->sq_task = NULL;
    }
}

static u32 trs_hw_sq_get_wq_flag(u32 ts_inst_flag)
{
    u32 flag = __WQ_LEGACY | WQ_MEM_RECLAIM | WQ_SYSFS;

    if ((ts_inst_flag & TRS_CORE_SQ_TRIGGER_WQ_UNBIND_FLAG) != 0) {
        flag |= WQ_UNBOUND;
    }

    return flag;
}

static u32 trs_hw_sq_get_trigger_wq_flag(u32 ts_inst_flag)
{
    return trs_hw_sq_get_wq_flag(ts_inst_flag) | WQ_HIGHPRI;
}

static u32 trs_hw_sq_get_fair_wq_flag(u32 ts_inst_flag)
{
    return trs_hw_sq_get_wq_flag(ts_inst_flag);
}

static int trs_hw_sq_trigger_init(struct trs_core_ts_inst *ts_inst)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    int ret;

    if (ts_inst->ops.get_sq_trigger_irq == NULL) {
        return 0;
    }

    ret = ts_inst->ops.get_sq_trigger_irq(inst, &ts_inst->sq_trigger_irq, &ts_inst->irq_type);
    if (ret != 0) {
        trs_err("Get irq failed. (devid=%u; tsid=%u; ret=%d)\n", inst->devid, inst->tsid, ret);
        return ret;
    }

    ts_inst->work_queue = ka_task_alloc_workqueue("dev%u_sq_send_wq",
        trs_hw_sq_get_trigger_wq_flag(ts_inst->ts_inst_flag), 1, inst->devid);
    if (ts_inst->work_queue == NULL) {
        trs_err("Createn wq failed. (devid=%u; tsid=%u)\n", inst->devid, inst->tsid);
#ifndef EMU_ST
        return -EFAULT;
#endif
    }

    ts_inst->fair_work_queue = ka_task_alloc_workqueue("dev%u_sq_fair_wq",
        trs_hw_sq_get_fair_wq_flag(ts_inst->ts_inst_flag), 1, inst->devid);
    if (ts_inst->fair_work_queue == NULL) {
        ka_task_destroy_workqueue(ts_inst->work_queue);
        trs_err("Create fair wq failed. (devid=%u; tsid=%u)\n", inst->devid, inst->tsid);
#ifndef EMU_ST
        return -EFAULT;
#endif
    }

    KA_TASK_INIT_WORK(&ts_inst->sq_trigger_work, trs_sq_trigger_work);
    KA_TASK_INIT_WORK(&ts_inst->sq_fair_work, trs_sq_send_fair_work);
    ret = trs_hw_sq_send_thread_create(ts_inst);
    if (ret != 0) {
#ifndef EMU_ST
        ka_task_destroy_workqueue(ts_inst->fair_work_queue);
        ka_task_destroy_workqueue(ts_inst->work_queue);
#endif
        trs_err("Failed to create hw sq send thread. (devid=%u; tsid=%u; ret=%d)\n", inst->devid, inst->tsid, ret);
        return ret;
    }

    ret = ts_inst->ops.request_irq(inst, ts_inst->irq_type, ts_inst->sq_trigger_irq,
        (void *)ts_inst, trs_sq_trigger_irq_proc);
    if (ret != 0) {
#ifndef EMU_ST
        trs_hw_sq_send_thread_destroy(ts_inst);
        ka_task_destroy_workqueue(ts_inst->fair_work_queue);
        ka_task_destroy_workqueue(ts_inst->work_queue);
#endif
        trs_err("Request irq failed. (devid=%u; tsid=%u; ret=%d)\n", inst->devid, inst->tsid, ret);
        return ret;
    }

    if (ts_inst->ops.set_trigger_irq_affinity != NULL) {
        ts_inst->ops.set_trigger_irq_affinity(inst, ts_inst->sq_trigger_irq, 1);
    }
    trs_info("Sq trigger send thread init ok. add fair work (devid=%u)\n", inst->devid);
    return 0;
}

void trs_hw_sq_trigger_irq_hw_res_uninit(struct trs_core_ts_inst * ts_inst)
{
    if (ts_inst->ops.get_sq_trigger_irq != NULL) {
        if (ts_inst->ops.set_trigger_irq_affinity != NULL) {
            ts_inst->ops.set_trigger_irq_affinity(&ts_inst->inst, ts_inst->sq_trigger_irq, 0);
        }
        ts_inst->ops.free_irq(&ts_inst->inst, ts_inst->irq_type, ts_inst->sq_trigger_irq, (void *)ts_inst);
        trs_hw_sq_send_thread_destroy(ts_inst);

        if (ts_inst->work_queue != NULL) {
            (void)ka_task_cancel_work_sync(&ts_inst->sq_trigger_work);
            ka_task_destroy_workqueue(ts_inst->work_queue);
        }

        if (ts_inst->fair_work_queue != NULL) {
            (void)ka_task_cancel_work_sync(&ts_inst->sq_fair_work);
            ka_task_destroy_workqueue(ts_inst->fair_work_queue);
        }
    }
}

int trs_hw_sqcq_init(struct trs_core_ts_inst *ts_inst)
{
    struct trs_id_inst *inst = &ts_inst->inst;
    u32 sq_id_num = trs_res_get_id_num(ts_inst, TRS_HW_SQ);
    u32 sq_max_id = trs_res_get_max_id(ts_inst, TRS_HW_SQ);
    u32 cq_id_num = trs_res_get_id_num(ts_inst, TRS_HW_CQ);
    u32 cq_max_id = trs_res_get_max_id(ts_inst, TRS_HW_CQ);
    int ret;

    trs_debug("Init hw sqcq. (devid=%u; tsid=%u; sq_id_num=%u; sq_max_id=%u; cq_id_num=%u; cq_max_id=%u)\n",
        inst->devid, inst->tsid, sq_id_num, sq_max_id, cq_id_num, cq_max_id);

    ts_inst->sq_ctx = (struct trs_sq_ctx *)trs_vzalloc(sizeof(struct trs_sq_ctx) * sq_max_id);
    if (ts_inst->sq_ctx == NULL) {
        trs_err("Mem alloc failed. (devid=%u; tsid=%u; size=%lx)\n",
            inst->devid, inst->tsid, sizeof(struct trs_sq_ctx) * sq_max_id);
        return -ENOMEM;
    }
    ts_inst->sq_ctx->sq_num = sq_max_id;
    trs_sq_ctxs_init(ts_inst->sq_ctx, sq_max_id);

    ts_inst->cq_ctx = (struct trs_cq_ctx *)trs_vzalloc(sizeof(struct trs_cq_ctx) * cq_max_id);
    if (ts_inst->cq_ctx == NULL) {
        trs_sq_ctxs_uninit(ts_inst->sq_ctx);
        trs_vfree(ts_inst->sq_ctx);
        ts_inst->sq_ctx = NULL;
        trs_err("Mem alloc failed. (devid=%u; tsid=%u; size=%lx)\n",
            inst->devid, inst->tsid, sizeof(struct trs_cq_ctx) * cq_max_id);
        return -ENOMEM;
    }

    ret = trs_hw_sq_trigger_init(ts_inst);
    if (ret != 0) {
        trs_sq_ctxs_uninit(ts_inst->sq_ctx);
        trs_vfree(ts_inst->sq_ctx);
        ts_inst->sq_ctx = NULL;
        trs_vfree(ts_inst->cq_ctx);
        ts_inst->cq_ctx = NULL;
        return ret;
    }

    return 0;
}

void trs_hw_sqcq_uninit(struct trs_core_ts_inst *ts_inst)
{
    if (ts_inst->sq_ctx != NULL) {
        trs_sq_ctxs_uninit(ts_inst->sq_ctx);
        trs_vfree(ts_inst->sq_ctx);
        ts_inst->sq_ctx = NULL;
    }
    if (ts_inst->cq_ctx != NULL) {
        trs_vfree(ts_inst->cq_ctx);
        ts_inst->cq_ctx = NULL;
    }
}
