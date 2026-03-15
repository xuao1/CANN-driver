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

#include "ka_base_pub.h"
#include "ka_fs_pub.h"
#include "ka_task_pub.h"
#include "ka_common_pub.h"
#include "ka_memory_pub.h"
#include "ka_kernel_def_pub.h"
#include "ka_system_pub.h"

#include "ascend_hal_define.h"

#include "pbl/pbl_davinci_api.h"
#include "pbl/pbl_task_ctx.h"

#include "pbl_uda.h"
#include "dpa_kernel_interface.h"
#include "trs_id.h"
#include "trs_proc_fs.h"
#include "trs_ts_inst.h"
#include "trs_ioctl.h"
#include "trs_core.h"
#include "trs_hw_sqcq.h"
#include "trs_fops.h"
#include "trs_shr_proc.h"
#include "trs_core_adapt.h"

TRS_INIT_REBOOT_NOTIFY;

static int (*const trs_res_id_handles[TRS_MAX_CMD])(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct trs_res_id_para *para) = {
    [_IOC_NR(TRS_RES_ID_ALLOC)] = trs_res_id_alloc,
    [_IOC_NR(TRS_RES_ID_FREE)] = trs_res_id_free,
    [_IOC_NR(TRS_RES_ID_ENABLE)] = trs_res_id_enable,
    [_IOC_NR(TRS_RES_ID_DISABLE)] = trs_res_id_disable,
    [_IOC_NR(TRS_RES_ID_NUM_QUERY)] = trs_res_id_num_query,
    [_IOC_NR(TRS_RES_ID_MAX_QUERY)] = trs_res_id_max_query,
    [_IOC_NR(TRS_RES_ID_USED_NUM_QUERY)] = trs_res_id_used_query,
    [_IOC_NR(TRS_RES_ID_AVAIL_NUM_QUERY)] = trs_res_id_avail_query,
    [_IOC_NR(TRS_RES_ID_REG_OFFSET_QUERY)] = trs_res_id_reg_offset_query,
    [_IOC_NR(TRS_RES_ID_REG_SIZE_QUERY)] = trs_res_id_reg_size_query,
    [_IOC_NR(TRS_RES_ID_CFG)] = trs_res_id_cfg
};

static bool trs_is_id_query_cmd(unsigned int cmd)
{
    return ((cmd == TRS_RES_ID_NUM_QUERY) || (cmd == TRS_RES_ID_MAX_QUERY) || (cmd == TRS_RES_ID_USED_NUM_QUERY) ||
        (cmd == TRS_RES_ID_REG_OFFSET_QUERY) || (cmd == TRS_RES_ID_REG_SIZE_QUERY) ||
        (cmd == TRS_RES_ID_AVAIL_NUM_QUERY));
}

static int ioctl_trs_res_id_comm(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct trs_res_id_para *usr_para = (struct trs_res_id_para __user *)arg;
    struct trs_res_id_para para;
    int ret;

    ret = (int)ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (cmd=%d, ret=%d)\n", _IOC_NR(cmd), ret);
        return -EFAULT;
#endif
    }

    if ((para.res_type < 0) || (trs_is_id_query_cmd(cmd) && (para.res_type >= TRS_CORE_MAX_ID_TYPE)) ||
        (!trs_is_id_query_cmd(cmd) && (para.res_type >= TRS_HW_SQ))) {
        trs_err("Invalid para. (cmd=%d; res_type=%d)\n", _IOC_NR(cmd), para.res_type);
        return -EINVAL;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsid);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsid);
        return -EINVAL;
    }

    ret = trs_res_id_handles[_IOC_NR(cmd)](proc_ctx, ts_inst, &para);
    trs_core_inst_put(ts_inst);

    if (ret == 0) {
        if (cmd == TRS_RES_ID_ALLOC) {
            ret = ka_base_put_user(para.id, &usr_para->id);
            if (ret != 0) {
#ifndef EMU_ST
                goto Exit;
#endif
            }
            ret = ka_base_put_user(para.value[0], &usr_para->value[0]);
            if (ret != 0) {
#ifndef EMU_ST
                goto Exit;
#endif
            }
            ret = ka_base_put_user(para.value[1], &usr_para->value[1]); /* for id reg addr */
            if (ret != 0) {
#ifndef EMU_ST
                goto Exit;
#endif
            }
        } else if (trs_is_id_query_cmd(cmd)) {
            ret = ka_base_put_user(para.para, &usr_para->para);
        } else {
            /* do nothing */
        }
    }

Exit:
    if ((ret != 0) && (cmd != TRS_RES_ID_ALLOC)) {
        trs_err("Fail. (devid=%u; tsid=%u; cmd=%d; res_type=%d; id=%u; ret=%d)\n",
            proc_ctx->devid, para.tsid, _IOC_NR(cmd), para.res_type, para.id, ret);
    }

    return ret;
}

static int ioctl_trs_ssid_query(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct trs_ssid_query_para *usr_para = (struct trs_ssid_query_para __user *)arg;
    int user_visible_flag;
    int ret;

    if (usr_para == NULL) {
#ifndef EMU_ST
        trs_err("Invalid para, user para is NULL.\n");
        return -EINVAL;
#endif
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, 0);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u)\n", proc_ctx->devid);
        return -EINVAL;
    }

    ret = ts_inst->ops.ssid_query(&ts_inst->inst, &user_visible_flag, &proc_ctx->cp_ssid);
    trs_core_inst_put(ts_inst);
    if (ret == 0) {
        if (user_visible_flag == 1) {
            ret = ka_base_put_user(proc_ctx->cp_ssid, &usr_para->ssid);
        } else {
            ret = ka_base_put_user(proc_ctx->pid, &usr_para->ssid);
        }
    }

    return ret;
}

static int ioctl_trs_hw_info_query(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct trs_hw_info_query_para *usr_para = (struct trs_hw_info_query_para __user *)arg;
    int hw_type = TRS_HW_TYPE_STARS;
    int connection_type = TRS_CONNECT_PROTOCOL_UNKNOWN;
    enum trsSqSendMode sq_send_mode = TRS_MODE_TYPE_SQ_SEND_HIGH_PERFORMANCE;
    int tsnum = 0;
    int ret, tsid;

    if (usr_para == NULL) {
#ifndef EMU_ST
        trs_err("Invalid para, user para is NULL.\n");
        return -EINVAL;
#endif
    }

    for (tsid = 0; tsid < TRS_TS_MAX_NUM; tsid++) {
        ts_inst = trs_core_inst_get(proc_ctx->devid, (u32)tsid);
        if (ts_inst != NULL) {
            hw_type = ts_inst->hw_type;
            tsnum++;
            if (ts_inst->ops.get_connect_protocol != NULL) {
                connection_type = ts_inst->ops.get_connect_protocol(&ts_inst->inst);
            }

            if (ts_inst->ops.get_sq_send_mode != NULL) {
                sq_send_mode = ts_inst->ops.get_sq_send_mode(proc_ctx->devid);
            }
            trs_core_inst_put(ts_inst);
        }
    }
    if ((tsnum == 0) || (connection_type == TRS_CONNECT_PROTOCOL_UNKNOWN)) {
        trs_err("Invalid para. (devid=%u; tsnum=%d; connection_type=%d)\n", proc_ctx->devid, tsnum, connection_type);
        return -EINVAL;
    }

    ret = ka_base_put_user(hw_type, &usr_para->hw_type);
    ret |= ka_base_put_user(tsnum, &usr_para->tsnum);
    ret |= ka_base_put_user(connection_type, &usr_para->connection_type);
    ret |= ka_base_put_user(sq_send_mode, &usr_para->sq_send_mode);

    return ret;
}

static int (*const trs_sqcq_alloc_handles[DRV_INVALID_TYPE])(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct halSqCqInputInfo *para) = {
    [DRV_NORMAL_TYPE] = trs_hw_sqcq_alloc,
    [DRV_CALLBACK_TYPE] = trs_cb_sqcq_alloc,
    [DRV_LOGIC_TYPE] = trs_logic_cq_alloc,
    [DRV_SHM_TYPE] = trs_shm_sqcq_alloc,
    [DRV_CTRL_TYPE] = trs_sw_sqcq_alloc,
    [DRV_GDB_TYPE] = trs_gdb_sqcq_alloc
};

#define TRS_SQCQ_EXT_INFO_MAX_LEN 256
static int ioctl_trs_sqcq_alloc(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct halSqCqInputInfo para;
    struct trs_alloc_para *alloc_para = NULL;
    struct trs_uio_info *user_uio_info = NULL;
    struct trs_uio_info uio_info;
    int ret;

    ret = ka_base_copy_from_user(&para, (struct halSqCqInputInfo __user *)arg, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }

    alloc_para = get_alloc_para_addr(&para);
    user_uio_info = alloc_para->uio_info;
    ret = ka_base_copy_from_user(&uio_info, (struct trs_uio_info __user *)user_uio_info, sizeof(uio_info));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user uio info failed. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }

    if ((para.type < 0) || (para.type >= DRV_INVALID_TYPE)) {
        trs_err("Invalid value. (type=%d)\n", para.type);
        return -EINVAL;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsId);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsId);
        return -EINVAL;
    }

    ka_task_mutex_lock(&proc_ctx->ts_ctx[para.tsId].mutex);
    alloc_para->uio_info = &uio_info;
    ret = trs_sqcq_alloc_handles[para.type](proc_ctx, ts_inst, &para);
    alloc_para->uio_info = user_uio_info; /* restore use addr. otherwise, user-mode access will be abnormal. */
    ka_task_mutex_unlock(&proc_ctx->ts_ctx[para.tsId].mutex);

    trs_core_inst_put(ts_inst);

    if (ret == 0) {
        // pr_info("sqcq_alloc uio_info: sq_que_addr=0x%lx (depth=%u, sqe_size=%u, que_len=0x%lx), "
        //     "HEAD=0x%lx, TAIL=0x%lx, DB=0x%lx, "
        //     "HEAD_REG=0x%lx, TAIL_REG=0x%lx, SHR_INFO=0x%lx, "
        //     "uio_flag=%u, soft_que_flag=%u, type=%d, sqId=%u\n",
        //     uio_info.sq_que_addr,
        //     para.sqeDepth, para.sqeSize,
        //     (unsigned long)PAGE_ALIGN((unsigned long)para.sqeDepth * para.sqeSize),
        //     uio_info.sq_ctrl_addr[TRS_UIO_HEAD],
        //     uio_info.sq_ctrl_addr[TRS_UIO_TAIL],
        //     uio_info.sq_ctrl_addr[TRS_UIO_DB],
        //     uio_info.sq_ctrl_addr[TRS_UIO_HEAD_REG],
        //     uio_info.sq_ctrl_addr[TRS_UIO_TAIL_REG],
        //     uio_info.sq_ctrl_addr[TRS_UIO_SHR_INFO],
        //     uio_info.uio_flag,
        //     uio_info.soft_que_flag,
        //     para.type, para.sqId);

        ret = ka_base_copy_to_user((struct halSqCqInputInfo __user *)arg, &para, sizeof(para));
        ret |= ka_base_copy_to_user((struct trs_uio_info __user *)user_uio_info, &uio_info, sizeof(uio_info));
        if (ret != 0) {
            trs_err("Copy to user failed. (ret=%d)\n", ret);
        }
    }

    return ret;
}

static int (*const trs_sqcq_free_handles[DRV_INVALID_TYPE])(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct halSqCqFreeInfo *para) = {
    [DRV_NORMAL_TYPE] = trs_hw_sqcq_free,
    [DRV_CALLBACK_TYPE] = trs_cb_sqcq_free,
    [DRV_LOGIC_TYPE] = trs_logic_cq_free,
    [DRV_SHM_TYPE] = trs_shm_sqcq_free,
    [DRV_CTRL_TYPE] = trs_sw_sqcq_free,
    [DRV_GDB_TYPE] = trs_gdb_sqcq_free
};

static int ioctl_trs_sqcq_free(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct halSqCqFreeInfo para;
    int ret;

    ret = ka_base_copy_from_user(&para, (struct halSqCqFreeInfo __user *)arg, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }

    if ((para.type < 0) || (para.type >= DRV_INVALID_TYPE)) {
        trs_err("Invalid value. (type=%d)\n", para.type);
        return -EINVAL;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsId);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsId);
        return -EINVAL;
    }

    ka_task_mutex_lock(&proc_ctx->ts_ctx[para.tsId].mutex);
    ret = trs_sqcq_free_handles[para.type](proc_ctx, ts_inst, &para);
    ka_task_mutex_unlock(&proc_ctx->ts_ctx[para.tsId].mutex);

    trs_core_inst_put(ts_inst);
    return ret;
}

static int (*const trs_sqcq_config_handles[DRV_INVALID_TYPE])(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct halSqCqConfigInfo *para) = {
    [DRV_NORMAL_TYPE] = trs_hw_sqcq_config,
    [DRV_LOGIC_TYPE] = trs_logic_cq_config,
};

static int ioctl_trs_sqcq_config(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct halSqCqConfigInfo para;
    int ret;

    ret = ka_base_copy_from_user(&para, (struct halSqCqConfigInfo __user *)arg, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }

    if ((para.type < 0) || (para.type >= DRV_INVALID_TYPE) || (trs_sqcq_config_handles[para.type] == NULL)) {
        trs_err("Invalid value. (type=%d; prop=%d)\n", para.type, para.prop);
        return -EINVAL;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsId);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsId);
        return -EINVAL;
    }

    ret = trs_sqcq_config_handles[para.type](proc_ctx, ts_inst, &para);

    trs_core_inst_put(ts_inst);
    return ret;
}

static int ioctl_trs_sqcq_query(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct halSqCqQueryInfo *usr_para = (struct halSqCqQueryInfo __user *)arg;
    struct halSqCqQueryInfo para;
    int ret;

    ret = (int)ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return ret;
    }

    if (para.type != DRV_NORMAL_TYPE) {
        trs_err("Invalid value. (type=%d; prop=%d)\n", para.type, para.prop);
        return -EINVAL;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsId);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsId);
        return -EINVAL;
    }

    ret = trs_sqcq_query(proc_ctx, ts_inst, &para);

    trs_core_inst_put(ts_inst);

    if (ret == 0) {
        ret = (int)ka_base_put_user(para.value[0], &usr_para->value[0]);
        if (para.prop == DRV_SQCQ_PROP_SQ_REG_BASE) {
            ret |= (int)ka_base_put_user(para.value[1], &usr_para->value[1]);
            ret |= (int)ka_base_put_user(para.value[2], &usr_para->value[2]); /* 2 return sq reg size */
        }
    }

    return ret;
}

static int (*const trs_sqcq_send_handles[DRV_INVALID_TYPE])(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct halTaskSendInfo *para) = {
    [DRV_NORMAL_TYPE] = trs_hw_sqcq_send,
    [DRV_CALLBACK_TYPE] = trs_cb_sqcq_send,
    [DRV_GDB_TYPE] = trs_gdb_sqcq_send,
};

static int ioctl_trs_sqcq_send(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct halTaskSendInfo __user *usr_para = (struct halTaskSendInfo __user *)arg;
    struct halTaskSendInfo para;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }

    if ((para.type < 0) || (para.type >= DRV_INVALID_TYPE) || (trs_sqcq_send_handles[para.type] == NULL) ||
        (para.sqe_num == 0) || (para.sqe_addr == NULL)) {
        trs_err("Invalid value. (type=%d; sqe_num=%u)\n", para.type, para.sqe_num);
        return -EINVAL;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsId);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsId);
        return -EINVAL;
    }

    ret = trs_sqcq_send_handles[para.type](proc_ctx, ts_inst, &para);

    trs_core_inst_put(ts_inst);

    if ((ret == 0) && (para.type == DRV_NORMAL_TYPE)) {
        ret = ka_base_put_user(para.pos, &usr_para->pos);
        if (ret != 0) {
            trs_err("Put to user fail. (devid=%u; tsid=%u; sqId=%u)\n", proc_ctx->devid, para.tsId, para.sqId);
        }
    }

    return ret;
}

static int (*const trs_sqcq_recv_handles[DRV_INVALID_TYPE])(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct halReportRecvInfo *para) = {
    [DRV_NORMAL_TYPE] = trs_hw_sqcq_recv,
    [DRV_LOGIC_TYPE] = trs_logic_cq_recv,
    [DRV_GDB_TYPE] = trs_gdb_sqcq_recv,
};

static int ioctl_trs_sqcq_recv(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct halReportRecvInfo *usr_para = (struct halReportRecvInfo __user *)arg;
    struct halReportRecvInfo para;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }

    if ((para.type < 0) || (para.type >= DRV_INVALID_TYPE) || (trs_sqcq_recv_handles[para.type] == NULL) ||
        (para.cqe_num == 0) || (para.cqe_addr == NULL)) {
        trs_err("Invalid value. (type=%d; cqe_num=%u)\n", para.type, para.cqe_num);
        return -EINVAL;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsId);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsId);
        return -EINVAL;
    }

    ret = trs_sqcq_recv_handles[para.type](proc_ctx, ts_inst, &para);
    if (ret == 0) {
        ret = ka_base_put_user(para.report_cqe_num, &usr_para->report_cqe_num);
        if (ret != 0) {
            trs_err("Put to user fail. (devid=%u; tsid=%u; cqId=%u)\n", proc_ctx->devid, para.tsId, para.cqId);
        }
    } else {
        u32 ts_status;
        if (ts_inst->ops.get_ts_inst_status != NULL) {
            if (ts_inst->ops.get_ts_inst_status(&ts_inst->inst, &ts_status) == 0) {
                ret = (ts_status == TRS_INST_STATUS_ABNORMAL) ? -EBUSY : ret;
            }
        }
    }

    trs_core_inst_put(ts_inst);

    return ret;
}

int ioctl_trs_stl_bind(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_id_inst id_inst = {};
    struct trs_core_ts_inst *ts_inst = NULL;
    int ret;
    unsigned int i;

    id_inst.devid = proc_ctx->devid;
    for (i = 0; i < TRS_TS_MAX_NUM; i++) {
        id_inst.tsid = i;
        ts_inst = trs_core_inst_get(proc_ctx->devid, i);
        if (ts_inst == NULL) {
            continue;
        }
        if (ts_inst->ops.stl_bind == NULL) {
            trs_core_inst_put(ts_inst);
            continue;
        }
        ret = ts_inst->ops.stl_bind(&id_inst);
        trs_core_inst_put(ts_inst);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

int ioctl_trs_stl_launch(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_stl_launch_para para = {};
    struct trs_id_inst id_inst = {};
    struct trs_stl_launch_para *usr_para = (struct trs_stl_launch_para __user *)arg;
    struct trs_core_ts_inst *ts_inst = NULL;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (cmd=%d; ret=%d)\n", _IOC_NR(cmd), ret);
        return -EFAULT;
#endif
    }
    id_inst.devid = proc_ctx->devid;
    id_inst.tsid = para.tsid;

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsid);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsid);
        return -EINVAL;
    }
    if (ts_inst->ops.stl_launch == NULL) {
        trs_core_inst_put(ts_inst);
        return -EINVAL;
    }
    ret = ts_inst->ops.stl_launch(&id_inst, &para);
    trs_core_inst_put(ts_inst);
    return ret;
}

int ioctl_trs_stl_query(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_id_inst id_inst = {};
    struct trs_stl_query_para para;
    struct trs_stl_query_para *usr_para = (struct trs_stl_query_para __user *)arg;
    struct trs_core_ts_inst *ts_inst = NULL;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (cmd=%d; ret=%d)\n", _IOC_NR(cmd), ret);
        return -EFAULT;
#endif
    }
    id_inst.devid = proc_ctx->devid;
    id_inst.tsid = para.tsid;

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsid);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsid);
        return -EINVAL;
    }
    if (ts_inst->ops.stl_query == NULL) {
        trs_core_inst_put(ts_inst);
        return -EINVAL;
    }
    ret = ts_inst->ops.stl_query(&id_inst, &para);
    trs_core_inst_put(ts_inst);
    if (ret != 0) {
#ifndef EMU_ST
        return ret;
#endif
    }

    ret = ka_base_copy_to_user((void __user *)usr_para, (void *)&para, sizeof(struct trs_stl_query_para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy to user err. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }
    return 0;
}

static int trs_rpc_call_msg_fill(int pid, struct trs_rpc_call_msg *rpc_call_msg, u8 *msg, u32 msg_len)
{
    int ret;

    trs_mbox_init_header(&rpc_call_msg->header, rpc_call_msg->header.cmd_type);
    rpc_call_msg->rpc_call_header.pid = (u32)pid;
    rpc_call_msg->rpc_call_header.len = msg_len;

    ret = memcpy_s(rpc_call_msg->data, sizeof(rpc_call_msg->data), msg, msg_len);
    if (ret != 0) {
        trs_err("Memcpy failed. (dest_len=%u; src_len=%u; ret=%d)\n", (u32)sizeof(rpc_call_msg->data), msg_len, ret);
    }

    return ret;
}

int ioctl_trs_ub_info_query(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_id_inst id_inst = {0};
    struct trs_ub_info_query_para para;
    struct trs_ub_info_query_para *usr_para = (struct trs_ub_info_query_para __user *)arg;
    struct trs_core_ts_inst *ts_inst = NULL;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (cmd=%d; ret=%d)\n", _IOC_NR(cmd), ret);
        return ret;
#endif
    }

    id_inst.devid = proc_ctx->devid;
    id_inst.tsid = para.tsid;

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsid);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsid);
        return -EINVAL;
    }

    if (ts_inst->ops.ub_info_query == NULL) {
        trs_warn("Not support. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsid);
        trs_core_inst_put(ts_inst);
        return 0;
    }

    ret = ts_inst->ops.ub_info_query(&id_inst, &para.die_id, &para.func_id);
    trs_core_inst_put(ts_inst);
    if (ret != 0) {
#ifndef EMU_ST
        return ret;
#endif
    }
    trs_debug("(devid=%u; dieid=%u; funcid=%u)\n", id_inst.devid, para.die_id, para.func_id);
    ret = ka_base_copy_to_user((void __user *)usr_para, (void *)&para, sizeof(struct trs_ub_info_query_para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy to user err. (ret=%d)\n", ret);
        return ret;
#endif
    }
    return 0;
}

int trs_rpc_msg_ctrl(struct trs_id_inst *inst, int pid, void *msg, u32 msg_len, struct trs_rpc_call_msg *rpc_call_msg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    int ret;

    ret = trs_rpc_call_msg_fill(pid, rpc_call_msg, msg, msg_len);
    if (ret != 0) {
        return ret;
    }

    ts_inst = trs_core_inst_get(inst->devid, inst->tsid);
    if (ts_inst == NULL) {
        trs_err("Failed to get ts inst. (devid=%u; tsid=%u)\n", inst->devid, inst->tsid);
        return -EINVAL;
    }

    if ((ts_inst->ops.ts_rpc_call == NULL) || (ts_inst->featur_mode == TRS_INST_PART_FEATUR_MODE)) {
        trs_warn("Not support rpc call. (devid=%u; featur_mode=%u)\n", inst->devid, ts_inst->featur_mode);
        trs_core_inst_put(ts_inst);
        return -EOPNOTSUPP;
    }

    ret = ts_inst->ops.ts_rpc_call(inst, (u8 *)rpc_call_msg, sizeof(struct trs_rpc_call_msg));
    if ((ret != 0) || (rpc_call_msg->header.result != 0)) {
        trs_core_inst_put(ts_inst);
        if (ret != -EOPNOTSUPP) {
            trs_err("Notice ts and write back failed. (devid=%u; result=%u; ret=%d)\n",
                inst->devid, rpc_call_msg->header.result, ret);
        }
        return (ret != 0) ? ret : -EINVAL;
    }
    trs_core_inst_put(ts_inst);
    trs_debug("Rpc call success. (devid=%u)\n", inst->devid);
    return 0;
}
KA_EXPORT_SYMBOL_GPL(trs_rpc_msg_ctrl);

int ioctl_trs_msg_ctrl(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_id_inst id_inst = {0};
    struct trs_ctrl_msg_para para = {0};
    struct trs_ctrl_msg_para *usr_para = (struct trs_ctrl_msg_para __user *)arg;
    struct trs_rpc_call_msg rpc_call_msg;
    int ret = 0;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (cmd=%d; ret=%d)\n", _IOC_NR(cmd), ret);
        return -EFAULT;
#endif
    }

    if ((para.msg_len == 0) || (para.msg_len > TRS_CTRL_MSG_MAX_LEN)) {
        trs_err("Invalid para. (msg_len=%u, max_msg_len=%d)\n", para.msg_len, TRS_CTRL_MSG_MAX_LEN);
        return -EINVAL;
    }

    trs_id_inst_pack(&id_inst, proc_ctx->devid, para.tsid);
    rpc_call_msg.header.cmd_type = TRS_MBOX_RPC_CALL;
    ret = trs_rpc_msg_ctrl(&id_inst, proc_ctx->pid, para.msg, para.msg_len, &rpc_call_msg);
    if (ret != 0) {
        return ret;
    }

    ret = ka_base_put_user(rpc_call_msg.rpc_call_header.len, &usr_para->msg_len);
    ret |= ka_base_copy_to_user((void __user *)usr_para->msg, (void *)rpc_call_msg.data, rpc_call_msg.rpc_call_header.len);
    if (ret != 0) {
        trs_err("Copy to user err. (len=%u; ret=%d)\n", rpc_call_msg.rpc_call_header.len, ret);
    }

    return ret;
}

static int ioctl_trs_set_close_type(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_set_close_para para;
    struct trs_set_close_para *usr_para = (struct trs_set_close_para __user *)arg;
    struct trs_core_ts_inst *ts_inst = NULL;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
        trs_err("Copy from user failed. (cmd=%d; ret=%d)\n", _IOC_NR(cmd), ret);
        return ret;
    }

    if (para.close_type >= TRS_PROC_RELEASE_TYPE_MAX) {
        trs_err("Invalid para. (devid=%u; close_type=%u)\n", proc_ctx->devid, para.close_type);
        return -EINVAL;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsid);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsid);
        return -EINVAL;
    }

    if ((ts_inst->location == UDA_LOCAL) && (para.close_type == TRS_PROC_RELEASE_LOCAL)) {
        trs_core_inst_put(ts_inst);
        return -EOPNOTSUPP;
    }

    proc_ctx->release_type = para.close_type;
    trs_core_inst_put(ts_inst);
    trs_debug("Set close type. (devId=%u; close_type=%d)\n", proc_ctx->devid, para.close_type);
    return 0;
}

static int ioctl_trs_id_res_map(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_cmd_res_map *usr_para = (struct trs_cmd_res_map __user *)arg;
    struct trs_core_ts_inst *ts_inst = NULL;
    struct trs_notify_reg_map_para map_para = {0};
    struct trs_cmd_res_map para = {0};
    struct trs_id_inst id_inst = {0};
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (cmd=%d; ret=%d)\n", _IOC_NR(cmd), ret);
        return -EFAULT;
#endif
    }

    ts_inst = trs_core_inst_get_for_res_map(&id_inst, proc_ctx, para);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u)\n", proc_ctx->devid);
        return -EINVAL;
    }

    if (ts_inst->ops.notify_reg_map == NULL) {
        trs_debug("Not support. (devid=%u)\n", proc_ctx->devid);
        trs_core_inst_put(ts_inst);
        return -EINVAL;
    }

    map_para.rudevid = para.res_info.rudevid;
    map_para.res_id = para.res_info.res_id;
    map_para.flag = para.res_info.flag;
    map_para.res_type = para.res_info.res_type;
    ret = ts_inst->ops.notify_reg_map(&id_inst, &map_para);
    if (ret != 0) {
        trs_core_inst_put(ts_inst);
        return ret;
    }

    para.va = map_para.va;
    para.len = map_para.len;
    ret = ka_base_copy_to_user((void __user *)usr_para, (void *)&para, sizeof(struct trs_cmd_res_map));
    if (ret != 0) {
#ifndef EMU_ST
        if (ts_inst->ops.notify_reg_unmap != NULL) {
            (void)ts_inst->ops.notify_reg_unmap(&id_inst, &map_para);
        }
#endif
        trs_core_inst_put(ts_inst);
        trs_err("Copy to user err. (ret=%d)\n", ret);
#ifndef EMU_ST
        return -EFAULT;
#endif
    }

    trs_core_inst_put(ts_inst);
    return 0;
}

static int (*const trs_sqcq_get_handles[DRV_INVALID_TYPE])(struct trs_proc_ctx *proc_ctx,
    struct trs_core_ts_inst *ts_inst, struct halSqCqInputInfo *para) = {
    [DRV_NORMAL_TYPE] = trs_hw_sqcq_get,
    [DRV_LOGIC_TYPE] = trs_logic_cq_get
};

int ioctl_trs_sqcq_get(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct halSqCqInputInfo *usr_para = (struct halSqCqInputInfo __user *)arg;
    struct trs_core_ts_inst *ts_inst = NULL;
    struct halSqCqInputInfo para = {0};
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (cmd=%d; ret=%d)\n", _IOC_NR(cmd), ret);
        return -EFAULT;
#endif
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsId);
    if (ts_inst == NULL) {
#ifndef EMU_ST
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsId);
        return -EINVAL;
#endif
    }

    if ((para.type != DRV_NORMAL_TYPE) && (para.type != DRV_LOGIC_TYPE)) {
        trs_core_inst_put(ts_inst);
        trs_debug("Not support. (devid=%u; type=%u)\n", proc_ctx->devid, para.type);
        return -EPERM;
    }

    ka_task_mutex_lock(&proc_ctx->ts_ctx[para.tsId].mutex);
    ret = trs_sqcq_get_handles[para.type](proc_ctx, ts_inst, &para);
    ka_task_mutex_unlock(&proc_ctx->ts_ctx[para.tsId].mutex);
    trs_core_inst_put(ts_inst);

    if (ret == 0) {
        ret = ka_base_copy_to_user((void __user *)usr_para, (void *)&para, sizeof(struct halSqCqInputInfo));
        if (ret != 0) {
            trs_err("Copy to user failed. (ret=%d)\n", ret);
        }
    }

    return ret;
}

int ioctl_trs_sqcq_restore(struct trs_proc_ctx *proc_ctx, unsigned int cmd,
    unsigned long arg)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    struct trs_alloc_para *alloc_para = NULL;
    struct trs_uio_info *user_uio_info = NULL;
    struct trs_uio_info uio_info = {0};
    struct halSqCqInputInfo para = {0};
    int ret;

    ret = ka_base_copy_from_user(&para, (struct halSqCqInputInfo __user *)arg, sizeof(para));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }

    alloc_para = get_alloc_para_addr(&para);
    user_uio_info = alloc_para->uio_info;
    ret = ka_base_copy_from_user(&uio_info, (struct trs_uio_info __user *)user_uio_info, sizeof(uio_info));
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Copy from user uio info failed. (ret=%d)\n", ret);
        return -EFAULT;
#endif
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsId);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsId);
        return -EINVAL;
    }

#ifndef EMU_ST
    if (para.type != DRV_NORMAL_TYPE) {
        trs_core_inst_put(ts_inst);
        trs_warn("Not support. (devid=%u; type=%u)\n", proc_ctx->devid, para.type);
        return -EINVAL;
    }
#endif

    ka_task_mutex_lock(&proc_ctx->ts_ctx[para.tsId].mutex);
    alloc_para->uio_info = &uio_info;
    ret = trs_hw_sqcq_restore(proc_ctx, ts_inst, &para);
    alloc_para->uio_info = user_uio_info;
    ka_task_mutex_unlock(&proc_ctx->ts_ctx[para.tsId].mutex);

    trs_core_inst_put(ts_inst);

    if (ret == 0) {
        ret = ka_base_copy_to_user((struct halSqCqInputInfo __user *)arg, &para, sizeof(para));
        ret |= ka_base_copy_to_user((struct trs_uio_info __user *)user_uio_info, &uio_info, sizeof(uio_info));
        if (ret != 0) {
            trs_err("Copy to user failed. (ret=%d)\n", ret);
        }
    }

    return ret;
}

static int ioctl_trs_dma_desc_create(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_cmd_dma_desc *usr_para = (struct trs_cmd_dma_desc __user *)arg;
    struct trs_cmd_dma_desc para;
    struct trs_core_ts_inst *ts_inst = NULL;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
    }

    if (para.type != DRV_NORMAL_TYPE) {
        return -EOPNOTSUPP;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsid);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsid);
        return -EINVAL;
    }

    ka_task_mutex_lock(&proc_ctx->ts_ctx[para.tsid].mutex);
    ret = trs_hw_sqcq_dma_desc_create(proc_ctx, ts_inst, &para);
    ka_task_mutex_unlock(&proc_ctx->ts_ctx[para.tsid].mutex);
    if (ret != 0) {
        trs_core_inst_put(ts_inst);
        trs_err("Sqcq dma desc failed. (devid=%u; tsid=%u; ret=%d)\n", proc_ctx->devid, para.tsid, ret);
        return ret;
    }

    trs_core_inst_put(ts_inst);
    ret = ka_base_put_user(para.dma_base, &usr_para->dma_base);
    ret |= ka_base_put_user(para.dma_node_num, &usr_para->dma_node_num);
    return ret;
}

static int ioctl_trs_ts_cmdlist_map_unmap(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_cmd_cmdlist_map_unmap *usr_para = (struct trs_cmd_cmdlist_map_unmap __user *)(uintptr_t)arg;
    struct trs_cmd_cmdlist_map_unmap para;
    struct trs_core_ts_inst *ts_inst = NULL;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(para));
    if (ret != 0) {
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, para.tsid);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u; tsid=%u)\n", proc_ctx->devid, para.tsid);
        return -EINVAL;
    }

    if ((ts_inst->ops.ts_cmdlist_mem_map == NULL) || ts_inst->ops.ts_cmdlist_mem_unmap == NULL) {
        trs_core_inst_put(ts_inst);
        return -EOPNOTSUPP;
    }

    ka_task_mutex_lock(&proc_ctx->ts_ctx[para.tsid].mutex);
    if (para.op == 0) {
        ret = ts_inst->ops.ts_cmdlist_mem_unmap(&ts_inst->inst);
    } else {
        ret = ts_inst->ops.ts_cmdlist_mem_map(&ts_inst->inst);
    }

    if (ret != 0) {
        trs_warn("Map/Unmap warn. (devid=%u; tsid=%u; op=%u; ret=%d)\n", proc_ctx->devid, para.tsid, para.op, ret);
        ka_task_mutex_unlock(&proc_ctx->ts_ctx[para.tsid].mutex);
        trs_core_inst_put(ts_inst);
        return ret;
    }

    ka_task_mutex_unlock(&proc_ctx->ts_ctx[para.tsid].mutex);
    trs_core_inst_put(ts_inst);
    return ret;
}

#ifdef CFG_FEATURE_SUPPORT_STREAM_TASK
static int ioctl_trs_stream_task_fill(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_stream_task_para *usr_para = (struct trs_stream_task_para __user *)arg;
    struct trs_stream_task_para para;
    struct trs_core_ts_inst *ts_inst = NULL;
    void *usr_task_info = NULL;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(struct trs_stream_task_para));
    if (ret != 0) {
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
    }

    if ((para.stream_mem == NULL) || (para.task_info == NULL)) {
        trs_err("Invalid para.\n");
        return -EINVAL;
    }

    if ((para.task_cnt == 0) || (para.task_cnt > 32768)) { /* 32768: max depth is 32K */
        trs_err("Invalid task_cnt. (task_cnt=%u)\n", para.task_cnt);
        return -EINVAL;
    }

    usr_task_info = para.task_info;
    para.task_info = trs_vzalloc(para.task_cnt * TRS_HW_SQE_SIZE);
    if (para.task_info == NULL) {
        trs_err("Alloc memory failed. (size=%u)\n", para.task_cnt * TRS_HW_SQE_SIZE);
        return -ENOMEM;
    }
    ret = ka_base_copy_from_user(para.task_info, usr_task_info, para.task_cnt * TRS_HW_SQE_SIZE);
    if (ret != 0) {
        trs_err("Copy task from user failed. (ret=%d)\n", ret);
        trs_vfree(para.task_info);
        return -EFAULT;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, 0);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u)\n", proc_ctx->devid);
        trs_vfree(para.task_info);
        para.task_info = usr_task_info;
        return -EINVAL;
    }

    ka_task_mutex_lock(&proc_ctx->ts_ctx[0].mutex);
    ret = trs_stream_task_fill_proc(proc_ctx, ts_inst, &para);
    if (ret != 0) {
        trs_err("Failed to fill stream task. (ret=%d)\n", ret);
    }
    ka_task_mutex_unlock(&proc_ctx->ts_ctx[0].mutex);
    trs_core_inst_put(ts_inst);
    trs_vfree(para.task_info);
    para.task_info = usr_task_info;
    return ret;
}
#endif

#ifdef CFG_FEATURE_SUPPORT_STREAM_TASK
static int ioctl_trs_sq_switch_stream_batch(struct trs_proc_ctx *proc_ctx, unsigned int cmd, unsigned long arg)
{
    struct trs_sq_switch_stream_para *usr_para = (struct trs_sq_switch_stream_para __user *)arg;
    struct trs_sq_switch_stream_para para;
    struct sq_switch_stream_info *info = NULL;
    struct trs_core_ts_inst *ts_inst = NULL;
    unsigned long cur_jiffies = ka_jiffies;
    u32 i;
    int ret;

    ret = ka_base_copy_from_user(&para, usr_para, sizeof(struct trs_sq_switch_stream_para));
    if (ret != 0) {
        trs_err("Copy from user failed. (ret=%d)\n", ret);
        return -EFAULT;
    }

    if ((para.info == NULL) || (para.num == 0) || (para.num > 32768)) { /* 32768: max stream num */
        trs_err("Invalid para. (num=%u)\n", para.num);
        return -EINVAL;
    }

    if (!uda_is_phy_dev(proc_ctx->devid)) {
        return -ENOTSUPP;
    }

    info = trs_vzalloc(sizeof(struct sq_switch_stream_info) * para.num);
    if (info == NULL) {
        trs_err("Alloc memory failed. (num=%u)\n", para.num);
        return -ENOMEM;
    }

    ret = ka_base_copy_from_user(info, para.info, sizeof(struct sq_switch_stream_info) * para.num);
    if (ret != 0) {
        trs_err("Copy from user failed. (ret=%d; num=%u)\n", ret, para.num);
        trs_vfree(info);
        return -EFAULT;
    }

    ts_inst = trs_core_inst_get(proc_ctx->devid, 0);
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u)\n", proc_ctx->devid);
        trs_vfree(info);
        return -EINVAL;
    }

    ka_task_mutex_lock(&proc_ctx->ts_ctx[0].mutex);
    for (i = 0; i < para.num; i++) {
        ret = trs_sq_switch_stream(proc_ctx, ts_inst, &info[i]);
        if (ret != 0) {
            ka_task_mutex_unlock(&proc_ctx->ts_ctx[0].mutex);
            trs_err("Failed to switch stream. (ret=%d; devid=%u; num=%u; i=%u)\n", ret, proc_ctx->devid, para.num, i);
            trs_core_inst_put(ts_inst);
            trs_vfree(info);
            return ret;
        }
        trs_try_resched(&cur_jiffies, 1000); /* timeout is 1000 ms */
    }
    ka_task_mutex_unlock(&proc_ctx->ts_ctx[0].mutex);
    trs_core_inst_put(ts_inst);
    trs_vfree(info);
    return 0;
}
#endif

static int (*const trs_ioctl_handles[TRS_MAX_CMD])(struct trs_proc_ctx *ctx, unsigned int cmd, unsigned long arg) = {
    [_IOC_NR(TRS_RES_ID_ALLOC)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_FREE)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_ENABLE)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_DISABLE)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_NUM_QUERY)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_MAX_QUERY)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_USED_NUM_QUERY)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_AVAIL_NUM_QUERY)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_REG_OFFSET_QUERY)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_REG_SIZE_QUERY)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_RES_ID_CFG)] = ioctl_trs_res_id_comm,
    [_IOC_NR(TRS_SSID_QUERY)] = ioctl_trs_ssid_query,
    [_IOC_NR(TRS_HW_INFO_QUERY)] = ioctl_trs_hw_info_query,
    [_IOC_NR(TRS_SQCQ_ALLOC)] = ioctl_trs_sqcq_alloc,
    [_IOC_NR(TRS_SQCQ_FREE)] = ioctl_trs_sqcq_free,
    [_IOC_NR(TRS_SQCQ_CONFIG)] = ioctl_trs_sqcq_config,
    [_IOC_NR(TRS_SQCQ_QUERY)] = ioctl_trs_sqcq_query,
    [_IOC_NR(TRS_SQCQ_SEND)] = ioctl_trs_sqcq_send,
    [_IOC_NR(TRS_SQCQ_RECV)] = ioctl_trs_sqcq_recv,
    [_IOC_NR(TRS_STL_BIND)] = ioctl_trs_stl_bind,
    [_IOC_NR(TRS_STL_LAUNCH)] = ioctl_trs_stl_launch,
    [_IOC_NR(TRS_STL_QUERY)] = ioctl_trs_stl_query,
    [_IOC_NR(TRS_MSG_CTRL)] = ioctl_trs_msg_ctrl,
    [_IOC_NR(TRS_ID_RES_MAP)] = ioctl_trs_id_res_map,
    [_IOC_NR(TRS_UB_INFO_QUERY)] = ioctl_trs_ub_info_query,
    [_IOC_NR(TRS_SET_CLOSE_TYPE)] = ioctl_trs_set_close_type,
    [_IOC_NR(TRS_ID_SQCQ_GET)] = ioctl_trs_sqcq_get,
    [_IOC_NR(TRS_ID_SQCQ_RESTORE)] = ioctl_trs_sqcq_restore,
    [_IOC_NR(TRS_DMA_DESC_CREATE)] = ioctl_trs_dma_desc_create,
    [_IOC_NR(TRS_TS_CMDLIST_MAP_UNMAP)] = ioctl_trs_ts_cmdlist_map_unmap,
#ifdef CFG_FEATURE_SUPPORT_STREAM_TASK
    [_IOC_NR(TRS_STREAM_TASK_FILL)] = ioctl_trs_stream_task_fill,
    [_IOC_NR(TRS_SQ_SWITCH_STREAM)] = ioctl_trs_sq_switch_stream_batch,
#endif
};

static int trs_ioctl_cmd_is_support(u32 devid, int cmd)
{
    struct trs_core_ts_inst *ts_inst = NULL;

    ts_inst = trs_core_inst_get(devid, 0);
    if (ts_inst == NULL) {
#ifndef EMU_ST
        trs_debug("Invalid para. (devid=%u)\n", devid);
#endif
        return -ENODEV;
    }

    if ((ts_inst->featur_mode == TRS_INST_PART_FEATUR_MODE) && (!trs_is_id_query_cmd(cmd))) {
        trs_core_inst_put(ts_inst);
        trs_debug("Unsupported command. (devid=%u; cmd=%u)\n", devid, _IOC_NR(cmd));
        return -EPERM;
    }

    trs_core_inst_put(ts_inst);
    return 0;
}

static long trs_ioctl(ka_file_t *file, unsigned int cmd, unsigned long arg)
{
    struct trs_task_info_struct *task_info = ka_fs_get_file_private_data(file);
    struct trs_proc_ctx *proc_ctx = NULL;
    int cmd_nr = _IOC_NR(cmd);
    int ret;

    /* pr_info("==========> [DEBUG] trs_ioctl is CALLED! cmd: %u, file: %p, arg: %lx <==========\n", cmd, file, arg); */

    if ((cmd_nr < 0) || (cmd_nr >= TRS_MAX_CMD) || (trs_ioctl_handles[cmd_nr] == NULL)) {
        trs_err("Unsupported command. (cmd_nr=%d)\n", cmd_nr);
        return -EOPNOTSUPP;
    }

    if ((void *)arg == NULL) {
#ifndef EMU_ST
        trs_err("Invalid para. (cmd_nr=%d)\n", cmd_nr);
#endif
        return -EINVAL;
    }

    if (task_info == NULL) {
        trs_err("Invalid task info. (cmd_nr=%d)\n", cmd_nr);
        return -EINVAL;
    }

    proc_ctx = task_info->proc_ctx;
    if ((proc_ctx == NULL) || (proc_ctx->status != TRS_PROC_STATUS_NORMAL)) {
        trs_err("Proc is exiting. (cmd_nr=%d; status=%d)\n", cmd_nr, (proc_ctx == NULL) ? -1 : proc_ctx->status);
        return -ESRCH;
    }

    if (trs_proc_support_cmd_check(task_info, cmd) == false) {
        return -EACCES;
    }

    ret = trs_ioctl_cmd_is_support(proc_ctx->devid, (int)cmd);
    if (ret != 0) {
        if (proc_ctx->cmd_support_dfx_times <= TRS_PROC_DFX_TIMES_MAX) {
            proc_ctx->cmd_support_dfx_times++;
            trs_err("Unsupported command. (devid=%u; cmd_nr=%d; ret=%d)\n", proc_ctx->devid, cmd_nr, ret);
        }

        return ret;
    }

    // pr_info("[DEBUG] trs_ioctl dispatch. cmd_nr=%d, handler=%ps\n", cmd_nr, trs_ioctl_handles[cmd_nr]);
    return (long)trs_ioctl_handles[cmd_nr](proc_ctx, cmd, arg);
}

static int trs_ts_inst_open(struct trs_proc_ctx *proc_ctx)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    u32 devid = proc_ctx->devid;
    u32 tsid;
    int i;

    for (tsid = 0; tsid < TRS_TS_MAX_NUM; tsid++) {
        ts_inst = proc_ctx->ts_ctx[tsid].ts_inst;
        if (ts_inst == NULL) {
            continue;
        }

        /* When core ts inst is obtained, the module reference counting of ops must be added. */
        if (!try_module_get(ts_inst->ops.owner)) {
            trs_err("Get trs core module failed. (devid=%u)\n", devid);
            goto exit;
        }

        if (ts_inst->ops.proc_bind_smmu != NULL) {
            void *smmu_inst = ts_inst->ops.proc_bind_smmu(&ts_inst->inst);
            if (smmu_inst == NULL) {
#ifndef EMU_ST
                ka_system_module_put(ts_inst->ops.owner);
                trs_err("Proc bind smmu failed. (devid=%u; tsid=%u)\n", devid, tsid);
                goto exit;
#endif
            }
            proc_ctx->ts_ctx[tsid].smmu_inst = smmu_inst;
        }

        proc_ctx->mm = ka_task_get_current_mm();
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
        ka_mm_mmget(proc_ctx->mm);
#else
        ka_base_atomic_inc(&proc_ctx->mm->mm_users);
#endif

        trs_debug("Ts inst open success. (devid=%u; tsid=%u)\n", devid, tsid);
    }

    return 0;

exit:
#ifndef EMU_ST
    for (i = (int)(tsid - 1); i >= 0; i--) {
        ts_inst = proc_ctx->ts_ctx[tsid].ts_inst;
        if (ts_inst == NULL) {
            continue;
        }

        if (proc_ctx->mm != NULL) {
            ka_mm_mmput(proc_ctx->mm);
        }
        if (ts_inst->ops.proc_unbind_smmu != NULL) {
            ts_inst->ops.proc_unbind_smmu(proc_ctx->ts_ctx[tsid].smmu_inst, proc_ctx->pid);
        }
        ka_system_module_put(ts_inst->ops.owner);
    }
#endif
    return -EINVAL;
}

static int trs_open(ka_inode_t *inode, ka_file_t *file)
{
    struct trs_core_ts_inst *all_ts_inst[TRS_TS_MAX_NUM] = {0};
    struct trs_task_info_struct *task_info = NULL;
    struct trs_core_ts_inst *ts_inst = NULL;
    struct trs_proc_ctx *proc_ctx = NULL;
    u32 devid, tsid;
    int ret;

    devid = drv_davinci_get_device_id(file);
    ka_fs_set_file_private_data(file, NULL);

    if (davinci_intf_confirm_user() == false) {
        trs_err("The user is not allowed to open. (devid=%u)\n", devid);
        return -ENODEV;
    }

    if (uda_can_access_udevid(devid) == false) {
        trs_err("Device is not in container. (devid=%u)\n", devid);
        return -EINVAL;
    }

    for (tsid = 0; tsid < TRS_TS_MAX_NUM; tsid++) {
        all_ts_inst[tsid] = trs_core_inst_get(devid, tsid);
    }

    ts_inst = all_ts_inst[0];
    if (ts_inst == NULL) {
        trs_err("Invalid para. (devid=%u)\n", devid);
        ret = -EINVAL;
        goto exit;
    }

    /*
     * When a process is reclaiming resources,
     * Prevent starting processes with the same pid.
     */
    ret = trs_proc_wait_for_exit(ts_inst, ka_task_get_current_tgid());
    if (ret != 0) {
        trs_warn("Wait for proc exit timeout. (devid=%u; pid=%d; ret=%d)\n", devid, ka_task_get_current_tgid(), ret);
        goto exit;
    }

    task_info = trs_kzalloc(sizeof(struct trs_task_info_struct), KA_GFP_KERNEL | __KA_GFP_ACCOUNT);
    if (task_info == NULL) {
        trs_err("Mem alloc failed. (size=%lx)\n", sizeof(struct trs_task_info_struct));
        goto exit;
    }

    if (trs_proc_cp2_type_check(ts_inst) == true) {
#ifndef EMU_ST
        ret = trs_shr_proc_open(file, ts_inst, task_info);
        if (ret != 0) {
            goto task_info_destroy;
        }

        return 0;
#endif
    }

    ka_task_down_write(&ts_inst->sem);
    proc_ctx = trs_proc_ctx_find(ts_inst, ka_task_get_current_tgid());
    if (proc_ctx != NULL) {
        trs_err("Repeat init. (devid=%u; pid=%d)\n", devid, ka_task_get_current_tgid());
        ret = -EEXIST;
        goto error;
    }

    if (ka_base_atomic_read(&ts_inst->ctx_num) >= TRS_PROC_CTX_MAX_NUM) {
#ifndef EMU_ST
        trs_warn("Ctx is exceeding the max num.(devid=%u; pid=%d)\n", devid, ka_task_get_current_tgid());
        ret = -EMFILE;
        goto error;
#endif
    }

    proc_ctx = trs_proc_ctx_create(all_ts_inst, TRS_TS_MAX_NUM);
    if (proc_ctx == NULL) {
        trs_err("Proc ctx create failed. (devid=%u)\n", devid);
        ret = -ENOMEM;
        goto error;
    }

    ret = trs_ts_inst_open(proc_ctx);
    if (ret != 0) {
#ifndef EMU_ST
        trs_err("Ts inst open failed. (devid=%u; pid=%d; ret=%d)\n", devid, ka_task_get_current_tgid(), ret);
        goto ctx_destroy;
#endif
    }

    ka_list_add_tail(&proc_ctx->node, &ts_inst->proc_list_head);
    ka_base_atomic_inc(&ts_inst->ctx_num);
    ka_task_up_write(&ts_inst->sem);

    task_info->proc_ctx = proc_ctx;
    task_info->unique_id = proc_ctx->task_id;

    ka_task_down_write(&ts_inst->ctrl_sem); /* wait exited app release finish when tsfw busy */
    ka_fs_set_file_private_data(file, task_info);
    ka_task_up_write(&ts_inst->ctrl_sem);

    trs_debug("Proc open success. (devid=%u; task_info_unique_id=%lld; proc_ctx_taskid=%lld; cp2_taskid=%lld)\n",
        devid, task_info->unique_id, proc_ctx->task_id, proc_ctx->cp2_task_id);
    return 0;

ctx_destroy:
    trs_proc_ctx_put(proc_ctx);
error:

    ka_task_up_write(&ts_inst->sem);
task_info_destroy:
    trs_kfree(task_info);
exit:
    for (tsid = 0; tsid < TRS_TS_MAX_NUM; tsid++) {
        if (all_ts_inst[tsid] != NULL) {  /* only ts0 is valid */
            trs_core_inst_put(all_ts_inst[tsid]);
        }
    }

    return ret;
}

static void trs_try_flush_id_to_pool(struct trs_proc_ctx *proc_ctx)
{
    if (!trs_still_has_proc(proc_ctx->ts_ctx[0].ts_inst, 0)) {
        u32 tsid;
        for (tsid = 0; tsid < TRS_TS_MAX_NUM; tsid++) {
            struct trs_core_ts_inst *ts_inst = proc_ctx->ts_ctx[tsid].ts_inst;
            if (ts_inst != NULL) {
                trs_debug("Flush id to pool. (devid=%u; tsid=%u)\n", ts_inst->inst.devid, ts_inst->inst.tsid);
                (void)trs_id_flush_to_pool(&ts_inst->inst);
            }
        }
    }
}

static void trs_core_dev_inst_put(struct trs_proc_ctx *proc_ctx)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    u32 tsid;

    for (tsid = 0; tsid < TRS_TS_MAX_NUM; tsid++) {
        ts_inst = proc_ctx->ts_ctx[tsid].ts_inst;
        if (ts_inst != NULL) {
            if (proc_ctx->mm != NULL) {
                ka_mm_mmput(proc_ctx->mm);
            }
            ka_system_module_put(ts_inst->ops.owner);
            trs_core_inst_put(ts_inst); /* get has been called in open */
        }
    }
}

static int trs_mmap_not_support(ka_file_t *file, ka_vm_area_struct_t *vma)
{
    return -ENOTSUPP;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)
static int trs_mremap(ka_vm_area_struct_t * area)
{
    return -ENOTSUPP;
}
#endif

static struct vm_operations_struct trs_vm_ops = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)
    .mremap = trs_mremap,
#endif
};

static int trs_mmap(ka_file_t *file, ka_vm_area_struct_t *vma)
{
    vma->vm_ops = &trs_vm_ops;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
    vm_flags_set(vma, vma->vm_flags | VM_LOCKED | VM_DONTEXPAND | VM_DONTDUMP | VM_DONTCOPY | VM_IO);
    if (!(vma->vm_flags & VM_WRITE)) {
        vm_flags_clear(vma, VM_MAYWRITE);
    }
#else
    vma->vm_flags |= VM_DONTEXPAND;
    vma->vm_flags |= VM_LOCKED;
    vma->vm_flags |= VM_PFNMAP;
    vma->vm_flags |= VM_DONTDUMP;
    vma->vm_flags |= VM_DONTCOPY;
    vma->vm_flags |= VM_IO;
    if (!(vma->vm_flags & VM_WRITE)) {
        vma->vm_flags &= ~VM_MAYWRITE;
    }
#endif
    vma->vm_private_data = ka_fs_get_file_private_data(file);

    return 0;
}

static int trs_release(ka_inode_t *inode, ka_file_t *file)
{
    return 0;
}

static ka_file_operations_t trs_fops = {
    .owner = KA_THIS_MODULE,
    .open = trs_open,
    .release = trs_release,
    .unlocked_ioctl = trs_ioctl,
    .mmap = trs_mmap_not_support,
};

static int trs_mmap_open(ka_inode_t *inode, ka_file_t *file)
{
    struct trs_core_ts_inst *ts_inst = NULL;
    u32 devid = drv_davinci_get_device_id(file);
    struct trs_proc_ctx *proc_ctx = NULL;
    u32 pid = ka_task_get_current_tgid();
    u32 share_pid = 0;
    int ret;

    if (uda_can_access_udevid(devid) == false) {
#ifndef EMU_ST
        trs_err("Device can not access. (devid=%u)\n", devid);
        return -EINVAL;
#endif
    }

    ts_inst = trs_core_inst_get(devid, 0);
    if (ts_inst == NULL) {
#ifndef EMU_ST
        trs_err("Ts inst not found. (devid=%u)\n", devid);
        return -ENODEV;
#endif
    }

    if (trs_proc_cp2_type_check(ts_inst) == true) {
#ifndef EMU_ST
        ret = trs_shr_proc_get_share_pid(ts_inst, &share_pid);
        if (ret == 0) {
            pid = share_pid;
        }
#endif
    }

    trs_debug("Map open. (devid=%u; cur_pid=%u; share_pid=%u)\n", devid, ka_task_get_current_tgid(), pid);

    ka_task_down_write(&ts_inst->sem);
    proc_ctx = trs_proc_ctx_find(ts_inst, pid);
    ka_fs_set_file_private_data(file, proc_ctx);
    ka_task_up_write(&ts_inst->sem);

    trs_core_inst_put(ts_inst);

    if (ka_fs_get_file_private_data(file) == NULL) {
#ifndef EMU_ST
        trs_err("Proc ctx not found. (devid=%u; pid=%d)\n", devid, pid);
        return -ESRCH;
#endif
    }

    return 0;
}

static int trs_mmap_release(ka_inode_t *inode, ka_file_t *file)
{
    ka_fs_set_file_private_data(file, NULL);
    return 0;
}

static ka_file_operations_t trs_mmap_fops = {
    .owner = KA_THIS_MODULE,
    .open = trs_mmap_open,
    .release = trs_mmap_release,
    .mmap = trs_mmap,
};

void trs_handle_proc_release_result(struct trs_core_ts_inst *ts_inst, struct trs_proc_ctx *proc_ctx,
    int exit_stage, bool is_success)
{
    if (is_success) {
        if (exit_stage == APM_STAGE_RECYCLE_RES) {
            ka_task_down_write(&ts_inst->sem);
            ka_list_del(&proc_ctx->node);
            ka_base_atomic_dec(&ts_inst->ctx_num);
            if ((ts_inst->location == UDA_LOCAL) || (proc_ctx->release_type == TRS_PROC_RELEASE_LOCAL_REMOTE)) {
                trs_try_flush_id_to_pool(proc_ctx);
            }
            ka_task_up_write(&ts_inst->sem);
            trs_core_dev_inst_put(proc_ctx);
            trs_debug("Proc destroy. (devid=%u; pid=%d; task_id=%lld)\n",
                proc_ctx->devid, proc_ctx->pid, proc_ctx->task_id);
            trs_proc_ctx_put(proc_ctx);
        }
    } else {
        trs_warn("Release warn, add to exit_list. (devid=%u; pid=%d; task_id=%lld; stage=%u)\n",
            proc_ctx->devid, proc_ctx->pid, proc_ctx->task_id, exit_stage);
        ka_task_down_write(&ts_inst->sem);
        ka_list_move(&proc_ctx->node, &ts_inst->exit_proc_list_head);
        ka_base_atomic_dec(&ts_inst->ctx_num);
        ka_task_up_write(&ts_inst->sem);
        trs_core_dev_inst_put(proc_ctx);
    }
}

static int _trs_release_by_exit_stage(struct trs_proc_ctx *proc_ctx, int stage)
{
    int ret = 0;
    int result = 0;
    u32 tsid;

    for (tsid = 0; tsid < TRS_TS_MAX_NUM; tsid++) {
        struct trs_core_ts_inst *ts_inst = proc_ctx->ts_ctx[tsid].ts_inst;
        if (ts_inst == NULL) {
            continue;
        }

        if (stage == APM_STAGE_DO_EXIT) {
            if (ts_inst->ops.proc_unbind_smmu != NULL) {
                ts_inst->ops.proc_unbind_smmu(proc_ctx->ts_ctx[tsid].smmu_inst, proc_ctx->pid);
            }

            trs_id_clear_reserved_flag(&ts_inst->inst, proc_ctx->pid);
            trs_proc_release(ts_inst, proc_ctx);
        } else if (stage == APM_STAGE_STOP_STREAM) {
            if ((ts_inst->location == UDA_NEAR) && (proc_ctx->release_type == TRS_PROC_RELEASE_LOCAL)) {
                return 0;
            }

            trs_proc_release_notice_ts(ts_inst, proc_ctx);
            if (trs_proc_is_res_leak(proc_ctx, tsid) == false) {
                continue;
            }

            ret = trs_proc_release_check_ts(proc_ctx, ts_inst->inst.tsid);
            if (ret != 0) {
                trs_warn("Recycle warn. (devid=%u; tsid=%u; name=%s; pid=%d; task_id=%lld; ret=%d)\n",
                    ts_inst->inst.devid, ts_inst->inst.tsid, proc_ctx->name, proc_ctx->pid, proc_ctx->task_id, ret);
                trs_proc_leak_res_show(proc_ctx, ts_inst);
                trs_proc_release_ras_report(proc_ctx, ts_inst);
                ret = -EBUSY;
            }
        } else if (((stage == APM_STAGE_PRE_RECYCLE_RES) && (ts_inst->location != UDA_LOCAL)) ||
                   ((stage == APM_STAGE_RECYCLE_RES) && (ts_inst->location == UDA_LOCAL))) {
            if ((ts_inst->location == UDA_NEAR) && (proc_ctx->release_type == TRS_PROC_RELEASE_LOCAL)) {
                return 0;
            }
            ret = trs_proc_recycle(ts_inst, proc_ctx);
        } else if ((stage == APM_STAGE_RECYCLE_RES) && (ts_inst->location != UDA_LOCAL)) {
            if (ts_inst->ops.notice_proc_release != NULL) {
                (void)ts_inst->ops.notice_proc_release(&ts_inst->inst, proc_ctx->pid);
            }
        }

        result |= ret;
    }

    return result;
}

#ifdef CFG_FEATURE_SUPPORT_APM
static bool trs_notify_apm_release_task(struct trs_proc_ctx *proc_ctx)
{
    TRS_PROC_RELEASE_FLAG release_flag;

    if (ka_base_atomic_read(&proc_ctx->release_flag) == TRS_PROC_RELEASE_BY_TSDRV_FD) {
        return false;
    }

    if (task_is_exit(proc_ctx->pid, &proc_ctx->start_time)) {
        /* notify apm handle release */
        if (apm_notify_task_exit(proc_ctx->pid, &proc_ctx->start_time)) {
            return true;
        }
    }

    /* mutex tsdrv fd release and apm release */
    release_flag = ka_base_atomic_cmpxchg(&proc_ctx->release_flag, TRS_PROC_RELEASE_FLAG_NONE, TRS_PROC_RELEASE_BY_TSDRV_FD);
    if ((release_flag == TRS_PROC_RELEASE_BY_APM_MASTER) || (release_flag == TRS_PROC_RELEASE_BY_APM_SLAVE)) {
        return true;
    }

    return false;
}

static int trs_release_by_exit_stage(struct notifier_block *self, unsigned long val, void *data,
    TRS_PROC_RELEASE_FLAG flag)
{
    int tgid = apm_get_exit_tgid(val);
    int stage = apm_get_exit_stage(val);
    int ret, i;

    for (i = 0; i < TRS_DEV_MAX_NUM; i++) {
        struct trs_proc_ctx *proc_ctx = NULL;
        struct trs_id_inst inst = {i, 0};
        struct trs_core_ts_inst *ts_inst = NULL;
        TRS_PROC_RELEASE_FLAG proc_release_flag;

        ts_inst = trs_core_ts_inst_get(&inst);
        if (ts_inst == NULL) {
            continue;
        }

        ka_task_down_write(&ts_inst->sem);
        proc_ctx = trs_proc_ctx_find(ts_inst, tgid);
        if (proc_ctx == NULL) {
            goto release_next;
        }

        if (task_is_exit(proc_ctx->pid, &proc_ctx->start_time) == false) {
            (void)ka_base_atomic_cmpxchg(&proc_ctx->release_flag, TRS_PROC_RELEASE_FLAG_NONE, TRS_PROC_RELEASE_BY_TSDRV_FD);
            goto release_next;
        }

        /* mutex tsdrv fd release and apm release, or apm multi release */
        proc_release_flag = ka_base_atomic_cmpxchg(&proc_ctx->release_flag, TRS_PROC_RELEASE_FLAG_NONE, flag);
        if ((proc_release_flag == TRS_PROC_RELEASE_BY_TSDRV_FD) ||
            ((stage == APM_STAGE_DO_EXIT) && (proc_release_flag != TRS_PROC_RELEASE_FLAG_NONE)) ||
            ((stage > APM_STAGE_DO_EXIT) && (proc_release_flag != flag))) {
            goto release_next;
        }

        /* only one same pid proc_ctx in list */
        ka_task_up_write(&ts_inst->sem);
        ret = _trs_release_by_exit_stage(proc_ctx, stage);
        trs_handle_proc_release_result(ts_inst, proc_ctx, stage, (ret == 0));

        trs_core_ts_inst_put(ts_inst);
        trs_info("Stage release. (devid=%d; stage=%d; pid=%d)\n", i, stage, tgid);
        continue;

release_next:
        ka_task_up_write(&ts_inst->sem);
        trs_core_ts_inst_put(ts_inst);
    }

    return NOTIFY_OK;
}

static int trs_master_release_by_exit_stage(struct notifier_block *self, unsigned long val, void *data)
{
    return trs_release_by_exit_stage(self, val, data, TRS_PROC_RELEASE_BY_APM_MASTER);
}

static int trs_slave_release_by_exit_stage(struct notifier_block *self, unsigned long val, void *data)
{
    return trs_release_by_exit_stage(self, val, data, TRS_PROC_RELEASE_BY_APM_SLAVE);
}

static struct notifier_block trs_master_task_exit_nb = {
    .notifier_call = trs_master_release_by_exit_stage,
    .priority = APM_EXIT_NOTIFIY_PRI_TSDRV,
};

static struct notifier_block trs_slave_task_exit_nb = {
    .notifier_call = trs_slave_release_by_exit_stage,
    .priority = APM_EXIT_NOTIFIY_PRI_TSDRV,
};
#endif

static int trs_ts_inst_release(struct trs_proc_ctx *proc_ctx)
{
    struct trs_core_ts_inst *ts_inst = proc_ctx->ts_ctx[0].ts_inst;
    int stage, ret;

    stage = APM_STAGE_DO_EXIT;
    ret = _trs_release_by_exit_stage(proc_ctx, stage);
    if (ret != 0) {
        goto handle_result;
    }

    stage = APM_STAGE_STOP_STREAM;
    ret = _trs_release_by_exit_stage(proc_ctx, stage);
    if (ret != 0) {
        goto handle_result;
    }

    stage = APM_STAGE_PRE_RECYCLE_RES;
    ret = _trs_release_by_exit_stage(proc_ctx, stage);
    if (ret != 0) {
        goto handle_result;
    }

    stage = APM_STAGE_RECYCLE_RES;
    ret = _trs_release_by_exit_stage(proc_ctx, stage);
    if (ret != 0) {
        goto handle_result;
    }

handle_result:
    trs_handle_proc_release_result(ts_inst, proc_ctx, stage, (ret == 0));
    return ret;
}

static int trs_release_prepare(ka_file_t *file, unsigned long mode)
{
    struct trs_task_info_struct *task_info = ka_fs_get_file_private_data(file);
    struct trs_proc_ctx *proc_ctx = NULL;

    if (task_info == NULL) {
        return 0;
    }

    ka_fs_set_file_private_data(file, NULL);;

    proc_ctx = task_info->proc_ctx;
    if (proc_ctx == NULL) { /* ioctl open fail */
        trs_kfree(task_info);
        return 0;
    }

    trs_debug("Task info. (devid=%u; task_info_unique_id=%lld; proc_ctx_taskid=%lld; cp2_taskid=%lld)\n",
        proc_ctx->devid, task_info->unique_id, proc_ctx->task_id, proc_ctx->cp2_task_id);
    if (trs_shr_proc_check(task_info) == true) {
        trs_debug("Shr proc exit. (cur_pid=%u; proc_ctx_pid=%u)\n", ka_task_get_current_tgid(), proc_ctx->pid);
        trs_shr_proc_close(task_info);
        trs_kfree(task_info);
        return 0;
    } else {
        trs_debug("Set proc ctx exit. (cur_pid=%u; proc_ctx_pid=%u)\n", ka_task_get_current_tgid(), proc_ctx->pid);
        ka_task_write_lock_bh(&proc_ctx->ctx_rwlock);
        proc_ctx->status = TRS_PROC_STATUS_EXIT;
        ka_task_write_unlock_bh(&proc_ctx->ctx_rwlock);
        trs_kfree(task_info);
    }

#ifdef CFG_FEATURE_SUPPORT_APM
    if (trs_notify_apm_release_task(proc_ctx) == true) {
        trs_info("Apm release. (devid=%u; pid=%d)\n", proc_ctx->devid, proc_ctx->pid);
        return 0;
    }
#endif

    trs_debug("Proc release. (devid=%u; pid=%d; task_id=%lld; close_type=%d)\n",
        proc_ctx->devid, proc_ctx->pid, proc_ctx->task_id, proc_ctx->release_type);
    return trs_ts_inst_release(proc_ctx);
}

const struct notifier_operations trs_notifier_ops = {
    .notifier_call =  trs_release_prepare,
};

int trs_core_init_module(void)
{
    int ret;

    ret = drv_davinci_register_sub_parallel_module(TRS_MODULE_NAME, &trs_fops);
    if (ret != 0) {
        trs_err("Register sub module fail. (ret=%d)\n", ret);
        return ret;
    }

    ret = drv_ascend_register_notify(TRS_MODULE_NAME, &trs_notifier_ops);
    if (ret != 0) {
        (void)drv_ascend_unregister_sub_module(TRS_MODULE_NAME);
        trs_err("Register notify fail. (ret=%d)\n", ret);
        return ret;
    }

    ret = drv_davinci_register_sub_parallel_module(DAVINCI_INTF_MODULE_TSDRV_MMAP_NAME, &trs_mmap_fops);
    if (ret != 0) {
        (void)drv_ascend_unregister_notify(TRS_MODULE_NAME);
        (void)drv_ascend_unregister_sub_module(TRS_MODULE_NAME);
        trs_err("Register sub module fail. (ret=%d)\n", ret);
        return ret;
    }

#ifdef CFG_FEATURE_SUPPORT_APM
    ret = apm_task_exit_register(&trs_master_task_exit_nb, &trs_slave_task_exit_nb);
    if (ret != 0) {
	    (void)drv_ascend_unregister_sub_module(DAVINCI_INTF_MODULE_TSDRV_MMAP_NAME);
        (void)drv_ascend_unregister_notify(TRS_MODULE_NAME);
        (void)drv_ascend_unregister_sub_module(TRS_MODULE_NAME);
        trs_err("Register apm task release notify fail. (ret=%d)\n", ret);
        return ret;
    }
#endif

    trs_proc_fs_init();
    trs_core_inst_init();
    TRS_REGISTER_REBOOT_NOTIFY;

    return 0;
}

void trs_core_exit_module(void)
{
    TRS_UNREGISTER_REBOOT_NOTIFY;
    trs_core_inst_uninit();
    trs_proc_fs_uninit();
#ifdef CFG_FEATURE_SUPPORT_APM
    apm_task_exit_unregister(&trs_master_task_exit_nb, &trs_slave_task_exit_nb);
#endif
    (void)drv_ascend_unregister_sub_module(DAVINCI_INTF_MODULE_TSDRV_MMAP_NAME);
    (void)drv_ascend_unregister_notify(TRS_MODULE_NAME);
    (void)drv_ascend_unregister_sub_module(TRS_MODULE_NAME);
}
