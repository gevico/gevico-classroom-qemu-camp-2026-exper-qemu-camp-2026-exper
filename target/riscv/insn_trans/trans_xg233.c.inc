/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
#include "qemu/osdep.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "cpu.h"
#include "translate.h"
#include "insn_macros.h"
*/
static bool trans_dma(DisasContext *ctx, arg_dma *a)
{
    /* 确保 helper 名字和你 helper.h 中定义的一致 */
    gen_helper_xg233_dma(tcg_env, cpu_gpr[a->rd], cpu_gpr[a->rs1], cpu_gpr[a->rs2]);
    return true;
}

/* 其他指令也依此类推，去掉 xg233_ 前缀 */
static bool trans_gemm(DisasContext *ctx, arg_gemm *a)
{
    return true; 
}