/*
 * RISC-V translation routines for the G233 AI custom extension.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * These instructions currently use a plain R-type encoding, but several of
 * them reinterpret the architectural rd field as an input operand carrying a
 * guest address or a scalar parameter.  The helpers below receive canonical
 * guest addresses, not host pointers.
 */

#define REQUIRE_XG233AI(ctx) do {           \
    if (!ctx->cfg_ptr->ext_xg233ai) {       \
        return false;                       \
    }                                       \
} while (0)

static bool trans_dma(DisasContext *ctx, arg_dma *a)
{
    TCGv dst, src, grain;

    REQUIRE_XG233AI(ctx);
    dst = get_address(ctx, a->rd, 0);
    src = get_address(ctx, a->rs1, 0);
    grain = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233_dma(tcg_env, dst, src, grain);
    return true;
}

static bool trans_gemm(DisasContext *ctx, arg_gemm *a)
{
    TCGv dst, lhs, rhs;

    REQUIRE_XG233AI(ctx);
    dst = get_address(ctx, a->rd, 0);
    lhs = get_address(ctx, a->rs1, 0);
    rhs = get_address(ctx, a->rs2, 0);
    gen_helper_xg233_gemm(tcg_env, dst, lhs, rhs);
    return true;
}

static bool trans_sort(DisasContext *ctx, arg_sort *a)
{
    TCGv k, base, n;

    REQUIRE_XG233AI(ctx);
    /*
     * The test ABI uses:
     *   rd  = k        (how many elements to sort from the front)
     *   rs1 = arr      (base address of the INT32 array)
     *   rs2 = n        (logical array length)
     */
    k = get_gpr(ctx, a->rd, EXT_NONE);
    base = get_address(ctx, a->rs1, 0);
    n = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233_sort(tcg_env, k, base, n);
    return true;
}

static bool trans_vadd(DisasContext *ctx, arg_vadd *a)
{
    TCGv dst, lhs, rhs;

    REQUIRE_XG233AI(ctx);
    dst = get_address(ctx, a->rd, 0);
    lhs = get_address(ctx, a->rs1, 0);
    rhs = get_address(ctx, a->rs2, 0);
    gen_helper_xg233_vadd(tcg_env, dst, lhs, rhs);
    return true;
}

static bool trans_crush(DisasContext *ctx, arg_crush *a)
{
    TCGv dst, src, n;

    REQUIRE_XG233AI(ctx);
    dst = get_address(ctx, a->rd, 0);
    src = get_address(ctx, a->rs1, 0);
    n = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233_crush(tcg_env, dst, src, n);
    return true;
}

static bool trans_expand(DisasContext *ctx, arg_expand *a)
{
    TCGv dst, src, n;

    REQUIRE_XG233AI(ctx);
    dst = get_address(ctx, a->rd, 0);
    src = get_address(ctx, a->rs1, 0);
    n = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233_expand(tcg_env, dst, src, n);
    return true;
}

static bool trans_vdot(DisasContext *ctx, arg_vdot *a)
{
    TCGv dest, lhs, rhs;

    REQUIRE_XG233AI(ctx);
    dest = dest_gpr(ctx, a->rd);
    lhs = get_address(ctx, a->rs1, 0);
    rhs = get_address(ctx, a->rs2, 0);
    gen_helper_xg233_vdot(dest, tcg_env, lhs, rhs);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_vrelu(DisasContext *ctx, arg_vrelu *a)
{
    TCGv dst, src, n;

    REQUIRE_XG233AI(ctx);
    dst = get_address(ctx, a->rd, 0);
    src = get_address(ctx, a->rs1, 0);
    n = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233_vrelu(tcg_env, dst, src, n);
    return true;
}

static bool trans_vscale(DisasContext *ctx, arg_vscale *a)
{
    TCGv dst, src, scale;

    REQUIRE_XG233AI(ctx);
    dst = get_address(ctx, a->rd, 0);
    src = get_address(ctx, a->rs1, 0);
    scale = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233_vscale(tcg_env, dst, src, scale);
    return true;
}

static bool trans_vmax(DisasContext *ctx, arg_vmax *a)
{
    TCGv dest, src, n;

    REQUIRE_XG233AI(ctx);
    dest = dest_gpr(ctx, a->rd);
    src = get_address(ctx, a->rs1, 0);
    n = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233_vmax(dest, tcg_env, src, n);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}
