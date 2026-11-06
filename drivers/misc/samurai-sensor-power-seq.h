/* SPDX-License-Identifier: GPL-2.0-only */
enum samurai_sensor_rail { SAMURAI_VDD, SAMURAI_VDDIO };
struct samurai_power_state { bool vdd_on, io_on; };
struct samurai_power_ops {
    int (*enable)(void *ctx, enum samurai_sensor_rail rail);
    int (*disable)(void *ctx, enum samurai_sensor_rail rail);
    void (*delay)(void *ctx);
};

static int samurai_power_on(struct samurai_power_state *s,
                            const struct samurai_power_ops *ops, void *ctx)
{
    int ret;
    if (!s->vdd_on) {
        ret = ops->enable(ctx, SAMURAI_VDD);
        if (ret) return ret;
        s->vdd_on = true;
    }
    if (!s->io_on) {
        ops->delay(ctx);
        ret = ops->enable(ctx, SAMURAI_VDDIO);
        if (ret) {
            if (!ops->disable(ctx, SAMURAI_VDD)) s->vdd_on = false;
            return ret;
        }
        s->io_on = true;
    }
    return 0;
}

static int samurai_power_off(struct samurai_power_state *s,
                             const struct samurai_power_ops *ops, void *ctx)
{
    int ret;
    if (s->io_on) {
        ret = ops->disable(ctx, SAMURAI_VDDIO);
        if (ret) return ret;
        s->io_on = false;
    }
    if (s->vdd_on) {
        ret = ops->disable(ctx, SAMURAI_VDD);
        if (ret) return ret;
        s->vdd_on = false;
    }
    return 0;
}
