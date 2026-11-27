// Host tests for lib/controls/xy_pid.c. Not part of the firmware build. From the repo root:
//
//   cc -std=c11 -Wall -Wextra -Wpedantic -Werror -Iulysses/include
//      ulysses/tests/host/xy_pid_test.c ulysses/lib/controls/xy_pid.c -lm -o xy_pid_test
//   ./xy_pid_test

#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/controls/xy_pid.h"

static int failures;
static int checks;
static jmp_buf assert_jump;
static int expect_assert;

// Replaces the weak default handler, which spins forever
void ulysses_assert_handler(const char *file, uint64_t line, const char *message)
{
    if (expect_assert) {
        longjmp(assert_jump, 1);
    }
    fprintf(stderr, "unexpected assert at %s:%llu: %s\n", file, (unsigned long long)line, message);
    exit(2);
}

#define CHECK(cond)                                                         \
    do {                                                                    \
        checks++;                                                           \
        if (!(cond)) {                                                      \
            failures++;                                                     \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                   \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((a) - (b)) <= (tol))

#define CHECK_ASSERTS(stmt)              \
    do {                                 \
        expect_assert = 1;               \
        int fired = setjmp(assert_jump); \
        if (!fired) {                    \
            stmt;                        \
        }                                \
        expect_assert = 0;               \
        CHECK(fired);                    \
    } while (0)

static xy_pid_gains_t gains_pid(float kp, float ki, float kd, float out_limit)
{
    xy_pid_gains_t g = {
        .x = {.kp = kp, .ki = ki, .kd = kd, .integral_limit = 10.0f, .output_limit = out_limit},
        .y = {.kp = kp, .ki = ki, .kd = kd, .integral_limit = 10.0f, .output_limit = out_limit},
    };
    return g;
}

static void test_proportional_and_derivative(void)
{
    xy_pid_state_t s;
    xy_pid_reset(&s);
    xy_pid_gains_t g = gains_pid(2.0f, 0.0f, 0.5f, INFINITY);
    g.y.kp = 3.0f;

    float out[2];
    unsigned sat =
        xy_pid_step(&s, &g, (const float[]){0.1f, -0.2f}, (const float[]){1.0f, 2.0f}, 0.01f, out);
    CHECK(sat == 0u);
    CHECK_NEAR(out[XY_PID_X], 2.0f * 0.1f + 0.5f * 1.0f, 1e-6f);
    CHECK_NEAR(out[XY_PID_Y], 3.0f * -0.2f + 0.5f * 2.0f, 1e-6f);
}

static void test_integral_and_integral_limit(void)
{
    xy_pid_state_t s;
    xy_pid_reset(&s);
    xy_pid_gains_t g = gains_pid(0.0f, 1.0f, 0.0f, INFINITY);
    g.x.integral_limit = 0.05f;

    float out[2];
    for (int k = 0; k < 10; k++) {
        xy_pid_step(&s, &g, (const float[]){1.0f, 1.0f}, (const float[]){0.0f, 0.0f}, 0.01f, out);
    }
    CHECK_NEAR(s.integral[XY_PID_X], 0.05f, 1e-6f); // clamped
    CHECK_NEAR(s.integral[XY_PID_Y], 0.10f, 1e-6f); // 10 steps of 1 * 0.01
    CHECK_NEAR(out[XY_PID_Y], 0.10f, 1e-6f);
}

static void test_output_clamp_and_flags(void)
{
    xy_pid_state_t s;
    xy_pid_reset(&s);
    const float limit = 0.3142f; // 18 degrees
    xy_pid_gains_t g = gains_pid(10.0f, 0.0f, 0.0f, limit);

    float out[2];
    unsigned sat =
        xy_pid_step(&s, &g, (const float[]){1.0f, -0.01f}, (const float[]){0.0f, 0.0f}, 0.01f, out);
    CHECK(sat == XY_PID_SATURATED_X);
    CHECK_NEAR(out[XY_PID_X], limit, 1e-6f);
    CHECK_NEAR(out[XY_PID_Y], -0.1f, 1e-6f);

    sat =
        xy_pid_step(&s, &g, (const float[]){-1.0f, -1.0f}, (const float[]){0.0f, 0.0f}, 0.01f, out);
    CHECK(sat == (XY_PID_SATURATED_X | XY_PID_SATURATED_Y));
    CHECK_NEAR(out[XY_PID_X], -limit, 1e-6f);
}

static void test_anti_windup_holds_integrator(void)
{
    xy_pid_state_t s;
    xy_pid_reset(&s);
    xy_pid_gains_t g = gains_pid(1.0f, 1.0f, 0.0f, 0.5f);

    float out[2];
    // Large positive error: saturated, so the integrator must not grow
    for (int k = 0; k < 100; k++) {
        xy_pid_step(&s, &g, (const float[]){2.0f, 0.0f}, (const float[]){0.0f, 0.0f}, 0.01f, out);
    }
    CHECK_NEAR(s.integral[XY_PID_X], 0.0f, 1e-6f);
    CHECK_NEAR(out[XY_PID_X], 0.5f, 1e-6f);

    // Error reverses: the output leaves the limit at once instead of waiting for the integrator
    xy_pid_step(&s, &g, (const float[]){-0.1f, 0.0f}, (const float[]){0.0f, 0.0f}, 0.01f, out);
    CHECK(out[XY_PID_X] < 0.0f);
}

static void test_parallel_matches_diagonal_coupled(void)
{
    xy_pid_state_t a;
    xy_pid_state_t b;
    xy_pid_reset(&a);
    xy_pid_reset(&b);
    xy_pid_gains_t g = {
        .x = {.kp = 1.2f, .ki = 0.3f, .kd = 0.1f, .integral_limit = 0.2f, .output_limit = 0.3f},
        .y = {.kp = 0.8f, .ki = 0.5f, .kd = 0.2f, .integral_limit = 0.1f, .output_limit = 0.25f},
    };
    xy_pid_coupled_gains_t c = {
        .kp = {{1.2f, 0.0f}, {0.0f, 0.8f}},
        .ki = {{0.3f, 0.0f}, {0.0f, 0.5f}},
        .kd = {{0.1f, 0.0f}, {0.0f, 0.2f}},
        .integral_limit = {0.2f, 0.1f},
        .output_limit = {0.3f, 0.25f},
    };

    for (int k = 0; k < 200; k++) {
        float e[2] = {0.4f * sinf(0.05f * (float)k), 0.3f * cosf(0.07f * (float)k)};
        float r[2] = {0.02f * cosf(0.05f * (float)k), -0.021f * sinf(0.07f * (float)k)};
        float oa[2];
        float ob[2];
        unsigned sa = xy_pid_step(&a, &g, e, r, 0.01f, oa);
        unsigned sb = xy_pid_coupled_step(&b, &c, e, r, 0.01f, ob);
        CHECK(sa == sb);
        CHECK_NEAR(oa[XY_PID_X], ob[XY_PID_X], 1e-6f);
        CHECK_NEAR(oa[XY_PID_Y], ob[XY_PID_Y], 1e-6f);
    }
}

static void test_coupled_cross_term(void)
{
    xy_pid_state_t s;
    xy_pid_reset(&s);
    xy_pid_coupled_gains_t c = {
        .kp = {{1.0f, 0.5f}, {-0.25f, 1.0f}},
        .integral_limit = {1.0f, 1.0f},
        .output_limit = {INFINITY, INFINITY},
    };

    float out[2];
    xy_pid_coupled_step(&s, &c, (const float[]){0.0f, 0.2f}, (const float[]){0.0f, 0.0f}, 0.01f,
                        out);
    CHECK_NEAR(out[XY_PID_X], 0.1f, 1e-6f); // only the y error, through kp[0][1]
    CHECK_NEAR(out[XY_PID_Y], 0.2f, 1e-6f);
}

static void test_bad_arguments_assert(void)
{
    xy_pid_state_t s;
    xy_pid_reset(&s);
    xy_pid_gains_t g = gains_pid(1.0f, 0.0f, 0.0f, 1.0f);
    float out[2];
    const float zero[2] = {0.0f, 0.0f};

    CHECK_ASSERTS(xy_pid_step(&s, &g, zero, zero, 0.0f, out));
    CHECK_ASSERTS(xy_pid_step(&s, &g, zero, zero, -0.01f, out));
    CHECK_ASSERTS(xy_pid_step(&s, &g, zero, zero, NAN, out));

    g.x.output_limit = 0.0f;
    CHECK_ASSERTS(xy_pid_step(&s, &g, zero, zero, 0.01f, out));
}

// A fake black box: records what the loops read and write
typedef struct fake_vehicle {
    float tilt[2];
    float rate[2];
    float command[2];
    int reads;
} fake_vehicle_t;

static void fake_get_tilt(void *ctx, float measurement[2])
{
    fake_vehicle_t *v = ctx;
    measurement[XY_PID_X] = v->tilt[XY_PID_X];
    measurement[XY_PID_Y] = v->tilt[XY_PID_Y];
    v->reads++;
}

static void fake_get_rate(void *ctx, float rate[2])
{
    fake_vehicle_t *v = ctx;
    rate[XY_PID_X] = v->rate[XY_PID_X];
    rate[XY_PID_Y] = v->rate[XY_PID_Y];
}

static void fake_set_gimbal(void *ctx, const float command[2])
{
    fake_vehicle_t *v = ctx;
    v->command[XY_PID_X] = command[XY_PID_X];
    v->command[XY_PID_Y] = command[XY_PID_Y];
}

static void test_run_through_black_box(void)
{
    fake_vehicle_t v = {.tilt = {0.1f, -0.05f}, .rate = {0.2f, 0.0f}};
    xy_pid_io_t io = {
        .ctx = &v,
        .get_measurement = fake_get_tilt,
        .get_measurement_rate = fake_get_rate,
        .set_command = fake_set_gimbal,
    };
    xy_pid_state_t s;
    xy_pid_reset(&s);
    xy_pid_gains_t g = gains_pid(2.0f, 0.0f, 0.5f, 0.3142f);

    xy_pid_run(&s, &g, &io, (const float[]){0.0f, 0.0f}, 0.01f);
    CHECK(v.reads == 1);
    // error = 0 - tilt, error rate = -rate
    CHECK_NEAR(v.command[XY_PID_X], 2.0f * -0.1f + 0.5f * -0.2f, 1e-6f);
    CHECK_NEAR(v.command[XY_PID_Y], 2.0f * 0.05f, 1e-6f);
}

// Closed loop on a small-angle tilt model, J * alpha'' = L * T * theta, per axis.
// The plant numbers are placeholders, not CAD values (issue #13).
static void test_closed_loop_settles_upright(void)
{
    const float J = 0.05f; // kg m^2, placeholder
    const float L = 0.3f;  // m, placeholder
    const float T = 20.0f; // N, placeholder
    const float dt = 0.002f;
    const float limit = 0.3142f;

    fake_vehicle_t v = {.tilt = {0.15f, -0.1f}};
    xy_pid_io_t io = {
        .ctx = &v,
        .get_measurement = fake_get_tilt,
        .get_measurement_rate = fake_get_rate,
        .set_command = fake_set_gimbal,
    };
    xy_pid_state_t s;
    xy_pid_reset(&s);
    xy_pid_gains_t g = gains_pid(1.0f, 0.0f, 0.13f, limit); // about 11 rad/s, damping 0.7

    float peak_command = 0.0f;
    for (int k = 0; k < (int)(3.0f / dt); k++) {
        xy_pid_run(&s, &g, &io, (const float[]){0.0f, 0.0f}, dt);
        for (int i = 0; i < 2; i++) {
            float accel = L * T * v.command[i] / J;
            v.rate[i] += accel * dt;
            v.tilt[i] += v.rate[i] * dt;
            peak_command = fmaxf(peak_command, fabsf(v.command[i]));
        }
    }
    CHECK(fabsf(v.tilt[XY_PID_X]) < 1e-3f);
    CHECK(fabsf(v.tilt[XY_PID_Y]) < 1e-3f);
    CHECK(peak_command <= limit + 1e-6f);
    printf("closed loop: tilt after 3 s = (%.2e, %.2e) rad, peak gimbal = %.3f rad\n",
           (double)v.tilt[XY_PID_X], (double)v.tilt[XY_PID_Y], (double)peak_command);
}

int main(void)
{
    test_proportional_and_derivative();
    test_integral_and_integral_limit();
    test_output_clamp_and_flags();
    test_anti_windup_holds_integrator();
    test_parallel_matches_diagonal_coupled();
    test_coupled_cross_term();
    test_bad_arguments_assert();
    test_run_through_black_box();
    test_closed_loop_settles_upright();

    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
