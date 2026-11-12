#include <math.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/controls/pid.h"
#include "ulysses/lib/math/quaternion.h"

#include "uart.h"

#define TOLERANCE 1e-5f
#define PI_F 3.14159265358979f

static unsigned failures;

static void report(const char *name, int passed)
{
    uart_puts("ULYSSES-SIM: ");
    uart_puts(name);
    uart_puts(passed ? " PASS\n" : " FAIL\n");

    if (!passed) {
        failures++;
    }
}

static int near(float actual, float expected)
{
    return fabsf(actual - expected) <= TOLERANCE;
}

void ulysses_assert_handler(const char *file, uint64_t line, const char *message)
{
    (void)line;

    // Fail loudly, since an assert in the code under test is a test failure
    uart_puts("ULYSSES-SIM: FAIL assert ");
    uart_puts(file);
    uart_puts(": ");
    uart_puts(message);
    uart_puts("\n");

    while (1) {
    }
}

static void test_pid_proportional(void)
{
    pid_state_t state;
    pid_reset(&state);

    float out = pid_step(&state, 2.0f, 0.0f, 0.0f, 1.0f, 3.0f, 0.0f, 0.01f);
    report("pid proportional", near(out, 6.0f));
}

static void test_pid_integral(void)
{
    pid_state_t state;
    pid_reset(&state);

    float out = 0.0f;
    for (int i = 0; i < 5; i++) {
        out = pid_step(&state, 0.0f, 1.0f, 0.0f, 10.0f, 1.0f, 0.0f, 0.1f);
    }
    // Integral is 5 * (1 * 0.1) = 0.5 and Ki = 1
    report("pid integral", near(out, 0.5f));
}

static void test_pid_integral_clamp(void)
{
    pid_state_t state;
    pid_reset(&state);

    float out = 0.0f;
    for (int i = 0; i < 100; i++) {
        out = pid_step(&state, 0.0f, 1.0f, 0.0f, 0.2f, 1.0f, 0.0f, 0.1f);
    }
    report("pid integral clamp", near(out, 0.2f) && near(state.integral, 0.2f));
}

static void test_pid_derivative(void)
{
    pid_state_t state;
    pid_reset(&state);

    float out = pid_step(&state, 0.0f, 0.0f, 0.5f, 1.0f, 0.0f, -4.0f, 0.01f);
    report("pid derivative", near(out, -2.0f));
}

static void test_pid3_cross_axis(void)
{
    // An error on axis 0 drives the output on axis 1 only
    const float kp[3][3] = {{0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    const float zero[3][3] = {{0.0f}};
    const float error[3] = {1.5f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 0.0f};
    float out[3];

    pid3_state_t state;
    pid3_reset(&state);
    pid3_step(&state, kp, zero, zero, 1.0f, error, rate, 0.01f, out);

    report("pid3 cross axis", near(out[0], 0.0f) && near(out[1], 3.0f) && near(out[2], 0.0f));
}

static void test_quaternion_normalize(void)
{
    quaternion_t q = {.q = {2.0f, 0.0f, 0.0f, 0.0f}};
    q = quaternion_normalize(q);
    report("quaternion normalize", near(q.w, 1.0f) && near(quaternion_norm(q), 1.0f));
}

static void test_quaternion_conjugate_inverse(void)
{
    // 90 degrees about z, unit length
    const float h = sqrtf(0.5f);
    quaternion_t q = {.q = {h, 0.0f, 0.0f, h}};
    quaternion_t id = quaternion_multiply(q, quaternion_conjugate(q));

    report("quaternion conjugate is inverse",
           near(id.q[0], 1.0f) && near(id.q[1], 0.0f) && near(id.q[2], 0.0f) && near(id.q[3], 0.0f));
}

static void test_quaternion_rotation_vector(void)
{
    // 90 degrees about z is the rotation vector (0, 0, pi / 2)
    const float h = sqrtf(0.5f);
    quaternion_t q = {.q = {h, 0.0f, 0.0f, h}};
    float phi[3];
    quaternion_to_rotation_vector(q, phi);

    report("quaternion rotation vector",
           near(phi[0], 0.0f) && near(phi[1], 0.0f) && near(phi[2], PI_F / 2.0f));
}

static void test_quaternion_shortest_path(void)
{
    // -q is the same rotation as q, so the rotation vector must not change
    const float h = sqrtf(0.5f);
    quaternion_t q = {.q = {-h, 0.0f, 0.0f, -h}};
    float phi[3];
    quaternion_to_rotation_vector(q, phi);

    report("quaternion shortest path",
           near(phi[0], 0.0f) && near(phi[1], 0.0f) && near(phi[2], PI_F / 2.0f));
}

int main(void)
{
    uart_init();
    uart_puts("ULYSSES-SIM: controls tests start\n");

    test_pid_proportional();
    test_pid_integral();
    test_pid_integral_clamp();
    test_pid_derivative();
    test_pid3_cross_axis();
    test_quaternion_normalize();
    test_quaternion_conjugate_inverse();
    test_quaternion_rotation_vector();
    test_quaternion_shortest_path();

    if (failures == 0u) {
        uart_puts("ULYSSES-SIM: ALL PASS\n");
    } else {
        uart_puts("ULYSSES-SIM: FAIL some tests failed\n");
    }

    // Park so the simulator runs until its timeout instead of executing garbage
    while (1) {
    }
}
