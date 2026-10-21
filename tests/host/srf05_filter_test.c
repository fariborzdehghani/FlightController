#include "VelocityEstimator.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t now_ms;

static void predict_ms(uint32_t duration)
{
  for (uint32_t elapsed = 0; elapsed < duration; elapsed += 5)
  {
    now_ms += 5;
    VelocityEstimator_Predict(0.005, now_ms);
  }
}

static VelocityEstimatorSrf05Result_t sample(double altitude_m)
{
  predict_ms(55);
  return VelocityEstimator_CorrectSrf05Altitude(altitude_m, now_ms);
}

static void setup(void)
{
  now_ms = 1000;
  VelocityEstimator_Init();
  const VelocityEstimatorImuSample_t stationary = {
      .acceleration_z_ms2 = 9.80665};
  for (unsigned i = 0; i < VELOCITY_ESTIMATOR_ACCEL_BIAS_SAMPLE_COUNT; i++)
    VelocityEstimator_UpdateImu(&stationary);
  for (unsigned i = 0; i < 30; i++) sample(1.0);
  const VelocityEstimatorEstimate_t estimate = VelocityEstimator_GetEstimate();
  assert(estimate.srf05_altitude_valid && estimate.vertical_velocity_valid);
  assert(fabs(estimate.srf05_altitude_cm - 100) < 0.001);
}

static void test_false_echo_bursts(void)
{
  const double false_echoes[] = {0.08, 0.5, 1.5, 3.0, 3.95, NAN, INFINITY};
  const unsigned burst_lengths[] = {1, 2, 4, 5, 10, 100};
  for (unsigned value = 0; value < sizeof(false_echoes) / sizeof(false_echoes[0]); value++)
  {
    for (unsigned burst = 0; burst < sizeof(burst_lengths) / sizeof(burst_lengths[0]); burst++)
    {
      setup();
      for (unsigned i = 0; i < burst_lengths[burst]; i++)
      {
        assert(sample(false_echoes[value]) == VELOCITY_ESTIMATOR_SRF05_REJECTED);
        const VelocityEstimatorEstimate_t estimate = VelocityEstimator_GetEstimate();
        assert(fabs(estimate.srf05_altitude_cm - 100) < 0.001);
        if ((i + 1) * 55 > 300) assert(!estimate.srf05_altitude_valid);
      }
      for (unsigned i = 0; i < 12; i++) sample(1.0);
      assert(VelocityEstimator_GetEstimate().srf05_altitude_valid);
      assert(fabs(VelocityEstimator_GetEstimate().srf05_altitude_cm - 100) < 0.001);
    }
  }
}

static void test_dropout_does_not_accept_a_new_surface(void)
{
  setup();
  predict_ms(305);
  assert(!VelocityEstimator_GetEstimate().srf05_altitude_valid);
  VelocityEstimator_HandleSrf05Timeout();
  for (unsigned i = 0; i < 20; i++)
    assert(sample(3.0) == VELOCITY_ESTIMATOR_SRF05_REJECTED);
  assert(!VelocityEstimator_GetEstimate().srf05_altitude_valid);
  for (unsigned i = 0; i < 12; i++) sample(1.0);
  assert(VelocityEstimator_GetEstimate().srf05_altitude_valid);
}

static void test_real_climb_with_interspersed_spikes(void)
{
  setup();
  unsigned accepted = 0;
  for (unsigned i = 1; i <= 160; i++)
  {
    const double actual_m = 1.0 + 0.2 * (double)i * 0.055;
    const int spike = (i % 11) == 0;
    const VelocityEstimatorSrf05Result_t result = sample(spike ? 0.08 : actual_m);
    if (spike) assert(result == VELOCITY_ESTIMATOR_SRF05_REJECTED);
    else if (result == VELOCITY_ESTIMATOR_SRF05_ACCEPTED) accepted++;
    const VelocityEstimatorEstimate_t estimate = VelocityEstimator_GetEstimate();
    assert(estimate.srf05_altitude_valid);
    assert(fabs(estimate.srf05_altitude_cm * 0.01 - actual_m) < 0.08);
  }
  assert(accepted > 130);
}

static void test_spikes_during_acquisition(void)
{
  for (unsigned initial_samples = 3; initial_samples <= 12; initial_samples++)
  {
    setup();
    VelocityEstimator_ResetVertical();
    for (unsigned i = 0; i < initial_samples; i++) sample(1.0);
    for (unsigned i = 0; i < 12; i++)
    {
      assert(sample(1.5) == VELOCITY_ESTIMATOR_SRF05_REJECTED);
      assert(fabs(VelocityEstimator_GetEstimate().srf05_altitude_cm - 100) < 0.001);
    }
  }
}

static void test_duplicate_old_and_wrapped_timestamps(void)
{
  setup();
  for (unsigned i = 0; i < 12; i++)
  {
    assert(VelocityEstimator_CorrectSrf05Altitude(3.0, now_ms) ==
           VELOCITY_ESTIMATOR_SRF05_REJECTED);
    assert(VelocityEstimator_CorrectSrf05Altitude(3.0, now_ms - 55) ==
           VELOCITY_ESTIMATOR_SRF05_REJECTED);
  }
  assert(sample(1.0) == VELOCITY_ESTIMATOR_SRF05_ACCEPTED);

  VelocityEstimator_Init();
  now_ms = UINT32_MAX - 164u;
  for (unsigned i = 0; i < 20; i++) sample(1.0); /* A sample also lands at tick 0. */
  assert(VelocityEstimator_GetEstimate().srf05_altitude_valid);
  assert(fabs(VelocityEstimator_GetEstimate().srf05_altitude_cm - 100) < 0.001);
}

int main(void)
{
  test_false_echo_bursts();
  test_dropout_does_not_accept_a_new_surface();
  test_real_climb_with_interspersed_spikes();
  test_spikes_during_acquisition();
  test_duplicate_old_and_wrapped_timestamps();
  puts("PASS: 5 SRF05 altitude-filter regression groups");
  return 0;
}
