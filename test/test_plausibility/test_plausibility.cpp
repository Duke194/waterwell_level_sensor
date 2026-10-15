#include "plausibility.h"

#include <cstdio>

namespace
{
int g_failed = 0;
const char *g_test = "";

#define CHECK(cond)                                                                     \
  do                                                                                    \
  {                                                                                     \
    if (!(cond))                                                                        \
    {                                                                                   \
      std::fprintf(stderr, "FAIL %s: %s (%s:%d)\n", g_test, #cond, __FILE__, __LINE__); \
      ++g_failed;                                                                       \
    }                                                                                   \
  } while (0)

PlausibilityDecision step(PlausibilityState &state, int candidateCm)
{
  return plausibilityDecide(state, candidateCm, PLAUSIBILITY_MAX_STEP_CM, PLAUSIBILITY_OUTLIER_DELTA_CM,
                            PLAUSIBILITY_CONFIRM_COUNT);
}

void test_first_sample_is_accepted()
{
  PlausibilityState state{0, 0, 0};
  const PlausibilityDecision decision = step(state, 25);
  CHECK(!decision.held);
  CHECK(decision.distanceCm == 25);
  CHECK(state.acceptedCm == 25);
  CHECK(state.pendingStreak == 0);
}

void test_heavy_rain_steps_track_immediately()
{
  PlausibilityState state{0, 0, 0};
  const int samples[] = {25, 37, 52, 62, 76, 90, 78, 64};
  for (int sample : samples)
  {
    const PlausibilityDecision decision = step(state, sample);
    CHECK(!decision.held);
    CHECK(decision.distanceCm == sample);
    CHECK(state.acceptedCm == sample);
    CHECK(state.pendingStreak == 0);
  }
}

void test_exact_max_step_is_accepted()
{
  PlausibilityState state{25, 0, 0};
  const PlausibilityDecision up = step(state, 45);
  CHECK(!up.held);
  CHECK(up.distanceCm == 45);

  const PlausibilityDecision down = step(state, 25);
  CHECK(!down.held);
  CHECK(down.distanceCm == 25);
}

void test_stable_jump_accepts_on_third_burst()
{
  PlausibilityState state{25, 0, 0};
  const int bursts[] = {58, 58, 57};

  const PlausibilityDecision first = step(state, bursts[0]);
  CHECK(first.held);
  CHECK(first.distanceCm == 25);
  CHECK(first.pendingCm == 58);
  CHECK(first.pendingStreak == 1);

  const PlausibilityDecision second = step(state, bursts[1]);
  CHECK(second.held);
  CHECK(second.distanceCm == 25);
  CHECK(second.pendingStreak == 2);

  const PlausibilityDecision third = step(state, bursts[2]);
  CHECK(!third.held);
  CHECK(third.distanceCm == 57);
  CHECK(state.acceptedCm == 57);
  CHECK(state.pendingStreak == 0);
}

void test_mixed_outliers_never_move_the_anchor()
{
  PlausibilityState state{25, 0, 0};
  const int bursts[] = {136, 163, 146, 165, 58, 163, 136, 146, 56, 165, 136, 58};
  for (int sample : bursts)
  {
    const PlausibilityDecision decision = step(state, sample);
    CHECK(decision.held);
    CHECK(decision.distanceCm == 25);
    CHECK(state.acceptedCm == 25);
  }
}

void test_alternating_levels_do_not_confirm()
{
  PlausibilityState state{25, 0, 0};
  for (int i = 0; i < 20; ++i)
  {
    const int sample = (i % 2) == 0 ? 58 : 146;
    const PlausibilityDecision decision = step(state, sample);
    CHECK(decision.held);
    CHECK(decision.distanceCm == 25);
    CHECK(state.acceptedCm == 25);
    CHECK(decision.pendingStreak == 1);
  }
}

void test_walking_cluster_does_not_confirm()
{
  PlausibilityState state{25, 0, 0};
  const PlausibilityDecision first = step(state, 58);
  CHECK(first.held);
  CHECK(first.pendingCm == 58);

  const PlausibilityDecision second = step(state, 64);
  CHECK(second.held);
  CHECK(second.pendingCm == 58);
  CHECK(second.pendingStreak == 2);

  const PlausibilityDecision third = step(state, 70);
  CHECK(third.held);
  CHECK(third.distanceCm == 25);
  CHECK(third.pendingCm == 70);
  CHECK(third.pendingStreak == 1);
  CHECK(state.acceptedCm == 25);
}

void test_small_step_clears_a_pending_jump()
{
  PlausibilityState state{25, 0, 0};
  const PlausibilityDecision held = step(state, 58);
  CHECK(held.held);

  const PlausibilityDecision accepted = step(state, 40);
  CHECK(!accepted.held);
  CHECK(accepted.distanceCm == 40);
  CHECK(state.pendingStreak == 0);

  const PlausibilityDecision again = step(state, 70);
  CHECK(again.held);
  CHECK(again.distanceCm == 40);
  CHECK(again.pendingStreak == 1);
}

void test_many_identical_outliers_still_need_three()
{
  PlausibilityState state{25, 0, 0};
  for (int i = 0; i < 2; ++i)
  {
    const PlausibilityDecision decision = step(state, 136);
    CHECK(decision.held);
    CHECK(decision.distanceCm == 25);
  }
  const PlausibilityDecision decision = step(state, 136);
  CHECK(!decision.held);
  CHECK(decision.distanceCm == 136);
  CHECK(state.acceptedCm == 136);
}

void test_garbage_streak_does_not_accept_immediately()
{
  PlausibilityState state{25, 999, 200};
  const PlausibilityDecision decision = step(state, 136);
  CHECK(decision.held);
  CHECK(decision.distanceCm == 25);
  CHECK(decision.pendingCm == 136);
  CHECK(decision.pendingStreak == 1);
}
} // namespace

int main()
{
#define RUN(fn)   \
  do              \
  {               \
    g_test = #fn; \
    fn();         \
  } while (0)

  RUN(test_first_sample_is_accepted);
  RUN(test_heavy_rain_steps_track_immediately);
  RUN(test_exact_max_step_is_accepted);
  RUN(test_stable_jump_accepts_on_third_burst);
  RUN(test_mixed_outliers_never_move_the_anchor);
  RUN(test_alternating_levels_do_not_confirm);
  RUN(test_walking_cluster_does_not_confirm);
  RUN(test_small_step_clears_a_pending_jump);
  RUN(test_many_identical_outliers_still_need_three);
  RUN(test_garbage_streak_does_not_accept_immediately);

#undef RUN

  if (g_failed != 0)
  {
    std::fprintf(stderr, "%d check(s) failed\n", g_failed);
    return 1;
  }
  std::printf("plausibility ok\n");
  return 0;
}
