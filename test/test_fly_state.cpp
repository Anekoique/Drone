#include "drone/mission/mission_types.hpp"

#include <gtest/gtest.h>

using drone::mission::fly_state_to_int;
using drone::mission::FlyState;

TEST(FlyState, EnumHasAllStates)
{
  // Verify all 9 states exist by constructing them
  FlyState states[] = {
    FlyState::init,    FlyState::takeoff,       FlyState::goto_shot,
    FlyState::airdrop, FlyState::goto_recon,    FlyState::recon_patrol,
    FlyState::landing, FlyState::land_to_start, FlyState::finished,
  };
  EXPECT_EQ(sizeof(states) / sizeof(states[0]), 9u);
}

TEST(FlyState, IntEncodingMatchesLegacy)
{
  // Legacy values from StateMachine.h fly_state_to_int
  EXPECT_EQ(fly_state_to_int(FlyState::init), 1);
  EXPECT_EQ(fly_state_to_int(FlyState::takeoff), 1);
  EXPECT_EQ(fly_state_to_int(FlyState::goto_shot), 1);
  EXPECT_EQ(fly_state_to_int(FlyState::airdrop), 0);
  EXPECT_EQ(fly_state_to_int(FlyState::goto_recon), 1);
  EXPECT_EQ(fly_state_to_int(FlyState::recon_patrol), 3);
  EXPECT_EQ(fly_state_to_int(FlyState::landing), 4);
  EXPECT_EQ(fly_state_to_int(FlyState::land_to_start), 4);
  EXPECT_EQ(fly_state_to_int(FlyState::finished), 2);
}
