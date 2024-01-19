// Copyright 2024 ROS-Industrial
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "canopen_motor_driver/canopen_motor/cia402.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace ros2_canopen;

class OperationModeMock : public OperationMode
{
public:
  friend class Canopen_402_Mode_Test;
  OperationModeMock(int8_t num, std::shared_ptr<CiA402Data> data) : OperationMode(num, data) {}
  FRIEND_TEST(Canopen_402_Mode_Test, test_set_target);
  FRIEND_TEST(Canopen_402_Mode_Test, test_start);
  FRIEND_TEST(Canopen_402_Mode_Test, test_stop);
  FRIEND_TEST(Canopen_402_Mode_Test, test_on_sync);
};

class Canopen_402_Mode_Test : public testing::Test
{
public:
  friend class OperationMode;
  void SetUp() override {}

  void TearDown() override {}
};

TEST_F(Canopen_402_Mode_Test, test_supported_modes)
{
  auto data = std::make_shared<CiA402Data>();
  auto mode = OperationMode(1, data);
  data->set_supported_modes(0x1);
  EXPECT_EQ(mode.on_boot_check(), true);
}

TEST_F(Canopen_402_Mode_Test, test_start)
{
  auto data = std::make_shared<CiA402Data>();
  auto mode = OperationModeMock(1, data);
  EXPECT_FALSE(mode.started_);
  EXPECT_EQ(mode.target_state_, OperationModeMock::TARGET_REACHED);
  EXPECT_EQ(mode.driver_, nullptr);
  EXPECT_EQ(mode.target_, 0.0);

  mode.start(nullptr);

  EXPECT_TRUE(mode.started_);
  EXPECT_EQ(mode.target_state_, OperationModeMock::TARGET_REACHED);
  EXPECT_EQ(mode.driver_, nullptr);
  EXPECT_EQ(mode.target_, 0.0);
}

TEST_F(Canopen_402_Mode_Test, test_stop)
{
  auto data = std::make_shared<CiA402Data>();
  auto mode = OperationModeMock(1, data);
  mode.start(nullptr);
  EXPECT_TRUE(mode.started_);
  EXPECT_EQ(mode.target_state_, OperationModeMock::TARGET_REACHED);
  EXPECT_EQ(mode.driver_, nullptr);
  EXPECT_EQ(mode.target_, 0.0);

  mode.stop();

  EXPECT_FALSE(mode.started_);
  EXPECT_EQ(mode.target_state_, OperationModeMock::TARGET_REACHED);
  EXPECT_EQ(mode.driver_, nullptr);
  EXPECT_EQ(mode.target_, 0.0);
}

TEST_F(Canopen_402_Mode_Test, test_set_target)
{
  auto data = std::make_shared<CiA402Data>();
  auto mode = OperationModeMock(1, data);
  mode.set_target(1.0);
  EXPECT_EQ(mode.target_, 0.0);
  mode.start(nullptr);
  mode.set_target(1.0);
  EXPECT_EQ(mode.get_target(), 1.0);
}

TEST_F(Canopen_402_Mode_Test, test_on_sync)
{
  auto data = std::make_shared<CiA402Data>();
  auto mode = OperationModeMock(1, data);
  mode.start(nullptr);
  mode.set_target(1.0);
  EXPECT_EQ(mode.target_, 1.0);
  EXPECT_EQ(mode.target_state_, OperationModeMock::NEW_TARGET);
  EXPECT_EQ(data->get_target_reached(), true);
  mode.on_sync();
  EXPECT_EQ(mode.target_state_, OperationModeMock::NEW_TARGET);
  EXPECT_EQ(data->get_target_reached(), true);

  data->set_target_reached(false);
  EXPECT_EQ(data->get_target_reached(), false);
  mode.on_sync();
  EXPECT_EQ(mode.target_state_, OperationModeMock::TARGET_ACTIVE);

  data->set_target_reached(true);
  mode.on_sync();
  EXPECT_EQ(mode.target_state_, OperationModeMock::TARGET_REACHED);
}
