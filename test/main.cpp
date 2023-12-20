// Copyright (c) 2023 808X_Group6
/**
 * @file main.cpp
 * @brief Entry point for running Google Test unit tests.
 */

#include <gtest/gtest.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
