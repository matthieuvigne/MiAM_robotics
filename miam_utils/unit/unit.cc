// Main function for unit unit tests: run all tests.

#include "gtest/gtest.h"

int main(int argc, char **argv) 
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
