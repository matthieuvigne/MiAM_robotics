#include <common/macros.hpp>
#include <module/test_bench.hpp>

int main(int argc, char* argv[])
{
  LOG("Initializing the test bench");
  module::test::initializeTestBench();
  LOG("Test bench initialized");
  
  return EXIT_SUCCESS;
}
