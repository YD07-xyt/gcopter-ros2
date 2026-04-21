include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(nanogrid_tests
  GridMapTest.cpp
  GridMapMathTest.cpp
  EigenPluginsTest.cpp
  ModernApiTest.cpp
  test_helpers.cpp
)

target_link_libraries(nanogrid_tests PRIVATE
  nanoGrid::nanoGrid
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(nanogrid_tests)
