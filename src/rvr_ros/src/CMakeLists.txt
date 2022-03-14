add_library(foraging MODULE foraging.h foraging.cpp)
target_link_libraries(foraging
  argos3core_simulator
  argos3plugin_simulator_rvr)