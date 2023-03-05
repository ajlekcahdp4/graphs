set (GRAPHS_TEST_SOURCES
  src/main.cc
  src/dag.cc
  src/directed_graph.cc
)

add_executable(graphs_unit_test ${GRAPHS_TEST_SOURCES})
target_link_libraries(graphs_unit_test ${GTEST_BOTH_LIBRARIES})
target_include_directories(graphs_unit_test PUBLIC ${GRAPHS_LIB_DIR})
gtest_discover_tests (graphs_unit_test)