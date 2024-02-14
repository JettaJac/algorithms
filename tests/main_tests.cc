#include <gtest/gtest.h>

#include "../s21_algorithms/s21_graph_algorithms.cc"
#include "../s21_graph/s21_graph.cc"
#include "ant_tests.cc"
#include "bfs_tests.cc"
#include "deikstra_tests.cc"
#include "dfs_tests.cc"
#include "floid_uolsher_tests.cc"
#include "graph_class_tests.cc"
#include "prim_tests.cc"

namespace s21 {

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
}  // namespace s21
