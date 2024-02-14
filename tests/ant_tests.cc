#include <gtest/gtest.h>

namespace s21 {

TEST(AntTest, weighted_graph_4_vert_1) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;

  std::string InputFileName = "./examples/weighted_graph_4_vert.txt";
  graph.LoadGraphFromFile(InputFileName);
  int distance = 10;
  TsmResult result_struct = algo.SolveTravelingSalesmanProblem(graph);
  const int size = graph.get_graph_size();
  std::vector<int> previer_path{1, 2, 4, 3, 1};

  ASSERT_EQ(12, result_struct.distance);
}

TEST(AntTest, weighted_graph_11_vert) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;

  std::string InputFileName = "./examples/weighted_graph_11_vert.txt";
  graph.LoadGraphFromFile(InputFileName);
  int distance = 10;
  TsmResult result_struct = algo.SolveTravelingSalesmanProblem(graph);
  const int size = graph.get_graph_size();
  ASSERT_EQ(true, abs(result_struct.distance - 253) < 10);
}

}  // namespace s21