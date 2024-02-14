#include <gtest/gtest.h>

namespace s21 {
TEST(GraphAlgorithmsTest, ShortestPathCase1) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_5_for_dijkstra.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 1, 4);
  EXPECT_EQ(shortestDistance, 14);
}

TEST(GraphAlgorithmsTest, ShortestPathCase2) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_8_vert_direct.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 2, 8);
  EXPECT_EQ(shortestDistance, 3);
}

TEST(GraphAlgorithmsTest, ShortestPathCase3) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_4_for_dijkstra.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 1, 3);
  EXPECT_EQ(shortestDistance, 8);
}

TEST(GraphAlgorithmsTest, ShortestPathCase4) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_4_for_dijkstra.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 4, 2);
  EXPECT_EQ(shortestDistance, 2);
}

TEST(GraphAlgorithmsTest, InvalidVertices) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_5_for_dijkstra.txt";
  graph.LoadGraphFromFile(InputFileName);

  EXPECT_THROW(algo.GetShortestPathBetweenVertices(graph, 1, 6),
               std::invalid_argument);
}

TEST(GraphAlgorithmsTest, InvalidVertices1) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_5_for_dijkstra.txt";
  graph.LoadGraphFromFile(InputFileName);

  EXPECT_THROW(algo.GetShortestPathBetweenVertices(graph, 0, 4),
               std::invalid_argument);
}

TEST(GraphAlgorithmsTest, ShortestPathCase5) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_graph_9_vert.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 1, 5);
  EXPECT_EQ(shortestDistance, 14);
}

TEST(GraphAlgorithmsTest, ShortestPathCase6) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_graph_9_vert.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 3, 7);
  EXPECT_EQ(shortestDistance, 13);
}

TEST(GraphAlgorithmsTest, ShortestPathCase7) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_directed_graph_8_vert.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 8, 5);
  EXPECT_EQ(shortestDistance, 18);
}

TEST(GraphAlgorithmsTest, ShortestPathCase8) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_5_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 1, 5);
  EXPECT_EQ(shortestDistance, 2);
}

TEST(GraphAlgorithmsTest, ShortestPathCase9) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_graph_50_vert.txt";
  graph.LoadGraphFromFile(InputFileName);

  int shortestDistance = algo.GetShortestPathBetweenVertices(graph, 33, 19);
  EXPECT_EQ(shortestDistance, 8);
}

TEST(GraphAlgorithmsTest, ShortestPathCase10) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/unconnected_graph_5.txt";
  graph.LoadGraphFromFile(InputFileName);

  EXPECT_THROW(algo.GetShortestPathBetweenVertices(graph, 1, 4),
               std::exception);
}

TEST(GraphAlgorithmsTest, InvalidVertices3) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_directed_graph_8_vert.txt";
  graph.LoadGraphFromFile(InputFileName);

  EXPECT_THROW(algo.GetShortestPathBetweenVertices(graph, 5, 8),
               std::exception);
}

TEST(GraphAlgorithmsTest, VeryBigGraph) {
  s21::s21_Graph graph;

  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/very_big_graph.txt";
  graph.LoadGraphFromFile(InputFileName);

  int start = 7, finish = 40;

  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, start, finish), 63);

  start = 1;
  finish = 50;
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, start, finish), 28);

  start = 16;
  finish = 37;
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, start, finish), 77);

  start = 42;
  finish = 36;
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, start, finish), 132);

  start = 22;
  finish = 49;
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, start, finish), 104);
}

TEST(GraphAlgorithmsTest, Deikstra_graph_with_loops) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_graph_6_vert_with_loops.txt";
  graph.LoadGraphFromFile(InputFileName);

  int start = 1, finish = 6;

  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, start, finish), 21);

  start = 4;
  finish = 6;

  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, start, finish), 16);
}

}  // namespace s21