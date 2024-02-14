#include <gtest/gtest.h>

namespace s21 {
TEST(GraphAlgorithmsTest, test_BFS_simple_graph_3_check_all_vert_and_throw) {
  s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_3_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  int start = 1;
  std::vector<int> visited_vertices;
  std::vector<int> result;
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {1, 2, 3};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }

  start = 2;
  visited_vertices.clear();
  result.clear();
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {2, 1, 3};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }

  start = 3;
  visited_vertices.clear();
  result.clear();
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {3, 1, 2};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }

  visited_vertices.clear();
  start = -1;
  ASSERT_ANY_THROW(visited_vertices = algo.BreadthFirstSearch(graph, start));

  visited_vertices.clear();
  start = 0;
  ASSERT_ANY_THROW(visited_vertices = algo.BreadthFirstSearch(graph, start));

  visited_vertices.clear();
  start = 4;
  ASSERT_ANY_THROW(visited_vertices = algo.BreadthFirstSearch(graph, start));
}

TEST(GraphAlgorithmsTest, test_BFS_simple_graph_13_check_verts_and_throw) {
  s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_13_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);

  int start = 1;
  std::vector<int> visited_vertices;
  std::vector<int> result;
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {1, 2, 9, 11, 13, 4, 10, 12, 8, 7, 5, 6, 3};

  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }

  start = 9;
  visited_vertices.clear();
  result.clear();
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {9, 1, 8, 13, 2, 11, 7, 5, 6, 10, 12, 4, 3};

  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }

  visited_vertices.clear();
  start = -10;
  ASSERT_ANY_THROW(visited_vertices = algo.BreadthFirstSearch(graph, start));

  visited_vertices.clear();
  start = 7777;
  ASSERT_ANY_THROW(visited_vertices = algo.BreadthFirstSearch(graph, start));
}

TEST(GraphAlgorithmsTest, test_BFS_directed_graph_8_check_vert) {
  s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_8_vert_direct.txt";
  graph.LoadGraphFromFile(InputFileName);
  int start = 2;
  std::vector<int> visited_vertices;
  std::vector<int> result;
  visited_vertices = algo.BreadthFirstSearch(graph, start);

  result = {2, 3, 1, 4, 6, 5, 8, 7};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }

  start = 6;
  visited_vertices.clear();
  result.clear();
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {6, 5, 7, 1, 2, 4, 3, 8};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }
}

TEST(GraphAlgorithmsTest, test_BFS_directed_graph_21_check_big_graph) {
  s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_21_vert_direct.txt";
  graph.LoadGraphFromFile(InputFileName);

  int start = 1;
  std::vector<int> visited_vertices;
  std::vector<int> result;
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {1,  2,  13, 12, 15, 16, 17, 3, 11, 14, 4,
            10, 19, 20, 5,  7,  21, 18, 6, 9,  8};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }

  start = 14;
  visited_vertices.clear();
  result.clear();
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {14, 13, 16, 15, 3,  4, 10, 5,  7, 11,
            21, 6,  9,  19, 20, 8, 18, 12, 17};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }

  visited_vertices.clear();
  result.clear();
}

TEST(GraphAlgorithmsTest, test_BFS_weighted_graph_9_check_vert) {
  s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_graph_9_vert.txt";
  graph.LoadGraphFromFile(InputFileName);

  int start = 9;
  std::vector<int> visited_vertices;
  std::vector<int> result;
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {9, 1, 4, 7, 8, 2, 3, 5, 6};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }
  visited_vertices.clear();
  result.clear();
  start = 4;
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {4, 1, 3, 5, 9, 2, 7, 8, 6};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }
  visited_vertices.clear();
  result.clear();

  start = 1;
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {1, 2, 4, 7, 8, 9, 3, 5, 6};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }
}

TEST(GraphAlgorithmsTest, test_BFS_very_big_graph) {
  s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/very_big_graph.txt";
  graph.LoadGraphFromFile(InputFileName);
  int start = 16;
  std::vector<int> visited_vertices;
  std::vector<int> result;
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {16, 1,  2,  17, 27, 45, 46, 3,  5,  7,  47, 48, 14, 11, 12, 20, 23,
            26, 19, 24, 44, 4,  49, 6,  34, 35, 15, 9,  21, 18, 40, 28, 29, 41,
            25, 43, 50, 8,  33, 36, 10, 13, 22, 30, 37, 42, 31, 32, 38, 39};
  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }
}

TEST(GraphAlgorithmsTest, test_BFS_simple_graph_with_loops) {
  s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_graph_6_vert_with_loops.txt";
  graph.LoadGraphFromFile(InputFileName);

  int start = 1;
  std::vector<int> visited_vertices;
  std::vector<int> result;
  visited_vertices = algo.BreadthFirstSearch(graph, start);
  result = {1, 2, 4, 3, 5, 6};

  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }
  result.clear();

  start = 3;
  result = {3, 2, 6, 1, 5, 4};

  visited_vertices = algo.BreadthFirstSearch(graph, start);

  for (int it = 0; it < result.size(); ++it) {
    ASSERT_EQ(visited_vertices[it], result[it]);
  }
}
}  // namespace s21