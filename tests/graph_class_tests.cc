#include <gtest/gtest.h>

namespace s21 {

TEST(Graph, getAjacencyMatrix) {
  s21::s21_Graph graph;
  std::string InputFileName = "./examples/graph_3_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  std::vector<std::vector<int>> adjacencyMatrix = graph.getAdjacencyMatrix();
  std::vector<std::vector<int>> expected{{0, 1, 1}, {1, 0, 1}, {1, 1, 0}};
  ASSERT_EQ(adjacencyMatrix, expected);
}

TEST(GraphAlgorithmsTest, getEdgeList) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_3_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  std::vector<std::pair<int, int>> GetedgesList = graph.getEdgesList();
  std::vector<std::pair<int, int>> expected{{1, 2}, {1, 3}, {2, 1},
                                            {2, 3}, {3, 1}, {3, 2}};
  ASSERT_EQ(GetedgesList, expected);
}

TEST(GraphAlgorithmsTest, errorOpeningFile) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_155_vert_simply.txt";
  EXPECT_ANY_THROW(graph.LoadGraphFromFile(InputFileName));
}

TEST(GraphAlgorithmsTest, DOT) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_3_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  std::string outputFile = "output.dot";
  graph.ExportGraphToDot(outputFile);
}

TEST(GraphAlgorithmsTest, Graph_DOTerror_Test) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_3_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  std::string outputFile = "output1.dot";
  graph.ExportGraphToDot(outputFile);
}

TEST(GraphAlgorithmsTest, ExportGraphToDot_ErrorOpeningFile) {
  s21::s21_Graph graph;
  testing::internal::CaptureStdout();
  graph.ExportGraphToDot("DD:/nonexistent_file.dot");
  std::string captured_stdout = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(captured_stdout.find("Error opening file") != std::string::npos);
}

TEST(Graph, Graph_DOTerror_Test) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_3_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  std::string outputFile = "output1.dot";
  graph.ExportGraphToDot(outputFile);
}

TEST(GraphAlgorithmsTest, CheckFile1) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_2_DIM_NOT_NUM.txt";
  ASSERT_ANY_THROW(graph.CheckFile(InputFileName));

  InputFileName = "./examples/graph_2_vert_char.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.CheckFile(InputFileName);
  bool expected = false;
  ASSERT_EQ(result, expected);
}

TEST(Graph, CheckFile2) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_2_vert_dimNE.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.CheckFile(InputFileName);
  bool expected = false;
  ASSERT_EQ(result, expected);
}

TEST(Graph, CheckFile3) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_2_vert_R_NE_C.txt";
  bool result = graph.CheckFile(InputFileName);
  bool expected = false;
  ASSERT_EQ(result, expected);
}

TEST(Graph, CheckFile4) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_2_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.CheckFile(InputFileName);
  bool expected = false;
  ASSERT_EQ(result, expected);
}

TEST(Graph, CheckFile5) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "DDD:/graph_2_vert_simply.txt";
  ASSERT_ANY_THROW(graph.LoadGraphFromFile(InputFileName));
}

TEST(Graph, CheckFile6) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_2_DIM_NOT_NUM.txt";
  ASSERT_ANY_THROW(graph.CheckFile(InputFileName));
}

TEST(Graph, CheckFile10) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_13_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.IsGraphConnected(graph, InputFileName);
  bool expected = true;
  ASSERT_EQ(result, expected);
}

TEST(Graph, DIM_NEGATIVE) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_2_DIM_NOT_NUM.txt";
  ASSERT_ANY_THROW(graph.CheckFile(InputFileName));
}

TEST(Graph, DIM_NEGATIVE_2) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_3_DIM_LE_ZERO.txt";
  ASSERT_ANY_THROW(graph.LoadGraphFromFile(InputFileName));
}

TEST(Graph, FILE_EMPTY) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_2__empty.txt";
  ASSERT_ANY_THROW(graph.CheckFile(InputFileName));
}

TEST(Graph, Graph_connectivity) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/unconnected_graph_5_vert.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.IsGraphConnected(graph, InputFileName);
  bool expected = false;
  ASSERT_EQ(result, expected);
}

TEST(Graph, Graph_connectivity_2) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_directed_graph_8_vert.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.IsGraphConnected(graph, InputFileName);
  bool expected = true;
  ASSERT_EQ(result, expected);
}
TEST(Graph, Graph_connectivity_3) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_5_vert_simply.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.IsGraphConnected(graph, InputFileName);
  bool expected = true;
  ASSERT_EQ(result, expected);
}

TEST(Graph, Graph_connectivity_4) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_4_unconnected.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.IsGraphConnected(graph, InputFileName);
  bool expected = false;
  ASSERT_EQ(result, expected);
}

TEST(Graph, Graph_connectivity_5) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_4_orient_uncon.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.IsGraphConnected(graph, InputFileName);
  bool expected = false;
  ASSERT_EQ(result, expected);
}
TEST(Graph, Graph_connectivity_6) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/graph_5_orient_connected.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.IsGraphConnected(graph, InputFileName);
  bool expected = true;
  ASSERT_EQ(result, expected);
}
TEST(Graph, Graph_connectivity_7) {
  s21::s21_Graph graph;
  s21::GraphAlgorithms algo;
  std::string InputFileName = "./examples/weighted_directed_graph_7_vert.txt";
  graph.LoadGraphFromFile(InputFileName);
  bool result = graph.IsGraphConnected(graph, InputFileName);
  bool expected = true;
  ASSERT_EQ(result, expected);
}

}  // namespace s21