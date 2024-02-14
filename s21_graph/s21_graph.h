#ifndef SRC_S21_GRAPH_H_
#define SRC_S21_GRAPH_H_
#include <cstring>
#include <iostream>
#include <vector>

#include "../s21_algorithms/s21_graph_algorithms.h"

class GraphAlgorithms;

namespace s21 {
class s21_Graph {
 public:
  s21_Graph() = default;
  s21_Graph(int size) { this->size_ = size; }
  ~s21_Graph() = default;
  std::vector<std::vector<int>> getAdjacencyMatrix();
  std::vector<std::vector<int>> getAdjacencyList();
  std::vector<std::pair<int, int>> getEdgesList();
  int get_graph_size();
  bool CheckFile(const std::string &filename);
  bool IsGraphConnected(s21_Graph &graph, std::string filename);
  std::vector<int> FindPath(s21_Graph &graph, int start_vertex);
  void MakeMatrixUndirected(s21::s21_Graph &graph);
  bool IsDerrected(std::vector<std::vector<int>> AdjacencyMatrix);
  void LoadGraphFromFile(std::string filename);
  void ExportGraphToDot(std::string filename);

 private:
  int size_;
  std::vector<std::vector<int>> AdjacencyMatrix_;
  std::vector<std::pair<int, int>> edgesList_;
  std::vector<std::vector<int>> adjacencyList_;
};

}  // namespace s21

#endif  // SRC_S21_GRAPH_H_