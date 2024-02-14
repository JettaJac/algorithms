/// \file
#include "s21_graph.h"

#include <errno.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
/// @brief Метод-геттер
/// @param () Без параметров
/// @return Возвращает матрицу векторов из приватного поля AdjacencyMatrix_
std::vector<std::vector<int>> s21::s21_Graph::getAdjacencyMatrix() {
  if (AdjacencyMatrix_.size() < 1) return std::vector<std::vector<int>>();
  return AdjacencyMatrix_;
}

/// @brief Метод-геттер бежит по матрице смежности, создает  вектор
/// edgesList_, который состоит из сущностей-пар(иными словами двух вершин между
/// которыми есть ребро). В такой паре - первое значение вершина, из которой
/// выходит ребро, а второе значение - вторая вершина докуда идет ребро
/// @param () Без параметров
/// @return  Возвращает вектор из пар из приватного поля edgesList_
std::vector<std::pair<int, int>> s21::s21_Graph::getEdgesList() {
  for (int i = 0; i < size_; ++i) {
    for (int j = 0; j < size_; ++j) {
      if (AdjacencyMatrix_[i][j] != 0) {
        edgesList_.push_back(std::make_pair(i + 1, j + 1));
      }
    }
  }
  return edgesList_;
}
/// @brief Метод-геттер, возвращающий размер графа
/// @param () Без параметров
/// @return  Возвращает размер графа из приватного поля size_
int s21::s21_Graph::get_graph_size() { return size_; }

/// @brief Метод-геттер, возвращающий список смежности (список вершин-соседей)
/// для каждой из вершин в рамках этого метода создается матрица смежности
/// adjacencyList_ размером size_ пробегая по исходной матрице смежности находим
/// ненулевые позиции, и для каждого ряда (то есть вершины)
///  adjacencyList_ записываем номера непустых столбцов
/// получая такми образом список смежных вершин
/// @param () Без параметров
/// @return список смежности из приватного поля, матрицы adjacencyList_

std::vector<std::vector<int>> s21::s21_Graph::getAdjacencyList() {
  std::vector<std::vector<int>> adjacencyList_(size_);
  for (int i = 0; i < size_; ++i) {
    for (int j = 0; j < size_; ++j) {
      if (AdjacencyMatrix_[i][j] != 0) {
        adjacencyList_[i].push_back(j + 1);
      }
    }
  }
  return adjacencyList_;
}

/// @brief метод CheckFile открывает файл, из которого грузим матрицу смежности
/// и проверяет на то, чтобы 1) файл открылся, чтоб путь был правильный и имя 2)
/// чтобы файл не содержал символов 3) матрица квадратная и соответствует
/// указанной размерности 4) веса графа (если взвешенный) - не отрицательные
/// @param const std::string& filename - входящее имя файла, в формате "строка"
/// @param int dimention
/// переменная int dimention  принимает с помощью инструкции ifs >> dimension
/// первую цифру над матрицей в файле dimension - int и если туда придет не
/// число, то будет ошибка приведения типов,
///   по аналогии с dimentsion контроль типов остальных данных файла
///   осуществляет переменная int value: в усвловии (ifs >> value && value >=0)
/// оно означает что будут приходить неотрицательные инты
/// Таким образом мы проверили п №2 выше
/// п3-4 проверяются следующим алгоритмом: заводим переменную, в которую
/// принимаем данные value два счетчика col row 1) начиная с нулевой колонки
/// создаем пустой вектор. Находясь на текущем ряду
///  пушим  неотрицательные инты в текущий ряд, увеличивая сччетчик колонок +1;
/// 2) как только счетчик сравнялся  с размерностью size_
/// мы его обнуляем и начинаем считать ряды, таким образом заполняя матрицу с
/// нового ряда который пустым сразу создаем в соответствии с условием col == 0
/// выше 3) когда закончились значения в файле, мы сравниваем. сколько насчитали
/// рядов и если они не равны значению dimension, значит raw !=col: CheckFile
/// вернул  false
/// @return список смежности из приватного поля, матрицы adjacencyList_

bool s21::s21_Graph::CheckFile(const std::string& filename) {
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    throw std::runtime_error("Error opening file");
    return false;
  }

  if (ifs.peek() == std::ifstream::traits_type::eof()) {
    throw std::runtime_error("File is empty: " + filename);
  }
  int dimension = 0;
  if (!(ifs >> dimension)) {
    throw std::runtime_error("Invalid value");
    return false;
  }
  std::vector<std::vector<int>> matrix;
  int value;
  int row = 0;
  int col = 0;
  while (ifs >> value && value >= 0) {
    if (col == 0) {
      matrix.push_back(std::vector<int>());
    }
    matrix[row].push_back(value);
    col++;
    if (col == dimension) {
      col = 0;
      row++;
    }
  }
  if (col != 0 || row != dimension) {
    return false;
  }
  ifs.close();
  return true;
}
/// @brief Метод, выгружающий матрицу  из файла
/// @param const std::string& filename - входящее имя файла, в формате "строка"
/// 3) чтобы размерность  матрицы была положительным и ненулевым значением
/// выполняется проверка if(size_ <= 0) { std::cout << "Dimension must be > 0";
/// return;}
/// @return  no

void s21::s21_Graph::LoadGraphFromFile(std::string filename) {
  std::ifstream ifs(filename);
  try {
    if (!ifs.is_open()) {
      throw std::runtime_error("Error opening file");
    }
  } catch (const std::exception& e) {
  }

  try {
    if (!CheckFile(filename)) {
      throw std::runtime_error("Invalid value");
    }
  } catch (const std::exception& e) {
  }
  ifs >> size_;
  try {
    if (size_ <= 0) {
      throw std::runtime_error("Dimension must be > 0");
    }
  } catch (const std::exception& e) {
  }
  AdjacencyMatrix_.resize(size_, std::vector<int>(size_));

  for (int i = 0; i < size_; ++i) {
    for (int j = 0; j < size_; ++j) {
      ifs >> AdjacencyMatrix_[i][j];
    }
  }
  ifs.close();
}

/// @brief Метод, специально встроенный для определения связности графа
/// @param graph - текущий граф.
/// @param start_vertex - точка начала происка пути.
/// @return std::vector<int> visited_vertices - результатом работы метода
/// является вектор посещенных точек
std::vector<int> s21::s21_Graph::FindPath(s21_Graph& graph, int start_vertex) {
  std::vector<int> visited_vertices;
  std::stack<int> vertex_stack;
  std::vector<std::vector<int>> adjacency_list = graph.getAdjacencyList();

  vertex_stack.push(start_vertex);

  while (!vertex_stack.empty()) {
    int current_vertex = vertex_stack.top();
    vertex_stack.pop();
    if (std::find(visited_vertices.begin(), visited_vertices.end(),
                  current_vertex) != visited_vertices.end()) {
      continue;
    }
    visited_vertices.push_back(current_vertex);
    std::vector<int> adjacent_vertices = adjacency_list[current_vertex - 1];
    for (int it = adjacent_vertices.size() - 1; it >= 0; --it) {
      int adjacent_vertex = adjacent_vertices[it];
      if (std::find(visited_vertices.begin(), visited_vertices.end(),
                    adjacent_vertex) == visited_vertices.end()) {
        vertex_stack.push(adjacent_vertex);
      }
    }
  }
  return visited_vertices;
}

/// @brief Метод, меняющий матрицу смежности ориентированного графа на матрицу
/// смежности неориентированного у нас в матрице ориентированного графа могут
/// быть разные веса
///  поэтому условие AdjacencyMatrix_[i][j] != AdjacencyMatrix_[j][i] будет
///  работать только для нулей и единиц
/// проверяем каждую пару вершин (i, j) и (j, i) не пустые ли. Существует ли
/// ребро между i или j. Если AdjacencyMatrix_[i][j] != 0 значит
///  ребро есть в виде его веса. Мы не знаем какой из них может быть ненулевым
///  поэтому выбираем максимум из них
/// и сохраняем в обе вершины
/// @param graph - текущий граф.
/// @return ничего не возвращает

void s21::s21_Graph::MakeMatrixUndirected(s21::s21_Graph& graph) {
  for (int i = 0; i < size_; ++i) {
    for (int j = i + 1; j < size_; ++j) {
      if (AdjacencyMatrix_[i][j] != 0 || AdjacencyMatrix_[j][i] != 0) {
        int weight = std::max(AdjacencyMatrix_[i][j], AdjacencyMatrix_[j][i]);
        AdjacencyMatrix_[i][j] = weight;
        AdjacencyMatrix_[j][i] = weight;
      }
    }
  }
}
/// @brief Метод, определяющий, является ли граф направленным
/// @param std::vector<std::vector<int>> AdjacencyMatrix - матрица смежности
/// графа
/// @return возвращает либо false либо true
bool s21::s21_Graph::IsDerrected(
    std::vector<std::vector<int>> AdjacencyMatrix) {
  for (int i = 0; i < size_; ++i) {
    for (int j = 0; j < size_; ++j) {
      if (AdjacencyMatrix[i][j] != AdjacencyMatrix[j][i]) {
        return true;
      }
    }
  }
  return false;
}

/// @brief Метод, определяющий, является ли граф связным
/// @param  graph - текущий граф.
/// @param filename - файл, из которого приходит матрица
/// @return возвращает либо false либо true
bool s21::s21_Graph::IsGraphConnected(s21_Graph& graph, std::string filename) {
  if (IsDerrected(graph.AdjacencyMatrix_) == false) {
    std::vector<int> visited = FindPath(graph, 3);
    if (visited.size() == size_) {
      return true;
    }
  } else {
    MakeMatrixUndirected(graph);
    std::vector<int> visited = FindPath(graph, 3);
    if (visited.size() == size_) {
      graph.LoadGraphFromFile(filename);
      return true;
    }
  }
  return false;
}
/// @brief Метод, загружающий матрицу в файл
/// @param const std::string& filename - входящее имя файла, в формате "строка"
/// @return  no
void s21::s21_Graph::ExportGraphToDot(std::string filename) {
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    std::cout << "Error opening file";
    return;
  }

  ofs << "graph G {\n";
  for (int i = 0; i < size_; ++i) {
    for (int j = 0; j < size_; ++j) {
      if (AdjacencyMatrix_[i][j] != 0) {
        ofs << "  " << i << " -- " << j << ";\n";
      }
    }
  }
  ofs << "}\n";

  ofs.close();
}
