/// \file
#include "consolemenu.h"

namespace s21 {
/// @brief Метод ConsoleMenuInterface::Deploy() представляет собой цикл, в
/// котором отображается меню консольного интерфейса и выполняются определенные
/// действия в зависимости от выбранного пользователем пункта меню. Вот описание
/// каждой части метода:
/// 1. case 1: Вызывается функция CallGraphLoading(pathToFile), которая
///    загружает граф из файла, указанного в переменной pathToFile.
/// 2. case 2 - case 7: Проверяется значение флага flag. Если флаг равен 0,
///    вызывается функция SendMassegeToUser() для отправки сообщения
///    пользователю о необходимости загрузки графа. В противном случае
///    вызывается соответствующая функция для выполнения выбранного алгоритма.
///    Если значение флага не равно 0, то вызывается функция CallBFS(),
///    CallDFS(), CallDeikstra(), CallFloid(), CallPrim() или CallAnt() в
///    зависимости от выбранного пункта меню.
void ConsoleMenuInterface::Deploy() {
  std::string pathToFile;
  s21::s21_Graph graph;
  int flag = 0;
  int number = 0;
  while (number != 8) {
    ShowMenu();
    std::string input = "\0";
    std::cin >> input;
    try {
      number = std::stoi(input);
    } catch (const std::exception& e) {
      std::cout << "(Use simple numbers from 1 to 8)." << std::endl;
    }
    std::cout << std::endl;
    if (number < 1 || number > 8) {
      std::cout << "Plese, choose from menu!" << std::endl;
      ShowMenu();
    }
    switch (number) {
      case 1:
        CallGraphLoading(pathToFile);
        if (graph_.get_graph_size() > 1) flag += 1;
        break;
      case 2: {
        if (flag == 0) {
          SendMassegeToUser();
          break;
        }
        CallBFS();
        break;
      }
      case 3: {
        if (flag == 0) {
          SendMassegeToUser();
          break;
        }
        CallDFS();
        break;
      }
      case 4: {
        if (flag == 0) {
          SendMassegeToUser();
          break;
        }
        CallDeikstra();
        break;
      }
      case 5: {
        if (flag == 0) {
          SendMassegeToUser();
          break;
        }
        CallFloid();
        break;
      }
      case 6: {
        if (flag == 0) {
          SendMassegeToUser();
          break;
        }
        CallPrim();
        break;
      }
      case 7: {
        if (flag == 0) {
          SendMassegeToUser();
          break;
        }
        CallAnt();
        break;
      }
    }
  }
}
/// @brief Метод ConsoleMenuInterface::ShowMenu() отображает меню консольного
/// интерфейса для пользователя. Он выводит на экран список доступных операций с
/// соответствующими номерами, чтобы пользователь мог выбрать желаемое действие.
void ConsoleMenuInterface::ShowMenu() {
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".                       WELCOME TO SIMPLE NAVIGATOR! WHAT DO "
               "YOU WANT TO DO?                                           ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".  1. Load the original graph from a file                      "
               "                                                        ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".  2. Go thru graph in Breadth and print the result            "
               "                                                        ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".  3. Go thru graph in Depth and print the result              "
               "                                                        ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".  4. Find the shortest path between any two vertices and "
               "print the result                                             ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".  5. Find the shortest paths between all pairs of vertices in "
               "the graph and print the resulting matrix                ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".  6. Find the minimal spanning tree in the graph and print "
               "the resulting adjacency matrix and the tree weight         ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".  7. Solve the salesman problem and print the resulting route "
               "and its length                                          ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << ".  8. EXIT                                                     "
               "                                                        ."
            << std::endl;
  std::cout << "..............................................................."
               "........................................................."
            << std::endl;
  std::cout << std::endl;
  std::cout << ">";
}

void ConsoleMenuInterface::CallGraphLoading(std::string pathToFile) {
  std::cout << "specify the path to the file:" << std::endl;
  std::cin >> pathToFile;
  RemoveSrcPrefix(pathToFile);
  if (graph_.CheckFile(pathToFile)) {
    graph_.LoadGraphFromFile(pathToFile);
    if (graph_.IsGraphConnected(graph_, pathToFile)) {
      std::cout << "                                                     "
                   "                                                     "
                   "              "
                << std::endl;
      std::cout << "                               Adjacency matrix for "
                   "loaded graph                                         "
                   "               "
                << std::endl;
      std::cout << "                              "
                   "...................................."
                << std::endl;
      GraphAlgorithms_.PrintAdjacencyMatrix(graph_.getAdjacencyMatrix());
      std::cout << std::endl;
      std::cout << "Graph size is    " << graph_.get_graph_size() << std::endl;

      std::cout << std::endl;

    } else {
      std::cout << std::endl;
      std::cout << "                             Oooops.... This graph "
                   "is not connected. Try another "
                << std::endl;
      std::cout << "                           "
                   "........................................................."
                << std::endl;
    }
  }
}

/// @brief Метод ConsoleMenuInterface::CallBFS() вызывает метод поиска в
/// ширину (BreadthFirstSearch) в графе и выводит результат на консоль.
void ConsoleMenuInterface::CallBFS() {
  s21::GraphAlgorithms algo;
  int start_for_bfs = GetNumForAlgo();
  std::cout << std::endl;
  std::cout << "Graph breadth search result:" << std::endl;
  std::cout << "............................" << std::endl;
  std::cout << std::endl;
  std::vector<int> result =
      GraphAlgorithms_.BreadthFirstSearch(graph_, start_for_bfs);
  algo.PrintResultWay(result);
  std::cout << std::endl;
}

/// @brief Метод ConsoleMenuInterface::CallDFS() вызывает метод поиска в
/// глубину (DepthFirstSearch) в графе и выводит результат на консоль.
void ConsoleMenuInterface::CallDFS() {
  s21::GraphAlgorithms algo;
  int start_for_dfs = GetNumForAlgo();
  std::cout << std::endl;
  std::cout << "Graph deapth search result: " << std::endl;
  std::cout << "..........................." << std::endl;
  std::cout << std::endl;
  std::vector<int> result1 =
      GraphAlgorithms_.DepthFirstSearch(graph_, start_for_dfs);
  algo.PrintResultWay(result1);
  std::cout << std::endl;
}
/// @brief Метод ConsoleMenuInterface::GetNumForAlgo() получает от пользователя
/// номер вершины, с которой нужно выполнить операцию алгоритма и проверяет ее
/// на валидность.
/// @return возвращает проверенную введенную пользователем вершину
int ConsoleMenuInterface::GetNumForAlgo() {
  int vertex = 0;
  std::string input = "\0";
  while (vertex < 1 || vertex > graph_.get_graph_size()) {
    std::cout << "Please select a vertex to start with. (It must be less "
                 "then count of vertices in current graph.)"
              << std::endl;
    std::cin >> input;
    try {
      vertex = std::stoi(input);
    } catch (const std::exception& e) {
      std::cout << "(Use simple numbers)." << std::endl;
    }
  }
  return vertex;
}
/// @brief Метод ConsoleMenuInterface::GetNumForAlgo() получает от пользователя
/// номер вершины, с которой нужно выполнить операцию алгоритма и проверяет ее
/// на валидность.
/// @return возвращает проверенную введенную пользователем вершину
int ConsoleMenuInterface::GetNumForAlgoB() {
  int vertex = 0;
  std::string input = "\0";
  while (vertex < 1 || vertex > graph_.get_graph_size()) {
    std::cout << "Please select a vertex to finish (It must be less "
                 "then count of vertices in current graph.)"
              << std::endl;
    std::cin >> input;
    try {
      vertex = std::stoi(input);
    } catch (const std::exception& e) {
      std::cout << "(Use simple numbers)." << std::endl;
    }
  }
  return vertex;
}
/// @brief Метод ConsoleMenuInterface::CallDeikstra() вызывает метод поиска
/// кратчайшего пути (GetShortestPathBetweenVertices) в графе и выводит
/// результат на консоль.
void ConsoleMenuInterface::CallDeikstra() {
  s21::GraphAlgorithms algo;
  int vertex1 = GetNumForAlgo();
  int vertex2 = GetNumForAlgoB();
  std::cout << std::endl;

  std::cout << "Search for the shortest path between any two vertices: "
            << std::endl;
  std::cout << "......................................................"
            << std::endl;
  std::cout << std::endl;

  int shortestDistance =
      algo.GetShortestPathBetweenVertices(graph_, vertex1, vertex2);
  std::cout << "Shortest Distance from Vertex    " << vertex1
            << "   to Vertex    " << vertex2 << "  :  " << shortestDistance
            << std::endl;
}

/// @brief Метод ConsoleMenuInterface::CallFloid() вызывает метод поиска
/// кратчайших путей между всеми вершинами(GetShortestPathsBetweenAllVertices) в
/// графе и выводит результат на консоль.
void ConsoleMenuInterface::CallFloid() {
  s21::GraphAlgorithms algo;
  std::cout << std::endl;
  std::cout << "The result of algorithm Floid_Uolshera : " << std::endl;
  std::cout << "......................................." << std::endl;
  std::cout << std::endl;

  std::vector<std::vector<int>> minimum_distance =
      algo.GetShortestPathsBetweenAllVertices(graph_);
  algo.PrintAdjacencyMatrix(minimum_distance);

  std::cout << std::endl;
}

/// @brief Метод ConsoleMenuInterface::CallPrim() вызывает метод поиска
/// минимального оставного дерева(GetLeastSpanningTree) в графе и выводит
/// результат на консоль.
void ConsoleMenuInterface::CallPrim() {
  s21::GraphAlgorithms algo;

  std::cout << std::endl;
  std::cout << "The result of algorithm Prima : " << std::endl;
  std::cout << "............................." << std::endl;
  std::cout << std::endl;

  std::vector<std::vector<int>> result = algo.GetLeastSpanningTree(graph_);
  algo.PrintAdjacencyMatrix(result);
  int weight = algo.GetGraphWeigt(result);
  std::cout << std::endl;

  std::cout << "Spanning Tree Weight = " << weight / 2 << std::endl;
}
/// @brief Метод ConsoleMenuInterfaceCallAntCallPrim() вызывает метод поиска
/// пути при помощи муравьинного алгоритма(SolveTravelingSalesmanProblem) в
/// графе и выводит результат на консоль.
void ConsoleMenuInterface::CallAnt() {
  s21::GraphAlgorithms algo;

  std::cout << std::endl;
  std::cout << "The result of algorithm of Salesman Problem : " << std::endl;
  std::cout << "............................................." << std::endl;
  std::cout << std::endl;
  std::vector<int> way;
  double distance;
  s21::TsmResult result = algo.SolveTravelingSalesmanProblem(graph_);
  way = result.path;
  distance = result.distance;

  std::cout << "Way is  -  ";
  for (int it = 0; it < way.size(); ++it) {
    if (it != way.size() - 1) {
      std::cout << way[it] << " -> ";
    } else {
      std::cout << way[it] << std::endl;
      ;
    }
  }

  std::cout << "Distance is   -  " << distance << std::endl;
}
/// @brief Метод ConsoleMenuInterface::RemoveSrcPrefix(const std::string& path)
/// удаляет префикс "src/" из заданного пути path.
std::string ConsoleMenuInterface::RemoveSrcPrefix(const std::string& path) {
  std::string srcPrefix = "src/";
  if (path.substr(0, srcPrefix.size()) == srcPrefix) {
    return path.substr(srcPrefix.size());
  }
  return path;
}
/// @brief Метод ConsoleMenuInterface::SendMessageToUser() выводит
/// предупреждение пользователю о том что граф не загружен в консоль.
void ConsoleMenuInterface::SendMassegeToUser() {
  std::cout << "* Warning!!! Plese, load a graph before using algorithms!"
            << std::endl;
}

}  // namespace s21
