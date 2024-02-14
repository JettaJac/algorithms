/// \file
#include "s21_graph_algorithms.h"

/// @brief Метод поиска пути в глубину в графе.
/// Алгоритм метода таков:
/// 1. Создаем списое пройденныйх точек типа std::vector<int>
/// 2. Создаем стэк для работы с точками.
/// 3. Получаем список смежности adjacency_list из класса s21_Graph
/// 4. Добавляем в стэк первую точку - start_vertex.
/// 5. Далее в цикле, условием выхода из которого является опустошение стэка,
///    проверяем, является ли вершина с вершины стэка посещенной, а так же
///    удаляем эту вершину из стэка.
/// 6. Если вершина уже посещена, переходим к следующей итерации цикла, если не
///    посещена, проверяем список вершин для текщей вершины, проверяя, являются
///    ли смежные вершины посещенными, если не являются, добавляем их на вершину
///    стэка

/// @param graph - ссылкана  текущий граф.
/// @param start_vertex - точка начала происка пути.
/// @return std::vector<int> visited_vertices - результатом работы метода
/// является вектор посещенных точек

std::vector<int> s21::GraphAlgorithms::DepthFirstSearch(s21_Graph &graph,
                                                        int start_vertex) {
  if (start_vertex > graph.get_graph_size() || start_vertex < 0) {
    throw std::length_error("Start vertex is incorrect");
  }
  std::vector<int> visited_vertices;
  s21::Stack<int> vertex_stack;
  matrix adjacency_list = graph.getAdjacencyList();

  vertex_stack.push(start_vertex);

  while (!vertex_stack.empty()) {
    int current_vertex = vertex_stack.top();
    vertex_stack.pop();
    if (CheckVisited(visited_vertices, current_vertex) == false) {
      continue;
    }

    std::vector<int> adjacent_vertices = adjacency_list[current_vertex - 1];
    visited_vertices.push_back(current_vertex);

    for (int it = adjacent_vertices.size() - 1; it >= 0; --it) {
      if (CheckVisited(visited_vertices, adjacent_vertices[it]) == true) {
        vertex_stack.push(adjacent_vertices[it]);
      }
    }
  }
  return visited_vertices;
}

/// @brief Метод поиска пути в ширину в графе.
/// Алгоритм метода таков:
/// 1. Создаем списое пройденныйх точек типа std::vector<int>
/// 2. Создаем очередь для работы с точками.
/// 3. Получаем список смежности adjacency_list из класса s21_Graph
/// 4. Добавляем в очередь первую точку - start_vertex.
/// 5. Далее в цикле, условием выхода из которого является опустошение очереди,
///    проверяем, является ли вершина с вершины очереди посещенной, а так же
///    удаляем эту вершину из очереди.
/// 6. Если вершина еще не посещена была, пушим в пройденный путь.
/// 7. В цикле поочереди забираем из листа смежности следующую вершину, если мы
///    ее не посещали, то пушим в очередь.

/// @param graph - ссылка на  текущий граф.
/// @param start_vertex - точка начала происка пути.
/// @return std::vector<int> visited_vertices - результатом работы метода
/// является вектор посещенных точек

std::vector<int> s21::GraphAlgorithms::BreadthFirstSearch(s21_Graph &graph,
                                                          int start_vertex) {
  if (start_vertex > graph.get_graph_size() || start_vertex <= 0) {
    throw std::length_error("Start vertex is incorrect");
  }
  std::vector<int> visited_vertices;
  s21::Queue<int> queue_vertices_add;
  matrix adjacency_list = graph.getAdjacencyList();
  int adjacent_vertices;
  queue_vertices_add.push(start_vertex);
  while (!queue_vertices_add.empty()) {
    int current_vertex = queue_vertices_add.front();
    queue_vertices_add.pop();
    if (CheckVisited(visited_vertices, current_vertex)) {
      visited_vertices.push_back(current_vertex);
    }
    for (int j = 0; j < adjacency_list[current_vertex - 1].size(); j++) {
      adjacent_vertices = adjacency_list[current_vertex - 1][j];
      if (CheckVisited(visited_vertices, adjacent_vertices)) {
        queue_vertices_add.push(adjacent_vertices);
      }
    }
  }
  visited_vertices.shrink_to_fit();
  return visited_vertices;
}

/// @brief Метод для поиска кратчайшего пути между двумя вершинами в графе с
/// использованием алгоритма Дейкстры.
///
/// Алгоритм работает следующим образом:
/// 1. Проверяем корректность входных вершин (vertex1 и vertex2) и получаем
///    размер графа (graphSize).
/// 2. Инициализируем матрицу смежности (adjacencyMatrix) из объекта графа.
/// 3. Преобразуем индексы вершин vertex1 и vertex2 в 0
/// 4. Создаем массив distances для хранения расстояний от начальной вершины до
///    всех остальных вершин графа. Исходное расстояние до начальной вершины
///    устанавливается как 0, а до всех остальных вершин - как бесконечность
///    (inf).
/// 5. Создаем массив visited для отслеживания посещенных вершин. Начально все
///    вершины помечены как не посещенные.
/// 6. Создаем приоритетную очередь q для выбора вершины с наименьшим
/// расстоянием.
/// 7. Добавляем начальную вершину vertex1 в очередь q с расстоянием 0.
/// 8. Основной цикл алгоритма выполняется до тех пор, пока очередь q не пуста.
/// 9. Извлекаем вершину с наименьшим расстоянием из очереди q.
/// 10. Проверяем, если текущее расстояние до вершины v короче, чем сохраненное
///     расстояние в массиве distances, то продолжаем выполнение.
/// 11. Перебираем смежные вершины (to) и обновляем расстояния до них, если
///     новое расстояние короче текущего.
/// 12. Если кратчайший путь до вершины vertex2 равен бесконечности (inf), то
///     выбрасываем исключение с сообщением об отсутствии пути между вершинами.
/// 13. Возвращаем кратчайшее расстояние до вершины vertex2.
///
/// @param graph - ссылка на объект графа.
/// @param vertex1 - номер первой вершины
/// @param vertex2 - номер второй вершины
/// @return int - кратчайшее расстояние между вершинами vertex1 и vertex2.
/// @throw std::invalid_argument если vertex1, vertex2 не являются
/// допустимыми вершинами графа или отсутствует путь между ними.

int s21::GraphAlgorithms::GetShortestPathBetweenVertices(s21_Graph &graph,
                                                         int vertex1,
                                                         int vertex2) {
  int graphSize = graph.get_graph_size();
  if (vertex1 < 1 || vertex2 < 1 || vertex1 > graphSize ||
      vertex2 > graphSize) {
    throw std::invalid_argument("Incorrect input vertices");
  }

  matrix adjacencyMatrix = graph.getAdjacencyMatrix();
  vertex1--;
  vertex2--;

  std::vector<int> distances(graphSize, inf);
  distances[vertex1] = 0;
  std::vector<bool> visited(graphSize, false);

  std::priority_queue<std::pair<int, int>> q;
  q.push({0, vertex1});

  while (!q.empty()) {
    int len = -q.top().first;
    int v = q.top().second;
    q.pop();

    if (len > distances[v]) continue;

    for (int to = 0; to < graphSize; ++to) {
      int length = adjacencyMatrix[v][to];
      if (length > 0 && distances[to] > distances[v] + length) {
        distances[to] = distances[v] + length;
        q.push({-distances[to], to});
      }
    }
  }

  if (distances[vertex2] == inf) {
    throw std::invalid_argument("No path exists between the vertices");
  }
  return distances[vertex2];
}

/// @brief Метод поиска минимального пути между всеми вершинами графа.
/// Алгоритм Флойда-Уоршелла
/// Алгоритм метода таков:
/// 1. Создаем матрицу смежности графа.
/// 2. Заменяем все "0" максимальным значением
/// 3. В первом цикле путь проходит через каждую промежуточную вершину V,
///    начиная с положение, когда нет промежуточной вершины  V = 0.
/// 4. Находим минимальное значение между (Значением между двумя вершина в
///    текущем положении и суммой значений между стартовой точкой и
///    промежуточной V и значением между промежуточной вершиной V и конечной
///    вершиной.
/// 5. Преобразуем результирующую матрицу: Заменяем максимальное значени е 0

/// @param graph - текущий граф.
/// @return matrix (std::vector<std::vector<int>>) min_distance - результатом
/// работы метода в виде матрицы смежности

std::vector<std::vector<int>>
s21::GraphAlgorithms::GetShortestPathsBetweenAllVertices(s21_Graph &graph) {
  matrix min_distance = graph.getAdjacencyMatrix();
  const int size = graph.get_graph_size();
  for (int v = 0; v <= size; v++) {
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (v == 0) {
          if (min_distance[i][j] == 0 || i == j) {
            min_distance[i][j] = inf;
          }
        } else {
          if ((i != j && (v - 1) != i && (v - 1) != j) &&
              (min_distance[i][v - 1] != inf &&
               min_distance[v - 1][j] != inf)) {
            min_distance[i][j] =
                std::min(min_distance[i][j],
                         min_distance[i][v - 1] + min_distance[v - 1][j]);
          }
        }
      }
    }
  }
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      if (min_distance[i][j] == inf) {
        min_distance[i][j] = 0;
      }
    }
  }
  return min_distance;
}

/// @brief Реализация алгоритма Прима.
/// В ходе работы алгоритма производится поиск минимального остовного дерева.
/// 1. Если на вход поступает ориентированный граф, мы преобразуем его в
///    неориентированный и продолжаем работу.
/// 2. Создаем структуру std::vector<bool> visited_or_not для отслеживания
///    посещенных и непосещенных точек.
/// 2.1. Создаем структуру matrix (std::vector<std::vector<int>>) result_matrix
/// для
///      хранения матрицы минимального остовного дерева.
/// 2.2. Создаем структуру matrix (std::vector<std::vector<int>>) working_matrix
/// для
///      работы с весами ребер.
/// 3. Устанавливаем первую посещенную вершину.
/// 4. Пока все вершины не посещены выполняем следующие действия:
/// 4.1. Для каждой посещенной вершины, обновляем working_matrix
///      соответствующими весами ребер, если эти вершины связаны и не посещены.
/// 4.2. Находим координаты ребра с минимальным весом в working_matrix с помощью
///      функции GetMinCoordinats.
/// 4.3. Если вершина, соответствующая минимальному ребру, уже посещена,
///      установливаем вес этого ребра в inf и находим новое минимальное ребро.
/// 4.4. Если вершина, соответствующая минимальному ребру, еще не посещена,
///      добавляем это ребро в result_matrix, устанавливаем вес этого ребра в
///      inf в working_matrix, и помечаем вершину как посещенную.

/// @param graph - объект класса граф.
/// @return matrix (std::vector<std::vector<int>>) - возвращаемое значение
/// матрица смежности минимально оставного дерева графа.
std::vector<std::vector<int>> s21::GraphAlgorithms::GetLeastSpanningTree(
    s21_Graph &graph) {
  matrix graph_matrix = graph.getAdjacencyMatrix();
  graph_matrix = ConvertToUndirected(graph_matrix);
  std::vector<bool> visited_or_not(graph.get_graph_size(), false);
  matrix result_matrix(graph.get_graph_size(),
                       std::vector<int>(graph.get_graph_size(), 0));
  matrix working_matrix(graph.get_graph_size(),
                        std::vector<int>(graph.get_graph_size(), inf));

  int start_vertex = 0;
  visited_or_not[start_vertex] = true;
  while (!IsAllVisited(visited_or_not)) {
    for (int cur_vertex = 0; cur_vertex < graph.get_graph_size();
         ++cur_vertex) {
      if (visited_or_not[cur_vertex]) {
        for (int it = 0; it < graph.get_graph_size(); ++it) {
          if (graph_matrix[cur_vertex][it] != 0 && !visited_or_not[it]) {
            working_matrix[cur_vertex][it] = graph_matrix[cur_vertex][it];
          }
        }
      }
    }
    std::pair<int, int> min_coordinats = GetMinCoordinats(working_matrix);

    if (visited_or_not[min_coordinats.second] == true) {
      working_matrix[min_coordinats.first][min_coordinats.second] = inf;
      min_coordinats = GetMinCoordinats(working_matrix);
    }

    if (visited_or_not[min_coordinats.second] == false) {
      result_matrix[min_coordinats.first][min_coordinats.second] =
          working_matrix[min_coordinats.first][min_coordinats.second];
      result_matrix[min_coordinats.second][min_coordinats.first] =
          working_matrix[min_coordinats.first][min_coordinats.second];
      working_matrix[min_coordinats.first][min_coordinats.second] = inf;

      visited_or_not[min_coordinats.second] = true;
    }
  }
  return result_matrix;
}

/// @brief Алгоритм муравьиной колонии является метаэвристикой, основанной
/// на поведении муравьев при поиске пути к источнику пищи. Он может
/// использоваться для решения задач коммивояжера, минимального остовного
/// дерева и других оптимизационных задач.
/// Алгоритм метода таков:
/// 1. Устанавливаем количество муравьев
/// 2. Загрузка  графа в матрицe смежности и создание ее временной копии,
///  создаем матрицу ферамонов, количество муравьев, количество итераций,
/// коэффициент испарения феромона, коэффициент важности расстояния,
/// коэффициент важности феромона.
/// 3. Для каждого муравья выбирается вершина и запускаеться цикл пока
/// муравей не пройдет все вершины
/// 4. До цикла создаем временный путь посещения муравья,
/// 5. Если посетили все вершиы графа и есть ребро из последней вершины до
/// первой, то прокладдываем, добаляем вес ребра к итоговой сумме и добаляем
///  первую вершину и в конец.
/// 6. Если текущее расстояни (сумма весов пути) меньше, чем записана в
/// итоговой структуре,  заменяее итоговые зачения выходной струкуры
/// текущим значениям.

/// @param graph - текущий граф, представленный в виде матрицы смежности.
/// @return result_struct -  структура с результатом работы алгоритма,
/// содержащая
/// минимальные расстояния, которые прошел муровей между всеми вершинами
/// графа и путь этот путь.

s21::TsmResult s21::GraphAlgorithms::SolveTravelingSalesmanProblem(
    s21_Graph &graph) {
  const int num_ants = 20000;
  const int size = graph.get_graph_size();

  std::vector<std::vector<int>> adjacency_matrix = graph.getAdjacencyMatrix();
  std::vector<std::vector<int>> tmp_adjacency_matrix =
      graph.getAdjacencyMatrix();

  std::vector<std::vector<double>> pheramone_matrix(
      size, std::vector<double>(size, 1.0));

  TsmResult result_struct;
  result_struct.distance = inf;

  int vertex = 0;
  for (int one_ant = 0; one_ant < num_ants; one_ant++) {
    vertex = (one_ant % size) + 1;

    int prev_vertex = 0;
    int min_distance = 0;

    tmp_adjacency_matrix = adjacency_matrix;
    std::vector<std::vector<int>> temp_path(size, std::vector<int>(size, 0));
    int err = 0;
    std::vector<int> res_path;
    for (int j = 0; j < size && err == 0; j++) {
      for (int i = 0; i < size; i++) {
        tmp_adjacency_matrix[i][vertex - 1] = 0;
      }
      std::vector<double> probability_list(size, 0.0);
      err = CreateProbabilityPath(probability_list, pheramone_matrix,
                                  tmp_adjacency_matrix, vertex);
      res_path.push_back(vertex);
      if (err == 0) {
        prev_vertex = vertex;
        vertex = SelectNextVertex(probability_list);
        if (vertex <= 0 || vertex > size) {
          err = 1;
        } else {
          temp_path[vertex - 1][prev_vertex - 1] =
              adjacency_matrix[vertex - 1][prev_vertex - 1];
        }
      }
    }

    if ((err == 0 || res_path.size() == size) &&
        adjacency_matrix[vertex - 1][res_path.front() - 1] != 0) {
      res_path.push_back(res_path.front());
      temp_path[vertex - 1][res_path.front() - 1] =
          adjacency_matrix[vertex - 1][res_path.front() - 1];
      min_distance = GetGraphWeigt(temp_path);
      RecalculatePheramoneMatrix(pheramone_matrix, temp_path, min_distance);
      if ((min_distance < result_struct.distance)) {
        result_struct.distance = min_distance;
        result_struct.path = move(res_path);
      }
    }
  }
  return result_struct;
}

/// @brief - Метод cчитает с какой вероятностью из текуей вершины муравей
/// попадет в другие возможные и не посещенные вершины
/// Алгоритм метода таков:
/// 1. Вводим значения констант: a: коэффициент важности расстояния при
/// выборе следующего шага. b: коэффициент важности феромона при выборе
/// следующего шага.

/// @param probability_list - итоговый лист вероятности посеения
/// возможых вершин
/// @param pheramone_matrix - текущая матрица ферамонта на всем графе
/// @param tmp_adjacency_matrix - временная матрица связностьи. актуальные
/// вершины
/// @param vertex - текуая вершина из которой считаем вероятность
/// @return int - если 0, то вероятность посчитана корректна и в принципе у
/// нас есть доступ ло какой-либо свободной не посещаемой вершины.  1 = ошибка

int s21::GraphAlgorithms::CreateProbabilityPath(
    std::vector<double> &probability_list,
    std::vector<std::vector<double>> pheramone_matrix,
    std::vector<std::vector<int>> tmp_adjacency_matrix, int vertex) noexcept {
  const int a = 1;
  const int b = 1;
  int err = 1;

  const int size = probability_list.size();
  double feramont_distance = 0;
  double sum_feramont_distance = 0;

  for (int j = 0; j < size; j++) {
    if (tmp_adjacency_matrix[vertex - 1][j] != 0) {
      sum_feramont_distance +=
          std::pow(1.0 / tmp_adjacency_matrix[vertex - 1][j], b) *
          std::pow(pheramone_matrix[vertex - 1][j], a);
    }
  }

  for (int j = 0; j < size; j++) {
    if (tmp_adjacency_matrix[vertex - 1][j] != 0 &&
        sum_feramont_distance != 0) {
      feramont_distance =
          std::pow(1.0 / tmp_adjacency_matrix[vertex - 1][j], b) *
          std::pow(pheramone_matrix[vertex - 1][j], a);
      probability_list[j] = feramont_distance / sum_feramont_distance;
    }
  }

  double probably = 0.0;
  for (int i = 0; i < probability_list.size(); i++) {
    probably += probability_list[i];
    if (abs((1.0 - probably)) <= 0.0000001) {
      err = 0;
    }
  }
  return err;
}

/// @brief - Метод, генерирующий дабл число

/// @param  min - минимальое число при рандоме > 0
/// @param  max - минимальое число при рандоме <= 1
/// @return double - сгенерированное дабл число от 0 до 1

double s21::GraphAlgorithms::VertexRandom(double min, double max) const {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> distribution(min, max);
  return distribution(gen);
}

/// @brief - Метод, который на основе листа вероятности, выбирает следующую
/// вершину

/// @param  probability_list - лист вероятности посещения той или иной вершины
/// @return int - 0 - у нас есть вершина, куда мы можем пойти, 1 - более
/// не посещенныъ вершин нет

int s21::GraphAlgorithms::SelectNextVertex(
    std::vector<double> probability_list) const noexcept {
  int vertex = 0;
  int size = probability_list.size();
  double random_c = VertexRandom(0.0, 1.0);
  if (random_c > 0 && random_c <= 1) {
    double sum_probability = 0;
    for (int j = 0; j != size && random_c - sum_probability >= 0.000001; j++) {
      sum_probability += probability_list[j];
      vertex++;
    }
  }
  return vertex;
}

/// @brief - Метод пересчитывающий матрицу ферамонов, исходя из пройденного
///  пути текущим муравьем

/// @param  pheramone_matrix - предыдущая матрица ферамонов
/// @param  temp_path - матрица смежности пройденного пути
/// @param  distance - расстояние, которое прошел текущий муравей

void s21::GraphAlgorithms::RecalculatePheramoneMatrix(
    std::vector<std::vector<double>> &pheramone_matrix,
    std::vector<std::vector<int>> temp_path, int distance) noexcept {
  const int q = 100.0;
  const double k = 0.01;

  const double p = 1 - k;
  const int size = pheramone_matrix.size();

  if (distance != 0) {
    double feromon_const = q / distance;
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        if (temp_path[i][j] != 0) {
          pheramone_matrix[i][j] = p * pheramone_matrix[i][j] + feromon_const;
        } else {
          pheramone_matrix[i][j] = p * (pheramone_matrix[i][j]);
        }
      }
    }
  }
}

/// @brief Данный метод IsAllVisited проверяет, все ли вершины графа были
/// посещены, на основе вектора visited_or_not, который содержит информацию о
/// посещении каждой вершины.
/// @param visited_of_not
/// @return true - если все вершины посещены, false - если еще не все вершины
/// посещены
bool s21::GraphAlgorithms::IsAllVisited(std::vector<bool> visited_of_not) {
  for (int it = 0; it < visited_of_not.size(); ++it) {
    if (visited_of_not[it] == false) {
      return false;
    }
  }
  return true;
}

/// @brief Данный метод ConvertToUndirected преобразует матрицу смежности
/// ориентированного графа в матрицу смежности неориентированного графа.
/// @param graph_matrix - исходная матрица
/// @return преобразованная матрица
std::vector<std::vector<int>> s21::GraphAlgorithms::ConvertToUndirected(
    const matrix graph_matrix) {
  int n = graph_matrix.size();
  matrix undirected_adj_matrix(n, std::vector<int>(n, 0));
  for (int i = 0; i < graph_matrix.size(); ++i) {
    for (int j = i + 1; j < graph_matrix.size(); ++j) {
      if (graph_matrix[i][j] != 0 || graph_matrix[j][i] != 0) {
        int weight = std::max(graph_matrix[i][j], graph_matrix[j][i]);
        undirected_adj_matrix[i][j] = weight;
        undirected_adj_matrix[j][i] = weight;
      }
    }
  }

  return undirected_adj_matrix;
}

/// @brief Данный метод GetMinCoordinats находит координаты (индексы) элемента с
/// минимальным значением в матрице working_matrix и возвращает их в виде пары
/// значений.
/// @param working_matrix - матрица для работы с весами ребер.
/// @return пара значений в которой первый эллемент - i, а второй элемент - j;
std::pair<int, int> s21::GraphAlgorithms::GetMinCoordinats(
    matrix working_matrix) {
  int min = inf, res_i = 0, res_j = 0;
  for (int i = 0; i < working_matrix.size(); ++i) {
    for (int j = 0; j < working_matrix[0].size(); ++j) {
      if (working_matrix[i][j] < min) {
        min = working_matrix[i][j];
        res_i = i;
        res_j = j;
      }
    }
  }
  std::pair<int, int> result(res_i, res_j);
  return result;
}

/// @brief Метод вычисляет суммарный вес остовного дерева, представленного в
/// виде матрицы least_spanning_tree.
/// @param least_spanning_tree - матрица смежности минимального остовного дерева
/// @return суммарный вес остовного дерева
int s21::GraphAlgorithms::GetGraphWeigt(matrix tmp_adjacency_matrix) {
  int result = 0;
  for (int i = 0; i < tmp_adjacency_matrix.size(); ++i) {
    for (int j = 0; j < tmp_adjacency_matrix.size(); ++j) {
      result += tmp_adjacency_matrix[i][j];
    }
  }
  return result;
}

/// @brief Метод проверки посещенных точек, в которм происходит итерация по
/// эллементам вектора посещенных точек и сравнение с текущей точкой.

/// @param visited_vertices - результатом работы алгоритмов,
/// вектор посещенных точек
/// @param current_vertix - текущая вершина графа.

bool s21::GraphAlgorithms::CheckVisited(std::vector<int> visited_vertices,
                                        int current_vertix) noexcept {
  for (int it = 0; it < visited_vertices.size(); ++it) {
    if (visited_vertices[it] == current_vertix) {
      return false;
    }
  }
  return true;
}

/// @brief Метод отображения пройденного пути в графе

/// @param visited_vertices - результатом работы алгоритмов,
/// вектор посещенных точек

void s21::GraphAlgorithms::PrintResultWay(
    std::vector<int> visited_vertices) noexcept {
  for (int it = 0; it < visited_vertices.size(); ++it) {
    if (it == visited_vertices.size() - 1) {
      std::cout << visited_vertices[it] << std::endl;
    } else {
      std::cout << visited_vertices[it] << " "
                << "->"
                << " ";
    }
  }
}

/// @brief Данный метод PrintAdjacencyMatrix выводит матрицу смежности в консоль
void s21::GraphAlgorithms::PrintAdjacencyMatrix(
    std::vector<std::vector<int>> matrix) noexcept {
  std::cout << "AdjacencyMatrix:" << std::endl;
  int size = matrix.size();
  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size; ++j) {
      std::cout << matrix[i][j] << " ";
    }
    std::cout << std::endl;
  }
}
