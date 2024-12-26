#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>
#include <cmath>
#include <limits>
#include <chrono>
#include <tuple>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace std;
using namespace std::chrono;

// Кастомная хэш-функция для std::pair
struct CustomHash {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const {
        auto hash1 = hash<T1>{}(p.first);
        auto hash2 = hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1);
    }
};

using Point = pair<double, double>;
using Link = pair<Point, double>;
using Map = unordered_map<Point, vector<Link>, CustomHash>;

// Функция для подсчёта расстояния между двумя точками
double calculateDistance(const Point& a, const Point& b) {
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

// Функция для загрузки карты из файла
Map loadMap(const string& filename) {
    Map map;
    ifstream file(filename); // Открытие файла
    if (!file.is_open()) {
        cerr << "Ошибка при открытии файла: " << filename << endl;
        return map;
    }

    string line;
    while (getline(file, line)) {
        istringstream stream(line);
        string parent, children;
        getline(stream, parent, ':');

        Point parentPoint;
        sscanf(parent.c_str(), "%lf,%lf", &parentPoint.first, &parentPoint.second);

        while (getline(stream, children, ';')) {
            Point childPoint;
            double weight;
            sscanf(children.c_str(), "%lf,%lf,%lf", &childPoint.first, &childPoint.second, &weight);
            map[parentPoint].push_back({childPoint, weight});
        }
    }

    file.close();
    return map;
}

// Рекурсивный DFS
bool depthFirstSearch(const Map& map, const Point& start, const Point& end, vector<Point>& path, unordered_map<Point, bool, CustomHash>& visited) {
    if (start == end) {
        path.push_back(start);
        return true;
    }

    visited[start] = true;
    path.push_back(start);

    if (map.find(start) == map.end()) {
        path.pop_back();
        return false;
    }

    for (const auto& link : map.at(start)) {
        const Point& neighbor = link.first;
        if (!visited[neighbor] && depthFirstSearch(map, neighbor, end, path, visited)) {
            return true;
        }
    }

    path.pop_back();
    return false;
}

// Итеративный BFS
bool breadthFirstSearch(const Map& map, const Point& start, const Point& end, vector<Point>& path) {
    unordered_map<Point, Point, CustomHash> parents;
    queue<Point> toVisit; // Очередь для обхода вершин
    unordered_map<Point, bool, CustomHash> visited;

    toVisit.push(start);
    visited[start] = true;

    while (!toVisit.empty()) {
        Point current = toVisit.front();
        toVisit.pop();

        if (current == end) {
            Point temp = end;
            while (temp != start) {
                path.push_back(temp);
                temp = parents[temp];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return true;
        }

        if (map.find(current) == map.end()) {
            continue;
        }

        for (const auto& link : map.at(current)) {
            const Point& neighbor = link.first;
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parents[neighbor] = current;
                toVisit.push(neighbor);
            }
        }
    }

    return false;
}

// Алгоритм Дейкстры
bool dijkstra(const Map& map, const Point& start, const Point& end, vector<Point>& path) {
    unordered_map<Point, double, CustomHash> distances;
    unordered_map<Point, Point, CustomHash> parents;
    auto compare = [](const pair<Point, double>& a, const pair<Point, double>& b) { return a.second > b.second; };
    priority_queue<pair<Point, double>, vector<pair<Point, double>>, decltype(compare)> queue(compare);

    for (const auto& node : map) {
        distances[node.first] = numeric_limits<double>::infinity();
    }

    distances[start] = 0;
    queue.push({start, 0});
    while (!queue.empty()) {
        Point current = queue.top().first;
        queue.pop();

        if (current == end) {
            Point temp = end;
            while (temp != start) {
                path.push_back(temp);
                temp = parents[temp];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return true;
        }

        if (map.find(current) == map.end()) {
            continue;
        }

        for (const auto& link : map.at(current)) {
            const Point& neighbor = link.first;
            double weight = link.second;

            if (distances[current] + weight < distances[neighbor]) {
                distances[neighbor] = distances[current] + weight;
                parents[neighbor] = current;
                queue.push({neighbor, distances[neighbor]});
            }
        }
    }

    return false;
}

// Функция эвристики (евклидово расстояние)
double heuristic(const Point& a, const Point& b) {
    return calculateDistance(a, b);
}

// Алгоритм A*
bool aStarSearch(const Map& map, const Point& start, const Point& end, vector<Point>& path) {
    unordered_map<Point, double, CustomHash> gScore;
    unordered_map<Point, double, CustomHash> fScore;
    unordered_map<Point, Point, CustomHash> parents;
    auto compare = [](const pair<Point, double>& a, const pair<Point, double>& b) { return a.second > b.second; };
    priority_queue<pair<Point, double>, vector<pair<Point, double>>, decltype(compare)> openSet(compare);

    for (const auto& node : map) {
        gScore[node.first] = numeric_limits<double>::infinity();
        fScore[node.first] = numeric_limits<double>::infinity();
    }

    gScore[start] = 0;
    fScore[start] = heuristic(start, end);
    openSet.push({start, fScore[start]});

    while (!openSet.empty()) {
        Point current = openSet.top().first;
        openSet.pop();

        if (current == end) {
            Point temp = end;
            while (temp != start) {
                path.push_back(temp);
                temp = parents[temp];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return true;
        }

        if (map.find(current) == map.end()) {
            continue;
        }

        for (const auto& link : map.at(current)) {
            const Point& neighbor = link.first;
            double weight = link.second;

            double tentative_gScore = gScore[current] + weight;
            if (tentative_gScore < gScore[neighbor]) {
                gScore[neighbor] = tentative_gScore;
                fScore[neighbor] = tentative_gScore + heuristic(neighbor, end);
                parents[neighbor] = current;
                openSet.push({neighbor, fScore[neighbor]});
            }
        }
    }

    return false;
}

// Основная программа
int main() {
    Map map = loadMap("spb_graph.txt");

    if (map.empty()) {
        cerr << "Карта не загружена." << endl;
        return 1;
    }

    Point start = {30.3585261, 59.8864419};
    Point end = {30.3027079, 59.9570161};
    vector<Point> route;

    // DFS
    unordered_map<Point, bool, CustomHash> visited;
    auto start_time = high_resolution_clock::now();
    if (depthFirstSearch(map, start, end, route, visited)) {
        cout << "DFS маршрут найден: ";
        for (const auto& point : route) {
            cout << "(" << point.first << ", " << point.second << ") ";
        }
        cout << endl;
    } else {
        cout << "DFS маршрут не найден." << endl;
    }
    auto end_time = high_resolution_clock::now();
    cout << "DFS время: " << duration_cast<milliseconds>(end_time - start_time).count() << " ms" << endl;

    // BFS
    route.clear();
    start_time = high_resolution_clock::now();
    if (breadthFirstSearch(map, start, end, route)) {
        cout << "BFS маршрут найден: ";
        for (const auto& point : route) {
            cout << "(" << point.first << ", " << point.second << ") ";
        }
        cout << endl;
    } else {
        cout << "BFS маршрут не найден." << endl;
    }
    end_time = high_resolution_clock::now();
    cout << "BFS время: " << duration_cast<milliseconds>(end_time - start_time).count() << " ms" << endl;

    // Dijkstra
    route.clear();
    start_time = high_resolution_clock::now();
    if (dijkstra(map, start, end, route)) {
        cout << "Дейкстра маршрут найден: ";
        for (const auto& point : route) {
            cout << "(" << point.first << ", " << point.second << ") ";
        }
        cout << endl;
    } else {
        cout << "Дейкстра маршрут не найден." << endl;
    }
    end_time = high_resolution_clock::now();
    cout << "Дейкстра время: " << duration_cast<milliseconds>(end_time - start_time).count() << " ms" << endl;

    // A*
    route.clear();
    start_time = high_resolution_clock::now();
    if (aStarSearch(map, start, end, route)) {
        cout << "A* маршрут найден: ";
        for (const auto& point : route) {
            cout << "(" << point.first << ", " << point.second << ") ";
        }
        cout << endl;
    } else {
        cout << "A* маршрут не найден." << endl;
    }
    end_time = high_resolution_clock::now();
    cout << "A* время: " << duration_cast<milliseconds>(end_time - start_time).count() << " ms" << endl;

    return 0;
}