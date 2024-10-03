#include <algorithm>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_set>

struct Node {
    int x, y;
    float g, h;
    Node* parent;

    Node(const int x, const int y, Node* parent = nullptr) : x(x), y(y), g(0), h(0), parent(parent) {}

    float f() const {
        return g + h;
    }

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct NodeHash {
    size_t operator()(const Node* node) const {
        return std::hash<int>()(node->x) ^ std::hash<int>()(node->y);
    }
};

struct CompareNode {
    bool operator()(const Node* a, const Node* b) const {
        return a->f() > b->f();
    }
};

class AStar {
public:
    AStar(const int width, const int height, const std::vector<std::vector<int>>& grid)
        : width(width), height(height), grid(grid) {}

    std::vector<Node*> findPath(Node* start, const Node* goal) const {
        std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSet;
        std::unordered_set<Node*, NodeHash> closedSet;

        start->h = heuristic(start, goal);
        openSet.push(start);

        while (!openSet.empty()) {
            Node* current = openSet.top();
            openSet.pop();

            if (*current == *goal) {
                return reconstructPath(current);
            }

            closedSet.insert(current);

            for (Node* neighbor : getNeighbors(current)) {
                if (closedSet.find(neighbor) != closedSet.end()) {
                    continue;
                }

                const float tentative_g = current->g + distance(current, neighbor);

                if (tentative_g < neighbor->g ||
                    closedSet.find(neighbor) == closedSet.end()) {
                    neighbor->parent = current;
                    neighbor->g = tentative_g;
                    neighbor->h = heuristic(neighbor, goal);

                    openSet.push(neighbor);
                    closedSet.insert(neighbor);
                }
            }
        }

        return {};
    }

private:
    int width, height;
    std::vector<std::vector<int>> grid;

    static float heuristic(const Node* a, const Node* b) {
        return std::abs(a->x - b->x) + std::abs(a->y - b->y);
    }

    static float distance(const Node* a, const Node* b) {
        return std::sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
    }

    std::vector<Node*> getNeighbors(const Node* node) const {
        std::vector<Node*> neighbors;
        constexpr int dx[4] = {1, -1, 0, 0};
        constexpr int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i) {
            const int nx = node->x + dx[i];
            const int ny = node->y + dy[i];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height && grid[ny][nx] != 3) {
                neighbors.push_back(new Node(nx, ny));
            }
        }

        return neighbors;
    }

    static std::vector<Node*> reconstructPath(Node* node) {
        std::vector<Node*> path;
        while (node) {
            path.push_back(node);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

int main() {
    constexpr int width = 20, height = 20;
    std::vector<std::vector<int>> grid(height, std::vector<int>(width, 0));

    // Dodaj przeszkody
    std::vector<std::pair<int, int>> obstacles = {
        {10, 10},
        {10, 11},
        {10, 12}
    };

    for (const auto& obstacle : obstacles) {
        grid[obstacle.first][obstacle.second] = 3;
    }

    const auto start = new Node(0, 0);
    const auto goal = new Node(19, 19);
    grid[goal->y][goal->x] = 5;

    const AStar astar(width, height, grid);
    const std::vector<Node*> path = astar.findPath(start, goal);

    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (const Node* node : path) {
            std::cout << "(" << node->x << ", " << node->y << ")\n";
        }
    } else {
        std::cout << "No path found.\n";
    }

    delete start;
    delete goal;
    return 0;
}