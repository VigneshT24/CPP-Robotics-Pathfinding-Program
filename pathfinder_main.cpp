#include <iostream>
#include "robotObject.hpp"
#include <ctime>
#include <cmath>
#include <vector>
#include <queue>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <algorithm>
#include <limits>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

struct Position {
    int row;
    int col;

    bool operator==(const Position& other) const {
        return row == other.row && col == other.col;
    }
};

struct Node {
    Position pos;
    int gCost; // cost from start
    int hCost; // estimated distance to goal

    Node(Position new_pos, int new_gCost, int new_hCost) {
        pos = new_pos;
        gCost = new_gCost;
        hCost = new_hCost;
    }

    int fCost() const {
        return gCost + hCost;
    }
};

struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.fCost() > b.fCost();
    }
};

constexpr char OBSTACLE = '0';
constexpr char GOAL = 'X';
constexpr int MAX_DIFFICULTY = 10;

int manhattanDistance(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool isInsideGrid(const std::vector<std::vector<Robot>>& grid, Position pos) {
    return (pos.row >= 0 && pos.row < grid.size()) && (pos.col >= 0 && pos.col < grid[0].size());
}

bool isWalkable(const std::vector<std::vector<Robot>>& grid, Position pos) {
    return isInsideGrid(grid, pos) && grid[pos.row][pos.col].getName() != OBSTACLE;
}

std::vector<Position> findPathAStar(const std::vector<std::vector<Robot>>& grid, Position start, Position goal) {
    std::priority_queue<Node, std::vector<Node>, CompareNode> openSet;
    int grid_size = grid.size();

    std::vector<std::vector<int>> gScore(grid_size, std::vector<int>(grid_size, std::numeric_limits<int>::max()));
    std::vector<std::vector<Position>> parent(grid_size, std::vector<Position>(grid_size, {-1, -1}));
    bool pathFound = false;

    gScore[start.row][start.col] = 0;

    openSet.push({start, 0, manhattanDistance(start, goal)});

    while (!openSet.empty()) {
        Node curr = openSet.top();
        openSet.pop();
        
        if (curr.pos == goal) {
            pathFound = true;
            break;
        }
        
        // top, right, bottom, left
        Position neighbor[4] = {{curr.pos.row - 1, curr.pos.col},
                                {curr.pos.row, curr.pos.col + 1},
                                {curr.pos.row + 1, curr.pos.col},
                                {curr.pos.row, curr.pos.col - 1}};

        for (Position next : neighbor) {
            if (!isWalkable(grid, next)) continue;
            int newGCost = curr.gCost + 1;
            int newHCost = manhattanDistance(next, goal);
            
            if (newGCost < gScore[next.row][next.col]) {
                gScore[next.row][next.col] = newGCost;
                parent[next.row][next.col] = curr.pos;
                openSet.push({next, newGCost, newHCost});
            }
        }
    }

    if (!pathFound) return {};

    std::vector<Position> path;
    Position current = goal;

    while (!(current == start)) {
        path.push_back(current);
        current = parent[current.row][current.col];
    }

    path.push_back(start);

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<std::vector<Robot>> createGrid(
                                char robotName, 
                                int gridSize, 
                                int difficulty, 
                                const std::string& robotType,
                                Position start_pos = {0, 0}, 
                                Position goal_pos = {-1, -1}
) {
    
    std::vector<std::vector<Robot>> grid(gridSize, std::vector<Robot>(gridSize, Robot(' ', "")));
    if (goal_pos == Position{-1, -1}) {
        goal_pos = {gridSize - 1, gridSize - 1};
    }

    for (int r = 0; r < gridSize; r++) {
        for (int c = 0; c < gridSize; c++) {
            Position current{r, c};

            if (current == start_pos) {
                grid[r][c] = Robot(' ', "");
            }
            else if (current == goal_pos) {
                grid[r][c] = Robot(GOAL, "goal");
            }
            else if (std::rand() % MAX_DIFFICULTY < difficulty) {
                grid[r][c] = Robot(OBSTACLE, "obstacle");
            }
        }
    }
    
    return grid;
}

ftxui::Element renderGrid(const std::vector<std::vector<Robot>>& grid, Position robotPos, char robotName) {
    ftxui::Elements rows;

    for (int r = 0; r < grid.size(); r++) {
        ftxui::Elements cells;

        for (int c = 0; c < grid[r].size(); c++) {
            Position current{r, c};

            char name = grid[r][c].getName();

            // robot visually overrides whatever is underneath
            if (current == robotPos) {
                name = robotName;
            }

            auto cell =
                ftxui::text(std::string(1, name))
                | ftxui::center
                | ftxui::border
                | ftxui::size(ftxui::WIDTH, ftxui::EQUAL, 5)
                | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 3);

            if (current == robotPos) {
                cell = cell | ftxui::color(ftxui::Color::Blue);
            }
            else if (name == OBSTACLE) {
                cell = cell | ftxui::color(ftxui::Color::Red);
            }
            else if (name == GOAL) {
                cell = cell | ftxui::color(ftxui::Color::Green);
            }

            cells.push_back(cell);
        }

        rows.push_back(ftxui::hbox(std::move(cells)));
    }

    return ftxui::vbox(std::move(rows));
}

int main() {
    std::srand(std::time(nullptr));

    char robotName = 'R';
    std::string robotType = "Test";

    int gridSize = 10;
    int difficulty = 4;

    Position start{0, 0};
    Position goal{gridSize - 1, gridSize - 1};

    auto grid = createGrid(
        robotName,
        gridSize,
        difficulty,
        robotType,
        start,
        goal
    );

    auto path = findPathAStar(grid, start, goal);

    Position robotPos = start;
    std::string status = path.empty() ? "No path found." : "Path found.";

    auto screen = ftxui::ScreenInteractive::Fullscreen();

    auto renderer = ftxui::Renderer([&] {
        return ftxui::vbox({
            ftxui::text("A* Pathfinding") | ftxui::bold | ftxui::center,
            ftxui::separator(),
            renderGrid(grid, robotPos, robotName)
            | ftxui::center,
            ftxui::separator(),
            ftxui::text(status) | ftxui::center,
            ftxui::text("Press Q to quit") | ftxui::center
        });
    });

    auto app = ftxui::CatchEvent(renderer, [&](ftxui::Event event) {
        if (event == ftxui::Event::Character('q') ||
            event == ftxui::Event::Character('Q')) {
            screen.ExitLoopClosure()();
            return true;
        }
        return false;
    });

    std::thread animation_thread([&] {
        if (path.empty()) {
            screen.PostEvent(ftxui::Event::Custom);
            return;
        }

        for (const Position& step : path) {
            robotPos = step;
            screen.PostEvent(ftxui::Event::Custom);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        status = "Goal reached.";
        screen.PostEvent(ftxui::Event::Custom);
    });

    screen.Loop(app);

    if (animation_thread.joinable()) {
        animation_thread.join();
    }

    return 0;
}