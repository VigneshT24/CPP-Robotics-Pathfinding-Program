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

constexpr int GRID_SIZE = 5;
constexpr char OBSTACLE = '0';
constexpr char GOAL = 'X';

int manhattanDistance(Position a, Position b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool isInsideGrid(Position pos) {
    return (pos.row >= 0 && pos.row < GRID_SIZE) && (pos.col >= 0 && pos.col < GRID_SIZE);
}

bool isWalkable(const std::vector<std::vector<Robot>>& grid, Position pos) {
    return isInsideGrid(pos) && grid[pos.row][pos.col].getName() != OBSTACLE;
}

std::vector<Position> findPathAStar(const std::vector<std::vector<Robot>>& grid, Position start, Position goal) {
    std::priority_queue<Node, std::vector<Node>, CompareNode> openSet;

    int gScore[GRID_SIZE][GRID_SIZE];
    Position parent[GRID_SIZE][GRID_SIZE];

    for (int r = 0; r < GRID_SIZE; r++) {
        for (int c = 0; c < GRID_SIZE; c++) {
            gScore[r][c] = std::numeric_limits<int>::max();
            parent[r][c] = {-1, -1};
        }
    }

    gScore[start.row][start.col] = 0;

    openSet.push({start, 0, manhattanDistance(start, goal)});

    while (!openSet.empty()) {
        Node curr = openSet.top();
        openSet.pop();
        
        if (curr.pos == goal) {
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