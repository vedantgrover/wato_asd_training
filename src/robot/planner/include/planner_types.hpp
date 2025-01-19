#ifndef PLANNER_TYPES_HPP_
#define PLANNER_TYPES_HPP_

#include <functional>

struct CellIndex {
    int x;
    int y;
    
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    
    bool operator==(const CellIndex &other) const {
        return (x == other.x && y == other.y);
    }
    
    bool operator!=(const CellIndex &other) const {
        return (x != other.x || y != other.y);
    }
};

// Hash function for CellIndex
struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

// Structure representing a node in the A* open set
struct AStarNode {
    CellIndex index;
    double f_score;  // f = g + h
    
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue
struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};
#endif
