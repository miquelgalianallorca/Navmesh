#pragma once

#include "pathfinder.h"

class PathNavmesh
{
public:
	struct Node
	{
        Node() : cost(0), parent(nullptr), g(0.f), f(0.f), navmeshIndex(-1) {}

		USVec2D pos; // World location
		int   cost;  // Traverse cost of grid location

		Node* parent;
		float g;             // Cost from origin to pos
		float f;             // f = g + h, h = Estimated cost from pos to goal

        int navmeshIndex;
	};    

    PathNavmesh();
	~PathNavmesh();
	
	void Load(std::vector<Pathfinder::NavPolygon>* navmesh, std::vector<Pathfinder::Link>* links);
	bool AStar(USVec2D start, USVec2D end);
	bool AStarStep();
	void BuildPath(Node* node);

	void DrawDebug();

private:
	// A* variables
	std::list<Node*> openList;
	std::list<Node*> closedList;
	std::list<Node*> path;
	Node* startNode;
	Node* endNode;

	// Navmesh variables
	std::vector<Pathfinder::NavPolygon>* navmesh;
    std::vector<Pathfinder::Link>* links;
	std::vector<Node*> nodes;

    std::vector<USVec2D> pathPoints;

	// false: P1 - true: P2
	bool isStepByStepModeOn;
	
	// Helper functions ----------------------------------------------------
	static bool OrderByShortest(const Node* first, const Node* second)
	{
		return (first->f < second->f);
	}
	
	std::list<Node*> GetConnections(const Node* node);
	void  ResetNodes();
	float Heuristics(const Node* next, const Node* goal) const;

	Node* GetClosestNode(const USVec2D& pos) const;
    USVec2D GetMidEdgePoint(const Node* node) const;
	// ---------------------------------------------------------------------
};