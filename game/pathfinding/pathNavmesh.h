#pragma once

#include "pathfinder.h"

class PathNavmesh
{
public:
	struct Node
	{
		USVec2D pos; // World location
		int   cost;  // Traverse cost of grid location

		Node* parent;
		float g;             // Cost from origin to pos
		float f;             // f = g + h, h = Estimated cost from pos to goal

		// Neighbouring polygons
		Pathfinder::NavPolygon* neighbors[2];
	};

	PathNavmesh() :
		isStepByStepModeOn(false)
	{}

	~PathNavmesh();
	
	void Load(std::vector<Pathfinder::NavPolygon>* navmesh);
	bool AStar(USVec2D start, USVec2D end);
	bool AStarStep();
	void BuildPath(Node* node);

	void DrawDebug(const size_t& squareSize);

private:
	// A* variables
	std::list<Node*> openList;
	std::list<Node*> closedList;
	std::list<Node*> path;
	Node* startNode;
	Node* endNode;

	// Navmesh variables
	std::vector<Pathfinder::NavPolygon>* navmesh;
	std::vector<Node*> nodes;

	// false: P1 - true: P2
	bool isStepByStepModeOn;
	
	// Helper functions ----------------------------------------------------
	static bool OrderByShortest(const Node* first, const Node* second)
	{
		return (first->f < second->f);
	}
	
	std::list<Node*> GetConnections();
	void  ResetNodes();
	float Heuristics(const Node* next, const Node* goal);

	Node* GetClosestNode(const USVec2D& pos);
	// ---------------------------------------------------------------------
};