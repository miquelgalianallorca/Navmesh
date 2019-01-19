#include <stdafx.h>
#include <algorithm>
#include "pathNavmesh.h"
#include "pugixml/pugixml.hpp"

PathNavmesh::PathNavmesh() :
    isStepByStepModeOn(false),
    startNode(nullptr),
    endNode(nullptr),
    navmesh(nullptr)
{}

PathNavmesh::~PathNavmesh()
{
	navmesh = nullptr;

	for (Node* node : nodes)
		delete node;
	nodes.clear();
	path.clear();
}

void PathNavmesh::Load(std::vector<Pathfinder::NavPolygon>* _navmesh, std::vector<Pathfinder::Link>* _links)
{
	navmesh = _navmesh;
    links   = _links;

	// Clear previous nodes
	for (Node* node : nodes)
		delete node;
	nodes.clear();
	
	// Get nodes from navmesh
    for (int i = 0; i < navmesh->size(); ++i)
	{
        Pathfinder::NavPolygon& polygon = navmesh->at(i);

        // Each polygon is a node
        Node* node = new Node();
        node->cost  = 1;
        node->navmeshIndex = i;
        
        // Node position: centroid of verts
        USVec2D centroid(0.f, 0.f);
        for (const USVec2D& vert : polygon.m_verts)
            centroid += vert;
        centroid /= polygon.m_verts.size();
        node->pos = centroid;

        nodes.push_back(node);
	}
}

void PathNavmesh::ResetNodes()
{
	for (Node* node : nodes)
	{
		node->parent = nullptr;
		node->g = 0.f;
		node->f = 0.f;
	}

	path.clear();
	openList.clear();
	closedList.clear();
}

// Heuristics: Euclidean distance. Manhattan could give longer paths than necessary.
// Start and end in screen coords
bool PathNavmesh::AStar(USVec2D start, USVec2D end)
{	
	/*
	openlist
	closedlist
	add nodeInicio a openlist, coste 0
	while not openlist.vacio
		nodo = openlist.pop_shortest()
		if isGoal(nodo)
			return buildPath(nodo)
		else
			for nextNode : conexiones(node)
				if nextNode in closedlist
					continue
				if nextNode in openlist
					update cost in openlist if smaller
				else
					add to openlist con padre node

	buildPath(pathnode)
		while(padre(pathNode))
			add node(pathNode) to path
		return path
	*/

	ResetNodes();

	// Set start & end nodes
	startNode = GetClosestNode(start);
	endNode   = GetClosestNode(end);
	if (!startNode || !endNode)
		return false;
	openList.push_back(startNode);

	if (!isStepByStepModeOn)
	{
		while (openList.size() > 0)
		{
			bool Step = AStarStep();
			if (Step)
				return true;
		}
	}

	return false;
}

PathNavmesh::Node* PathNavmesh::GetClosestNode(const USVec2D& pos)
{
	Node* closestNode = nullptr;
	float closestDist = 99999.f;
	for (Node* node : nodes)
	{
		const float dist = node->pos.Dist(pos);
		if (dist < closestDist)
		{
			closestNode = node;
			closestDist = dist;
		}
	}
	return closestNode;
}

bool PathNavmesh::AStarStep()
{
	// Get shortest
	openList.sort(PathNavmesh::OrderByShortest);
	Node* node = openList.front();
	
	// Check goal
	if (node == endNode)
	{
		BuildPath(node);
		return true;
	}

	openList.pop_front();
	
	// Connections
	std::list<Node*> connections = GetConnections(node);
	for (Node* next : connections)
	{
		// Connection is in closed list
		auto it = std::find(closedList.begin(), closedList.end(), next);
		if (it != closedList.end())
		{
			continue;
		}

		// Connection is in open list
		it = std::find(openList.begin(), openList.end(), next);
		if (it != openList.end())
		{
			// Update cost in open list if smaller
			const float newCost = node->g + next->cost;
			if (newCost < (*it)->g)
			{
				(*it)->g = newCost;
				(*it)->parent = node;
			}
		}
		// Add to openlist with node as parent
		else
		{
			next->parent = node;
			next->g = next->cost + node->g;
			next->f = next->g + Heuristics(next, endNode);
			openList.push_back(next);
			closedList.push_back(node);
		}
	}

	return false;
}

void PathNavmesh::DrawDebug(const size_t& squareSize)
{
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();
	
	gfxDevice.SetPenColor(1.f, 0.f, 0.f, 1.f);
	if (startNode)
	{
		USRect rect;
		USVec2D pos = startNode->pos;
		rect.mXMin = pos.mX - 6.f;
		rect.mXMax = pos.mX + 6.f;
		rect.mYMin = pos.mY - 6.f;
		rect.mYMax = pos.mY + 6.f;
		MOAIDraw::DrawEllipseFill(rect, 6);
	}

	//// Paint node cost in red
	//for (const Node* node : nodes)
	//{
	//	if (node->cost > 0)
	//	{
	//		float alpha = (node->cost) / 10.f;
	//		gfxDevice.SetPenColor(alpha + .3f, 0.f, 0.f, .1f);

	//		float wPosX, wPosY;
	//		CoordToWorldPos(node->posX, node->posY, squareSize, wPosX, wPosY);
	//		MOAIDraw::DrawRectFill(wPosX, wPosY, wPosX + squareSize*2, wPosY + squareSize*2);
	//	}
	//}	

	//// Paint A*
	//if (isStepByStepModeOn)
	//{
	//	// Paint open nodes
	//	for (const Node* node : openList)
	//	{
	//		gfxDevice.SetPenColor(0.f, 0.f, .3f, .1f);
	//		float wPosX, wPosY;
	//		CoordToWorldPos(node->posX, node->posY, squareSize, wPosX, wPosY);
	//		MOAIDraw::DrawRectFill(wPosX, wPosY, wPosX + squareSize * 2, wPosY + squareSize * 2);
	//	}
	//	// Paint close nodes
	//	for (const Node* node : closedList)
	//	{
	//		gfxDevice.SetPenColor(0.f, 0.3f, 0.f, .1f);
	//		float wPosX, wPosY;
	//		CoordToWorldPos(node->posX, node->posY, squareSize, wPosX, wPosY);
	//		MOAIDraw::DrawRectFill(wPosX, wPosY, wPosX + squareSize * 2, wPosY + squareSize * 2);
	//	}
	//}
	//else
	//{
	//	// Paint path
	//	for (const Node* node : path)
	//	{
	//		gfxDevice.SetPenColor(.3f, .3f, 0.f, .1f);
	//		float wPosX, wPosY;
	//		CoordToWorldPos(node->posX, node->posY, squareSize, wPosX, wPosY);
	//		MOAIDraw::DrawRectFill(wPosX, wPosY, wPosX + squareSize * 2, wPosY + squareSize * 2);
	//	}
	//}
}

std::list<PathNavmesh::Node*> PathNavmesh::GetConnections(const Node* node)
{
	// cout << "Get connections of: " << posX << " " << posY;
	std::list<Node*> connections;
    if (!node) return connections;

    const Pathfinder::NavPolygon& polygon = navmesh->at(node->navmeshIndex);
    for (const Pathfinder::NavPolygon::Edge& edge : polygon.m_Edges)
    {
        const int neighbourIndex = edge.m_neighbourIndex;
        for (Node* n : nodes)
        {
            if (n->navmeshIndex == neighbourIndex)
            {
                connections.push_back(n);
                break;
            }
        }
    }

	return connections;
}

void PathNavmesh::BuildPath(Node* node)
{
	path.clear();

	cout << "Building path..." << endl;
	while (node->parent != nullptr)
	{
		path.push_back(node);
		node = node->parent;
	}
	path.push_back(node);

	cout << "Path built." << endl;
}

float PathNavmesh::Heuristics(const Node* next, const Node* goal)
{
	USVec2D nextPos = next->pos;
	USVec2D goalPos = goal->pos;
	return nextPos.Dist(goalPos);
}
