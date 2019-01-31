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
    for (unsigned i = 0; i < navmesh->size(); ++i)
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
        centroid /= static_cast<float>(polygon.m_verts.size());
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
    endPos = end;

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

PathNavmesh::Node* PathNavmesh::GetClosestNode(const USVec2D& pos) const
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

void PathNavmesh::DrawDebug()
{
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();
	gfxDevice.SetPenColor(1.f, 0.f, .5f, 1.f);

    // Draw points to follow
    for (unsigned i = 0; i < pathPoints.size(); ++i)
    {
        USVec2D& pos = pathPoints.at(i);

        USRect rect;
        rect.mXMin = pos.mX - 6.f;
        rect.mXMax = pos.mX + 6.f;
        rect.mYMin = pos.mY - 6.f;
        rect.mYMax = pos.mY + 6.f;
        MOAIDraw::DrawEllipseFill(rect, 6);

        if (i < pathPoints.size() - 1)
            MOAIDraw::DrawLine(pos, pathPoints.at(i + 1));
    }
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
    pathPoints.clear();
        
	std::cout << "Building path..." << endl;
    pathPoints.push_back(endPos);
    while (node->parent != nullptr)
	{
		path.push_back(node);

        // Locations for pathfollowing
        pathPoints.push_back(node->pos);
        if (node->parent)
            pathPoints.push_back(GetMidEdgePoint(node));

		node = node->parent;
	}
	path.push_back(node);
    pathPoints.push_back(startNode->pos);
    
	std::cout << "Path built." << endl;
}

float PathNavmesh::Heuristics(const Node* next, const Node* goal) const
{
	USVec2D nextPos = next->pos;
	USVec2D goalPos = goal->pos;
	return nextPos.Dist(goalPos);
}

USVec2D PathNavmesh::GetMidEdgePoint(const Node* node) const
{
    USVec2D point;

    const int nodeIndex = node->navmeshIndex;
    const int parentIndex = node->parent->navmeshIndex;
    
    for (auto& edge : navmesh->at(parentIndex).m_Edges)
    {
        if (edge.m_neighbourIndex == nodeIndex)
        {
            point = edge.m_center;
            break;
        }
    }

    return point;
}
