#include <stdafx.h>
#include <algorithm>
#include "path.h"

Path::~Path()
{
	for (Node* node : nodes)
		delete node;
	nodes.clear();
	path.clear();
}

void Path::Load(const std::string& filename, const unsigned int rows, const unsigned int cols)
{
	this->rows = rows;
	this->cols = cols;

	// Clear previous nodes
	for (Node* node : nodes)
	{
		delete node;
	}
	nodes.clear();
	
	// Read file
	std::ifstream istream(filename, std::ios_base::binary);
	std::stringstream sstream;
	sstream << istream.rdbuf();
	mapSrc = sstream.str();
	// Sanitize
	mapSrc.erase(std::remove(mapSrc.begin(), mapSrc.end(), '\n'), mapSrc.end());
	mapSrc.erase(std::remove(mapSrc.begin(), mapSrc.end(), '\r'), mapSrc.end());

	// Get nodes from map
	for (size_t i = 0; i < mapSrc.length(); ++i)
	{
		Node* node = new Node();
		node->cost = static_cast<int>(mapSrc.at(i)) - static_cast<int>('0');
		node->posY = i / cols;
		node->posX = i % cols;
		node->parent = nullptr;
		node->g = 0.f;
		node->f = 0.f;
		nodes.push_back(node);
	}
}

void Path::ResetNodes()
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
// Start and end in map coords (not in screen coords)
bool Path::AStar(const float startX, const float startY, const float endX, const float endY)
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

	startNode = GetNodeAtPosition(static_cast<unsigned int>(startX), static_cast<unsigned int>(startY));
	endNode   = GetNodeAtPosition(static_cast<unsigned int>(endX),   static_cast<unsigned int>(endY));
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

bool Path::AStarStep()
{
	// Get shortest
	openList.sort(Path::OrderByShortest);
	Node* node = openList.front();
	
	// Check goal
	if (node == endNode)
	{
		BuildPath(node);
		return true;
	}

	openList.pop_front();
	
	// Connections
	std::list<Node*> connections = GetConnections(node->posX, node->posY);
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
			float newCost = node->g + next->cost;
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

void Path::DrawDebug(const size_t& squareSize)
{
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();
	
	// Paint node cost in red
	for (const Node* node : nodes)
	{
		if (node->cost > 0)
		{
			float alpha = (node->cost) / 10.f;
			gfxDevice.SetPenColor(alpha + .3f, 0.f, 0.f, .1f);

			float wPosX, wPosY;
			CoordToWorldPos(node->posX, node->posY, squareSize, wPosX, wPosY);
			MOAIDraw::DrawRectFill(wPosX, wPosY, wPosX + squareSize*2, wPosY + squareSize*2);
		}
	}	

	// Paint A*
	if (isStepByStepModeOn)
	{
		// Paint open nodes
		for (const Node* node : openList)
		{
			gfxDevice.SetPenColor(0.f, 0.f, .3f, .1f);
			float wPosX, wPosY;
			CoordToWorldPos(node->posX, node->posY, squareSize, wPosX, wPosY);
			MOAIDraw::DrawRectFill(wPosX, wPosY, wPosX + squareSize * 2, wPosY + squareSize * 2);
		}
		// Paint close nodes
		for (const Node* node : closedList)
		{
			gfxDevice.SetPenColor(0.f, 0.3f, 0.f, .1f);
			float wPosX, wPosY;
			CoordToWorldPos(node->posX, node->posY, squareSize, wPosX, wPosY);
			MOAIDraw::DrawRectFill(wPosX, wPosY, wPosX + squareSize * 2, wPosY + squareSize * 2);
		}
	}
	else
	{
		// Paint path
		for (const Node* node : path)
		{
			gfxDevice.SetPenColor(.3f, .3f, 0.f, .1f);
			float wPosX, wPosY;
			CoordToWorldPos(node->posX, node->posY, squareSize, wPosX, wPosY);
			MOAIDraw::DrawRectFill(wPosX, wPosY, wPosX + squareSize * 2, wPosY + squareSize * 2);
		}
	}
}

void Path::CoordToWorldPos(const unsigned int InPosX, const unsigned int InPosY, const size_t squareSize,
	float& OutPosX, float& OutPosY)
{
	// Top-left
	float offsetX = -1.f * squareSize * GetCols();
	float offsetY = -1.f * squareSize * GetRows();

	OutPosX = offsetX + static_cast<float>(InPosX) * squareSize * 2;
	OutPosY = offsetY + static_cast<float>(InPosY) * squareSize * 2;
}

void Path::WorldPosToCoord(const float InPosX, const float InPosY, const size_t squareSize,
	unsigned int& OutPosX, unsigned int& OutPosY)
{
	float offsetX = -1.f * squareSize * GetCols();
	float offsetY = -1.f * squareSize * GetRows();

	OutPosX = static_cast<unsigned int>((InPosX - offsetX) / (squareSize * 2));
	OutPosY = static_cast<unsigned int>((InPosY - offsetY) / (squareSize * 2));
}

std::list<Node*> Path::GetConnections(unsigned int posX, unsigned int posY)
{
	// cout << "Get connections of: " << posX << " " << posY;
	std::list<Node*> connections;
	for (Node* elem : nodes)
	{
		// cout << "Elem x: " << elem.posX << " y: " << elem.posY << endl;

		// Left
		if (elem->posX == posX - 1 && elem->posY == posY)
			connections.push_back(elem);
		// Right
		else if (elem->posX == posX + 1 && elem->posY == posY)
			connections.push_back(elem);
		// Up
		else if (elem->posX == posX && elem->posY == posY - 1)
			connections.push_back(elem);
		// Down
		else if (elem->posX == posX && elem->posY == posY + 1)
			connections.push_back(elem);
	}

	// cout << endl;

	return connections;
}

void Path::BuildPath(Node* node)
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

Node* Path::GetNodeAtPosition(unsigned int posX, unsigned int posY)
{
	for (Node* node : nodes)
	{
		if (node->posX == posX && node->posY == posY)
		{
			return node;
		}
	}
	return nullptr;
}

float Path::Heuristics(const Node* next, const Node* goal)
{
	USVec2D nextPos = USVec2D(next->posX, next->posY);
	USVec2D goalPos = USVec2D(goal->posX, goal->posY);

	return nextPos.Dist(goalPos);
}

bool Path::IsValidCoord(const USVec2D pos)
{
	unsigned int x = static_cast<unsigned int>(pos.mX);
	unsigned int y = static_cast<unsigned int>(pos.mY);
	if (x >= 0 && x < cols && y >= 0 && y < rows)
		return true;
	else
		return false;
}
