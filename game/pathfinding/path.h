#pragma once

struct Node
{
	Node() {}
	Node(unsigned int _posX, unsigned int _posY, unsigned int _cost) :
		posX(_posX),
		posY(_posY),
		cost(_cost),
		parent(nullptr)
	{}

	unsigned int posX;
	unsigned int posY;   // Map grid location
	int   cost;          // Traverse cost of grid location
	
	Node* parent;	
	float g;             // Cost from origin to pos
	float f;             // f = g + h, h = Estimated cost from pos to goal
};

class Path
{
public:
	Path() :
		rows(0),
		cols(0),
		isStepByStepModeOn(false)
	{}

	~Path();
	
	void Load(const std::string& filename, const unsigned int rows, const unsigned int cols);
	bool AStar(const float startX, const float startY, const float endX, const float endY);
	bool AStarStep();
	void BuildPath(Node* node);

	std::string  GetMap()  const { return mapSrc; }
	unsigned int GetRows() const { return rows;   }
	unsigned int GetCols() const { return cols;   }

	void DrawDebug(const size_t& squareSize);

	bool IsValidCoord(const USVec2D pos);
	void CoordToWorldPos(const unsigned int InPosX, const unsigned int InPosY, const size_t squareSize,
		float& OutPosX, float& OutPosY);
	void WorldPosToCoord(const float InPosX, const float InPosY, const size_t squareSize, 
		unsigned int& OutPosX, unsigned int& OutPosY);

private:
	// Grid variables
	unsigned int rows, cols;
	std::string mapSrc;
	std::vector<Node*> nodes;

	// A* variables
	std::list<Node*> openList;
	std::list<Node*> closedList;
	std::list<Node*> path;
	Node* startNode;
	Node* endNode;

	// false: P1 - true: P2
	bool isStepByStepModeOn;
	
	// Helper functions ----------------------------------------------------
	static bool OrderByShortest(const Node* first, const Node* second)
	{
		return (first->f < second->f);
	}
	
	std::list<Node*> GetConnections(unsigned int posX, unsigned int posY);
	Node* GetNodeAtPosition(unsigned int posX, unsigned int posY);
	void  ResetNodes();
	float Heuristics(const Node* next, const Node* goal);
	// ---------------------------------------------------------------------
};