#ifndef __PATHFINDER_H__
#define __PATHFINDER_H__

#include <moaicore/MOAIEntity2D.h>

class Pathfinder: public virtual MOAIEntity2D
{
public:
	struct NavPolygon
	{
		struct Edge
		{
			int         m_verts[2];   // Indexs of m_verts
			// NavPolygon* m_pNeighbour; // Other polygon
            int         m_neighbourIndex;
			USVec2D     m_center;     // Pathfinding: Center of edge
		};

		std::vector<USVec2D> m_verts;
		std::vector<Edge>    m_Edges;
		// TerrainType m_terreno;
		// Otros: centro, área, plano, etc

        int GetEdgeIndex(const int startVert, const int endVert) const;
	};

    struct Link
    {
        int start_polygon;
        int start_edgeStart;
        int start_edgeEnd;
    
        int end_polygon;
        int end_edgeStart;
        int end_edgeEnd;
    };

	Pathfinder();
	~Pathfinder();

	virtual void DrawDebug() override;

	void SetStartPosition(float x, float y);
	void SetEndPosition  (float x, float y);
	const USVec2D& GetStartPosition() const { return m_startPosition;}
	const USVec2D& GetEndPosition()   const { return m_endPosition;}

    bool PathfindStep();

private:
	void UpdatePath();
	bool LoadNavmesh(const char* filename);
	void CreateEdges();

	USVec2D m_startPosition;
	USVec2D m_endPosition;
	class PathNavmesh* m_path;

	std::vector<NavPolygon> m_navmesh;
    std::vector<Link> m_links;

    bool m_isStartPositionSet;
    bool m_isEndPositionSet;

	// Lua configuration -------------------------------------------
public:
	DECL_LUA_FACTORY(Pathfinder)
public:
	virtual void RegisterLuaFuncs(MOAILuaState& state);
private:
	static int _setStartPosition(lua_State* L);
	static int _setEndPosition(lua_State* L);
    static int _pathfindStep(lua_State* L);
	// -------------------------------------------------------------
};

#endif