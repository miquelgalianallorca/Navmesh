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
			int m_verts[2]; // Indices de m_verts
			NavPolygon* m_pNeighbour;
		};
		std::vector<USVec2D> m_verts;
		std::vector<Edge> m_Edges;
		// TerrainType m_terreno;
		// Otros: centro, área, plano, etc
	};
	
	Pathfinder();
	~Pathfinder();

	virtual void DrawDebug();

	void SetStartPosition(float x, float y) { m_StartPosition = USVec2D(x, y); UpdatePath();}
	void SetEndPosition(float x, float y) { m_EndPosition = USVec2D(x, y); UpdatePath();}
	const USVec2D& GetStartPosition() const { return m_StartPosition;}
	const USVec2D& GetEndPosition() const { return m_EndPosition;}

    bool PathfindStep();
private:
	void UpdatePath();
private:
	USVec2D m_StartPosition;
	USVec2D m_EndPosition;

	std::vector<NavPolygon> m_Navmesh;

	void CreateEdges();


	// Lua configuration
public:
	DECL_LUA_FACTORY(Pathfinder)
public:
	virtual void RegisterLuaFuncs(MOAILuaState& state);
private:
	static int _setStartPosition(lua_State* L);
	static int _setEndPosition(lua_State* L);
    static int _pathfindStep(lua_State* L);
};


#endif