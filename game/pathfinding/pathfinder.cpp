#include <stdafx.h>
#include "pathfinder.h"
#include "pugixml/pugixml.hpp"

Pathfinder::Pathfinder() : MOAIEntity2D()
{
	RTTI_BEGIN
		RTTI_EXTEND(MOAIEntity2D)
	RTTI_END

	// Read file
	const char* filename = "../sample/navmesh.xml";
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(filename);
	
	// Loading correct
	if (result)
	{
		// Load polygons
		pugi::xml_node polygonsNode = doc.child("navmesh").child("polygons");
		for (pugi::xml_node polygonNode = polygonsNode.child("polygon"); polygonNode; polygonNode = polygonNode.next_sibling("polygon"))
		{
			NavPolygon polygon;
			for (pugi::xml_node pointNode = polygonNode.child("point"); pointNode; pointNode = pointNode.next_sibling("point"))
			{
				float x = pointNode.attribute("x").as_float();
				float y = pointNode.attribute("y").as_float();
				polygon.m_verts.push_back(USVec2D(x, y));
			}
			m_Navmesh.push_back(polygon);
		}
		CreateEdges();

		// Load links
		pugi::xml_node linksNode = doc.child("navmesh").child("links");
		for (pugi::xml_node linkNode = linksNode.child("link"); linkNode; linkNode = linkNode.next_sibling("link"))
		{
			pugi::xml_node startNode = linkNode.child("start");
			int s_polygon = startNode.attribute("polygon").as_int();
			int s_edgestart = startNode.attribute("edgestart").as_int();
			int s_edgeend = startNode.attribute("edgeend").as_int();

			pugi::xml_node endNode = linkNode.child("end");
			int e_polygon = endNode.attribute("polygon").as_int();
			int e_edgestart = endNode.attribute("edgestart").as_int();
			int e_edgeend = endNode.attribute("edgeend").as_int();

			m_Navmesh[s_polygon].m_Edges[s_edgestart].m_pNeighbour = &m_Navmesh[e_polygon];
			m_Navmesh[e_polygon].m_Edges[e_edgestart].m_pNeighbour = &m_Navmesh[s_polygon];
		}
	}
	// Loading failed	
	else cout << result.description() << endl;
}

Pathfinder::~Pathfinder()
{

}

void Pathfinder::CreateEdges()
{
	for (NavPolygon& polygon : m_Navmesh)
	{
		size_t nVerts = polygon.m_verts.size();
		for (int i = 0; i < nVerts; ++i)
		{
			// Edge to previous vertex
			NavPolygon::Edge edge;
			if (i == 0)	edge.m_verts[0] = nVerts - 1;
			else		edge.m_verts[0] = i - 1;
			edge.m_verts[1] = i;

			polygon.m_Edges.push_back(edge);
		}
	}
}

void Pathfinder::UpdatePath()
{
	cout << "Update path" << endl;
}

void Pathfinder::DrawDebug()
{
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();
	gfxDevice.SetPenWidth(2.f);

	for (NavPolygon& polygon : m_Navmesh)
	{
		gfxDevice.SetPenColor(.2f, .3f, 0.f, .1f);
		MOAIDraw::DrawPolygonFilled(polygon.m_verts);
		gfxDevice.SetPenColor(0.f, 1.0f, 0.f, 1.f);
		MOAIDraw::DrawPolygon(polygon.m_verts);
	}
}

bool Pathfinder::PathfindStep()
{
    // returns true if pathfinding process finished
    return true;
}















//lua configuration ----------------------------------------------------------------//
void Pathfinder::RegisterLuaFuncs(MOAILuaState& state)
{
	MOAIEntity::RegisterLuaFuncs(state);

	luaL_Reg regTable [] = {
		{ "setStartPosition",		_setStartPosition},
		{ "setEndPosition",			_setEndPosition},
        { "pathfindStep",           _pathfindStep},
		{ NULL, NULL }
	};

	luaL_register(state, 0, regTable);
}

int Pathfinder::_setStartPosition(lua_State* L)
{
	MOAI_LUA_SETUP(Pathfinder, "U")

	float pX = state.GetValue<float>(2, 0.0f);
	float pY = state.GetValue<float>(3, 0.0f);
	self->SetStartPosition(pX, pY);
	return 0;
}

int Pathfinder::_setEndPosition(lua_State* L)
{
	MOAI_LUA_SETUP(Pathfinder, "U")

	float pX = state.GetValue<float>(2, 0.0f);
	float pY = state.GetValue<float>(3, 0.0f);
	self->SetEndPosition(pX, pY);
	return 0;
}

int Pathfinder::_pathfindStep(lua_State* L)
{
    MOAI_LUA_SETUP(Pathfinder, "U")

    self->PathfindStep();
    return 0;
}