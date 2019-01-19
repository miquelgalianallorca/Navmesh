#include <stdafx.h>
#include "pathfinder.h"
#include "pathNavmesh.h"
#include "pugixml/pugixml.hpp"

// P8

Pathfinder::Pathfinder() :
    MOAIEntity2D(),
    m_isStartPositionSet(false),
    m_isEndPositionSet(false)
{
	RTTI_BEGIN
		RTTI_EXTEND(MOAIEntity2D)
	RTTI_END

	// Load navmesh
	const char* filename = "../sample/navmesh.xml";
	if (LoadNavmesh(filename))
		cout << "Navmesh loaded." << endl;

	// Load path
	m_path = new PathNavmesh();
	m_path->Load(&m_navmesh, &m_links);
}

Pathfinder::~Pathfinder()
{
	delete m_path;
}

bool Pathfinder::LoadNavmesh(const char* filename)
{
	// Read file
	pugi::xml_document doc;
	const pugi::xml_parse_result result = doc.load_file(filename);

	// Loading correct
	if (result)
	{
		// Load polygons
		const pugi::xml_node polygonsNode = doc.child("navmesh").child("polygons");
		for (pugi::xml_node polygonNode = polygonsNode.child("polygon"); polygonNode; polygonNode = polygonNode.next_sibling("polygon"))
		{
			NavPolygon polygon;
			for (pugi::xml_node pointNode = polygonNode.child("point"); pointNode; pointNode = pointNode.next_sibling("point"))
			{
				const float x = pointNode.attribute("x").as_float();
				const float y = pointNode.attribute("y").as_float();
				polygon.m_verts.push_back(USVec2D(x, y));
			}
			m_navmesh.push_back(polygon);
		}
		CreateEdges();

		// Load links
		const pugi::xml_node linksNode = doc.child("navmesh").child("links");
		for (pugi::xml_node linkNode = linksNode.child("link"); linkNode; linkNode = linkNode.next_sibling("link"))
		{
            Link link;
			const pugi::xml_node startNode = linkNode.child("start");
            link.start_polygon   = startNode.attribute("polygon").as_int();
			link.start_edgeStart = startNode.attribute("edgestart").as_int();
			link.start_edgeEnd   = startNode.attribute("edgeend").as_int();

			const pugi::xml_node endNode = linkNode.child("end");
			link.end_polygon   = endNode.attribute("polygon").as_int();
			link.end_edgeStart = endNode.attribute("edgestart").as_int();
			link.end_edgeEnd   = endNode.attribute("edgeend").as_int();

            // -- ??
			/*m_navmesh[link.start_polygon].m_Edges[link.start_edgeStart].m_pNeighbour = &m_navmesh[link.end_polygon];
			m_navmesh[link.end_polygon].m_Edges[link.end_edgeStart].m_pNeighbour = &m_navmesh[link.start_polygon];*/
            m_navmesh[link.start_polygon].m_Edges[link.start_edgeStart].m_neighbourIndex = link.end_polygon;
            m_navmesh[link.end_polygon].m_Edges[link.end_edgeStart].m_neighbourIndex = link.start_polygon;
            // -- ??

            m_links.push_back(link);
		}
	}
	// Loading failed	
	else cout << result.description() << endl;

	return result;
}

void Pathfinder::CreateEdges()
{
	for (NavPolygon& polygon : m_navmesh)
	{
		const size_t nVerts = polygon.m_verts.size();
		for (size_t i = 0; i < nVerts; ++i)
		{
			// Edge to previous vertex
			NavPolygon::Edge edge;
			if (i == 0)	edge.m_verts[0] = nVerts - 1;
			else		edge.m_verts[0] = i - 1;
			edge.m_verts[1] = i;

			// Center of edge
			const float centerX = (polygon.m_verts[edge.m_verts[0]].mX + polygon.m_verts[edge.m_verts[1]].mX) / 2;
			const float centerY = (polygon.m_verts[edge.m_verts[0]].mY + polygon.m_verts[edge.m_verts[1]].mY) / 2;
			edge.m_center = USVec2D(centerX, centerY);

			polygon.m_Edges.push_back(edge);
		}
	}
}

void Pathfinder::UpdatePath()
{
	cout << "Update path" << endl;
    
	// Check if valid path
	if (!m_isStartPositionSet || !m_isEndPositionSet)
	{
		cout << "Path doesn't have start and end." << endl;
		return;
	}

    if (m_path)
        m_path->AStar(m_startPosition, m_endPosition);
}

void Pathfinder::SetStartPosition(float x, float y)
{ 
	m_startPosition = USVec2D(x, y);
    m_isStartPositionSet = true;
	UpdatePath();
}

void Pathfinder::SetEndPosition(float x, float y)
{
	m_endPosition = USVec2D(x, y);
    m_isEndPositionSet = true;
    UpdatePath();
}

void Pathfinder::DrawDebug()
{
	MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();
	
	// Draw navmesh
	gfxDevice.SetPenWidth(2.f);
	for (NavPolygon& polygon : m_navmesh)
	{
		gfxDevice.SetPenColor(.2f, .3f, 0.f, .1f);
		MOAIDraw::DrawPolygonFilled(polygon.m_verts);
		gfxDevice.SetPenColor(0.f, 1.0f, 0.f, 1.f);
		MOAIDraw::DrawPolygon(polygon.m_verts);

		// Draw edge centers
		for (auto edge : polygon.m_Edges)
		{
			gfxDevice.SetPenColor(0.f, 0.f, 1.f, .5f);
			USRect rect;
			rect.mXMin = edge.m_center.mX - 3.f;
			rect.mXMax = edge.m_center.mX + 3.f;
			rect.mYMin = edge.m_center.mY - 3.f;
			rect.mYMax = edge.m_center.mY + 3.f;
			MOAIDraw::DrawEllipseFill(rect, 6);
		}
	}

	// Draw start pos
    const unsigned posHalfSize = 5;
	gfxDevice.SetPenColor(1.f, 0.f, 0.f, 1.f);
	MOAIDraw::DrawRectFill(m_startPosition.mX - posHalfSize, m_startPosition.mY - posHalfSize, m_startPosition.mX + posHalfSize, m_startPosition.mY + posHalfSize);
	// Draw end pos
    gfxDevice.SetPenColor(0.f, 0.f, 1.f, 1.f);
    MOAIDraw::DrawRectFill(m_endPosition.mX - posHalfSize, m_endPosition.mY - posHalfSize, m_endPosition.mX + posHalfSize, m_endPosition.mY + posHalfSize);
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