#ifndef __CHARACTER_H__
#define __CHARACTER_H__

#include <moaicore/MOAIEntity2D.h>
#include "steering/params.h"
#include "steering/steeringPathFollow.h"

class Pathfinder;

class Character: public MOAIEntity2D
{
public:
    DECL_LUA_FACTORY(Character)
protected:
	virtual void OnStart();
	virtual void OnStop();
	virtual void OnUpdate(float step);
public:
	virtual void DrawDebug();

	Character();
	~Character();
	
	void SetLinearVelocity(float x, float y) { mLinearVelocity.mX = x; mLinearVelocity.mY = y;}
	void SetAngularVelocity(float angle) { mAngularVelocity = angle;}
    void SetPath(std::vector<USVec2D> path) { steeringPathFollow.SetPath(path); };
    void SetPathfinder(Pathfinder* _pathfinder) { pathfinder = _pathfinder; }
	
	USVec2D GetLinearVelocity()  const { return mLinearVelocity; }
	float   GetAngularVelocity() const { return mAngularVelocity; }
    float   GetMaxVelocity()     const { return mParams.max_velocity; }
    float   GetMaxAcceleration() const { return mParams.max_acceleration; }

private:
	USVec2D mLinearVelocity;
	float mAngularVelocity;
    Params mParams;
    SteeringPathFollow steeringPathFollow;
    USVec2D followingAcceleration;

    Pathfinder* pathfinder;

	// Lua configuration
public:
	virtual void RegisterLuaFuncs(MOAILuaState& state);
private:
	static int _setLinearVel(lua_State* L);
	static int _setAngularVel(lua_State* L);
};

#endif