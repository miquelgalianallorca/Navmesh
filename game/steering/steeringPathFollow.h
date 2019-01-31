#pragma once

#include "steering.h"
#include "steeringSeek.h"

class SteeringPathFollow : public Steering {
public:
    SteeringPathFollow()  {}
    ~SteeringPathFollow() {}

    bool ReadPath(const char* filename);
    void GetAcceleration(
        Character &character,
        Params &params,
        USVec2D &outLinearAcceleration,
        float &outAngularAcceleration);

    void DrawDebug();

private:
    void UpdatePursueLocation(float lookAhead);

    // Delegate Steering
    SteeringSeek steeringSeek;

    std::vector<USVec2D> path;

    USVec2D characterLocation;
    USVec2D pursueLocation;
};