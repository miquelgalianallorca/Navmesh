#pragma once

class Character;
struct Params;

class Steering {
public:
    Steering() {}
    virtual ~Steering() {}

    virtual void GetAcceleration(
        Character &character,
        Params &params,
        USVec2D &outLinearAcceleration,
        float &outAngularAcceleration) = 0;
    virtual void DrawDebug() = 0;
};