#include "stdafx.h"
#include "steeringPathFollow.h"
#include "character.h"
#include "params.h"
#include <tinyxml.h>

void SteeringPathFollow::GetAcceleration(
    Character &character,
    Params &params,
    USVec2D &outLinearAcceleration,
    float &outAngularAcceleration)
{
    characterLocation = character.GetLoc();

    UpdatePursueLocation(params.look_ahead);
    Params p(params);
    p.targetPosition = pursueLocation;
    steeringSeek.GetAcceleration(character, p, outLinearAcceleration, outAngularAcceleration);
}

void SteeringPathFollow::DrawDebug() {
    //steeringSeek.DrawDebug();

    MOAIGfxDevice& gfxDevice = MOAIGfxDevice::Get();
    gfxDevice.SetPenColor(0.0f, 1.0f, 1.0f, 0.5f);
    gfxDevice.SetPenWidth(3.f);
    // Draw Path
    MOAIDraw::DrawPoint(0.0f, 0.0f);
    for (size_t i = 0; i < path.size(); i++) {
        MOAIDraw::DrawEllipseFill(path.at(i).mX, path.at(i).mY, 6.f, 6.f, 10);
        if (i > 0) {
            MOAIDraw::DrawLine(path.at(i - 1), path.at(i));
        }
    }

    // Draw Pursue target
    gfxDevice.SetPenColor(1.f, 1.f, 0.f, 1.f);
    MOAIDraw::DrawEllipseFill(pursueLocation.mX, pursueLocation.mY, 6.f, 6.f, 10);
}

bool SteeringPathFollow::ReadPath(const char *filename) {
    // Load XML
    TiXmlDocument doc(filename);
    if (!doc.LoadFile())
    {
        fprintf(stderr, "Couldn't read params from %s", filename);
        return false;
    }
    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem = hDoc.FirstChildElement().Element();
    if (!pElem)
    {
        fprintf(stderr, "Invalid format for %s", filename);
        return false;
    }

    TiXmlElement* child = hDoc.FirstChild("root").FirstChild("points").FirstChild("point").ToElement();
    for (child; child; child=child->NextSiblingElement()) {
        int x, y;
        child->Attribute("x", &x);
        child->Attribute("y", &y);
        path.push_back(USVec2D((float)x, (float)y));
    }

    return true;
}

void SteeringPathFollow::UpdatePursueLocation(float lookAhead) {
    // Find closest segment of path
    USVec2D closestPointLoc, nextPointLoc;
    USVec2D nextPointLoc2;
    float closestDist = 9999999.f;
    for (size_t i = 0; i < path.size(); ++i) {
        float dist = characterLocation.Dist(path[i]);
        if (dist < closestDist) {
            closestPointLoc = path[i];
            if (i < path.size() - 1) {
                nextPointLoc = path[i + 1];
                if (i < path.size() - 2)
                    nextPointLoc2 = path[i + 2];/////
            }
            else {
                nextPointLoc = path[i];
                nextPointLoc2 = path[i];//////
            }
            closestDist = dist;
        }
    }

    // End of path
    if (closestPointLoc.Equals(nextPointLoc)) {
        pursueLocation = closestPointLoc;
        return;
    }

    // Find closest point in segment
    USVec2D segmentDir = nextPointLoc - closestPointLoc;
    segmentDir.NormSafe();

    USVec2D charToSegmentStartDir = closestPointLoc - characterLocation;
    float dot = charToSegmentStartDir.Dot(segmentDir);
    // Projection after end of segment
    if (dot > (nextPointLoc - closestPointLoc).Length())
        pursueLocation = closestPointLoc;
    // Projection before beginning of segment
    else if (dot < 0)
        pursueLocation = nextPointLoc;
    // Projection within segment
    else pursueLocation = closestPointLoc + segmentDir * dot;

    // Add lookAhead to projection
    pursueLocation += segmentDir * lookAhead;

    // Clamp ahead position to path
    float distAhead = pursueLocation.Dist(characterLocation);
    float distNextPoint = nextPointLoc.Dist(characterLocation);
    float diffDist = distAhead - distNextPoint;
    if (diffDist > 0) {
        USVec2D nextSegment = nextPointLoc2 - nextPointLoc;
        nextSegment.NormSafe();
        nextSegment.Scale(diffDist);
        pursueLocation = nextPointLoc + nextSegment;
    }    
}