#pragma once

#include "common.h"

struct Camera {
    Vector3d from, to, up;
    float fieldOfView;
    Vector2i imageResolution;

    float focusDistance = 1.f;
    float aspect;

    Vector3d u, v, w;
    Vector3d pixelDeltaU, pixelDeltaV;
    Vector3d upperLeft;

    Camera() {};
    Camera(Vector3d from, Vector3d to, Vector3d up, float fieldOfView, Vector2i imageResolution);

    Ray generateRay(int x, int y);
};

