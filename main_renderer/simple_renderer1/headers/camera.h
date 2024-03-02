#pragma once

#include "common.h"

struct Camera {
    Vector3d from, to, up;
    double fieldOfView;
    Vector2i imageResolution;

    double focusDistance = 1.f;
    double aspect;

    Vector3d u, v, w;
    Vector3d pixelDeltaU, pixelDeltaV;
    Vector3d upperLeft;

    Camera() {};
    Camera(Vector3d from, Vector3d to, Vector3d up, double fieldOfView, Vector2i imageResolution);

    Ray generateRay(int x, int y);
};