#include "camera.h"
#include "vec.h"

Camera::Camera(Vector3d from, Vector3d to, Vector3d up, double fieldOfView, Vector2i imageResolution): from(from),
    to(to),
    up(up), //vertival ,not necessary in camera's plane
    fieldOfView(fieldOfView),
    imageResolution(imageResolution)
{
    this->aspect = imageResolution.x / float(imageResolution.y);

    // Determine viewport dimensions in 3D
    float fovRadians = fieldOfView * M_PI / 180.f;
    float h = std::tan(fovRadians / 2.f);
    float viewportHeight = 2.f * h * this->focusDistance;
    float viewportWidth = viewportHeight * this->aspect;

    // Calculate basis vectors of the camera for the given transform
    this->w = Normalize(this->from - this->to); //to camera opposite
    this->u = Normalize(Cross(up, this->w)); //right
    this->v = Normalize(Cross(this->w, this->u)); //vertical up in camera surface plane

    // Pixel delta vectors
    Vector3d viewportU = viewportWidth * this->u;   //viewport vector: magnitude * direction
    Vector3d viewportV = viewportHeight * (-this->v);

    this->pixelDeltaU = viewportU / float(imageResolution.x); //x change/per pixel
    this->pixelDeltaV = viewportV / float(imageResolution.y); //y change/per pixel

    // Upper left
    this->upperLeft = from - this->w * this->focusDistance - viewportU / 2.f - viewportV / 2.f;
}

Ray Camera::generateRay(int x, int y)
{
    Vector3d pixelCenter = this->upperLeft + 0.5f * (this->pixelDeltaU + this->pixelDeltaV);//top-left pixel centre
    pixelCenter = pixelCenter + x * this->pixelDeltaU + y * this->pixelDeltaV;//curreach pixel_centre

    Vector3d direction = Normalize(pixelCenter - this->from);//direction:pointing ray to pixel unit vector

    return Ray(this->from, direction);//camera pos,camera to pixel unit vector
}