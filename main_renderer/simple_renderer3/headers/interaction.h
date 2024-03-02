#pragma once

#include "vec.h"

// Forward declaration of BSDF class
class BSDF;

struct Interaction {
    // Position of interaction
    Vector3f p;
    // Normal of the surface at interaction
    Vector3f n;
    // The uv co-ordinates at the intersection point
    Vector2f uv;
    // The viewing direction in local shading frame
    Vector3f wi; 
    // Distance of intersection point from origin of the ray
    float t = 1e30f; 
    // Used for light intersection, holds the radiance emitted by the emitter.
    Vector3f emissiveColor = Vector3f(0.f, 0.f, 0.f);
    // BSDF at the shading point
    BSDF* bsdf;
    // Vectors defining the orthonormal basis
    Vector3f a, b, c;

    //
    Vector3f sampled;//generated in render, just passing pupose
    double area;// area of area_light

    bool didIntersect = false;

    //ORIGINAL: ISSUE  there's a potential issue with this implementation. The camera ray direction (cameraRay) is not necessarily perpendicular to the surface normal (n
    // Vector3f toWorld(Vector3f w,Vector3f cameraRay) { //Local to Global
    //         // TODO: Implement this
        
    //     Vector3f n=this->n;//**uses n

    //     Vector3f z_das=n;  //z_das is z' Global coordinate  
    //     Vector3f y_das= Normalize(Cross(z_das,cameraRay));
    //     Vector3f x_das= Normalize(Cross(y_das,z_das));

    //     Vector3f x(1,0,0);
    //     Vector3f y(0,1,0);
    //     Vector3f z(0,0,1);
        
    //     //we are  in local x,y,z(0,0,1) wrt to it doing sampling & convet it in x',y',z' global

    //     Vector3f r1(Dot(x_das,x),Dot(x_das,y),Dot(x_das,z));//row 1 of matrix
    //     Vector3f r2(Dot(y_das,x),Dot(y_das,y),Dot(y_das,z));
    //     Vector3f r3(Dot(z_das,x),Dot(z_das,y),Dot(z_das,z));
        
    //     return Vector3f(Dot(r1,w),Dot(r2,w) , Dot(r3,w));
    // }
    Vector3f toWorld(Vector3f w, Vector3f cameraRay) { // Local to Global:Gram-Schmidt process to orthogonalize the vectors.
        // Calculate the local coordinate axes
        Vector3f n = this->n; // **uses n
        Vector3f z_das = n;   // z_das is z' Global coordinate
        Vector3f y_das = Normalize(Cross(z_das, cameraRay));
        Vector3f x_das = Normalize(Cross(y_das, z_das));

        // Orthonormalize the local coordinate axes
        y_das = Normalize(y_das - Dot(y_das, z_das) * z_das);
        x_das = Normalize(x_das - Dot(x_das, y_das) * y_das - Dot(x_das, z_das) * z_das);

        // Form the transformation matrix
        Vector3f r1(x_das.x, y_das.x, z_das.x); // row 1 of matrix
        Vector3f r2(x_das.y, y_das.y, z_das.y); // row 2 of matrix
        Vector3f r3(x_das.z, y_das.z, z_das.z); // row 3 of matrix

        // Transform the input vector from local coordinates to global coordinates
        return Vector3f(Dot(r1, w), Dot(r2, w), Dot(r3, w));
        }


    // Vector3f toLocal(Vector3f w,Vector3f cameraRay) { //**Correct equation with edge case ,of normal || camera        // TODO: Implement this

    //     Vector3f n=this->n;

    //     Vector3f x(1,0,0);
    //     Vector3f y(0,1,0);
    //     Vector3f z(0,0,1);

    //     Vector3f z_das=n;  //z_das is z' Global coordinate
    //     Vector3f y_das= Normalize(Cross(z_das,cameraRay));
    //     Vector3f x_das= Normalize(Cross(y_das,z_das));

    //     //we are in world ,conveting its z' to z(0,0,1) 

    //     Vector3f r1(Dot(x,x_das),Dot(x,y_das),Dot(x,z_das));
    //     Vector3f r2(Dot(y,x_das),Dot(y,y_das),Dot(y,z_das));
    //     Vector3f r3(Dot(z,x_das),Dot(z,y_das),Dot(z,z_das));

    //     return Vector3f(Dot(r1,w),Dot(r2,w) , Dot(r3,w));
    // }

    Vector3f toLocal(Vector3f w, Vector3f cameraRay) {//above modified with Gram-Schmidt process to orthogonalize the vectors.
        // Calculate the local coordinate axes
        Vector3f n = this->n;
        Vector3f z_das = n;   // z_das is z' Global coordinate
        Vector3f y_das = Normalize(Cross(z_das, cameraRay));
        Vector3f x_das = Normalize(Cross(y_das, z_das));

        // Orthonormalize the local coordinate axes
        y_das = Normalize(y_das - Dot(y_das, z_das) * z_das);
        x_das = Normalize(x_das - Dot(x_das, y_das) * y_das - Dot(x_das, z_das) * z_das);

        // Form the transformation matrix
        Vector3f r1(x_das.x, x_das.y, x_das.z); // row 1 of matrix
        Vector3f r2(y_das.x, y_das.y, y_das.z); // row 2 of matrix
        Vector3f r3(z_das.x, z_das.y, z_das.z); // row 3 of matrix

        // Transform the input vector from global coordinates to local coordinates
    return Vector3f(Dot(r1, w), Dot(r2, w), Dot(r3, w));
}



};