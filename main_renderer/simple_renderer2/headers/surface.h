#pragma once

#include "common.h"
#include "texture.h"

struct NewInteraction
{
    Interaction Interactiondash;

    Texture curtexture; // added propert compared to interaction
};

struct Tri
{
    Vector3d v1, v2, v3;
    Vector2f uv1, uv2, uv3;
    Vector3d normal;
    Vector3d centroid;

    AABB bbox;
};

struct Surface
{
    std::vector<Vector3d> vertices, normals;
    std::vector<Vector3i> indices;
    std::vector<Vector2f> uvs;

    BVHNode *nodes;
    int numBVHNodes = 0;

    std::vector<Tri> tris;
    std::vector<uint32_t> triIdxs;
    AABB bbox;

    bool isLight;
    uint32_t shapeIdx;

    Vector3d diffuse;
    float alpha;

    Texture diffuseTexture, alphaTexture;

    void buildBVH();
    uint32_t getIdx(uint32_t idx);
    void updateNodeBounds(uint32_t nodeIdx);
    void subdivideNode(uint32_t nodeIdx);
    void intersectBVH(uint32_t nodeIdx, Ray &ray, Interaction &si);

    Interaction rayPlaneIntersect(Ray ray, Vector3d p, Vector3d n);
    Interaction rayTriangleIntersect(Ray ray, Vector3d v1, Vector3d v2, Vector3d v3, Vector3d n);
    Interaction rayIntersect(Ray &ray);

private:
    bool hasDiffuseTexture();
    bool hasAlphaTexture();
};

std::vector<Surface> createSurfaces(std::string pathToObj, bool isLight, uint32_t shapeIdx);