#pragma once

#include "common.h"
#include "texture.h"


struct Tri{
    Vector3d  vertex0, vertex1, vertex2;  ;
    Vector3d Tricentroid;
    Vector3d Normal;
};
 struct BVHNodeT
{
	Vector3d aabbMin, aabbMax;
	int leftFirst, triCount;
	bool isLeaf() { return triCount > 0; }
    Vector3d Normal;
};


struct Surface
{
    std::vector<Vector3d> vertices, normals;
    std::vector<Vector3i> indices;
    std::vector<Vector2f> uvs;
    // std::vector<Vector3d> Centeroid;//tri

    // Vector3d Centeroid;  //BODY
    Vector3d aabbMin, aabbMax;    //ADDED for BVH
     
    bool isLight;
    uint32_t shapeIdx;

    Vector3d diffuse;
    double alpha;

    // Vector3d AABB_box_min,AABB_box_max;int iscomputed=0;//susheel 

    Texture diffuseTexture, alphaTexture;

    Interaction rayPlaneIntersect(Ray ray, Vector3d p, Vector3d n);
    Interaction rayTriangleIntersect(Ray ray, Vector3d v1, Vector3d v2, Vector3d v3, Vector3d n);
    Interaction rayIntersect(Ray ray);

    Interaction rayIntersectT(Ray ray);
    int indsurf;
    // Interaction rayIntersectTri(Ray ray); //give surface info
    int totsurfTri=0; //down represent properties of surface 
    std::vector<Tri> tri; //Tri tri[N];
    std::vector<int> triIdx;
    std::vector<BVHNodeT> bvhNodeT;  //use this-> for 
    int rootNodeIdx = 0, nodesUsed = 1; //rootnodeIdx is always zero, at each ray passing, may change in func parameter only
    // Interaction rayIntersectbvhnTri(Ray ray); //give surface info
    void InitT();void BuildBVHT();void UpdateNodeBounds(int nodeIdx);
    void Subdivide(int nodeIdx);void  IntersectBVHTleaf(Ray &ray,int nodeIdx,Interaction& siFinal);
    int isbuildS=0;
    bool IntersectAABBspace(Ray ray,Vector3d bmin,Vector3d bmax );

private:
    bool hasDiffuseTexture();
    bool hasAlphaTexture();
};

std::vector<Surface> createSurfaces(std::string pathToObj, bool isLight, uint32_t shapeIdx);

