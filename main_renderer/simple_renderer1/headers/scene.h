#pragma once

#include "camera.h"
#include "surface.h"


struct Box{
    Surface surf;
    int indsurf;
    Vector3d BoxMin,BoxMax;
    Vector3d BoxCentroid;

};
struct BVHNode
{
	Vector3d aabbMin, aabbMax;
	int leftFirst, boxCount;
	bool isLeaf() { return boxCount > 0; }
    // Surface surf;

     int indsurf; // Index of the associated Box in the 'boxes' array
};


struct Scene {
    std::vector<Surface> surfaces;
    Camera camera;
    Vector2i imageResolution;

    Scene() {};
    Scene(std::string sceneDirectory, std::string sceneJson);
    Scene(std::string pathToJson);
    
    void parse(std::string sceneDirectory, nlohmann::json sceneConfig);

    Interaction rayIntersect(Ray& ray); //1
    Interaction rayIntersectAABB(Ray& ray);//2.1
    Interaction rayIntersectBox(Ray& ray) ;//2.2
   
   
   int totBox=0;
    std::vector<Box> box; //Tri tri[N];
    std::vector<int> boxIdx;
    std::vector<BVHNode> bvhNode;
      int build=0;
    int rootNodeIdx = 0, nodesUsed = 1; //rootnodeIdx is always zero, at each ray passing, may change in func parameter only
    void  Init();void  BuildBVH();void  Subdivide( int nodeIdx );void  UpdateNodeBounds(int nodeIdx);void  IntersectBVH(Ray &ray,int nodeIdx,Interaction& siFinal );
    
    
    Interaction rayIntersectBoxT(Ray& ray);//2.3 //heap error when function delcareation name,defination name is different 
   void IntersectBVHT(Ray &ray,int nodeIdx,Interaction& siFinal );

};


