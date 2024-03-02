#include "scene.h"

Scene::Scene(std::string sceneDirectory, std::string sceneJson)
{
    nlohmann::json sceneConfig;
    try {
        sceneConfig = nlohmann::json::parse(sceneJson);
    }
    catch (std::runtime_error e) {
        std::cerr << "Could not parse json." << std::endl;
        exit(1);
    }

    this->parse(sceneDirectory, sceneConfig);
}

Scene::Scene(std::string pathToJson)
{
    std::string sceneDirectory;

#ifdef _WIN32
    const size_t last_slash_idx = pathToJson.rfind('\\');
#else
    const size_t last_slash_idx = pathToJson.rfind('/');
#endif

    if (std::string::npos != last_slash_idx) {
        sceneDirectory = pathToJson.substr(0, last_slash_idx);
    }

    nlohmann::json sceneConfig;
    try {
        std::ifstream sceneStream(pathToJson.c_str());
        sceneStream >> sceneConfig;
    }
    catch (std::runtime_error e) {
        std::cerr << "Could not load scene .json file." << std::endl;
        exit(1);
    }

    this->parse(sceneDirectory, sceneConfig);
}

void Scene::parse(std::string sceneDirectory, nlohmann::json sceneConfig)
{
    // Output
    try {
        auto res = sceneConfig["output"]["resolution"];
        this->imageResolution = Vector2i(res[0], res[1]);
    }
    catch (nlohmann::json::exception e) {
        std::cerr << "\"output\" field with resolution, filename & spp should be defined in the scene file." << std::endl;
        exit(1);
    }

    // Cameras
    try {
        auto cam = sceneConfig["camera"];

        this->camera = Camera(
            Vector3d(cam["from"][0], cam["from"][1], cam["from"][2]),
            Vector3d(cam["to"][0], cam["to"][1], cam["to"][2]),
            Vector3d(cam["up"][0], cam["up"][1], cam["up"][2]),
            float(cam["fieldOfView"]),
            this->imageResolution
        );
    }
    catch (nlohmann::json::exception e) {
        std::cerr << "No camera(s) defined. Atleast one camera should be defined." << std::endl;
        exit(1);
    }

    // Surface
    try {
        auto surfacePaths = sceneConfig["surface"];

        uint32_t surfaceIdx = 0;
        for (std::string surfacePath : surfacePaths) {
             
            surfacePath = sceneDirectory + "/" + surfacePath;

             std::cout<<"surfacePath: "<<surfacePath<<std::endl;///////////single time
            
            auto surf = createSurfaces(surfacePath, /*isLight=*/false, /*idx=*/surfaceIdx);       //ITS VECTOR OF SURFACES
           
            //  std::cout<<"surface .size : "<<surf.size()<<std::endl; //TOTAL OBJECTS NUMBER
            /*Theefore ,all objects are stored in surf*/

            this->surfaces.insert(this->surfaces.end(), surf.begin(), surf.end());//size find

            surfaceIdx = surfaceIdx + surf.size();//0+ objects  ;like sphere, cube ,monkey 
        }
    }
    catch (nlohmann::json::exception e) {
        std::cout << "No surfaces defined." << std::endl;
    }
}


Interaction Scene::rayIntersect(Ray& ray)
{
    Interaction siFinal;

    for (auto& surface : this->surfaces) { //traversing each surface at time
        // std::cout<<" sur  :   "<<sur++ << std::endl;//very large :all pixels call this once
        Interaction si = surface.rayIntersect(ray);
        if (si.t <= ray.t) {    
            siFinal = si;
            ray.t = si.t;
        }
    }

    return siFinal;
}



////////////////////////////////2nd part






bool IntersectAABB(Ray ray,Vector3d bmin,Vector3d bmax )
{
	double tx1 = (bmin.x - ray.o.x) / ray.d.x, tx2 = (bmax.x - ray.o.x) / ray.d.x;
	double tmin = min( tx1, tx2 ), tmax = max( tx1, tx2 );
	double ty1 = (bmin.y - ray.o.y) / ray.d.y, ty2 = (bmax.y - ray.o.y) / ray.d.y;
	tmin = max( tmin, min( ty1, ty2 ) ), tmax = min( tmax, max( ty1, ty2 ) );
	double tz1 = (bmin.z - ray.o.z) / ray.d.z, tz2 = (bmax.z - ray.o.z) / ray.d.z;
	tmin = max( tmin, min( tz1, tz2 ) ), tmax = min( tmax, max( tz1, tz2 ) );
	return tmax >= tmin && tmin < ray.t && tmax > 0;
   
}


Interaction Scene::rayIntersectAABB(Ray& ray)
{
    Interaction siFinal;
    
    for (auto& surface : this->surfaces) { //traversing each object at time
        // std::cout<<" sur  :   "<<sur++ << std::endl;//very large :all pixels call this once

        // surface.aabbMax.x,surface.aabbMax.y,surface.aabbMax.z
        Interaction si;
        if(  IntersectAABB(ray , surface.aabbMin ,surface.aabbMax ) ){ //if ray intersects this object urF surface
            si = surface.rayIntersect(ray);
            if (si.t <= ray.t) {    
                siFinal = si;
                ray.t = si.t;
            }
        }
        else{
            //ray not goes through object
            si.t=1e30f;
        }

        
    }

    return siFinal;
}






///////////////////////////////////////////////3rd part
// below 2.2









void  Scene:: IntersectBVH(Ray &ray,int nodeIdx,Interaction& siFinal )//, std::vector<double>&sv){//,vector<Surface>&surfVec
{
    BVHNode & node = bvhNode[nodeIdx];
    if (!IntersectAABB( ray, node.aabbMin, node.aabbMax )) {  return ; }

    if (node.isLeaf())  //leaf may have 2 bodies
	{   
       
        for (int i = 0; i < node.boxCount; i++ ){  
             
            if(!IntersectAABB(ray, box[boxIdx[node.leftFirst + i]].BoxMin, box[boxIdx[node.leftFirst + i]].BoxMax)){continue;} //if not pass ,why  to intersect

            // Interaction  si=  box[boxIdx[node.leftFirst + i]].surf.rayIntersect(ray);//boxmin in set

			 Interaction  si=  box[box[boxIdx[node.leftFirst + i]].indsurf].surf.rayIntersect(ray);            //boxmin in set
             if (si.t <= ray.t) {    
                    siFinal = si;
                    ray.t = si.t;
            }
        }   
			   
	}
    else
	{
		IntersectBVH( ray, node.leftFirst     ,siFinal  );
		IntersectBVH( ray, node.leftFirst + 1  ,siFinal );
	}

    
}







void  Scene::UpdateNodeBounds(int nodeIdx){

    BVHNode& node = bvhNode[nodeIdx];
    node.aabbMin.x=node.aabbMin.y=node.aabbMin.z=1e30f;//big node min,max update
    node.aabbMax.x=node.aabbMax.y=node.aabbMax.z=-1e30f;

    for(int first= node.leftFirst,i=0 ; i<node.boxCount ;i++){
       
        int leafBoxIdx = boxIdx[first + i];
        Box& leafBox = box[leafBoxIdx];

        node.aabbMin.x = min( node.aabbMin.x, leafBox.BoxMin.x ); node.aabbMin.y = min( node.aabbMin.y, leafBox.BoxMin.y );node.aabbMin.z = min( node.aabbMin.z, leafBox.BoxMin.z );
		node.aabbMin.x= min( node.aabbMin.x, leafBox.BoxMax.x ) ; node.aabbMin.y= min( node.aabbMin.y, leafBox.BoxMax.y ) ;node.aabbMin.z= min( node.aabbMin.z, leafBox.BoxMax.z ) ;
		
		node.aabbMax.x = max( node.aabbMax.x, leafBox.BoxMax.x );node.aabbMax.y = max( node.aabbMax.y, leafBox.BoxMax.y );node.aabbMax.z = max( node.aabbMax.z, leafBox.BoxMax.z ),
		node.aabbMax.x = max( node.aabbMax.x, leafBox.BoxMin.x );node.aabbMax.y = max( node.aabbMax.y, leafBox.BoxMin.y );node.aabbMax.z = max( node.aabbMax.z, leafBox.BoxMin.z );
	


    }

}





void  Scene::Subdivide( int nodeIdx )
{
	// terminate recursion
	BVHNode& node = bvhNode[nodeIdx];
	if (node.boxCount <= 2) return;
	// determine split axis and position
	Vector3d extent = node.aabbMax - node.aabbMin; //current big superbox
	int axis = 0;
	if (extent.y > extent.x) axis = 1;
	if (extent.z > extent[axis]) axis = 2;
	float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;
	// in-place partition
	int i = node.leftFirst;
	int j = i + node.boxCount - 1;
	while (i <= j)
	{
		if (box[boxIdx[i]].BoxCentroid[axis] < splitPos)
			i++;
		else
			swap( boxIdx[i], boxIdx[j--] );
	}
	// abort split if one of the sides is empty
	int leftCount = i - node.leftFirst;
	if (leftCount == 0 || leftCount == node.boxCount) return;
	// create child nodes
	int leftChildIdx = nodesUsed++;
	int rightChildIdx = nodesUsed++;
	bvhNode[leftChildIdx].leftFirst = node.leftFirst;
	bvhNode[leftChildIdx].boxCount = leftCount;

	bvhNode[rightChildIdx].leftFirst = i;
	bvhNode[rightChildIdx].boxCount = node.boxCount - leftCount;
   
	node.leftFirst = leftChildIdx;
	node.boxCount = 0;
	UpdateNodeBounds( leftChildIdx );
	UpdateNodeBounds( rightChildIdx );
	// recurse
	Subdivide( leftChildIdx );
	Subdivide( rightChildIdx );
}







void  Scene::BuildBVH(){
    for(int i=0;i<totBox;i++) boxIdx[i]=i;

    //centeroid updated in main ,box.centroid=aabbmin+aabbmax
    for(int i=0;i<totBox;i++){

        box[i].BoxCentroid.x = (box[i].BoxMax.x + box[i].BoxMin.x) * 0.500f;
        box[i].BoxCentroid.y = (box[i].BoxMax.y + box[i].BoxMin.y) * 0.500f;
        box[i].BoxCentroid.z = (box[i].BoxMax.z + box[i].BoxMin.z) * 0.500f;
    }

    BVHNode& root=bvhNode[rootNodeIdx];
    root.leftFirst = 0, root.boxCount = totBox;
	this->UpdateNodeBounds( rootNodeIdx );
	// subdivide recursively
	this->Subdivide( rootNodeIdx );

    
}





void  Scene::Init(){
    BuildBVH();
    std::cout<<"hi should be once"<<std::endl;
}



Interaction Scene::rayIntersectBox(Ray& ray) //bvhonbox
{
    
    if(this->build==0){
         auto startTime = std::chrono::high_resolution_clock::now();
        this->build=1;
        this->totBox= this->surfaces.size();

        // box.resize(totBox);//removed push back
        
        for (auto& surface : this->surfaces) { //traversing each surface at time
            Box b;
            b.surf=surface; //directly accesss ,uncomment it
           
            b.BoxMin=surface.aabbMin;
            b.BoxMax=surface.aabbMax;
            b.indsurf=surface.indsurf; //box[surfinf].sirface
            
            (this->box).push_back(b);

        }
        int N=box.size();
        boxIdx.resize(N);
        bvhNode.resize(2*N);

        Init();
         auto finishTime = std::chrono::high_resolution_clock::now();

        auto renderTime= std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
         std::cout <<" N "<<N<< " Box Tree : " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;
    }

    Interaction siFinal;
                  
    auto startTime = std::chrono::high_resolution_clock::now();                                                                                                                                                     //  std::cout<<"hi should be always\n\n\n"<<std::endl;per pixel
    IntersectBVH( ray, rootNodeIdx ,siFinal); //return type Box ,or vector,(set not support) ,or sufrace ,or min,max
    //do ray intersection on this Box/object 's : Interaction si = b.surf.rayIntersect(ray);
    
    if(siFinal.t!=1e30f){
        // std::cout<<siFinal.t<<std::endl;
    }
   
     auto finishTime = std::chrono::high_resolution_clock::now();

     auto renderTime= std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
        //  std::cout << "1 Pixel Taversing box tree  : " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;
    
    return siFinal;

}











// above  .2.2
///////////////////


void Scene::IntersectBVHT(Ray &ray,int nodeIdx,Interaction& siFinal )//, std::vector<double>&sv){//,vector<Surface>&surfVec
{
    BVHNode & node = bvhNode[nodeIdx];
    if (!IntersectAABB( ray, node.aabbMin, node.aabbMax )) {  return ; }

    if (node.isLeaf())  //leaf may have 2 bodies
	{   
        //  std::cout<<"boxCount   "<<node.boxCount<<std::endl;
        for (int i = 0; i < node.boxCount; i++ ){  

            if(!IntersectAABB(ray , box[boxIdx[node.leftFirst + i]].BoxMin, box[boxIdx[node.leftFirst + i]].BoxMax)){continue;}
            
            Interaction  si=  box[box[boxIdx[node.leftFirst + i]].indsurf].surf.rayIntersectT(ray);//boxmin in set
			// IntersectTri( ray, box[boxIdx[node.leftFirst + i]] );//psuh its surface 
             if (si.t <= ray.t) {    
                    siFinal = si;
                    ray.t = si.t;
            }
        }   
			// IntersectTri( ray, box[boxIdx[node.leftFirst + i]] );//psuh its surface      
	}
    else
	{
		IntersectBVHT( ray, node.leftFirst     ,siFinal  );
		IntersectBVHT( ray, node.leftFirst + 1  ,siFinal );
	}

    
}


Interaction Scene::rayIntersectBoxT(Ray& ray) //bvhonbox
{
    
    if(this->build==0){
         auto startTime = std::chrono::high_resolution_clock::now();

        this->build=1;
        totBox= this->surfaces.size();

        for (auto& surface : this->surfaces) { //traversing each surface at time
            Box b;
            b.surf=surface; //directly accesss
            
            b.BoxMin=surface.aabbMin;
            b.BoxMax=surface.aabbMax;
            b.indsurf=surface.indsurf; //box[surfind].surface
            box.push_back(b);

        }
        int N=box.size();
        boxIdx.resize(N);
        bvhNode.resize(2*N);

        Init();
        
         auto finishTime = std::chrono::high_resolution_clock::now();

        auto renderTime= std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
         std::cout <<" N "<<N<< " Box Tree : " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;

    }

    Interaction siFinal;
         auto startTime = std::chrono::high_resolution_clock::now();                                                                                                                                                                          //  std::cout<<"hi should be always\n\n\n"<<std::endl;per pixel
    IntersectBVHT( ray, rootNodeIdx ,siFinal); //return type Box ,or vector,(set not support) ,or sufrace ,or min,max
    //do ray intersection on this Box/object 's : Interaction si = b.surf.rayIntersect(ray);
    
     auto finishTime = std::chrono::high_resolution_clock::now();
 auto renderTime= std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
        //  std::cout << " perpixel : " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;

    // if(siFinal.t!=1e30f){
    //     // std::cout<<siFinal.t<<std::endl;
    // }
   
       
    return siFinal;

}















/* SUSHEEL
Interaction Scene::rayIntersectAABB(Ray& ray)
{
    Interaction siFinal;

    for (auto& surface : this->surfaces) { //traversing each surface at time
        // std::cout<<" sur  :   "<<sur++ << std::endl;//very large :all pixels call this once
        Interaction si = surface.rayIntersectAB(ray);
        if (si.t <= ray.t) {    
            siFinal = si;
            ray.t = si.t;
        }
    }

    return siFinal;
}
*/


////////////////////////////////////////

















