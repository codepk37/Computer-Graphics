#include "surface.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"



std::vector<Surface> createSurfaces(std::string pathToObj, bool isLight, uint32_t shapeIdx)
{
    std::string objDirectory;
    const size_t last_slash_idx = pathToObj.rfind('/');
    if (std::string::npos != last_slash_idx) {
        objDirectory = pathToObj.substr(0, last_slash_idx);
    }

    std::vector<Surface> surfaces;

    tinyobj::ObjReader reader;
    tinyobj::ObjReaderConfig reader_config;
    if (!reader.ParseFromFile(pathToObj, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }

    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Loop over shapes

    for (size_t s = 0; s < shapes.size(); s++) {
        Surface surf;
        surf.indsurf=s;
        surf.aabbMin.x=surf.aabbMin.y=surf.aabbMin.z = 1e32f ;
        surf.aabbMax.x=surf.aabbMax.y=surf.aabbMax.z =-1e32f;

        // surf.Centeroid.x=surf.Centeroid.y=surf.Centeroid.z=0;

        std::cout<<"shaphes.size "<<s<<std::endl;
        surf.isLight = isLight;
        surf.shapeIdx = shapeIdx;
        std::set<int> materialIds;

        //XXXXXX Loop over faces(polygon)
        size_t index_offset = 0;
        
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            if (fv > 3) {
                std::cerr << "Not a triangle mesh" << std::endl;
                exit(1);
            }

            //XXXXXXX Loop over vertices in the face. Assume 3 vertices per-face
           
            Vector3d vertices[3], normals[3];
            Vector2f uvs[3];
            //
            
            for (size_t v = 0; v < fv; v++) {
                // access to all vertex(in loop) of single object at time
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                // Check if `normal_index` is zero or positive. negative = no normal data
                if (idx.normal_index >= 0) {
                    tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
                    tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
                    tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];

                    normals[v] = Vector3d(nx, ny, nz);
                }

                // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                if (idx.texcoord_index >= 0) {
                    tinyobj::real_t tx = attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
                    tinyobj::real_t ty = attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];

                    uvs[v] = Vector2f(tx, ty);
                }

                vertices[v] = Vector3d(vx, vy, vz);
            }

            int vSize = surf.vertices.size();
            Vector3i findex(vSize, vSize + 1, vSize + 2);

            surf.vertices.push_back(vertices[0]);
            surf.vertices.push_back(vertices[1]);
            surf.vertices.push_back(vertices[2]);

            surf.normals.push_back(normals[0]);
            surf.normals.push_back(normals[1]);
            surf.normals.push_back(normals[2]);

            surf.uvs.push_back(uvs[0]);
            surf.uvs.push_back(uvs[1]);
            surf.uvs.push_back(uvs[2]);
            
            // surf.Centeroid.x+=vertices[0].x;surf.Centeroid.x+=vertices[1].x;surf.Centeroid.x+=vertices[2].x;
            // surf.Centeroid.y+=vertices[0].y;surf.Centeroid.y+=vertices[1].y;surf.Centeroid.y+=vertices[2].y;
            // surf.Centeroid.z+=vertices[0].z;surf.Centeroid.z+=vertices[1].z;surf.Centeroid.z+=vertices[2].z;

            


            if(surf.aabbMax.x < vertices[0].x) {  surf.aabbMax.x =  vertices[0].x; }
            if(surf.aabbMax.x < vertices[1].x)  { surf.aabbMax.x =  vertices[1].x; }
            if(surf.aabbMax.x < vertices[2].x) { surf.aabbMax.x =  vertices[2].x;  }
            // surf.aabbMax.x =  max(vertices[0].x,max(vertices[1].x,vertices[2].x));
            if(surf.aabbMax.y < vertices[0].y) {  surf.aabbMax.y =  vertices[0].y;   }
            if(surf.aabbMax.y < vertices[1].y) {  surf.aabbMax.y =  vertices[1].y;   }
            if(surf.aabbMax.y < vertices[2].y) { surf.aabbMax.y =  vertices[2].y; }
            // surf.aabbMax.y = max(vertices[0].y,max(vertices[1].y,vertices[2].y)); //for finding max corner of body
             if(surf.aabbMax.z < vertices[0].z)  { surf.aabbMax.z =  vertices[0].z; }
            if(surf.aabbMax.z < vertices[1].z)   { surf.aabbMax.z =  vertices[1].z; }
            if(surf.aabbMax.z < vertices[2].z)   { surf.aabbMax.z =  vertices[2].z;   }
            // surf.aabbMax.z = max(vertices[0].z,max(vertices[1].z,vertices[2].z));
            
            if(surf.aabbMin.x > vertices[0].x) {  surf.aabbMin.x =  vertices[0].x; }
            if(surf.aabbMin.x > vertices[1].x) {  surf.aabbMin.x =  vertices[1].x; }
            if(surf.aabbMin.x > vertices[2].x) {  surf.aabbMin.x =  vertices[2].x; }
            // surf.aabbMin.x = min(vertices[0].x,min(vertices[1].x,vertices[2].x)); 3 min or max dosent workright/correct
            if(surf.aabbMin.y > vertices[0].y){ surf.aabbMin.y =  vertices[0].y; }
            if(surf.aabbMin.y > vertices[1].y){ surf.aabbMin.y =  vertices[1].y; }
            if(surf.aabbMin.y > vertices[2].y){ surf.aabbMin.y =  vertices[2].y; }
            // surf.aabbMin.y = min(vertices[0].y,min(vertices[1].y,vertices[2].y));//for finding min corner of body
            
            if(surf.aabbMin.z > vertices[0].z) { surf.aabbMin.z =  vertices[0].z;}
            if(surf.aabbMin.z > vertices[1].z) { surf.aabbMin.z =  vertices[1].z; }
            if(surf.aabbMin.z > vertices[2].z) {surf.aabbMin.z =  vertices[2].z;  }
            // surf.aabbMin.z = min(vertices[0].z,min(vertices[1].z,vertices[2].z));

            


            surf.indices.push_back(findex);  //surf.indices is collection of triangles 

            // per-face material
            materialIds.insert(shapes[s].mesh.material_ids[f]);

            index_offset += fv;
        }
        // surf.Centeroid.x=surf.Centeroid.x/3.0f;
        // surf.Centeroid.y=surf.Centeroid.y/3.0f;
        // surf.Centeroid.z=surf.Centeroid.z/3.0f;//centroidBody

        if (materialIds.size() > 1) {
            std::cerr << "One of the meshes has more than one material. This is not allowed." << std::endl;
            exit(1);
        }


        if (materialIds.size() == 0) {
            std::cerr << "One of the meshes has no material definition, may cause unexpected behaviour." << std::endl;
        }
        else {
            // Load textures from Materials
            auto matId = *materialIds.begin();
            if (matId != -1) {
                auto mat = materials[matId];

                surf.diffuse = Vector3d(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
                if (mat.diffuse_texname != "")
                    surf.diffuseTexture = Texture(objDirectory + "/" + mat.diffuse_texname);

                surf.alpha = mat.specular[0];
                if (mat.alpha_texname != "")
                    surf.alphaTexture = Texture(objDirectory + "/" + mat.alpha_texname);
            }
        }

        std::cout<<"live surface.size  "<<surfaces.size()<<std::endl;
        std::cout<<"min "<<surf.aabbMin.x<<" " <<surf.aabbMin.y <<" " <<surf.aabbMin.z<<std::endl;
        std::cout<<"max "<<surf.aabbMax.x<<" " <<surf.aabbMax.y <<" " <<surf.aabbMax.z<<std::endl;
        surfaces.push_back(surf);//surf   //surf is collection of all triangles of single surface
        shapeIdx++;              //surfaces is collection of surf
    }
   

    return surfaces;
}

bool Surface::hasDiffuseTexture() { return this->diffuseTexture.data != 0; }

bool Surface::hasAlphaTexture() { return this->alphaTexture.data != 0; }

Interaction Surface::rayPlaneIntersect(Ray ray, Vector3d p, Vector3d n)
{
    Interaction si;

    double dDotN = Dot(ray.d, n);
    if (dDotN != 0.f) {
        double t = -Dot((ray.o - p), n) / dDotN;

        if (t >= 0.f) {
            si.didIntersect = true;
            si.t = t;
            si.n = n;
            si.p = ray.o + ray.d * si.t;
        }
    }
   
    

    return si;//point ray,plane interasect
}

Interaction Surface::rayTriangleIntersect(Ray ray, Vector3d v1, Vector3d v2, Vector3d v3, Vector3d n)
{
    Interaction si = this->rayPlaneIntersect(ray, v1, n);//have point of intersection, Check if comes/present inside this->triagle

    if (si.didIntersect) {
        bool edge1 = false, edge2 = false, edge3 = false;

        // Check edge 1
        {
            Vector3d nIp = Cross((si.p - v1), (v3 - v1));
            Vector3d nTri = Cross((v2 - v1), (v3 - v1));
            edge1 = Dot(nIp, nTri) > 0;
        }

        // Check edge 2
        {
            Vector3d nIp = Cross((si.p - v1), (v2 - v1));
            Vector3d nTri = Cross((v3 - v1), (v2 - v1));
            edge2 = Dot(nIp, nTri) > 0;
        }

        // Check edge 3
        {
            Vector3d nIp = Cross((si.p - v2), (v3 - v2));
            Vector3d nTri = Cross((v1 - v2), (v3 - v2));
            edge3 = Dot(nIp, nTri) > 0;
        }

        if (edge1 && edge2 && edge3) {
            // Intersected triangle!
            si.didIntersect = true;
        }
        else {
            si.didIntersect = false;
        }
    }

    return si;
}

Interaction Surface::rayIntersect(Ray ray)  //surface  is object ,traverse each triangle of object
{
    Interaction siFinal;
    float tmin = ray.t;
    //looping through all faces

    for (auto face : this->indices) { //face:set of 3 points representing Triangle in net ~~not vertices,normals ... Just triangle num sort of
        Vector3d p1 = this->vertices[face.x];//are vertices of triangle
        Vector3d p2 = this->vertices[face.y];
        Vector3d p3 = this->vertices[face.z];

        Vector3d n1 = this->normals[face.x];//normal of one ofsingle vertex
        Vector3d n2 = this->normals[face.y];//normal of one ofsingle vertex
        Vector3d n3 = this->normals[face.z];//normal of one ofsingle vertex
        Vector3d n = Normalize(n1 + n2 + n3);

        Interaction si = this->rayTriangleIntersect(ray, p1, p2, p3, n);
        if (si.t <= tmin && si.didIntersect) {
            siFinal = si;
            tmin = si.t;
        }
    }

    return siFinal;
}







/////////////////////////////


void Surface::UpdateNodeBounds(int nodeIdx){

    BVHNodeT& node =bvhNodeT[nodeIdx];
    node.aabbMin.x=node.aabbMin.y=node.aabbMin.z=1e30f;//big node min,max update
    node.aabbMax.x=node.aabbMax.y=node.aabbMax.z=-1e30f;

    for(int first= node.leftFirst,i=0 ; i<node.triCount ;i++){
       
        int leafTriIdx = triIdx[first + i];
        Tri& leafTri = tri[leafTriIdx];

        // node.aabbMin = fminf( node.aabbMin, leafTri.vertex0 ),
		// node.aabbMin = fminf( node.aabbMin, leafTri.vertex1 ),
		// node.aabbMin = fminf( node.aabbMin, leafTri.vertex2 ),
		// node.aabbMax = fmaxf( node.aabbMax, leafTri.vertex0 ),
		// node.aabbMax = fmaxf( node.aabbMax, leafTri.vertex1 ),
		// node.aabbMax = fmaxf( node.aabbMax, leafTri.vertex2 );

        node.aabbMin.x = min( node.aabbMin.x, leafTri.vertex0.x );node.aabbMin.x = min( node.aabbMin.x, leafTri.vertex1.x );node.aabbMin.x = min( node.aabbMin.x, leafTri.vertex2.x );
        node.aabbMin.y = min( node.aabbMin.y, leafTri.vertex0.y );node.aabbMin.y = min( node.aabbMin.y, leafTri.vertex1.y );node.aabbMin.y = min( node.aabbMin.y, leafTri.vertex2.y );
        node.aabbMin.z = min( node.aabbMin.z, leafTri.vertex0.z );node.aabbMin.z = min( node.aabbMin.z, leafTri.vertex1.z );node.aabbMin.z = min( node.aabbMin.z, leafTri.vertex2.z );

        node.aabbMax.x = max( node.aabbMax.x, leafTri.vertex0.x );node.aabbMax.x = max( node.aabbMax.x, leafTri.vertex1.x );node.aabbMax.x = max( node.aabbMax.x, leafTri.vertex2.x );
        node.aabbMax.y = max( node.aabbMax.y, leafTri.vertex0.y );node.aabbMax.y = max( node.aabbMax.y, leafTri.vertex1.y );node.aabbMax.y = max( node.aabbMax.y, leafTri.vertex2.y );
        node.aabbMax.z = max( node.aabbMax.z, leafTri.vertex0.z );node.aabbMax.z = max( node.aabbMax.z, leafTri.vertex1.z );node.aabbMax.z = max( node.aabbMax.z, leafTri.vertex2.z );

    }

}




void Surface::Subdivide(int nodeIdx){
    BVHNodeT& node = bvhNodeT[nodeIdx];
    if (node.triCount <= 2) return;
	// determine split axis and position
	Vector3d extent = node.aabbMax - node.aabbMin;
	int axis = 0;
	if (extent.y > extent.x) axis = 1;
	if (extent.z > extent[axis]) axis = 2;
	float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;
	// in-place partition
	int i = node.leftFirst;
	int j = i + node.triCount - 1;
	while (i <= j)
	{
		if (tri[triIdx[i]].Tricentroid[axis] < splitPos)
			i++;
		else
			swap( triIdx[i], triIdx[j--] );
	}
    // abort split if one of the sides is empty
	int leftCount = i - node.leftFirst;
	if (leftCount == 0 || leftCount == node.triCount) return;
	// create child nodes
	int leftChildIdx = nodesUsed++;
	int rightChildIdx = nodesUsed++;
	bvhNodeT[leftChildIdx].leftFirst = node.leftFirst;
	bvhNodeT[leftChildIdx].triCount = leftCount;
	bvhNodeT[rightChildIdx].leftFirst = i;
	bvhNodeT[rightChildIdx].triCount = node.triCount - leftCount;
	node.leftFirst = leftChildIdx;
	node.triCount = 0;
	UpdateNodeBounds( leftChildIdx );
	UpdateNodeBounds( rightChildIdx );
	// recurse
	Subdivide( leftChildIdx );
	Subdivide( rightChildIdx );
    
}








void Surface:: BuildBVHT(){
     for(int i=0;i<totsurfTri;i++) triIdx[i]=i;

    for (int i = 0; i < totsurfTri; i++){
		tri[i].Tricentroid.x = (tri[i].vertex0.x + tri[i].vertex1.x + tri[i].vertex2.x) * 0.3333f;
        tri[i].Tricentroid.y = (tri[i].vertex0.y + tri[i].vertex1.y + tri[i].vertex2.y) * 0.3333f;
        tri[i].Tricentroid.z = (tri[i].vertex0.z + tri[i].vertex1.z + tri[i].vertex2.z) * 0.3333f;
    }
    BVHNodeT& root=bvhNodeT[rootNodeIdx];
    root.leftFirst = 0, root.triCount = totsurfTri;
	this->UpdateNodeBounds( rootNodeIdx );
	// subdivide recursively
	this->Subdivide( rootNodeIdx );
}



void Surface::InitT(){
    BuildBVHT();
    // std::cout<<" init "<<totsurfTri<<std::endl; 

}

bool Surface:: IntersectAABBspace(Ray ray,Vector3d bmin,Vector3d bmax )
{
	double tx1 = (bmin.x - ray.o.x) / ray.d.x, tx2 = (bmax.x - ray.o.x) / ray.d.x;
	double tmin = min( tx1, tx2 ), tmax = max( tx1, tx2 );
	double ty1 = (bmin.y - ray.o.y) / ray.d.y, ty2 = (bmax.y - ray.o.y) / ray.d.y;
	tmin = max( tmin, min( ty1, ty2 ) ), tmax = min( tmax, max( ty1, ty2 ) );
	double tz1 = (bmin.z - ray.o.z) / ray.d.z, tz2 = (bmax.z - ray.o.z) / ray.d.z;
	tmin = max( tmin, min( tz1, tz2 ) ), tmax = min( tmax, max( tz1, tz2 ) );
	return tmax >= tmin && tmin < ray.t && tmax > 0;
   
}


void Surface:: IntersectBVHTleaf(Ray &ray,int nodeIdx,Interaction& siFinal){
    
    BVHNodeT & node = bvhNodeT[nodeIdx];
    if (!IntersectAABBspace( ray, node.aabbMin, node.aabbMax )) {  return ; }

    if (node.isLeaf())
	{       

        // std::cout<<" leaf size "<<node.triCount<<std::endl; //10
        for (int i = 0; i < node.triCount; i++ ){  //are intersecting triangles
			// IntersectTri( ray, tri[triIdx[node.leftFirst + i]] );//psuh its surface 
            Tri t= tri[triIdx[node.leftFirst + i]];
            Vector3d p1 =t.vertex0; 
            Vector3d p2 = t.vertex1;
            Vector3d p3 =t.vertex2;
            Vector3d n =t.Normal;

            Interaction si = this->rayTriangleIntersect(ray, p1, p2, p3, n);
            if (si.t <= ray.t && si.didIntersect) {
                siFinal = si;
                ray.t = si.t;
            }
        }
        
	}
    else
	{
		IntersectBVHTleaf( ray, node.leftFirst     ,siFinal  );
		IntersectBVHTleaf( ray, node.leftFirst + 1  ,siFinal );
	}
}




Interaction Surface::rayIntersectT(Ray ray)  //surface  is object ,traverse each triangle of object
{   
     //
     auto startTime = std::chrono::high_resolution_clock::now();

    if(this->isbuildS==0){
        this->isbuildS=1;
        this->totsurfTri= this->indices.size();

        for (auto face : this->indices) { //face:set of 3 points representing Triangle in net ~~not vertices,normals ... Just triangle num sort of
            Tri t;
            
            Vector3d p1 = this->vertices[face.x];//are vertices of triangle
            t.vertex0   =p1;
            Vector3d p2 = this->vertices[face.y];
            t.vertex1   =p2;
            Vector3d p3 = this->vertices[face.z];
            t.vertex2   =p3;

            Vector3d n1 = this->normals[face.x];//normal of one ofsingle vertex
            Vector3d n2 = this->normals[face.y];//normal of one ofsingle vertex
            Vector3d n3 = this->normals[face.z];//normal of one ofsingle vertex
            Vector3d n = Normalize(n1 + n2 + n3);
            t.Normal  =n;

            (this->tri).push_back(t);     //heap ,link error means:  not using (this->tri) or restart vs ,vcode

        }     
        int N=tri.size();
        triIdx.resize(N);
        bvhNodeT.resize(2*N);
        // std::cout<<" this shold once"<<std::endl;

        InitT();
         auto finishTime = std::chrono::high_resolution_clock::now();

        auto renderTime= std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
         std::cout<<"  N "<<N << "   tri Tree : "<<  std::to_string(renderTime / 1000.f) << " ms" << std::endl;


    }
    
    // std::cout<<" many "<<std::endl;



    Interaction siFinal;
    float tmin = ray.t;
    //looping through all faces

    // for (auto face : this->indices) { //face:set of 3 points representing Triangle in net ~~not vertices,normals ... Just triangle num sort of
    //     Vector3d p1 = this->vertices[face.x];//are vertices of triangle
    //     Vector3d p2 = this->vertices[face.y];
    //     Vector3d p3 = this->vertices[face.z];

    //     Vector3d n1 = this->normals[face.x];//normal of one ofsingle vertex
    //     Vector3d n2 = this->normals[face.y];//normal of one ofsingle vertex
    //     Vector3d n3 = this->normals[face.z];//normal of one ofsingle vertex
    //     Vector3d n = Normalize(n1 + n2 + n3);

    //     Interaction si = this->rayTriangleIntersect(ray, p1, p2, p3, n);
    //     if (si.t <= tmin && si.didIntersect) {
    //         siFinal = si;
    //         tmin = si.t;
    //     }
    // }
   auto startTime2 = std::chrono::high_resolution_clock::now();

    this->IntersectBVHTleaf( ray, rootNodeIdx ,siFinal);

    auto finishTime2 = std::chrono::high_resolution_clock::now();

        auto renderTime2= std::chrono::duration_cast<std::chrono::microseconds>(finishTime2 - startTime2).count();
        //  std::cout << "one pixel traversal Tri  : " << std::to_string(renderTime2 / 1000.f) << " ms" << std::endl;


    return siFinal;
}














































































































/* SUSHEEL
Interaction Surface::rayIntersectAB(Ray ray)  //surface  is object ,traverse each triangle of object
{
    Interaction siFinal;
    // float tmin = ray.t;
    //looping through all faces

    double box_min_x = 1e30f,  box_min_y =  1e30f ,  box_min_z =  1e30f;
    double box_max_x = - 1e30f , box_max_y = - 1e30f ,  box_max_z =  -1e30f;
    
    if(this->iscomputed == 0){
        this->iscomputed=1;
        for(auto my_face : this->indices)
        {
            Vector3d p1 = this->vertices[my_face.x];
            Vector3d p2 = this->vertices[my_face.y];
            Vector3d p3 = this->vertices[my_face.z];

          
            // box_min_x=min(p1.x,min(p2.x,p3.x)); //take min if()if()if() , single line not(work_right)
           
            // box_min_y=min(p1.y,min(p2.y,p3.y));
        
            // box_min_z=min(p1.z,min(p2.z,p3.z));
           
            // box_max_x=max(p1.x,max(p2.x,p3.x));
           
            // box_max_y=max(p1.y,max(p2.y,p3.y));

            // box_max_z=max(p1.z,max(p2.z,p3.z));
            AABB_box_min.x = box_min_x;
            AABB_box_min.y = box_min_y;
            AABB_box_min.z = box_min_z;
            AABB_box_max.x = box_max_x;
            AABB_box_max.y = box_max_y;
            AABB_box_max.z = box_max_z;

            

        }
        std::cout<<"min "<<box_min_x<<" " <<box_min_y <<" " <<box_min_z<<std::endl;
        std::cout<<"max "<<box_max_x<<" " <<box_max_y <<" " <<box_max_z<<std::endl;
    
    }
    
    /////////////////////correct till nOW
    double tx1 = (this->AABB_box_min.x - ray.o.x) / ray.d.x, tx2 = (this->AABB_box_max.x - ray.o.x) / ray.d.x;
    double tmin = min( tx1, tx2 ), tmax = max( tx1, tx2 );
    double ty1 = (this->AABB_box_min.y - ray.o.y) / ray.d.y, ty2 = (this->AABB_box_max.y - ray.o.y) / ray.d.y;
    tmin = max( tmin, min( ty1, ty2 ) ), tmax = min( tmax, max( ty1, ty2 ) );
    double tz1 = (this->AABB_box_min.z - ray.o.z) / ray.d.z, tz2 = (this->AABB_box_max.z - ray.o.z) / ray.d.z;
    tmin = max( tmin, min( tz1, tz2 ) ), tmax = min( tmax, max( tz1, tz2 ) );
    bool intersect = (tmax >= tmin) && (tmin < ray.t && tmax > 0);
    
    
    float tmin2 = ray.t;
    if(intersect == true) 
    {  
        for (auto face : this->indices) {
            Vector3d pp1 = this->vertices[face.x];
            Vector3d pp2 = this->vertices[face.y];
            Vector3d pp3 = this->vertices[face.z];

            Vector3d n1 = this->normals[face.x];
            Vector3d n2 = this->normals[face.y];
            Vector3d n3 = this->normals[face.z];
            Vector3d n = Normalize(n1 + n2 + n3);

            // count++;

            Interaction si = this->rayTriangleIntersect(ray, pp1, pp2, pp3, n);
            if (si.t <= tmin2 && si.didIntersect) {
                siFinal = si;
                tmin2 = si.t;
            }
        }
    }
    else
    {
        siFinal.t = 1e30f;
    }
    return siFinal;
}
*/