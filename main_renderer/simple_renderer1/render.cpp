#include "render.h"
#include <string.h>
Integrator::Integrator(Scene &scene)
{
    this->scene = scene;
    this->outputImage.allocate(TextureType::UNSIGNED_INTEGER_ALPHA, this->scene.imageResolution);
}
Vector3f tofloat(Vector3d v){
    Vector3f a;
    a.x=v.x;a.y=v.y;a.z=v.z;
    return a;
}

std::string option ="0";
int pix=0;
long long Integrator::render()
{   
     auto startTime = std::chrono::high_resolution_clock::now();
    for (int x = 0; x < this->scene.imageResolution.x; x++) {
        for (int y = 0; y < this->scene.imageResolution.y; y++) {
            pix++;
            Ray cameraRay = this->scene.camera.generateRay(x, y);
            
            Interaction si; //made global for options
            if(option=="0"){
                        si = this->scene.rayIntersect(cameraRay); //Naive method  ;Interaction si= 
            }
            else if(option=="1"){
                         si = this->scene.rayIntersectAABB(cameraRay); //AABB bounding boxes iterrated
            }
            else if(option=="2"){
                         si = this->scene.rayIntersectBox(cameraRay); //AABB bounding boxes iterrated
            }
            else if(option=="3"){
                         si = this->scene.rayIntersectBoxT(cameraRay); //AABB bounding boxes iterrated
            }
            if (si.didIntersect)
                this->outputImage.writePixelColor(0.5f * ( tofloat(si.n) + Vector3f(1.f, 1.f, 1.f)), x, y);
            else
                this->outputImage.writePixelColor(Vector3f(0.f, 0.f, 0.f), x, y);
        }
    }
    auto finishTime = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
}

int main(int argc, char **argv)
{
    if (argc != 4) {
        std::cerr << "Usage: ./render <scene_config> <out_path>  <intersection_variant>";
        return 1;
    }
    Scene scene(argv[1]);

    option =argv[3];
    if(option=="0"){
         std::cout <<" 1:  Naive approach \n"<< std::endl;          
    }
    if(option=="1"){
                std::cout <<"2.1: AABB bounding boxes iterrated \n"<< std::endl;
    }
    if(option=="2"){
                std::cout <<"2.2: BVH on AABB only\n"<< std::endl;
    }
    if(option=="3"){
                std::cout <<"2.3: BVH on Triangles on top of 2.2 \n"<< std::endl;
    }

    Integrator rayTracer(scene);
    auto renderTime = rayTracer.render();

    std::cout<<" pix "<<pix << " Render Time: " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;
    rayTracer.outputImage.save(argv[2]);

    return 0;
}
//v1+v2 =>v3       
//v1+=v2    
//v1*k =>v3 
//v1*=k   
//v1/k =>v2  
//v1/=k  
//v[0],v[1],v[2]
//abs(v): abs(v.x),abs(v.y),abs(v.z)     
//Dot(v1,v2)=> scaler :v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
//AbsDot(v1,v2) :abs(Dot(v1, v2))=>scaler   
//Cross(v1,v2)=> v3    
//Normalize(v1):v / v.Length() => unit vector

/*camera: makes a ray (form ,to pixel)  //scenes:  vector of surfaces ,has camera info     */