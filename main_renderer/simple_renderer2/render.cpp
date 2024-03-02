#include "render.h"

Integrator::Integrator(Scene &scene)
{
    this->scene = scene;
    this->outputImage.allocate(TextureType::UNSIGNED_INTEGER_ALPHA, this->scene.imageResolution);
}
std::string op;
long long Integrator::render()
{
    auto startTime = std::chrono::high_resolution_clock::now();

    this->scene.interpolation_variant=op;//option

    for (int x = 0; x < this->scene.imageResolution.x; x++)
    {
        for (int y = 0; y < this->scene.imageResolution.y; y++)
        {
            Ray cameraRay = this->scene.camera.generateRay(x, y);
            // std::cout<<"yes\n";

            Interaction si = this->scene.rayIntersect(cameraRay);
             
            Vector3d c(1, 1, 1); // colour  defalut if no color,i.e. surf.diffusetexture.data =0

            
            if(si.hastexture!=0){c.x=si.r;c.y=si.g;c.z=si.b;}//if interacted surface have texture
                //gives correct color at texture interaction


            


            Vector3f n;
            n.x = si.n.x;
            n.y = si.n.y;
            n.z = si.n.z;


            
            // DOING FOR EACH PIXEL NEAREST INTERACTION
            if (si.didIntersect)
            { // now check if,from interaction point ,is there ray passing to source light directly(single intesection)

                Vector3d LoD(0.f, 0.f, 0.f);
                //  std::cout<<"mid\n";

                // directional source
                for (auto dirlight : this->scene.light.directionalLights)
                {
                    si.n;
                    si.p;
                    si.t;
                    dirlight.direction;
                    dirlight.radiance;

                    Ray interactiontolightR(si.p + (1e-5) * si.n, (dirlight.direction)); // from ,dirn opposite to dirn of light? ask ishaan ,works in opposite ldirection fine
                 
                    Interaction shadowray = this->scene.rayIntersect(interactiontolightR);
              
                    // std::cout<<shadowray.didIntersect<<"  distance surf intescet"<<shadowray.t<<std::endl;
                    //**- shadowray.didIntersect repersents Visibilility FACTOR

                    // if(!shadowray.didIntersect){//i.e. intersect to light directly
                    //      this->outputImage.writePixelColor(Vector3f(1.0f, 1.0f, 0.0f), x, y);
                    // }
                    // else(shadowray.didIntersect){//to some object brfore light
                    //     this->outputImage.writePixelColor(Vector3f(0.f, 0.f, 0.0f), x, y);
                    // }
                    Vector3d Li = dirlight.radiance; // incoming radiance

                    Vector3d f = c / M_PI;
                    double V = !shadowray.didIntersect; // Visibility
                    double wn = Dot(dirlight.direction, si.n);

                    LoD += (((Li * f) * V) * wn); // simple product of components

                    // Vector3d Lo = (((Li*f) *V ) * wn);
                    // this->outputImage.writePixelColor(Vector3f(Lo.x,Lo.y,Lo.z), x, y);//for current light
                }
                // this->outputImage.writePixelColor(Vector3f(LoD.x,LoD.y,LoD.z), x, y);//for current light


                // std::cout<<"mid2\n";

                Vector3d LoP(0.f, 0.f, 0.f);
                // Point source
                for (auto poilight : this->scene.light.pointLights)
                {
                    si.n;
                    si.p;
                    si.t;
                    poilight.location;
                    poilight.radiance;

                    

                    Vector3d p_x = (poilight.location - si.p);
                    Ray interactiontolightR(si.p + (1e-5) * si.n, Normalize(p_x)); // dirlight.direction-> p_x //from ,dirn opposite to dirn of light? ask ishaan ,works in opposite ldirection fine

                   
                    
                    double raytouch = (poilight.location - si.p).Length();

                    // std::cout<<"rhis\n";//error after

                    Interaction shadowray = this->scene.rayIntersect(interactiontolightR);  //segmentation line
                  
                    // std::cout<<"loopend\n";//error before

                    //**- shadowray.didIntersect repersents Visibilility FACTOR ,there jsut that wont help us

                    Vector3d Li = poilight.radiance; // incoming radiance

                    Vector3d f = c / M_PI;
                    // double V=!shadowray.didIntersect; //Visibility
                    double wn = Dot(Normalize(p_x), Normalize(si.n));

                    int V = -1;                  // Visibility_logic
                    if (!shadowray.didIntersect) // light falls,no object inbetween
                        V = 1;
                    else if (shadowray.didIntersect && (p_x.LengthSquared() > (shadowray.t * shadowray.t)))
                    {          //(shadowray.t*shadowray.t)-p_x.LengthSquared() )>-(1e-3) &&
                        V = 0; // no  particular this light touching
                    }
                    else
                        V = 1;

                    LoP += (Li / p_x.LengthSquared()) * ((f * V) * wn);

                    // this->outputImage.writePixelColor(Vector3f(LoP.x,LoP.y,LoP.z), x, y);//for current light

                    /////
                    //  if(! shadowray.didIntersect)
                    //     this->outputImage.writePixelColor(0.5f * (n + Vector3f(1.f, 0.f, 0.f)), x, y);
                    // else if (  shadowray.didIntersect &&  (p_x.LengthSquared() >= (shadowray.t*shadowray.t) ) ){//(shadowray.t*shadowray.t)-p_x.LengthSquared() )>-(1e-3) &&
                    //         this->outputImage.writePixelColor(Vector3f(0.f, 0.f, 0.f), x, y);
                    // }
                    // else
                    //     this->outputImage.writePixelColor(0.5f * (n + Vector3f(1.f, 0.f, 0.f)), x, y);
                }

                Vector3d Lo = LoD + LoP;
                
                // std::cout<<"No\n";

                this->outputImage.writePixelColor(Vector3f(Lo.x, Lo.y, Lo.z), x, y); // after total sum
                
            }
            else
            {
                this->outputImage.writePixelColor(Vector3f(0.f, 0.f, 0.f), x, y);
                // std::cout<<" not with any obj start "<<std::endl;
            }
        }
    }
    auto finishTime = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cerr << "Usage: ./render <scene_config> <out_path>";
        return 1;
    }
    Scene scene(argv[1]);

    op=argv[3];

    if(op=="0"){
        std::cout<<"Nearest Neighbor Fetching\n";
    }
    else if(op=="1"){
            std::cout<<"Bilinear Fetching\n";
    }
    else{
        std::cout<<"Incorrect option,so will do typecasting of pixel posn to int\n";
    }



    Integrator rayTracer(scene);
    auto renderTime = rayTracer.render();

    std::cout << "Render Time: " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;
    rayTracer.outputImage.save(argv[2]);

    return 0;
}
