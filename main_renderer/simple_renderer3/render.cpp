#include "render.h"

float spp=0; // sample per pixel 
int option=-1;//samping_method

Vector3f uniformSampleHemi(){
     // 1)UNIFORM SAMPLING
    float rand1=next_float();
    float rand2=next_float();
    float thetah= acos(rand1); //in radian ; 108/PI ->degree
    float phi   =2* M_PI * rand2;

    float z= cos( thetah );
    float y= sin( thetah) * sin(phi);
    float x= sin( thetah) * cos(phi); 
    Vector3f samp_dir(x,y,z); //uniform sample hrmisphee

    return samp_dir;

    // ls.wo= samp_dir;//dir 

    // // convert to global****
    // {
    //         ls.wo= si.toWorld(ls.wo,cameraRay);//needed to pass ca,eraray so did transformation here
    // }
}

Vector3f cosinesampling(){
    // 2)COSINE SAMPLING
    double rand1=next_float();
    double rand2=next_float();
    // double thetah= acos(  (1- 2*rand1)/2.0f   ); //in radian : of slides ,surface parallel to area lights are dark***
    double thetah = acos(sqrt(1.0 - rand1)); // chatgpt : surface parallel to area lights are visible(not dark)

    double phi   =2* M_PI * rand2;

    float z= cos( thetah );
    float y= sin( thetah) * sin(phi);
    float x= sin( thetah) * cos(phi); 
    Vector3f samp_dir(x,y,z); //cosine sampling

    return samp_dir;

}

Vector3f areaoflighsampling(Light light,Interaction si ){
    // 3)LIGHT SAMPLING
    float rand1=next_float();
    float rand2=next_float();
    Vector3f samp_point_onarea(0,0,0);

    light.center;
    light.vx;light.vy;
    light.normal;

    samp_point_onarea= light.center + 2*(rand1 - 0.5f)*light.vx + 2*(rand2-0.5f)*light.vy;


    Vector3f samp_dir = (samp_point_onarea - si.p);//interaction point to arealight_point    
    return  samp_dir;

}



Integrator::Integrator(Scene &scene)
{
    this->scene = scene;
    this->outputImage.allocate(TextureType::UNSIGNED_INTEGER_ALPHA, this->scene.imageResolution);
}

long long Integrator::render()
{
    auto startTime = std::chrono::high_resolution_clock::now();
    for (int x = 0; x < this->scene.imageResolution.x; x++) {
        for (int y = 0; y < this->scene.imageResolution.y; y++) {

            Vector3f result(0, 0, 0);//contribution of area light
            Vector3f poi_dirlig(0,0,0); //added: contribution of point and direction , making it differnt to measure each component measurement

            Vector3f emmitedcolor(0,0,0);
            //area ligths occlude anything behind them
            for(int sppcur =spp ;sppcur >0;sppcur--)//added loop for spp
            {               
                Ray cameraRay = this->scene.camera.generateRay(x, y);
                Interaction si = this->scene.rayIntersect(cameraRay);


                ////property of surface only having light
                Interaction si_withlight = this->scene.rayEmitterIntersect(cameraRay); //property of surface point emmiting light
                if(si_withlight.didIntersect ){  //
                    emmitedcolor+= si_withlight.emissiveColor ;
                    
                }
                //// different part above

                
                if (si.didIntersect) {
                    Vector3f radiance;
                    LightSample ls;
                    for (Light &light : this->scene.lights) {

                        if(light.type==AREA_LIGHT){
                            Vector3f local_dir=Vector3f(0,0,0);
                            Vector3f global_dir=Vector3f(0,0,0);
                            if(option==0){
                                local_dir= uniformSampleHemi();//local dir
                                global_dir= si.toWorld(local_dir,cameraRay.d);//uniform sampled surf to light(also maybe not be light) 
                            }
                            else if(option==1){
                                local_dir= cosinesampling();
                                global_dir= si.toWorld(local_dir,cameraRay.d);//uniform sampled surf to light(also maybe not be light) 
                            }
                            else if(option==2){
                                Vector3f dir= areaoflighsampling(light,si);
                                global_dir= Normalize(dir);//no need to globalize, its already in global
                            }

                            // Vector3f global_dir= si.toWorld(local_dir,cameraRay.d);//uniform sampled surf to light(also maybe not be light) 
                            si.sampled = global_dir;
                        }


                        std::tie(radiance, ls) = light.sample(&si);//send si.sampled **

                        

                        Ray shadowRay(si.p + 1e-3f * si.n, ls.wo);//ls.wo gives direction of ray to light
                        Interaction siShadow = this->scene.rayIntersect(shadowRay);

                        


                        if (!siShadow.didIntersect || siShadow.t > ls.d) {
                            if(light.type != AREA_LIGHT){//point or directional
                                //**
                                poi_dirlig += si.bsdf->eval(&si, si.toLocal(ls.wo,cameraRay.d))  * radiance * std::abs(Dot(si.n, ls.wo)); 

                            }//AREA LIGHT
                            else if(option==0 &&light.type == AREA_LIGHT)
                                result += si.bsdf->eval(&si, si.toLocal(ls.wo,cameraRay.d))  * radiance * std::abs(Dot(si.n, ls.wo)); //find meaning of this local() why used here
                        
                            else if(option==1 &&light.type == AREA_LIGHT){
                                    result += si.bsdf->eval(&si, si.toLocal(ls.wo,cameraRay.d))  * radiance ;//* std::abs(Dot(si.n, ls.wo)); //find meaning of this local() why used here
                            }
                            else if(option==2 &&light.type == AREA_LIGHT){

                                float costheta_l= Dot(light.normal, -ls.wo);

                                float r= ls.d;

                                ls.area;
                                ls.d;
                                ls.p;
                                ls.wo;


                                result += si.bsdf->eval(&si, si.toLocal(ls.wo,cameraRay.d))  * radiance * std::abs(Dot(si.n, ls.wo))* ((costheta_l*ls.area)/(r*r)); //find meaning of this local() why used here
                            }
                            
                        
                        }       //ansh said :global -> local : sample : local -> global 
                    }
                }           

                
                  
                
            }//
            //Uniform distribution
            if(option==0)
                this->outputImage.writePixelColor(emmitedcolor/spp+ (2* M_PI/spp) *result + poi_dirlig/spp , x, y); //at end, avergage out effect of sub-pixel of single pixel result/spp
            else if(option==1)
                this->outputImage.writePixelColor(emmitedcolor/spp+ ( M_PI/spp) *result    + poi_dirlig/spp, x, y); //at end, avergage out effect of sub-pixel of single pixel result/spp
            else if(option==2)
                this->outputImage.writePixelColor(emmitedcolor/spp+ result/spp             + poi_dirlig/spp, x, y); //**at end, (1/spp) reperesents->  (1/N)
            //GENERAL
            else  {//
                // std::cout<<(result/spp).x <<"    "<<result.x<<std::endl; 
                //just consider surface as light property(emitted/spp)  and point,direction light effect(poi_dirlight/spp)  ,not effect of area light on surface(result/spp)  
                this->outputImage.writePixelColor(emmitedcolor/spp            + poi_dirlig/spp, x, y);  //not a correct option though, used for first 2 questions
            }
            //GENERAL
            // this->outputImage.writePixelColor(emmitedcolor/spp+ result/spp, x, y); //at end, avergage out effect of sub-pixel of single pixel result/spp
        }
    }
    auto finishTime = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::cerr << "Usage: ./render <scene_config> <out_path> <num_samples> <sampling_strategy>";
        return 1;
    }
    Scene scene(argv[1]);

    Integrator rayTracer(scene);
    int sppcnt = atoi(argv[3]);
    spp=sppcnt;
    option= atoi(argv[4]);
    auto renderTime = rayTracer.render();
    
    std::cout << "Render Time: " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;
    rayTracer.outputImage.save(argv[2]);

    return 0;
}
