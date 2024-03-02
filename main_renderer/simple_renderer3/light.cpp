#include "light.h"

// void print(Vector3f v){
//     std::cout<<v.x<<"  "<<v.y<<"  "<<v.z<<std::endl;
// }

Light::Light(LightType type, nlohmann::json config) {
    
    switch (type) {
        case LightType::POINT_LIGHT:
            this->position = Vector3f(config["location"][0], config["location"][1], config["location"][2]);
            break;
        case LightType::DIRECTIONAL_LIGHT:
            this->direction = Vector3f(config["direction"][0], config["direction"][1], config["direction"][2]);

            // print(this->direction);

            break;
        case LightType::AREA_LIGHT:
            // TODO: Implement this
            // std::cout<<config<<"\n"; DONE
            this->center = Vector3f(config["center"][0], config["center"][1], config["center"][2]);
            this->vx = Vector3f(config["vx"][0], config["vx"][1], config["vx"][2]);
            this->vy = Vector3f(config["vy"][0], config["vy"][1], config["vy"][2]);
            this->normal = Vector3f(config["normal"][0], config["normal"][1], config["normal"][2]);
            
            // print(this->center);  print(this->vx);  print(this->vy); print(this->normal);
                    

            break;
        default:
            std::cout << "WARNING: Invalid light type detected";
            break;
    }

    this->radiance = Vector3f(config["radiance"][0], config["radiance"][1], config["radiance"][2]);
    this->type = type;
}

std::pair<Vector3f, LightSample> Light::sample(Interaction *si) {
    LightSample ls;
    memset(&ls, 0, sizeof(ls));

    Vector3f radiance;
    switch (type) {
        case LightType::POINT_LIGHT:
            ls.wo = (position - si->p);//interaction point o light
            ls.d = ls.wo.Length();
            ls.wo = Normalize(ls.wo);
            radiance = (1.f / (ls.d * ls.d)) * this->radiance;
            break;
        case LightType::DIRECTIONAL_LIGHT:
            ls.wo = Normalize(direction);//interaction point o light
            ls.d = 1e10;
            radiance = this->radiance;
            break;
        case LightType::AREA_LIGHT:

            ls.wo=  si->sampled;//sampled dirn to area light
            Ray ray(si->p + 1e-3f * si->n, ls.wo);//ls.wo gives direction of ray to light
            Interaction witharealig= this->intersectLight(ray);

            ls.area= witharealig.area;//**

            ls.d= witharealig.t;//**
            ls.wo= Normalize(ls.wo);//** surf to ray
            if(Dot(ls.wo,normal)<0) { //**imp: cahnged coz area light only emits from a single face in direction of the normal n
                radiance= witharealig.emissiveColor; 
            }
            
            ls.p= witharealig.p; //no need of this 

            // light.intersectLight(ray);
            

            // ls.wo=  (center-si->p);  //need to sample wo :in case of are light
            // ls.d= ls.wo.Length(); //for now center
            // ls.wo = Normalize(ls.wo);
            // radiance= this->radiance;
            // i think we need to implement monte carlo here
             
            // TODO: Implement this  
            //  need to use randiance calc formula here acc area light
            // also make mechanism to find ls.d ,later used in render.cpp




            break;
    }
    return { radiance, ls };
}



Interaction Light::rayPlaneIntersect(Ray ray, Vector3f p, Vector3f n)
{
    Interaction si;

    float dDotN = Dot(ray.d, n);
    if (dDotN != 0.f) {
        float t = -Dot((ray.o - p), n) / dDotN;

        if (t >= 0.f) {
            si.didIntersect = true;
            si.t = t;
            si.n = n;
            si.p = ray.o + ray.d * si.t;
        }
    }

    return si;
}


// Interaction Surface::rayTriangleIntersect(Ray ray, Vector3f v1, Vector3f v2, Vector3f v3, Vector3f n)
Interaction Light::rayRectangleLightIntersect(Ray ray, Vector3f center, Vector3f vx, Vector3f vy, Vector3f normal) 
{
    // Interaction si = this->rayPlaneIntersect(ray, v1, n);
    Interaction si = this->rayPlaneIntersect(ray, center, normal);
    if (si.didIntersect) {

        // Calculate the four corner coordinates
        Vector3f tL = center - vx + vy;//topleft
        Vector3f tR = center + vx + vy;
        Vector3f bR = center + vx - vy;//bottomright
        Vector3f bL = center - vx - vy;

        /////
        Vector3f side1 = tR - tL;
        Vector3f side2 = bL - tL;
        Vector3f area = Cross(side1, side2);
        si.area=area.Length();  //area of area_light**
        //////
        
        bool edge1 = false, edge2 = false, edge3 = false, edge4=false;

        // Check edge 1
        {
            Vector3f nIp = Cross((si.p - bL), (bR - bL)); //bL
            Vector3f nTri = Cross((tL - bL), (bR - bL));
            edge1 = Dot(nIp, nTri) > 0;
        }

        // Check edge 2
        {
            Vector3f nIp = Cross((si.p - bL), (tL - bL));
            Vector3f nTri = Cross((bR - bL), (tL - bL));
            edge2 = Dot(nIp, nTri) > 0;
        }

        // Check edge 3
        {
            Vector3f nIp = Cross((si.p - tR), (bR - tR));
            Vector3f nTri = Cross((tL - tR), (bR - tR));
            edge3 = Dot(nIp, nTri) > 0;
        }

        // Check edge 4
        {
            Vector3f nIp = Cross((si.p - tL), (tR - tL));
            Vector3f nTri = Cross((bL - tL), (tR - tL));
            edge4 = Dot(nIp, nTri) > 0;
        }

        if (edge1 && edge2 && edge3 && edge4 && si.t >= 0.f) {
            // Intersected triangle!
            si.didIntersect = true;
        }
        else {
            si.didIntersect = false;
        }
    }

    return si;
}


Interaction Light::intersectLight(Ray ray) {
    Interaction si;
    memset(&si, 0, sizeof(si));

    if (type == LightType::AREA_LIGHT) { //

        si = rayRectangleLightIntersect(ray, this->center, this->vx ,this->vy ,this->normal);

        if(si.didIntersect ){
            si.emissiveColor= this->radiance;//stored radinace of light 

            // std:: cout<<" emisscolo"<< si.emissiveColor.y<<std::endl;
        }
        else{
            si.emissiveColor= Vector3f(0,0,0);
        }

        // TODO: Implement this
    }

    return si;
}