
#include "light.h"




Light Light:: loadlight(nlohmann::json sceneConfig){
    Light lig;

    // std::cout<<" in light .cpp folder"<<sceneConfig<<std::endl; in json format given
    // sceneConfig //is json format
    
    auto dirlight = sceneConfig["directionalLights"];

    for(auto curr: dirlight){
        DirectionalLight t;
        t.direction.x=curr["direction"][0];
        t.direction.y=curr["direction"][1];
        t.direction.z=curr["direction"][2];

        t.radiance.x = curr["radiance"][0];
        t.radiance.y = curr["radiance"][1];
        t.radiance.z = curr["radiance"][2];

        lig.directionalLights.push_back(t);

 }

    


    auto poilight = sceneConfig["pointLights"];

    for(auto curr: poilight){
        PointLight t;
        t.location.x=curr["location"][0];
        t.location.y=curr["location"][1];
        t.location.z=curr["location"][2];

        t.radiance.x = curr["radiance"][0];
        t.radiance.y = curr["radiance"][1];
        t.radiance.z = curr["radiance"][2];

        lig.pointLights.push_back(t);
        
    }

    return lig;



            
}