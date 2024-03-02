#pragma once

#include "common.h"

//**Modification for loadlight:  Needed to add light.cpp in CMakeLists.txt and various where camera.cpp was mentioned

enum LightType
{
	POINT_LIGHT = 0,
	DIRECTIONAL_LIGHT = 1,
	NUM_LIGHT_TYPES
};

struct DirectionalLight
{
	Vector3d direction, radiance;
};
struct PointLight
{
	Vector3d location, radiance;
};

struct Light
{
	std::vector<DirectionalLight> directionalLights; //
	std::vector<PointLight> pointLights;			 //

	Light loadlight(nlohmann::json sceneConfig);
};
