#pragma once

#include "common.h"

enum TextureType {
    UNSIGNED_INTEGER_ALPHA = 0, // RGBA uint32
    FLOAT_ALPHA, // RGBA float
    NUM_TEXTURE_TYPES
};

struct uv_cor{
    uv_cor(double x,double y): P(x,y){}
    Vector2d P;//u,v
    Vector3d C;//r,g,b
};

struct Texture {
    unsigned long long data = 0;
    TextureType type;

    Vector2i resolution;

    Texture() {};
    Texture(std::string pathToImage);

    void allocate(TextureType type, Vector2i resolution);
    void writePixelColor(Vector3f color, int x, int y);
    Vector3d loadPixelColor(int x, int y);
    
    Vector3d nearestNeighbourFetch(double x,double y);//0
    Vector3d bilinearFetch(double uxr,double uyr);
    
    void loadJpg(std::string pathToJpg);
    void loadPng(std::string pathToPng);
    void loadExr(std::string pathToExr);
        
    void save(std::string path);
    void saveExr(std::string path);
    void savePng(std::string path);
};