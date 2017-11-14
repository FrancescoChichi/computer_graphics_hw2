#include "yocto_math.h"

#ifndef GRAPHICS17B_HW02_TESSELATION_H
#define GRAPHICS17B_HW02_TESSELATION_H
struct face {
  ym::vec3f v[4];
  ym::vec3f c;
  face(ym::vec3f v0, ym::vec3f v1, ym::vec3f v2, ym::vec3f v3){
    v[0]=v0;
    v[1]=v1;
    v[2]=v2;
    v[3]=v3;
    c=(v0+v1+v2+v3)/ym::vec3f(4);
  }
};
struct tesselation {

  std::vector<face> faces;
  std::vector<ym::vec4i> quads;


  void add_face(ym::vec3f v0, ym::vec3f v1, ym::vec3f v2, ym::vec3f v3, ym::vec4i quad){
    faces.push_back(face({v0,v1,v2,v3}));
    quads.push_back(quad);
  }

  std::vector<face> get_faces(){
    return faces;
  }
  std::vector<ym::vec4i> get_quads(){
    return quads;
  }
  void clear(){
    faces.clear();
    quads.clear();
  }
};
#endif //GRAPHICS17B_HW02_TESSELATION_H
