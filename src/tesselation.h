#include "yocto_math.h"

#ifndef GRAPHICS17B_HW02_TESSELATION_H
#define GRAPHICS17B_HW02_TESSELATION_H
struct face {
  ym::vec3f v0, v1, v2, v3, c;
  face(ym::vec3f vc0, ym::vec3f vc1, ym::vec3f vc2, ym::vec3f vc3){
    v0=vc0;
    v1=vc1;
    v2=vc2;
    v3=vc3;
    c=(v0+v1+v2+v3)/ym::vec3f(4);
  }
};
struct tesselation {

  std::vector<face> faces;
  std::vector<ym::vec4i> quads;
  std::vector<ym::vec3f> vertices;

  void add_face(ym::vec3f v0, ym::vec3f v1, ym::vec3f v2, ym::vec3f v3, ym::vec4i quad){
    faces.push_back(face({v0,v1,v2,v3}));
    quads.push_back(quad);
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
  }

  std::vector<face> get_faces(){
    return faces;
  }
  std::vector<ym::vec4i> get_quads(){
    return quads;
  }
  void add_vertex(ym::vec3f v){
    vertices.push_back(v);
  }
  std::vector<ym::vec3f> get_vertices(){
    return vertices;
  }
};
#endif //GRAPHICS17B_HW02_TESSELATION_H
