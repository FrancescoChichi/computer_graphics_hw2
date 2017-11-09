//
// Created by s1r on 09/11/17.
//

#ifndef GRAPHICS17B_HW02_TESSELATION_H
#define GRAPHICS17B_HW02_TESSELATION_H

struct tesselation{

  yscn::shape* shape;
  std::vector<ym::vec3f> vertices;

  tesselation(yscn::shape* shp){shape=shp;};

  inline void add_vertices(ym::vec3f v){
    vertices.push_back(v);
  }

};
#endif //GRAPHICS17B_HW02_TESSELATION_H
