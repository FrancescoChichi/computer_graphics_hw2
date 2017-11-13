#include <thread>
#include "yocto_img.h"
#include "yocto_math.h"
#include "yocto_scn.h"
#include "yocto_utils.h"
#include "tesselation.h"
#include "printData.h"

ym::ray3f eval_camera(const yscn::camera* cam, const ym::vec2f& uv) {
  auto h = 2 * std::tan(cam->yfov / 2);
  auto w = h * cam->aspect;
  auto q = ym::vec3f{w * cam->focus * (uv.x - 0.5f),
                     h * cam->focus * (uv.y - 0.5f), -cam->focus};
  return ym::ray3f(transform_point(cam->frame, ym::zero3f),
                   transform_direction(cam->frame, ym::normalize(q - ym::zero3f)));
}

inline ym::vec4f lookup_texture(
    const yscn::texture* txt, const ym::vec2i& ij, bool srgb = true) {
  if (txt->ldr) {
    auto v = txt->ldr[ij];
    return (srgb) ? ym::srgb_to_linear(v) : ym::byte_to_float(v);
  } else if (txt->hdr) {
    return txt->hdr[ij];
  } else {
    assert(false);
    return {};
  }
}

ym::vec4f eval_texture(
    const yscn::texture* txt, const ym::vec2f& texcoord, bool srgb = true) {
  if (!txt) return {1, 1, 1, 1};
  auto wh = ym::vec2i{txt->width(), txt->height()};

  auto st = ym::vec2f{
      std::fmod(texcoord.x, 1.0f) * wh.x, std::fmod(texcoord.y, 1.0f) * wh.y};
  if (st.x < 0) st.x += wh.x;
  if (st.y < 0) st.y += wh.y;

  auto ij = ym::clamp(ym::vec2i{(int)st.x, (int)st.y}, {0, 0}, wh);
  auto uv = st - ym::vec2f{(float)ij.x, (float)ij.y};

  ym::vec2i idx[4] = {ij, {ij.x, (ij.y + 1) % wh.y},
                      {(ij.x + 1) % wh.x, ij.y}, {(ij.x + 1) % wh.x, (ij.y + 1) % wh.y}};
  auto w = ym::vec4f{(1 - uv.x) * (1 - uv.y), (1 - uv.x) * uv.y,
                     uv.x * (1 - uv.y), uv.x * uv.y};

  // handle interpolation
  return (lookup_texture(txt, idx[0], srgb) * w.x +
          lookup_texture(txt, idx[1], srgb) * w.y +
          lookup_texture(txt, idx[2], srgb) * w.z +
          lookup_texture(txt, idx[3], srgb) * w.w);
}

ym::vec4f shade(const yscn::scene* scn,
                const std::vector<yscn::instance*>& lights, const ym::vec3f& amb,
                const ym::ray3f& ray) {
  auto isec = yscn::intersect_ray(scn, ray, false);
  if (!isec) return {0, 0, 0, 0};

  auto ist = scn->instances[isec.iid];
  auto shp = ist->shp;
  auto mat = shp->mat;

  auto pos = ym::zero3f;
  auto norm = ym::zero3f;
  auto texcoord = ym::zero2f;
  if (!shp->triangles.empty()) {
    auto t = shp->triangles[isec.eid];
    pos = ym::eval_barycentric_triangle(
        shp->pos[t.x], shp->pos[t.y], shp->pos[t.z], isec.euv);
    norm = ym::normalize(ym::eval_barycentric_triangle(
        shp->norm[t.x], shp->norm[t.y], shp->norm[t.z], isec.euv));
    if (!shp->texcoord.empty())
      texcoord = ym::eval_barycentric_triangle(shp->texcoord[t.x],
                                               shp->texcoord[t.y], shp->texcoord[t.z], isec.euv);
  } else if (!shp->lines.empty()) {
    auto l = shp->lines[isec.eid];
    pos = ym::eval_barycentric_line(shp->pos[l.x], shp->pos[l.y], isec.euv);
    norm = ym::normalize(ym::eval_barycentric_line(
        shp->norm[l.x], shp->norm[l.y], isec.euv));
    if (!shp->texcoord.empty())
      texcoord = ym::eval_barycentric_line(
          shp->texcoord[l.x], shp->texcoord[l.y], isec.euv);
  } else if (!shp->points.empty()) {
    auto p = shp->points[isec.eid];
    pos = shp->pos[p];
    norm = shp->norm[p];
    if (!shp->texcoord.empty()) texcoord = shp->texcoord[p];
  }

  pos = ym::transform_point(ym::to_frame(ist->xform()), pos);
  norm = ym::transform_direction(ym::to_frame(ist->xform()), norm);

  auto ke = mat->ke * eval_texture(mat->ke_txt.txt, texcoord).xyz();
  auto kd = mat->kd * eval_texture(mat->kd_txt.txt, texcoord).xyz();
  auto ks = mat->ks * eval_texture(mat->ks_txt.txt, texcoord).xyz();
  auto ns = (mat->rs) ? 2 / (mat->rs * mat->rs) - 2 : 1e6f;

  auto l = ke + kd * amb;
  for (auto lgt : lights) {
    auto lshp = lgt->shp;
    for (auto p : lgt->shp->points) {
      auto lpos =
          ym::transform_point(ym::to_frame(lgt->xform()), lshp->pos[p]);
      auto lr = ym::length(lpos - pos);
      auto wi = ym::normalize(lpos - pos);
      if (yscn::intersect_ray(scn, {pos, wi, 0.01f, lr - 0.04f}, true))
        continue;

      auto le = lshp->mat->ke / (lr * lr * ym::pif);  // normalized to pi
      auto wo = -ray.d;
      auto wh = ym::normalize(wi + wo);
      if (!shp->triangles.empty()) {
        l += le * kd * ym::max(0.0f, ym::dot(norm, wi)) +
             le * ks * ym::pow(ym::max(0.0f, ym::dot(norm, wh)), ns);
      } else if (!shp->lines.empty()) {
        l += le * kd *
             ym::sqrt(ym::clamp(
                 1 - ym::dot(norm, wi) * ym::dot(norm, wi), 0.0f,
                 1.0f)) +
             le * ks *
             ym::pow(ym::sqrt(ym::clamp(
                 1 - ym::dot(norm, wh) * ym::dot(norm, wh),
                 0.0f, 1.0f)),
                     ns);
      }
    }
  }

  return {l.x, l.y, l.z, 1};
}

ym::image4f raytrace(const yscn::scene* scn, const ym::vec3f& amb,
                     int resolution, int samples, bool facets) {
  auto cam = scn->cameras.front();
  auto img = ym::image4f(
      (int)std::round(cam->aspect * resolution), resolution, {0, 0, 0, 0});

  auto lights = std::vector<yscn::instance*>();
  for (auto ist : scn->instances) {
    if (ist->shp->mat->ke == ym::zero3f) continue;
    if (ist->shp->points.empty()) continue;
    lights.push_back(ist);
  }

  for (auto j = 0; j < img.height(); j++) {
    for (auto i = 0; i < img.width(); i++) {
      img[{i, j}] = {0, 0, 0, 0};
      for (auto sj = 0; sj < samples; sj++) {
        for (auto si = 0; si < samples; si++) {
          auto u = (i + (si + 0.5f) / samples) / img.width();
          auto v = ((img.height() - j) + (sj + 0.5f) / samples) /
                   img.height();
          auto ray = eval_camera(cam, {u, v});
          img.at(i, j) += shade(scn, lights, amb, ray);
        }
      }
      img[{i, j}] /= (float)(samples * samples);
    }
  }

  return img;
}

ym::image4f raytrace_mt(const yscn::scene* scn, const ym::vec3f& amb,
                        int resolution, int samples, bool facets) {
  auto cam = scn->cameras.front();
  auto img = ym::image4f(
      (int)std::round(cam->aspect * resolution), resolution, {0, 0, 0, 0});

  auto lights = std::vector<yscn::instance*>();
  for (auto ist : scn->instances) {
    if (ist->shp->mat->ke == ym::zero3f) continue;
    if (ist->shp->points.empty()) continue;
    lights.push_back(ist);
  }

  auto nthreads = std::thread::hardware_concurrency();
  auto threads = std::vector<std::thread>();
  for (auto tid = 0; tid < nthreads; tid++) {
    threads.push_back(std::thread([=, &img]() {
      for (auto j = tid; j < img.height(); j += nthreads) {
        for (auto i = 0; i < img.width(); i++) {
          img[{i, j}] = {0, 0, 0, 0};
          for (auto sj = 0; sj < samples; sj++) {
            for (auto si = 0; si < samples; si++) {
              auto u = (i + (si + 0.5f) / samples) / img.width();
              auto v =
                  ((img.height() - j) + (sj + 0.5f) / samples) /
                  img.height();
              auto ray = eval_camera(cam, {u, v});
              img.at(i, j) += shade(scn, lights, amb, ray);
            }
          }
          img[{i, j}] /= (float)(samples * samples);
        }
      }
    }));
  }
  for (auto& thread : threads) thread.join();
  return img;
}


//
// Displace each vertex of the shape along the normal of a distance
// <texture_value> * scale. After displacement, compute smooth normals
// with `ym::compute_normals()`.
//
void displace(yscn::shape* shp, float scale) {

  scale=0.25f;
  auto tex = shp->mat->disp_txt.txt;

  for(auto q:shp->quads)
  {
    shp->pos[q.x]+=((eval_texture(tex,shp->texcoord[q.x])).xyz()*scale)*shp->norm[q.x];
    shp->pos[q.y]+=((eval_texture(tex,shp->texcoord[q.y])).xyz()*scale)*shp->norm[q.y];
    shp->pos[q.z]+=((eval_texture(tex,shp->texcoord[q.z])).xyz()*scale)*shp->norm[q.z];
    shp->pos[q.w]+=((eval_texture(tex,shp->texcoord[q.w])).xyz()*scale)*shp->norm[q.w];
  }

  ym::compute_normals((int)shp->quads.size(), shp->quads.data(), (int)shp->pos.size(), shp->pos.data(), shp->norm.data());

}

void add_triangle(yscn::shape* new_shp, std::vector<ym::vec3f> vp, std::map<ym::vec3f,int>* vec_map){
  new_shp->triangles.push_back(ym::vec3i({vec_map->at(vp[0]),vec_map->at((vp[0]+vp[1])/ym::vec3f(2)),vec_map->at((vp[0]+vp[2])/ym::vec3f(2))}));
  new_shp->triangles.push_back(ym::vec3i({vec_map->at((vp[0]+vp[1])/ym::vec3f(2)),vec_map->at((vp[1]+vp[2])/ym::vec3f(2)),vec_map->at((vp[0]+vp[2])/ym::vec3f(2))}));
  new_shp->triangles.push_back(ym::vec3i({vec_map->at((vp[0]+vp[1])/ym::vec3f(2)),vec_map->at(vp[1]),vec_map->at((vp[1]+vp[2])/ym::vec3f(2))}));
  new_shp->triangles.push_back(ym::vec3i({vec_map->at((vp[0]+vp[2])/ym::vec3f(2)),vec_map->at((vp[1]+vp[2])/ym::vec3f(2)),vec_map->at(vp[2])}));
}
void add_quad(yscn::shape* new_shp, std::vector<ym::vec3f> vp, std::map<ym::vec3f,int>* vec_map, ym::vec3f poseC, tesselation* t, bool deg=false){
  if(!deg){
    new_shp->quads.push_back(ym::vec4i({vec_map->at(vp[0]),
                                        vec_map->at((vp[0]+vp[1])/ym::vec3f(2)),
                                        vec_map->at(poseC),
                                        vec_map->at((vp[0]+vp[3])/ym::vec3f(2))}));
    t->add_face(vp[0],
                (vp[0]+vp[1]/ym::vec3f(2)),
                poseC,
                (vp[0]+vp[3]/ym::vec3f(2)),
                new_shp->quads.back());

    new_shp->quads.push_back(ym::vec4i({vec_map->at((vp[0]+vp[1])/ym::vec3f(2)),
                                        vec_map->at(vp[1]),
                                        vec_map->at((vp[1]+vp[2])/ym::vec3f(2)),
                                        vec_map->at(poseC)}));
    t->add_face((vp[0]+vp[1]/ym::vec3f(2)),
                vp[1],
                (vp[1]+vp[2]/ym::vec3f(2)),
                poseC,
                new_shp->quads.back());

    new_shp->quads.push_back(ym::vec4i({vec_map->at(poseC),
                                        vec_map->at((vp[1]+vp[2])/ym::vec3f(2)),
                                        vec_map->at(vp[2]),
                                        vec_map->at((vp[2]+vp[3])/ym::vec3f(2))}));
    t->add_face(poseC,
                (vp[1]+vp[2]/ym::vec3f(2)),
                vp[2],
                (vp[2]+vp[3]/ym::vec3f(2)),
                new_shp->quads.back());

    new_shp->quads.push_back(ym::vec4i({vec_map->at((vp[3]+vp[0])/ym::vec3f(2)),
                                        vec_map->at(poseC),
                                        vec_map->at((vp[2]+vp[3])/ym::vec3f(2)),
                                        vec_map->at(vp[3])}));
    t->add_face((vp[3]+vp[0]/ym::vec3f(2)),
                poseC,
                (vp[2]+vp[3]/ym::vec3f(2)),
                vp[3],
                new_shp->quads.back());
  }
  else{
    new_shp->quads.push_back(ym::vec4i({vec_map->at(vp[0]),
                                        vec_map->at((vp[0]+vp[1])/ym::vec3f(2)),
                                        vec_map->at(poseC),
                                        vec_map->at((vp[0]+vp[2])/ym::vec3f(2))}));
    t->add_face(vp[0],
                (vp[0]+vp[1]/ym::vec3f(2)),
                poseC,
                (vp[0]+vp[2]/ym::vec3f(2)),
                new_shp->quads.back());

    new_shp->quads.push_back(ym::vec4i({vec_map->at((vp[0]+vp[1])/ym::vec3f(2)),
                                        vec_map->at(vp[1]),
                                        vec_map->at((vp[1]+vp[2])/ym::vec3f(2)),
                                        vec_map->at(poseC)}));
    t->add_face((vp[0]+vp[1]/ym::vec3f(2)),
                vp[1],
                (vp[1]+vp[2]/ym::vec3f(2)),
                poseC,
                new_shp->quads.back());

    new_shp->quads.push_back(ym::vec4i({vec_map->at(poseC),
                                        vec_map->at((vp[1]+vp[2])/ym::vec3f(2)),
                                        vec_map->at(vp[2]),
                                        vec_map->at((vp[0]+vp[2])/ym::vec3f(2))}));
    t->add_face(poseC,
                (vp[1]+vp[2]/ym::vec3f(2)),
                vp[2],
                (vp[0]+vp[2]/ym::vec3f(2)),
                new_shp->quads.back());
  }
}

void add_corner(yscn::shape* new_shp, std::map<ym::vec3f,int>* vec_map,
                std::vector<ym::vec3f> vp,std::vector<ym::vec2f> tx,
                int j, int total_pos){
  vec_map->at(vp[j]) = total_pos;
  new_shp->pos.at(vec_map->at(vp[j]))=vp[j];
  if(!new_shp->texcoord.empty())
    new_shp->texcoord.at(vec_map->at(vp[j]))=tx[j];
}
void add_edge(yscn::shape* new_shp, std::map<ym::vec3f,int>* vec_map,
              std::vector<ym::vec3f> vp,std::vector<ym::vec2f> tx,
              int j, int skip, int total_pos, int vn) {
  vec_map->at((vp[j] + vp[(j + skip) % vn])/ ym::vec3f(2)) = total_pos;
  new_shp->pos.at(vec_map->at((vp[j] + vp[(j + skip) % vn]) / ym::vec3f(2))) = (vp[j] + vp[(j + skip) % vn]) / ym::vec3f(2);
  if(!new_shp->texcoord.empty())
    new_shp->texcoord.at(vec_map->at((vp[j] + vp[(j + skip) % vn]) / ym::vec3f(2))) = (tx[j] + tx[(j + skip) % vn]) / ym::vec2f(2);
}


//
// Linear tesselation thta split each triangle in 4 triangles and each quad in
// four quad. Vertices are placed in teh middle of edges and faces. See slides
// on subdivision for the geometry.
// The tesselation operation should be repeated `level` times.
// To simplify coding, shapes are either quads ot triangles (depending on which
// array is filled). In the case of quad meshes, we include triangles by
// specifying a degenerate quad that has the last vertex index duplicated as
// v0, v1, v2, v2.
// Implement a different algorithm for quead meshes and triangle meshes.
//
void tesselate(yscn::shape* shp, int level, tesselation &tes) {

  yscn::shape new_shp;
  bool tex = !shp->texcoord.empty();
  std::map<ym::vec3f,int> vec_map;

  std::vector<ym::vec3f> vp;
  vp.resize(4);
  std::vector<ym::vec2f> tx;
  tx.resize(4);

  ym::vec3f poseC;
  ym::vec2f texC;

  int vn;
  int total_pos = 0;

  if(!shp->quads.empty()||!shp->triangles.empty()){

    !shp->quads.empty() ? vn=4 : vn=3;

    for(int l=0; l<level; ++l) {

      new_shp = yscn::shape();
      tes.clear();
      vec_map.clear();
      total_pos = 0;

      new_shp.pos.resize(shp->pos.size()*(vn+1));


      if(tex) {
        new_shp.texcoord.resize(shp->pos.size()*(vn+1));
        tx.resize(shp->texcoord.size()*vn +1);
      }

      for (int i = 0; i <(vn == 3 ? shp->triangles.size() : shp->quads.size()) ; ++i) {

        poseC = ym::vec3f();
        texC = ym::vec2f();
        for (int j = 0; j < vn; ++j) {

          if(vn==3){
            vp.at(j)=shp->pos[shp->triangles.at(i).operator[](j)];
            if(tex)
              tx.at(j)=shp->texcoord[shp->triangles.at(i).operator[](j)];
          }
          else{
            vp.at(j)=shp->pos[shp->quads.at(i).operator[](j)];
            if(tex)
              tx.at(j)=shp->texcoord[shp->quads.at(i).operator[](j)];
          }
          poseC+=vp[j];
          if(tex)
            texC+=tx[j];
        }

        if(vn==4){
          poseC/=ym::vec3f(4);
          if(tex)
            texC/=ym::vec2f(4);

          //add centroid
          vec_map[poseC]=total_pos;
          ++total_pos;
        }

        if (vn ==4 && shp->quads.at(i).z == shp->quads.at(i).w) {
          int skip = 0;
          for (int j = 0; j < 4; ++j) {

            (j < 2 ? skip=1 : skip=2);

            //add corner vertex
            if(vec_map[vp[j]]==0) {
              add_corner(&new_shp, &vec_map, vp, tx,j, total_pos);
              ++total_pos;
            }

            //add first edge vertex
            if(vec_map[(vp[j]+vp[(j+skip)%vn])/ym::vec3f(2)]==0){
              add_edge(&new_shp, &vec_map, vp, tx, j, skip, total_pos, vn);
              ++total_pos;
            }

            //add second edge vertex
            if(vec_map[(vp[j]+vp[(j+(vn-skip))%vn])/ym::vec3f(2)]==0) {
              add_edge(&new_shp, &vec_map, vp, tx, j, vn-skip, total_pos, vn);
              ++total_pos;
            }

            if(vn==4){
              new_shp.pos.at(vec_map[poseC])=poseC;
              if(tex)
                new_shp.texcoord.at(vec_map[poseC])=texC;
            }

          }

          add_quad(&new_shp, vp, &vec_map,poseC,&tes, true);

        }
        else {
          for (int j = 0; j < vn; ++j) {

            //add corner vertex
            if(vec_map[vp[j]]==0) {
              add_corner(&new_shp, &vec_map, vp, tx,j, total_pos);
              ++total_pos;
            }

            //add first edge vertex
            if(vec_map[(vp[j]+vp[(j+1)%vn])/ym::vec3f(2)]==0){//va diviso per 2
              add_edge(&new_shp, &vec_map, vp, tx, j, 1, total_pos, vn);
              ++total_pos;
            }

            //add second edge vertex
            if(vec_map[(vp[j]+vp[(j+(vn-1))%vn])/ym::vec3f(2)]==0) {
              add_edge(&new_shp, &vec_map, vp, tx, j, (vn-1), total_pos, vn);
              ++total_pos;
            }

            if(vn==4){
              new_shp.pos.at(vec_map[poseC])=poseC;
              if(tex)
                new_shp.texcoord.at(vec_map[poseC])=texC;
            }

          }

          if(vn==4)
            add_quad(&new_shp, vp, &vec_map,poseC, &tes);
          else
            add_triangle(&new_shp, vp, &vec_map);


        }

      }//*******fine quad***********
      shp->quads=new_shp.quads;
      shp->pos=new_shp.pos;
      shp->pos.resize(total_pos);
      shp->norm.resize(total_pos);
      if(tex) {
        shp->texcoord=new_shp.texcoord;
        shp->texcoord.resize(total_pos);
      }
    }
  }

  tes.vec_map=vec_map;
  ym::compute_normals((int)shp->quads.size(), shp->quads.data(), (int)shp->pos.size(), shp->pos.data(), shp->norm.data());

}

//
// Implement Catmull-Clark subdivision with the algorithm specified in class.
// You should only handle quad meshes, but note that some quad might be
// degenerate and in facgt represent a triangle as described above.
// The whole subvision should be repeated `level` timers.
// At the end, smooth the normals with `ym::compute_normals()`.
//
void catmull_clark(yscn::shape* shp, int level) {

  //step 1
  tesselation tes;
  tesselate(shp,level,tes);

  //step 2
  std::vector<ym::vec3f> avg_v = std::vector<ym::vec3f>(shp->pos.size());
  std::vector<ym::vec3f> avg_n = std::vector<ym::vec3f>(shp->pos.size());

  ym::vec3f c;
  for(int f=0;f<tes.get_faces().size();++f){
    c=tes.get_faces().at(f).c;
    for(int i=0; i<4; ++i){
      auto x=tes.get_quads().at(f).operator[](i);
      avg_v.at(x) += c;
      avg_n.at(x) += ym::vec3f(1);
    }
  }
  for(int i=0; i< avg_n.size(); ++i)
    avg_v.at(i) /= avg_n.at(i);

  //step 3
  ym::vec3f four = ym::vec3f(4);
  for(int i=0; i<shp->pos.size(); ++i)
    shp->pos.at(i) += (avg_v.at(i)-shp->pos.at(i))*(four/avg_n.at(i));
    //tes.get_vertices().at(i) +=
      //  (avg_v.at(i)-tes.get_vertices().at(i))*(four/avg_n.at(i));


}

//
// Add nhair to surfaces by creating a hair shape, made of lines, that is
// sampled over a shape `shp`. Each hair is a line of one segment that starts
// at a location on the surface and grows along the normals to a distance
// `length`. Each hair has the surface normal as it norm and the surface
// texcoord as its texcoord. Use the function `ym::sample_triangles_points()`
// with a seed of 0 to generate the surface pos, norm and texcoord.
//
yscn::shape* make_hair(
    const yscn::shape* shp, int nhair, float length, float radius) {
  auto hair = new yscn::shape();
  // YOUR CODE GOES HERE
  return hair;
}

inline std::array<ym::vec3f, 4> make_rnd_curve(
    const ym::vec3f& pos, const ym::vec3f& norm) {
  // HACK: this is a cheese hack that works
  auto rng = ym::init_rng_pcg32(*(uint32_t*)&pos.x, *(uint32_t*)&pos.y);
  auto v0 = pos;
  auto v1 = v0 + norm * 0.1f +
            (ym::next_rand3f(rng) - ym::vec3f{0.5f, 0.5f, 0.5f}) * 0.05f;
  auto v2 = v1 + norm * 0.1f +
            (ym::next_rand3f(rng) - ym::vec3f{0.5f, 0.5f, 0.5f}) * 0.05f;
  auto v3 = v2 + norm * 0.1f +
            (ym::next_rand3f(rng) - ym::vec3f{0.5f, 0.5f, 0.5f}) * 0.05f;
  return {{v0, v1, v2, v3}};
}

//
// Add ncurve Bezier curves to the shape. Each curcve should be sampled with
// `ym::sample_triangles_points()` and then created using `make_rnd_curve()`.
// The curve is then tesselated as lines using uniform subdivision with `nsegs`
// segments per curve. the final shape contains all tesselated curves as lines.
//
yscn::shape* make_curves(
    const yscn::shape* shp, int ncurve, int nsegs, float radius) {
  auto hair = new yscn::shape();
  // YOUR CODE GOES HERE
  return hair;
}

int main(int argc, char** argv) {
  // command line parsing
  auto parser =
      yu::cmdline::make_parser(argc, argv, "raytrace", "raytrace scene");
  auto parallel =
      yu::cmdline::parse_flag(parser, "--parallel", "-p", "runs in parallel");
  auto facets =
      yu::cmdline::parse_flag(parser, "--facets", "-f", "use facet normals");
  auto nhair = yu::cmdline::parse_opti(
      parser, "--hair", "-H", "number of hairs to generate", 0);
  auto ncurve = yu::cmdline::parse_opti(
      parser, "--curve", "-C", "number of curves to generate", 0);
  auto subdiv =
      yu::cmdline::parse_flag(parser, "--subdiv", "-S", "enable subdivision");
  auto tesselation = yu::cmdline::parse_flag(
      parser, "--tesselation", "-T", "enable tesselation");
  auto resolution = yu::cmdline::parse_opti(
      parser, "--resolution", "-r", "vertical resolution", 720);
  auto samples = yu::cmdline::parse_opti(
      parser, "--samples", "-s", "per-pixel samples", 1);
  auto amb = yu::cmdline::parse_optf(
      parser, "--ambient", "-a", "ambient color", 0.1f);
  auto imageout = yu::cmdline::parse_opts(
      parser, "--output", "-o", "output image", "out.png");
  auto scenein = yu::cmdline::parse_args(
      parser, "scenein", "input scene", "scene.obj", true);
  yu::cmdline::check_parser(parser);

  // load scene
  yu::logging::log_info("loading scene " + scenein);
  auto load_opts = yscn::load_options{};
  load_opts.preserve_quads = true;
  auto scn = yscn::load_scene(scenein, load_opts);

  yu::logging::log_info("applying subdivision");

  // apply subdivision
  if (subdiv || tesselation) {
    for (auto shp : scn->shapes) {
      // HACK: pick subdivision level from name
      if (!yu::string::startswith(shp->name, "subdiv_")) continue;
      auto level = 0;
      if (yu::string::startswith(shp->name, "subdiv_01_")) level = 1;
      if (yu::string::startswith(shp->name, "subdiv_02_")) level = 2;
      if (yu::string::startswith(shp->name, "subdiv_03_")) level = 3;
      if (yu::string::startswith(shp->name, "subdiv_04_")) level = 4;
      if (subdiv) {
        catmull_clark(shp, level);
      } else {
        struct tesselation tes;
        tesselate(shp, level, tes);
      }
    }
  }
  yu::logging::log_info("subdivision applied");


  // handle displacement
  for (auto shp : scn->shapes) {
    if (!shp->mat->disp_txt.txt) continue;
    displace(shp, 1);
  }

  // handle faceted rendering
  if (facets) {
    for (auto shp : scn->shapes) {
      if (!shp->triangles.empty()) {
        ym::facet_triangles(shp->triangles, shp->pos, shp->norm,
                            shp->texcoord, shp->color, shp->radius);
        ym::compute_normals(shp->triangles, shp->pos, shp->norm);
      } else if (!shp->quads.empty()) {
        ym::facet_quads(shp->quads, shp->pos, shp->norm, shp->texcoord,
                        shp->color, shp->radius);
        ym::compute_normals(shp->quads, shp->pos, shp->norm);
      } else if (!shp->lines.empty()) {
        ym::facet_lines(shp->lines, shp->pos, shp->norm, shp->texcoord,
                        shp->color, shp->radius);
        ym::compute_tangents(shp->lines, shp->pos, shp->norm);
      }
    }
  }

  // add missing data
  auto add_opts = yscn::add_elements_options::none();
  add_opts.smooth_normals = true;
  add_opts.pointline_radius = 0.001f;
  add_opts.shape_instances = true;
  add_opts.default_camera = true;
  yscn::add_elements(scn, add_opts);

  // convert quads to triangles
  for (auto shp : scn->shapes) {
    if (shp->quads.empty()) continue;
    shp->triangles = ym::convert_quads_to_triangles(shp->quads);
    shp->quads.clear();
  }

  // make hair
  if (nhair) {
    for (auto ist : scn->instances) {
      // HACK skip by using name
      if (ist->shp->name == "floor") continue;
      if (ist->shp->triangles.empty()) continue;
      auto hair = make_hair(ist->shp, nhair, 0.1f, 0.001f);
      hair->name = ist->shp->name + "_hair";
      hair->mat = new yscn::material();
      hair->mat->kd = ist->shp->mat->kd;
      hair->mat->kd_txt = ist->shp->mat->kd_txt;
      auto hist = new yscn::instance();
      hist->frame = ist->frame;
      hist->shp = hair;
      hist->name = ist->name + "_hair";
      scn->instances.push_back(hist);
      scn->shapes.push_back(hist->shp);
      scn->materials.push_back(hair->mat);
    }
  }

  // make curve
  if (ncurve) {
    for (auto ist : scn->instances) {
      // HACK skip by using name
      if (ist->shp->name == "floor") continue;
      if (ist->shp->triangles.empty()) continue;
      auto hair = make_curves(ist->shp, ncurve, 8, 0.001f);
      hair->name = ist->shp->name + "_curve";
      hair->mat = new yscn::material();
      hair->mat->kd = ist->shp->mat->kd;
      hair->mat->kd_txt = ist->shp->mat->kd_txt;
      auto hist = new yscn::instance();
      hist->frame = ist->frame;
      hist->shp = hair;
      hist->name = ist->name + "_hair";
      scn->instances.push_back(hist);
      scn->shapes.push_back(hair);
      scn->materials.push_back(hair->mat);
    }
  }

  // create bvh
  yu::logging::log_info("creating bvh");
  yscn::build_bvh(scn);

  // raytrace
  yu::logging::log_info("tracing scene");
  auto hdr =
      (parallel)
      ? raytrace_mt(scn, {amb, amb, amb}, resolution, samples, facets)
      : raytrace(scn, {amb, amb, amb}, resolution, samples, facets);
  // tonemap and save
  yu::logging::log_info("saving image " + imageout);
  auto ldr = ym::tonemap_image(hdr, ym::tonemap_type::srgb, 0, 2.2);
  yimg::save_image4b(imageout, ldr);
}
