
#include "../rays/tri_mesh.h"
#include "lib/mat4.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect
    
    float x_min = std::min({vertex_list[v0].position.x,
                           vertex_list[v1].position.x,
                           vertex_list[v2].position.x});
    float y_min = std::min({vertex_list[v0].position.y,
                           vertex_list[v1].position.y,
                           vertex_list[v2].position.y});
    float z_min = std::min({vertex_list[v0].position.z,
                           vertex_list[v1].position.z,
                           vertex_list[v2].position.z});

    float x_max = std::max({vertex_list[v0].position.x,
                           vertex_list[v1].position.x,
                           vertex_list[v2].position.x});
    float y_max = std::max({vertex_list[v0].position.y,
                           vertex_list[v1].position.y,
                           vertex_list[v2].position.y});
    float z_max = std::max({vertex_list[v0].position.z,
                           vertex_list[v1].position.z,
                           vertex_list[v2].position.z});

    Vec3 vec_min(x_min, y_min, z_min);
    Vec3 vec_max(x_max, y_max, z_max);

    BBox box(vec_min, vec_max);
    return box;
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    // See rays/tri_mesh.h for a description of this struct
    
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    
    // (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the above three points.
    // Intersection should yield a ray t-value, and a hit point (u,v) on the surface of the triangle

    // Define barycentric weights
    Vec3 e_1 = v_1.position - v_0.position;
    Vec3 e_2 = v_2.position - v_0.position;
    Vec3 s = ray.point - v_0.position;
    Vec3 d = ray.dir;

    // Use Cramer's Rule
    float frac = 1 / (dot(cross(e_1, d), e_2));
    float det_x = dot(-cross(s, e_2), d);
    float det_y = dot(cross(e_1, d), s);
    float det_z = dot(-cross(s, e_2), e_1);

    Vec3 soln = frac * Vec3(det_x, det_y, det_z);

    // You'll need to fill in a "Trace" struct describing information about the hit (or lack of hit)
    float u = soln.x;
    float v = soln.y;
    float t = soln.z;

    Trace ret;
    // ret.origin = ray.point;
    ret.hit = (u > 0 &&
               v > 0 &&
               (u + v) <= 1 &&
               t > ray.dist_bounds.x &&
               t < ray.dist_bounds.y);          // intersection
    if (ret.hit) {
      ret.distance = t;                         // distance
      ret.position = (ray.dir * t) + ray.point; // position

      // vertex-interpolated normal
      Vec3 n_total = ((1 - u - v) * v_0.normal + u * v_1.normal + v * v_2.normal).unit();
      ret.normal = n_total;

      // update ray dist_bounds
      ray.dist_bounds = Vec2(ray.dist_bounds.x, t);
    }

    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
