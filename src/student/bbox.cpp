
#include "../lib/mathlib.h"
#include "lib/mat4.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    /* 6 Different Faces:
     * - V0, V1, V2, V4: F0
     * - V1, V6, V4, V7: F1
     * - V6, V3, V7, V5: F2
     * - V3, V0, V5, V2: F3
     * - V2, V4, V5, V7: F4
     * - V0, V1, V3, V6: F5
     * 12 Different Triangles 
     */
    std::vector<Vec3> b_verts = corners();
    std::vector<std::vector<Vec3>> bbox_tri = {
      {b_verts[0], b_verts[1], b_verts[2]}, // F0
      {b_verts[4], b_verts[2], b_verts[1]},
      {b_verts[1], b_verts[6], b_verts[4]}, // F1
      {b_verts[7], b_verts[4], b_verts[6]},
      {b_verts[6], b_verts[3], b_verts[7]}, // F2
      {b_verts[5], b_verts[7], b_verts[3]},
      {b_verts[3], b_verts[0], b_verts[5]}, // F3
      {b_verts[2], b_verts[5], b_verts[0]},
      {b_verts[2], b_verts[4], b_verts[5]}, // F4
      {b_verts[7], b_verts[5], b_verts[4]},
      {b_verts[0], b_verts[1], b_verts[3]}, // F5
      {b_verts[6], b_verts[3], b_verts[1]},
    };
    
    bool bbox_hit = false;

    // check if any bbox triangle is hit
    for (size_t i = 0; i < bbox_tri.size(); i++) {
      // Define barycentric weights
      Vec3 e_1 = bbox_tri[i][1] - bbox_tri[i][0];
      Vec3 e_2 = bbox_tri[i][2] - bbox_tri[i][0];
      Vec3 s = ray.point - bbox_tri[i][0];
      Vec3 d = ray.dir;

      // Use Cramer's Rule
      float frac = 1 / (dot(cross(e_1, d), e_2));
      float det_x = dot(-cross(s, e_2), d);
      float det_y = dot(cross(e_1, d), s);
      float det_z = dot(-cross(s, e_2), e_1);

      Vec3 soln = frac * Vec3(det_x, det_y, det_z);

      // You'll need to fill in a "Trace" struct describing 
      // information about the hit (or lack of hit)
      float u = soln.x;
      float v = soln.y;
      float t = soln.z;

      bbox_hit = 
        (u > 0 &&
         v > 0 &&
         (u + v) <= 1 &&
         t > ray.dist_bounds.x && // boundaries
         t < ray.dist_bounds.y &&
         t >= times.x &&
         t <= times.y); // intersection
      
      // update times boundaries
      if (bbox_hit) {
        times = Vec2(times.x, t);
      }
    }

    return bbox_hit;
}
