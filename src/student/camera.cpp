
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"
#include <math.h>

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    //
    // The input screen_coord is a normalized screen coordinate [0,1]^2
    //
    // You need to transform this 2D point into a 3D position on the sensor plane, which is
    // located one unit away from the pinhole in camera space (aka view space).
    //
    // You'll need to compute this position based on the vertial field of view
    // (vert_fov) of the camera, and the aspect ratio of the output image (aspect_ratio).
    //
    // Tip: compute the ray direction in view space and use
    // the camera space to world space transform (iview) to transform the ray back into world space.
    // Create ray origin
    Vec3 origin(0, 0, 0);
    
    // Create ray direction. Start with 2D
    Vec2 pix_screen(2 * screen_coord.x - 1, 2 * screen_coord.y - 1);
    float fov_rads = vert_fov * M_PI / 180;
    float tan_fov = tan(fov_rads / 2);
    Vec3 pix_camera(pix_screen.x * aspect_ratio * tan_fov, 
                    pix_screen.y * tan_fov,
                    -1.0);
    // Convert from view space to world-space. cf. iview
    Vec3 dir(pix_camera - origin);

    // Construct ray
    Ray out(origin, dir);
    out.transform(iview);

    return out;
}
