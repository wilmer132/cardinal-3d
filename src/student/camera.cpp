
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"
#include <math.h>

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.
    
    // Create ray origin
    Vec3 origin(position);
    
    // Create ray direction. Start with 2D
    Vec2 pix_screen(2 * screen_coord.x - 1, 1 - 2 * screen_coord.y);
    float fov_rads = vert_fov * M_PI / 180;
    float tan_fov = tan(fov_rads / 2);
    Vec3 pix_camera((2 * pix_screen.x - 1) * aspect_ratio * tan_fov, 
                     (1 - 2 * pix_screen.y) * tan_fov, -1.0);
    // Convert from view space to world-space. cf. iview
    Vec3 dir(pix_camera - origin);

    // Construct ray
    Ray out(origin, dir);
    return out;
}
