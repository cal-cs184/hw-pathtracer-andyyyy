#include "sphere.h"

#include <cmath>
#include <system_error>
#include <utility>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"
#include "vector3D.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  double a = dot(r.d, r.d);
  double b = dot((2 * (r.o - o)), r.d);
  double c = dot((r.o - o), (r.o - o)) - r2;

  double disc = pow(b, 2) - (4 * a * c);
  if (disc <= 0) {
    return false;
  }

  t1 = (-b - sqrt(disc)) / (2 * a);
  t2 = (-b + sqrt(disc)) / (2 * a);
  if ((t1 < r.min_t || t1 > r.max_t) && (t2 < r.min_t || t2 > r.max_t)) {
    return false;
  }

  if (t2 < t1) {
    swap(t1, t2);
  }

  r.max_t = t1;

  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1;
  double t2;
  return test(r, t1, t2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1;
  double t2;
  if (!(test(r, t1, t2))) {
    return false;
  }

  Vector3D intersectPt = r.o + (t1 * r.d);
  i->n = (intersectPt - o).unit();
  i->t = t1;
  i->primitive = this;
  i->bsdf = get_bsdf();

  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
