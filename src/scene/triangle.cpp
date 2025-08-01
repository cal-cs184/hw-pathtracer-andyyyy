#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"
#include "scene/object.h"
#include "vector3D.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  constexpr float EPSILON = std::numeric_limits<float>::epsilon();

  Vector3D edge1 = p2 - p1;
  Vector3D edge2 = p3 - p1;
  Vector3D rayVector = r.d;
  Vector3D rayCrossEdge2 = cross(rayVector, edge2);

  double det = dot(edge1, rayCrossEdge2);
  if (det > -EPSILON && det < EPSILON)
    return false;

  float invDet = 1.0 / det;
  Vector3D originMinusVertex0 = r.o - p1;
  float baryU = invDet * dot(originMinusVertex0, rayCrossEdge2);
  if (baryU < 0.0 || baryU > 1.0) {
    return false;
  }

  Vector3D originMinusVertex0CrossEdge1 = cross(originMinusVertex0, edge1);
  float baryV = invDet * dot(rayVector, originMinusVertex0CrossEdge1);
  if (baryV < 0.0 || baryU + baryV > 1.0) {
    return false;
  }

  float t = invDet * dot(edge2, originMinusVertex0CrossEdge1);
  if (t < r.min_t || t > r.max_t) {
    return false;
  }

  r.max_t = t;

  return true;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  constexpr float EPSILON = std::numeric_limits<float>::epsilon();

  Vector3D edge1 = p2 - p1;
  Vector3D edge2 = p3 - p1;
  Vector3D rayVector = r.d;
  Vector3D rayCrossEdge2 = cross(rayVector, edge2);

  double det = dot(edge1, rayCrossEdge2);
  if (det > -EPSILON && det < EPSILON)
    return false;

  float invDet = 1.0 / det;
  Vector3D originMinusVertex0 = r.o - p1;
  float baryU = invDet * dot(originMinusVertex0, rayCrossEdge2);
  if (baryU < 0.0 || baryU > 1.0) {
    return false;
  }

  Vector3D originMinusVertex0CrossEdge1 = cross(originMinusVertex0, edge1);
  float baryV = invDet * dot(rayVector, originMinusVertex0CrossEdge1);
  if (baryV < 0.0 || baryU + baryV > 1.0) {
    return false;
  }

  float t = invDet * dot(edge2, originMinusVertex0CrossEdge1);
  if (t < r.min_t || t > r.max_t) {
    return false;
  }

  r.max_t = t;

  isect->t = t;
  isect->primitive = this;
  isect->n = ((1 - baryU - baryV) * n1) + (baryU * n2) + (baryV * n3);
  isect->bsdf = get_bsdf();

  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
