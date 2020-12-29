// A test program for asserting the difference and the relation between
// matrices, vectors, and quaternions, in glm, eigen, and trackball.
//
// Dov Grobgeld <dov.grobgeld@gmail.com>
// 2020-12-29 Tue

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>
#include "trackball.h"
#include <fmt/core.h>

using namespace Eigen;
using namespace std;
using namespace fmt;

// Print a floating matrix from a pointer
void print_mat(float *m)
{
  for (int i=0; i<16; i++)
    {
      print("{:.2} ", m[i]);
      if ((i+1)%4==0)
        print("\n");
    }
}

// Assert matrix equality from a vector
void assert_eqm(float *m1, float *m2)
{
  float eps=1e-6;
  for (int i=0; i<16; i++)
    if (fabs(m1[i]-m2[i])>eps)
      {
        print("m1=\n");
        print_mat(m1);
        print("m2=\n");
        print_mat(m2);
        fflush(stdout);
        throw runtime_error("Unequal matrices!");
      }
}

void print_vec(float*v)
{
  print("v= ({:.2f} {:.2f} {:.2f})\n",v[0],v[1],v[2]);
}

// Print a floating matrix from a pointer
void assert_eqv(float *v1, float *v2)
{
  double eps=1e-10;
  for (int i=0; i<3; i++)
    if (fabs(v1[i]-v2[i])>eps)
      {
        print("v1= ({:.2f} {:.2f} {:.2f})\n",v1[0],v1[1],v1[2]);
        print("v2= ({:.2f} {:.2f} {:.2f})\n",v2[0],v2[1],v2[2]);
        throw runtime_error("Unequal vectors!");
      }
}

int main(int argc, char **argv)
{
  // Just asserting that two identify matrices pass the assertion.
  glm::mat4 gm1(1.0);
  glm::mat4 gm2(1.0);
  assert_eqm(&gm1[0][0],&gm2[0][0]);

  // Create same matrices in both glm and eigen and compare
  Eigen::Affine3f em1(Eigen::Affine3f::Identity());
  Eigen::Affine3f em2(Eigen::Affine3f::Identity());
  assert_eqm(&gm1[0][0], em1.data());
  
  // Translate both. This tests that the data order is the same!
  gm1 = glm::translate(gm1, glm::vec3(1,2,3));
  em1.translate(Eigen::Vector3f(1,2,3)); // always inline
  assert_eqm(&gm1[0][0], em1.data());

  // Chaining translations
  gm2 = glm::translate(gm1, glm::vec3(10,0,0));
  em2 = em1;
  em2.translate(Eigen::Vector3f(10,0,0));

  gm1 = gm1*gm2;
  em1 = em1*em2;
  assert_eqm(&gm1[0][0], em1.data());

  // Test non-commutative transformations multiplication order
  const double deg2rad = 3.1415926535/180;
  double rotx = 15*deg2rad, roty = -8*deg2rad;

  gm1 = glm::rotate<float>(rotx,glm::vec3(1,0,0));
  gm2 = glm::rotate<float>(roty,glm::vec3(0,1,0));
  em1 = Eigen::AngleAxis<float>(rotx, Eigen::Vector3f(1,0,0));
  em2 = Eigen::AngleAxis<float>(roty,Eigen::Vector3f(0,1,0));
  assert_eqm(&gm1[0][0], em1.data());
  assert_eqm(&gm2[0][0], em2.data());

  // Chain transformations
  gm1 = gm1*gm2;
  em1 = em1*em2;
  assert_eqm(&gm1[0][0], em1.data());
  
  // Trivial vector test
  glm::vec3 gv1 { 1,2,3}, gv2;
  Eigen::Vector3f ev1(1,2,3), ev2;
  assert_eqv(&gv1[0],ev1.data());

  // Compare rotate around z with eigen and glm
  float angle = 3.141/3;
  gv2 = glm::rotateZ(gv1, angle);
  ev2 = Eigen::AngleAxis<float>(angle,Eigen::Vector3f(0,0,1))*ev1;
  assert_eqv(&gv2[0],ev2.data());
  print_vec(&gv2[0]);

  // quaternions
  glm::quat gq1(1.0,0.1,0.2,0.3), gq2(1.0,0.2,0.3,0.4);
  gm1 = glm::toMat4(gq1);
  gm2 = glm::toMat4(gq2);

  Eigen::Quaternion<float> eq1(1.0,0.1,0.2,0.3), eq2(1.0,0.2,0.3,0.4);
  em1 = eq1; // Transform quaternion to affine matrix

  assert_eqm(&gm1[0][0],em1.data());

  // Compare composition of two quaternions
  gq1 = gq1 * gq2;
  eq1 = eq1 * eq2;
  gm1 = glm::toMat4(gq1);
  em1 = eq1; 
  assert_eqm(&gm1[0][0],em1.data());

  // Test track ball quats. Note that they have different signs
  // and different order!
  float tq2[4] = { -0.2,-0.3,-0.4,1.0}; // Note negative sign!!!
  float md[4][4];
  build_rotmatrix(md, tq2);
  assert_eqm(&md[0][0],&gm2[0][0]);
  build_rotmatrix_glm(gm1, tq2);
  assert_eqm(&gm1[0][0],&gm2[0][0]);

  print("ok\n");
  exit(0);
}
