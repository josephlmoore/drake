#ifndef PRISMATICJOINT_H_
#define PRISMATICJOINT_H_

#include "FixedAxisOneDoFJoint.h"

class PrismaticJoint: public FixedAxisOneDoFJoint
{
  // disable copy construction and assignment
  PrismaticJoint(const PrismaticJoint&) = delete;
  PrismaticJoint& operator=(const PrismaticJoint&) = delete;

private:
  Eigen::Vector3d translation_axis;
public:
  PrismaticJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Vector3d& translation_axis);

  virtual ~PrismaticJoint();

  virtual Eigen::Isometry3d jointTransform(double* const q) const; //override;

private:
  static Eigen::Matrix<double, TWIST_SIZE, 1> spatialJointAxis(const Eigen::Vector3d& translation_axis);
};

#endif /* PRISMATICJOINT_H_ */
