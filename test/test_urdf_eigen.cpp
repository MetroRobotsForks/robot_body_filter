#include "gtest/gtest.h"
#include <robot_body_filter/utils/urdf_eigen.hpp>
#include <urdf_model/model.h>

TEST(UrdfEigen, PoseTransform)
{
  urdf::Pose uPose;
  uPose.position = {1.0, 2.0, 3.0};
  uPose.rotation.setFromRPY(0, M_PI_2, M_PI);

  const auto ePose = robot_body_filter::urdfPose2EigenTransform(uPose);

  EXPECT_DOUBLE_EQ(uPose.position.x, ePose.translation().x());
  EXPECT_DOUBLE_EQ(uPose.position.y, ePose.translation().y());
  EXPECT_DOUBLE_EQ(uPose.position.z, ePose.translation().z());

  const Eigen::Quaterniond quat(ePose.rotation());
  EXPECT_DOUBLE_EQ(uPose.rotation.x, quat.x());
  EXPECT_DOUBLE_EQ(uPose.rotation.y, quat.y());
  EXPECT_DOUBLE_EQ(uPose.rotation.z, quat.z());
  EXPECT_DOUBLE_EQ(uPose.rotation.w, quat.w());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}