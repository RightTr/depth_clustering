// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "utils/cloud.h"

namespace depth_clustering {

Cloud::Cloud(const Cloud& cloud)
    : _points{cloud.points()},
      _pose(cloud.pose()),
      _sensor_pose(cloud.sensor_pose()) {
  if (!cloud.projection_ptr()) {
    // no need to copy projection, there is none yet
    return;
  }
  // projection is a polymorphic type, we use clone therefore
  auto ptr = cloud.projection_ptr()->Clone();
  _projection = ptr;
}

std::list<const RichPoint*> Cloud::PointsProjectedToPixel(int row,
                                                          int col) const {
  std::list<const RichPoint*> point_list;
  if (!_projection) {
    return point_list;
  }
  for (const auto& index : _projection->at(row, col).points()) {
    point_list.push_back(&_points[index]);
  }
  return point_list;
}

void Cloud::TransformInPlace(const Pose& pose) {
  for (auto& point : _points) {
    point = pose * point.AsEigenVector();
  }
  // the projection makes no sense anymore after the coords of points changed.
  this->_projection.reset();
}

Cloud::Ptr Cloud::Transform(const Pose& pose) const {
  Cloud cloud_copy(*this);
  cloud_copy.TransformInPlace(pose);
  return make_shared<Cloud>(cloud_copy);
}

void Cloud::SetProjectionPtr(typename CloudProjection::Ptr proj_ptr) {
  _projection = proj_ptr;
}

void Cloud::InitProjection(const ProjectionParams& params) { //TODO:InitProjection
  if (_projection) {
    throw std::runtime_error("projection is already initialized");
  }
  _projection = CloudProjection::Ptr(new SphericalProjection(params));
  if (!_projection) {
    fprintf(stderr, "ERROR: failed to initalize projection.\n");
    return;
  }
  _projection = _projection->Clone();
  _projection->InitFromPoints(_points);
}

Cloud::Ptr Cloud::FromImage(const cv::Mat& image,
                            const ProjectionParams& params) {
  CloudProjection::Ptr proj = CloudProjection::Ptr(new RingProjection(params));
  proj->CheckImageAndStorage(image);
  proj->CloneDepthImage(image);
  Cloud cloud;
  for (int r = 0; r < image.rows; ++r) {
    for (int c = 0; c < image.cols; ++c) {
      if (image.at<float>(r, c) < 0.0001f) {
        continue;
      }
      RichPoint point = proj->UnprojectPoint(image, r, c);
      cloud.push_back(point);
      proj->at(r, c).points().push_back(cloud.points().size() - 1);
    }
  }
  cloud.SetProjectionPtr(proj);
  // we cannot share ownership of this cloud with others, so create a new one
  return boost::make_shared<Cloud>(cloud);
}

float Cloud::ComputePointsCenterZ() const //TODO:ComputePointsCenterZ
{
  float z_sum = 0.0;
  for(const auto& point : _points)
  {
    z_sum += point.z();
  }
  return z_sum / _points.size();
}

Eigen::Vector2f Cloud::ComputePointCenterXY() const
{
  float x_sum = 0.0;
  float y_sum = 0.0;
  for(const auto& point : _points)
  {
    x_sum += point.x();
    y_sum += point.y();
  }
  return Eigen::Vector2f(x_sum / _points.size(), y_sum / _points.size());
}

float Cloud::ComputeDistance2DMax() const //TODO:ComputeDistanceMax
{
  Eigen::Vector2f max_point(std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest());
  Eigen::Vector2f min_point(std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());
  for (const auto& point : _points) 
  {
    min_point << std::min(min_point.x(), point.x()),
        std::min(min_point.y(), point.y());
    max_point << std::max(max_point.x(), point.x()),
        std::max(max_point.y(), point.y());
  }
  return sqrt(max_point.x() * max_point.x() + min_point.y() * min_point.y());
}

Eigen::Vector4f Cloud::ComputeClusterCenterRadius() const
{
  Eigen::Vector4f cluster_ = Eigen::Vector4f::Zero();
  Eigen::Vector2f max_point(std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest());
  Eigen::Vector2f min_point(std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());
  for (const auto& point : _points) 
  {
    cluster_ += Eigen::Vector4f(point.x(), point.y(), point.z(), 0);
    min_point << std::min(min_point.x(), point.x()),
      std::min(min_point.y(), point.y());
    max_point << std::max(max_point.x(), point.x()),
      std::max(max_point.y(), point.y());
  }
  cluster_ /= static_cast<float>(_points.size());
  cluster_.w() = sqrt(max_point.x() * max_point.x() + min_point.y() * min_point.y());
  return cluster_;
}

bool Cloud::IsClustersOutside(const Pose& pose, 
          const float& width, const float& length) const
{
  // fprintf(stderr, "x:%f, y:%f, yaw:%f\n", pose.x(), pose.y(), pose.theta());
  Eigen::Vector2f cloudxy = ComputePointCenterXY();
  if(abs(cloudxy.x() * cosf(pose.theta())) > pose.x())
  {
    return true;
  }
  if(abs(cloudxy.x() * cosf(pose.theta())) > width - pose.x())
  {
    return true;
  }
  if(abs(cloudxy.y() * sinf(pose.theta())) > length)
  {
    return true;
  }
  if(abs(cloudxy.y() * sinf(pose.theta())) > length - pose.y())
  {
    return true;
  }
  return false;
}

// this code will be only there if we use pcl
#ifdef PCL_FOUND

typename pcl::PointCloud<pcl::PointXYZL>::Ptr Cloud::ToPcl() const {
  using pcl::PointXYZL;
  pcl::PointCloud<PointXYZL>::Ptr pcl_cloud;
  for (const auto& point : _points) {
    PointXYZL pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    pcl_point.label = point.ring();
    pcl_cloud->push_back(pcl_point);
  }
  return pcl_cloud;
}

template <>
Cloud::Ptr Cloud::FromPcl(const pcl::PointCloud<pcl::PointXYZL>& pcl_cloud) {
  Cloud cloud;
  for (const auto& pcl_point : pcl_cloud) {
    RichPoint point(pcl_point.x, pcl_point.y, pcl_point.z);
    point.ring() = pcl_point.label;
    cloud.push_back(point);
  }
  return make_shared<Cloud>(cloud);
}

#endif  // PCL_FOUND

}  // namespace depth_clustering
