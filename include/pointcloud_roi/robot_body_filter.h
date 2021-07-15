#ifndef ROBOT_BODY_FILTER_H
#define ROBOT_BODY_FILTER_H

#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

namespace pointcloud_roi
{

class RobotBodyFilter
{
public:
  RobotBodyFilter(const std::string &robot_description = "robot_description") : shape_mask_()
  {
    robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    model_frame_ = kinematic_model->getModelFrame();
    const std::vector<const moveit::core::LinkModel*>& links = kinematic_model->getLinkModelsWithCollisionGeometry();
    for (const moveit::core::LinkModel* link : links)
    {
      std::vector<shapes::ShapeConstPtr> shapes = link->getShapes();  // copy shared ptrs on purpuse
      for (std::size_t j = 0; j < shapes.size(); ++j)
        shape_mask_.addShape(shapes[j]);
    }
  }

  template <typename PointT, size_t VALUE=point_containment_filter::ShapeMask::OUTSIDE>
  pcl::IndicesPtr getIndices(const pcl::PointCloud<PointT> &input)
  {
    pcl::IndicesPtr mask(new pcl::Indices);
    for (size_t i=0; i < input.size(); i++)
    {
      const PointT &p = input.at(i);
      if (shape_mask_.getMaskContainment(p.x, p.y, p.z) == VALUE)
        mask->push_back(i);
    }
    return mask;
  }

  const std::string& getModelFrame()
  {
    return model_frame_;
  }

private:
  point_containment_filter::ShapeMask shape_mask_;
  std::string model_frame_;

};

} // namespace pointcloud_roi

#endif // ROBOT_BODY_FILTER_H
