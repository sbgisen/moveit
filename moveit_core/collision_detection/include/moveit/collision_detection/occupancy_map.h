/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Jon Binney */

#pragma once

#include <geometric_shapes/shapes.h>
#include <octomap/octomap.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/function.hpp>
#include <memory>
#include "moveit/robot_model/link_model.h"
#include <ros/console.h>
#include <shape_msgs/SolidPrimitive.h>

#include <geometric_shapes/shapes.h>  // Include the missing header file
namespace collision_detection
{
typedef octomap::OcTreeNode OccMapNode;

class OccMapTree : public octomap::OcTree
{
public:
  OccMapTree(double resolution) : octomap::OcTree(resolution)
  {
  }

  OccMapTree(const std::string& filename) : octomap::OcTree(filename)
  {
  }

  /** @brief lock the underlying octree. it will not be read or written by the
   *  monitor until unlockTree() is called */
  void lockRead()
  {
    tree_mutex_.lock_shared();
  }

  /** @brief unlock the underlying octree. */
  void unlockRead()
  {
    tree_mutex_.unlock_shared();
  }

  /** @brief lock the underlying octree. it will not be read or written by the
   *  monitor until unlockTree() is called */
  void lockWrite()
  {
    tree_mutex_.lock();
  }

  /** @brief unlock the underlying octree. */
  void unlockWrite()
  {
    tree_mutex_.unlock();
  }

  void clearBBX(const octomap::point3d& min, const octomap::point3d& max)
  {
    for (octomap::OcTree::leaf_bbx_iterator it = this->begin_leafs_bbx(min, max), end = this->end_leafs_bbx();
         it != end; ++it)
    {
      it->setLogOdds(octomap::logodds(this->getClampingThresMin()));
    }
    this->updateInnerOccupancy();
  }

  void clearShape(const shapes::ShapeConstPtr& shape, const Eigen::Isometry3d& origin)
  {
    // Get the type of the shape and handle accordingly
    switch (shape->type)
    {
      case shapes::BOX: {
        const shapes::Box* box = static_cast<const shapes::Box*>(shape.get());
        clearBox(*box, origin);
        break;
      }
      case shapes::SPHERE: {
        const shapes::Sphere* sphere = static_cast<const shapes::Sphere*>(shape.get());
        clearSphere(*sphere, origin);
        break;
      }
      case shapes::CYLINDER: {
        const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(shape.get());
        clearCylinder(*cylinder, origin);
        break;
      }
      case shapes::CONE: {
        const shapes::Cone* cone = static_cast<const shapes::Cone*>(shape.get());
        clearCone(*cone, origin);
        break;
      }
      case shapes::MESH: {
        const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
        clearMesh(*mesh, origin);
        break;
      }
      default:
        ROS_ERROR("Unsupported shape type for clearing octomap");
        break;
    }
  }

  void clearBox(const shapes::Box& box, const Eigen::Isometry3d& origin)
  {
    // Get max and min xyz values from eight corners of the oriented box
    double x_min, y_min, z_min, x_max, y_max, z_max;
    x_min = y_min = z_min = std::numeric_limits<double>::max();
    x_max = y_max = z_max = -std::numeric_limits<double>::max();
    ROS_INFO("Clearing box of size %f %f %f", box.size[0], box.size[1], box.size[2]);
    for (int i = -1; i <= 1; i += 2)
    {
      for (int j = -1; j <= 1; j += 2)
      {
        for (int k = -1; k <= 1; k += 2)
        {
          Eigen::Vector3d corner =
              origin * Eigen::Vector3d(i * box.size[0] / 2.0, j * box.size[1] / 2.0, k * box.size[2] / 2.0);
          x_min = std::min(x_min, corner.x());
          y_min = std::min(y_min, corner.y());
          z_min = std::min(z_min, corner.z());
          x_max = std::max(x_max, corner.x());
          y_max = std::max(y_max, corner.y());
          z_max = std::max(z_max, corner.z());
        }
      }
    }
    // Clear octomap inside the oriented box
    octomap::point3d area_min = octomap::point3d(x_min, y_min, z_min);
    octomap::point3d area_max = octomap::point3d(x_max, y_max, z_max);
    for (octomap::OcTree::leaf_bbx_iterator it = this->begin_leafs_bbx(area_min, area_max), end = this->end_leafs_bbx();
         it != end; ++it)
    {
      // check current boxel is inside the oriented box
      Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());
      p = origin.inverse() * p;
      if (p.x() >= -box.size[0] / 2.0 && p.x() <= box.size[0] / 2.0 && p.y() >= -box.size[1] / 2.0 &&
          p.y() <= box.size[1] / 2.0 && p.z() >= -box.size[2] / 2.0 && p.z() <= box.size[2] / 2.0)
      {
        it->setLogOdds(octomap::logodds(this->getClampingThresMin()));
      }
    }
  }

  void clearSphere(const shapes::Sphere& sphere, const Eigen::Isometry3d& origin)
  {
    // Clear octomap inside the sphere
    ROS_INFO("Clearing sphere of radius %f", sphere.radius);
    octomap::point3d center(origin.translation().x(), origin.translation().y(), origin.translation().z());
    for (octomap::OcTree::leaf_bbx_iterator
             it = this->begin_leafs_bbx(center - octomap::point3d(sphere.radius, sphere.radius, sphere.radius),
                                        center + octomap::point3d(sphere.radius, sphere.radius, sphere.radius)),
             end = this->end_leafs_bbx();
         it != end; ++it)
    {
      // check current boxel is inside the sphere
      Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());
      p = origin.inverse() * p;
      if (p.norm() <= sphere.radius)
      {
        it->setLogOdds(octomap::logodds(this->getClampingThresMin()));
      }
    }
  }

  void clearCylinder(const shapes::Cylinder& cylinder, const Eigen::Isometry3d& origin)
  {
    // Clear octomap inside the cylinder
    ROS_INFO("Clearing cylinder of radius %f and length %f", cylinder.radius, cylinder.length);
    octomap::point3d center(origin.translation().x(), origin.translation().y(), origin.translation().z());
    for (octomap::OcTree::leaf_bbx_iterator
             it = this->begin_leafs_bbx(
                 center - octomap::point3d(cylinder.radius, cylinder.radius, cylinder.length / 2),
                 center + octomap::point3d(cylinder.radius, cylinder.radius, cylinder.length / 2)),
             end = this->end_leafs_bbx();
         it != end; ++it)
    {
      // check current boxel is inside the cylinder
      Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());
      p = origin.inverse() * p;
      if (p.x() * p.x() + p.y() * p.y() <= cylinder.radius * cylinder.radius && std::abs(p.z()) <= cylinder.length / 2)
      {
        it->setLogOdds(octomap::logodds(this->getClampingThresMin()));
      }
    }
  }

  void clearCone(const shapes::Cone& cone, const Eigen::Isometry3d& origin)
  {
    // Clear octomap inside the cone
    ROS_INFO("Clearing cone of radius %f and length %f", cone.radius, cone.length);
    octomap::point3d center(origin.translation().x(), origin.translation().y(), origin.translation().z());
    for (octomap::OcTree::leaf_bbx_iterator
             it = this->begin_leafs_bbx(center - octomap::point3d(cone.radius, cone.radius, cone.length / 2),
                                        center + octomap::point3d(cone.radius, cone.radius, cone.length / 2)),
             end = this->end_leafs_bbx();
         it != end; ++it)
    {
      // check current boxel is inside the cone
      Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());
      p = origin.inverse() * p;
      if (p.x() * p.x() + p.y() * p.y() <= (cone.radius * cone.radius) * (1 - p.z() / (cone.length / 2)))
      {
        it->setLogOdds(octomap::logodds(this->getClampingThresMin()));
      }
    }
  }

  void clearMesh(const shapes::Mesh& mesh, const Eigen::Isometry3d& origin)
  {
    // Calculate the axis-aligned bounding box for the mesh in the transformed space
    double x_min = std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max();
    double z_min = std::numeric_limits<double>::max();
    double x_max = -std::numeric_limits<double>::max();
    double y_max = -std::numeric_limits<double>::max();
    double z_max = -std::numeric_limits<double>::max();

    for (size_t i = 0; i < mesh.vertex_count; ++i)
    {
      Eigen::Vector3d vertex(mesh.vertices[3 * i], mesh.vertices[3 * i + 1], mesh.vertices[3 * i + 2]);
      Eigen::Vector3d transformed_vertex = origin * vertex;

      x_min = std::min(x_min, transformed_vertex.x());
      y_min = std::min(y_min, transformed_vertex.y());
      z_min = std::min(z_min, transformed_vertex.z());
      x_max = std::max(x_max, transformed_vertex.x());
      y_max = std::max(y_max, transformed_vertex.y());
      z_max = std::max(z_max, transformed_vertex.z());
    }

    octomap::point3d min(x_min, y_min, z_min);
    octomap::point3d max(x_max, y_max, z_max);

    // Clear the octomap in the axis-aligned bounding box
    for (octomap::OcTree::leaf_bbx_iterator it = this->begin_leafs_bbx(min, max), end = this->end_leafs_bbx();
         it != end; ++it)
    {
      it->setLogOdds(octomap::logodds(this->getClampingThresMin()));
    }
  }

  using ReadLock = boost::shared_lock<boost::shared_mutex>;
  using WriteLock = boost::unique_lock<boost::shared_mutex>;

  ReadLock reading()
  {
    return ReadLock(tree_mutex_);
  }

  WriteLock writing()
  {
    return WriteLock(tree_mutex_);
  }

  void triggerUpdateCallback()
  {
    if (update_callback_)
      update_callback_();
  }

  /** @brief Set the callback to trigger when updates are received */
  void setUpdateCallback(const boost::function<void()>& update_callback)
  {
    update_callback_ = update_callback;
  }

private:
  boost::shared_mutex tree_mutex_;
  boost::function<void()> update_callback_;
};

using OccMapTreePtr = std::shared_ptr<OccMapTree>;
using OccMapTreeConstPtr = std::shared_ptr<const OccMapTree>;
}  // namespace collision_detection
