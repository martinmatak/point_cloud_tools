#include "transformation_analyzation.h"
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

template <typename PointT>
void TransformationAnalyzation<PointT>::yawObject(tf::Transform &transform, double radian)
{
  tf::Vector3 Rot_Axis(0,0,1);
  tf::Quaternion object_rotation(Rot_Axis, radian);
  transform.setRotation(object_rotation);
  //return transform;
}

// table compute methods
template <typename PointT>
double TransformationAnalyzation<PointT>::getAreaofBB(BBox BB)
{
  double length = BB.max_x - BB.min_x;
  double width = BB.max_y - BB.min_y;
  return length * width;
}

template <typename PointT>
size_t TransformationAnalyzation<PointT>::findTable(const std::vector<pcl::PointIndices> &inlier_inds) const
{
    size_t tableIndex = 0;
    for (size_t i = 1; i < inlier_inds.size(); i++)
    {
        tableIndex = inlier_inds[i].indices.size() > inlier_inds[tableIndex].indices.size() ? i : tableIndex;
    }
    return tableIndex;
}

template <typename PointT>
std::vector<size_t> TransformationAnalyzation<PointT>::findMultiplePlanes(size_t tableIndex,
                                                                  const std::vector<pcl::ModelCoefficients> &models) const {
    std::vector<size_t> ret;
    auto tableNormal = normalize(models[tableIndex].values);

    for (size_t i = 0; i < models.size(); i++) {
        if (i == tableIndex)
            continue;
        float theta = computeTheta(tableNormal, normalize(models[i].values));
        if (theta < PI / 8)
            ret.push_back(i);
    }

    return ret;
}

template <typename PointT>
double TransformationAnalyzation<PointT>::computeYaw(CloudConstPtr cloud)
{
  tf::Transform temptf;
  tf::Vector3 translation(0,0,0);
  temptf.setOrigin(translation);
  return computeYaw(cloud, temptf);
}

template <typename PointT>
double TransformationAnalyzation<PointT>::computeYaw(CloudConstPtr cloud, tf::Transform transform)
{
  CloudPtr tempcloud(new Cloud(*cloud));
  tf::Transform temptf = transform;
  BBox BB = computeAABoundingBox(tempcloud, temptf);
  double Area0 = getAreaofBB(BB);
  double radian = 0;
  double step = 0.017453;
  radian += step;
  yawObject(temptf, radian);
  BB = computeAABoundingBox(tempcloud, temptf);
  double Area1 = getAreaofBB(BB);
  std::stringstream ys;
  while (Area0 != Area1)
    {
      //ys << "Areas are: " << Area0 << " " << Area1 << "\n";
      //ys << "Trans Z: " << transform.getRotation().z();
      //ROS_INFO("%s",ys.str().c_str());
      if(Area1 > Area0)
	step = -step / 2;
      Area0 = Area1;
      radian += step;
      yawObject(temptf, radian);
      BB = computeAABoundingBox(tempcloud, temptf);
      Area1 = getAreaofBB(BB);
    }
  //ys << "Areas are: " << Area0 << " " << Area1 << "\n";
  //ys << "Yaw Angle: " << radian << "\n";
  //ROS_INFO("%s",ys.str().c_str());
  //ROS_INFO("Has Converged");
  return radian;
}

template <typename PointT>
struct BBox TransformationAnalyzation<PointT>::computeCompositeAABoundingBox(CloudConstPtr cloud1, const tf::Transform transform1, CloudConstPtr cloud2, const tf::Transform transform2)
{
  struct BBox box1 = computeAABoundingBox(cloud1, transform1);
  struct BBox box2 = computeAABoundingBox(cloud2, transform2);
  struct BBox ret;
  ret.min_x = std::min(box1.min_x, box2.min_x);
  ret.max_x = std::max(box1.max_x, box2.max_x);
  ret.min_y = std::min(box1.min_y, box2.min_y);
  ret.max_y = std::max(box1.max_y, box2.max_y);
  ret.min_z = std::min(box1.min_z, box2.min_z);
  ret.max_z = std::max(box1.max_z, box2.max_z);
  return ret;
}

struct BBox computeCompositeAABoundingBox(struct BBox box1, struct BBox box2)
{
  struct BBox ret;
  ret.min_x = std::min(box1.min_x, box2.min_x);
  ret.max_x = std::max(box1.max_x, box2.max_x);
  ret.min_y = std::min(box1.min_y, box2.min_y);
  ret.max_y = std::max(box1.max_y, box2.max_y);
  ret.min_z = std::min(box1.min_z, box2.min_z);
  ret.max_z = std::max(box1.max_z, box2.max_z);
  return ret;
}

template <typename PointT>
struct BBox TransformationAnalyzation<PointT>::computeBoundingBox(CloudConstPtr cloud)
{
 struct BBox ret(FLT_MAX, FLT_MIN, FLT_MAX, FLT_MIN);
 //pcl::PointCloud<pcl::PointXYZ>::iterator i;
 for (auto i = cloud->points.begin(); i < cloud->points.end(); i++)
   {
     if(std::isnormal(i->x))
       {
	 ret.max_x = std::max(ret.max_x, i->x);
	 ret.min_x = std::min(ret.min_x, i->x);
       }
     if(std::isnormal(i->y))
       {
	 ret.max_y = std::max(ret.max_y, i->y);
	 ret.min_y = std::min(ret.min_y, i->y);
       }
     if(std::isnormal(i->z))
       {
	 ret.max_z = std::max(ret.max_z, i->z);
	 ret.min_z = std::min(ret.min_z, i->z);
       }
   }
 //std::stringstream bs;
 return ret;
}

template <typename PointT>
struct BBox TransformationAnalyzation<PointT>::computeAABoundingBox(CloudConstPtr cloud, const tf::Transform transform)
{
  CloudPtr transcloud(new Cloud(*cloud));
  Eigen::Affine3d trans;
  tf::transformTFToEigen(transform, trans);
  pcl::transformPointCloud(*cloud, *transcloud, trans);
  struct BBox AABox = computeBoundingBox(transcloud);
  return AABox;
}

template <typename PointT>
struct BBox TransformationAnalyzation<PointT>::computeAABoundingBoxOfPlane(const CloudConstPtr cloud, const pcl::PointIndices &inlier, const tf::Transform transform) const
{
  CloudPtr transcloud(new Cloud(*cloud));
  Eigen::Affine3d trans;
  tf::transformTFToEigen(transform, trans);
  pcl::transformPointCloud(*cloud, *transcloud, trans.inverse());
  struct BBox AABox = computeBoundingBoxOfPlane(transcloud, inlier);
  return AABox;
}

template <typename PointT>
struct BBox TransformationAnalyzation<PointT>::computeBoundingBoxOfPlane(CloudConstPtr cloud,
                                                                const pcl::PointIndices &inlier) const
{
    // initialize the max values with max float values and vice versa for min
    struct BBox ret(FLT_MAX, FLT_MIN, FLT_MAX, FLT_MIN);
    for (size_t j = 0; j < inlier.indices.size(); j++)
    {
        // if float is normal (ie not zero/infinite/NaN etc)
        if (std::isnormal(cloud->points[inlier.indices[j]].x))
        {
            ret.max_x = std::max(ret.max_x, cloud->points[inlier.indices[j]].x);
            ret.min_x = std::min(ret.min_x, cloud->points[inlier.indices[j]].x);
        }
        if (std::isnormal(cloud->points[inlier.indices[j]].y))
        {
            ret.max_y = std::max(ret.max_y, cloud->points[inlier.indices[j]].y);
            ret.min_y = std::min(ret.min_y, cloud->points[inlier.indices[j]].y);
        }
        if (std::isnormal(cloud->points[inlier.indices[j]].z))
        {
            ret.max_z = std::max(ret.max_z, cloud->points[inlier.indices[j]].z);
            ret.min_z = std::min(ret.min_z, cloud->points[inlier.indices[j]].z);
        }
    }
    return ret;
}

template <typename PointT>
float TransformationAnalyzation<PointT>::computeMaxDistance(CloudConstPtr cloud, const pcl::PointIndices &inlier) const
{
    float max_dis = -1;
    for (size_t i = 0; i < 1000 && i < inlier.indices.size(); i++)
    {
        size_t iIndex = rand() % inlier.indices.size();
        if (std::isnormal(cloud->points[inlier.indices[iIndex]].x)
            && std::isnormal(cloud->points[inlier.indices[iIndex]].y))
        {
            float tmp = FLT_MAX;
            for (size_t j = 0; j < inlier.indices.size(); j++)
            {
                if (iIndex == j)
                    continue;

                if (std::isnormal(cloud->points[inlier.indices[j]].x)
                    && std::isnormal(cloud->points[inlier.indices[j]].y))
                {
                    float dx = cloud->points[inlier.indices[iIndex]].x -
                               cloud->points[inlier.indices[j]].x;
                    float dy = cloud->points[inlier.indices[iIndex]].y -
                               cloud->points[inlier.indices[j]].y;
                    float sq = std::sqrt(dx * dx + dy * dy);
                    if (!isNearZero(sq))
                    {
                        tmp = std::min(tmp, sq);
                    }
                }
            }
            max_dis = std::max(max_dis, tmp);
        }
        else
        {
            i--;
        }
    }
    return max_dis;
}

template <typename PointT>
std::unordered_map<std::pair<int, int>, std::vector<int> > TransformationAnalyzation<PointT>::remapPointCloudWithErosion(
        CloudConstPtr cloud, const pcl::PointIndices &inlier,
        const struct BBox &bb, float max_dis, const struct Size &size) const
{
    // map indices into size
    std::unordered_map<std::pair<int, int>, std::vector<int> > ret;
    for (size_t i = 0; i < inlier.indices.size(); i++)
    {
        int xIndex = (int) floor((cloud->points[inlier.indices[i]].x - bb.min_x) / max_dis);
        int yIndex = (int) floor((cloud->points[inlier.indices[i]].y - bb.min_y) / max_dis);
        std::pair<int, int> tmp = std::make_pair(xIndex, yIndex);
        auto it = ret.find(tmp);
        if (it == ret.end())
        {
            std::vector<int> tmp2;
            tmp2.push_back(inlier.indices[i]);
            ret.insert(std::make_pair(tmp, tmp2));
        }
        else // if indices found, already store their indices
        {
            it->second.push_back(inlier.indices[i]);
        }
    }

    // erosion
    std::unordered_map<std::pair<int, int>, std::vector<int> > tmpMap;
    const int limit = 10;
    for (auto itt = ret.begin(); itt != ret.end(); ++itt)
    {
        int fillCount = 0;
        auto it = itt->first;
        for (int i = 1; i <= limit; i++)
        {
            if ((it.first) + i >= size.width)
            {
                fillCount = 0;
                break;
            }
            if (ret.find(std::make_pair((it.first) + i, it.second)) == ret.end())
            {
                if (i == limit)
                    fillCount = 0;
                else
                    fillCount = i;
            } else
            {
                break;
            }
        }
        for (size_t i = 1; i <= fillCount; i++)
        {
            if ((it.first) + i < size.width)
            {
                std::vector<int> tmp2;
                tmpMap.insert(std::make_pair(std::make_pair((it.first) + i, it.second), tmp2));
            }
        }
        fillCount = 0;
        for (int i = 1; i <= limit; i++)
        {
            if ((it.second) + i >= size.height)
            {
                fillCount = 0;
                break;
            }
            if (ret.find(std::make_pair(it.first, (it.second) + i)) == ret.end())
            {
                if (i == limit)
                    fillCount = 0;
                else
                    fillCount = i;
            } else
            {
                break;
            }
        }
        for (size_t i = 1; i <= fillCount; i++)
        {
            if ((it.second) + i < size.height)
            {
                std::vector<int> tmp2;
                tmpMap.insert(std::make_pair(std::make_pair(it.first, i + (it.second)), tmp2));
            }
        }
    }

    ret.insert(tmpMap.begin(), tmpMap.end());
    return ret;
}

template class TransformationAnalyzation<pcl::PointXYZRGBA>;

template class TransformationAnalyzation<pcl::PointXYZ>;
