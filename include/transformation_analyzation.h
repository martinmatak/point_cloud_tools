
#ifndef TRANSFORMATION_ANALYZATION_H
#define TRANSFORMATION_ANALYZATION_H

//#define TRANSFORMATION_ANALYZATION_DEBUG
//#define DEBUG_HASSAN

//#define PI 3.1415926

// PCL
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <unordered_map>

#include <tf/transform_broadcaster.h>

const double PI = 3.1415926;

struct BBox
{
  float min_x, max_x;
  float min_y, max_y;
  float min_z, max_z;
  
  BBox() {}
  
  BBox(float min_x, float max_x, float min_y, float max_y)
   : min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y) { }
  
  BBox(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
   : min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y), min_z(min_z), max_z(max_z) { }
};

struct Size {
    size_t width, height;

    Size() {}

    Size(size_t width, size_t height)
            : width(width), height(height) { }
};

template <class T>
inline void hash_combine(std::size_t & seed, const T & v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std
{
    template<typename S, typename T> struct hash<pair<S, T>>
    {
        inline size_t operator()(const pair<S, T> & v) const
        {
            size_t seed = 0;
            ::hash_combine(seed, v.first);
            ::hash_combine(seed, v.second);
            return seed;
        }
    };
}

struct BBox computeCompositeAABoundingBox(BBox B1, BBox B2);

template <typename PointT>
class TransformationAnalyzation {
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

public:
    size_t findTable(const std::vector<pcl::PointIndices> &inlier_inds) const;

    std::vector<size_t> findMultiplePlanes(size_t tableIndex, const std::vector<pcl::ModelCoefficients> &models) const;

  void yawObject(tf::Transform &transform, double radian);
  
  double computeYaw(CloudConstPtr Cloud);
  double computeYaw(CloudConstPtr Cloud, tf::Transform transform);
    
  double getAreaofBB(BBox BB);
  
  struct BBox computeCompositeAABoundingBox(CloudConstPtr cloud1, const tf::Transform transform1, CloudConstPtr cloud2, const tf::Transform transform2);
    
  struct BBox computeAABoundingBoxOfPlane(CloudConstPtr cloud, const pcl::PointIndices &inlier, const tf::Transform transform) const;

  struct BBox computeAABoundingBox(CloudConstPtr cloud, const tf::Transform transform);	    
    
  struct BBox computeBoundingBox(CloudConstPtr cloud);	

  struct BBox computeBoundingBoxOfPlane(CloudConstPtr cloud, const pcl::PointIndices &inlier) const;

  float computeMaxDistance(CloudConstPtr cloud, const pcl::PointIndices &inlier) const;

  std::unordered_map<std::pair<int, int>, std::vector<int> > remapPointCloudWithErosion(CloudConstPtr cloud,
											const pcl::PointIndices &inlier_inds,
											const struct BBox &bb, float max_dis,
											const struct Size &size) const;

  // inline - replace all instantiations of this function by the actual code of the function
  // the last const keyword makes it an error for this function to change any member variables of this (transformAnalyzation) class
  // TODO : Analyze the computeTransformMatrix function
  inline void transformPointCloud(const CloudConstPtr cloud, const CloudPtr transformed_cloud, const std::vector<float> &v) const
  {
#ifdef DEBUG_HASSAN
    std::cout << "Plane vector = " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << std::endl;
    Eigen::Affine3f ret;
    ret = computeTransformMatrix(v);
    std::cout << "Transformation matrix from input object cloud to 'biggest plane normal pointing in z direction' is : " << std::endl;
    std::cout << ret.matrix() << std::endl;
#endif
    pcl::transformPointCloud(*cloud, *transformed_cloud, computeTransformMatrix(v));
    }
  
  inline struct Size computeSize(const struct BBox &box, float d) const
  {
    // initialize a size with (width, height)
    // normalize both dimensions of the boxes with respect to the distance d
    struct Size size(computeCells(box.max_x, box.min_x, d), computeCells(box.max_y, box.min_y, d));
    return size;
  }

  inline Eigen::Affine3f computeTransformMatrix(const std::vector<float> &v) const
  {
    // row vector with a magnitude of 1
    // explicitly use only the first three elements of the plane equation
    const Eigen::Matrix<float, 1, 3> floor_plane_normal_vector = normalize(v);
    // unit row vector pointing in the z direction
    const Eigen::Matrix<float, 1, 3> xy_plane_normal_vector = normalize(0.0, 0.0, 1.0);

    const Eigen::Vector3f rotation_vector = floor_plane_normal_vector.cross(xy_plane_normal_vector);

#ifdef DEBUG_HASSAN
    std::cout << "Floor plane normal vector : " << floor_plane_normal_vector[0] << " "
	      << floor_plane_normal_vector[1] << floor_plane_normal_vector[2] << std::endl;
    std::cout << "Rotation vector: " << rotation_vector[0] << " " << rotation_vector[1] << " " << rotation_vector[2] << std::endl;
#endif

#ifdef TRANSFORMATION_ANALYZATION_DEBUG
    std::cout << "Rotation vector: " << rotation_vector[0] << " " << rotation_vector[1] << " " << rotation_vector[2] << std::endl;
#endif
    // calculate the angle between the two vectors
    float theta = computeTheta(floor_plane_normal_vector, xy_plane_normal_vector);
    // ret is a transformation matrix
    Eigen::Affine3f ret = Eigen::Affine3f::Identity();
    // ret is a rotation of -theta about rotation vector
    // TODO: visualize the rotations
    ret.rotate(Eigen::AngleAxisf(theta, rotation_vector));
    return ret;
  }

  inline size_t computeCells(float a, float b, float d) const
  {
    return (size_t) ceil((a - b) / d);
  }

private:
  // mathematics methods
  inline bool isNearZero(float x) const {
    const float epsilon = 0.001;
    return std::abs(x) < epsilon;
  }

  inline float length(float a, float b, float c) const {
    return sqrtf(a * a + b * b + c * c);
  }

  inline Eigen::Matrix<float, 1, 3> normalize(float a, float b, float c) const {
    float len = length(a, b, c);
    const Eigen::Matrix<float, 1, 3> ret{a / len, b / len, c / len};
    return ret;
  }

  inline Eigen::Matrix<float, 1, 3> normalize(const std::vector<float> &v) const {
    return normalize(v[0], v[1], v[2]);
  }

  inline float computeTheta(const Eigen::Matrix<float, 1, 3> &a, const Eigen::Matrix<float, 1, 3> &b) const {
    float theta = (float) acos(a.dot(b));

#ifdef TRANSFORMATION_ANALYZATION_DEBUG
    std::cout << "Dot value: " << a.dot(b) << std::endl;
    std::cout << "Original theta value: " << theta << std::endl;
#endif

    //if (theta > PI / 2)
    //  theta = PI - theta;

#ifdef TRANSFORMATION_ANALYZATION_DEBUG
    std::cout << "Theta value: " << theta << std::endl;
#endif
    return theta;
  }


};

#endif
