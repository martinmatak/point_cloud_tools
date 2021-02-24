import rospy
import roslib
import sys
sys.path.append(roslib.packages.get_pkg_dir('point_cloud_tools'))
from point_cloud_tools.srv import *
from pypcd import PointCloud 

pkg_path = roslib.packages.get_pkg_dir('point_cloud_tools')

SERVICE_NAME = "TableTop_PointCloud_Pose_Estimation"
MESH_PATH = pkg_path + "/data/mesh.stl"
CLOUD_PATH = pkg_path + "/data/cloud.pcd"

def main():
    rospy.wait_for_service(SERVICE_NAME)
    req = rospy.ServiceProxy(SERVICE_NAME, MeshToCloudPose)

    mesh_path = MESH_PATH
    object_cloud = PointCloud.from_path(CLOUD_PATH)
    table_cloud = PointCloud.from_path(CLOUD_PATH)

    resp=req(mesh_path, object_cloud.to_msg(), table_cloud.to_msg())
    print(resp)



if __name__ == '__main__':
    main()
