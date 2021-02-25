import rospy
import roslib
import sys
sys.path.append(roslib.packages.get_pkg_dir('point_cloud_tools'))
from point_cloud_tools.srv import *
from pypcd import PointCloud 
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

pkg_path = roslib.packages.get_pkg_dir('point_cloud_tools')

SERVICE_NAME = "TableTop_PointCloud_Pose_Estimation"
MESH_PATH = pkg_path + "/data/mesh.stl"
CLOUD_PATH = pkg_path + "/data/cloud.pcd"

def main():
    rospy.wait_for_service(SERVICE_NAME)
    req = rospy.ServiceProxy(SERVICE_NAME, MeshToCloudPose)

    object_cloud = PointCloud.from_path(CLOUD_PATH)
   
    table_cloud = PointCloud.from_path(CLOUD_PATH)
    resp=req(MESH_PATH, object_cloud.to_msg(), table_cloud.to_msg())
    print("service response [start]")
    print(resp)
    print("service response [end]")

    '''
    The code below is just for your sake to
    convince yourself that it's working by visualizing mesh and pcd.
    '''

    rospy.init_node("demo")

    # visualize object cloud
    pcd_pub = rospy.Publisher("pcd", sensor_msgs.msg.PointCloud2, queue_size=1)
    pcd_msg = object_cloud.to_msg()
    pcd_msg.header.frame_id = "world"
    pcd_msg.header.stamp = rospy.Time.now()
    rate = rospy.Rate(10)
    for i in range(10):
        pcd_pub.publish(pcd_msg)
        rate.sleep()
    print("pcd visualized")

    # visualize mesh that needs to fit
    mesh_original = rospy.Publisher("mesh_original", Marker, queue_size=1)
    marker_original = get_mesh_marker()
    for i in range(10):
        mesh_original.publish(marker_original)
        rate.sleep()
    print("mesh visualized")
    
    # visualize result
    mesh_fit = rospy.Publisher("mesh_fit", Marker, queue_size=1)
    marker_fit = get_mesh_marker()
    marker_fit.pose = resp.estimated_pose.pose
    for i in range(10):
        mesh_fit.publish(marker_fit)
        rate.sleep()
    print("fitted mesh visualized")

def get_mesh_marker():
    marker = Marker()
    marker.type = marker.MESH_RESOURCE
    marker.pose = Pose()
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.pose.orientation.w = 1.
    marker.mesh_resource = "package://point_cloud_tools/data/mesh.stl"
    marker.mesh_use_embedded_materials = True
    marker.header.frame_id = "world"

    return marker



if __name__ == '__main__':
    main()
