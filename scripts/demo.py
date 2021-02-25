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
    rospy.init_node("demo")
    rospy.wait_for_service(SERVICE_NAME)
    req = rospy.ServiceProxy(SERVICE_NAME, MeshToCloudPose)

    mesh_path = MESH_PATH

    pc = PointCloud.from_path(CLOUD_PATH)
    pc.pc_data = [1,2,3]
    
    object_cloud = pc
    pcd_pub = rospy.Publisher("pcd", sensor_msgs.msg.PointCloud2, queue_size=1)
    pcd_msg = object_cloud.to_msg()
    pcd_msg.header.frame_id = "world"
    pcd_msg.header.stamp = rospy.Time.now()
    print(pcd_msg.header)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pcd_pub.publish(pcd_msg)
        rate.sleep()
    print("published")


    '''
    table_cloud = PointCloud.from_path(CLOUD_PATH)

    resp=req(mesh_path, object_cloud.to_msg(), table_cloud.to_msg())
    print(resp)
    '''




if __name__ == '__main__':
    main()
