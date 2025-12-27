#!/usr/bin/env python3
import rospy
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def spawn_qr_model(model_name, pose, material_name):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Get the path to the generic poster SDF file
        sdf_path = os.path.join(rospy.get_param("~model_path"), "poster.sdf")
        
        with open(sdf_path, "r") as f:
            sdf_string = f.read()

        # Replace the material placeholder with the desired material
        # IMPORTANT: The placeholder is an empty <material/> tag
        sdf_string = sdf_string.replace(
            "<material/>",
            "<material><script><uri>model://robot_supurge_ros/models/materials</uri><name>{}</name></script></material>".format(material_name)
        )

        resp = spawn_model(model_name, sdf_string, "", pose, "world")
        rospy.loginfo("Spawned model: %s", model_name)
    except rospy.ServiceException as e:
        rospy.logerr("SpawnModel service call failed: {0}".format(e))

if __name__ == '__main__':
    rospy.init_node('spawn_qr_codes_node')
    
    # Give Gazebo some time to start up
    rospy.sleep(5.0)

    # Define model path
    model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models')
    rospy.set_param("~model_path", model_path)

    # --- Define QR Code Positions and Materials ---
    qr_codes = {
        "qr_code_corridor": ((-5.189, 2.125, 1.0, 1.57), "QRCode/Corridor"),
        "qr_code_livingroom": ((-0.110, -0.093, 1.0, 1.57), "QRCode/LivingRoom"),
        "qr_code_kitchen": ((3.036, -0.089, 1.0, 1.57), "QRCode/Kitchen"),
        "qr_code_bedroom": ((5.378, -0.088, 1.0, 0.0), "QRCode/Bedroom")
    }

    for name, ((x, y, z, yaw), material) in qr_codes.items():
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        q = quaternion_from_euler(0, 0, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        spawn_qr_model(name, pose, material)
