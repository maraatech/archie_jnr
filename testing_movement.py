import rospy
from cares_msgs.msg import PlatformGoalActionGoal
from geometry_msgs.msg import PoseStamped
import sys
from cares_lib_ros import utils

def generate_platform_goal(xyz, rpy, arm="a"):
    goal = PlatformGoalActionGoal()
    pose = PoseStamped()
    goal.goal.command = goal.goal.MOVE
    goal.goal.link_id.data = f"{arm}/stereo_pair/left_frame"
    goal.goal.path_constraints.volume_constraint = 1
    goal.goal.path_constraints.allowed_planning_time = 1.0

    
    init_pose = utils.create_pose_stamped_msg(float(xyz[0]), 
                                              float(xyz[1]), 
                                              float(xyz[2]), 
                                              f"{arm}/planning_link",
                                              rpy_deg=[float(rpy[0]), 
                                                       float(rpy[1]),
                                                       float(rpy[2])])
            
    goal.goal.target_pose = init_pose



    return goal

if __name__ == "__main__":
    rospy.init_node("arm_testing")
    rospy.loginfo("Arm testing initializing...")
    a_pub = rospy.Publisher("/a/ur_arm_a_server/goal", PlatformGoalActionGoal, queue_size=10)
    b_pub = rospy.Publisher("/b/ur_arm_b_server/goal", PlatformGoalActionGoal, queue_size=10)

    xyz = [float(x) for x in sys.argv[1].split(" ")]
    rpy = [float(x) for x in sys.argv[2].split(" ")]

    a_goal = generate_platform_goal(xyz, rpy)
    b_goal = generate_platform_goal(xyz, rpy, arm="b")
    print(a_goal)

    rospy.sleep(0.5)

    a_pub.publish(a_goal)
    rospy.sleep(1.5)
    b_pub.publish(b_goal)

    rospy.spin()