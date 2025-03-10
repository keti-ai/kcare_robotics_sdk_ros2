from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.master.kcare_utils import *
from rosinterfaces.action import SendStringData as ActSendData
from rosinterfaces.srv import SendStringData as SrvSendData

from pyconnect.utils import str2dict, data_info, dict2str
from kcare_robot_ros2_controller.src.master.vision_calibration import Head2BaseCalibration

import time

class KcareMaster(Node):
    def __init__(self):
        super().__init__('kcare_master')

        self.rbutils=RobotUtils(self)
        self.srv_callback_group = MutuallyExclusiveCallbackGroup()
        self.femto_calib=Head2BaseCalibration()

        #TODO 메시지 안에 height 정보 계산결과 추가하여 해당 변수 제거
        self.obj_height=0.0
        self.place_height=0.0

        SERVICE_SERVER ={
            'pick':self.skill_pick_callback,
            'place':self.skill_place_callback,
        }
        self.service_server = {}
        for service_tag, service_callbackfn in SERVICE_SERVER.items():
            self.service_server[service_tag] = self.create_service(
                SrvSendData,
                service_tag,
                service_callbackfn,
                callback_group=self.srv_callback_group
            )
            self.get_logger().info(f"Service Server created: {service_tag} -> {service_callbackfn}")

    def dict2pose(self,data):
        obj_dict=data['ins']
        obj_name=next(iter(obj_dict))
        obj_pose=obj_dict[obj_name]['pose'][obj_name]
        obj_pose3d=obj_dict[obj_name]['pose3d']

        return obj_pose3d

    def dict2pickpose(self,data):
        obj_pose=data['pose_list'][0]
        return obj_pose

    def dict2placepose(self,data):
        obj_pose=data['pose_list'][1]
        return obj_pose

    def skill_pick_callback(self,request,response):
        self.get_logger().info(f"Pick Skill Processing")

        rev_data = str2dict(request.req)
        pose=self.dict2pickpose(rev_data)



        self.rbutils.call_gripper_command(RobotParam.grip_open)
        self.rbutils.call_elevation_command(RobotParam.elev_home)
        self.rbutils.call_set_servo_angle(RobotParam.arm_ready)

        mid_x = (pose[0]+pose[2])/2
        mid_y = (pose[1]+pose[3])/2
        obj_depth = pose[4]

        cur_lift_position=self.rbutils.get_elev_pose()*1000
        cur_arm_robot_pose=self.rbutils.get_robot_pose()['pose']
        cur_head_pose=self.rbutils.get_head_pose()
        cur_angle_ry=copy.deepcopy(cur_head_pose[1])
        cur_angle_rz=copy.deepcopy(cur_head_pose[0])


        self.obj_height=self.femto_calib.convert_femto_object2Height(pose, cur_head_pose)

        self.trans_lift_position,self.trans_armrobot_XYZ=self.femto_calib.convert_femto_to_arm(mid_x,mid_y,obj_depth,#[x,y,z] mm 
                                                                                cur_lift_position, # scalar (meter)
                                                                                cur_arm_robot_pose, # x y z r p y
                                                                                cur_angle_ry,# scalar
                                                                                cur_angle_rz# scalar
                                                                                )
        self.trans_x,self.trans_y,self.trans_z=self.trans_armrobot_XYZ
        self.get_logger().info(f'Lift : {self.trans_lift_position},Trans X: {self.trans_x}, Trans Y: {self.trans_y}, Trans Z: {self.trans_z}')
        

        #TODO Calibration Offset
        transy_offset = 40.0

        self.rbutils.call_elevation_command(self.trans_lift_position)
        self.rbutils.call_set_relative_robot_pose(dy=self.trans_y-transy_offset)
        self.get_logger().info(f'Calibration Movement Complete')

        self.rbutils.call_set_relative_robot_pose(dx=self.trans_x-RobotParam.tool_length)

        self.rbutils.call_gripper_command(RobotParam.grip_close)

        time.sleep(2) # Gripper Grasp wait

        self.rbutils.call_set_relative_robot_pose(dz=50.0)
        self.rbutils.call_set_relative_robot_pose(dx=-(self.trans_x-RobotParam.tool_length)+50.0)
        self.rbutils.call_set_servo_angle(RobotParam.arm_ready)
        #self.rbutils.call_elevation_command(RobotParam.elev_home)



        rev_data.update({'isdone': True})

        response.ret = dict2str(rev_data)
        return response

    def skill_place_callback(self,request,response):
        self.get_logger().info(f"Place Skill Processing")
        rev_data = str2dict(request.req)
        pose=self.dict2placepose(rev_data)

        mid_x = (pose[0]+pose[2])/2
        mid_y = (pose[1]+pose[3])/2
        obj_depth = pose[4]
        cur_lift_position=self.rbutils.get_elev_pose()*1000
        cur_arm_robot_pose=self.rbutils.get_robot_pose()['pose']
        cur_head_pose=self.rbutils.get_head_pose()
        cur_angle_ry=copy.deepcopy(cur_head_pose[1])
        cur_angle_rz=copy.deepcopy(cur_head_pose[0])
        
        self.trans_lift_position,self.trans_armrobot_XYZ=self.femto_calib.convert_femto_to_arm(mid_x,mid_y,obj_depth,#[x,y,z] mm 
                                                                                RobotParam.elev_home*1000, # scalar (meter)
                                                                                cur_arm_robot_pose, # x y z r p y
                                                                                cur_angle_ry,# scalar
                                                                                cur_angle_rz# scalar
                                                                                )
        
        self.trans_x,self.trans_y,self.trans_z=self.trans_armrobot_XYZ
        self.get_logger().info(f'Lift : {self.trans_lift_position},Trans X: {self.trans_x}, Trans Y: {self.trans_y}, Trans Z: {self.trans_z}')
        elev_offset = (self.obj_height/1000.0)
        
        self.rbutils.call_elevation_command(self.trans_lift_position+elev_offset+(RobotParam.tool_radius/1000))
        
        #TODO Calibration Offset
        transy_offset = 40.0
        self.rbutils.call_set_relative_robot_pose(dy=self.trans_y-transy_offset)
        self.rbutils.call_set_relative_robot_pose(dx=self.trans_x-RobotParam.tool_length)
        self.rbutils.call_gripper_command(RobotParam.grip_open)
        time.sleep(2)

        self.rbutils.call_set_relative_robot_pose(dx=-(self.trans_x-RobotParam.tool_length)+50.0)
        self.rbutils.call_set_servo_angle(RobotParam.arm_ready)
        self.rbutils.call_elevation_command(RobotParam.elev_home)

        rev_data.update({'isdone': True})

        response.ret = dict2str(rev_data)
        return response

    def rb_init(self):
        # ROS Spin wait
        time.sleep(1)
        self.rbutils.call_gripper_command(RobotParam.grip_open)
        self.rbutils.call_head_command([0.0, 20.0])
        #로봇 전원 On
        self.rbutils.call_motion_enable(8, 1)
        #로봇팔 포지션 제어모드
        self.rbutils.call_set_mode(0)
        #로봇 스테이트 셋
        self.rbutils.call_set_state(0)
        #조인트 기반 홈자세 이동
        self.rbutils.call_set_servo_angle(RobotParam.arm_home)
        self.rbutils.call_elevation_command(RobotParam.elev_home)
        self.rbutils.call_head_command([0.0, -30.0])



def main(args=None):
    rclpy.init(args=args)
    master_node = KcareMaster()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(master_node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    executor.create_task(master_node.rb_init)
    try:
        master_node.get_logger().info("✅ Master Server is running...")
        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

