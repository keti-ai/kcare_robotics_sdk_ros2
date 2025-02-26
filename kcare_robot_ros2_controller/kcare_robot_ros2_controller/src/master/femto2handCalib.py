import numpy as np
import cv2
import copy
from kcare_robot_ros2_controller_msgs.msg import HeadCommand, HeadState, LMCommand, LMState
import time
class calibration():
    def __init__(self):
        print("calibration")
        self.base2femto_robot=np.eye(4)
        self.base2femto_robot[0,3]=53.0
        self.base2femto_robot[2,3]=1139.5

        self.base2LMbase_robot=np.eye(4)
        self.base2LMbase_robot[0,3]=240.0   
        self.base2LMbase_robot[2,3]=190

    def rotate_x(self,theta, point):
        # Convert theta to radians
        theta_rad = np.deg2rad(theta)
        
        # Rotation matrix for X-axis
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(theta_rad), -np.sin(theta_rad)],
            [0, np.sin(theta_rad), np.cos(theta_rad)]
        ])
        
        # Apply the rotation
        rotated_point = np.matmul(rotation_matrix, point)
        
        return rotated_point
    def rotate_y(self,theta, point):
        # Convert theta to radians
        theta_rad = np.deg2rad(theta)
        
        # Rotation matrix for X-axis
        rotation_matrix = np.array([
            [np.cos(theta_rad), 0, np.sin(theta_rad)],
            [0, 1, 0],
            [-np.sin(theta_rad), 0, np.cos(theta_rad)]
        ])
        
        # Apply the rotation
        rotated_point = np.matmul(rotation_matrix, point)
        
        return rotated_point
    def convert_sensor_to_link(self,point,rotate_y,rotate_z):
    # print(_angle_set)
        T_1=[32,-(20.13+55),+36.9+24]
        T_2=[0,0,28]
        rotate_point_x = self.rotate_x(rotate_y,point) + T_1
        rotate_point_y = self.rotate_y(-rotate_z,rotate_point_x) + T_2
        return rotate_point_y                
#    def convert_femto_to_arm(self,target_point,cur_lift_position_mm,cur_robot,current_ry,current_rz):
    def convert_femto_to_arm(self,X_2d,Y_2d,depth,cur_lift_position_mm,cur_robot,current_ry,current_rz):
        
        # fx= 1122.0177001953125
        # ppx= 956.3211669921875
        # fy=1121.2445068359375
        # ppy=547.1177368164062
        fx= 748.0117797851562
        ppx= 637.5474243164062
        fy=747.496337890625
        ppy=364.7451477050781
        # fx=504.58392333984375
        # ppx=320.40264892578125
        # fy=504.67828369140625
        # ppy=331.8777770996094

        target_point=[(X_2d-ppx)/fx*depth,(Y_2d-ppy)/fy*depth,depth] 
        # mm단위로 변경
        if cur_lift_position_mm<1:
            cur_lift_position=cur_lift_position_mm*1000
        else:
            cur_lift_position=cur_lift_position_mm
        #femto 카메라에서 링크까지 변환
        link_point=self.convert_sensor_to_link(target_point,current_ry,current_rz)
        point_x,point_y,point_z=link_point
        
        #링크에서 카메라좌표계에서 베이스로봇좌표계로 변환
        robot_x=point_z
        robot_y=-point_x
        robot_z=-point_y             
        base_femto_point=np.dot((self.base2femto_robot),[robot_x,robot_y,robot_z,1])
        
        #현재 리프트 위치에서 base2robot 관계 계산
        current_base2LMbase_robot=copy.deepcopy(self.base2LMbase_robot)        
        current_base2LMbase_robot[2,3]=cur_lift_position
        base_cur_robot=np.matmul(current_base2LMbase_robot,cur_robot[0:3].tolist()+[1])
 
        #리스트 이동 거리 계산       
        trans_lift_position=(base_femto_point[2]-base_cur_robot[2])
        print(f"robot point :{robot_x,robot_y,robot_z}")
        print(f"base_femto_point :{base_femto_point}")
        print(f"base_cur_robot :{base_cur_robot}")

 
        #리프트 이동 후, arm로봇의 이동거리 계산 
        target_base2LMbase_robot=copy.deepcopy(current_base2LMbase_robot)
        if trans_lift_position<0:
            trans_lift_position=0
            target_base2LMbase_robot[2,3]=self.base2LMbase_robot[2,3]
        else:
            target_base2LMbase_robot[2,3]+=trans_lift_position
        target_cur_robot=np.matmul(target_base2LMbase_robot,cur_robot[0:3].tolist()+[1])
                
        trans_x,trans_y,trans_z,trans_t=base_femto_point-target_cur_robot
        print(f"trans_x:{trans_x},trans_y:{trans_y},trans_z:{trans_z}")       
        
        # meter 단위로 변경
        trans_lift_position_meter=(cur_lift_position+trans_lift_position)/1000 
        return trans_lift_position_meter,[trans_x,trans_y,trans_z]