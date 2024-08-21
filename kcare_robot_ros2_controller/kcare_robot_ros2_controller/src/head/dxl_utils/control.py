import time
from threading import Thread


from dxl_utils.dxl_prop import Dynamixel


def deg2encoder(deg):
    encoder = deg * (4096 / 360)
    return int(encoder)

def encoder2deg(encoder):
    deg = encoder * (360 / 4096)
    return deg

class DynamixelControl:
    def __init__(self):
        # self.dxl_config = LoadConfig("./config/dxl_config.yaml").info
        self.dxl_config = ''

        self.dxl = None
        self.hmd = None

        self.dxl_pose = []
        self.hmd_pose = []

        self.dxl_started = False
        self.hmd_started = False

        self.cal_thread = None
        self.dxl_thread = None
        self.hmd_thread = None

        self.state = "DISABLE"

        self.init()

        self.run()

    def init(self):
        self.dxl = Dynamixel(self.dxl_config)
        # self.dxl.init()

        self.hmd = HMDPose(self.dxl_config)
        self.hmd.init()

    def run(self):
        self.hmd_thread = Thread(target=self.read_hmd)
        self.hmd_thread.start()

        self.cal_thread = Thread(target=self.pose_cal)
        self.cal_thread.start()

        self.dxl_thread = Thread(target=self.ctrl_dxl)
        self.dxl_thread.start()

    def close(self):
        self.dxl.close()
        self.dxl = None

    def pose_cal(self):
        pass

    def ctrl_dxl(self):
        homepose = [self.dxl_config[f'id{i}']['HOME'] for i in self.dxl_config['dxl_ids']]
        dxl_pose = []
        tar_pose = [0, 0]

        init_dxl_pose = []
        init_hmd_pose = []

        offset = [99999999, 99999999]

        home_margin = 1

        v_delta = GetDelta()
        h_delta = GetDelta()

        while True:
            if self.state == "DISABLE":
                if self.dxl.state != "DISABLE":
                    self.dxl.write(1, 1300)
                    self.dxl.write(2, 1935)

                    time.sleep(1)
                    [self.dxl.disable(i) for i in self.dxl_config['dxl_ids']]

            else:
                if self.dxl.state != "ENABLE":
                    self.dxl.init()

                    v_delta = GetDelta()
                    h_delta = GetDelta()

                if self.state != "STOP":
                    dxl_pose = [self.dxl.pose(i) for i in self.dxl_config['dxl_ids']]

                    init_hmd_pose = []

                if self.state == "HOME":

                    if dxl_pose[0] not in range(homepose[0]-home_margin, homepose[0]+home_margin):
                        self.dxl.write(1, homepose[0])

                    if dxl_pose[1] not in range(homepose[1]-home_margin, homepose[1]+home_margin):
                        self.dxl.write(2, homepose[1])

                    if dxl_pose[0] in range(homepose[0]-home_margin, homepose[0]+home_margin) and \
                       dxl_pose[1] in range(homepose[1]-home_margin, homepose[1]+home_margin):

                        self.state = "ENABLE"

                    offset = [99999999, 99999999]
                    v_delta.pose_list.clear()
                    h_delta.pose_list.clear()

                if self.state == "TEST":
                    self.dxl.write(1, dxl_pose[0] - 300)
                    self.dxl.write(2, dxl_pose[1] + 300)

                    time.sleep(0.5)

                    self.state = "HOME"

                if self.state == "SYNC":
                    if len(self.hmd_pose) == 2:
                        if self.hmd_pose[0] != 0 and self.hmd_pose[1] != 0:

                            if not init_hmd_pose:
                                init_hmd_pose = self.hmd_pose

                            if offset[0] == 99999999:
                                offset[0] = self.hmd_pose[0] - encoder2deg(dxl_pose[0])

                            if offset[1] == 99999999:
                                offset[1] = self.hmd_pose[1] - encoder2deg(dxl_pose[1])

                            if offset != [99999999, 99999999]:
                                v_delta.update(self.hmd_pose[0])
                                h_delta.update(self.hmd_pose[1])

                                tar_pose[0] = v_delta.pose_list[-1] - offset[0]
                                tar_pose[1] = h_delta.pose_list[-1] - offset[1]

                                tar_pose[0] = deg2encoder(tar_pose[0])
                                tar_pose[1] = deg2encoder(tar_pose[1])

                                if tar_pose[0] >= self.dxl_config['id1']['MAX_POSITION']:
                                    tar_pose[0] = self.dxl_config['id1']['MAX_POSITION']
                                elif tar_pose[0] <= self.dxl_config['id1']['MIN_POSITION']:
                                    tar_pose[0] = self.dxl_config['id1']['MIN_POSITION']

                                if tar_pose[1] >= self.dxl_config['id2']['MAX_POSITION']:
                                    tar_pose[1] = self.dxl_config['id2']['MAX_POSITION']
                                elif tar_pose[1] <= self.dxl_config['id2']['MIN_POSITION']:
                                    tar_pose[1] = self.dxl_config['id2']['MIN_POSITION']

                                self.dxl.write(1, tar_pose[0])
                                self.dxl.write(2, tar_pose[1])

                if self.state == "STOP":
                    offset = [99999999, 99999999]

                    v_delta.pose_list.clear()
                    h_delta.pose_list.clear()

            time.sleep(0.002)

    def read_hmd(self):
        while True:
            conn, addr = self.hmd.sock.accept()

            while True:
                try:
                    self.hmd.read(conn)

                    if self.hmd.pose[0] != 0 and self.hmd.pose[1] != 0:
                        break
                except:
                    break
                # time.sleep(0.002)

            while True:
                try:
                    self.hmd.read(conn)
                    self.hmd_pose = self.hmd.pose
                except:
                    break

                # time.sleep(0.002)

            conn.close()

            time.sleep(0.002)

class GetDelta:
    def __init__(self):
        self.pose_list = []
        self.reverse = False
        self.last = 0

    def update(self, value):
        if len(self.pose_list) < 2:
            self.pose_list.append(value)
        else:
            if abs(self.pose_list[-1] - value) > 330:
                self.reverse = True

            if self.reverse:
                if abs(self.pose_list[-1] - value) < 40:
                    self.reverse = False

            else:
                del self.pose_list[0]
                self.pose_list.append(value)

                self.last = self.pose_list[-1]
