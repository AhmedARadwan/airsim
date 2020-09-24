import os
import subprocess
import time
import airsim
from std_msgs.msg import Header
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pcl2
from yonoarc_utils.image import to_ndarray, from_ndarray
from yonoarc_utils.header import set_timestamp, get_timestamp
from std_msgs.msg import Float32


class AirSim:
    def download_run_airsim(self):
        # Environment variables to run AirSim headless
        os.environ["SDL_VIDEODRIVER_VALUE"] = "offscreen"
        os.environ["SDL_HINT_CUDA_DEVICE"] = "0"
        # AirSim Binaries List
        list_env = [
            "Blocks",
            "Africa",
            "Building_99",
            "Zhangjiajie",
            "Neighborhood",
            "SoccerField",
            "LandscapeMountains",
            "TrapCam",
        ]
        # Let's get the selected environment from our block properities
        self.enviroment = list_env[self.get_property("env")]
        # get the settings file path from block properities
        self.settings_path = self.get_property("settings")
        # Download and run the environment
        if self.enviroment == "TrapCam":
            download_path = "https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/TrapCam.zip.001"
            self.alert("Downloading the Environment.....", "INFO")
            subprocess.call(
                "cd /home/airsim && wget {}".format(download_path), shell=True
            )
            download_path = "https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/TrapCam.zip.002"
            self.alert("Downloading the Environment.....", "INFO")
            subprocess.call(
                "cd /home/airsim && wget {}".format(download_path), shell=True
            )
            subprocess.call(
                "cd /home/airsim && cat TrapCam.zip.00* > TrapCam.zip && unzip TrapCam.zip",
                shell=True,
            )
        else:
            download_path = "https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/{}.zip".format(
                self.enviroment
            )
            self.alert("Downloading the Environment.....", "INFO")
            subprocess.call(
                "cd /home/airsim && wget {}".format(download_path), shell=True
            )
            self.alert("Unzip the Downloaded File.....", "INFO")
            subprocess.call(
                "runuser -l airsim -c'cd /home/airsim/ && unzip {}.zip'".format(
                    self.enviroment
                ),
                shell=True,
            )
        self.alert("Copying settings..", "INFO")
        subprocess.call(
            "runuser -l airsim -c 'mkdir -p /home/airsim/Documents/AirSim/ && cp {} ~/Documents/AirSim/settings.json'".format(
                self.settings_path
            ),
            shell=True,
        )
        self.alert("Run AirSim Env......", "INFO")
        if self.enviroment == "Africa":
            self.pro = subprocess.Popen(
                "runuser -l airsim -c 'sh /home/airsim/Africa_001.sh -opengl'",
                shell=True,
            )
        elif self.enviroment == "Zhangjiajie":
            self.pro = subprocess.Popen(
                "runuser -l airsim -c 'sh /home/airsim/Zhangjiajie.sh -opengl'",
                shell=True,
            )
        elif self.enviroment == "Neighborhood":
            self.pro = subprocess.Popen(
                "runuser -l airsim -c 'sh /home/airsim/AirSimNH.sh -opengl'", shell=True
            )
        elif self.enviroment == "TrapCam":
            self.pro = subprocess.Popen(
                "runuser -l airsim -c 'sh /home/airsim/TrapCam.sh -opengl'", shell=True
            )
        else:
            self.pro = subprocess.Popen(
                "runuser -l airsim -c 'sh /home/airsim/{}/{}.sh -opengl'".format(
                    self.enviroment, self.enviroment
                ),
                shell=True,
            )

    def on_start(self):
        self.alert("creating airsim user and running ssh.....", "INFO")
        # username and password for ssh
        os.environ["USERNAME"] = "airsim"
        os.environ["PASSWORD"] = "airsim1234"
        # Run the ssh-server
        os.system(
            "(mkdir /home/$USERNAME && useradd $USERNAME && echo $USERNAME:$PASSWORD | chpasswd && usermod -aG sudo $USERNAME && chown airsim:airsim /home/airsim && usermod -d /home/airsim airsim && mkdir /var/run/sshd && /usr/sbin/sshd) &"
        )
        # Download and Run AirSim Binary
        self.download_run_airsim()
        # check if the actor is a car or a multirotor
        self.car = self.get_property("car")
        # Cech AirSim client Connection
        while True:
            try:
                if self.car:
                    self.alert("Connecting Car Client", "INFO")
                    self.client = airsim.CarClient()
                    self.client.confirmConnection()
                    self.client.enableApiControl(True)
                    self.car_controls = airsim.CarControls()
                    break
                else:
                    self.alert("Connecting Multirotor Client", "INFO")
                    self.client = airsim.MultirotorClient()
                    self.client.confirmConnection()
                    self.client.enableApiControl(True)
                    self.client.armDisarm(True)
                    # Async methods returns Future. Call join() to wait for task to complete.
                    self.client.takeoffAsync()
                    self.client.moveToPositionAsync(-10, 10, -10, 5)
                    break
            except Exception as e:
                self.alert("Client Not Connected Yet, trying after 1 s", "WARN")
                print(e)
                time.sleep(1)
        # Define Some Variables
        self.period = 1 / self.get_property("frame_rate")  # 1/frame rate
        self.car_moved = False  # Check if the Moving Command is Executed
        self.stop_at = 0  # when to stop
        self.command_list = []  # Command List
        self.move_command = False

    def parse_lidarData(self, data):
        # function to create PointCloud2 from Lidar Data
        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype("f4"))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))
        header = Header()
        header.frame_id = "lidar"
        pcl = pcl2.create_cloud_xyz32(header, points)
        return pcl

    def run(self):
        # Block Main Loop
        last_time = time.time()  # start time of the loop
        while True:
            # timer to limit the frame rate
            if time.time() - last_time < self.period:
                try:
                    time.sleep(self.period - (time.time() - last_time))
                except:
                    pass
            # check for the time limit for the executed command
            if self.move_command:
                if self.car:
                    self.car_controls = airsim.CarControls()
                    self.car_controls.throttle = self.command_list[0]
                    self.car_controls.steering = self.command_list[1]
                    self.client.setCarControls(self.car_controls)
                    self.alert("Moving Forward ", "INFO")
                    self.car_moved = True
                    self.stop_at = time.time() + self.command_list[2]
                else:
                    self.alert("Moving To {}".format(self.command_list), "INFO")
                    self.client.moveToPositionAsync(
                        self.command_list[0],
                        self.command_list[1],
                        self.command_list[2],
                        self.command_list[3],
                    )
                self.move_command = False
            if time.time() >= self.stop_at and self.car_moved:
                self.car_controls.throttle = 0
                self.client.setCarControls(self.car_controls)
                self.car_moved = False
                self.alert("Stopping the Car ", "INFO")
            # AirSim Default Cameras names
            cameras = [
                "front_center",
                "front_right",
                "front_left",
                "fpv",
                "back_center",
            ]
            # Reading front_center camera
            camera_id = 0
            responses = self.client.simGetImages(
                [airsim.ImageRequest(0, airsim.ImageType.Scene, False, False)]
            )
            # print('Retrieved images: %d', len(responses))
            for response in responses:
                # get numpy array
                img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
                # reshape array to 4 channel image array H X W X 4
                img_rgb = img1d.reshape(response.height, response.width, 3)
                # Create Header for the ROS message
                header = Header()
                set_timestamp(header, time.time())
                header.frame_id = cameras[camera_id]
                # Convert Numpy array to ROS message using yonoarc_utils.image
                img_msg = from_ndarray(img_rgb, header)
                # Publish this message on Block's output port
                self.publish("cam_{}".format(camera_id), img_msg)
                camera_id += 1
            # Read Semantic Segmentation Image from Camera 0
            # Create ROS message and publish it like the previous steps
            seg_responses = self.client.simGetImages(
                [airsim.ImageRequest(0, airsim.ImageType.Segmentation, False, False)]
            )
            img1d = np.fromstring(seg_responses[0].image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(seg_responses[0].height, seg_responses[0].width, 3)
            header = Header()
            header.frame_id = "front_center"
            img_msg = from_ndarray(img_rgb, header)
            self.publish("sem_segm", img_msg)
            # Read Lidar Data and convert it to PointCloud2 ROS message
            lidarData = self.client.getLidarData()
            if len(lidarData.point_cloud) < 3:
                self.alert("\tNo points received from Lidar data", "WARN")
            else:
                points = self.parse_lidarData(lidarData)
                self.publish("lidar", points)

            try:
                states = self.client.getCarState()
                float_msg = Float32()
                float_msg.data = states.speed
                self.publish("speed", float_msg)
            except Exception as e:
                print(e)
                pass
            # Update Timer
            last_time = time.time()

    def on_button_clicked(self, button_key):
        # Handle button Clicked Events.
        if button_key == "move_to":
            # Parse the Move Command
            pos_string = self.get_property("pos").split(",")
            self.command_list = []
            for st in pos_string:
                self.command_list.append(float(st))
            self.move_command = True

    def on_new_messages(self, messages):
        if "throttle" in messages:
            if messages["throttle"] == self.car_controls.throttle:
                pass
            else:
                self.car_controls.throttle = float(messages["throttle"].data)
        if "steering" in messages:
            if messages["steering"] == self.car_controls.steering:
                pass
            else:
                self.car_controls.steering = float(messages["steering"].data)

        if "brake" in messages:
            if messages["brake"].data:
                self.car_controls.throttle = 0
                self.car_controls.brake = 1
            else:
                self.car_controls.brake = 0
        self.client.setCarControls(self.car_controls)
