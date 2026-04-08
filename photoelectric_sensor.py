from Robot import RPC
import time

class PhotoSensorSDK:
    def __init__(self):
        self.robot = RPC("192.168.57.4")

    # IO口为0
    def get_cloth_status(self):
        err, state = self.robot.GetDI(4, 0)
        return state
    
    
if __name__ == "__main__":
     sensor = PhotoSensorSDK()
     while True:
        s0 = sensor.get_cloth_status()
        print(f" {s0} ")
        time.sleep(0.2)