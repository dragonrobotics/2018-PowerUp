from networktables import NetworkTables
import cscore
import time
import numpy as np
import math
from collections import deque
from rplidar import RPLidar

# tuples are (camera_name, camera_device_id)
cameras = [
    ('Camera 1', 0),
    #('Camera 2', 1),
]

screen_radius = 200
screen_scale = screen_radius / 8000

chassis_width_px = int(screen_scale * (27 * 25.4))
chassis_length_px = int(screen_scale * (24 * 25.4))

def draw_scan(out_array, scan_data):
    for angle, dist in scan_data:
        rel_dist = screen_radius * (dist / 8000)
        angle_rad = math.radians(angle)

        pos = [
            int(screen_radius) + math.trunc(rel_dist * math.cos(angle_rad)),
            int(screen_radius) + math.trunc(rel_dist * math.sin(angle_rad))
        ]

        if pos[0] < 0 or pos[0] >= 400:
            pos[0] = 0

        if pos[1] < 0 or pos[1] >= 400:
            pos[1] = 0

        out_array[pos[0]][pos[1]][2] = 255
        out_array[pos[0]][pos[1]][1] = 0
        out_array[pos[0]][pos[1]][0] = 0

def main():
    f = open('vis-log.txt', 'w')

    cs_instance = cscore.CameraServer.getInstance()
    cs_instance.enableLogging()

    lidar = RPLidar(port='/dev/ttyUSB0')

    print("Resetting LIDAR...", file=f)
    f.flush()

    lidar.stop_scan()

    model, fw, hw, serial_no = lidar.get_device_info()
    health_status, err_code = lidar.get_device_health()

    print(
        '''
        ===

        Opened LIDAR on serial port /dev/ttyUSB0
        Model ID: {}
        Firmware: {}
        Hardware: {}
        Serial Number: {}
        Device Health Status: {} (Error Code: 0x{:X})

        ===
        '''.format(
            model, fw, hw, serial_no.hex(),
            health_status, err_code
        ),
        file=f
    )

    print("Setting board PWM...", file=f)
    f.flush()
    lidar.dev.dtr = False
    lidar.acc_board_set_pwm(1023)

    lidar.start_scan()

    table = NetworkTables.getTable("Preferences")

    res_w = int(table.getNumber('Camera Res Width', 320))
    res_h = int(table.getNumber('Camera Res Height', 200))
    fps = int(table.getNumber('Camera FPS', 30))

    camera_objects = []

    for cam_idx, cam_config in enumerate(cameras):
        name, dev_id = cam_config

        camera_obj = cscore.UsbCamera(name=name, dev=dev_id)
        camera_obj.setResolution(res_w, res_h)
        camera_obj.setFPS(fps)

        camera_objects.append(camera_obj)
        #camera_chooser.addDefault(name, cam_idx)

    cam_server = cs_instance.addServer(name='camera_server')
    current_selected = int(table.getNumber('Selected Camera', 0))

    if current_selected >= len(camera_objects):
        current_selected = 0

    outputStream = cs_instance.putVideo("LIDAR", screen_radius*2, screen_radius*2)

    cam_server.setSource(camera_objects[current_selected])

    lidar_scope = np.zeros(
        (screen_radius*2, screen_radius*2, 3),
        dtype=np.uint8
    )

    last_scans = deque([], 15)
    cur_scan = []

    while True:
        polled_samples = lidar.poll_scan_samples()
        if len(polled_samples) > 0:

            for angle, dist, new_scan in polled_samples:
                if new_scan:
                    last_scans.append(cur_scan)
                    cur_scan = []

                cur_scan.append((angle-90, dist))

        lidar_scope.fill(255)

        for scan in last_scans:
            draw_scan(lidar_scope, scan)
        draw_scan(lidar_scope, cur_scan)

        for x in range(-1 * int(chassis_width_px / 2), int(chassis_width_px/2)):
            x_offset = screen_radius + int(5.5 * 25.4 * screen_scale)
            for y in range(chassis_length_px):
                lidar_scope[x_offset+x][screen_radius+y][1] = 0
                lidar_scope[x_offset+x][screen_radius+y][2] = 0

        outputStream.putFrame(lidar_scope)

        selected_camera = int(table.getNumber('Selected Camera', 0))

        if (
            selected_camera < len(camera_objects)
            and selected_camera != current_selected
        ):
            res_w = int(table.getNumber('Camera Res Width', 320))
            res_h = int(table.getNumber('Camera Res Height', 200))
            fps = int(table.getNumber('Camera FPS', 30))

            camera_obj = camera_objects[selected_camera]

            camera_obj.setResolution(res_w, res_h)
            camera_obj.setFPS(fps)

            cam_server.setSource(camera_obj)
            current_selected = selected_camera

        time.sleep(0.02)
