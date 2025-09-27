#!/usr/bin/env python3
import rospy
import curses
from ros_ecat_msgs.msg import VescEcatRosCommands, DJIEcatRosCommands, DJIEcatRosMsg, VescEcatRosMsg, InitState 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from vision_msgs.msg import BoundingBox2DArray



# 用于存储各个话题的数据
data_dict = {
    'vesc_cmds': None,   # VescEcatRosCommands
    'ecat_cmd': None,    # DJIEcatRosCommands
    'ecat_msg': None,    # DJIEcatRosMsg
    'vesc_msg': None,     # VescEcatRosMsg
    'odom': None, # odom
    'mahony/euler': None, # imu data
    'yolo/detections': None, # camera data
    'init_state': None # init state
}

# 用于计数，监测是否有话题没有被发布
data_count = {
    'vesc_cmds': 0,   # VescEcatRosCommands
    'ecat_cmd': 0,    # DJIEcatRosCommands
    'ecat_msg': 0,    # DJIEcatRosMsg
    'vesc_msg': 0,     # VescEcatRosMsg
    'odom': 0, # odom
    'mahony/euler': 0, # imu data
    'yolo/detections': 0, # camera data
    'init_state': 0 # init state
}

def vesc_commands_callback(msg):
    data_dict['vesc_cmds'] = msg
    data_count['vesc_cmds'] += 1

def format_vesc_cmd_table(msg):
    headers = ["Field", "Value"]
    col_widths = [25, 50]
    header_line = ""
    for h, w in zip(headers, col_widths):
        header_line += "{:<{w}} ".format(h, w=w)
    table_lines = [header_line, "-" * (sum(col_widths) + len(col_widths))]
    table_lines.append("{:<{w0}} {:<{w1}}".format("vesc_can0_motor_commands", str(msg.vesc_can0_motor_commands),
                                                   w0=col_widths[0], w1=col_widths[1]))
    table_lines.append("{:<{w0}} {:<{w1}}".format("vesc_can1_motor_commands", str(msg.vesc_can1_motor_commands),
                                                   w0=col_widths[0], w1=col_widths[1]))
    return table_lines

def dje_commands_callback(msg):
    data_dict['ecat_cmd'] = msg
    data_count['ecat_cmd'] += 1

def format_dje_cmd_table(msg):
    headers = ["Field", "Value"]
    col_widths = [20, 40]
    header_line = ""
    for h, w in zip(headers, col_widths):
        header_line += "{:<{w}} ".format(h, w=w)
    table_lines = [header_line, "-" * (sum(col_widths) + len(col_widths))]
    table_lines.append("{:<{w0}} {:<{w1}}".format("output_io", str(msg.output_io),
                                                   w0=col_widths[0], w1=col_widths[1]))
    table_lines.append("{:<{w0}} {:<{w1}}".format("can0_motor_commands", str(msg.can0_motor_commands),
                                                   w0=col_widths[0], w1=col_widths[1]))
    table_lines.append("{:<{w0}} {:<{w1}}".format("can1_motor_commands", str(msg.can1_motor_commands),
                                                   w0=col_widths[0], w1=col_widths[1]))
    return table_lines

def dje_msg_callback(msg):
    data_dict['ecat_msg'] = msg
    data_count['ecat_msg'] += 1

def format_dje_msg_table(msg):
    headers = ["Index", "Name", "Input_IO", "Output_IO", "Position", "Velocity", "Torque", "Temp"]
    col_widths = [10, 30, 10, 10, 10, 10, 10, 8]
    header_line = ""
    for h, w in zip(headers, col_widths):
        header_line += "{:<{w}} ".format(h, w=w)
    table_lines = [header_line, "-" * len(header_line)]
    n = len(msg.names)
    for i in range(n):
        row = "{:<10} ".format(i)
        row += "{:<30} ".format(msg.names[i] if i < len(msg.names) else "")
        row += "{:<10} ".format("T" if (i < len(msg.input_io) and msg.input_io[i]) else "F")
        row += "{:<10} ".format("T" if (i < len(msg.output_io) and msg.output_io[i]) else "F")
        row += "{:<10} ".format("{:.5f}".format(msg.position[i]) if i < len(msg.position) else "")
        row += "{:<10} ".format("{:.5f}".format(msg.velocity[i]) if i < len(msg.velocity) else "")
        row += "{:<10} ".format("{:.5f}".format(msg.torque[i]) if i < len(msg.torque) else "")
        row += "{:<8} ".format("{:.2f}".format(msg.temperature[i]) if i < len(msg.temperature) else "")
        table_lines.append(row)
    return table_lines

def vesc_msg_callback(msg):
    data_dict['vesc_msg'] = msg
    data_count['vesc_msg'] += 1

def format_vesc_msg_table(msg):
    headers = ["Index", "Name", "Velocity", "Torque"]
    col_widths = [10, 30, 15, 15]
    header_line = ""
    for h, w in zip(headers, col_widths):
        header_line += "{:<{w}} ".format(h, w=w)
    table_lines = [header_line, "-" * len(header_line)]
    n = len(msg.names)
    for i in range(n):
        row = "{:<10} ".format(i)
        row += "{:<30} ".format(msg.names[i] if i < len(msg.names) else "")
        row += "{:<15} ".format("{:.2f}".format(msg.velocity[i]) if (msg.velocity and i < len(msg.velocity)) else "")
        row += "{:<15} ".format("{:.2f}".format(msg.torque[i]) if (msg.torque and i < len(msg.torque)) else "")
        table_lines.append(row)
    return table_lines

def odom_callback(msg):
    data_dict['odom'] = msg
    data_count['odom'] += 1

def format_odom_table(msg):
    headers = ["Field", "Value"]
    col_widths = [20, 50]
    header_line  = ""
    for h, w in zip(headers, col_widths):
        header_line += "{:<{w}} ".format(h, w=w)
    table_lines = [header_line, "-" * (sum(col_widths) + len(col_widths))]
    # 提取位姿
    pose = msg.pose.pose
    table_lines.append("{:<{w0}} {:<{w1}}".format("Position", 
        "x:{:.2f}, y:{:.2f}, z:{:.2f}".format(pose.position.x, pose.position.y, pose.position.z),
        w0=col_widths[0], w1=col_widths[1]))
    return table_lines

def imu_data_callback(msg):
    data_dict['mahony/euler'] = msg
    data_count['mahony/euler'] += 1

def format_imudata_table(msg):
    headers = ["Field", "Value"]
    col_widths = [10, 50]
    header_line = "".join("{:<{w}} ".format(h, w=w) for h, w in zip(headers, col_widths))
    # 不再分隔为多行，而是合并到一行
    data_line = "x:{:.2f}, y:{:.2f}, z:{:.2f}".format(msg.vector.x, msg.vector.y, msg.vector.z)
    table_lines = [header_line, "-" * (sum(col_widths)+len(col_widths)), "{:<10} {:<50}".format("vector", data_line)]
    return table_lines   

def camera_data_callback(msg):
    data_dict["yolo/detections"] =msg
    data_count["yolo/detections"] += 1

def format_cameradata_table(msg):
    headers = ["Field", "Value"]
    col_widths = [10,50]
    header_line = "".join("{:<{w}} ".format(h, w=w) for h, w in zip(headers, col_widths))
    # 不再分隔为多行，而是合并到一行
    data_line = "center_x:{:.2f}, center_y:{:.2f}".format(msg.boxes[0].center.x, msg.boxes[0].center.y)
    table_lines = [header_line, "-" * (sum(col_widths)+len(col_widths)), "{:<10} {:<50}".format("vector", data_line)]
    return table_lines   

def init_state_callback(msg):
    data_dict["init_state"] =msg
    data_count["init_state"] += 1

# row += "{:<10} ".format("T" if (i < len(msg.input_io) and msg.input_io[i]) else "F")

def format_initstate_table(msg):
    headers = ["Field", "Value"]
    col_widths = [20, 40]
    header_line = ""
    for h, w in zip(headers, col_widths):
        header_line += "{:<{w}} ".format(h, w=w)
    table_lines = [header_line, "-" * (sum(col_widths) + len(col_widths))]
    table_lines.append("{:<{w0}} {:<{w1}}".format("init_camera_flag", str(msg.init_camera_flag), w0=col_widths[0], w1=col_widths[1]))
    table_lines.append("{:<{w0}} {:<{w1}}".format("init_basket_flag", str(msg.init_basket_flag), w0=col_widths[0], w1=col_widths[1]))
    table_lines.append("{:<{w0}} {:<{w1}}".format("camera_center", "{:.2f}".format(msg.camera_center), w0=col_widths[0], w1=col_widths[1]))
    table_lines.append("{:<{w0}} {:<{w1}}".format("basket_x", "{:.2f}".format(msg.basket_x), w0=col_widths[0], w1=col_widths[1]))
    table_lines.append("{:<{w0}} {:<{w1}}".format("basket_y", "{:.2f}".format(msg.basket_y), w0=col_widths[0], w1=col_widths[1]))
    return table_lines

def safe_addstr(stdscr, y, x, text):
    max_y, _ = stdscr.getmaxyx()
    if y < max_y:
        try:
            stdscr.addstr(y, x, text)
        except curses.error:
            pass

def main(stdscr):
    rospy.init_node("ros_topic_monitor", anonymous=False)
    rospy.Subscriber('/vesc_cmds', VescEcatRosCommands, vesc_commands_callback)
    rospy.Subscriber('/ecat_cmd', DJIEcatRosCommands, dje_commands_callback)
    rospy.Subscriber('/ecat_msg', DJIEcatRosMsg, dje_msg_callback)
    rospy.Subscriber('/vesc_msg', VescEcatRosMsg, vesc_msg_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback) 
    rospy.Subscriber('/mahony/euler', Vector3Stamped, imu_data_callback)
    rospy.Subscriber('/yolo/detections' , BoundingBox2DArray, camera_data_callback)
    rospy.Subscriber('/init_state' , InitState, init_state_callback)


    stdscr.nodelay(1)
    curses.noecho()
    # 强制把 curses 画布扩到 60 行、150 列
    curses.resizeterm(100, 180)
    stdscr.resize(100, 180)
    max_y, max_x = stdscr.getmaxyx()
    blank_line = " " * max_x

    # 使用 current_row 动态跟踪当前输出行号
    while not rospy.is_shutdown():
        stdscr.erase()
        safe_addstr(stdscr, 0, 0, "ROS Topic Dynamic Monitor")
        current_row = 2

        # VescEcatRosCommands 表格显示
        # safe_addstr(
        #             stdscr,
        #             current_row,
        #             0,
        #             f"[/vesc_cmds] VescEcatRosCommands (已接收: {data_count['vesc_cmds']} 条):")
        # if data_dict['vesc_cmds']:
        #     msg = data_dict['vesc_cmds']
        #     cmd_table = format_vesc_cmd_table(msg)
        #     for idx, line in enumerate(cmd_table):
        #         safe_addstr(stdscr, current_row + 1 + idx, 2, line)
        #     height = len(cmd_table)
        # else:
        #     safe_addstr(stdscr, current_row + 1, 2, "暂无数据")
        #     height = 1
        # current_row = current_row + 1 + height + 1  # 留1行空白

        # DJIEcatRosCommands (ecat_cmd) 表格显示
        # safe_addstr(stdscr, current_row, 0, f"[/ecat_cmd] DJIEcatRosCommands (已接收：{data_count['ecat_cmd']} 条):")
        # if data_dict['ecat_cmd']:
        #     msg = data_dict['ecat_cmd']
        #     cmd_table = format_dje_cmd_table(msg)
        #     for idx, line in enumerate(cmd_table):
        #         safe_addstr(stdscr, current_row + 1 + idx, 2, line)
        #     height = len(cmd_table)
        # else:
        #     safe_addstr(stdscr, current_row + 1, 2, "暂无数据")
        #     height = 1
        # current_row = current_row + 1 + height + 1

        # DJIEcatRosMsg 表格显示
        safe_addstr(stdscr, current_row, 0, f"[/ecat_msg] DJIEcatRosMsg (已接收: {data_count['ecat_msg']} 条):")
        if data_dict['ecat_msg']:
            msg = data_dict['ecat_msg']
            table_lines = format_dje_msg_table(msg)
            for idx, line in enumerate(table_lines):
                safe_addstr(stdscr, current_row + 1 + idx, 2, line)
            height = len(table_lines)
        else:
            safe_addstr(stdscr, current_row + 1, 2, "暂无数据")
            height = 1
        current_row = current_row + 1 + height + 1

        # VescEcatRosMsg 表格显示
        safe_addstr(stdscr, current_row, 0, f"[/vesc_msg] VescEcatRosMsg (已接收：{data_count['vesc_msg']} 条):")
        if data_dict['vesc_msg']:
            msg = data_dict['vesc_msg']
            table_lines = format_vesc_msg_table(msg)
            for idx, line in enumerate(table_lines):
                safe_addstr(stdscr, current_row + 1 + idx, 2, line)
            height = len(table_lines)    
        else:
            safe_addstr(stdscr, current_row + 1, 2, "暂无数据")
            height = 1
        current_row = current_row + 1 + height + 1
        
        # odom 表格显示
        safe_addstr(stdscr, current_row, 0, f"[/odom] Odometry (已接收：{data_count['odom']} 条):")
        if data_dict['odom']:
            msg = data_dict['odom']
            table_lines = format_odom_table(msg)
            for idx, line in enumerate(table_lines):
                safe_addstr(stdscr, current_row + 1 + idx, 2, line)
            height = len(table_lines)
        else:
            safe_addstr(stdscr, current_row + 1, 2, "暂无数据")
            height = 1
        current_row = current_row + 1 + height + 1

        # imu data 表格显示
        safe_addstr(stdscr, current_row, 0, f"[/mahony/euler] Euler (Vector3Stamped) (已接收: {data_count['mahony/euler']} 条):")
        if data_dict['mahony/euler']:
            msg = data_dict['mahony/euler']
            lines = format_imudata_table(msg)
            for idx, line in enumerate(lines):
                safe_addstr(stdscr, current_row + 1 + idx, 2, line)
            height = len(lines)
        else:
            safe_addstr(stdscr, current_row + 1, 2, "暂无数据")
            height = 1
        current_row += 1 + height + 1

        # camera data show
        safe_addstr(stdscr, current_row, 0, f"[/yolo/detections] detections (BoundingBox2DArray) (已接收: {data_count['yolo/detections']} 条):")
        if data_dict['yolo/detections']:
            msg = data_dict['yolo/detections']
            lines = format_cameradata_table(msg)
            for idx, line in enumerate(lines):
                safe_addstr(stdscr, current_row + 1 + idx, 2, line)
            height = len(lines)
        else:
            safe_addstr(stdscr, current_row + 1, 2, "暂无数据")
            height = 1
        current_row += 1 + height + 1

        safe_addstr(stdscr, current_row, 0, f"[/init_state] InitState (已接收: {data_count['init_state']} 条):")
        if data_dict['init_state']:
            msg = data_dict['init_state']
            table_lines = format_initstate_table(msg)
            for idx, line in enumerate(table_lines):
                safe_addstr(stdscr, current_row + 1 + idx, 2, line)
            height = len(table_lines)
        else:
            safe_addstr(stdscr, current_row + 1, 2, "暂无数据")
            height = 1
        current_row = current_row + 1 + height + 1


        stdscr.refresh()
        rospy.sleep(0.1)

if __name__ == '__main__':
    curses.wrapper(main)