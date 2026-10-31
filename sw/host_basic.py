import tkinter as tk
from tkinter import messagebox
import time,serial

import serial.threaded

FRAME_HEAD=0xAA
FRAME_TAIL=0x55
FRAME_LENGTH=44
BAUD_RATE=115200
DELAY=30

COMMANDS={"停止":b"S","前进":b"F","后退":b"B","左转":b"L","右转":b"R","进入循迹模式":b"T","返回手动蓝牙模式":b"M","清除载荷故障":b"C","清除瓦斯报警":b"E","设置当前重量为零点":b"G"}
STATUS_TEXT={0:"停止",1:"前进",2:"后退",4:"左转",8:"右转",}
TRACK_TEXT={0:"正常循迹",1:"降级循迹",2:"断轨恢复",3:"安全停车"}
LOAD_TEXT={0:"无装卸事件",1:"装料事件",2:"卸料事件",3:"行驶载荷异常"}
FAULT_TEXT={0:"无停车记录",1:"瓦斯超标停车",2:"前方障碍停车",3:"后方障碍停车",4:"人工停车",5:"载荷异常停车",6:"超载停车",7:"未定义停车码"}
STALL_TEXT={0:"正常",1:"左轮堵转",2:"右轮堵转",3:"双轮堵转"}
serial_port=None
receive_buffer=bytearray()
frame_count=0
requested_mode="unknown"

def u16(low,high):
    return(high<<8|low)
def u24(low,mid,high):
    return(high<<16|mid<<8|low)

def parse_data(data):
    receive_buffer.extend(data)
    packets=[]
    while True:
        head=receive_buffer.find(bytes([FRAME_HEAD]))
        if head<0:
            receive_buffer.clear()
            break
        del receive_buffer[:head]
        if len(receive_buffer)<FRAME_LENGTH:
            break
        if receive_buffer[FRAME_LENGTH-1]!=FRAME_TAIL:
            del receive_buffer[0]
            continue
        frame = bytes(receive_buffer[:FRAME_LENGTH])
        del receive_buffer[:FRAME_LENGTH]
        car_state=(frame[1]&0xF)
        heavy_load_state=bool(frame[2]&0x4)
        overload_state=bool(frame[2]&0x2)
        gas_alarm=bool(frame[2]&0x1)
        weight=u24(frame[5],frame[4],frame[3])
        faults=[]
        for code in range(6,11):
            frame_code=frame[code]&0x7
            if frame_code!=0:
                faults.append(FAULT_TEXT.get(frame_code,"未知错误"))
        if faults==[]:
                faults.append("无停车记录")
        front_distance=u16(frame[11],frame[12])
        rear_distance=u16(frame[13],frame[14])
        actual_left_speed=u16(frame[15],frame[16])
        actual_right_speed=u16(frame[17],frame[18])
        target_left_speed=u16(frame[19],frame[20])
        target_right_speed=u16(frame[21],frame[22])
        error_mpu=frame[23]&0xF
        mpu_dir=frame[24]&0x1
        mpu_dir_z=frame[25]&0x1
        error_mpu_dir=frame[26]&0xF
        track_lose=bool(frame[27]&0x1)
        sensor_fault=frame[28]
        track_confidence=frame[29]&0xf
        control_mode=frame[30]&0x3
        load_event_state=frame[31]&0x3
        load_event_toggle=bool(frame[32]&0x1)
        load_change_amount=u24(frame[33],frame[34],frame[35])
        settle_timeout_fault=bool(frame[36]&0x1)
        overload_state_copy=bool(frame[37]&0x1)
        vibration_data=u16(frame[38],frame[39])
        vibration_vid=bool(frame[40]&0x1)
        stall_state=frame[41]&0x3
        stall_alarm=bool(frame[42]&0x01)

        packet={"车辆状态":STATUS_TEXT.get(car_state,"未知状态"),
                "瓦斯状态":gas_alarm,
                "重载状态":heavy_load_state,
                "超载状态":overload_state,
                "重量":weight,
                "五次停车记录":faults,
                "前方距离":front_distance,
                "后方距离":rear_distance,
                "左轮实际速度":actual_left_speed,
                "右轮实际速度":actual_right_speed,
                "左轮目标速度":target_left_speed,
                "右轮目标速度":target_right_speed,
                "MPU传感器故障码":error_mpu,
                "MPU水平方向标志位":mpu_dir,
                "MPU Z轴方向标志位":mpu_dir_z,
                "MPU 方向监测故障码":error_mpu_dir,
                "循迹丢失":track_lose,
                "传感器故障":sensor_fault,
                "循迹置信度":track_confidence,
                "控制模式":control_mode,
                "装载事件状态":load_event_state,
                "装载事件切换":load_event_toggle,
                "装载变化量":load_change_amount,
                "设置超时错误":settle_timeout_fault,
                "超载状态副本":overload_state_copy,
                "振动数据":vibration_data,
                "震动有效标志位":vibration_vid,
                "堵转状态":stall_state,
                "堵转报警":stall_alarm,}
        if gas_alarm==True:
            packet["瓦斯状态"]="报警"
        else:
            packet["瓦斯状态"]="正常"
        if heavy_load_state==True:
            packet["重载状态"]="是"
        else:
            packet["重载状态"]="否"
        if overload_state==True:
            packet["超载状态"]="是"
        else:
            packet["超载状态"]="否"
        if track_lose== True:
            packet["循迹丢失"] = "是"
        else:
            packet["循迹丢失"] = "否"
        if load_event_toggle== True:
            packet["装载事件切换"] = "是"
        else:
            packet["装载事件切换"] = "否"
        if settle_timeout_fault==True:
            packet["设置超时错误"]="是"
        else:
            packet["设置超时错误"]="否"
        if overload_state_copy==True:
            packet["超载状态副本"] ="是"
        else:
            packet["超载状态副本"]="否"
        if vibration_vid==True:
            packet["震动有效标志位"]="是"
        else:
            packet["震动有效标志位"]="否"
        if stall_alarm==True:
            packet["堵转报警"]="是"
        else:
            packet["堵转报警"]="否"
        packets.append(packet)
    return packets
def send_command(name):
    global serial_port

    if serial_port is None or not serial_port.is_open:
        message_var.set("发送失败：串口未连接")
        return

    command = COMMANDS.get(name)

    if command is None:
        message_var.set("未知命令")
        return

    try:
        serial_port.write(command)
        serial_port.flush()

        message_var.set(f"已发送：{name}")
        print("TX:", name, command)

    except Exception as e:
        message_var.set("命令发送失败")
        messagebox.showerror("发送错误", str(e))

def read_serial():
    global serial_port

    try:
        if serial_port is not None and serial_port.is_open:
            count = serial_port.in_waiting
            if count > 0:
                data = serial_port.read(count)
                print("RX:", data.hex(" "))
                packets = parse_data(data)
                for packet in packets:
                    print(packet)
                    update_display(packet)

    except Exception as e:
        print("串口读取错误：", e)
        message_var.set("串口读取错误")

    root.after(DELAY, read_serial)
def connect_serial():
    global serial_port
    port_name = port_entry.get().strip()

    if port_name == "":
        messagebox.showwarning("提示", "请输入串口号，例如 COM3")
        return

    if serial_port is not None and serial_port.is_open:
        message_var.set("串口已经连接")
        return

    try:
        serial_port = serial.Serial(
            port=port_name,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=0.5
        )

        serial_port.reset_input_buffer()
        receive_buffer.clear()

        message_var.set(f"已连接 {port_name}")
        print("串口连接成功：", port_name)

    except Exception as e:
        serial_port = None
        message_var.set("串口连接失败")
        messagebox.showerror("串口错误", str(e))
def disconnect_serial():
    global serial_port

    try:
        if serial_port is not None:
            if serial_port.is_open:
                serial_port.close()

        serial_port = None
        receive_buffer.clear()

        message_var.set("串口已断开")
        print("串口已断开")

    except Exception as e:
        messagebox.showerror("断开失败", str(e))

def on_close():
    disconnect_serial()
    root.destroy()

def update_display(packet):
    car_state_var.set(packet["车辆状态"])
    gas_var.set(packet["瓦斯状态"])
    weight_var.set(packet["重量"])
    load_var.set(packet["重载状态"])
    overload_var.set(packet["超载状态"])
    front_distance_var.set(packet["前方距离"])
    rear_distance_var.set(packet["后方距离"])
    left_speed_var.set(packet["左轮实际速度"])
    right_speed_var.set(packet["右轮实际速度"])
    right_target_var.set(packet["右轮目标速度"])
    left_target_var.set(packet["左轮目标速度"])
    track_lost_var.set(packet["循迹丢失"])
    track_confidence_var.set(packet["循迹置信度"])
    mode = packet["控制模式"]
    control_mode_var.set(TRACK_TEXT.get(mode, f"未知模式 {mode}"))
    vibration_var.set(packet["振动数据"])
    stall = packet["堵转状态"]
    stall_state_var.set(STALL_TEXT.get(stall, f"未知状态 {stall}"))
    stall_alarm_var.set(packet["堵转报警"])


root = tk.Tk()
root.title("AGV检测系统")
root.geometry("900x2000")


port_frame = tk.LabelFrame(root, text="串口连接")
port_frame.pack()

tk.Label(port_frame, text="串口:").pack()

port_entry = tk.Entry(port_frame, width=10)
port_entry.pack()

tk.Button(port_frame, text="连接",command=connect_serial).pack()
tk.Button(port_frame, text="断开",command=disconnect_serial).pack()


real_time_data = tk.LabelFrame(root, text="实时数据")
real_time_data.pack()

car_state_var=tk.StringVar(value="--")
gas_var=tk.StringVar(value="--")
weight_var=tk.StringVar(value="--")
load_var=tk.StringVar(value="--")
overload_var=tk.StringVar(value="--")
front_distance_var=tk.StringVar(value="--")
rear_distance_var=tk.StringVar(value="--")
left_speed_var=tk.StringVar(value="--")
right_speed_var=tk.StringVar(value="--")
track_lost_var=tk.StringVar(value="--")
track_confidence_var=tk.StringVar(value="--")
control_mode_var=tk.StringVar(value="--")
vibration_var=tk.StringVar(value="--")
stall_state_var=tk.StringVar(value="--")
stall_alarm_var=tk.StringVar(value="--")
right_target_var = tk.StringVar(value="--")
left_target_var = tk.StringVar(value="--")

tk.Label(real_time_data,text="车辆状态").grid(row=0,column=0)
tk.Label(real_time_data,textvariable=car_state_var).grid(row=0,column=1)

tk.Label(real_time_data,text="瓦斯状态").grid(row=1,column=0)
tk.Label(real_time_data,textvariable=gas_var).grid(row=1,column=1)

tk.Label(real_time_data,text="重量").grid(row=2,column=0)
tk.Label(real_time_data,textvariable=weight_var).grid(row=2,column=1)

tk.Label(real_time_data,text="重载状态").grid(row=3,column=0)
tk.Label(real_time_data,textvariable=load_var).grid(row=3,column=1)

tk.Label(real_time_data,text="超载状态").grid(row=4,column=0)
tk.Label(real_time_data,textvariable=overload_var).grid(row=4,column=1)

tk.Label(real_time_data,text="前方距离").grid(row=5,column=0)
tk.Label(real_time_data,textvariable=front_distance_var).grid(row=5,column=1)

tk.Label(real_time_data,text="后方距离").grid(row=6,column=0)
tk.Label(real_time_data,textvariable=rear_distance_var).grid(row=6,column=1)

tk.Label(real_time_data,text="左轮实际速度").grid(row=7,column=0)
tk.Label(real_time_data,textvariable=left_speed_var).grid(row=7,column=1)

tk.Label(real_time_data,text="右轮实际速度").grid(row=8,column=0)
tk.Label(real_time_data,textvariable=right_speed_var).grid(row=8,column=1)

tk.Label(real_time_data,text="循迹丢失").grid(row=9,column=0)
tk.Label(real_time_data,textvariable=track_lost_var).grid(row=9,column=1)

tk.Label(real_time_data,text="循迹置信度").grid(row=10,column=0)
tk.Label(real_time_data,textvariable=track_confidence_var).grid(row=10,column=1)

tk.Label(real_time_data,text="控制模式").grid(row=11,column=0)
tk.Label(real_time_data,textvariable=control_mode_var).grid(row=11,column=1)

tk.Label(real_time_data,text="振动数据").grid(row=12,column=0)
tk.Label(real_time_data,textvariable=vibration_var).grid(row=12,column=1)

tk.Label(real_time_data,text="堵转状态").grid(row=13,column=0)
tk.Label(real_time_data,textvariable=stall_state_var).grid(row=13,column=1)

tk.Label(real_time_data,text="堵转报警").grid(row=14,column=0)
tk.Label(real_time_data,textvariable=stall_alarm_var).grid(row=14,column=1)

tk.Label(real_time_data,text="右轮目标速度").grid(row=15,column=0)
tk.Label(real_time_data,textvariable=right_target_var).grid(row=15,column=1)

tk.Label(real_time_data,text="左轮目标速度").grid(row=16,column=0)
tk.Label(real_time_data,textvariable=left_target_var).grid(row=16,column=1)

control_frame = tk.LabelFrame(root, text="控制区域")
control_frame.pack()

tk.Button(control_frame, text="前进",command=lambda :send_command("前进")).grid()
tk.Button(control_frame, text="左转",command=lambda :send_command("左转")).grid()
tk.Button(control_frame, text="停止",command=lambda :send_command("停止")).grid()
tk.Button(control_frame, text="右转",command=lambda :send_command("右转")).grid()
tk.Button(control_frame, text="后退",command=lambda :send_command("后退")).grid()

other_frame = tk.LabelFrame(root, text="控制与维护")
other_frame.pack()

tk.Button(other_frame, text="进入循迹模式",command=lambda :send_command("进入循迹模式")).pack()
tk.Button(other_frame, text="返回蓝牙手动模式", command=lambda:send_command("返回蓝牙手动模式")).pack()
tk.Button(other_frame, text="清除载荷故障",command=lambda:send_command("清除载荷故障")).pack()
tk.Button(other_frame, text="清除瓦斯报警",command=lambda:send_command("清除瓦斯报警")).pack()
tk.Button(other_frame, text="当前重量设置为零点",command=lambda:send_command("当前重量设置为零点")).pack()

message_frame = tk.LabelFrame(root, text="最新消息")
message_frame.pack()

message_var = tk.StringVar(value="等待消息")

tk.Label(
    message_frame,
    textvariable=message_var
).pack()


root.after(DELAY, read_serial)
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()