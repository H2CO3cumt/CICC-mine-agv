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

# =============================================================================
# 下面开始为“显示层 / 安全包装层”
# 说明：
# 1) 上面 1~264 行来自“最终版(2).py”，原样保留，不改 parse_data / send_command /
#    read_serial / connect_serial / disconnect_serial / 原 update_display 的核心实现。
# 2) 下面只做变量名适配、工业 UI、美化、曲线与安全冗余。
# =============================================================================
from tkinter import ttk
import math
from collections import deque

# 第一份工业版使用的名称；这里只做兼容别名，不删除第二份核心原有名称。
MODE_SWITCH_DELAY = DELAY
HISTORY_POINTS = 240
CAR_STATE_TEXT = STATUS_TEXT
STALL_STATE_TEXT = STALL_TEXT

# 保留第二份原命令键，同时增加第一份界面使用的中文命名别名；实际发送字节不变。
COMMANDS.setdefault("返回蓝牙手动模式", COMMANDS["返回手动蓝牙模式"])
COMMANDS.setdefault("当前重量设为零点", COMMANDS["设置当前重量为零点"])

# 保存第二份原始显示函数，后面用“包装函数”扩展第一份 UI 的完整显示。
_core_update_display = update_display

mode_switch_job = None
last_packet_monotonic = None


def _serial_is_open():
    return serial_port is not None and getattr(serial_port, "is_open", False)


def safe_connect_serial():
    """安全包装：不改原 connect_serial，只在外层防重复连接并连接后先发停止。"""
    global requested_mode, frame_count, mode_switch_job, last_packet_monotonic

    if _serial_is_open():
        message_var.set("串口已经连接，请勿重复连接")
        status_var.set("已连接 " + port_entry.get().strip())
        return

    if mode_switch_job is not None:
        try:
            root.after_cancel(mode_switch_job)
        except (tk.TclError, ValueError):
            pass
        mode_switch_job = None

    connect_serial()

    if _serial_is_open():
        frame_count = 0
        requested_mode = "unknown"
        last_packet_monotonic = None
        status_var.set("已连接 " + port_entry.get().strip())
        mode_var.set("模式未知")

        # 安全冗余：连接建立后先发送停止，防止下位机保留上一次运动状态。
        send_command("停止")
        message_var.set("连接成功，已发送停止命令")
    else:
        status_var.set("连接失败")
        mode_var.set("模式未知")


def safe_disconnect_serial():
    """安全包装：断开前尽量回到手动模式并发送停止，再调用原 disconnect_serial。"""
    global requested_mode, mode_switch_job, last_packet_monotonic

    if mode_switch_job is not None:
        try:
            root.after_cancel(mode_switch_job)
        except (tk.TclError, ValueError):
            pass
        mode_switch_job = None

    if _serial_is_open():
        try:
            if requested_mode == "tracking":
                send_command("返回手动蓝牙模式")
                time.sleep(0.03)
            send_command("停止")
        except Exception:
            # 即使设备已经拔掉，也继续释放本地串口资源。
            pass

    disconnect_serial()
    requested_mode = "unknown"
    last_packet_monotonic = None
    status_var.set("未连接")
    mode_var.set("模式未知")
    message_var.set("串口已断开")


def _schedule_mode_finish(callback):
    global mode_switch_job
    if mode_switch_job is not None:
        message_var.set("模式正在切换，请勿重复操作")
        return False
    mode_switch_job = root.after(MODE_SWITCH_DELAY, callback)
    return True


def _finish_manual_mode():
    global requested_mode, mode_switch_job
    mode_switch_job = None
    if not _serial_is_open():
        requested_mode = "unknown"
        mode_var.set("模式未知")
        message_var.set("模式切换失败：串口已断开")
        return
    send_command("停止")
    requested_mode = "manual"
    mode_var.set("蓝牙手动模式")
    message_var.set("已进入蓝牙手动模式")


def request_manual_mode():
    global requested_mode
    if not _serial_is_open():
        message_var.set("串口未连接")
        return
    if requested_mode == "switching" or mode_switch_job is not None:
        message_var.set("模式正在切换，请勿重复操作")
        return

    requested_mode = "switching"
    mode_var.set("切换中")
    send_command("返回手动蓝牙模式")
    if not _schedule_mode_finish(_finish_manual_mode):
        requested_mode = "unknown"
        mode_var.set("模式未知")


def _finish_tracking_mode():
    global requested_mode, mode_switch_job
    mode_switch_job = None
    if not _serial_is_open():
        requested_mode = "unknown"
        mode_var.set("模式未知")
        message_var.set("模式切换失败：串口已断开")
        return
    send_command("进入循迹模式")
    requested_mode = "tracking"
    mode_var.set("循迹模式")
    message_var.set("已进入循迹模式")


def request_tracking_mode():
    global requested_mode
    if not _serial_is_open():
        message_var.set("串口未连接")
        return
    if requested_mode == "switching" or mode_switch_job is not None:
        message_var.set("模式正在切换，请勿重复操作")
        return

    requested_mode = "switching"
    mode_var.set("切换中")
    send_command("停止")
    if not _schedule_mode_finish(_finish_tracking_mode):
        requested_mode = "unknown"
        mode_var.set("模式未知")


def send_movement(name):
    """方向控制安全包装：循迹/切换中不直接发送人工运动命令。"""
    if not _serial_is_open():
        message_var.set("串口未连接")
        return

    if requested_mode == "switching":
        message_var.set("模式切换中，暂不接受方向命令")
        return

    if requested_mode == "tracking":
        if name == "停止":
            request_manual_mode()
        else:
            message_var.set("当前为循迹模式，请先返回蓝牙手动模式")
        return

    send_command(name)


def clear_load_fault():
    if not _serial_is_open():
        message_var.set("串口未连接")
        return
    if car_state_var.get() != "停止":
        message_var.set("清除载荷故障前请先停车")
        return
    if not messagebox.askyesno(
        "清除载荷故障",
        "将发送 C，清除载荷故障和行驶载荷异常锁存，确认继续吗？"
    ):
        return
    send_command("清除载荷故障")


def clear_gas_alarm():
    if not _serial_is_open():
        message_var.set("串口未连接")
        return
    if not messagebox.askyesno(
        "清除瓦斯报警",
        "请确认瓦斯风险已经消除。将发送 E，确认继续吗？"
    ):
        return
    send_command("清除瓦斯报警")


def tare_zero():
    if not _serial_is_open():
        message_var.set("串口未连接")
        return
    if car_state_var.get() != "停止":
        message_var.set("重量设零前请先停车")
        return
    if weight_timeout_var.get() == "超时":
        message_var.set("载荷故障仍在锁存，请先清除载荷故障")
        return
    if not messagebox.askyesno(
        "当前重量设为零点",
        "将发送 G，把当前显示重量 " + weight_var.get() + " 设置为零点，确认继续吗？"
    ):
        return
    send_command("设置当前重量为零点")


def _normalize_to_first_packet(packet):
    """
    第二份核心 packet -> 第一份工业 UI 字段名。
    这里只做显示适配，不改变第二份 parse_data() 的字节位置和解析算法。
    """
    faults = packet.get("五次停车记录", [])
    if isinstance(faults, (list, tuple)):
        fault_items = [str(item) for item in faults if str(item).strip()]
        fault_text = "；".join(fault_items) if fault_items else "无停车记录"
    else:
        fault_text = str(faults).strip() or "无停车记录"

    heavy = packet.get("重载状态")
    overload = packet.get("超载状态")
    track_lost = packet.get("循迹丢失")
    load_toggle = packet.get("装载事件切换")
    timeout = packet.get("设置超时错误")
    overload_copy = packet.get("超载状态副本")
    vib_valid = packet.get("震动有效标志位")
    stall_alarm = packet.get("堵转报警")

    control_mode = int(packet.get("控制模式", 0))
    load_event = int(packet.get("装载事件状态", 0))
    stall_state = int(packet.get("堵转状态", 0))

    left_actual = int(packet.get("左轮实际速度", 0))
    right_actual = int(packet.get("右轮实际速度", 0))
    left_target = int(packet.get("左轮目标速度", 0))
    right_target = int(packet.get("右轮目标速度", 0))

    return {
        "车辆状态": packet.get("车辆状态", "未知状态"),
        "瓦斯状态": packet.get("瓦斯状态", "--"),
        "过载状态": "过载" if overload in (True, 1, "是", "过载") else "正常",
        "重载状态": "重载" if heavy in (True, 1, "是", "重载") else "正常",
        "重量": int(packet.get("重量", 0)),
        "停车原因": fault_text,
        "前方距离": int(packet.get("前方距离", 0)),
        "后方距离": int(packet.get("后方距离", 0)),
        "左轮速度": f"实际 {left_actual} / 目标 {left_target}",
        "右轮速度": f"实际 {right_actual} / 目标 {right_target}",
        "MPU误差": int(packet.get("MPU传感器故障码", 0)),
        "MPU方向": "1" if packet.get("MPU水平方向标志位") in (True, 1, "1", "是") else "0",
        "MPU_Z方向": "1" if packet.get("MPU Z轴方向标志位") in (True, 1, "1", "是") else "0",
        "MPU_Z误差": int(packet.get("MPU 方向监测故障码", 0)),
        "循迹丢失": "丢失" if track_lost in (True, 1, "1", "是", "丢失") else "正常",
        "传感器故障": int(packet.get("传感器故障", 0)),
        "循迹置信度": int(packet.get("循迹置信度", 0)),
        "循迹状态": TRACK_TEXT.get(control_mode, "未知状态"),
        "装卸状态": LOAD_TEXT.get(load_event, "未知状态"),
        "装载转换": "是" if load_toggle in (True, 1, "1", "是") else "否",
        "重量变化": int(packet.get("装载变化量", 0)),
        "重量超时": "超时" if timeout in (True, 1, "1", "是", "超时") else "正常",
        "过载副本": "过载" if overload_copy in (True, 1, "1", "是", "过载") else "正常",
        "振动数据": int(packet.get("振动数据", 0)),
        "振动有效": "有效" if vib_valid in (True, 1, "1", "是", "有效") else "无效",
        "堵转状态": STALL_TEXT.get(stall_state, "未知状态"),
        "堵转报警": "报警" if stall_alarm in (True, 1, "1", "是", "报警") else "正常",
    }

# =============================================================================
# 实时折线图（显示层）
# 与 UI最终1.0.py 相同：Tkinter Canvas 绘制，不依赖 matplotlib
# =============================================================================
class LineChart(tk.Canvas):
    PALETTE=("#2f6fad","#5b6773","#2f8f6b","#c05a4a")

    def __init__(self,parent,series_names,unit,max_points=HISTORY_POINTS,fixed_range=None,height=330):
        super().__init__(
            parent,
            background="#fbfcfe",
            highlightbackground="#cbd5df",
            highlightthickness=1,
            height=height,
        )
        self.series_names=list(series_names)
        self.unit=unit
        self.max_points=max_points
        self.fixed_range=fixed_range
        self.series={name:deque(maxlen=max_points) for name in self.series_names}
        self.bind("<Configure>",lambda _event:self.redraw())

    def append(self,values):
        for name in self.series_names:
            value=values.get(name)
            self.series[name].append(None if value is None else float(value))
        if self.winfo_ismapped():
            self.redraw()

    def clear_history(self):
        for data in self.series.values():
            data.clear()
        self.redraw()

    def redraw(self):
        self.delete("all")
        width=max(300,self.winfo_width())
        height=max(190,self.winfo_height())
        left,right=62,width-18
        top=38
        bottom=height-35

        valid=[
            value
            for data in self.series.values()
            for value in data
            if value is not None and math.isfinite(value)
        ]
        if not valid:
            self.create_text(width/2,height/2,text="等待数据",fill="#777777",font=("Microsoft YaHei UI",11))
            return

        if self.fixed_range is not None:
            min_value,max_value=self.fixed_range
        else:
            min_value,max_value=min(valid),max(valid)
            if min_value==max_value:
                pad=max(1.0,abs(min_value)*0.08)
            else:
                pad=(max_value-min_value)*0.10
            min_value-=pad
            max_value+=pad
            if min_value>0:
                min_value=0.0

        if max_value<=min_value:
            max_value=min_value+1.0

        plot_width=max(1,right-left)
        plot_height=max(1,bottom-top)

        # 图例
        legend_x=left
        for index,name in enumerate(self.series_names):
            color=self.PALETTE[index%len(self.PALETTE)]
            self.create_line(legend_x,18,legend_x+22,18,fill=color,width=2)
            self.create_text(legend_x+28,18,anchor="w",text=name,fill="#334155",font=("Microsoft YaHei UI",9,"bold"))
            legend_x+=110

        # 网格与刻度
        for row in range(6):
            ratio=row/5
            y=top+ratio*plot_height
            value=max_value-ratio*(max_value-min_value)
            self.create_line(left,y,right,y,fill="#e8edf3")
            self.create_text(left-7,y,anchor="e",text=f"{value:.0f}",fill="#64748b",font=("Consolas",8))

        for col in range(7):
            x=left+col/6*plot_width
            self.create_line(x,top,x,bottom,fill="#f0f3f7")

        self.create_line(left,top,left,bottom,fill="#9aa8b6")
        self.create_line(left,bottom,right,bottom,fill="#9aa8b6")
        self.create_text(8,top,anchor="nw",text=self.unit,fill="#64748b",font=("Microsoft YaHei UI",8,"bold"))

        if min_value<0<max_value:
            zero_y=bottom-(0-min_value)/(max_value-min_value)*plot_height
            self.create_line(left,zero_y,right,zero_y,fill="#aab6c2",dash=(4,3))

        denominator=max(1,self.max_points-1)
        for series_index,name in enumerate(self.series_names):
            data=list(self.series[name])
            color=self.PALETTE[series_index%len(self.PALETTE)]
            start_index=self.max_points-len(data)
            segment=[]
            last_xy=None

            for local_index,value in enumerate(data):
                global_index=start_index+local_index
                x=left+global_index/denominator*plot_width
                if value is None or not math.isfinite(value):
                    if len(segment)>=4:
                        self.create_line(*segment,fill=color,width=2)
                    segment=[]
                    continue
                y=bottom-(value-min_value)/(max_value-min_value)*plot_height
                y=max(top,min(bottom,y))
                segment.extend((x,y))
                last_xy=(x,y)

            if len(segment)>=4:
                self.create_line(*segment,fill=color,width=2)
            elif len(segment)==2:
                x,y=segment
                self.create_oval(x-2,y-2,x+2,y+2,fill=color,outline=color)

            if last_xy is not None:
                x,y=last_xy
                latest=next((v for v in reversed(data) if v is not None),None)
                if latest is not None:
                    self.create_oval(x-2,y-2,x+2,y+2,fill=color,outline=color)
                    self.create_text(
                        min(right-2,x-4),
                        y-8-series_index*13,
                        anchor="se",
                        text=f"{latest:.0f}",
                        fill=color,
                        font=("Consolas",8,"bold"),
                    )

        self.create_text(
            right, height-11, anchor="se",
            text=f"最近 {self.max_points} 个采样点",
            fill="#94a3b8", font=("Microsoft YaHei UI",8)
        )



root=tk.Tk()
root.title("基于 Robei EDA 的多传感融合智能矿用 AGV")
root.geometry("1380x820")
root.minsize(1120,720)
root.configure(background="#f2f2f2")

# =============================================================================
# 界面变量（仅显示层，不改变原有通信与控制逻辑）
# =============================================================================
port_var=tk.StringVar(value="COM5")
status_var=tk.StringVar(value="未连接")
mode_var=tk.StringVar(value="模式未知")
message_var=tk.StringVar(value="请输入串口并点击连接")
car_state_var=tk.StringVar(value="--")
gas_var=tk.StringVar(value="--")
overload_var=tk.StringVar(value="--")
heavy_load_var=tk.StringVar(value="--")
weight_var=tk.StringVar(value="--")
fault_var=tk.StringVar(value="--")
fault_history_var=tk.StringVar(value="--")
front_distance_var=tk.StringVar(value="--")
rear_distance_var=tk.StringVar(value="--")
distance_pair_var=tk.StringVar(value="前 -- mm\n后 -- mm")
distance_pair_inline_var=tk.StringVar(value="前 -- / 后 --")
left_speed_var=tk.StringVar(value="--")
right_speed_var=tk.StringVar(value="--")
mpu_error_var=tk.StringVar(value="--")
mpu_dir_var=tk.StringVar(value="--")
mpu_z_dir_var=tk.StringVar(value="--")
mpu_z_error_var=tk.StringVar(value="--")
track_lose_var=tk.StringVar(value="--")
sensor_fault_var=tk.StringVar(value="--")
track_confidence_var=tk.StringVar(value="--")
track_state_var=tk.StringVar(value="--")
load_state_var=tk.StringVar(value="--")
load_switch_var=tk.StringVar(value="--")
weight_change_var=tk.StringVar(value="--")
weight_timeout_var=tk.StringVar(value="--")
overload_copy_var=tk.StringVar(value="--")
vibration_data_var=tk.StringVar(value="--")
stall_state_var=tk.StringVar(value="--")
stall_alarm_var=tk.StringVar(value="--")
frame_length_var=tk.StringVar(value=str(FRAME_LENGTH)+" Byte")
baud_rate_var=tk.StringVar(value=str(BAUD_RATE))

# =============================================================================
# 视觉样式：参考 UI最终1.0.py 的浅灰背景、分页、卡片式信息区和统一字体
# =============================================================================
style=ttk.Style()
try:
    style.theme_use("clam")
except tk.TclError:
    pass

BG="#e9edf1"
CARD="#f8fafb"
TEXT="#15212b"
MUTED="#5f6e7a"
BORDER="#aeb8c2"
PRIMARY="#1f5f8b"
NAVY="#102a43"
SUCCESS="#2f7655"
WARNING="#9a691f"
DANGER="#a33e3e"
SOFT_BLUE="#e2ebf3"

style.configure("TFrame",background=BG)
style.configure("TLabel",background=BG,foreground=TEXT,font=("Microsoft YaHei UI",9))
style.configure("Title.TLabel",background=BG,foreground=TEXT,font=("Microsoft YaHei UI",16,"bold"))
style.configure("SubTitle.TLabel",background=BG,foreground=MUTED,font=("Microsoft YaHei UI",8))
style.configure("Value.TLabel",background=BG,foreground="#0e3654",font=("Consolas",14,"bold"))
style.configure("Status.TLabel",background=BG,foreground=TEXT,font=("Microsoft YaHei UI",9,"bold"))
style.configure("TButton",font=("Microsoft YaHei UI",9,"bold"),padding=(10,6),background="#d7dee5",foreground="#20303d",borderwidth=1)
style.configure("Direction.TButton",font=("Microsoft YaHei UI",14,"bold"),padding=10,background="#dce5ed",foreground=NAVY)
style.configure("Stop.TButton",font=("Microsoft YaHei UI",11,"bold"),padding=10,background="#ead5d5",foreground="#7f2525")
style.configure("Primary.TButton",font=("Microsoft YaHei UI",9,"bold"),padding=(11,6),background="#cbddeb",foreground="#123f60")
style.map("Primary.TButton",background=[("active","#b9d2e4"), ("pressed","#a8c5da")])
style.map("Stop.TButton",background=[("active","#dfc2c2"), ("pressed","#d4abab")])
style.configure("TLabelframe",background=BG,bordercolor="#97a5b1",relief="solid",borderwidth=1)
style.configure("TLabelframe.Label",background=BG,foreground="#21394b",font=("Microsoft YaHei UI",10,"bold"))
style.configure("TNotebook",background=BG,borderwidth=0,tabmargins=(0,4,0,0))
style.configure("TNotebook.Tab",font=("Microsoft YaHei UI",9,"bold"),padding=(14,7),background="#d3dbe2",foreground="#465563",borderwidth=1)
style.map("TNotebook.Tab",background=[("selected","#f8fafb"), ("active","#e1e8ee")],foreground=[("selected","#123f60"), ("active",NAVY)])
style.configure("TEntry",padding=5,fieldbackground="#f9fbfc",bordercolor="#8f9ba6")


def value_row(parent,row,title,variable,title_width=15):
    ttk.Label(parent,text=title,width=title_width).grid(row=row,column=0,sticky="w",padx=(6,4),pady=5)
    ttk.Label(parent,textvariable=variable,style="Value.TLabel").grid(row=row,column=1,sticky="w",padx=(4,8),pady=5)


def value_grid(parent,items,columns=3):
    for index,(title,variable) in enumerate(items):
        row=index//columns
        base_col=(index%columns)*2
        ttk.Label(parent,text=title).grid(row=row,column=base_col,sticky="w",padx=(6,4),pady=5)
        ttk.Label(parent,textvariable=variable,style="Value.TLabel").grid(row=row,column=base_col+1,sticky="w",padx=(0,12),pady=5)
    for col in range(columns*2):
        parent.columnconfigure(col,weight=1 if col%2==1 else 0)


def metric_card(parent,title,variable,accent,subtitle=""):
    """工业仪表卡片：仅显示现有 StringVar，不参与控制逻辑。"""
    card=tk.Frame(parent,background=CARD,highlightbackground="#9aa7b2",highlightthickness=1)
    head=tk.Frame(card,background="#e3e8ed",height=25)
    head.pack(fill=tk.X)
    head.pack_propagate(False)
    tk.Frame(head,background=accent,width=5).pack(side=tk.LEFT,fill=tk.Y)
    tk.Label(head,text=title,background="#e3e8ed",foreground="#344655",font=("Microsoft YaHei UI",9,"bold")).pack(side=tk.LEFT,padx=7)
    inner=tk.Frame(card,background=CARD,padx=10,pady=7)
    inner.pack(fill=tk.BOTH,expand=True)
    tk.Label(inner,textvariable=variable,background=CARD,foreground="#0c3553",font=("Consolas",20,"bold")).pack(anchor="w")
    if subtitle:
        tk.Label(inner,text=subtitle,background=CARD,foreground="#7b8792",font=("Consolas",8)).pack(anchor="w",pady=(1,0))
    return card


def bind_badge(label,variable,kind):
    """根据已有显示变量改变徽标颜色，不产生新的业务状态。"""
    def refresh(*_args):
        value=variable.get()
        bg="#d8e0e6"
        fg="#34434f"
        if kind=="connection":
            if value.startswith("已连接"):
                bg="#cfe5d8"; fg="#245e42"
            elif "失败" in value:
                bg="#ead0d0"; fg="#842f2f"
        elif kind=="mode":
            if "循迹" in value:
                bg="#d9d5e9"; fg="#4f4682"
            elif "手动" in value:
                bg="#d2e0eb"; fg="#285575"
            elif "切换" in value:
                bg="#e9ddc4"; fg="#79551f"
        label.configure(background=bg,foreground=fg,text=value)
    variable.trace_add("write",refresh)
    refresh()


charts=[]
curves_paused=False


def make_chart_box(parent,title,series,unit,fixed_range=None,height=330):
    box=ttk.LabelFrame(parent,text=title,padding=8)
    chart=LineChart(box,series_names=series,unit=unit,fixed_range=fixed_range,height=height)
    chart.pack(fill=tk.BOTH,expand=True)
    charts.append(chart)
    return box,chart


def toggle_pause_curves():
    global curves_paused
    curves_paused=not curves_paused
    pause_button.configure(text="继续曲线" if curves_paused else "暂停曲线")
    message_var.set("曲线刷新已暂停" if curves_paused else "曲线刷新已继续")


def clear_curves():
    for chart in charts:
        chart.clear_history()
    message_var.set("全部实时曲线已清空")


def mpu_signed_for_chart(packet):
    error=int(packet["MPU误差"])
    if error==0:
        return 0
    return error if packet["MPU方向"]=="1" else -error


def speed_pair_for_chart(speed_text):
    # 原 packet 只保存“实际 x / 目标 y”显示文本；折线图仅从该文本读取数值，
    # 不增加协议字段，也不改变 parse_data() 的核心输出结构。
    try:
        actual_text,target_text=speed_text.split("/",1)
        actual=int(actual_text.replace("实际","").strip())
        target=int(target_text.replace("目标","").strip())
        return actual,target
    except (ValueError,AttributeError):
        return None,None


def format_distance_pair(front_text,rear_text):
    front=(front_text or "--").strip()
    rear=(rear_text or "--").strip()
    return f"前 {front} mm\n后 {rear} mm", f"前 {front} / 后 {rear}"


def format_sensor_fault(mask):
    """把 8 位 sensor_fault 掩码转换为灰度传感器的物理位置。"""
    try:
        mask=int(mask)&0xFF
    except (TypeError,ValueError):
        return "--"

    # TRACKING_SENSOR 中 0~7 号通道依次对应：右4、右3、右2、右1、左1、左2、左3、左4
    physical_positions=("右4","右3","右2","右1","左1","左2","左3","左4")
    fault_positions=[
        physical_positions[bit]
        for bit in range(8)
        if mask & (1<<bit)
    ]

    if not fault_positions:
        return "无故障"
    return "、".join(fault_positions)


sensor_fault_widgets={}
SENSOR_VISUAL_ORDER=(("左4",7),("左3",6),("左2",5),("左1",4),("右1",3),("右2",2),("右3",1),("右4",0))

class SensorChannelIndicator(tk.Frame):
    """工业 I/O 风格的单路灰度传感器状态灯。"""
    def __init__(self,parent,name,bit):
        super().__init__(parent,background="#f7f9fa",highlightbackground="#9ba7b1",highlightthickness=1)
        self.name=name
        self.bit=bit
        tk.Label(self,text=name,background="#dfe5ea",foreground="#263846",font=("Microsoft YaHei UI",9,"bold"),pady=4).pack(fill=tk.X)
        self.canvas=tk.Canvas(self,width=54,height=42,background="#f7f9fa",highlightthickness=0)
        self.canvas.pack(pady=(5,0))
        self.led=self.canvas.create_oval(14,5,40,31,fill="#aeb7bf",outline="#68737d",width=2)
        self.state_label=tk.Label(self,text="--",background="#f7f9fa",foreground="#697783",font=("Microsoft YaHei UI",9,"bold"),pady=4)
        self.state_label.pack(fill=tk.X)

    def set_state(self,state):
        if state is None:
            fill="#aeb7bf"; outline="#68737d"; text="未判定"; fg="#697783"
        elif state:
            fill="#c64545"; outline="#7d2424"; text="故障"; fg="#922f2f"
        else:
            fill="#3d8b62"; outline="#245c40"; text="正常"; fg="#2c6f4d"
        self.canvas.itemconfigure(self.led,fill=fill,outline=outline)
        self.state_label.configure(text=text,foreground=fg)


def update_sensor_fault_visual(mask):
    """把 sensor_fault 掩码渲染成 8 路物理位置工业状态灯。"""
    try:
        mask=int(mask)&0xFF
    except (TypeError,ValueError):
        mask=None

    for name,bit in SENSOR_VISUAL_ORDER:
        widget=sensor_fault_widgets.get(name)
        if widget is None:
            continue
        widget.set_state(None if mask is None else bool(mask & (1<<bit)))

def format_fault_history(fault_text):
    text=(fault_text or "").strip()
    if not text or text=="--":
        return "最近一次：--\n第 2 次：--\n第 3 次：--\n第 4 次：--\n第 5 次：--"
    if text=="无停车记录":
        return "最近一次：无停车记录\n第 2 次：--\n第 3 次：--\n第 4 次：--\n第 5 次：--"
    items=[item.strip() for item in text.split("；") if item.strip()]
    labels=["最近一次","第 2 次","第 3 次","第 4 次","第 5 次"]
    lines=[]
    for i,label in enumerate(labels):
        value=items[i] if i < len(items) else "--"
        lines.append(f"{label}：{value}")
    return "\n".join(lines)


# =============================================================================
# 顶部标题与串口连接区
# =============================================================================
header=tk.Frame(root,background=NAVY,height=70)
header.pack(fill=tk.X)
header.pack_propagate(False)

title_area=tk.Frame(header,background=NAVY)
title_area.pack(side=tk.LEFT,fill=tk.BOTH,expand=True,padx=(16,8),pady=10)
tk.Label(
    title_area,
    text="基于 Robei EDA 的多传感融合智能矿用 AGV",
    background="#182b39",foreground="#eef4f8",
    font=("Microsoft YaHei UI",16,"bold"),
).pack(anchor="w")
tk.Label(
    title_area,
    text="FPGA 多源感知 / 安全决策 / 闭环运动控制 / 运行诊断",
    background=NAVY,foreground="#b9c7d3",
    font=("Consolas",9),
).pack(anchor="w",pady=(4,0))

connection=tk.Frame(header,background=NAVY)
connection.pack(side=tk.RIGHT,padx=(8,16),pady=13)
tk.Label(connection,text="串口",background=NAVY,foreground="#d8e2ec",font=("Microsoft YaHei UI",9,"bold")).pack(side=tk.LEFT,padx=(0,6))
port_entry=ttk.Entry(connection,textvariable=port_var,width=14)
port_entry.pack(side=tk.LEFT,padx=4,ipady=1)
ttk.Button(connection,text="连接",command=safe_connect_serial,style="Primary.TButton").pack(side=tk.LEFT,padx=4)
ttk.Button(connection,text="断开",command=safe_disconnect_serial).pack(side=tk.LEFT,padx=4)

status_bar=tk.Frame(root,background="#263b4b",highlightbackground="#172a38",highlightthickness=1)
status_bar.pack(fill=tk.X,padx=8,pady=(6,6))

status_left=tk.Frame(status_bar,background="#263b4b")
status_left.pack(side=tk.LEFT,padx=10,pady=7)
tk.Label(status_left,text="通信",background="#263b4b",foreground="#b8c5cf",font=("Microsoft YaHei UI",9,"bold")).pack(side=tk.LEFT,padx=(0,5))
connection_badge=tk.Label(status_left,text="",font=("Microsoft YaHei UI",9,"bold"),padx=10,pady=4)
connection_badge.pack(side=tk.LEFT,padx=(0,16))
tk.Label(status_left,text="模式",background="#263b4b",foreground="#b8c5cf",font=("Microsoft YaHei UI",9,"bold")).pack(side=tk.LEFT,padx=(0,5))
mode_badge=tk.Label(status_left,text="",font=("Microsoft YaHei UI",9,"bold"),padx=10,pady=4)
mode_badge.pack(side=tk.LEFT)
bind_badge(connection_badge,status_var,"connection")
bind_badge(mode_badge,mode_var,"mode")

status_right=tk.Frame(status_bar,background="#263b4b")
status_right.pack(side=tk.RIGHT,padx=7,pady=5)
ttk.Button(status_right,text="清空曲线",command=clear_curves).pack(side=tk.RIGHT,padx=3)
pause_button=ttk.Button(status_right,text="暂停曲线",command=toggle_pause_curves)
pause_button.pack(side=tk.RIGHT,padx=3)

# =============================================================================
# 主分页区域
# =============================================================================
notebook=ttk.Notebook(root)
notebook.pack(fill=tk.BOTH,expand=True,padx=8,pady=(0,6))

# -----------------------------------------------------------------------------
# 1. 系统总览
# -----------------------------------------------------------------------------
overview_page=ttk.Frame(notebook,padding=6)
notebook.add(overview_page,text="系统总览")

summary=ttk.LabelFrame(overview_page,text="系统关键状态",padding=10)
summary.pack(fill=tk.X)
summary_grid=tk.Frame(summary,background=BG)
summary_grid.pack(fill=tk.X)
for col in range(6):
    summary_grid.columnconfigure(col,weight=1,uniform="overview_metric")

metric_card(summary_grid,"车辆状态",car_state_var,PRIMARY,"CAR_STATE").grid(row=0,column=0,sticky="nsew",padx=(0,5),pady=2)
metric_card(summary_grid,"当前重量",weight_var,"#5e7d9a","HX711 / weight_data").grid(row=0,column=1,sticky="nsew",padx=5,pady=2)
metric_card(summary_grid,"瓦斯状态",gas_var,DANGER,"MQ-4 safety").grid(row=0,column=2,sticky="nsew",padx=5,pady=2)
metric_card(summary_grid,"双向距离",distance_pair_var,"#4a8f8b","Ultrasonic / mm").grid(row=0,column=3,sticky="nsew",padx=5,pady=2)
metric_card(summary_grid,"循迹状态",track_state_var,"#6d63a8","Tracking FSM").grid(row=0,column=4,sticky="nsew",padx=5,pady=2)
metric_card(summary_grid,"堵转状态",stall_state_var,WARNING,"Motor protection").grid(row=0,column=5,sticky="nsew",padx=(5,0),pady=2)

overview_detail=ttk.Frame(summary)
overview_detail.pack(fill=tk.X,pady=(9,0))
value_grid(overview_detail,(
    ("载重等级",heavy_load_var),
    ("过载状态",overload_var),
    ("双向距离",distance_pair_inline_var),
    ("循迹置信度",track_confidence_var),
    ("装卸状态",load_state_var),
    ("载荷安全故障",weight_timeout_var),
),columns=6)

body=ttk.Frame(overview_page)
body.pack(fill=tk.BOTH,expand=True,pady=(6,0))

safety_box=ttk.LabelFrame(body,text="关键安全状态",padding=12)
safety_box.pack(side=tk.LEFT,fill=tk.Y,padx=(0,8))
value_row(safety_box,0,"瓦斯报警",gas_var)
value_row(safety_box,1,"过载状态",overload_var)
value_row(safety_box,2,"循迹丢失",track_lose_var)
value_row(safety_box,3,"电机堵转",stall_alarm_var)
value_row(safety_box,4,"载荷安全故障",weight_timeout_var)

fault_box=ttk.LabelFrame(body,text="最近停车记录",padding=12)
fault_box.pack(side=tk.LEFT,fill=tk.BOTH,expand=True)
ttk.Label(fault_box,text="STOP_DETECTING 最近 5 次停车原因（按 最新 → 最早 显示）",style="SubTitle.TLabel").pack(anchor="w",pady=(0,8))
fault_display=tk.Label(
    fault_box,
    textvariable=fault_history_var,
    background="white",
    foreground=TEXT,
    font=("Microsoft YaHei UI",12,"bold"),
    anchor="nw",
    justify=tk.LEFT,
    wraplength=820,
    relief=tk.GROOVE,
    borderwidth=1,
    padx=16,
    pady=16,
)
fault_display.pack(fill=tk.BOTH,expand=True)

# -----------------------------------------------------------------------------
# 2. 蓝牙控制
# -----------------------------------------------------------------------------
control_page=ttk.Frame(notebook,padding=6)
notebook.add(control_page,text="蓝牙控制")
control_page.columnconfigure(0,weight=0)
control_page.columnconfigure(1,weight=1)
control_page.columnconfigure(2,weight=0)
control_page.rowconfigure(0,weight=1)

mode_box=ttk.LabelFrame(control_page,text="控制模式",padding=12)
mode_box.grid(row=0,column=0,sticky="ns",padx=(0,8))
ttk.Button(mode_box,text="蓝牙手动模式（M → S）",command=request_manual_mode,width=23,style="Primary.TButton").pack(fill=tk.X,pady=(4,8),ipady=4)
ttk.Button(mode_box,text="循迹模式（S → T）",command=request_tracking_mode,width=23).pack(fill=tk.X,pady=8,ipady=4)
ttk.Separator(mode_box).pack(fill=tk.X,pady=10)
ttk.Label(mode_box,text="当前 UI 请求模式").pack(anchor="w")
ttk.Label(mode_box,textvariable=mode_var,style="Value.TLabel",wraplength=220,justify=tk.LEFT).pack(anchor="w",pady=(5,10))

move_box=ttk.LabelFrame(control_page,text="蓝牙手动方向控制",padding=15)
move_box.grid(row=0,column=1,sticky="nsew",padx=(0,8))

pad=ttk.Frame(move_box)
pad.pack(expand=True,padx=35,pady=(20,12))
for i in range(3):
    pad.rowconfigure(i,weight=1)
    pad.columnconfigure(i,weight=1)

ttk.Button(pad,text="↑\n前进",command=lambda:send_movement("前进"),style="Direction.TButton",width=11).grid(row=0,column=1,padx=6,pady=6,ipadx=8,ipady=13,sticky="nsew")
ttk.Button(pad,text="←\n左转",command=lambda:send_movement("左转"),style="Direction.TButton",width=11).grid(row=1,column=0,padx=6,pady=6,ipadx=8,ipady=13,sticky="nsew")
ttk.Button(pad,text="停止",command=lambda:send_movement("停止"),style="Stop.TButton",width=11).grid(row=1,column=1,padx=6,pady=6,ipadx=8,ipady=13,sticky="nsew")
ttk.Button(pad,text="→\n右转",command=lambda:send_movement("右转"),style="Direction.TButton",width=11).grid(row=1,column=2,padx=6,pady=6,ipadx=8,ipady=13,sticky="nsew")
ttk.Button(pad,text="↓\n后退",command=lambda:send_movement("后退"),style="Direction.TButton",width=11).grid(row=2,column=1,padx=6,pady=6,ipadx=8,ipady=13,sticky="nsew")

maintenance_box=ttk.LabelFrame(move_box,text="故障清除与称重校准",padding=8)
maintenance_box.pack(fill=tk.X,padx=18,pady=(0,10))
for column in range(3):
    maintenance_box.columnconfigure(column,weight=1)
ttk.Button(maintenance_box,text="清除载荷故障 [C]",command=clear_load_fault).grid(row=0,column=0,sticky="ew",padx=5,pady=4,ipady=4)
ttk.Button(maintenance_box,text="清除瓦斯报警 [E]",command=clear_gas_alarm).grid(row=0,column=1,sticky="ew",padx=5,pady=4,ipady=4)
ttk.Button(maintenance_box,text="当前重量设为零点 [G]",command=tare_zero).grid(row=0,column=2,sticky="ew",padx=5,pady=4,ipady=4)

state_box=ttk.LabelFrame(control_page,text="运行状态",padding=12)
state_box.grid(row=0,column=2,sticky="ns")
value_row(state_box,0,"车辆状态",car_state_var)
value_row(state_box,1,"控制模式",mode_var)
value_row(state_box,2,"左轮速度",left_speed_var)
value_row(state_box,3,"右轮速度",right_speed_var)
value_row(state_box,4,"瓦斯状态",gas_var)
value_row(state_box,5,"载重等级",heavy_load_var)
value_row(state_box,6,"过载状态",overload_var)
value_row(state_box,7,"堵转状态",stall_state_var)

# -----------------------------------------------------------------------------
# 3. 瓦斯安全
# -----------------------------------------------------------------------------
gas_page=ttk.Frame(notebook,padding=6)
notebook.add(gas_page,text="瓦斯安全")
gas_page.rowconfigure(0,weight=1)
gas_page.columnconfigure(0,weight=1)

gas_state=ttk.LabelFrame(gas_page,text="瓦斯传感器状态",padding=12)
gas_state.grid(row=0,column=0,sticky="nsew")
gas_state.columnconfigure(0,weight=3)
gas_state.columnconfigure(1,weight=5)
gas_state.rowconfigure(0,weight=1)

# 左侧：大尺寸瓦斯状态指示区，占满纵向空间
gas_visual=tk.Frame(gas_state,background="#edf2f5",highlightbackground="#9ba7b1",highlightthickness=1)
gas_visual.grid(row=0,column=0,sticky="nsew",padx=(0,8),pady=0)
gas_visual.rowconfigure(1,weight=1)
gas_visual.columnconfigure(0,weight=1)
tk.Label(gas_visual,text="MQ-4  GAS SAFETY",background="#dfe5ea",foreground="#2a3d4c",font=("Consolas",11,"bold"),pady=8).grid(row=0,column=0,sticky="ew")
gas_card=tk.Label(gas_visual,textvariable=gas_var,background="#e1e7ec",foreground="#123f60",font=("Microsoft YaHei UI",32,"bold"),relief=tk.FLAT,borderwidth=0)
gas_card.grid(row=1,column=0,sticky="nsew",padx=12,pady=12)
tk.Label(gas_visual,text="瓦斯数字报警输入状态",background="#edf2f5",foreground="#6b7782",font=("Microsoft YaHei UI",10,"bold"),pady=8).grid(row=2,column=0,sticky="ew")

# 右侧：运行信息与人工确认区，纵向展开以填充页面
gas_info=tk.Frame(gas_state,background=BG)
gas_info.grid(row=0,column=1,sticky="nsew",padx=(8,0))
gas_info.columnconfigure(0,weight=1)
gas_info.rowconfigure(0,weight=1)
gas_info.rowconfigure(1,weight=1)
gas_info.rowconfigure(2,weight=2)
gas_info.rowconfigure(3,weight=1)

def gas_status_panel(parent,row,title,variable):
    panel=tk.Frame(parent,background="#f7f9fa",highlightbackground="#aab5be",highlightthickness=1)
    panel.grid(row=row,column=0,sticky="nsew",pady=(0 if row==0 else 5,5))
    tk.Label(panel,text=title,background="#dfe5ea",foreground="#445563",font=("Microsoft YaHei UI",10,"bold"),anchor="w",padx=12,pady=6).pack(fill=tk.X)
    tk.Label(panel,textvariable=variable,background="#f7f9fa",foreground="#20313e",font=("Microsoft YaHei UI",18,"bold"),anchor="w",padx=14,pady=12,wraplength=760,justify=tk.LEFT).pack(fill=tk.BOTH,expand=True)
    return panel

gas_status_panel(gas_info,0,"瓦斯状态",gas_var)
gas_status_panel(gas_info,1,"车辆状态",car_state_var)
gas_status_panel(gas_info,2,"最近停车原因",fault_var)

gas_action=tk.Frame(gas_info,background="#e7edf1",highlightbackground="#aab5be",highlightthickness=1)
gas_action.grid(row=3,column=0,sticky="nsew",pady=(5,0))
ttk.Button(gas_action,text="确认风险已消除并清除瓦斯报警 [E]",command=clear_gas_alarm).pack(side=tk.LEFT,padx=14,pady=14,ipadx=8,ipady=7)

# -----------------------------------------------------------------------------
# 4. 载荷监测
# -----------------------------------------------------------------------------
weight_page=ttk.Frame(notebook,padding=6)
notebook.add(weight_page,text="载荷监测")
weight_info=ttk.LabelFrame(weight_page,text="HX711 载荷状态",padding=10)
weight_info.pack(fill=tk.X)
value_grid(weight_info,(
    ("当前重量",weight_var),
    ("载重等级",heavy_load_var),
    ("过载状态",overload_var),
    ("装卸事件",load_state_var),
    ("装载转换",load_switch_var),
    ("重量变化量",weight_change_var),
    ("载荷安全故障",weight_timeout_var),
),columns=4)
weight_action=ttk.LabelFrame(weight_page,text="称重操作",padding=10)
weight_action.pack(fill=tk.X,pady=(10,0))
ttk.Button(weight_action,text="清除载荷故障 [C]",command=clear_load_fault).pack(side=tk.LEFT,padx=5,pady=3,ipady=4)
ttk.Button(weight_action,text="将当前重量设置为零点 [G]",command=tare_zero).pack(side=tk.LEFT,padx=5,pady=3,ipady=4)
weight_chart_box,weight_chart=make_chart_box(weight_page,"重量实时曲线",("重量",),"weight_data")
weight_chart_box.pack(fill=tk.BOTH,expand=True,pady=(6,0))

# -----------------------------------------------------------------------------
# 5. 超声波避障
# -----------------------------------------------------------------------------
ultra_page=ttk.Frame(notebook,padding=6)
notebook.add(ultra_page,text="超声波避障")
ultra_info=ttk.LabelFrame(ultra_page,text="双向测距状态",padding=15)
ultra_info.pack(fill=tk.X)
value_grid(ultra_info,(
    ("车辆状态",car_state_var),
    ("前方距离",front_distance_var),
    ("后方距离",rear_distance_var),
    ("载重等级",heavy_load_var),
),columns=4)
ultra_charts=ttk.Frame(ultra_page)
ultra_charts.pack(fill=tk.BOTH,expand=True,pady=(6,0))
ultra_charts.columnconfigure(0,weight=1)
ultra_charts.columnconfigure(1,weight=1)
ultra_charts.rowconfigure(0,weight=1)
front_chart_box,front_chart=make_chart_box(ultra_charts,"前方距离",("前方距离",),"mm")
rear_chart_box,rear_chart=make_chart_box(ultra_charts,"后方距离",("后方距离",),"mm")
front_chart_box.grid(row=0,column=0,sticky="nsew",padx=(0,5))
rear_chart_box.grid(row=0,column=1,sticky="nsew",padx=(5,0))

# -----------------------------------------------------------------------------
# 6. 循迹与容错
# -----------------------------------------------------------------------------
tracking_page=ttk.Frame(notebook,padding=6)
notebook.add(tracking_page,text="循迹与容错")
track_info=ttk.LabelFrame(tracking_page,text="循迹状态",padding=10)
track_info.pack(fill=tk.X)
value_grid(track_info,(
    ("控制模式",track_state_var),
    ("轨迹置信度",track_confidence_var),
    ("循迹丢失",track_lose_var),
    ("故障通道",sensor_fault_var),
),columns=4)
track_sensor_box=ttk.LabelFrame(tracking_page,text="8 路灰度传感器 I/O 状态",padding=8)
track_sensor_box.pack(fill=tk.X,pady=(6,0))
sensor_header=tk.Frame(track_sensor_box,background="#dfe5ea",highlightbackground="#a1adb7",highlightthickness=1)
sensor_header.pack(fill=tk.X,pady=(0,6))
tk.Label(sensor_header,text="FRONT SENSOR ARRAY",background="#dfe5ea",foreground="#2a3d4c",font=("Consolas",9,"bold")).pack(side=tk.LEFT,padx=8,pady=4)
tk.Label(sensor_header,text="车头方向 ↑",background="#dfe5ea",foreground="#536573",font=("Microsoft YaHei UI",9,"bold")).pack(side=tk.RIGHT,padx=8,pady=4)
track_sensor_panel=tk.Frame(track_sensor_box,background=BG)
track_sensor_panel.pack(fill=tk.X)
for column,_item in enumerate(SENSOR_VISUAL_ORDER):
    track_sensor_panel.columnconfigure(column,weight=1,uniform="sensor_fault")
for column,(name,bit) in enumerate(SENSOR_VISUAL_ORDER):
    indicator=SensorChannelIndicator(track_sensor_panel,name,bit)
    indicator.grid(row=0,column=column,sticky="nsew",padx=3,pady=2)
    sensor_fault_widgets[name]=indicator
axis=tk.Frame(track_sensor_box,background="#8795a1",height=2)
axis.pack(fill=tk.X,padx=14,pady=(6,4))

# 与上方 8 路传感器卡片逐列对齐的物理位置标签
sensor_axis_labels=tk.Frame(track_sensor_box,background=BG)
sensor_axis_labels.pack(fill=tk.X,padx=0)
for column,_item in enumerate(SENSOR_VISUAL_ORDER):
    sensor_axis_labels.columnconfigure(column,weight=1,uniform="sensor_fault_axis")
for column,(name,_bit) in enumerate(SENSOR_VISUAL_ORDER):
    label=tk.Label(
        sensor_axis_labels,
        text=name,
        background=BG,
        foreground="#34495a",
        font=("Microsoft YaHei UI",11,"bold"),
        anchor="center",
    )
    label.grid(row=0,column=column,sticky="ew",padx=3,pady=(1,2))

# 左右阵列之间增加视觉分界，便于一眼区分左/右四路
center_mark=tk.Label(track_sensor_box,text="左侧  ←────────   车体中心   ────────→  右侧",background=BG,foreground="#71808d",font=("Microsoft YaHei UI",9,"bold"))
center_mark.pack(anchor="center",pady=(1,0))
ttk.Label(track_sensor_box,text="状态灯：绿色正常 / 红色故障 / 灰色未判定",style="SubTitle.TLabel").pack(anchor="w",pady=(3,0))
update_sensor_fault_visual(None)
track_chart_box,track_chart=make_chart_box(tracking_page,"轨迹置信度实时曲线",("置信度",),"0~15",fixed_range=(-0.5,15.5))
track_chart_box.pack(fill=tk.BOTH,expand=True,pady=(6,0))

# -----------------------------------------------------------------------------
# 7. 姿态与振动
# -----------------------------------------------------------------------------
mpu_page=ttk.Frame(notebook,padding=6)
notebook.add(mpu_page,text="姿态与振动")
mpu_info=ttk.LabelFrame(mpu_page,text="MPU6050 数据",padding=10)
mpu_info.pack(fill=tk.X)
value_grid(mpu_info,(
    ("航向误差等级",mpu_error_var),
    ("航向纠偏方向",mpu_dir_var),
    ("路面粗糙度等级",mpu_z_error_var),
    ("强冲击状态",mpu_z_dir_var),
    ("Z轴瞬时振动幅值",vibration_data_var),
),columns=3)
mpu_charts=ttk.Frame(mpu_page)
mpu_charts.pack(fill=tk.BOTH,expand=True,pady=(6,0))
mpu_charts.columnconfigure(0,weight=1)
mpu_charts.columnconfigure(1,weight=1)
mpu_charts.rowconfigure(0,weight=1)
mpu_charts.rowconfigure(1,weight=1)
mpu_main_box,mpu_main_chart=make_chart_box(mpu_charts,"航向误差等级",("航向误差等级",),"等级",fixed_range=(-15.5,15.5),height=220)
mpu_z_box,mpu_z_chart=make_chart_box(mpu_charts,"路面粗糙度等级",("粗糙度",),"等级",fixed_range=(0,15.5),height=220)
vibration_box,vibration_chart=make_chart_box(mpu_charts,"Z轴瞬时振动幅值",("振动幅值",),"vibration_data",height=220)
mpu_main_box.grid(row=0,column=0,sticky="nsew",padx=(0,5),pady=(0,5))
mpu_z_box.grid(row=0,column=1,sticky="nsew",padx=(5,0),pady=(0,5))
vibration_box.grid(row=1,column=0,columnspan=2,sticky="nsew",pady=(5,0))

# -----------------------------------------------------------------------------
# 8. 电机与编码器
# -----------------------------------------------------------------------------
motor_page=ttk.Frame(notebook,padding=6)
notebook.add(motor_page,text="电机与编码器")
motor_info=ttk.LabelFrame(motor_page,text="轮速与堵转状态",padding=10)
motor_info.pack(fill=tk.X)
value_grid(motor_info,(
    ("左轮速度",left_speed_var),
    ("右轮速度",right_speed_var),
    ("堵转状态",stall_state_var),
    ("堵转报警",stall_alarm_var),
),columns=2)
motor_charts=ttk.Frame(motor_page)
motor_charts.pack(fill=tk.BOTH,expand=True,pady=(6,0))
motor_charts.columnconfigure(0,weight=1)
motor_charts.columnconfigure(1,weight=1)
motor_charts.rowconfigure(0,weight=1)
left_chart_box,left_chart=make_chart_box(motor_charts,"左轮编码器",("目标速度","实际速度"),"计数/采样窗")
right_chart_box,right_chart=make_chart_box(motor_charts,"右轮编码器",("目标速度","实际速度"),"计数/采样窗")
left_chart_box.grid(row=0,column=0,sticky="nsew",padx=(0,5))
right_chart_box.grid(row=0,column=1,sticky="nsew",padx=(5,0))

# -----------------------------------------------------------------------------
# 9. 停车记录
# -----------------------------------------------------------------------------
fault_page=ttk.Frame(notebook,padding=6)
notebook.add(fault_page,text="停车记录")
fault_panel=ttk.LabelFrame(fault_page,text="STOP_DETECTING 最近 5 次停车原因（按 最新 → 最早 显示）",padding=12)
fault_panel.pack(fill=tk.BOTH,expand=True)
fault_large=tk.Label(fault_panel,textvariable=fault_history_var,background="white",foreground=TEXT,font=("Microsoft YaHei UI",18,"bold"),anchor="nw",justify=tk.LEFT,wraplength=1180,relief=tk.GROOVE,borderwidth=1,padx=18,pady=18)
fault_large.pack(fill=tk.BOTH,expand=True)
ttk.Label(fault_page,text="停车码文本映射完全沿用原程序 FAULT_TEXT，不改变任何故障含义。",style="SubTitle.TLabel").pack(fill=tk.X,pady=(8,0))

# -----------------------------------------------------------------------------
# 10. 通信调试
# -----------------------------------------------------------------------------
debug_page=ttk.Frame(notebook,padding=6)
notebook.add(debug_page,text="通信调试")

debug_status=ttk.LabelFrame(debug_page,text="通信状态",padding=10)
debug_status.pack(fill=tk.X)
value_grid(debug_status,(
    ("连接状态",status_var),
    ("控制模式",mode_var),
    ("帧长度",frame_length_var),
    ("波特率",baud_rate_var),
),columns=4)

debug_protocol=ttk.LabelFrame(debug_page,text="44 字节协议字段",padding=12)
debug_protocol.pack(fill=tk.BOTH,expand=True,pady=(10,0))
protocol_text=(
    "[00] AA    [01] car_state    [02] heavy_load / overload / gas\n"
    "[03:05] weight    [06:10] fault_0 ~ fault_4\n"
    "[11:14] front/rear distance    [15:22] actual/target wheel speed\n"
    "[23:26] MPU data    [27:30] tracking status\n"
    "[31:37] load event / weight change / timeout / overload copy\n"
    "[38:42] vibration / stall state / stall alarm    [43] 55"
)
ttk.Label(debug_protocol,text=protocol_text,font=("Consolas",10),justify=tk.LEFT).pack(anchor="nw",pady=(4,12))
ttk.Separator(debug_protocol).pack(fill=tk.X,pady=6)
ttk.Label(debug_protocol,text="最近通信消息",style="SubTitle.TLabel").pack(anchor="w",pady=(6,4))
ttk.Label(debug_protocol,textvariable=message_var,style="Value.TLabel",wraplength=1150,justify=tk.LEFT).pack(anchor="w")

# =============================================================================
# 底部消息栏
# =============================================================================
message_frame=tk.Frame(root,background="#182b39",height=32)
message_frame.pack(fill=tk.X)
message_frame.pack_propagate(False)
tk.Label(message_frame,text="系统消息",background="#182b39",foreground="#92a7b6",font=("Microsoft YaHei UI",9,"bold")).pack(side=tk.LEFT,padx=(14,8))
tk.Label(message_frame,textvariable=message_var,background="#182b39",foreground="#eef4f8",font=("Microsoft YaHei UI",9)).pack(side=tk.LEFT,fill=tk.X,expand=True)
tk.Label(message_frame,text="44B / 115200 baud",background="#182b39",foreground="#92a7b6",font=("Consolas",8)).pack(side=tk.RIGHT,padx=14)

def _refresh_gas_card(*_args):
    value=gas_var.get()
    if value=="报警":
        gas_card.configure(background="#f4d9d9",foreground="#943c3c")
    elif value=="正常":
        gas_card.configure(background="#dfeee6",foreground="#286247")
    else:
        gas_card.configure(background="#e8eef4",foreground=TEXT)

gas_var.trace_add("write",_refresh_gas_card)
_refresh_gas_card()


# =============================================================================
# 第二份核心 update_display 的兼容变量
# 这些变量仅用于让上面未改动的核心函数继续工作；界面统一使用第一份变量名显示。
# =============================================================================
load_var = heavy_load_var
track_lost_var = track_lose_var
control_mode_var = track_state_var
vibration_var = vibration_data_var

# 第二份核心函数需要两个独立目标速度 StringVar；它们不作为额外 UI 字段展示。
left_target_var = tk.StringVar(value="--")
right_target_var = tk.StringVar(value="--")


def update_display(packet):
    """
    外层显示包装：
    先调用第二份原始 update_display，再补齐第一份工业版的全部字段与曲线。
    """
    global frame_count, last_packet_monotonic

    _core_update_display(packet)
    normalized = _normalize_to_first_packet(packet)

    car_state_var.set(normalized["车辆状态"])
    gas_var.set(normalized["瓦斯状态"])
    overload_var.set(normalized["过载状态"])
    heavy_load_var.set(normalized["重载状态"])
    weight_var.set(str(normalized["重量"]))
    fault_var.set(normalized["停车原因"])
    fault_history_var.set(format_fault_history(normalized["停车原因"]))

    front_text = str(normalized["前方距离"])
    rear_text = str(normalized["后方距离"])
    front_distance_var.set(front_text)
    rear_distance_var.set(rear_text)
    distance_card_text, distance_inline_text = format_distance_pair(front_text, rear_text)
    distance_pair_var.set(distance_card_text)
    distance_pair_inline_var.set(distance_inline_text)

    left_speed_var.set(normalized["左轮速度"])
    right_speed_var.set(normalized["右轮速度"])
    mpu_error_var.set(str(normalized["MPU误差"]))
    mpu_dir_var.set(normalized["MPU方向"])
    mpu_z_dir_var.set("强冲击" if normalized["MPU_Z方向"] == "1" else "正常")
    mpu_z_error_var.set(str(normalized["MPU_Z误差"]))
    track_lose_var.set(normalized["循迹丢失"])
    sensor_fault_var.set(format_sensor_fault(normalized["传感器故障"]))
    update_sensor_fault_visual(normalized["传感器故障"])
    track_confidence_var.set(str(normalized["循迹置信度"]))
    track_state_var.set(normalized["循迹状态"])
    load_state_var.set(normalized["装卸状态"])
    load_switch_var.set(normalized["装载转换"])
    weight_change_var.set(str(normalized["重量变化"]))
    weight_timeout_var.set(normalized["重量超时"])
    overload_copy_var.set(normalized["过载副本"])
    vibration_data_var.set(str(normalized["振动数据"]))
    stall_state_var.set(normalized["堵转状态"])
    stall_alarm_var.set(normalized["堵转报警"])

    if not curves_paused:
        weight_chart.append({"重量": normalized["重量"]})

        front_distance = normalized["前方距离"]
        rear_distance = normalized["后方距离"]
        front_chart.append({"前方距离": None if front_distance == 0 else front_distance})
        rear_chart.append({"后方距离": None if rear_distance == 0 else rear_distance})

        mpu_main_chart.append({"航向误差等级": mpu_signed_for_chart(normalized)})
        mpu_z_chart.append({"粗糙度": normalized["MPU_Z误差"]})
        vibration_chart.append({"振动幅值": normalized["振动数据"]})
        track_chart.append({"置信度": normalized["循迹置信度"]})

        left_actual, left_target = speed_pair_for_chart(normalized["左轮速度"])
        right_actual, right_target = speed_pair_for_chart(normalized["右轮速度"])
        left_chart.append({"目标速度": left_target, "实际速度": left_actual})
        right_chart.append({"目标速度": right_target, "实际速度": right_actual})

    frame_count += 1
    last_packet_monotonic = time.monotonic()
    message_var.set(f"已接收 {frame_count} 帧")


def _monitor_telemetry_health():
    """只做提示，不改协议、不自动发运动命令。"""
    if _serial_is_open() and last_packet_monotonic is not None:
        age = time.monotonic() - last_packet_monotonic
        if age > 2.0:
            status_var.set("已连接 / 数据超时")
        else:
            status_var.set("已连接 " + port_entry.get().strip())
    root.after(500, _monitor_telemetry_health)


def close_program():
    # 安全冗余：关闭窗口也走“先停止、再断开”的包装。
    try:
        safe_disconnect_serial()
    finally:
        root.destroy()


root.protocol("WM_DELETE_WINDOW", close_program)
root.after(DELAY, read_serial)
root.after(500, _monitor_telemetry_health)
root.mainloop()
