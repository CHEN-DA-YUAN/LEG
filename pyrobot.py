#!/usr/bin/env python3
"""
yiersan
ik_parallel_leg_interactive_record.py with WiFi control - Fixed Version

Interactive parallel-leg simulator with ESP32 WiFi synchronization:
 - click/drag target (clamped to feasible region; foot below y<=0)
 - recording (Start/Stop) during drag or playback
 - Export trajectory to CSV
 - Playback recorded trajectory in-sim
 - WiFi同步控制ESP32执行实时运动
# CHEN-DA-YUAN 2025-09-07到此一改  gg
Run:
  pip install numpy matplotlib pandas websockets
  python pyrobot.py
"""

import math, os, sys, time, asyncio, threading
import websockets, json
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import pandas as pd
import matplotlib.font_manager as fm

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'SimSun', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# ---------- CONFIG ----------
L1_mm = 350.0
L2_mm = 650.0
BASE_SEP_mm = 350.0

SERVO_NEUTRAL_DEG = 90.0
NEUTRAL_ANGLE_FRONT_DEG = 0.0
NEUTRAL_ANGLE_REAR_DEG  = 180.0

GRID_RES_MM = 6.0
MARGIN_MM = 80.0
OUT_CSV = "trajectory_export.csv"
FPS_PLAYBACK = 30
RECORD_MIN_DT_MS = 40

# WiFi配置
ESP32_IP = "192.168.218.145"  # 替换为你的ESP32 IP地址
ESP32_PORT = 81
WS_URI = f"ws://{ESP32_IP}:{ESP32_PORT}"

EPS = 1e-9

# ---------- IK helpers ----------
def normalize_deg(a: float) -> float:
    v = ((a + 180.0) % 360.0)
    if v <= 0: v += 360.0
    return v - 180.0

def ik_2link_branch_mm(bx: float, by: float, x: float, y: float, elbowSign: int):
    dx = x - bx; dy = y - by
    r2 = dx*dx + dy*dy; r = math.sqrt(r2)
    if r > (L1_mm + L2_mm) + EPS: raise ValueError("too far")
    if r < abs(L1_mm - L2_mm) - EPS: raise ValueError("too close")
    c2 = (r2 - L1_mm*L1_mm - L2_mm*L2_mm) / (2.0 * L1_mm * L2_mm)
    c2 = max(-1.0, min(1.0, c2))
    s2 = math.sqrt(max(0.0, 1.0 - c2*c2))
    if elbowSign < 0: s2 = -s2
    theta2 = math.atan2(s2, c2)
    k1 = L1_mm + L2_mm * c2
    k2 = L2_mm * s2
    theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)
    return math.degrees(theta1), math.degrees(theta2)

def baseDeg_to_servoDeg(baseDeg: float, neutralBaseDeg: float) -> float:
    rel = normalize_deg(baseDeg - neutralBaseDeg)
    # 修复舵机方向：反转角度映射以匹配实际运动方向
    return SERVO_NEUTRAL_DEG - rel

def find_servo_solution_for_point(x_mm: float, y_mm: float):
    bxF = BASE_SEP_mm * 0.5; byF = 0.0
    bxR = -BASE_SEP_mm * 0.5; byR = 0.0
    def reachable_from(bx,by):
        r = math.hypot(x_mm - bx, y_mm - by)
        return (abs(L1_mm - L2_mm) - 1e-6) <= r <= (L1_mm + L2_mm + 1e-6)
    if not (reachable_from(bxF,byF) and reachable_from(bxR,byR)):
        raise ValueError("physically unreachable")
    for eF in (1, -1):
        for eR in (1, -1):
            try:
                baseF_deg, elbowF_deg = ik_2link_branch_mm(bxF, byF, x_mm, y_mm, eF)
                baseR_deg, elbowR_deg = ik_2link_branch_mm(bxR, byR, x_mm, y_mm, eR)
            except ValueError:
                continue
            sF = baseDeg_to_servoDeg(baseF_deg, NEUTRAL_ANGLE_FRONT_DEG)
            sR = baseDeg_to_servoDeg(baseR_deg, NEUTRAL_ANGLE_REAR_DEG)
            if 0.5 < sF < 179.5 and 0.5 < sR < 179.5:
                return {
                    'servoF_deg': float(sF), 'servoR_deg': float(sR),
                    'baseF_deg': float(baseF_deg), 'elbowF_deg': float(elbowF_deg),
                    'baseR_deg': float(baseR_deg), 'elbowR_deg': float(elbowR_deg),
                    'branchF': int(eF), 'branchR': int(eR)
                }
    raise ValueError("no servo-valid branch solution")

def compute_forward_point_for_servo90():
    baseF_deg = 0.0; baseR_deg = 180.0
    bxF = BASE_SEP_mm * 0.5; byF = 0.0
    bxR = -BASE_SEP_mm * 0.5; byR = 0.0
    exF = bxF + L1_mm * math.cos(math.radians(baseF_deg))
    eyF = byF + L1_mm * math.sin(math.radians(baseF_deg))
    exR = bxR + L1_mm * math.cos(math.radians(baseR_deg))
    eyR = byR + L1_mm * math.sin(math.radians(baseR_deg))
    dx = exR - exF; dy = eyR - eyF; d = math.hypot(dx,dy)
    if d < 1e-9: return (0.0, - (L1_mm + L2_mm)/2.0)
    if d > 2*L2_mm + 1e-6 or d < 1e-6: return (0.0, -min(L1_mm + L2_mm - 10.0, L1_mm + L2_mm)/2.0)
    a = d/2.0
    h = math.sqrt(max(0.0, L2_mm*L2_mm - a*a))
    xm = exF + a * dx / d; ym = eyF + a * dy / d
    rx = -dy * (h / d); ry = dx * (h / d)
    xi1, yi1 = xm + rx, ym + ry
    xi2, yi2 = xm - rx, ym - ry
    cand = [(xi1, yi1), (xi2, yi2)]
    below = [p for p in cand if p[1] <= 0]
    return min(below, key=lambda t: t[1]) if below else min(cand, key=lambda t: t[1])

# ---------- feasible grid utilities ----------
def precompute_feasible_grid(xmin, xmax, ymin, ymax, res_mm=GRID_RES_MM):
    xs = np.arange(xmin, xmax + 1e-9, res_mm)
    ys = np.arange(ymin, ymax + 1e-9, res_mm)
    pts = []
    infos = []
    for y in ys:
        if y > 0:  # 限制在y轴负半平面
            break
        for x in xs:
            try:
                info = find_servo_solution_for_point(x, y)
                pts.append((x,y))
                infos.append(info)
            except Exception:
                continue
    arr = np.array(pts) if pts else np.zeros((0,2))
    return arr, infos, xs, ys

# ---------- WiFi通信类 ----------
class ESP32Controller:
    def __init__(self):
        self.uri = WS_URI
        self.websocket = None
        self.connected = False
        self.last_position = None
        
    def run_async_task(self, coro):
        """运行异步任务"""
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                # 如果事件循环已在运行，创建新的事件循环
                new_loop = asyncio.new_event_loop()
                asyncio.set_event_loop(new_loop)
                try:
                    return new_loop.run_until_complete(coro)
                finally:
                    new_loop.close()
                    asyncio.set_event_loop(loop)
            else:
                return loop.run_until_complete(coro)
        except Exception as e:
            print(f"运行异步任务失败: {e}")
            return None
    
    async def _connect_async(self):
        """异步连接到ESP32"""
        try:
            self.websocket = await websockets.connect(self.uri)
            self.connected = True
            print("已连接到ESP32")
            return True
        except Exception as e:
            print(f"连接ESP32失败: {e}")
            self.connected = False
            return False
    
    def connect(self):
        """连接到ESP32（同步版本）"""
        return self.run_async_task(self._connect_async())
    
    async def _disconnect_async(self):
        """异步断开连接"""
        if self.websocket:
            await self.websocket.close()
            self.connected = False
            print("已断开ESP32连接")
    
    def disconnect(self):
        """断开连接（同步版本）"""
        if self.websocket:
            return self.run_async_task(self._disconnect_async())
    
    async def _send_move_command_async(self, x: float, y: float, speed: float = 90.0):
        """异步发送移动指令"""
        if not self.connected or not self.websocket:
            return False
            
        try:
            command = {
                "type": "move_to",
                "x": x,
                "y": y,
                "speed": speed
            }
            await self.websocket.send(json.dumps(command))
            
            # 等待响应
            response = await asyncio.wait_for(self.websocket.recv(), timeout=2.0)
            data = json.loads(response)
            return data.get("success", False)
        except Exception as e:
            print(f"发送移动指令失败: {e}")
            return False
    
    def send_move_command(self, x: float, y: float, speed: float = 90.0):
        """发送移动指令（同步版本）"""
        return self.run_async_task(self._send_move_command_async(x, y, speed))

# ---------- interactive app ----------
class RecorderApp:
    def __init__(self):
        maxreach = L1_mm + L2_mm
        xl = - (maxreach + BASE_SEP_mm*0.5) - MARGIN_MM
        xr =   (maxreach + BASE_SEP_mm*0.5) + MARGIN_MM
        yb = - (maxreach) - MARGIN_MM
        yt =   50.0  # 限制在y轴负半平面，只允许小范围向上
        print("Precomputing feasible grid...")
        t0 = time.time()
        self.coords, self.infos, _, _ = precompute_feasible_grid(xl, xr, yb, yt, res_mm=GRID_RES_MM)
        print("Precompute done, feasible count:", len(self.coords), "time:", time.time()-t0)
        init = compute_forward_point_for_servo90()
        self.target = self.find_nearest(init)
        self.target_desired = self.target
        
        # WiFi控制器
        self.esp32 = ESP32Controller()
        self.wifi_enabled = False
        self.auto_sync = False
        
        # 录制和播放
        self.recording = False
        self.trajectory = []
        self.last_record_ms = 0
        self.playing = False
        self.play_index = 0
        
        self.build_ui(xl,xr,yb,yt)
        self.update_plot()
    
    def clamp_y(self, x, y):
        # 移除y轴限制，允许上下运动
        return x, y

    def find_nearest(self, pt):
        if len(self.coords)==0: return pt
        dx = self.coords[:,0] - pt[0]
        dy = self.coords[:,1] - pt[1]
        idx = int(np.argmin(dx*dx + dy*dy))
        return (float(self.coords[idx,0]), float(self.coords[idx,1]))

    def build_ui(self, xl, xr, yb, yt):
        self.fig, (self.ax, self.ax_info) = plt.subplots(ncols=2, figsize=(15,7), 
                                                         gridspec_kw={'width_ratios':[3,1]})
        plt.subplots_adjust(left=0.05, right=0.98, top=0.96, bottom=0.15)
        self.ax.set_aspect('equal','box')
        self.ax.set_xlim(xl, xr); self.ax.set_ylim(yb, yt)
        self.ax.set_title('ESP32 WiFi同步控制 (拖动目标移动并联足)')
        self.ax.set_xlabel('x (mm)'); self.ax.set_ylabel('y (mm)')
        
        # 添加坐标系说明
        self.ax.text(0.02, 0.98, '↑ +Y\n→ +X', transform=self.ax.transAxes, 
                    va='top', ha='left', fontsize=10, color='gray')
        
        # feasible points
        if len(self.coords)>0:
            self.ax.scatter(self.coords[:,0], self.coords[:,1], s=6, c='limegreen', alpha=0.25)
        
        # bases
        self.bxF = BASE_SEP_mm*0.5; self.byF = 0.0
        self.bxR = -BASE_SEP_mm*0.5; self.byR = 0.0
        self.ax.plot([self.bxR,self.bxF],[self.byR,self.byF], color='k')
        self.ax.scatter([self.bxR,self.bxF],[self.byR,self.byF], s=50, c='k')
        self.origin = self.ax.scatter([0],[0], c='gray', s=25)
        
        # target visuals
        self.desired_scatter = self.ax.scatter([self.target_desired[0]],[self.target_desired[1]], 
                                               s=48, c='red', alpha=0.35)
        self.target_scatter = self.ax.scatter([self.target[0]],[self.target[1]], s=80, c='tab:blue')
        self.front_line, = self.ax.plot([], [], linewidth=3, marker='o', color='tab:blue', zorder=6)
        self.rear_line,  = self.ax.plot([], [], linewidth=3, marker='o', color='tab:orange', zorder=6)
        
        # info panel
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.02,0.98,'',va='top',ha='left', family='monospace')
        
        # buttons布局调整
        button_width = 0.11
        button_height = 0.045
        button_spacing = 0.12
        
        # 第一行按钮
        y_pos = 0.12
        ax_wifi = plt.axes([0.02, y_pos, button_width, button_height])
        ax_sync = plt.axes([0.02 + button_spacing, y_pos, button_width, button_height])
        ax_start = plt.axes([0.02 + 2*button_spacing, y_pos, button_width, button_height])
        ax_stop = plt.axes([0.02 + 3*button_spacing, y_pos, button_width, button_height])
        
        # 第二行按钮
        y_pos2 = 0.06
        ax_export = plt.axes([0.02, y_pos2, button_width, button_height])
        ax_play = plt.axes([0.02 + button_spacing, y_pos2, button_width, button_height])
        ax_clear = plt.axes([0.02 + 2*button_spacing, y_pos2, button_width, button_height])
        
        self.btn_wifi = Button(ax_wifi, '连接ESP32')
        self.btn_sync = Button(ax_sync, '同步开关')
        self.btn_start = Button(ax_start, '开始录制')
        self.btn_stop = Button(ax_stop, '停止录制')
        self.btn_export = Button(ax_export, '导出CSV')
        self.btn_play = Button(ax_play, '播放轨迹')
        self.btn_clear = Button(ax_clear, '清除轨迹')
        
        self.btn_wifi.on_clicked(self.on_wifi_toggle)
        self.btn_sync.on_clicked(self.on_sync_toggle)
        self.btn_start.on_clicked(self.on_start)
        self.btn_stop.on_clicked(self.on_stop)
        self.btn_export.on_clicked(self.on_export)
        self.btn_play.on_clicked(self.on_play)
        self.btn_clear.on_clicked(self.on_clear)
        
        # 事件
        self.dragging=False
        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def on_wifi_toggle(self, event):
        """WiFi连接按钮"""
        if not self.wifi_enabled:
            success = self.esp32.connect()
            if success:
                self.wifi_enabled = True
                self.btn_wifi.label.set_text('已连接')
                print("ESP32连接成功")
            else:
                self.btn_wifi.label.set_text('连接失败')
        else:
            self.esp32.disconnect()
            self.wifi_enabled = False
            self.btn_wifi.label.set_text('连接ESP32')
    
    def on_sync_toggle(self, event):
        """同步开关按钮"""
        self.auto_sync = not self.auto_sync
        self.btn_sync.label.set_text(f'同步: {"开" if self.auto_sync else "关"}')
        print(f"WiFi同步 {'开启' if self.auto_sync else '关闭'}")

    def on_press(self,event):
        if event.inaxes!=self.ax: return
        if event.button==1:
            self.dragging=True
            self.move_target(event.xdata,event.ydata)
    def on_release(self,event):
        if event.button==1:
            self.dragging=False
    def on_motion(self,event):
        if not self.dragging: return
        if event.inaxes!=self.ax: return
        self.move_target(event.xdata,event.ydata)
    def on_key(self,event):
        if event.key=='escape': plt.close(self.fig); sys.exit(0)

    def move_target(self, xd, yd):
        if xd is None or yd is None: return
        # 修复镜像问题：反转x坐标，并限制y <= 0
        if yd > 0:
            yd = 0  # 强制限制在y轴负半平面
        self.target_desired = (xd, yd)
        clamped = self.find_nearest(self.target_desired)
        self.target = clamped
        self.desired_scatter.set_offsets([self.target_desired])
        self.target_scatter.set_offsets([self.target])
        self.update_plot()
        
        # WiFi同步
        if self.wifi_enabled and self.auto_sync:
            x, y = self.target
            # 镜像修复：反转x坐标发送给实际机器人
            x_mirrored = -x
            success = self.esp32.send_move_command(x_mirrored, y)
            if success:
                print(f"已同步到ESP32: x={x_mirrored:.1f}, y={y:.1f}")
            else:
                print("同步到ESP32失败")
        
        # 录制
        if self.recording:
            tms = int(time.time()*1000)
            if tms - self.last_record_ms >= RECORD_MIN_DT_MS:
                try:
                    info = find_servo_solution_for_point(self.target[0], self.target[1])
                    row = {
                        't_ms': tms,
                        'desired_x_mm': self.target_desired[0],
                        'desired_y_mm': self.target_desired[1],
                        'clamped_x_mm': self.target[0],
                        'clamped_y_mm': self.target[1],
                        'servoF_deg': info['servoF_deg'],
                        'servoR_deg': info['servoR_deg'],
                        'baseF_deg': info['baseF_deg'],
                        'elbowF_deg': info['elbowF_deg'],
                        'baseR_deg': info['baseR_deg'],
                        'elbowR_deg': info['elbowR_deg'],
                        'branchF': info['branchF'],
                        'branchR': info['branchR']
                    }
                except Exception as e:
                    row = {
                        't_ms': tms,
                        'desired_x_mm': self.target_desired[0],
                        'desired_y_mm': self.target_desired[1],
                        'clamped_x_mm': self.target[0],
                        'clamped_y_mm': self.target[1],
                        'error': str(e)
                    }
                self.trajectory.append(row)
                self.last_record_ms = tms

    def on_start(self,event):
        print("Recording started")
        self.recording=True
        self.last_record_ms = 0
    def on_stop(self,event):
        self.recording=False
        print("Recording stopped. Samples:", len(self.trajectory))
    def on_clear(self,event):
        self.trajectory = []
        print("Cleared trajectory")
    def on_export(self,event):
        if not self.trajectory:
            print("No trajectory to export")
            return
        df = pd.DataFrame(self.trajectory)
        df.to_csv(OUT_CSV, index=False)
        print("Exported trajectory to", OUT_CSV)
    
    def on_play(self,event):
        if not self.trajectory:
            print("No trajectory to play")
            return
        
        self.playing = True
        times = [r['t_ms'] for r in self.trajectory]
        seq = []
        for i, r in enumerate(self.trajectory):
            x = r.get('clamped_x_mm', r.get('desired_x_mm', 0.0))
            y = r.get('clamped_y_mm', r.get('desired_y_mm', 0.0))
            if i==0:
                dt = 0
            else:
                dt = max(1, r['t_ms'] - times[i-1])
            seq.append((x,y,dt))
        
        # 本地播放
        for (x,y,dt) in seq:
            if not self.playing: break
            self.target_desired = (x,y)
            self.target = (x,y)
            self.desired_scatter.set_offsets([self.target_desired])
            self.target_scatter.set_offsets([self.target])
            self.update_plot()
            
            # 发送到ESP32
            if self.wifi_enabled:
                self.esp32.send_move_command(x, y, 180.0)  # 快速移动
            
            plt.pause(dt/1000.0)
        
        self.playing = False
        print("Playback end")

    def update_plot(self):
        x,y = self.target
        try:
            info = find_servo_solution_for_point(x,y)
            reachable=True
        except Exception as e:
            info=None; reachable=False
        
        # 更新连杆可视化
        if info:
            bxF = BASE_SEP_mm*0.5; byF = 0.0
            bxR = -BASE_SEP_mm*0.5; byR = 0.0
            
            # 前连杆
            baseF_rad = math.radians(info['baseF_deg'])
            exF = bxF + L1_mm * math.cos(baseF_rad)
            eyF = byF + L1_mm * math.sin(baseF_rad)
            elbowF_rad = baseF_rad + math.radians(info['elbowF_deg'])
            fx = exF + L2_mm * math.cos(elbowF_rad)
            fy = eyF + L2_mm * math.sin(elbowF_rad)
            
            # 后连杆
            baseR_rad = math.radians(info['baseR_deg'])
            exR = bxR + L1_mm * math.cos(baseR_rad)
            eyR = byR + L1_mm * math.sin(baseR_rad)
            elbowR_rad = baseR_rad + math.radians(info['elbowR_deg'])
            rx = exR + L2_mm * math.cos(elbowR_rad)
            ry = eyR + L2_mm * math.sin(elbowR_rad)
            
            self.front_line.set_data([bxF, exF, fx], [byF, eyF, fy])
            self.rear_line.set_data([bxR, exR, rx], [byR, eyR, ry])
        
        # 更新信息面板
        s = f"Target: ({x:.1f}, {y:.1f})\n"
        if info:
            s += f"Servo F: {info['servoF_deg']:.1f}°\n"
            s += f"Servo R: {info['servoR_deg']:.1f}°\n"
            s += f"Reachable: ✓"
        else:
            s += "Reachable: ✗"
        s += f"\nWiFi: {'✓' if self.wifi_enabled else '✗'}"
        s += f"\nSync: {'✓' if self.auto_sync else '✗'}"
        s += f"\nRecording: {'✓' if self.recording else '✗'}"
        s += f"\nSamples: {len(self.trajectory)}"
        self.info_text.set_text(s)
        
        # 强制刷新图形
        self.fig.canvas.draw_idle()

def main():
    print("ESP32 WiFi同步控制程序")
    print(f"连接到: {WS_URI}")
    print("功能说明:")
    print("- 拖动蓝色目标点控制并联足")
    print("- 点击'连接ESP32'建立WiFi连接")
    print("- 开启'同步开关'实现实时同步")
    print("- 支持轨迹录制和播放")
    
    # 确保matplotlib使用合适的后端
    plt.switch_backend('TkAgg')
    
    app = RecorderApp()
    
    # 添加调试信息
    print("界面已启动，请使用鼠标在左侧区域拖拽蓝色目标点")
    print("确保点击在左侧的可视化区域内")
    
    plt.show()

if __name__ == "__main__":
    main()