import sys
import math
import platform as platform_mod
import serial
import serial.tools.list_ports
import pygame
from pygame.locals import *
import time
import numpy as np

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    HAS_OPENGL = True
except ImportError:
    HAS_OPENGL = False

BAUD_RATE = 153600
SCAN_SIZE = 720
MAX_DISTANCE = 12000
INVALID_DISTANCE = 16000
MIN_QUALITY = 10
POINT_FADE_SCANS = 3

BG_COLOR = (15, 15, 20)
GRID_COLOR = (35, 35, 45)
GRID_TEXT_COLOR = (60, 60, 80)
CENTER_COLOR = (255, 60, 60)
POINT_COLOR_FRESH = (0, 255, 100)
POINT_COLOR_OLD = (0, 100, 50)
WALL_COLOR = (0, 200, 80, 180)
SWEEP_COLOR = (0, 255, 100, 30)
TEXT_COLOR = (200, 200, 200)
STATUS_GOOD = (0, 255, 100)
STATUS_BAD = (255, 60, 60)


def find_lidar_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        device = port.device.lower()
        if platform_mod.system() == "Darwin":
            if "usbserial" in device or "cu.usb" in device:
                return port.device
        elif platform_mod.system() == "Windows":
            desc = (port.description or "").lower()
            if "com" in device and ("ftdi" in desc or "usb" in desc or "serial" in desc):
                return port.device
        else:
            if "ttyusb" in device or "ttyacm" in device:
                return port.device
    for port in ports:
        if "usb" in port.device.lower():
            return port.device
    return None


class LidarSerial:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD_RATE, timeout=0)
        self.ser.reset_input_buffer()
        self.buffer = bytearray()
        self.packet_count = 0
        self.points_per_sec = 0
        self._pts_count = 0
        self._pts_time = time.time()
    
    def read(self):
        points = []
        
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                self.buffer.extend(self.ser.read(min(waiting, 8192)))
        except Exception:
            return points
        
        if len(self.buffer) > 30000:
            trim_from = len(self.buffer) - 10000
            found = -1
            for i in range(trim_from, len(self.buffer) - 1):
                if self.buffer[i] == 0xAA and self.buffer[i + 1] == 0x55:
                    found = i
                    break
            if found > 0:
                del self.buffer[:found]
            else:
                self.buffer = self.buffer[-5000:]
        
        while len(self.buffer) >= 10:
            try:
                idx = self.buffer.index(0xAA)
            except ValueError:
                self.buffer.clear()
                break
            
            if idx > 0:
                del self.buffer[:idx]
                continue
            
            if len(self.buffer) < 2 or self.buffer[1] != 0x55:
                del self.buffer[:1]
                continue
            
            if len(self.buffer) < 10:
                break
            
            num_measurements = self.buffer[3]
            if num_measurements > 100 or num_measurements == 0:
                del self.buffer[:2]
                continue
            
            packet_len = 10 + num_measurements * 3
            if len(self.buffer) < packet_len:
                break
            
            pkt = bytes(self.buffer[:packet_len])
            del self.buffer[:packet_len]
            
            start_angle = (pkt[4] | (pkt[5] << 8)) / 100.0
            end_angle = (pkt[6] | (pkt[7] << 8)) / 100.0
            
            if end_angle < start_angle:
                end_angle += 360.0
            
            if num_measurements > 1:
                angle_step = (end_angle - start_angle) / (num_measurements - 1)
            else:
                angle_step = 0
            
            for i in range(num_measurements):
                offset = 10 + i * 3
                if offset + 2 >= len(pkt):
                    break
                
                quality = pkt[offset]
                distance = pkt[offset + 1] | (pkt[offset + 2] << 8)
                angle = (start_angle + i * angle_step) % 360.0
                
                if quality >= MIN_QUALITY and 50 < distance < INVALID_DISTANCE:
                    points.append((angle, distance, quality))
            
            self.packet_count += 1
        
        self._pts_count += len(points)
        now = time.time()
        if now - self._pts_time >= 1.0:
            self.points_per_sec = self._pts_count
            self._pts_count = 0
            self._pts_time = now
        
        return points
    
    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class Lidar3DView:
    WALL_HEIGHT = 200.0
    GROUND_SIZE = 15000.0
    GROUND_GRID_STEP = 1000.0

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.cam_dist = 8000.0
        self.cam_pitch = 35.0
        self.cam_yaw = 45.0
        self.cam_target = [0.0, 0.0, 0.0]
        self._dragging = False
        self._last_mouse = (0, 0)
        self._panning = False

    def init_gl(self, width, height):
        self.width = width
        self.height = height
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glClearColor(0.06, 0.06, 0.08, 1.0)
        self._setup_projection()

    def _setup_projection(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(50, self.width / max(1, self.height), 50, 50000)
        glMatrixMode(GL_MODELVIEW)

    def handle_event(self, event):
        if event.type == MOUSEBUTTONDOWN:
            if event.button == 1:
                self._dragging = True
                self._last_mouse = event.pos
            elif event.button == 3:
                self._panning = True
                self._last_mouse = event.pos
            elif event.button == 4:
                self.cam_dist = max(1000, self.cam_dist - 500)
            elif event.button == 5:
                self.cam_dist = min(20000, self.cam_dist + 500)
        elif event.type == MOUSEBUTTONUP:
            if event.button == 1:
                self._dragging = False
            elif event.button == 3:
                self._panning = False
        elif event.type == MOUSEMOTION:
            if self._dragging:
                dx = event.pos[0] - self._last_mouse[0]
                dy = event.pos[1] - self._last_mouse[1]
                self.cam_yaw += dx * 0.4
                self.cam_pitch = max(5, min(85, self.cam_pitch + dy * 0.4))
                self._last_mouse = event.pos
            elif self._panning:
                dx = event.pos[0] - self._last_mouse[0]
                dy = event.pos[1] - self._last_mouse[1]
                yaw_rad = math.radians(self.cam_yaw)
                self.cam_target[0] -= (math.cos(yaw_rad) * dx + math.sin(yaw_rad) * dy) * 5
                self.cam_target[2] -= (-math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy) * 5
                self._last_mouse = event.pos

    def _set_camera(self):
        glLoadIdentity()
        pitch_rad = math.radians(self.cam_pitch)
        yaw_rad = math.radians(self.cam_yaw)
        cx = self.cam_target[0] + self.cam_dist * math.cos(pitch_rad) * math.cos(yaw_rad)
        cy = self.cam_target[1] + self.cam_dist * math.sin(pitch_rad)
        cz = self.cam_target[2] + self.cam_dist * math.cos(pitch_rad) * math.sin(yaw_rad)
        gluLookAt(cx, cy, cz,
                  self.cam_target[0], self.cam_target[1], self.cam_target[2],
                  0, 1, 0)

    def _draw_ground(self):
        gs = self.GROUND_SIZE
        step = self.GROUND_GRID_STEP
        glBegin(GL_QUADS)
        glColor4f(0.08, 0.08, 0.10, 0.8)
        glVertex3f(-gs, 0, -gs)
        glVertex3f(gs, 0, -gs)
        glVertex3f(gs, 0, gs)
        glVertex3f(-gs, 0, gs)
        glEnd()

        glBegin(GL_LINES)
        glColor4f(0.15, 0.15, 0.20, 0.6)
        v = -gs
        while v <= gs:
            glVertex3f(v, 0.5, -gs)
            glVertex3f(v, 0.5, gs)
            glVertex3f(-gs, 0.5, v)
            glVertex3f(gs, 0.5, v)
            v += step
        glEnd()

        glBegin(GL_LINES)
        for r_m in range(1, 7):
            r = r_m * 1000
            segs = 72
            glColor4f(0.15, 0.2, 0.15, 0.4)
            for i in range(segs):
                a1 = 2 * math.pi * i / segs
                a2 = 2 * math.pi * (i + 1) / segs
                glVertex3f(r * math.cos(a1), 1, r * math.sin(a1))
                glVertex3f(r * math.cos(a2), 1, r * math.sin(a2))
        glEnd()

    def _draw_origin(self):
        glPointSize(8)
        glBegin(GL_POINTS)
        glColor3f(1.0, 0.24, 0.24)
        glVertex3f(0, 2, 0)
        glEnd()

        glBegin(GL_LINES)
        glColor3f(1.0, 0.3, 0.3)
        glVertex3f(0, 0, 0)
        glVertex3f(0, self.WALL_HEIGHT * 1.5, 0)
        glEnd()

    def render(self, scan_data, max_range_m, show_walls):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._set_camera()
        self._draw_ground()
        self._draw_origin()

        max_range_mm = max_range_m * 1000
        screen_points = []

        for i in range(SCAN_SIZE):
            if scan_data[i] is None:
                continue
            distance, quality, age = scan_data[i]
            if distance > max_range_mm:
                continue
            angle_deg = i / 2.0
            rad = math.radians(angle_deg)
            x = distance * math.cos(rad)
            z = distance * math.sin(rad)
            screen_points.append((x, z, age, i, distance))

        if not screen_points:
            return

        glPointSize(4)
        glBegin(GL_POINTS)
        for x, z, age, idx, d in screen_points:
            if age == 0:
                glColor3f(0.0, 1.0, 0.4)
            elif age == 1:
                glColor3f(0.0, 0.86, 0.31)
            else:
                glColor3f(0.0, 0.4, 0.2)
            glVertex3f(x, 2, z)
        glEnd()

        if show_walls and len(screen_points) >= 2:
            sorted_pts = sorted(screen_points, key=lambda p: p[3])
            wall_h = self.WALL_HEIGHT

            for j in range(1, len(sorted_pts)):
                x1, z1, age1, idx1, d1 = sorted_pts[j - 1]
                x2, z2, age2, idx2, d2 = sorted_pts[j]

                angle_gap = idx2 - idx1
                if angle_gap > 10:
                    continue

                dx = x2 - x1
                dz = z2 - z1
                dist_2d = math.sqrt(dx * dx + dz * dz)
                max_gap = max(100, 500 * (1.0 / max(0.1, max_range_m / 6.0)))
                if dist_2d > max_gap:
                    continue

                max_age = max(age1, age2)
                if max_age == 0:
                    r, g, b = 0.0, 0.9, 0.4
                elif max_age == 1:
                    r, g, b = 0.0, 0.65, 0.3
                else:
                    r, g, b = 0.0, 0.4, 0.18

                glBegin(GL_QUADS)
                glColor4f(r, g, b, 0.5)
                glVertex3f(x1, 0, z1)
                glVertex3f(x2, 0, z2)
                glColor4f(r, g, b, 0.8)
                glVertex3f(x2, wall_h, z2)
                glVertex3f(x1, wall_h, z1)
                glEnd()

                glBegin(GL_LINE_LOOP)
                glColor4f(r, g, b, 1.0)
                glVertex3f(x1, 0, z1)
                glVertex3f(x2, 0, z2)
                glVertex3f(x2, wall_h, z2)
                glVertex3f(x1, wall_h, z1)
                glEnd()

            glBegin(GL_LINES)
            glColor4f(0.0, 1.0, 0.4, 0.6)
            for j in range(1, len(sorted_pts)):
                x1, z1, _, idx1, _ = sorted_pts[j - 1]
                x2, z2, _, idx2, _ = sorted_pts[j]
                if idx2 - idx1 > 10:
                    continue
                dx = x2 - x1
                dz = z2 - z1
                if math.sqrt(dx*dx + dz*dz) > max_gap:
                    continue
                glVertex3f(x1, wall_h, z1)
                glVertex3f(x2, wall_h, z2)
            glEnd()


class LidarMap:
    def __init__(self):
        pygame.init()
        
        info = pygame.display.Info()
        self.width = min(1200, info.current_w - 100)
        self.height = min(900, info.current_h - 100)
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("MB-1R2T LiDAR Map")
        
        self.font = pygame.font.SysFont("monospace", 14)
        self.font_big = pygame.font.SysFont("monospace", 18, bold=True)
        self.font_small = pygame.font.SysFont("monospace", 11)
        
        self.scan_data = [None] * SCAN_SIZE
        self.scan_count = 0
        self.last_angle = 0.0
        
        self.zoom = 1.0
        self.max_range_m = 6
        self.show_walls = True
        self.show_grid = True
        self.fullscreen = False
        self.mode_3d = False
        self.view_3d = Lidar3DView(self.width, self.height) if HAS_OPENGL else None
        
        self.lidar = None
        self.port_name = ""
        self.connected = False
        
        self.clock = pygame.time.Clock()
        
        self._auto_connect()
    
    def _auto_connect(self):
        port = find_lidar_port()
        if port:
            try:
                self.lidar = LidarSerial(port)
                self.port_name = port
                self.connected = True
            except Exception as e:
                print(f"Failed to connect: {e}")
    
    def _update_zoom(self):
        center_x = self.width // 2
        center_y = self.height // 2
        usable = min(center_x, center_y) - 40
        self.zoom = usable / (self.max_range_m * 1000)
    
    def _world_to_screen(self, x_mm, y_mm):
        cx = self.width // 2
        cy = self.height // 2
        sx = cx + int(x_mm * self.zoom)
        sy = cy - int(y_mm * self.zoom)
        return sx, sy
    
    def _angle_to_xy(self, angle_deg, distance_mm):
        rad = math.radians(angle_deg)
        x = distance_mm * math.cos(rad)
        y = distance_mm * math.sin(rad)
        return x, y
    
    def _draw_grid(self):
        if not self.show_grid:
            return
        
        cx, cy = self.width // 2, self.height // 2
        
        for r_m in range(1, self.max_range_m + 1):
            r_px = int(r_m * 1000 * self.zoom)
            if r_px > 5:
                pygame.draw.circle(self.screen, GRID_COLOR, (cx, cy), r_px, 1)
                label = self.font_small.render(f"{r_m}m", True, GRID_TEXT_COLOR)
                self.screen.blit(label, (cx + 5, cy - r_px - 14))
        
        max_r_px = int(self.max_range_m * 1000 * self.zoom)
        for angle in range(0, 360, 45):
            rad = math.radians(angle)
            ex = cx + int(max_r_px * math.cos(rad))
            ey = cy - int(max_r_px * math.sin(rad))
            pygame.draw.line(self.screen, (25, 25, 35), (cx, cy), (ex, ey), 1)
        
        pygame.draw.circle(self.screen, CENTER_COLOR, (cx, cy), 5)
        pygame.draw.circle(self.screen, (255, 100, 100), (cx, cy), 3)
    
    def _process_data(self):
        if not self.lidar:
            return
        
        points = self.lidar.read()
        
        for angle, distance, quality in points:
            if angle < 30 and self.last_angle > 330:
                self.scan_count += 1
                for i in range(SCAN_SIZE):
                    if self.scan_data[i] is not None:
                        d, q, age = self.scan_data[i]
                        if age > POINT_FADE_SCANS:
                            self.scan_data[i] = None
                        else:
                            self.scan_data[i] = (d, q, age + 1)
            
            self.last_angle = angle
            
            idx = int(angle * 2) % SCAN_SIZE
            self.scan_data[idx] = (distance, quality, 0)
    
    def _draw_scan(self):
        max_range_mm = self.max_range_m * 1000
        
        screen_points = []
        
        for i in range(SCAN_SIZE):
            if self.scan_data[i] is None:
                continue
            
            distance, quality, age = self.scan_data[i]
            
            if distance > max_range_mm:
                continue
            
            angle_deg = i / 2.0
            x, y = self._angle_to_xy(angle_deg, distance)
            sx, sy = self._world_to_screen(x, y)
            
            if 0 <= sx < self.width and 0 <= sy < self.height:
                screen_points.append((sx, sy, age, i, distance))
                
                if age == 0:
                    color = POINT_COLOR_FRESH
                    size = 4
                elif age == 1:
                    color = (0, 220, 80)
                    size = 3
                else:
                    color = POINT_COLOR_OLD
                    size = 2
                
                pygame.draw.circle(self.screen, color, (sx, sy), size)
        
        if self.show_walls and len(screen_points) >= 2:
            screen_points.sort(key=lambda p: p[3])
            
            for j in range(1, len(screen_points)):
                sx1, sy1, age1, idx1, d1 = screen_points[j - 1]
                sx2, sy2, age2, idx2, d2 = screen_points[j]
                
                angle_gap = idx2 - idx1
                if angle_gap > 10:
                    continue
                
                pixel_dist = math.sqrt((sx2 - sx1)**2 + (sy2 - sy1)**2)
                
                max_pixel_gap = max(30, 150 * self.zoom)
                
                if pixel_dist < max_pixel_gap:
                    max_age = max(age1, age2)
                    if max_age == 0:
                        wc = (0, 255, 100)
                    elif max_age == 1:
                        wc = (0, 180, 70)
                    else:
                        wc = (0, 120, 50)
                    
                    pygame.draw.line(self.screen, wc, (sx1, sy1), (sx2, sy2), 2)
    
    def _draw_sweep_line(self):
        if not self.connected:
            return
        cx, cy = self.width // 2, self.height // 2
        rad = math.radians(self.last_angle)
        max_r = int(self.max_range_m * 1000 * self.zoom)
        ex = cx + int(max_r * math.cos(rad))
        ey = cy - int(max_r * math.sin(rad))
        pygame.draw.line(self.screen, (0, 80, 40), (cx, cy), (ex, ey), 1)
    
    def _draw_hud(self):
        pygame.draw.rect(self.screen, (20, 20, 28), (0, 0, self.width, 36))
        pygame.draw.line(self.screen, (40, 40, 50), (0, 36), (self.width, 36), 1)
        
        title = self.font_big.render("● LiDAR Map", True, (0, 255, 100))
        self.screen.blit(title, (12, 8))
        
        if self.connected:
            status_color = STATUS_GOOD
            port_short = self.port_name.split("/")[-1] if "/" in self.port_name else self.port_name
            status_text = f"Connected: {port_short}"
            
            pts = sum(1 for s in self.scan_data if s is not None)
            pps = self.lidar.points_per_sec if self.lidar else 0
            pkts = self.lidar.packet_count if self.lidar else 0
            stats = f"  │  {pts} pts  │  {pps} pts/s  │  {pkts} pkts  │  Scan #{self.scan_count}"
            status_text += stats
        else:
            status_color = STATUS_BAD
            status_text = "No LiDAR detected - check USB connection"
        
        status = self.font.render(status_text, True, status_color)
        self.screen.blit(status, (160, 10))
        
        help_y = self.height - 24
        pygame.draw.rect(self.screen, (20, 20, 28), (0, help_y - 4, self.width, 28))
        mode_hint = "  │  3 → 3D View" if HAS_OPENGL else ""
        help_text = f"Range: {self.max_range_m}m  │  +/- Zoom  │  W Walls: {'ON' if self.show_walls else 'OFF'}  │  G Grid  │  R Reset  │  F Fullscreen{mode_hint}  │  ESC Quit"
        help_surf = self.font_small.render(help_text, True, (100, 100, 120))
        self.screen.blit(help_surf, (12, help_y))
    
    def _draw_legend(self):
        lx = self.width - 170
        ly = 46
        lw = 160
        lh = 130
        
        legend_bg = pygame.Surface((lw, lh), pygame.SRCALPHA)
        legend_bg.fill((20, 20, 28, 200))
        self.screen.blit(legend_bg, (lx, ly))
        pygame.draw.rect(self.screen, (40, 40, 50), (lx, ly, lw, lh), 1)
        
        header = self.font_small.render("LEGEND", True, (150, 150, 160))
        self.screen.blit(header, (lx + 8, ly + 6))
        
        items = [
            (POINT_COLOR_FRESH, "Fresh point"),
            ((0, 220, 80), "1-scan old"),
            (POINT_COLOR_OLD, "2-3 scans old"),
            (WALL_COLOR[:3], "Wall segment"),
            (CENTER_COLOR, "LiDAR origin"),
            ((0, 80, 40), "Sweep line"),
        ]
        
        for i, (color, label) in enumerate(items):
            y = ly + 24 + i * 17
            pygame.draw.circle(self.screen, color, (lx + 16, y + 5), 4)
            text = self.font_small.render(label, True, (140, 140, 150))
            self.screen.blit(text, (lx + 28, y - 2))
    
    def _switch_to_3d(self):
        if not HAS_OPENGL or not self.view_3d:
            return
        self.mode_3d = True
        flags = DOUBLEBUF | OPENGL
        if self.fullscreen:
            flags |= FULLSCREEN
        self.screen = pygame.display.set_mode((self.width, self.height), flags)
        self.view_3d.init_gl(self.width, self.height)
        pygame.display.set_caption("MB-1R2T LiDAR Map [3D]")

    def _switch_to_2d(self):
        self.mode_3d = False
        flags = RESIZABLE
        if self.fullscreen:
            flags = FULLSCREEN
        self.screen = pygame.display.set_mode((self.width, self.height), flags)
        self._update_zoom()
        pygame.display.set_caption("MB-1R2T LiDAR Map")

    def run(self):
        running = True
        self._update_zoom()
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    elif event.key == pygame.K_3 and HAS_OPENGL and not self.mode_3d:
                        self._switch_to_3d()
                    elif event.key == pygame.K_2 and self.mode_3d:
                        self._switch_to_2d()
                    elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                        self.max_range_m = max(1, self.max_range_m - 1)
                        self._update_zoom()
                    elif event.key == pygame.K_MINUS:
                        self.max_range_m = min(20, self.max_range_m + 1)
                        self._update_zoom()
                    elif event.key == pygame.K_w:
                        self.show_walls = not self.show_walls
                    elif event.key == pygame.K_g:
                        self.show_grid = not self.show_grid
                    elif event.key == pygame.K_r:
                        self.scan_data = [None] * SCAN_SIZE
                        self.scan_count = 0
                        if self.mode_3d and self.view_3d:
                            self.view_3d.cam_dist = 8000.0
                            self.view_3d.cam_pitch = 35.0
                            self.view_3d.cam_yaw = 45.0
                            self.view_3d.cam_target = [0.0, 0.0, 0.0]
                    elif event.key == pygame.K_f:
                        self.fullscreen = not self.fullscreen
                        if self.fullscreen:
                            flags = FULLSCREEN
                            if self.mode_3d:
                                flags |= DOUBLEBUF | OPENGL
                            self.screen = pygame.display.set_mode((0, 0), flags)
                            info = pygame.display.Info()
                            self.width = info.current_w
                            self.height = info.current_h
                        else:
                            self.width, self.height = 1200, 900
                            flags = RESIZABLE
                            if self.mode_3d:
                                flags = DOUBLEBUF | OPENGL
                            self.screen = pygame.display.set_mode(
                                (self.width, self.height), flags)
                        self._update_zoom()
                        if self.mode_3d and self.view_3d:
                            self.view_3d.init_gl(self.width, self.height)
                
                elif event.type == pygame.VIDEORESIZE:
                    self.width, self.height = event.w, event.h
                    self._update_zoom()
                    if self.mode_3d and self.view_3d:
                        self.view_3d._setup_projection()
                
                if self.mode_3d and self.view_3d:
                    self.view_3d.handle_event(event)
            
            self._process_data()
            
            if self.mode_3d and self.view_3d:
                self.view_3d.render(self.scan_data, self.max_range_m, self.show_walls)
                pygame.display.flip()
            else:
                self.screen.fill(BG_COLOR)
                self._draw_grid()
                self._draw_sweep_line()
                self._draw_scan()
                self._draw_hud()
                self._draw_legend()
                pygame.display.flip()
            
            self.clock.tick(60)
        
        if self.lidar:
            self.lidar.close()
        pygame.quit()


def main():
    app = LidarMap()
    app.run()


if __name__ == "__main__":
    main()
