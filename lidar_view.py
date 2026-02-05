import sys
import platform
import serial
import serial.tools.list_ports
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QComboBox, QSlider,
                             QCheckBox, QFrame, QSizePolicy)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QColor, QPalette
import pyqtgraph as pg
import pyqtgraph.opengl as gl

BAUD_RATE = 153600

STYLESHEET = """
QMainWindow {
    background-color: #1a1a1f;
}
QWidget {
    background-color: #1a1a1f;
    color: #e0e0e0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    font-size: 13px;
}
QLabel {
    color: #a0a0a0;
    padding: 2px;
}
QLabel#title {
    color: #ffffff;
    font-size: 16px;
    font-weight: bold;
}
QLabel#stats {
    color: #00ff88;
    font-family: "SF Mono", Menlo, Monaco, monospace;
    font-size: 12px;
    background-color: #252530;
    padding: 6px 12px;
    border-radius: 4px;
}
QPushButton {
    background-color: #2d2d35;
    border: 1px solid #3d3d45;
    border-radius: 6px;
    padding: 8px 16px;
    color: #e0e0e0;
    font-weight: 500;
}
QPushButton:hover {
    background-color: #3d3d45;
    border-color: #4d4d55;
}
QPushButton:pressed {
    background-color: #252530;
}
QPushButton#connect {
    background-color: #00aa55;
    border-color: #00cc66;
    color: white;
}
QPushButton#connect:hover {
    background-color: #00bb66;
}
QPushButton#disconnect {
    background-color: #cc4444;
    border-color: #dd5555;
    color: white;
}
QPushButton#disconnect:hover {
    background-color: #dd5555;
}
QComboBox {
    background-color: #2d2d35;
    border: 1px solid #3d3d45;
    border-radius: 6px;
    padding: 6px 12px;
    min-width: 180px;
}
QComboBox:hover {
    border-color: #4d4d55;
}
QComboBox::drop-down {
    border: none;
    padding-right: 10px;
}
QComboBox QAbstractItemView {
    background-color: #2d2d35;
    border: 1px solid #3d3d45;
    selection-background-color: #00aa55;
}
QSlider::groove:horizontal {
    background: #2d2d35;
    height: 6px;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #00aa55;
    width: 16px;
    height: 16px;
    margin: -5px 0;
    border-radius: 8px;
}
QSlider::handle:horizontal:hover {
    background: #00cc66;
}
QCheckBox {
    spacing: 8px;
}
QCheckBox::indicator {
    width: 18px;
    height: 18px;
    border-radius: 4px;
    border: 2px solid #3d3d45;
    background-color: #2d2d35;
}
QCheckBox::indicator:checked {
    background-color: #00aa55;
    border-color: #00aa55;
}
QFrame#toolbar {
    background-color: #222228;
    border-bottom: 1px solid #333340;
    padding: 8px;
}
"""


def find_lidar_port():
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        desc = (port.description or "").lower()
        device = port.device.lower()
        
        # Look for FTDI/USB serial adapters
        if platform.system() == "Darwin":  # macOS
            if "usbserial" in device or "cu.usb" in device:
                return port.device
        elif platform.system() == "Windows":
            if "com" in device and ("ftdi" in desc or "usb" in desc or "serial" in desc):
                return port.device
        else:  # Linux
            if "ttyusb" in device or "ttyacm" in device:
                return port.device
    
    # Fallback: return first USB-like port
    for port in ports:
        if "usb" in port.device.lower():
            return port.device
    
    return None


class LidarReader:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD_RATE, timeout=0)
        self.ser.reset_input_buffer()
        self.buffer = bytearray()
        self.packet_count = 0
        
    def read_packets(self):
        points = []
        
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                self.buffer.extend(self.ser.read(waiting))
        except:
            return points
        
        if len(self.buffer) > 50000:
            self.buffer = self.buffer[-25000:]
            try:
                del self.buffer[:self.buffer.index(0xAA)]
            except:
                self.buffer.clear()
        
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
            
            if len(self.buffer) < 9:
                break
                
            num = self.buffer[3]
            if num > 100:
                del self.buffer[:2]
                continue
                
            plen = 9 + num * 3
            if len(self.buffer) < plen:
                break
            
            pkt = bytes(self.buffer[:plen])
            del self.buffer[:plen]
            
            sa = (pkt[4] | (pkt[5] << 8)) / 100.0
            ea = (pkt[6] | (pkt[7] << 8)) / 100.0
            if ea < sa:
                ea += 360.0
            
            step = (ea - sa) / (num - 1) if num > 1 else 0
            
            for i in range(num):
                off = 9 + i * 3
                if off + 2 < len(pkt):
                    q = pkt[off]
                    d = pkt[off + 1] | (pkt[off + 2] << 8)
                    if d < 60000 and q > 5:
                        points.append(((sa + i * step) % 360, d, q))
            
            self.packet_count += 1
        
        return points
    
    def close(self):
        try:
            self.ser.close()
        except:
            pass


class LidarVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MB-1R2T LiDAR")
        self.setMinimumSize(900, 700)
        self.resize(1100, 850)
        
        self.scan_buffer = {}
        self.lidar = None
        self.current_view = "2D"
        self.max_range = 8000
        
        self.setup_ui()
        self.setup_timer()
        self.auto_connect()
        
    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        
        # Main layout - no margins for full use of space
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Top toolbar
        toolbar = QFrame()
        toolbar.setObjectName("toolbar")
        toolbar_layout = QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(12, 8, 12, 8)
        toolbar_layout.setSpacing(12)
        
        # Title
        title = QLabel("⦿ LiDAR")
        title.setObjectName("title")
        toolbar_layout.addWidget(title)
        
        toolbar_layout.addSpacing(20)
        
        # Port selection
        self.port_combo = QComboBox()
        self.refresh_ports()
        toolbar_layout.addWidget(self.port_combo)
        
        refresh_btn = QPushButton("↻")
        refresh_btn.setFixedWidth(36)
        refresh_btn.clicked.connect(self.refresh_ports)
        toolbar_layout.addWidget(refresh_btn)
        
        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        toolbar_layout.addWidget(self.connect_btn)
        
        toolbar_layout.addSpacing(20)
        
        # View toggle
        self.view_btn = QPushButton("3D View")
        self.view_btn.clicked.connect(self.toggle_view)
        toolbar_layout.addWidget(self.view_btn)
        
        toolbar_layout.addSpacing(20)
        
        # Range control
        toolbar_layout.addWidget(QLabel("Range:"))
        self.range_slider = QSlider(Qt.Orientation.Horizontal)
        self.range_slider.setMinimum(2)
        self.range_slider.setMaximum(20)
        self.range_slider.setValue(8)
        self.range_slider.setFixedWidth(120)
        self.range_slider.valueChanged.connect(self.update_range)
        toolbar_layout.addWidget(self.range_slider)
        self.range_label = QLabel("8m")
        self.range_label.setFixedWidth(30)
        toolbar_layout.addWidget(self.range_label)
        
        toolbar_layout.addStretch()
        
        # Stats
        self.stats_label = QLabel("⬚ Ready")
        self.stats_label.setObjectName("stats")
        toolbar_layout.addWidget(self.stats_label)
        
        layout.addWidget(toolbar)
        
        # Visualization area
        self.setup_2d_view()
        self.setup_3d_view()
        
        self.plot_widget_2d.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.plot_widget_3d.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        
        layout.addWidget(self.plot_widget_2d, 1)  # stretch factor 1
        self.plot_widget_3d.hide()
        layout.addWidget(self.plot_widget_3d, 1)
        
    def setup_2d_view(self):
        self.plot_widget_2d = pg.PlotWidget()
        self.plot_widget_2d.setBackground((18, 18, 22))
        self.plot_widget_2d.setAspectLocked(True)
        self.plot_widget_2d.hideAxis('left')
        self.plot_widget_2d.hideAxis('bottom')
        
        # Range circles
        for r in range(1000, 20001, 1000):
            circle = pg.PlotCurveItem()
            theta = np.linspace(0, 2*np.pi, 100)
            alpha = 60 if r % 2000 == 0 else 25
            width = 1.5 if r % 2000 == 0 else 0.5
            circle.setData(r * np.cos(theta), r * np.sin(theta), 
                          pen=pg.mkPen((50, 50, 60, alpha), width=width))
            self.plot_widget_2d.addItem(circle)
            
            # Range labels on major circles
            if r % 2000 == 0:
                label = pg.TextItem(f"{r//1000}m", color=(70, 70, 80), anchor=(0.5, 0.5))
                label.setPos(0, r + 200)
                self.plot_widget_2d.addItem(label)
        
        # Angle lines
        for angle in range(0, 360, 30):
            rad = np.radians(angle)
            line = pg.PlotCurveItem()
            line.setData([0, 20000*np.cos(rad)], [0, 20000*np.sin(rad)],
                        pen=pg.mkPen((40, 40, 50), width=0.5))
            self.plot_widget_2d.addItem(line)
        
        # Origin
        origin = pg.ScatterPlotItem()
        origin.setData([0], [0], size=14, 
                      brush=pg.mkBrush(255, 60, 60),
                      pen=pg.mkPen((255, 100, 100), width=2))
        self.plot_widget_2d.addItem(origin)
        
        # Scan points
        self.scatter_2d = pg.ScatterPlotItem()
        self.plot_widget_2d.addItem(self.scatter_2d)
        
        self.plot_widget_2d.setXRange(-self.max_range, self.max_range)
        self.plot_widget_2d.setYRange(-self.max_range, self.max_range)
        
    def setup_3d_view(self):
        self.plot_widget_3d = gl.GLViewWidget()
        self.plot_widget_3d.setCameraPosition(distance=12000, elevation=35, azimuth=45)
        self.plot_widget_3d.setBackgroundColor((18, 18, 22))
        
        # Grid
        grid = gl.GLGridItem()
        grid.setSize(20000, 20000, 1)
        grid.setSpacing(2000, 2000, 1)
        grid.setColor((50, 50, 60, 80))
        self.plot_widget_3d.addItem(grid)
        
        # Points
        self.scatter_3d = gl.GLScatterPlotItem()
        self.scatter_3d.setGLOptions('translucent')
        self.plot_widget_3d.addItem(self.scatter_3d)
        
        # Origin
        origin = gl.GLScatterPlotItem(pos=np.array([[0, 0, 30]]), size=12, color=(1, 0.25, 0.25, 1))
        self.plot_widget_3d.addItem(origin)
        
    def setup_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.frame_count = 0
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps = 0
        
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            device = port.device.lower()
            if "usb" in device or "serial" in device or "com" in device:
                self.port_combo.addItem(port.device)
        
        # Auto-select detected port
        detected = find_lidar_port()
        if detected:
            idx = self.port_combo.findText(detected)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
                
    def auto_connect(self):
        """Try to auto-connect on startup"""
        if self.port_combo.count() > 0:
            port = find_lidar_port()
            if port:
                try:
                    self.lidar = LidarReader(port)
                    self.connect_btn.setText("Disconnect")
                    self.connect_btn.setObjectName("disconnect")
                    self.connect_btn.setStyle(self.connect_btn.style())
                    self.timer.start(33)
                    self.fps_timer.start(1000)
                except:
                    pass
        
    def toggle_connection(self):
        if self.lidar is None:
            try:
                port = self.port_combo.currentText()
                if not port:
                    return
                self.lidar = LidarReader(port)
                self.connect_btn.setText("Disconnect")
                self.connect_btn.setObjectName("disconnect")
                self.connect_btn.setStyle(self.connect_btn.style())
                self.timer.start(33)
                self.fps_timer.start(1000)
                self.scan_buffer.clear()
            except Exception as e:
                self.stats_label.setText(f"⚠ Error: {e}")
        else:
            self.timer.stop()
            self.fps_timer.stop()
            self.lidar.close()
            self.lidar = None
            self.connect_btn.setText("Connect")
            self.connect_btn.setObjectName("connect")
            self.connect_btn.setStyle(self.connect_btn.style())
            self.stats_label.setText("⬚ Disconnected")
            
    def toggle_view(self):
        if self.current_view == "2D":
            self.current_view = "3D"
            self.view_btn.setText("2D View")
            self.plot_widget_2d.hide()
            self.plot_widget_3d.show()
        else:
            self.current_view = "2D"
            self.view_btn.setText("3D View")
            self.plot_widget_3d.hide()
            self.plot_widget_2d.show()
            
    def update_range(self, value):
        self.max_range = value * 1000
        self.range_label.setText(f"{value}m")
        self.plot_widget_2d.setXRange(-self.max_range, self.max_range)
        self.plot_widget_2d.setYRange(-self.max_range, self.max_range)
            
    def update_fps(self):
        self.fps = self.frame_count
        self.frame_count = 0
        
    def update(self):
        if self.lidar is None:
            return
            
        new_points = self.lidar.read_packets()
        
        for angle, distance, quality in new_points:
            self.scan_buffer[round(angle * 2) / 2] = (distance, quality)
        
        if not self.scan_buffer:
            return
            
        sorted_angles = sorted(self.scan_buffer.keys())
        
        angles_rad = np.array([np.radians(a) for a in sorted_angles])
        data = [self.scan_buffer[a] for a in sorted_angles]
        distances = np.array([d[0] for d in data])
        qualities = np.array([d[1] for d in data])
        
        x = distances * np.cos(angles_rad)
        y = distances * np.sin(angles_rad)
        
        if self.current_view == "2D":
            # Color by quality: cyan -> yellow
            norm = np.clip(qualities / 180, 0, 1)
            colors = []
            for n in norm:
                r = int(50 + 205 * n)
                g = int(200 + 55 * n)
                b = int(255 * (1 - n * 0.7))
                colors.append(pg.mkBrush(r, g, b))
            self.scatter_2d.setData(x, y, brush=colors, size=7, pen=pg.mkPen(None))
        else:
            z = np.zeros_like(x) + 30
            pos = np.column_stack([x, y, z])
            norm = np.clip(qualities / 180, 0, 1)
            colors = np.column_stack([
                0.2 + 0.8 * norm,
                0.8 + 0.2 * norm,
                1.0 - 0.7 * norm,
                np.ones(len(norm))
            ])
            self.scatter_3d.setData(pos=pos, color=colors, size=5)
        
        self.frame_count += 1
        self.stats_label.setText(
            f"● {len(self.scan_buffer)} pts  │  {self.lidar.packet_count} pkts  │  {self.fps} fps"
        )
        
    def closeEvent(self, event):
        if self.lidar:
            self.timer.stop()
            self.lidar.close()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(STYLESHEET)
    
    window = LidarVisualizer()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
