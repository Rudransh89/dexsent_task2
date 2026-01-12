#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QSlider, QGroupBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer

class DualArmGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ros()
        self.init_ui()
        
    def init_ros(self):
        rclpy.init(args=None)
        self.node = Node('dual_arm_gui')
        self.pub = self.node.create_publisher(Float64MultiArray, '/dual_arm/command', 10)
        
        # Default Positions
        self.lx, self.ly, self.lz = 0.5, 0.0, 0.5
        self.rx, self.ry, self.rz = 0.5, 0.0, 0.5

    def init_ui(self):
        self.setWindowTitle("DexSent Dual-Arm Control Center")
        self.setGeometry(100, 100, 700, 450)
        self.setStyleSheet("""
            QWidget { background-color: #2b2b2b; color: #ffffff; font-family: Arial; }
            QGroupBox { border: 2px solid orange; border-radius: 5px; margin-top: 10px; font-weight: bold; }
            QGroupBox::title { color: orange; subcontrol-origin: margin; left: 10px; }
            QSlider::groove:horizontal { height: 8px; background: #444; border-radius: 4px; }
            QSlider::handle:horizontal { background: orange; width: 16px; margin: -4px 0; border-radius: 8px; }
            QCheckBox { font-size: 14px; color: #00ff00; spacing: 10px; }
        """)

        main_layout = QVBoxLayout()
        
        # === HEADER & SYNC ===
        top_layout = QHBoxLayout()
        title = QLabel("MANUAL CONTROL INTERFACE")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #aaa;")
        
        # THE NEW SYNC CHECKBOX
        self.sync_checkbox = QCheckBox("🔗 SYNCHRONIZE ARMS (Master/Slave)")
        self.sync_checkbox.stateChanged.connect(self.toggle_sync)
        
        top_layout.addWidget(title)
        top_layout.addStretch()
        top_layout.addWidget(self.sync_checkbox)
        main_layout.addLayout(top_layout)

        controls_layout = QHBoxLayout()

        # === LEFT ARM CONTROLS (MASTER) ===
        left_group = QGroupBox("Left Arm (Master)")
        left_layout = QVBoxLayout()
        
        self.l_x_slider = self.create_slider("X Axis (Reach)", 0.3, 0.8, self.update_command)
        self.l_y_slider = self.create_slider("Y Axis (Swing)", -0.5, 0.5, self.update_command)
        self.l_z_slider = self.create_slider("Z Axis (Height)", 0.2, 0.8, self.update_command)
        
        left_layout.addLayout(self.l_x_slider)
        left_layout.addLayout(self.l_y_slider)
        left_layout.addLayout(self.l_z_slider)
        left_group.setLayout(left_layout)

        # === RIGHT ARM CONTROLS (SLAVE) ===
        right_group = QGroupBox("Right Arm (Slave)")
        right_layout = QVBoxLayout()

        self.r_x_slider = self.create_slider("X Axis (Reach)", 0.3, 0.8, self.update_command)
        self.r_y_slider = self.create_slider("Y Axis (Swing)", -0.5, 0.5, self.update_command)
        self.r_z_slider = self.create_slider("Z Axis (Height)", 0.2, 0.8, self.update_command)

        right_layout.addLayout(self.r_x_slider)
        right_layout.addLayout(self.r_y_slider)
        right_layout.addLayout(self.r_z_slider)
        right_group.setLayout(right_layout)

        controls_layout.addWidget(left_group)
        controls_layout.addWidget(right_group)
        main_layout.addLayout(controls_layout)

        self.setLayout(main_layout)

        # Store references to the actual slider widgets for enabling/disabling
        self.right_sliders = [
            self.r_x_slider.itemAt(1).widget(),
            self.r_y_slider.itemAt(1).widget(),
            self.r_z_slider.itemAt(1).widget()
        ]

        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50) # Faster update rate for smooth sync

    def create_slider(self, label_text, min_val, max_val, callback):
        layout = QVBoxLayout()
        label = QLabel(label_text)
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(min_val * 100))
        slider.setMaximum(int(max_val * 100))
        slider.setValue(int((min_val + max_val) * 50))
        slider.valueChanged.connect(callback)
        layout.addWidget(label)
        layout.addWidget(slider)
        return layout

    def toggle_sync(self):
        """ Lock Right Arm controls if Sync is ON """
        is_sync = self.sync_checkbox.isChecked()
        for slider in self.right_sliders:
            slider.setEnabled(not is_sync)
            # Visually dim them if disabled
            slider.setStyleSheet("background: #555;" if is_sync else "")
        
        if is_sync:
            # Force immediate update to snap Slave to Master position
            self.update_command()

    def update_command(self):
        # 1. Read Left (Master) Values
        self.lx = self.l_x_slider.itemAt(1).widget().value() / 100.0
        self.ly = self.l_y_slider.itemAt(1).widget().value() / 100.0
        self.lz = self.l_z_slider.itemAt(1).widget().value() / 100.0
        
        # 2. Handle Synchronization
        if self.sync_checkbox.isChecked():
            # Copy Master values to Slave variables
            self.rx, self.ry, self.rz = self.lx, self.ly, self.lz
            
            # Update Slave Sliders visually (Block signals to prevent infinite loop)
            for i, val in enumerate([self.lx, self.ly, self.lz]):
                slider = self.right_sliders[i]
                slider.blockSignals(True)
                slider.setValue(int(val * 100))
                slider.blockSignals(False)
        else:
            # Read Right Values normally
            self.rx = self.r_x_slider.itemAt(1).widget().value() / 100.0
            self.ry = self.r_y_slider.itemAt(1).widget().value() / 100.0
            self.rz = self.r_z_slider.itemAt(1).widget().value() / 100.0

        # 3. Publish
        msg = Float64MultiArray()
        msg.data = [self.lx, self.ly, self.lz, self.rx, self.ry, self.rz]
        self.pub.publish(msg)

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = DualArmGUI()
    gui.show()
    sys.exit(app.exec_())
