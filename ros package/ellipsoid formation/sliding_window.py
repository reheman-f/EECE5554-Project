#!/usr/bin/env python3
"""
IMU Data Comparator GUI - ROS2 Version
Subscribes to three IMU topics publishing VNYMR strings and displays variance ellipsoids.

Usage:
    ros2 run <package_name> imu_ellipsoid_ros2.py
    
    Or run directly:
    python3 imu_ellipsoid_ros2.py

Expects three topics publishing std_msgs/String with VNYMR format:
    /imu1
    /imu2
    /imu3

Expected VNYMR string format:
    $VNYMR,<sensor_id>,<yaw>,<pitch>,<roll>,<magX>,<magY>,<magZ>,<accX>,<accY>,<accZ>,<gyrX>,<gyrY>,<gyrZ>*<checksum>
"""

import sys
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict, List
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


@dataclass
class IMUSample:
    """Data class to store a single IMU sample."""
    timestamp: float
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    mag_x: float
    mag_y: float
    mag_z: float


class IMURos2Subscriber(Node):
    """ROS2 node that subscribes to three IMU topics publishing VNYMR strings."""
    
    def __init__(self, data_callback, topic_names: List[str]):
        super().__init__('imu_ellipsoid_visualizer')
        
        self.data_callback = data_callback
        self.topic_names = topic_names
        
        # QoS profile - RELIABLE to match default publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Debug: List available topics
        self.get_logger().info("Checking available topics...")
        topic_list = self.get_topic_names_and_types()
        for topic_name, topic_types in topic_list:
            if 'imu' in topic_name.lower():
                self.get_logger().info(f"  Found: {topic_name} -> {topic_types}")
        
        # Create subscribers for each IMU topic
        self.subscribers = []
        for i, topic in enumerate(topic_names):
            sensor_id = i + 1
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, sid=sensor_id: self.imu_callback(msg, sid),
                qos_profile
            )
            self.subscribers.append(sub)
            self.get_logger().info(f"Subscribed to {topic} as Sensor {sensor_id}")
            
            # Check if topic exists and show info
            matching_topics = [t for t, _ in topic_list if t == topic]
            if matching_topics:
                self.get_logger().info(f"  -> Topic {topic} exists")
            else:
                self.get_logger().warn(f"  -> Topic {topic} not found")
    
    def parse_vnymr(self, vnymr_string: str) -> Optional[IMUSample]:
        """
        Parse VNYMR string format.
        Format: $VNYMR,<id>,<timestamp>,<yaw>,<pitch>,<roll>,<magX>,<magY>,<magZ>,<accX>,<accY>,<accZ>,<gyrX>,<gyrY>,<gyrZ>*<checksum>
        Indices:   0     1       2        3      4      5      6      7      8      9      10     11     12     13     14
        """
        try:
            # Remove checksum if present
            if '*' in vnymr_string:
                vnymr_string = vnymr_string.split('*')[0]
            
            # Split by comma
            parts = vnymr_string.split(',')
            
            # Check for valid VNYMR message
            if not parts[0].startswith('$VNYMR'):
                return None
            
            if len(parts) < 15:
                return None
            
            sample = IMUSample(
                timestamp=float(parts[2]),
                accel_x=float(parts[9]),
                accel_y=float(parts[10]),
                accel_z=float(parts[11]),
                gyro_x=float(parts[12]),
                gyro_y=float(parts[13]),
                gyro_z=float(parts[14]),
                mag_x=float(parts[6]),
                mag_y=float(parts[7]),
                mag_z=float(parts[8])
            )
            return sample
            
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f"Failed to parse VNYMR: {e}")
            return None
    
    def imu_callback(self, msg: String, sensor_id: int):
        """Callback for IMU String messages."""
        try:
            # Parse the VNYMR string
            sample = self.parse_vnymr(msg.data)
            
            if sample:
                # Send to GUI callback
                self.data_callback(sensor_id, sample)
            
        except Exception as e:
            self.get_logger().error(f"Error processing IMU message: {e}")


class IMUComparatorGUI:
    """GUI for visualizing IMU variance across three sensors."""
    
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Data Comparator - ROS2 Version")
        self.root.geometry("750x600")
        
        # Topic configuration
        self.default_topics = [
            "/imu1",
            "/imu2", 
            "/imu3"
        ]
        self.topic_vars = []
        
        # Parameters
        self.update_interval = 500  # ms (10Hz update rate)
        self.window_size = 10  # Number of recent samples for std calculation
        
        # Tracking
        self.sample_count = 0
        self.is_running = False
        self.ros_node: Optional[IMURos2Subscriber] = None
        self.executor: Optional[MultiThreadedExecutor] = None
        self.spin_thread: Optional[threading.Thread] = None
        
        # Thread-safe data storage
        self.data_lock = threading.Lock()
        self.sample_windows: Dict[int, deque] = {
            1: deque(maxlen=self.window_size),
            2: deque(maxlen=self.window_size),
            3: deque(maxlen=self.window_size)
        }
        self.last_samples: Dict[int, Optional[IMUSample]] = {1: None, 2: None, 3: None}
        self.message_counts: Dict[int, int] = {1: 0, 2: 0, 3: 0}
        
        self.setup_gui()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_gui(self):
        """Setup the GUI components."""
        # Topic configuration frame
        topic_frame = ttk.LabelFrame(self.root, text="ROS2 Topics", padding="10")
        topic_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=10, pady=5)
        
        for i in range(3):
            ttk.Label(topic_frame, text=f"Sensor {i+1}:").grid(row=i, column=0, sticky=tk.W)
            var = tk.StringVar(value=self.default_topics[i])
            self.topic_vars.append(var)
            entry = ttk.Entry(topic_frame, textvariable=var, width=30)
            entry.grid(row=i, column=1, padx=5, pady=2)
        
        # Control buttons frame
        control_frame = ttk.Frame(self.root, padding="5")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), padx=10)
        
        self.start_button = ttk.Button(control_frame, text="Start Monitoring", 
                                        command=self.start_monitoring)
        self.start_button.grid(row=0, column=0, padx=5)
        
        self.stop_button = ttk.Button(control_frame, text="Stop", 
                                       command=self.stop_monitoring, state="disabled")
        self.stop_button.grid(row=0, column=1, padx=5)
        
        # Connection status
        self.connection_label = ttk.Label(control_frame, text="Not connected", 
                                          foreground="gray")
        self.connection_label.grid(row=0, column=2, padx=20)
        
        # Message count labels
        self.msg_count_labels = {}
        for i in range(3):
            label = ttk.Label(control_frame, text=f"S{i+1}: 0", foreground="gray")
            label.grid(row=0, column=3+i, padx=5)
            self.msg_count_labels[i+1] = label
        
        # Window size slider frame
        slider_frame = ttk.Frame(self.root, padding="5")
        slider_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), padx=10)
        
        ttk.Label(slider_frame, text="Window Size:").grid(row=0, column=0, sticky=tk.W)
        
        self.window_size_var = tk.IntVar(value=self.window_size)
        self.window_slider = ttk.Scale(
            slider_frame, 
            from_=2, 
            to=100, 
            orient=tk.HORIZONTAL,
            variable=self.window_size_var,
            command=self.on_window_size_change
        )
        self.window_slider.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        slider_frame.columnconfigure(1, weight=1)
        
        self.window_size_label = ttk.Label(slider_frame, text=f"{self.window_size} samples")
        self.window_size_label.grid(row=0, column=2, padx=5)
        
        # Ellipsoid visualization frame
        status_frame = ttk.LabelFrame(self.root, text="Sensor Variance Visualization", 
                                       padding="10")
        status_frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), 
                          padx=10, pady=10)
        
        # Create figure with two 3D subplots
        self.ellipsoid_figure = Figure(figsize=(10, 5), dpi=80)
        
        # Accelerometer subplot
        self.ax_accel = self.ellipsoid_figure.add_subplot(121, projection='3d')
        self.ax_accel.set_title("Accelerometer Variance", fontsize=10)
        self.ax_accel.set_xlabel('X', fontsize=8)
        self.ax_accel.set_ylabel('Y', fontsize=8)
        self.ax_accel.set_zlabel('Z', fontsize=8)
        self.ax_accel.set_box_aspect([1, 1, 1])
        
        # Gyroscope subplot
        self.ax_gyro = self.ellipsoid_figure.add_subplot(122, projection='3d')
        self.ax_gyro.set_title("Gyroscope Variance", fontsize=10)
        self.ax_gyro.set_xlabel('X', fontsize=8)
        self.ax_gyro.set_ylabel('Y', fontsize=8)
        self.ax_gyro.set_zlabel('Z', fontsize=8)
        self.ax_gyro.set_box_aspect([1, 1, 1])
        
        self.ellipsoid_canvas = FigureCanvasTkAgg(self.ellipsoid_figure, master=status_frame)
        self.ellipsoid_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.status_text = ttk.Label(status_frame, text="Not Started", font=("Arial", 12))
        self.status_text.pack(pady=5)
        
        self.chunk_info = ttk.Label(status_frame, text="", font=("Arial", 10))
        self.chunk_info.pack()
        
        self.timestamp_info = ttk.Label(status_frame, text="", font=("Arial", 9), 
                                         foreground="gray")
        self.timestamp_info.pack()
        
        # Legend
        legend_frame = ttk.Frame(status_frame)
        legend_frame.pack(pady=5)
        ttk.Label(legend_frame, 
                  text="Ellipsoid size represents standard deviation across sensors",
                  font=("Arial", 8), foreground="gray").pack()
        ttk.Label(legend_frame, 
                  text="Smaller = Better sensor agreement | Larger = More variance",
                  font=("Arial", 8), foreground="gray").pack()
        
        self.root.grid_rowconfigure(3, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        
        # Initialize ellipsoids with neutral state
        self.draw_ellipsoids(None, None)
    
    def on_window_size_change(self, value):
        """Handle window size slider change."""
        new_size = int(float(value))
        self.window_size = new_size
        self.window_size_label.config(text=f"{new_size} samples")
        
        # Update the deque maxlen by creating new deques with the new size
        # and copying over existing data
        with self.data_lock:
            for sensor_id in [1, 2, 3]:
                old_deque = self.sample_windows[sensor_id]
                new_deque = deque(old_deque, maxlen=new_size)
                self.sample_windows[sensor_id] = new_deque
    
    def draw_ellipsoids(self, accel_stds: Optional[List[float]], 
                        gyro_stds: Optional[List[float]]):
        """Draw 3D ellipsoids representing standard deviations."""
        # Clear previous plots
        self.ax_accel.clear()
        self.ax_gyro.clear()
        
        # Set labels
        self.ax_accel.set_title("Accelerometer Variance", fontsize=10)
        self.ax_accel.set_xlabel('X', fontsize=8)
        self.ax_accel.set_ylabel('Y', fontsize=8)
        self.ax_accel.set_zlabel('Z', fontsize=8)
        
        self.ax_gyro.set_title("Gyroscope Variance", fontsize=10)
        self.ax_gyro.set_xlabel('X', fontsize=8)
        self.ax_gyro.set_ylabel('Y', fontsize=8)
        self.ax_gyro.set_zlabel('Z', fontsize=8)
        
        if accel_stds is None or gyro_stds is None:
            # Draw placeholder spheres when no data
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            x = 0.1 * np.outer(np.cos(u), np.sin(v))
            y = 0.1 * np.outer(np.sin(u), np.sin(v))
            z = 0.1 * np.outer(np.ones(np.size(u)), np.cos(v))
            
            self.ax_accel.plot_surface(x, y, z, alpha=0.3, color='gray')
            self.ax_gyro.plot_surface(x, y, z, alpha=0.3, color='gray')
            
            for ax in [self.ax_accel, self.ax_gyro]:
                ax.set_xlim([-1, 1])
                ax.set_ylim([-1, 1])
                ax.set_zlim([-1, 1])
            
            self.status_text.config(text="Waiting...", foreground="gray")
        else:
            # Draw ellipsoids based on standard deviations
            self._draw_single_ellipsoid(self.ax_accel, accel_stds, 'Accelerometer')
            self._draw_single_ellipsoid(self.ax_gyro, gyro_stds, 'Gyroscope')
            
            # Update status text based on variance magnitude
            total_variance = sum(accel_stds) + sum(gyro_stds)
            if total_variance < 0.5:
                self.status_text.config(text="EXCELLENT MATCH", foreground="#4CAF50")
            elif total_variance < 1.0:
                self.status_text.config(text="GOOD MATCH", foreground="#8BC34A")
            elif total_variance < 2.0:
                self.status_text.config(text="MODERATE VARIANCE", foreground="#FFC107")
            else:
                self.status_text.config(text="HIGH VARIANCE", foreground="#f44336")
        
        self.ellipsoid_canvas.draw()
    
    def _draw_single_ellipsoid(self, ax, stds: List[float], sensor_type: str):
        """Draw a single ellipsoid on the given axis."""
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 20)
        
        # Scale factors (add small minimum to avoid zero-size ellipsoids)
        rx = max(stds[0], 0.05)
        ry = max(stds[1], 0.05)
        rz = max(stds[2], 0.05)
        
        # Generate ellipsoid coordinates
        x = rx * np.outer(np.cos(u), np.sin(v))
        y = ry * np.outer(np.sin(u), np.sin(v))
        z = rz * np.outer(np.ones(np.size(u)), np.cos(v))
        
        # Color based on total variance
        total_std = sum(stds)
        if total_std < 0.3:
            color = '#4CAF50'  # Green - good
        elif total_std < 0.6:
            color = '#8BC34A'  # Light green
        elif total_std < 1.0:
            color = '#FFC107'  # Yellow - moderate
        else:
            color = '#f44336'  # Red - high variance
        
        # Plot ellipsoid
        ax.plot_surface(x, y, z, alpha=0.6, color=color, edgecolor='none')
        
        # Add wireframe for better visibility
        ax.plot_wireframe(x, y, z, alpha=0.2, color='black', linewidth=0.5)
        
        # Add axis lines through origin
        axis_length = max(rx, ry, rz) * 1.2
        ax.plot([0, axis_length], [0, 0], [0, 0], 'r-', alpha=0.5, linewidth=1)
        ax.plot([0, 0], [0, axis_length], [0, 0], 'g-', alpha=0.5, linewidth=1)
        ax.plot([0, 0], [0, 0], [0, axis_length], 'b-', alpha=0.5, linewidth=1)
        
        # Set axis limits
        max_radius = max(rx, ry, rz) * 1.5
        ax.set_xlim([-max_radius, max_radius])
        ax.set_ylim([-max_radius, max_radius])
        ax.set_zlim([-max_radius, max_radius])
        
        # Add text showing standard deviations
        ax.text2D(0.05, 0.95, f"σx: {stds[0]:.3f}", transform=ax.transAxes, fontsize=8)
        ax.text2D(0.05, 0.90, f"σy: {stds[1]:.3f}", transform=ax.transAxes, fontsize=8)
        ax.text2D(0.05, 0.85, f"σz: {stds[2]:.3f}", transform=ax.transAxes, fontsize=8)
    
    def imu_data_callback(self, sensor_id: int, sample: IMUSample):
        """Callback called when new IMU data arrives from ROS2."""
        with self.data_lock:
            self.sample_windows[sensor_id].append(sample)
            self.last_samples[sensor_id] = sample
            self.message_counts[sensor_id] += 1
    
    def start_monitoring(self):
        """Start ROS2 subscriber and visualization."""
        try:
            # Get topic names
            topics = [var.get().strip() for var in self.topic_vars]
            
            # Validate topics
            if any(not topic for topic in topics):
                messagebox.showerror("Error", "All topic names must be specified")
                return
            
            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()
            
            # Create node
            self.ros_node = IMURos2Subscriber(self.imu_data_callback, topics)
            
            # Create executor and add node
            self.executor = MultiThreadedExecutor(num_threads=2)
            self.executor.add_node(self.ros_node)
            
            # Clear data
            with self.data_lock:
                for sensor_id in [1, 2, 3]:
                    self.sample_windows[sensor_id].clear()
                    self.last_samples[sensor_id] = None
                    self.message_counts[sensor_id] = 0
            
            self.sample_count = 0
            self.is_running = True  # Set BEFORE starting thread!
            
            # Start spin thread
            self.spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
            self.spin_thread.start()
            
            # Give ROS2 time to discover topics and establish connections
            time.sleep(0.5)
            
            # Update UI
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.connection_label.config(text="Connected", foreground="green")
            
            # Disable topic entries
            for child in self.root.winfo_children()[0].winfo_children():
                if isinstance(child, ttk.Entry):
                    child.config(state="disabled")
            
            # Start update loop
            self.update_display()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start monitoring: {e}")
            self.stop_monitoring()
    
    def _spin_ros(self):
        """Spin ROS2 executor in background thread."""
        try:
            while self.is_running and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.01)
        except Exception as e:
            print(f"ROS2 spin error: {e}")
    
    def stop_monitoring(self):
        """Stop monitoring and cleanup ROS2."""
        self.is_running = False
        
        # Cleanup ROS2
        if self.executor:
            self.executor.shutdown()
            self.executor = None
        
        if self.ros_node:
            self.ros_node.destroy_node()
            self.ros_node = None
        
        # Update UI
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.connection_label.config(text="Disconnected", foreground="gray")
        
        # Re-enable topic entries
        for child in self.root.winfo_children()[0].winfo_children():
            if isinstance(child, ttk.Entry):
                child.config(state="normal")
        
        self.draw_ellipsoids(None, None)
        self.chunk_info.config(text="Monitoring stopped")
        self.timestamp_info.config(text="")
    
    def calculate_variance(self) -> tuple:
        """Calculate variance across sensors from recent samples."""
        with self.data_lock:
            # Count sensors with data
            window_sizes = {sid: len(self.sample_windows[sid]) for sid in [1, 2, 3]}
            active_sensors = sum(1 for size in window_sizes.values() if size >= 1)
            
            if active_sensors < 2:
                return None, None
            
            accel_stds = [0.0, 0.0, 0.0]
            gyro_stds = [0.0, 0.0, 0.0]
            
            # Calculate mean values for each sensor, then std across sensors
            # Accelerometer
            for i, attr in enumerate(['accel_x', 'accel_y', 'accel_z']):
                sensor_means = []
                for sensor_id in [1, 2, 3]:
                    if len(self.sample_windows[sensor_id]) >= 1:
                        values = [getattr(s, attr) for s in self.sample_windows[sensor_id]]
                        sensor_means.append(np.mean(values))
                if len(sensor_means) >= 2:
                    accel_stds[i] = float(np.std(sensor_means))
            
            # Gyroscope
            for i, attr in enumerate(['gyro_x', 'gyro_y', 'gyro_z']):
                sensor_means = []
                for sensor_id in [1, 2, 3]:
                    if len(self.sample_windows[sensor_id]) >= 1:
                        values = [getattr(s, attr) for s in self.sample_windows[sensor_id]]
                        sensor_means.append(np.mean(values))
                if len(sensor_means) >= 2:
                    gyro_stds[i] = float(np.std(sensor_means))
            
            return accel_stds, gyro_stds
    
    def update_display(self):
        """Update display at 10Hz."""
        if not self.is_running:
            return
        
        self.sample_count += 1
        
        # Update message counts
        with self.data_lock:
            for sensor_id in [1, 2, 3]:
                count = self.message_counts[sensor_id]
                color = "green" if count > 0 else "red"
                self.msg_count_labels[sensor_id].config(
                    text=f"S{sensor_id}: {count}", foreground=color)
        
        # Calculate variance and update ellipsoids
        accel_stds, gyro_stds = self.calculate_variance()
        
        if accel_stds is not None and gyro_stds is not None:
            self.draw_ellipsoids(accel_stds, gyro_stds)
        
        # Update info labels
        self.chunk_info.config(text=f"Update #{self.sample_count}")
        self.timestamp_info.config(
            text=f"Updating at {1000//self.update_interval}Hz | "
                 f"Window size: {self.window_size} samples")
        
        # Schedule next update
        self.root.after(self.update_interval, self.update_display)
    
    def on_closing(self):
        """Handle window close event."""
        self.stop_monitoring()
        
        # Shutdown ROS2
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        
        self.root.destroy()


def main():
    """Main function."""
    root = tk.Tk()
    app = IMUComparatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()