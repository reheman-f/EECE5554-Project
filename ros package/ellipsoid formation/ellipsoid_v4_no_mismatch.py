#!/usr/bin/env python3
"""
IMU Data Comparator GUI - Single File Version
This version handles a single CSV file with a SensorID column.
"""

import csv
import sys
import numpy as np
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
from collections import deque, defaultdict
import bisect


class IMUComparatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Data Comparator - Single File Version")
        self.root.geometry("700x500")  # Adjusted for clean ellipsoid-focused layout
        
        # File path and data
        self.file_path = None
        self.sensor_data = defaultdict(list)  # {sensor_id: [data_rows]}
        self.start_timestamps = {}  # {sensor_id: start_timestamp}
        self.min_common_time = 0
        self.max_common_time = 0
        self.sensor_indices = {1: 0, 2: 0, 3: 0}  # Current index for each sensor
        
        # Parameters
        self.sampling_rate = 10  # Hz
        self.update_interval = 0.1  # seconds (10Hz update rate)
        self.window_size = 10  # Number of recent samples to keep for std calculation
        
        # Tracking
        self.sample_number = 0
        self.current_time_offset = 0
        self.is_running = False
        self.processing_thread = None
        
        # Sliding window for standard deviation calculation
        self.sample_windows = {1: deque(maxlen=self.window_size),
                               2: deque(maxlen=self.window_size),
                               3: deque(maxlen=self.window_size)}
        
        self.setup_gui()
        
    def setup_gui(self):
        """Setup the GUI components."""
        # File selection frame
        file_frame = ttk.Frame(self.root, padding="10")
        file_frame.grid(row=0, column=0, sticky=(tk.W, tk.E))
        
        ttk.Label(file_frame, text="Select CSV File:").grid(row=0, column=0, sticky=tk.W)
        self.file_label = ttk.Label(file_frame, text="No file selected", foreground="gray")
        self.file_label.grid(row=0, column=1, padx=10)
        
        ttk.Button(file_frame, text="Browse File", command=self.select_file).grid(row=0, column=2)
        self.start_button = ttk.Button(file_frame, text="Start Monitoring", command=self.start_monitoring, state="disabled")
        self.start_button.grid(row=0, column=3, padx=5)
        self.stop_button = ttk.Button(file_frame, text="Stop", command=self.stop_monitoring, state="disabled")
        self.stop_button.grid(row=0, column=4)
        
        # Info frame
        info_frame = ttk.Frame(self.root, padding="5")
        info_frame.grid(row=1, column=0, sticky=(tk.W, tk.E))
        
        self.info_label = ttk.Label(info_frame, text="Single file mode with SensorID column", 
                                   font=("Arial", 9, "bold"), foreground="green")
        self.info_label.grid(row=0, column=0, padx=10)
        
        # Status indicator frame - now contains 3D ellipsoids
        status_frame = ttk.LabelFrame(self.root, text="Sensor Variance Visualization", padding="10")
        status_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10, pady=10)
        
        # Create figure with two 3D subplots for ellipsoids
        self.ellipsoid_figure = Figure(figsize=(10, 5), dpi=80)
        
        # Accelerometer subplot
        self.ax_accel = self.ellipsoid_figure.add_subplot(121, projection='3d')
        self.ax_accel.set_title("Accelerometer Variance", fontsize=10)
        self.ax_accel.set_xlabel('X', fontsize=8)
        self.ax_accel.set_ylabel('Y', fontsize=8)
        self.ax_accel.set_zlabel('Z', fontsize=8)
        self.ax_accel.set_box_aspect([1,1,1])
        
        # Gyroscope subplot
        self.ax_gyro = self.ellipsoid_figure.add_subplot(122, projection='3d')
        self.ax_gyro.set_title("Gyroscope Variance", fontsize=10)
        self.ax_gyro.set_xlabel('X', fontsize=8)
        self.ax_gyro.set_ylabel('Y', fontsize=8)
        self.ax_gyro.set_zlabel('Z', fontsize=8)
        self.ax_gyro.set_box_aspect([1,1,1])
        
        self.ellipsoid_canvas = FigureCanvasTkAgg(self.ellipsoid_figure, master=status_frame)
        self.ellipsoid_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.status_text = ttk.Label(status_frame, text="Not Started", font=("Arial", 12))
        self.status_text.pack(pady=5)
        
        self.chunk_info = ttk.Label(status_frame, text="", font=("Arial", 10))
        self.chunk_info.pack()
        
        self.timestamp_info = ttk.Label(status_frame, text="", font=("Arial", 9), foreground="gray")
        self.timestamp_info.pack()
        
        # Add legend for ellipsoids
        legend_frame = ttk.Frame(status_frame)
        legend_frame.pack(pady=5)
        ttk.Label(legend_frame, text="Ellipsoid size represents standard deviation across sensors", 
                 font=("Arial", 8), foreground="gray").pack()
        ttk.Label(legend_frame, text="Smaller = Better sensor agreement | Larger = More variance", 
                 font=("Arial", 8), foreground="gray").pack()
        
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        
        # Initialize ellipsoids with neutral state
        self.draw_ellipsoids(None, None)
    
    def draw_ellipsoids(self, accel_stds, gyro_stds):
        """Draw 3D ellipsoids representing standard deviations of accelerometer and gyroscope data."""
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
            
            # Set axis limits
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
    
    def _draw_single_ellipsoid(self, ax, stds, sensor_type):
        """Draw a single ellipsoid on the given axis."""
        # Create ellipsoid based on standard deviations
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 20)
        
        # Scale factors (add small minimum to avoid zero-size ellipsoids)
        rx = max(stds[0], 0.05)  # X standard deviation
        ry = max(stds[1], 0.05)  # Y standard deviation
        rz = max(stds[2], 0.05)  # Z standard deviation
        
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
        
        # Set axis limits to show ellipsoid properly
        max_radius = max(rx, ry, rz) * 1.5
        ax.set_xlim([-max_radius, max_radius])
        ax.set_ylim([-max_radius, max_radius])
        ax.set_zlim([-max_radius, max_radius])
        
        # Add text showing standard deviations
        ax.text2D(0.05, 0.95, f"σx: {stds[0]:.3f}", transform=ax.transAxes, fontsize=8)
        ax.text2D(0.05, 0.90, f"σy: {stds[1]:.3f}", transform=ax.transAxes, fontsize=8)
        ax.text2D(0.05, 0.85, f"σz: {stds[2]:.3f}", transform=ax.transAxes, fontsize=8)
    
    def select_file(self):
        """Select a single CSV file with SensorID column."""
        file_path = filedialog.askopenfilename(
            title="Select CSV File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if file_path:
            self.file_path = file_path
            self.file_label.config(text=f"File: {Path(file_path).name}")
            self.start_button.config(state="normal")
            # Clear any previous data
            self.sensor_data.clear()
            self.start_timestamps.clear()
            self.sensor_indices = {1: 0, 2: 0, 3: 0}
    
    def load_and_parse_file(self):
        """Load and parse the single CSV file, organizing data by SensorID."""
        try:
            with open(self.file_path, 'r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                
                # Check if required columns exist
                if 'SensorID' not in reader.fieldnames:
                    raise ValueError("CSV file must contain a 'SensorID' column")
                
                # Read all data and organize by SensorID
                for row in reader:
                    sensor_id = int(row['SensorID'])
                    if sensor_id in [1, 2, 3]:
                        # Parse the row data
                        parsed_data = self.parse_row(row)
                        if parsed_data:
                            self.sensor_data[sensor_id].append(parsed_data)
                
                # Check if we have data for all three sensors
                if len(self.sensor_data) != 3:
                    missing = [s for s in [1, 2, 3] if s not in self.sensor_data]
                    raise ValueError(f"Missing data for sensor(s): {missing}")
                
                # Check if all sensors have data
                for sensor_id in [1, 2, 3]:
                    if not self.sensor_data[sensor_id]:
                        raise ValueError(f"No data found for sensor {sensor_id}")
                
                # Get start timestamps for each sensor
                for sensor_id in [1, 2, 3]:
                    if self.sensor_data[sensor_id]:
                        self.start_timestamps[sensor_id] = self.sensor_data[sensor_id][0][0]
                
                # Find common time range
                latest_start = max(self.start_timestamps.values())
                earliest_ends = []
                for sensor_id in [1, 2, 3]:
                    if self.sensor_data[sensor_id]:
                        last_timestamp = self.sensor_data[sensor_id][-1][0]
                        earliest_ends.append(last_timestamp)
                earliest_end = min(earliest_ends)
                
                self.min_common_time = latest_start
                self.max_common_time = earliest_end
                
                # Reset indices
                self.sensor_indices = {1: 0, 2: 0, 3: 0}
                
                # Find starting indices for each sensor
                for sensor_id in [1, 2, 3]:
                    timestamps = [row[0] for row in self.sensor_data[sensor_id]]
                    idx = bisect.bisect_left(timestamps, self.min_common_time)
                    self.sensor_indices[sensor_id] = idx
                
                print(f"Loaded data for 3 sensors from single file")
                print(f"Sensor 1: {len(self.sensor_data[1])} rows")
                print(f"Sensor 2: {len(self.sensor_data[2])} rows")
                print(f"Sensor 3: {len(self.sensor_data[3])} rows")
                print(f"Common time range: {self.min_common_time:.3f} to {self.max_common_time:.3f}")
                
                return True
                
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file: {str(e)}")
            return False
    
    def parse_row(self, row):
        """Parse a row from the CSV file."""
        try:
            # Extract timestamp
            timestamp = float(row.get('timestamp_epoch_s', 0))
            
            # Extract IMU values
            values = []
            # Expected columns in order (after timestamp, VNYMR, SensorID, MCUtime)
            columns = ['Yaw', 'Pitch', 'Roll', 'MagX', 'MagY', 'MagZ', 
                      'AccX', 'AccY', 'AccZ', 'GyrX', 'GyrY', 'GyrZ']
            
            for col in columns:
                if col in row:
                    val = row[col]
                    # Handle potential asterisk in values
                    if '*' in str(val):
                        val = str(val).split('*')[0]
                    try:
                        values.append(float(val))
                    except ValueError:
                        values.append(0.0)
                else:
                    values.append(0.0)
            
            return [timestamp] + values
            
        except Exception as e:
            print(f"Error parsing row: {e}")
            return None
    
    def get_next_samples(self):
        """Get the next sample from each sensor closest to current time."""
        target_time = self.min_common_time + self.current_time_offset
        
        # Check if we've reached the end of data
        if target_time >= self.max_common_time:
            return None
        
        samples = {}
        
        for sensor_id in [1, 2, 3]:
            data = self.sensor_data[sensor_id]
            idx = self.sensor_indices[sensor_id]
            
            # Check if we've run out of data for this sensor
            if idx >= len(data):
                continue
            
            # Find the sample closest to target time
            best_sample = None
            best_time_diff = float('inf')
            best_idx = idx
            
            # Look ahead for the closest sample
            for i in range(idx, min(idx + 20, len(data))):
                timestamp = data[i][0]
                time_diff = abs(timestamp - target_time)
                
                if time_diff < best_time_diff:
                    best_sample = data[i]
                    best_time_diff = time_diff
                    best_idx = i
                
                # Stop if we're past the target time
                if timestamp > target_time + 0.5:  # Allow 0.5 second tolerance
                    break
            
            if best_sample and best_time_diff < 0.5:  # Within 0.5 second tolerance
                samples[sensor_id] = best_sample
                # Only advance index if we found a good sample
                self.sensor_indices[sensor_id] = best_idx + 1
        
        # Return samples even if we don't have all three sensors (graceful degradation)
        if len(samples) >= 2:  # Need at least 2 sensors
            # Fill in missing sensor with last known value if available
            for sensor_id in [1, 2, 3]:
                if sensor_id not in samples and self.sample_windows[sensor_id]:
                    samples[sensor_id] = self.sample_windows[sensor_id][-1]
            return samples
        else:
            return None
    
    def process_samples(self, samples):
        """Process single samples from each sensor and calculate standard deviations."""
        if not samples:
            return True, [], [False, False, False], None, None
        
        # Add new samples to sliding windows
        for sensor_id in [1, 2, 3]:
            if sensor_id in samples:
                self.sample_windows[sensor_id].append(samples[sensor_id])
        
        # Need at least 2 samples from each sensor to calculate meaningful std
        min_window_size = min(len(self.sample_windows[sid]) for sid in [1, 2, 3])
        if min_window_size < 2:
            # Return small default values for initial samples
            return True, [], [False, False, False], [0.01, 0.01, 0.01], [0.01, 0.01, 0.01]
        
        # Define tolerance
        tolerance = 0.5
        
        discrepancies = []
        sensor_issues = [False, False, False]
        
        # Parameter names and indices (1-based after timestamp)
        param_info = {
            'Yaw': 1, 'Pitch': 2, 'Roll': 3,
            'MagX': 4, 'MagY': 5, 'MagZ': 6,
            'AccX': 7, 'AccY': 8, 'AccZ': 9,
            'GyrX': 10, 'GyrY': 11, 'GyrZ': 12
        }
        
        # Calculate standard deviations for accelerometer and gyroscope
        accel_stds = [0, 0, 0]  # X, Y, Z
        gyro_stds = [0, 0, 0]   # X, Y, Z
        
        # For accelerometer (indices 7, 8, 9)
        for i, param_idx in enumerate([7, 8, 9]):
            sensor_values = []
            for sensor_id in [1, 2, 3]:
                if self.sample_windows[sensor_id]:
                    # Get average of recent samples for this sensor
                    recent_values = [sample[param_idx] for sample in self.sample_windows[sensor_id]]
                    sensor_values.append(np.mean(recent_values))
            if len(sensor_values) >= 2:
                accel_stds[i] = np.std(sensor_values) if len(sensor_values) > 1 else 0
        
        # For gyroscope (indices 10, 11, 12)
        for i, param_idx in enumerate([10, 11, 12]):
            sensor_values = []
            for sensor_id in [1, 2, 3]:
                if self.sample_windows[sensor_id]:
                    # Get average of recent samples for this sensor
                    recent_values = [sample[param_idx] for sample in self.sample_windows[sensor_id]]
                    sensor_values.append(np.mean(recent_values))
            if len(sensor_values) >= 2:
                gyro_stds[i] = np.std(sensor_values) if len(sensor_values) > 1 else 0
        
        # Check for discrepancies using current samples only
        for param_name, param_idx in param_info.items():
            values_list = []
            valid_sensors = []
            
            for sensor_id in [1, 2, 3]:
                if sensor_id in samples:
                    values_list.append(samples[sensor_id][param_idx])
                    valid_sensors.append(sensor_id - 1)  # Convert to 0-based index
            
            if len(values_list) >= 2:
                # Check for discrepancy
                std_dev = np.std(values_list)
                max_diff = max(values_list) - min(values_list)
                
                if std_dev > tolerance or max_diff > tolerance * 2:
                    # Pad values_list to always have 3 elements for display
                    display_values = [0, 0, 0]
                    for i, sensor_idx in enumerate(valid_sensors):
                        display_values[sensor_idx] = values_list[i]
                    
                    discrepancies.append({
                        'param': param_name,
                        'std': std_dev,
                        'max_diff': max_diff,
                        'values': display_values,
                        'tolerance': tolerance
                    })
                    
                    # Mark sensors with outliers
                    mean_val = np.mean(values_list)
                    for i, val in enumerate(values_list):
                        if abs(val - mean_val) > tolerance:
                            sensor_issues[valid_sensors[i]] = True
        
        is_match = len(discrepancies) == 0
        return is_match, discrepancies, sensor_issues, accel_stds, gyro_stds
    
    def start_monitoring(self):
        """Start monitoring."""
        if not self.load_and_parse_file():
            return
        
        self.sample_number = 0
        self.current_time_offset = 0
        self.is_running = True
        
        # Clear sample windows
        for sensor_id in [1, 2, 3]:
            self.sample_windows[sensor_id].clear()
        
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.file_label.config(state="disabled")
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_data, daemon=True)
        self.processing_thread.start()
    
    def stop_monitoring(self):
        """Stop monitoring."""
        self.is_running = False
        self.sensor_data.clear()
        self.start_timestamps.clear()
        
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.file_label.config(state="normal")
        self.draw_ellipsoids(None, None)  # Reset ellipsoids to neutral
        self.chunk_info.config(text="Monitoring stopped")
        self.timestamp_info.config(text="")
    
    def process_data(self):
        """Process data in separate thread at 10Hz."""
        samples_without_data = 0  # Track consecutive samples without data
        max_samples_without_data = 50  # Stop after 5 seconds without data
        
        while self.is_running:
            try:
                samples = self.get_next_samples()
                
                if samples is None:
                    samples_without_data += 1
                    if samples_without_data >= max_samples_without_data:
                        # We've reached the end of the data
                        self.root.after(0, self.show_completion)
                        break
                    # Continue incrementing time even without samples
                    self.current_time_offset += self.update_interval
                    time.sleep(self.update_interval)
                    continue
                
                samples_without_data = 0  # Reset counter
                self.sample_number += 1
                is_match, discrepancies, sensor_issues, accel_stds, gyro_stds = self.process_samples(samples)
                
                current_time = self.min_common_time + self.current_time_offset
                
                self.root.after(0, self.update_display, self.sample_number, is_match, 
                              current_time, discrepancies, sensor_issues, 
                              accel_stds, gyro_stds)
                
                self.current_time_offset += self.update_interval
                
                # Sleep for update interval (0.1 seconds for 10Hz)
                time.sleep(self.update_interval)
                
            except Exception as e:
                print(f"Error in process_data: {e}")
                import traceback
                traceback.print_exc()
                self.root.after(0, lambda: messagebox.showerror("Error", str(e)))
                break
        
        self.root.after(0, self.stop_monitoring)
    
    def update_display(self, sample_num, is_match, current_time, discrepancies=None, 
                        sensor_issues=None, accel_stds=None, gyro_stds=None):
            """Update display with sample data and ellipsoid visualizations."""
            # Draw ellipsoids based on standard deviations
            if accel_stds is not None and gyro_stds is not None:
                self.draw_ellipsoids(accel_stds, gyro_stds)
            
            self.chunk_info.config(text=f"Sample {sample_num} (Time: {current_time:.2f}s)")
            self.timestamp_info.config(text=f"Updating at {int(1/self.update_interval)}Hz")
    
    def show_completion(self):
        """Show completion."""
        self.draw_ellipsoids(None, None)  # Reset ellipsoids
        self.status_text.config(text="Completed", foreground="blue")
        self.chunk_info.config(text="Reached end of data")
        total_time = self.current_time_offset
        messagebox.showinfo("Complete", f"Processed {self.sample_number} samples ({total_time:.1f} seconds)")


def main():
    """Main function."""
    root = tk.Tk()
    app = IMUComparatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()