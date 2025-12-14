#!/usr/bin/env python3
"""
IMU Data Comparator GUI - Fixed version with proper microsecond conversion
This version correctly handles 16-digit microsecond timestamps.
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
import threading
import time
from collections import deque
import bisect


class IMUComparatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Data Comparator - Fixed Timestamp Version")
        self.root.geometry("900x750")
        
        # File paths and data
        self.file_paths = []
        self.all_data = []
        self.start_timestamps = []
        self.min_common_time = 0
        self.max_common_time = 0
        self.file_indices = [0, 0, 0]
        
        # Parameters
        self.sampling_rate = 10
        self.time_increment = 0.1
        self.chunk_duration = 1.0
        self.rows_per_chunk = 10
        
        # Tracking
        self.chunk_number = 0
        self.current_time_offset = 0
        self.history = deque(maxlen=50)
        self.is_running = False
        self.processing_thread = None
        
        self.setup_gui()
        
    def setup_gui(self):
        """Setup the GUI components."""
        # File selection frame
        file_frame = ttk.Frame(self.root, padding="10")
        file_frame.grid(row=0, column=0, sticky=(tk.W, tk.E))
        
        ttk.Label(file_frame, text="Select 3 CSV Files:").grid(row=0, column=0, sticky=tk.W)
        self.file_label = ttk.Label(file_frame, text="No files selected", foreground="gray")
        self.file_label.grid(row=0, column=1, padx=10)
        
        ttk.Button(file_frame, text="Browse Files", command=self.select_files).grid(row=0, column=2)
        self.start_button = ttk.Button(file_frame, text="Start Monitoring", command=self.start_monitoring, state="disabled")
        self.start_button.grid(row=0, column=3, padx=5)
        self.stop_button = ttk.Button(file_frame, text="Stop", command=self.stop_monitoring, state="disabled")
        self.stop_button.grid(row=0, column=4)
        
        # Info frame
        info_frame = ttk.Frame(self.root, padding="5")
        info_frame.grid(row=1, column=0, sticky=(tk.W, tk.E))
        
        self.info_label = ttk.Label(info_frame, text="Fixed: Using microsecond conversion for timestamps", 
                                   font=("Arial", 9, "bold"), foreground="green")
        self.info_label.grid(row=0, column=0, padx=10)
        
        # Status indicator frame
        status_frame = ttk.LabelFrame(self.root, text="Current Status", padding="20")
        status_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10, pady=10)
        
        self.status_canvas = tk.Canvas(status_frame, width=150, height=150, bg="white")
        self.status_canvas.pack()
        
        self.status_text = ttk.Label(status_frame, text="Not Started", font=("Arial", 14))
        self.status_text.pack(pady=10)
        
        self.chunk_info = ttk.Label(status_frame, text="", font=("Arial", 10))
        self.chunk_info.pack()
        
        self.timestamp_info = ttk.Label(status_frame, text="", font=("Arial", 9), foreground="gray")
        self.timestamp_info.pack()
        
        # Discrepancy details frame
        discrepancy_frame = ttk.LabelFrame(self.root, text="Discrepancy Details", padding="10")
        discrepancy_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10, pady=10)
        
        # Text widget for discrepancy details
        self.discrepancy_text = tk.Text(discrepancy_frame, width=40, height=10, wrap=tk.WORD, 
                                       font=("Courier", 9))
        self.discrepancy_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Scrollbar for text widget
        scrollbar = ttk.Scrollbar(discrepancy_frame, orient="vertical", command=self.discrepancy_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.discrepancy_text.config(yscrollcommand=scrollbar.set)
        
        # Configure text tags for formatting
        self.discrepancy_text.tag_configure("header", font=("Courier", 9, "bold"))
        self.discrepancy_text.tag_configure("error", foreground="red")
        self.discrepancy_text.tag_configure("success", foreground="green")
        
        # Sensor status frame
        sensor_frame = ttk.LabelFrame(self.root, text="Sensor Status", padding="10")
        sensor_frame.grid(row=3, column=1, sticky=(tk.W, tk.E, tk.N), padx=10, pady=(0, 10))
        
        # Text widget for sensor status
        self.sensor_status_text = tk.Text(sensor_frame, width=40, height=5, wrap=tk.WORD, 
                                          font=("Courier", 10))
        self.sensor_status_text.pack(fill=tk.BOTH, expand=True)
        
        # Configure tags for sensor status
        self.sensor_status_text.tag_configure("sensor_ok", foreground="green", font=("Courier", 10, "bold"))
        self.sensor_status_text.tag_configure("sensor_bad", foreground="red", font=("Courier", 10, "bold"))
        self.sensor_status_text.tag_configure("sensor_name", font=("Courier", 10, "bold"))
        self.sensor_status_text.tag_configure("waiting", foreground="gray", font=("Courier", 10, "italic"))
        
        # Chart frame - now spans full width
        chart_frame = ttk.LabelFrame(self.root, text="Historical Results", padding="10")
        chart_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10, pady=10)
        
        self.figure = Figure(figsize=(11, 4), dpi=80)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Chunk Number")
        self.ax.set_ylabel("Status")
        self.ax.set_ylim(-0.5, 1.5)
        self.ax.set_yticks([0, 1])
        self.ax.set_yticklabels(["RED", "GREEN"])
        self.ax.grid(True, alpha=0.3)
        
        self.canvas = FigureCanvasTkAgg(self.figure, master=chart_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_rowconfigure(4, weight=1)
        self.root.grid_columnconfigure(0, weight=2)
        self.root.grid_columnconfigure(1, weight=1)
        
        # Initialize sensor status display
        self.update_sensor_status(None)
        
        self.draw_status_icon("neutral")
    
    def draw_status_icon(self, status):
        """Draw status icon on the canvas."""
        self.status_canvas.delete("all")
        
        if status == "green":
            self.status_canvas.create_oval(25, 25, 125, 125, fill="#4CAF50", outline="#45a049", width=3)
            self.status_canvas.create_line(45, 75, 65, 95, fill="white", width=5, capstyle="round")
            self.status_canvas.create_line(65, 95, 105, 55, fill="white", width=5, capstyle="round")
            self.status_text.config(text="MATCH", foreground="#4CAF50")
        elif status == "red":
            self.status_canvas.create_oval(25, 25, 125, 125, fill="#f44336", outline="#da190b", width=3)
            self.status_canvas.create_line(50, 50, 100, 100, fill="white", width=5, capstyle="round")
            self.status_canvas.create_line(100, 50, 50, 100, fill="white", width=5, capstyle="round")
            self.status_text.config(text="DISCREPANCY", foreground="#f44336")
        else:
            self.status_canvas.create_oval(25, 25, 125, 125, fill="#e0e0e0", outline="#bdbdbd", width=3)
            self.status_text.config(text="Waiting...", foreground="gray")
    
    def update_chart(self):
        """Update the historical chart."""
        if not self.history:
            return
        
        self.ax.clear()
        chunks = [h[0] for h in self.history]
        statuses = [1 if h[1] else 0 for h in self.history]
        colors = ['#4CAF50' if s == 1 else '#f44336' for s in statuses]
        
        bars = self.ax.bar(chunks, [1]*len(chunks), color=colors, alpha=0.7, edgecolor='black')
        
        self.ax.set_xlabel("Chunk Number (1 second each)")
        self.ax.set_ylabel("Status")
        self.ax.set_ylim(-0.1, 1.1)
        self.ax.set_yticks([0, 1])
        self.ax.set_yticklabels(["RED", "GREEN"])
        self.ax.grid(True, alpha=0.3, axis='x')
        
        green_count = sum(1 for s in statuses if s == 1)
        total_count = len(statuses)
        percentage = (green_count / total_count * 100) if total_count > 0 else 0
        self.ax.set_title(f"Match Rate: {percentage:.1f}% ({green_count}/{total_count} chunks)")
        
        if len(chunks) > 20:
            step = len(chunks) // 20
            self.ax.set_xticks(chunks[::step])
        
        self.canvas.draw()
    
    def select_files(self):
        """Open file dialog to select three CSV files."""
        files = filedialog.askopenfilenames(
            title="Select 3 CSV files",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if len(files) != 3:
            messagebox.showerror("Error", "Please select exactly 3 CSV files")
            return
        
        self.file_paths = list(files)
        file_names = [Path(f).name for f in self.file_paths]
        self.file_label.config(text=", ".join(file_names), foreground="black")
        self.start_button.config(state="normal")
    
    def load_all_data(self):
        """Load and parse all data from CSV files with FIXED timestamp conversion."""
        self.all_data = []
        self.start_timestamps = []
        
        print("\n" + "="*60)
        print("LOADING DATA WITH FIXED MICROSECOND CONVERSION")
        print("="*60)
        
        for file_idx, file_path in enumerate(self.file_paths):
            file_data = []
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    try:
                        # Parse VNYMR row
                        timestamp, values = parse_vnymr_row_fixed(row)
                        if timestamp and values:
                            # CRITICAL FIX: Force microsecond conversion
                            ts_str = str(timestamp).strip()
                            ts_float = float(ts_str)
                            
                            # For 16-digit timestamps, ALWAYS divide by 1,000,000
                            if len(ts_str) >= 15:  # 15+ digits = microseconds
                                timestamp_float = ts_float / 1e6  # THE FIX!
                            elif len(ts_str) >= 13:  # 13-14 digits = milliseconds
                                timestamp_float = ts_float / 1000.0
                            elif len(ts_str) >= 10:  # Old format microseconds
                                timestamp_float = ts_float / 1e6
                            else:
                                timestamp_float = ts_float
                            
                            file_data.append((timestamp_float, values))
                    except Exception as e:
                        continue
            
            if not file_data:
                raise ValueError(f"No valid data found in {file_path}")
            
            file_data.sort(key=lambda x: x[0])
            
            duration = file_data[-1][0] - file_data[0][0]
            print(f"\nFile {file_idx + 1}: {Path(file_path).name}")
            print(f"  Rows: {len(file_data)}")
            print(f"  Duration: {duration:.2f} seconds")
            if duration < 200:
                print(f"  ✅ Duration correct!")
            else:
                print(f"  ❌ Duration wrong - check conversion!")
            
            self.all_data.append(file_data)
            self.start_timestamps.append(file_data[0][0])
        
        # Auto-detect sampling rate
        if self.all_data[0]:
            diffs = []
            for i in range(1, min(200, len(self.all_data[0]))):
                diff = self.all_data[0][i][0] - self.all_data[0][i-1][0]
                if diff < 1.0:
                    diffs.append(diff)
            
            if diffs:
                avg_interval = sum(diffs) / len(diffs)
                self.sampling_rate = 1.0 / avg_interval
                self.time_increment = avg_interval
                self.rows_per_chunk = max(5, int(self.chunk_duration * self.sampling_rate))
                
                print(f"\nDetected: {self.sampling_rate:.1f} Hz sampling")
                print(f"Rows per chunk: {self.rows_per_chunk}")
        
        self.min_common_time = max(data[0][0] for data in self.all_data)
        self.max_common_time = min(data[-1][0] for data in self.all_data)
        total_dur = self.max_common_time - self.min_common_time
        
        print(f"\nTotal duration: {total_dur:.2f} seconds")
        print("="*60)
        
        self.file_indices = [0, 0, 0]
    
    def get_chunk_data(self):
        """Get next chunk of data."""
        chunk_data = []
        chunk_start_time = self.min_common_time + self.current_time_offset
        chunk_end_time = chunk_start_time + self.chunk_duration
        
        if chunk_start_time >= self.max_common_time:
            return None
        
        # Collect data in time window
        file_data_in_window = []
        for file_idx in range(3):
            data_in_window = []
            for timestamp, values in self.all_data[file_idx]:
                if chunk_start_time <= timestamp < chunk_end_time:
                    data_in_window.append((timestamp, values))
            file_data_in_window.append(data_in_window)
        
        min_samples = min(len(data) for data in file_data_in_window)
        
        if min_samples < 3:
            if chunk_end_time < self.max_common_time:
                return []  # Skip chunk
            return None
        
        # Match timestamps
        match_tolerance = 0.1 if self.sampling_rate < 50 else 0.05
        reference_timestamps = [t for t, v in file_data_in_window[0]][:min_samples]
        
        for ref_ts in reference_timestamps:
            data_rows = []
            
            ref_data = next((v for t, v in file_data_in_window[0] if t == ref_ts), None)
            if ref_data is None:
                continue
            data_rows.append((ref_ts, ref_data))
            
            for file_idx in range(1, 3):
                best_match = None
                best_diff = float('inf')
                
                for timestamp, values in file_data_in_window[file_idx]:
                    diff = abs(timestamp - ref_ts)
                    if diff < best_diff:
                        best_diff = diff
                        best_match = (timestamp, values)
                
                if best_match and best_diff < match_tolerance:
                    data_rows.append(best_match)
                else:
                    break
            
            if len(data_rows) == 3:
                chunk_data.append(data_rows)
        
        if len(chunk_data) < 3:
            if chunk_end_time < self.max_common_time:
                return []
            return None
        
        return chunk_data
    
    def compare_chunk_data(self, chunk_data, tolerance_dict=None):
        """Compare chunk data and return details about discrepancies."""
        if tolerance_dict is None:
            # Updated tolerance values from user
            tolerance_dict = {
                # Orient - degrees
                0: 0.01,   # Yaw
                1: 0.01,   # Pitch
                2: 0.01,   # Roll
                # Mag - Gauss
                3: 1000,   # Mag X
                4: 1000,   # Mag Y
                5: 1000,   # Mag Z
                # Accel - m/s^2
                6: 1.6,   # Accel X
                7: 1.6,   # Accel Y
                8: 1.6,   # Accel Z
                # Gyro - rad/s
                9: 0.03,   # Gyro X
                10: 0.03,  # Gyro Y
                11: 0.03,  # Gyro Z
            }
        
        # Parameter names for reporting
        param_names = [
            "Yaw (deg)", "Pitch (deg)", "Roll (deg)",
            "Mag X (G)", "Mag Y (G)", "Mag Z (G)",
            "Accel X (m/s²)", "Accel Y (m/s²)", "Accel Z (m/s²)",
            "Gyro X (rad/s)", "Gyro Y (rad/s)", "Gyro Z (rad/s)"
        ]
        
        discrepancies = []
        all_match = True
        sensor_issues = [False, False, False]  # Track which sensors have issues
        
        # Check each row set in the chunk
        for row_idx, row_set in enumerate(chunk_data):
            values_arrays = [row[1] for row in row_set]
            
            for param_idx in range(min(len(v) for v in values_arrays)):
                values = [v[param_idx] for v in values_arrays]
                tolerance = tolerance_dict.get(param_idx, 0.01)
                max_val = max(values)
                min_val = min(values)
                max_diff = max_val - min_val
                
                if max_diff > tolerance:
                    all_match = False
                    param_name = param_names[param_idx] if param_idx < len(param_names) else f"Param {param_idx}"
                    
                    # Identify which sensors are outliers
                    mean_val = sum(values) / len(values)
                    for i, val in enumerate(values):
                        if abs(val - mean_val) > tolerance / 2:
                            sensor_issues[i] = True
                    
                    discrepancies.append({
                        'param': param_name,
                        'param_idx': param_idx,
                        'values': values,
                        'max_diff': max_diff,
                        'tolerance': tolerance,
                        'row_in_chunk': row_idx,
                        'outlier_sensors': [i for i, val in enumerate(values) 
                                          if val == max_val or val == min_val]
                    })
        
        return all_match, discrepancies, sensor_issues
    
    def start_monitoring(self):
        """Start monitoring process."""
        if not self.file_paths:
            messagebox.showerror("Error", "Please select files first")
            return
        
        try:
            self.info_label.config(text="Loading data with FIXED microsecond conversion...")
            self.root.update()
            self.load_all_data()
            
            self.chunk_number = 0
            self.current_time_offset = 0
            self.history.clear()
            self.is_running = True
            
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.file_label.config(state="disabled")
            rate_text = f"Sampling rate: {self.sampling_rate:.1f}Hz | Chunk duration: 1 second | File status: OK"
            self.info_label.config(text=rate_text)
            
            self.processing_thread = threading.Thread(target=self.process_data, daemon=True)
            self.processing_thread.start()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start: {e}")
            self.stop_monitoring()
    
    def stop_monitoring(self):
        """Stop monitoring."""
        self.is_running = False
        self.all_data = []
        self.start_timestamps = []
        
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.file_label.config(state="normal")
        self.draw_status_icon("neutral")
        self.chunk_info.config(text="Monitoring stopped")
        self.timestamp_info.config(text="")
    
    def process_data(self):
        """Process data in separate thread."""
        while self.is_running:
            try:
                chunk_data = self.get_chunk_data()
                
                if chunk_data is None:
                    self.root.after(0, self.show_completion)
                    break
                elif len(chunk_data) == 0:
                    self.current_time_offset += self.chunk_duration
                    continue
                
                self.chunk_number += 1
                is_match, discrepancies, sensor_issues = self.compare_chunk_data(chunk_data)
                
                start_time = self.min_common_time + self.current_time_offset
                end_time = start_time + self.chunk_duration
                
                self.root.after(0, self.update_display, self.chunk_number, is_match, 
                              start_time, end_time, discrepancies, sensor_issues)
                self.current_time_offset += self.chunk_duration
                
                time.sleep(1)
                
            except Exception as e:
                print(f"Error: {e}")
                self.root.after(0, lambda: messagebox.showerror("Error", str(e)))
                break
        
        self.root.after(0, self.stop_monitoring)
    
    def update_sensor_status(self, sensor_issues):
        """Update the sensor status display."""
        self.sensor_status_text.delete('1.0', tk.END)
        
        if sensor_issues is None:
            # Initial state - waiting
            self.sensor_status_text.insert(tk.END, "Waiting for data...\n\n", "waiting")
            for i in range(1, 4):
                self.sensor_status_text.insert(tk.END, f"Sensor {i}: ", "sensor_name")
                self.sensor_status_text.insert(tk.END, "---\n", "waiting")
        else:
            # Update with actual status
            for i in range(3):
                self.sensor_status_text.insert(tk.END, f"Sensor {i+1}: ", "sensor_name")
                if sensor_issues[i]:
                    self.sensor_status_text.insert(tk.END, "✗ MISMATCH\n", "sensor_bad")
                else:
                    self.sensor_status_text.insert(tk.END, "✓ OK\n", "sensor_ok")
            
            # Add summary
            self.sensor_status_text.insert(tk.END, "\n")
            if any(sensor_issues):
                bad_sensors = [f"Sensor {i+1}" for i, bad in enumerate(sensor_issues) if bad]
                self.sensor_status_text.insert(tk.END, f"Issues: {', '.join(bad_sensors)}", "sensor_bad")
            else:
                self.sensor_status_text.insert(tk.END, "All sensors matching", "sensor_ok")
    
    def update_display(self, chunk_num, is_match, start_time, end_time, discrepancies=None, sensor_issues=None):
        """Update display with discrepancy details."""
        self.history.append((chunk_num, is_match))
        self.draw_status_icon("green" if is_match else "red")
        self.chunk_info.config(text=f"Chunk {chunk_num} (Time: {start_time:.1f}s - {end_time:.1f}s)")
        self.timestamp_info.config(text=f"Sampling at {self.sampling_rate:.1f}Hz")
        
        # Update sensor status
        if sensor_issues is None:
            sensor_issues = [False, False, False]
        self.update_sensor_status(sensor_issues)
        
        # Update discrepancy text box
        self.discrepancy_text.delete('1.0', tk.END)
        
        if is_match:
            self.discrepancy_text.insert(tk.END, f"Chunk {chunk_num}: ", "header")
            self.discrepancy_text.insert(tk.END, "ALL MATCH ✓\n", "success")
            self.discrepancy_text.insert(tk.END, "-"*35 + "\n")
            self.discrepancy_text.insert(tk.END, "All sensors within tolerance.\n")
        else:
            self.discrepancy_text.insert(tk.END, f"Chunk {chunk_num}: ", "header")
            self.discrepancy_text.insert(tk.END, "DISCREPANCY ✗\n", "error")
            self.discrepancy_text.insert(tk.END, "-"*35 + "\n")
            
            if discrepancies:
                # Group discrepancies by parameter
                params_with_issues = {}
                for disc in discrepancies:
                    param = disc['param']
                    if param not in params_with_issues:
                        params_with_issues[param] = []
                    params_with_issues[param].append(disc)
                
                # Display each parameter with issues
                self.discrepancy_text.insert(tk.END, "Exceeded Thresholds:\n\n", "header")
                
                for param, issues in params_with_issues.items():
                    # Get the worst case for this parameter
                    worst = max(issues, key=lambda x: x['max_diff'])
                    
                    self.discrepancy_text.insert(tk.END, f"• {param}\n", "error")
                    self.discrepancy_text.insert(tk.END, f"  Tolerance: {worst['tolerance']:.4f}\n")
                    self.discrepancy_text.insert(tk.END, f"  Max diff:  {worst['max_diff']:.4f}\n")
                    self.discrepancy_text.insert(tk.END, f"  Values: [{worst['values'][0]:.4f}, "
                                                        f"{worst['values'][1]:.4f}, "
                                                        f"{worst['values'][2]:.4f}]\n")
                    
                    # Show which sensors are outliers
                    # if 'outlier_sensors' in worst:
                    #     outlier_names = [f"Sensor {i+1}" for i in worst['outlier_sensors']]
                        # self.discrepancy_text.insert(tk.END, f"  Outliers: {', '.join(outlier_names)}\n")
                    
                    self.discrepancy_text.insert(tk.END, "\n")
        
        # Auto-scroll to bottom
        self.discrepancy_text.see(tk.END)
        
        self.update_chart()
    
    def show_completion(self):
        """Show completion."""
        self.draw_status_icon("neutral")
        self.status_text.config(text="Completed", foreground="blue")
        self.chunk_info.config(text="Reached end of data")
        total_time = self.current_time_offset
        messagebox.showinfo("Complete", f"Processed {self.chunk_number} chunks ({total_time:.1f} seconds)")


def parse_vnymr_row_fixed(row):
    """Parse VNYMR row - FIXED version."""
    parsed_values = []
    timestamp = None
    
    # Check format
    if len(row) >= 2 and ('$VNYMR' in row[1] or 'VNYMR' in row[1]):
        # New format: timestamp in col 0, $VNYMR in col 1
        timestamp = row[0].strip()
        start_col = 3  # Skip $VNYMR and ignored column
    elif len(row) >= 2 and ('$VNYMR' in row[0] or 'VNYMR' in row[0]):
        # Old format: $VNYMR in col 0, timestamp in col 1
        timestamp = row[1].strip()
        start_col = 2
    else:
        return None, []
    
    # Parse values
    for i in range(start_col, len(row)):
        val = row[i]
        if '*' in val:
            val = val.split('*')[0]
        try:
            parsed_values.append(float(val))
        except ValueError:
            parsed_values.append(0.0)
    
    return timestamp, parsed_values


def main():
    """Main function."""
    root = tk.Tk()
    app = IMUComparatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()