import tkinter as tk
from tkinter import ttk
from pymavlink import mavutil
import threading
import time
import queue


class DroneGUI:
    def __init__(self, root):
        self.root = root
        self.setup_ui()
        self.setup_data()
        self.connect_to_sitl()

    def setup_ui(self):
        """Initialize the user interface"""
        self.root.title("🛰️ Drone Monitor Pro")
        self.root.geometry("450x400")
        self.root.resizable(True, True)

        # Configure style
        style = ttk.Style()
        style.theme_use("clam")

        # Main container
        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # Title
        title_label = ttk.Label(
            main_frame, text="🛰️ Drone Monitor Pro", font=("Arial", 16, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))

        # Connection status frame
        status_frame = ttk.LabelFrame(
            main_frame, text="Connection Status", padding="10"
        )
        status_frame.grid(
            row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 15)
        )
        status_frame.columnconfigure(1, weight=1)

        self.status_indicator = ttk.Label(
            status_frame, text="●", font=("Arial", 16), foreground="orange"
        )
        self.status_indicator.grid(row=0, column=0, padx=(0, 10))

        self.status_text = ttk.Label(
            status_frame, text="Connecting to SITL...", font=("Arial", 11)
        )
        self.status_text.grid(row=0, column=1, sticky=tk.W)

        # Telemetry data frame
        data_frame = ttk.LabelFrame(main_frame, text="Telemetry Data", padding="10")
        data_frame.grid(
            row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 15)
        )
        data_frame.columnconfigure(1, weight=1)

        # Data labels
        self.create_data_row(data_frame, 0, "Flight Mode:", "mode_value", "--")
        self.create_data_row(data_frame, 1, "Arm Status:", "arm_value", "--")
        self.create_data_row(data_frame, 2, "Altitude:", "alt_value", "-- m")
        self.create_data_row(data_frame, 3, "Battery:", "battery_value", "--")
        self.create_data_row(data_frame, 4, "GPS Satellites:", "gps_value", "--")
        self.create_data_row(data_frame, 5, "Ground Speed:", "speed_value", "-- m/s")

        # Control buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, columnspan=2, pady=(15, 0))

        self.reconnect_btn = ttk.Button(
            button_frame, text="🔄 Reconnect", command=self.reconnect
        )
        self.reconnect_btn.grid(row=0, column=0, padx=(0, 10))

        self.exit_btn = ttk.Button(button_frame, text="❌ Exit", command=self.safe_exit)
        self.exit_btn.grid(row=0, column=1)

        # Status bar
        self.status_bar = ttk.Label(
            main_frame, text="Ready", relief=tk.SUNKEN, anchor=tk.W
        )
        self.status_bar.grid(
            row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(15, 0)
        )

    def create_data_row(self, parent, row, label_text, value_attr, default_value):
        """Create a label-value pair row"""
        label = ttk.Label(parent, text=label_text, font=("Arial", 10, "bold"))
        label.grid(row=row, column=0, sticky=tk.W, pady=2)

        value = ttk.Label(parent, text=default_value, font=("Arial", 10))
        value.grid(row=row, column=1, sticky=tk.W, padx=(20, 0), pady=2)

        setattr(self, value_attr, value)

    def setup_data(self):
        """Initialize data structures"""
        self.mav = None
        self.connected = False
        self.running = True
        self.update_queue = queue.Queue()

        # Start UI update thread
        self.root.after(100, self.process_updates)

    def connect_to_sitl(self):
        """Connect to SITL in a separate thread"""
        threading.Thread(target=self._connect_worker, daemon=True).start()

    def _connect_worker(self):
        """Worker thread for SITL connection"""
        try:
            self.update_queue.put(("status", "Connecting to SITL...", "orange"))
            self.update_queue.put(("status_bar", "Attempting connection..."))

            # Try multiple connection strings
            connection_strings = [
                "udp:127.0.0.1:14551",
                "udp:127.0.0.1:14550",
                "tcp:127.0.0.1:5762",
            ]

            for conn_str in connection_strings:
                try:
                    print(f"Trying connection: {conn_str}")
                    self.mav = mavutil.mavlink_connection(conn_str, timeout=5)
                    break
                except Exception as e:
                    print(f"Failed to connect to {conn_str}: {e}")
                    continue

            if not self.mav:
                raise Exception("Could not connect to any SITL instance")

            self.update_queue.put(("status", "Waiting for heartbeat...", "orange"))

            # Wait for heartbeat with timeout
            start_time = time.time()
            while time.time() - start_time < 10:  # 10 second timeout
                try:
                    self.mav.wait_heartbeat(timeout=1)
                    break
                except:
                    continue
            else:
                raise Exception("No heartbeat received within 10 seconds")

            self.connected = True
            self.update_queue.put(("status", "Connected ✓", "green"))
            self.update_queue.put(
                ("status_bar", f"Connected to system {self.mav.target_system}")
            )

            print(
                f"✅ Connected to system {self.mav.target_system}, component {self.mav.target_component}"
            )

            # Start message reading loop
            threading.Thread(target=self._message_loop, daemon=True).start()

        except Exception as e:
            print(f"Connection error: {e}")
            self.update_queue.put(("status", f"Connection failed ❌", "red"))
            self.update_queue.put(("status_bar", f"Error: {str(e)}"))

    def _message_loop(self):
        """Main message processing loop"""
        last_heartbeat = time.time()

        while self.running and self.connected:
            try:
                msg = self.mav.recv_match(timeout=1)
                if not msg:
                    # Check for connection timeout
                    if time.time() - last_heartbeat > 10:
                        self.connected = False
                        self.update_queue.put(("status", "Connection lost ❌", "red"))
                        self.update_queue.put(("status_bar", "Connection timeout"))
                        break
                    continue

                self._process_message(msg)

                if msg.get_type() == "HEARTBEAT":
                    last_heartbeat = time.time()

            except Exception as e:
                print(f"Message loop error: {e}")
                time.sleep(1)

    def _process_message(self, msg):
        """Process incoming MAVLink messages"""
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            # Arm status
            base_mode = msg.base_mode
            is_armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            arm_text = "🟢 ARMED" if is_armed else "🔴 DISARMED"
            self.update_queue.put(("data", "arm_value", arm_text))

            # Flight mode
            if hasattr(self.mav, "mode_mapping") and self.mav.mode_mapping():
                mode_map = self.mav.mode_mapping()
                custom_mode = msg.custom_mode
                mode = next(
                    (k for k, v in mode_map.items() if v == custom_mode), "Unknown"
                )
                self.update_queue.put(("data", "mode_value", f"🛩️ {mode}"))

        elif msg_type == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0  # mm to meters
            self.update_queue.put(("data", "alt_value", f"📏 {alt:.1f} m"))

        elif msg_type == "VFR_HUD":
            # Ground speed
            speed = msg.groundspeed
            self.update_queue.put(("data", "speed_value", f"🏃 {speed:.1f} m/s"))

        elif msg_type == "SYS_STATUS":
            # Battery voltage
            voltage = msg.voltage_battery / 1000.0  # mV to V
            self.update_queue.put(("data", "battery_value", f"🔋 {voltage:.1f}V"))

        elif msg_type == "GPS_RAW_INT":
            # GPS satellite count
            satellites = msg.satellites_visible
            self.update_queue.put(("data", "gps_value", f"🛰️ {satellites}"))

    def process_updates(self):
        """Process queued UI updates"""
        try:
            while True:
                update_type, *args = self.update_queue.get_nowait()

                if update_type == "status":
                    text, color = args
                    self.status_text.config(text=text)
                    self.status_indicator.config(foreground=color)

                elif update_type == "status_bar":
                    self.status_bar.config(text=args[0])

                elif update_type == "data":
                    attr_name, value = args
                    if hasattr(self, attr_name):
                        getattr(self, attr_name).config(text=value)

        except queue.Empty:
            pass
        finally:
            if self.running:
                self.root.after(100, self.process_updates)

    def reconnect(self):
        """Reconnect to SITL"""
        self.connected = False
        if self.mav:
            try:
                self.mav.close()
            except:
                pass
        self.mav = None

        # Reset UI
        self.mode_value.config(text="--")
        self.arm_value.config(text="--")
        self.alt_value.config(text="-- m")
        self.battery_value.config(text="--")
        self.gps_value.config(text="--")
        self.speed_value.config(text="-- m/s")

        self.connect_to_sitl()

    def safe_exit(self):
        """Safely close the application"""
        self.running = False
        self.connected = False

        if self.mav:
            try:
                self.mav.close()
            except:
                pass

        self.root.quit()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = DroneGUI(root)

    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.safe_exit)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.safe_exit()
