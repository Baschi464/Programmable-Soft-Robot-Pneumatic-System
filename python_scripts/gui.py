import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
from .communication import SerialCommunication
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import time
import json
import os
from bisect import bisect_left
import tempfile
import math
import csv


class PneumaticGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Pneumatic System Control")
        
        # Get screen dimensions
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        
        # Calculate window size (80% of screen)
        window_width = int(screen_width * 0.9)
        window_height = int(screen_height * 0.9)
        
        # Calculate center position
        center_x = int((screen_width - window_width) / 2)
        center_y = int((screen_height - window_height) / 2) - 40  # Slightly higher for taskbar
        
        self.root.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')

        self.comm = None

        # One-time warning flag to avoid repeated popups when hardware is missing.
        self.hardware_disconnected_warned = False

        # One-time warning flags (per channel) for missing sensors (Arduino sends "ERR").
        self.sensor_err_warned = [False] * 20
        
        self.num_channels = 8 # Default number of channels
        
        # Data storage: list of dicts for each channel
        # Each dict: {'t': [], 'target': [], 'actual': []}
        self.channel_data = [{'t': [], 'target': [], 'actual': []} for _ in range(20)]
        self.latest_pressures = [0.0] * 20 # Store latest actual values for E-Stop
        self.current_targets = [0.0] * 20  # Store current target values for plotting
        
        self.start_time = time.time()
        
        # --- Pause State ---
        self.is_paused = False
        self.total_paused_time = 0.0
        self.last_pause_timestamp = 0.0
        
        # --- Program Action State ---
        self.current_program = {i: [] for i in range(20)} # {channel_idx: [(t, p), ...]}

        # --- Action Queue State ---
        self.action_queue = []        # List of action names
        self.current_action = None    # Name of currently executing action
        self.current_action_duration = 0.0 # Duration of current action
        self.action_start_time = 0

        # --- Post-action hold: keep last targets alive for N seconds after action ends ---
        self.post_action_hold_until = 0.0   # time.time() when hold expires (0 = inactive)

        # Tracks the currently selected library action (Program tab)
        self.selected_library_action_name = None

        # Program tab action-name editing
        self.action_name_var = tk.StringVar()

        # Program tab: cycle-picking state for overlapping keypoints
        self._prog_pick_last_xy = None          # (x_px, y_px)
        self._prog_pick_candidates_key = None   # tuple of (ch_idx, pt_idx)
        self._prog_pick_cycle_index = 0
        
        # Path to Action Library (json files storage)
        self.action_lib_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "action_library"))
        if not os.path.exists(self.action_lib_path):
            try:
                os.makedirs(self.action_lib_path)
            except OSError:
                print(f"Warning: Could not create {self.action_lib_path}")
        
        # Default Y-axis limits: (min, max)
        self.y_limits = [( -100, 100 ) for _ in range(self.num_channels)]
        self.x_timespan = 10.0 # Default 10 seconds
        self.load_default_settings()

        # Live history retention (seconds). Keeps memory and plotting bounded.
        self.history_seconds = 60.0
        self.history_slider_resolution = 0.05

        # --- Tabs Setup ---
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(expand=True, fill="both", padx=5, pady=5)

        self.tab_comm = ttk.Frame(self.notebook)
        self.tab_program = ttk.Frame(self.notebook)
        self.tab_live = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_comm, text="1. Communication")
        self.notebook.add(self.tab_program, text="2. Program Action")
        self.notebook.add(self.tab_live, text="3. Live Control")
        
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_change)

        self.setup_communication_tab()
        self.setup_program_tab()
        self.setup_live_control_tab()

        # Initial port scan
        self.refresh_ports()
        
        # Load Library
        self.refresh_action_library()
        
        # Start the data update loop
        self.update_loop()

    def on_tab_change(self, event):
        """Callback for tab switching."""
        selected_tab = self.notebook.select()
        if selected_tab == str(self.tab_live):
            self.refresh_action_library()
            # Warn (once) if user enters Live Control while not connected.
            if (not self._is_hardware_connected()) and not self.hardware_disconnected_warned:
                self._show_hardware_disconnected_warning()

    def add_help_button(self, parent):
        """Adds a Help button to the top-right corner of the given parent frame."""
        btn = ttk.Button(parent, text="Help", command=self.open_manual)
        btn.place(relx=1.0, rely=0.0, x=-2, y=2, anchor="ne")

    def open_manual(self):
        """Opens manual.pdf located in the same directory as the action library folder."""
        # action_lib_path points to ".../action_library"
        # We look for ".../manual.pdf" (sibling of the folder)
        parent_dir = os.path.dirname(self.action_lib_path)
        manual_path = os.path.join(parent_dir, "manual.pdf")

        if not os.path.exists(manual_path):
            # Fallback: check inside the action library folder? 
            # User said "same path AS the action library FOLDER". 
            # Ambiguous: could be inside the folder too.
            alt_path = os.path.join(self.action_lib_path, "manual.pdf")
            if os.path.exists(alt_path):
                manual_path = alt_path
            else:
                messagebox.showerror("Error", "manual.pdf not found.")
                return

        try:
            os.startfile(manual_path)
        except Exception as e:
            messagebox.showerror("Error", f"Could not open manual: {e}")

    def _is_hardware_connected(self):
        """Returns True if we have an open serial connection."""
        if self.comm is None:
            return False
        if getattr(self.comm, 'disconnected', False):
            return False
        ser = getattr(self.comm, 'ser', None)
        return bool(ser and getattr(ser, 'is_open', False))

    def _warn_sensor_err_once(self, channel_idx: int):
        """Warns once when a sensor channel reports ERR; keeps GUI usable."""
        if channel_idx < 0:
            return
        if channel_idx >= len(self.sensor_err_warned):
            self.sensor_err_warned.extend([False] * (channel_idx + 1 - len(self.sensor_err_warned)))

        if self.sensor_err_warned[channel_idx]:
            return
        self.sensor_err_warned[channel_idx] = True

        try:
            messagebox.showwarning(
                "Sensor Disconnected",
                f"Pressure sensor on channel {channel_idx + 1} is disconnected or not responding (received 'ERR').\n\n"
                "The GUI will keep running; this channel's actual pressure will be unavailable.",
            )
        except Exception:
            # Avoid crashing the update loop if a modal can't be shown.
            pass

    def update_loop(self):
        """Periodic loop to read data and update graphs."""

        try:
            if self.is_paused:
                # In pause mode, we don't fetch new data or update graphs automatically
                # We just keep the loop alive to check for resume
                self.root.after(100, self.update_loop)
                return

            current_time = time.time() - self.start_time - self.total_paused_time

            # Process Action Queue
            self.process_action_queue()

            # 1. Fetch/Generate Data
            new_values = None

            if self.comm:
                raw_data = self.comm.read_latest_response()

                # If the serial layer detected a physical disconnect, tear down the GUI connection.
                if getattr(self.comm, 'disconnected', False):
                    self._handle_hardware_disconnected(getattr(self.comm, 'disconnected_reason', None))
                    raw_data = None

                if raw_data and raw_data.startswith("ACT:"):
                    # Expected format: ACT:val1,val2,...
                    payload = raw_data[4:].strip()
                    if payload:
                        parts = payload.split(',')
                        parsed = []
                        for i, p in enumerate(parts):
                            token = p.strip()
                            if not token:
                                # Empty token -> treat as missing
                                parsed.append(float('nan'))
                                self._warn_sensor_err_once(i)
                                continue
                            if token.upper() == "ERR":
                                parsed.append(float('nan'))
                                self._warn_sensor_err_once(i)
                                continue
                            try:
                                parsed.append(float(token))
                            except ValueError:
                                parsed.append(float('nan'))
                                self._warn_sensor_err_once(i)
                        new_values = parsed

            # 2. Store Data & Update Graphs ONLY if new data received
            if new_values:
                # Update latest pressures for E-Stop/Pause
                for i, val in enumerate(new_values):
                    if i < len(self.latest_pressures):
                        # Keep latest_pressures numeric for formatting in Pause/E-Stop.
                        if isinstance(val, (int, float)) and math.isfinite(val):
                            self.latest_pressures[i] = float(val)

                # Ensure channel_data has enough slots
                while len(self.channel_data) < len(new_values):
                    self.channel_data.append({'t': [], 'target': [], 'actual': []})

                for i, val in enumerate(new_values):
                    if i < len(self.channel_data):
                        self.channel_data[i]['t'].append(current_time)
                        self.channel_data[i]['actual'].append(val)
                        # Use the tracked current target
                        tgt = self.current_targets[i] if i < len(self.current_targets) else 0.0
                        self.channel_data[i]['target'].append(tgt)

                # Trim stored history (keep last self.history_seconds)
                cutoff = current_time - self.history_seconds
                if cutoff > 0:
                    max_channels = min(self.num_channels, len(self.channel_data))
                    for ch in range(max_channels):
                        t_list = self.channel_data[ch]['t']
                        if not t_list:
                            continue
                        idx = bisect_left(t_list, cutoff)
                        if idx > 0:
                            self.channel_data[ch]['t'] = self.channel_data[ch]['t'][idx:]
                            self.channel_data[ch]['actual'] = self.channel_data[ch]['actual'][idx:]
                            self.channel_data[ch]['target'] = self.channel_data[ch]['target'][idx:]

                # 3. Update Graphs
                if self.plot_handles:
                    # Determine X-Axis limits (Moving Window)
                    if current_time > self.x_timespan:
                        x_min = current_time - self.x_timespan
                        x_max = current_time
                    else:
                        x_min = 0
                        x_max = self.x_timespan

                    for ch_idx, handles in self.plot_handles.items():
                        ax = handles['ax']
                        line_actual = handles['line_actual']
                        line_target = handles['line_target']

                        # Update X-Axis
                        ax.set_xlim(x_min, x_max)

                        # Update Lines
                        if ch_idx < len(self.channel_data):
                            t_data = self.channel_data[ch_idx]['t']
                            act_data = self.channel_data[ch_idx]['actual']
                            tgt_data = self.channel_data[ch_idx]['target']

                            line_actual.set_data(t_data, act_data)
                            line_target.set_data(t_data, tgt_data)

                    self.canvas.draw_idle()
        except Exception as e:
            print(f"ERROR in update_loop: {e}")

        # Schedule next update (e.g., 100ms = 10Hz)
        self.root.after(100, self.update_loop)

    def _show_hardware_disconnected_warning(self):
        """Shows a single popup warning about missing hardware."""
        if self.hardware_disconnected_warned:
            return
        self.hardware_disconnected_warned = True
        try:
            messagebox.showwarning("Hardware Disconnected", "Hardware disconnected")
        except Exception:
            # In case a modal popup can't be shown (rare Tk timing issue)
            pass

    def _set_disconnected_ui_state(self):
        """Updates Communication tab widgets to the disconnected state."""
        if hasattr(self, 'btn_connect'):
            self.btn_connect.config(text="Connect")
        if hasattr(self, 'lbl_status'):
            self.lbl_status.config(text="Disconnected", foreground="red")
        if hasattr(self, 'port_combo'):
            self.port_combo.config(state="readonly")
        if hasattr(self, 'btn_refresh'):
            self.btn_refresh.config(state="normal")
        if hasattr(self, 'btn_send'):
            self.btn_send.config(state="disabled")

    def _handle_hardware_disconnected(self, reason=None):
        """Forcefully disconnects after a physical unplug and warns once."""
        # Close the serial port if still present.
        try:
            if self.comm is not None:
                self.comm.close()
        except Exception:
            pass
        self.comm = None

        # Update UI and warn once.
        self._set_disconnected_ui_state()
        self._show_hardware_disconnected_warning()

    def process_action_queue(self):
        """Manages the FIFO queue execution."""
        now = time.time()
        
        # Check if an action is currently running
        if self.current_action:
            elapsed = now - self.action_start_time
            
            # --- EXECUTE ACTION LOGIC ---
            # Calculate target pressure for each channel at current time 'elapsed'
            current_targets = []

            # Action data is loaded ONCE when the action starts and cached in memory.
            if not hasattr(self, 'current_action_data') or self.current_action_data is None:
                print(f"Error: No cached data for action '{self.current_action}'. Aborting.")
                self.current_action = None
                self.current_action_channels = {}
                return

            # Preprocessed per-channel arrays are built once at action start.
            # Execution uses per-channel indices to avoid scanning all points each tick.
            times_by_channel = getattr(self, 'current_action_times', None)
            pressures_by_channel = getattr(self, 'current_action_pressures', None)
            indices_by_channel = getattr(self, 'current_action_indices', None)

            if not times_by_channel or not pressures_by_channel or not indices_by_channel:
                print(f"Error: No preprocessed keypoints for action '{self.current_action}'. Aborting.")
                self.current_action = None
                self.current_action_data = None
                self.current_action_channels = {}
                self.current_action_times = []
                self.current_action_pressures = []
                self.current_action_indices = []
                return

            for i in range(self.num_channels):
                ch_times = times_by_channel[i] if i < len(times_by_channel) else []
                ch_pressures = pressures_by_channel[i] if i < len(pressures_by_channel) else []

                if not ch_times:
                    # Channel has no keypoints → send "off" token
                    current_targets.append("off")
                    continue

                idx = indices_by_channel[i] if i < len(indices_by_channel) else 0

                # Advance index while the next keypoint time has been reached
                # (Step function: hold the last pressure until next time.)
                while (idx + 1) < len(ch_times) and ch_times[idx + 1] <= elapsed:
                    idx += 1

                if i < len(indices_by_channel):
                    indices_by_channel[i] = idx

                p_val = ch_pressures[idx] if idx < len(ch_pressures) else 0.0
                if isinstance(p_val, str):
                    # Toggle keypoint: "off" or "on" (on resumes at 0.0)
                    current_targets.append("off" if p_val == "off" else "0.0")
                else:
                    current_targets.append(p_val)

            # Send ONE targets command per tick (all channels)
            # Format: <p1,p2,p3,...>  — channels with no keypoints send "off"
            cmd_str = "<" + ",".join(
                [str(p) if isinstance(p, str) else f"{p:.1f}" for p in current_targets]
            ) + ">"
            if self.comm:
                ok = self.comm.send_command(cmd_str)
                if not ok and getattr(self.comm, 'disconnected', False):
                    self._handle_hardware_disconnected(getattr(self.comm, 'disconnected_reason', None))

            # Update local target storage for plotting
            self.parse_and_store_target(cmd_str)

            # Check if finished
            if elapsed >= self.current_action_duration:
                # Flush the final targets into the plot data NOW, before all-off
                # overwrites current_targets with NaN.
                current_time = time.time() - self.start_time - self.total_paused_time
                for i in range(min(self.num_channels, len(self.channel_data))):
                    tgt = self.current_targets[i] if i < len(self.current_targets) else 0.0
                    self.channel_data[i]['t'].append(current_time)
                    self.channel_data[i]['target'].append(tgt)
                    # Use last known actual value so the actual line isn't broken
                    last_actual = self.channel_data[i]['actual'][-1] if self.channel_data[i]['actual'] else float('nan')
                    self.channel_data[i]['actual'].append(last_actual)

                print(f"Action '{self.current_action}' finished. Holding targets for 2s...")
                self.current_action = None
                self.current_action_data = None # Clear cache
                self.current_action_channels = {}
                self.current_action_times = []
                self.current_action_pressures = []
                self.current_action_indices = []
                
                # Start a 2-second hold: channels keep their last targets.
                # all-off will be sent when the timer expires (see below).
                self.post_action_hold_until = time.time() + 2.0
                
                # Remove from UI Queue (Head is at index 0)
                if self.queue_listbox.size() > 0:
                    self.queue_listbox.delete(0)
                
                # Remove from internal list
                if self.action_queue:
                    self.action_queue.pop(0)
        
        # --- Post-action hold timer: send all-off once the hold expires ---
        if self.post_action_hold_until > 0 and not self.current_action:
            if time.time() >= self.post_action_hold_until:
                off_cmd = "<" + ",".join(["off"] * self.num_channels) + ">"
                if self.comm:
                    self.comm.send_command(off_cmd)
                self.parse_and_store_target(off_cmd)
                self.post_action_hold_until = 0.0
                print("Post-action hold expired. Channels off.")
            else:
                # Keep re-sending current targets so Arduino doesn't time-out
                # (current_targets still holds the last action values)
                cmd_str = "<" + ",".join(
                    [str(p) if isinstance(p, str) else f"{p:.1f}"
                     for p in self.current_targets[:self.num_channels]]
                ) + ">"
                if self.comm:
                    self.comm.send_command(cmd_str)
        
        # If no action is running and hold timer is done (or queue has a new action to override), check if there is one waiting
        if not self.current_action and self.action_queue:
            # Start the next action (FIFO -> Index 0)
            next_action = self.action_queue[0]
            
            # Load action details from disk
            action_data = self.get_action_details(next_action)
            
            if action_data:
                self.current_action = next_action
                self.current_action_data = action_data # Cache it
                self.current_action_channels = action_data.get("channels", {})
                self.current_action_duration = action_data.get("total_duration", 0.0) + 0.5  # +0.5s grace so the last keypoint is held long enough
                self.action_start_time = now
                self.post_action_hold_until = 0.0  # Cancel any active hold timer

                # Pre-process keypoints once (float-convert + sort by time) for efficient execution.
                self.current_action_times = [[] for _ in range(self.num_channels)]
                self.current_action_pressures = [[] for _ in range(self.num_channels)]
                self.current_action_indices = [0 for _ in range(self.num_channels)]

                for i in range(self.num_channels):
                    ch_key = str(i + 1)
                    raw_points = self.current_action_channels.get(ch_key, [])
                    processed = []
                    for pt in raw_points:
                        try:
                            t = float(pt[0])
                            p_val = pt[1]
                            if isinstance(p_val, str) and p_val in ("off", "on"):
                                processed.append((t, p_val))
                            else:
                                processed.append((t, float(p_val)))
                        except (TypeError, ValueError, IndexError):
                            continue

                    processed.sort(key=lambda x: x[0])
                    if processed:
                        self.current_action_times[i] = [t for t, _ in processed]
                        self.current_action_pressures[i] = [p for _, p in processed]
                
                print(f"Starting Action: {next_action} (Duration: {self.current_action_duration}s)")
                
                # Highlight the active action in the listbox (Index 0)
                self.queue_listbox.itemconfig(0, {'bg': '#90EE90'}) # Light green
            else:
                print(f"Error: Could not load action '{next_action}'. Removing from queue.")
                self.action_queue.pop(0)
                if self.queue_listbox.size() > 0:
                    self.queue_listbox.delete(0)

    def get_action_details(self, action_name):
        """Loads action data from the JSON file."""
        filepath = os.path.join(self.action_lib_path, f"{action_name}.json")
        if not os.path.exists(filepath):
            return None
        
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Failed to load action file: {e}")
            return None

    def load_default_settings(self):
        config_path = os.path.join(os.path.dirname(__file__), "graphic_settings.json")
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    settings = json.load(f)
                    # Load Y limits
                    loaded_limits = settings.get("y_limits", [])
                    # We need to make sure we match the number of channels
                    for i in range(min(len(loaded_limits), len(self.y_limits))):
                        self.y_limits[i] = tuple(loaded_limits[i])
                    
                    # Load X timespan
                    self.x_timespan = settings.get("x_timespan", self.x_timespan)
            except Exception as e:
                print(f"Failed to load default settings: {e}")

    def save_default_settings(self):
        config_path = os.path.join(os.path.dirname(__file__), "graphic_settings.json")
        settings = {
            "y_limits": self.y_limits,
            "x_timespan": self.x_timespan
        }
        try:
            with open(config_path, 'w') as f:
                json.dump(settings, f)
            messagebox.showinfo("Success", "Settings saved as default.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save settings: {e}")

    def setup_communication_tab(self):
        # --- Connection Section ---
        conn_frame = ttk.LabelFrame(self.tab_comm, text="Connection Settings")
        conn_frame.pack(padx=10, pady=10, fill="x")

        ttk.Label(conn_frame, text="USB Port:").pack(side="left", padx=5)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, state="readonly", width=15)
        self.port_combo.pack(side="left", padx=5)
        
        # Refresh button
        self.btn_refresh = ttk.Button(conn_frame, text="⟳", width=3, command=self.refresh_ports)
        self.btn_refresh.pack(side="left", padx=2)

        # Connect button
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=10)

        # Status Label
        self.lbl_status = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack(side="left", padx=10)

        # --- Configuration Section ---
        config_frame = ttk.LabelFrame(self.tab_comm, text="System Configuration")
        config_frame.pack(padx=10, pady=10, fill="x")
        
        ttk.Label(config_frame, text="Number of Channels (1-20):").pack(side="left", padx=5)
        
        self.spin_channels = ttk.Spinbox(config_frame, from_=1, to=20, width=5)
        self.spin_channels.set(self.num_channels)
        self.spin_channels.pack(side="left", padx=5)
        
        ttk.Button(config_frame, text="Apply", command=self.apply_channel_count).pack(side="left", padx=5)

        # --- Manual Control Section ---
        ctrl_frame = ttk.LabelFrame(self.tab_comm, text="Manual Command (Debug)")
        ctrl_frame.pack(padx=10, pady=10, fill="x")

        self.cmd_entry = ttk.Entry(ctrl_frame)
        self.cmd_entry.pack(side="left", padx=5, expand=True, fill="x")
        self.cmd_entry.bind("<Return>", lambda event: self.send_command())

        self.btn_send = ttk.Button(ctrl_frame, text="Send", command=self.send_command, state="disabled")
        self.btn_send.pack(side="left", padx=5)

        self.add_help_button(self.tab_comm)

    def apply_channel_count(self):
        try:
            n = int(self.spin_channels.get())
            if 1 <= n <= 20:
                self.num_channels = n
                # Resize y_limits, preserving old values where possible
                old_limits = self.y_limits
                self.y_limits = []
                for i in range(n):
                    if i < len(old_limits):
                        self.y_limits.append(old_limits[i])
                    else:
                        self.y_limits.append((-100, 100))
                
                self.refresh_live_control_ui()
                self.update_program_channel_list() # Update program tab too
                messagebox.showinfo("Success", f"Channel count set to {n}.")
            else:
                messagebox.showerror("Error", "Channels must be between 1 and 20")
        except ValueError:
            messagebox.showerror("Error", "Invalid number")

    def setup_program_tab(self):
        """Sets up the Action Programming Interface."""
        # Split into Control (Left) and Graph (Right)
        paned = ttk.PanedWindow(self.tab_program, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)
        
        # --- Controls ---
        controls = ttk.Frame(paned, width=300, relief=tk.SUNKEN)
        paned.add(controls, weight=1)
        
        # Action Management
        frame_mgmt = ttk.LabelFrame(controls, text="Action Management")
        frame_mgmt.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(frame_mgmt, text="Action Name:").pack(pady=2)
        self.ent_action_name = ttk.Entry(frame_mgmt, textvariable=self.action_name_var)
        self.ent_action_name.pack(fill=tk.X, padx=5, pady=2)

        # Update rename button visibility when the name changes (no polling loop)
        try:
            self.action_name_var.trace_add('write', lambda *_: self._update_rename_button_visibility())
        except Exception:
            pass
        
        btn_new = ttk.Button(frame_mgmt, text="Program New Action", command=self.program_new_action)
        btn_new.pack(fill=tk.X, padx=5, pady=2)
        
        btn_save = ttk.Button(frame_mgmt, text="Save Action to Library", command=self.save_action_to_library)
        btn_save.pack(fill=tk.X, padx=5, pady=2)

        # Hidden by default; shown when selecting an existing action from the library.
        self.btn_delete_action = ttk.Button(frame_mgmt, text="Delete Action", command=self.delete_selected_action)

        # Hidden by default; shown when a selected library action is renamed in the textbox.
        self.btn_rename_action = ttk.Button(frame_mgmt, text="Rename Action", command=self.rename_selected_action)
        
        # Keypoint Editing
        frame_edit = ttk.LabelFrame(controls, text="Edit Channel Curves")
        frame_edit.pack(fill=tk.X, padx=5, pady=10)
        
        ttk.Label(frame_edit, text="Select Channel:").pack(pady=2)
        self.combo_prog_channel = ttk.Combobox(frame_edit, state="readonly")
        self.combo_prog_channel.pack(fill=tk.X, padx=5, pady=2)
        self.update_program_channel_list()
        
        ttk.Label(frame_edit, text="Time (s):").pack(pady=2)
        self.ent_prog_time = ttk.Entry(frame_edit)
        self.ent_prog_time.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(frame_edit, text="Pressure (kPa):").pack(pady=2)
        self.ent_prog_pressure = ttk.Entry(frame_edit)
        self.ent_prog_pressure.pack(fill=tk.X, padx=5, pady=2)
        
        self.btn_add_pt = ttk.Button(frame_edit, text="Add Keypoint", command=self.add_keypoint)
        self.btn_add_pt.pack(fill=tk.X, padx=5, pady=5)

        self.btn_delete_pt = ttk.Button(frame_edit, text="Delete Keypoint", command=self.delete_keypoint, state="disabled")
        self.btn_delete_pt.pack(fill=tk.X, padx=5, pady=(0, 5))

        # Toggle OFF / ON buttons — only require time, not pressure
        frame_toggle = ttk.LabelFrame(controls, text="Channel Toggle (OFF / ON)")
        frame_toggle.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(frame_toggle, text="Adds a toggle keypoint at the time entered above.").pack(pady=2)

        toggle_btn_row = ttk.Frame(frame_toggle)
        toggle_btn_row.pack(fill=tk.X, padx=5, pady=5)

        self.btn_toggle_off = ttk.Button(toggle_btn_row, text="Set OFF at Time", command=self.add_toggle_off_keypoint)
        self.btn_toggle_off.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))

        self.btn_toggle_on = ttk.Button(toggle_btn_row, text="Set ON at Time", command=self.add_toggle_on_keypoint)
        self.btn_toggle_on.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))

        # --- Action Library (Program Tab) ---
        frame_lib = ttk.LabelFrame(controls, text="Action Library")
        frame_lib.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.prog_action_listbox = tk.Listbox(frame_lib, height=6)
        self.prog_action_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.prog_action_listbox.bind('<<ListboxSelect>>', self.on_program_action_select)
        
        btn_refresh_lib = ttk.Button(frame_lib, text="Refresh Library", command=self.refresh_action_library)
        btn_refresh_lib.pack(fill=tk.X, padx=5, pady=2)
        
        # --- Graph ---
        graph_frame = ttk.Frame(paned)
        paned.add(graph_frame, weight=3)
        
        self.fig_prog = Figure(figsize=(5, 4), dpi=100)
        self.ax_prog = self.fig_prog.add_subplot(111)
        self.ax_prog.set_title("Action Preview")
        self.ax_prog.set_xlabel("Time (s)")
        self.ax_prog.set_ylabel("Pressure (kPa)")
        self.ax_prog.grid(True)
        
        self.canvas_prog = FigureCanvasTkAgg(self.fig_prog, master=graph_frame)
        self.canvas_prog.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        # Use click-based picking that can cycle through overlapping keypoints.
        self.canvas_prog.mpl_connect('button_press_event', self.on_program_canvas_click)

        # Channel Visibility Controls
        self.frame_prog_vis = ttk.LabelFrame(graph_frame, text="Show Channels")
        self.frame_prog_vis.pack(fill=tk.X, padx=5, pady=5)
        
        self.prog_vis_vars = []
        self.prog_vis_chks = []
        
        for i in range(20):
            var = tk.BooleanVar(value=True)
            self.prog_vis_vars.append(var)
            chk = ttk.Checkbutton(self.frame_prog_vis, text=str(i+1), variable=var, command=self.update_program_graph, width=3)
            self.prog_vis_chks.append(chk)
            
        self.update_program_channel_list() # Update visibility of checkboxes

        self.add_help_button(self.tab_program)

    def update_program_channel_list(self):
        """Updates the channel selector and visibility checkboxes in the program tab."""
        vals = [f"Channel {i+1}" for i in range(self.num_channels)]
        self.combo_prog_channel['values'] = vals
        if vals: self.combo_prog_channel.current(0)
        
        # Update visibility checkboxes if they exist
        if hasattr(self, 'prog_vis_chks'):
            for i, chk in enumerate(self.prog_vis_chks):
                if i < self.num_channels:
                    chk.grid(row=i//10, column=i%10, padx=2, sticky="w")
                else:
                    chk.grid_remove()

    def on_program_action_select(self, event):
        """Loads the selected action from the library into the editor."""
        selection = self.prog_action_listbox.curselection()
        if not selection:
            self.selected_library_action_name = None
            self._hide_delete_action_button()
            self._hide_rename_action_button()
            return
            
        action_name = self.prog_action_listbox.get(selection[0])
        action_data = self.get_action_details(action_name)
        
        if not action_data:
            self.selected_library_action_name = None
            self._hide_delete_action_button()
            self._hide_rename_action_button()
            return

        # Selected an existing library action -> enable delete button
        self.selected_library_action_name = action_name
        self._show_delete_action_button()
        self._update_rename_button_visibility()
            
        # Reset current program
        self.current_program = {i: [] for i in range(20)}
        self.selected_keypoint = None
        self.btn_add_pt.config(text="Add Keypoint")
        if hasattr(self, 'btn_delete_pt'):
            self.btn_delete_pt.config(state="disabled")
        
        # Load name
        self.action_name_var.set(action_data.get("name", action_name))
        
        # Load channels
        channels = action_data.get("channels", {})
        for ch_str, points in channels.items():
            try:
                ch_idx = int(ch_str) - 1 # Convert "1" -> 0
                if 0 <= ch_idx < 20:
                    loaded = []
                    for p in points:
                        t_val = float(p[0])
                        p_val = p[1]
                        if isinstance(p_val, str) and p_val in ("off", "on"):
                            loaded.append((t_val, p_val))
                        else:
                            loaded.append((t_val, float(p_val)))
                    self.current_program[ch_idx] = loaded
            except (ValueError, TypeError, IndexError):
                pass
                
        self.update_program_graph()

    def program_new_action(self):
        """Resets the programming interface."""
        self.current_program = {i: [] for i in range(20)}
        self.action_name_var.set("")
        self.selected_keypoint = None
        self.btn_add_pt.config(text="Add Keypoint")
        if hasattr(self, 'btn_delete_pt'):
            self.btn_delete_pt.config(state="disabled")
        self.selected_library_action_name = None
        self._hide_delete_action_button()
        self._hide_rename_action_button()
        self.update_program_graph()

    def _show_delete_action_button(self):
        """Shows the delete button (Program tab) under the save button."""
        if not hasattr(self, 'btn_delete_action'):
            return
        # Only show if there is an actual selected library action
        if not self.selected_library_action_name:
            self._hide_delete_action_button()
            return

        # Ensure it's placed under the save button in the same frame.
        try:
            if not self.btn_delete_action.winfo_ismapped():
                self.btn_delete_action.pack(fill=tk.X, padx=5, pady=2)
        except tk.TclError:
            # Widget may not be fully initialized yet
            pass

    def _hide_delete_action_button(self):
        """Hides the delete button (Program tab)."""
        if not hasattr(self, 'btn_delete_action'):
            return
        try:
            if self.btn_delete_action.winfo_ismapped():
                self.btn_delete_action.pack_forget()
        except tk.TclError:
            pass

    def _show_rename_action_button(self):
        """Shows the rename button (Program tab)."""
        if not hasattr(self, 'btn_rename_action'):
            return
        try:
            if not self.btn_rename_action.winfo_ismapped():
                self.btn_rename_action.pack(fill=tk.X, padx=5, pady=2)
        except tk.TclError:
            pass

    def _hide_rename_action_button(self):
        """Hides the rename button (Program tab)."""
        if not hasattr(self, 'btn_rename_action'):
            return
        try:
            if self.btn_rename_action.winfo_ismapped():
                self.btn_rename_action.pack_forget()
        except tk.TclError:
            pass

    def _update_rename_button_visibility(self):
        """Shows Rename button only when selected library action name != textbox name."""
        # Must have a selected existing library action
        if not self.selected_library_action_name:
            self._hide_rename_action_button()
            return

        current_name = self.action_name_var.get().strip() if hasattr(self, 'action_name_var') else ""
        if not current_name:
            self._hide_rename_action_button()
            return

        if current_name == self.selected_library_action_name:
            self._hide_rename_action_button()
            return

        self._show_rename_action_button()

    def rename_selected_action(self):
        """Renames the selected action JSON file and updates the saved action name."""
        old_name = self.selected_library_action_name
        new_name = self.action_name_var.get().strip() if hasattr(self, 'action_name_var') else ""

        if not old_name:
            return
        if not new_name:
            messagebox.showerror("Error", "Please enter a new action name.")
            self._update_rename_button_visibility()
            return
        if new_name == old_name:
            self._hide_rename_action_button()
            return

        old_path = os.path.join(self.action_lib_path, f"{old_name}.json")
        new_path = os.path.join(self.action_lib_path, f"{new_name}.json")

        if not os.path.exists(old_path):
            messagebox.showerror("Not Found", f"Action file not found: {old_path}")
            self.refresh_action_library()
            self.selected_library_action_name = None
            self._hide_delete_action_button()
            self._hide_rename_action_button()
            return

        if os.path.exists(new_path):
            overwrite = messagebox.askyesno(
                "Overwrite Action",
                f"An action named '{new_name}' already exists. Overwrite it?"
            )
            if not overwrite:
                return

        # Load existing saved action and update only the saved name.
        try:
            with open(old_path, 'r') as f:
                action_data = json.load(f)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read action '{old_name}': {e}")
            return

        action_data['name'] = new_name

        # Write to a temp file then atomically replace the destination.
        try:
            os.makedirs(self.action_lib_path, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(prefix="action_", suffix=".json", dir=self.action_lib_path)
            os.close(fd)

            with open(tmp_path, 'w') as f:
                json.dump(action_data, f, indent=2)

            # Replace destination (works even if it already exists)
            os.replace(tmp_path, new_path)

            # Remove old file if it was a true rename
            if os.path.abspath(old_path) != os.path.abspath(new_path) and os.path.exists(old_path):
                os.remove(old_path)

        except Exception as e:
            # Best effort cleanup
            try:
                if 'tmp_path' in locals() and os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except Exception:
                pass
            messagebox.showerror("Error", f"Failed to rename action: {e}")
            return

        # Update state + UI
        self.selected_library_action_name = new_name
        self.action_name_var.set(new_name)
        self.refresh_action_library()

        # Re-select the renamed action in the listbox
        try:
            items = self.prog_action_listbox.get(0, tk.END)
            if new_name in items:
                idx = items.index(new_name)
                self.prog_action_listbox.selection_clear(0, tk.END)
                self.prog_action_listbox.selection_set(idx)
                self.prog_action_listbox.see(idx)
        except Exception:
            pass

        self._update_rename_button_visibility()
        messagebox.showinfo("Renamed", f"Action '{old_name}' renamed to '{new_name}'.")

    def delete_selected_action(self):
        """Deletes the selected action JSON file from the library and refreshes the UI."""
        if not hasattr(self, 'prog_action_listbox'):
            return

        selection = self.prog_action_listbox.curselection()
        if selection:
            action_name = self.prog_action_listbox.get(selection[0])
        else:
            # Fallback to last known selection
            action_name = self.selected_library_action_name

        if not action_name:
            messagebox.showwarning("No Selection", "Please select an action from the library to delete.")
            return

        filepath = os.path.join(self.action_lib_path, f"{action_name}.json")
        if not os.path.exists(filepath):
            messagebox.showerror("Not Found", f"Action file not found: {filepath}")
            # Refresh anyway to clean up stale UI entries
            self.refresh_action_library()
            self.selected_library_action_name = None
            self._hide_delete_action_button()
            return

        confirm = messagebox.askyesno(
            "Delete Action",
            f"Delete action '{action_name}' from the library?\n\nThis will permanently remove its JSON file."
        )
        if not confirm:
            return

        try:
            os.remove(filepath)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to delete action '{action_name}': {e}")
            return

        # Refresh library and reset editor
        self.refresh_action_library()
        self.program_new_action()
        messagebox.showinfo("Deleted", f"Action '{action_name}' deleted.")

    def on_program_canvas_click(self, event):
        """Selects keypoints by clicking; cycles through overlapping keypoints on repeated clicks."""
        # Only left click inside the Program preview axes
        if event is None or event.inaxes != getattr(self, 'ax_prog', None):
            return
        if getattr(event, 'button', None) != 1:
            return
        if event.x is None or event.y is None:
            return

        # Collect candidates near click (in pixel space)
        candidates = self._find_keypoint_candidates_near_click(event.x, event.y, radius_px=9)
        if not candidates:
            return

        # Determine whether to cycle or reset
        click_xy = (int(round(event.x)), int(round(event.y)))
        candidates_key = tuple((c['channel'], c['index']) for c in candidates)

        same_spot = False
        if self._prog_pick_last_xy is not None:
            dx = click_xy[0] - self._prog_pick_last_xy[0]
            dy = click_xy[1] - self._prog_pick_last_xy[1]
            # Treat clicks within a small pixel window as the "same" spot
            same_spot = (dx * dx + dy * dy) <= (4 * 4)

        if same_spot and candidates_key == self._prog_pick_candidates_key:
            self._prog_pick_cycle_index = (self._prog_pick_cycle_index + 1) % len(candidates)
        else:
            self._prog_pick_cycle_index = 0

        self._prog_pick_last_xy = click_xy
        self._prog_pick_candidates_key = candidates_key

        chosen = candidates[self._prog_pick_cycle_index]
        self._select_program_keypoint(chosen['channel'], chosen['index'])

    def _find_keypoint_candidates_near_click(self, x_px, y_px, radius_px=9):
        """Returns a sorted list of keypoints near a click in display (pixel) coordinates."""
        if not hasattr(self, 'ax_prog'):
            return []

        try:
            transform = self.ax_prog.transData.transform
        except Exception:
            return []

        r2 = float(radius_px) * float(radius_px)
        candidates = []

        for ch_idx, points in self.current_program.items():
            if not points:
                continue
            if ch_idx >= self.num_channels:
                continue
            if hasattr(self, 'prog_vis_vars') and not self.prog_vis_vars[ch_idx].get():
                continue

            for pt_idx, (t, p) in enumerate(points):
                # Toggle keypoints are drawn at y=0 on the graph
                plot_p = 0 if isinstance(p, str) else p
                try:
                    sx, sy = transform((t, plot_p))
                except Exception:
                    continue

                dx = sx - x_px
                dy = sy - y_px
                d2 = dx * dx + dy * dy
                if d2 <= r2:
                    candidates.append({
                        'channel': ch_idx,
                        'index': pt_idx,
                        't': t,
                        'p': p,
                        'd2': d2,
                    })

        # Sort by distance then channel then index for deterministic cycling
        candidates.sort(key=lambda c: (c['d2'], c['channel'], c['index']))
        return candidates

    def _select_program_keypoint(self, ch_idx, pt_idx):
        """Selects a keypoint by channel/index and updates the editor controls."""
        if ch_idx is None or pt_idx is None:
            return
        if ch_idx not in self.current_program:
            return
        if pt_idx < 0 or pt_idx >= len(self.current_program[ch_idx]):
            return

        t, p = self.current_program[ch_idx][pt_idx]

        # Populate fields
        try:
            self.combo_prog_channel.current(ch_idx)
        except Exception:
            pass

        self.ent_prog_time.delete(0, tk.END)
        self.ent_prog_time.insert(0, str(t))
        self.ent_prog_pressure.delete(0, tk.END)
        if isinstance(p, str):
            self.ent_prog_pressure.insert(0, p.upper())  # "OFF" or "ON"
        else:
            self.ent_prog_pressure.insert(0, str(p))

        # Set state
        self.selected_keypoint = {'channel': ch_idx, 'original_t': t, 'original_p': p}
        self.btn_add_pt.config(text="Edit Keypoint")
        if hasattr(self, 'btn_delete_pt'):
            self.btn_delete_pt.config(state="normal")

    def delete_keypoint(self):
        """Deletes the currently selected keypoint (if any) and clears selection."""
        if not hasattr(self, 'selected_keypoint') or not self.selected_keypoint:
            return

        ch_idx = self.selected_keypoint.get('channel', None)
        orig_t = self.selected_keypoint.get('original_t', None)
        orig_p = self.selected_keypoint.get('original_p', None)

        if ch_idx is None or orig_t is None or orig_p is None:
            return

        if ch_idx in self.current_program:
            self.current_program[ch_idx] = [
                pt for pt in self.current_program[ch_idx]
                if not self._keypoint_matches(pt, orig_t, orig_p)
            ]

        # Clear selection state
        self.selected_keypoint = None
        self.btn_add_pt.config(text="Add Keypoint")
        if hasattr(self, 'btn_delete_pt'):
            self.btn_delete_pt.config(state="disabled")

        # Clear edit fields (optional but helps indicate unselected)
        self.ent_prog_time.delete(0, tk.END)
        self.ent_prog_pressure.delete(0, tk.END)

        self.update_program_graph()

    def add_keypoint(self):
        """Adds or edits a keypoint."""
        try:
            t = float(self.ent_prog_time.get())
            p = float(self.ent_prog_pressure.get())
            if t < 0: raise ValueError("Time must be positive")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numeric values for Time and Pressure.")
            return

        ch_idx = self.combo_prog_channel.current()
        if ch_idx == -1: return
        
        if ch_idx not in self.current_program:
            self.current_program[ch_idx] = []
            
        # If editing, remove the old point first
        if hasattr(self, 'selected_keypoint') and self.selected_keypoint:
            if self.selected_keypoint['channel'] == ch_idx:
                orig_t = self.selected_keypoint['original_t']
                orig_p = self.selected_keypoint['original_p']
                
                self.current_program[ch_idx] = [
                    pt for pt in self.current_program[ch_idx]
                    if not self._keypoint_matches(pt, orig_t, orig_p)
                ]
                
                # Reset state
                self.selected_keypoint = None
                self.btn_add_pt.config(text="Add Keypoint")
                if hasattr(self, 'btn_delete_pt'):
                    self.btn_delete_pt.config(state="disabled")

        # Special handling for t=0: Overwrite
        if abs(t) < 1e-5:
            # Remove existing point at t=0 if it exists
            self.current_program[ch_idx] = [pt for pt in self.current_program[ch_idx] if abs(pt[0]) >= 1e-5]
        else:
            # Check for duplicate time and shift if necessary
            while any(abs(pt[0] - t) < 1e-5 for pt in self.current_program[ch_idx]):
                t += 0.05
            
        self.current_program[ch_idx].append((t, p))
        self.current_program[ch_idx].sort(key=lambda x: x[0]) # Sort by time
        
        self.update_program_graph()
        
        # Clear inputs
        self.ent_prog_time.delete(0, tk.END)
        self.ent_prog_pressure.delete(0, tk.END)

    @staticmethod
    def _keypoint_matches(pt, orig_t, orig_p):
        """Returns True if keypoint `pt` matches the given time and pressure/toggle value."""
        if abs(pt[0] - orig_t) >= 1e-5:
            return False
        if isinstance(orig_p, str):
            return pt[1] == orig_p
        return isinstance(pt[1], (int, float)) and abs(pt[1] - orig_p) < 1e-5

    def _add_toggle_keypoint(self, toggle_type):
        """Shared logic for adding an 'off' or 'on' toggle keypoint at the given time."""
        try:
            t = float(self.ent_prog_time.get())
            if t < 0:
                raise ValueError
        except (ValueError, TypeError):
            messagebox.showerror("Invalid Input", "Please enter a valid time (seconds).")
            return

        ch_idx = self.combo_prog_channel.current()
        if ch_idx == -1:
            return

        if ch_idx not in self.current_program:
            self.current_program[ch_idx] = []

        # Remove any existing keypoint at this exact time for this channel
        self.current_program[ch_idx] = [
            pt for pt in self.current_program[ch_idx]
            if abs(pt[0] - t) >= 1e-5
        ]

        self.current_program[ch_idx].append((t, toggle_type))
        self.current_program[ch_idx].sort(key=lambda x: x[0])

        # Clear selection
        self.selected_keypoint = None
        self.btn_add_pt.config(text="Add Keypoint")
        if hasattr(self, 'btn_delete_pt'):
            self.btn_delete_pt.config(state="disabled")

        self.update_program_graph()
        self.ent_prog_time.delete(0, tk.END)
        self.ent_prog_pressure.delete(0, tk.END)

    def add_toggle_off_keypoint(self):
        """Inserts an 'off' toggle keypoint at the specified time."""
        self._add_toggle_keypoint("off")

    def add_toggle_on_keypoint(self):
        """Inserts an 'on' toggle keypoint at the specified time."""
        self._add_toggle_keypoint("on")

    def update_program_graph(self):
        """Redraws the preview graph in the program tab."""
        self.ax_prog.clear()
        self.ax_prog.set_title("Action Preview")
        self.ax_prog.set_xlabel("Time (s)")
        self.ax_prog.set_ylabel("Pressure (kPa)")
        self.ax_prog.grid(True)
        
        max_t = 0
        
        # Only plot channels that have points
        for ch_idx, points in self.current_program.items():
            if not points: continue
            if ch_idx >= self.num_channels: continue
            
            if hasattr(self, 'prog_vis_vars') and not self.prog_vis_vars[ch_idx].get():
                continue
            
            # Separate pressure keypoints from toggle keypoints
            pressure_pts = [(t, p) for t, p in points if isinstance(p, (int, float))]
            off_pts = [t for t, p in points if p == "off"]
            on_pts  = [t for t, p in points if p == "on"]
            
            # Get color from the default color cycle for this channel
            color = None  # will be assigned by the first plot call

            # Plot pressure curve (step function) with markers
            if pressure_pts:
                ts = [pt[0] for pt in pressure_pts]
                ps = [pt[1] for pt in pressure_pts]
                line, = self.ax_prog.plot(
                    ts, ps,
                    marker='o',
                    drawstyle='steps-post',
                    label=f"Ch {ch_idx+1}",
                    picker=5
                )
                line.set_gid(ch_idx)
                color = line.get_color()
                if ts: max_t = max(max_t, max(ts))
            else:
                # Channel has only toggle points — still pick a color
                color = f"C{ch_idx % 10}"
            
            # Plot OFF toggle markers (triangle-down)
            if off_pts:
                self.ax_prog.plot(
                    off_pts, [0] * len(off_pts),
                    marker='v', markersize=10, linestyle='None',
                    color=color, markeredgecolor='red', markeredgewidth=1.5,
                    label=f"Ch {ch_idx+1} OFF" if not pressure_pts else None
                )
                max_t = max(max_t, max(off_pts))

            # Plot ON toggle markers (triangle-up)
            if on_pts:
                self.ax_prog.plot(
                    on_pts, [0] * len(on_pts),
                    marker='^', markersize=10, linestyle='None',
                    color=color, markeredgecolor='green', markeredgewidth=1.5,
                    label=f"Ch {ch_idx+1} ON" if not pressure_pts else None
                )
                max_t = max(max_t, max(on_pts))

            # Draw shaded OFF regions
            all_sorted = sorted(points, key=lambda pt: pt[0])
            off_start = None
            for t_pt, val in all_sorted:
                if val == "off":
                    off_start = t_pt
                elif off_start is not None:
                    # Any non-off keypoint ends the off region
                    self.ax_prog.axvspan(off_start, t_pt, alpha=0.08, color=color)
                    off_start = None
            # If still off at the end, shade to max_t
            if off_start is not None and max_t > off_start:
                self.ax_prog.axvspan(off_start, max_t, alpha=0.08, color=color)
            
        if max_t > 0:
            self.ax_prog.set_xlim(0, max_t * 1.1)
            self.ax_prog.legend()
            
        self.canvas_prog.draw()

    def save_action_to_library(self):
        """Saves the current program to a JSON file."""
        name = self.action_name_var.get().strip() if hasattr(self, 'action_name_var') else self.ent_action_name.get().strip()
        if not name:
            messagebox.showerror("Error", "Please enter an action name.")
            return
            
        # 1. Find total duration
        total_duration = 0.0
        for i in range(self.num_channels):
            points = self.current_program.get(i, [])
            if points:
                total_duration = max(total_duration, points[-1][0])
                
        if total_duration == 0:
            messagebox.showwarning("Warning", "Action has no duration.")
            return

        # 2. Pad channels
        final_channels = {}
        for i in range(self.num_channels):
            points = self.current_program.get(i, [])
            export_points = [list(pt) for pt in points]

            # Check if channel has ANY pressure keypoints (not just toggles)
            has_pressure = any(isinstance(pt[1], (int, float)) for pt in points)
            has_any = len(export_points) > 0

            if not has_any:
                # No keypoints at all → will be auto-detected as off by execution
                # Don't pad with zeros; leave empty so automatic off detection works.
                final_channels[str(i+1)] = []
            elif not has_pressure:
                # Only toggle keypoints, no pressure values → pad start if needed
                if export_points[0][0] > 0:
                    export_points.insert(0, [0.0, 0.0])  # start active at 0 until first toggle
                final_channels[str(i+1)] = export_points
            else:
                if export_points[0][0] > 0:
                    export_points.insert(0, [0.0, 0.0])
                
                last_t, last_p = export_points[-1]
                if last_t < total_duration:
                    export_points.append([total_duration, last_p])
                    
                final_channels[str(i+1)] = export_points

        # 3. Create JSON structure
        action_data = {
            "name": name,
            "total_duration": total_duration,
            "channels": final_channels
        }
        
        # 4. Save to file
        if not os.path.exists(self.action_lib_path):
            os.makedirs(self.action_lib_path)
            
        filename = os.path.join(self.action_lib_path, f"{name}.json")
        try:
            with open(filename, 'w') as f:
                json.dump(action_data, f, indent=2)
            
            messagebox.showinfo("Success", f"Action '{name}' saved to library.")
            self.refresh_action_library()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save file: {e}")

    def refresh_action_library(self):
        """Reloads actions from the 'action_library' folder."""
        # Clear both lists if they exist
        if hasattr(self, 'action_listbox'):
            self.action_listbox.delete(0, tk.END)
        if hasattr(self, 'prog_action_listbox'):
            self.prog_action_listbox.delete(0, tk.END)
        
        if not os.path.exists(self.action_lib_path):
            return
            
        for f in os.listdir(self.action_lib_path):
            if f.endswith(".json"):
                name = f.replace(".json", "")
                # Populate Live Control list
                if hasattr(self, 'action_listbox'):
                    self.action_listbox.insert(tk.END, name)
                # Populate Program Tab list
                if hasattr(self, 'prog_action_listbox'):
                    self.prog_action_listbox.insert(tk.END, name)

        # If the selected action no longer exists, hide delete button
        if self.selected_library_action_name:
            expected = os.path.join(self.action_lib_path, f"{self.selected_library_action_name}.json")
            if not os.path.exists(expected):
                self.selected_library_action_name = None
                self._hide_delete_action_button()

    def setup_live_control_tab(self):
        # Use PanedWindow to split Library (Left) and Graphs (Right)
        paned = ttk.PanedWindow(self.tab_live, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)

        # --- Left Side: Action Library & Queue ---
        sidebar = ttk.Frame(paned, width=250, relief=tk.SUNKEN)
        paned.add(sidebar, weight=1)
        
        # E-STOP BUTTON
        btn_estop = tk.Button(sidebar, text="EMERGENCY STOP", bg="red", fg="white", font=("Arial", 12, "bold"), command=self.emergency_stop)
        btn_estop.pack(fill=tk.X, padx=5, pady=10)
        
        # PAUSE BUTTON
        self.btn_pause = tk.Button(sidebar, text="PAUSE", bg="orange", fg="black", font=("Arial", 10, "bold"), command=self.toggle_pause)
        self.btn_pause.pack(fill=tk.X, padx=5, pady=5)
        
        # CANCEL CURRENT ACTION BUTTON
        self.btn_cancel = tk.Button(sidebar, text="CANCEL CURRENT ACTION", bg="maroon", fg="white", font=("Arial", 9, "bold"), command=self.cancel_current_action)
        self.btn_cancel.pack(fill=tk.X, padx=5, pady=5)

        # 1. Library Section
        lbl_lib = ttk.Label(sidebar, text="Action Library", font=("Arial", 10, "bold"))
        lbl_lib.pack(pady=(5, 0))
        
        lib_frame = ttk.Frame(sidebar)
        lib_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.action_listbox = tk.Listbox(lib_frame, height=11)
        lib_scrollbar = ttk.Scrollbar(lib_frame, orient=tk.VERTICAL, command=self.action_listbox.yview)
        self.action_listbox.configure(yscrollcommand=lib_scrollbar.set)
        self.action_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        lib_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Note: Population happens in refresh_action_library()

        # 2. Add Button
        self.btn_add_queue = tk.Button(sidebar, text="Add to Queue ↓", bg="green", fg="white", font=("Arial", 9, "bold"), command=self.add_to_queue)
        self.btn_add_queue.pack(pady=5, padx=5, fill=tk.X)

        # 3. Queue Section
        lbl_queue = ttk.Label(sidebar, text="Execution Queue", font=("Arial", 10, "bold"))
        lbl_queue.pack(pady=(10, 0))

        self.queue_listbox = tk.Listbox(sidebar, height=10)
        self.queue_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # --- Right Side: Graphs ---
        graph_area = ttk.Frame(paned)
        paned.add(graph_area, weight=4)

        # Visibility Controls
        vis_frame = ttk.Frame(graph_area)
        vis_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Graph Settings Button
        btn_settings = ttk.Button(vis_frame, text="Graph Settings", command=self.open_graph_settings)
        btn_settings.pack(side=tk.LEFT, padx=5)

        # Download Graph Button (Added)
        self.btn_download_graph = ttk.Button(vis_frame, text="Download Graph", command=self.open_download_graph_window, state="disabled")
        self.btn_download_graph.pack(side=tk.LEFT, padx=5)

        ttk.Label(vis_frame, text="Show/Hide:").pack(side=tk.LEFT, padx=(10, 0))
        
        # Container for checkboxes to allow refreshing
        self.checkbox_frame = ttk.Frame(vis_frame)
        self.checkbox_frame.pack(side=tk.LEFT, padx=5)

        # HISTORY SCROLLBAR (Hidden by default; shown when paused)
        # Placed above the matplotlib canvas.
        self.history_frame = ttk.Frame(graph_area)
        self.history_slider = tk.Scale(
            self.history_frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            showvalue=0,
            resolution=self.history_slider_resolution,
            command=self.on_history_scroll,
        )
        self.history_slider.pack(fill=tk.X)

        self.ch_vars = []
        # Store handles to active plot objects: { channel_index: {'ax': ax, 'line_target': line, 'line_actual': line} }
        self.plot_handles = {}

        # Matplotlib Figure
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_area)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Initial population
        self.refresh_live_control_ui()

        self.add_help_button(self.tab_live)

    def refresh_live_control_ui(self):
        """Re-creates the checkboxes based on the current number of channels."""
        # Clear existing checkboxes
        for widget in self.checkbox_frame.winfo_children():
            widget.destroy()
        
        self.ch_vars = []
        
        for i in range(self.num_channels):
            var = tk.BooleanVar(value=True)
            self.ch_vars.append(var)
            cb = ttk.Checkbutton(self.checkbox_frame, text=f"Ch{i+1}", variable=var, command=self.update_graph_visibility)
            cb.pack(side=tk.LEFT, padx=2)
            
        self.update_graph_visibility()

    def open_graph_settings(self):
        """Opens a window to configure Y-axis limits for each channel."""
        settings_win = tk.Toplevel(self.root)
        settings_win.title("Graph Settings")
        settings_win.geometry("450x500") 
        
        # Main container with scrollbar
        main_frame = ttk.Frame(settings_win)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        entries = []

        # Header inside scrollable frame
        ttk.Label(scrollable_frame, text="Channel").grid(row=0, column=0, padx=5, pady=5)
        ttk.Label(scrollable_frame, text="Min Y (kPa)").grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(scrollable_frame, text="Max Y (kPa)").grid(row=0, column=2, padx=5, pady=5)

        for i in range(self.num_channels):
            ttk.Label(scrollable_frame, text=f"Ch {i+1}").grid(row=i+1, column=0, padx=5, pady=2)
            
            min_val, max_val = self.y_limits[i]
            
            ent_min = ttk.Entry(scrollable_frame, width=10)
            ent_min.insert(0, str(min_val))
            ent_min.grid(row=i+1, column=1, padx=5, pady=2)
            
            ent_max = ttk.Entry(scrollable_frame, width=10)
            ent_max.insert(0, str(max_val))
            ent_max.grid(row=i+1, column=2, padx=5, pady=2)
            
            entries.append((ent_min, ent_max))

        # X-axis Timespan Setting (Outside scrollable area, at bottom of window)
        bottom_frame = ttk.Frame(settings_win)
        bottom_frame.pack(fill="x", padx=10, pady=10)

        ttk.Label(bottom_frame, text="X-axis Timespan (s):").pack(side="left", padx=5)
        ent_timespan = ttk.Entry(bottom_frame, width=10)
        ent_timespan.insert(0, str(self.x_timespan))
        ent_timespan.pack(side="left", padx=5)

        def apply_changes_internal():
            # Validate and apply Y limits
            for i, (e_min, e_max) in enumerate(entries):
                try:
                    new_min = float(e_min.get())
                    new_max = float(e_max.get())
                    if new_min >= new_max:
                        messagebox.showerror("Error", f"Ch {i+1}: Min must be less than Max.")
                        return False
                    self.y_limits[i] = (new_min, new_max)
                except ValueError:
                    messagebox.showerror("Error", f"Ch {i+1}: Invalid number format.")
                    return False
            
            # Validate and apply X timespan
            try:
                new_span = float(ent_timespan.get())
                if new_span <= 0:
                    messagebox.showerror("Error", "Timespan must be positive.")
                    return False
                self.x_timespan = new_span
            except ValueError:
                messagebox.showerror("Error", "Invalid timespan format.")
                return False
            
            self.update_graph_visibility() # Redraw graphs with new limits
            return True

        def on_apply():
            if apply_changes_internal():
                settings_win.destroy()

        def on_set_default():
            if apply_changes_internal():
                self.save_default_settings()
                settings_win.destroy()

        btn_apply = ttk.Button(bottom_frame, text="Apply", command=on_apply)
        btn_apply.pack(side="right", padx=5)
        
        btn_default = ttk.Button(bottom_frame, text="Set as Default", command=on_set_default)
        btn_default.pack(side="right", padx=5)

    def update_graph_visibility(self):
        """Re-draws the figure with only selected channels, rearranging them."""
        self.fig.clear()
        self.plot_handles.clear()
        
        # Identify which channels are selected
        selected_indices = [i for i, var in enumerate(self.ch_vars) if var.get()]
        num_plots = len(selected_indices)
        
        if num_plots == 0:
            self.canvas.draw()
            return

        # Calculate grid dimensions
        cols = 2 if num_plots > 1 else 1
        rows = (num_plots + cols - 1) // cols
        
        for idx, channel_idx in enumerate(selected_indices):
            # add_subplot(rows, cols, index) where index starts at 1
            ax = self.fig.add_subplot(rows, cols, idx + 1)
            ax.set_title(f"Channel {channel_idx + 1}", fontsize=8)
            
            # Apply stored Y-limits
            y_min, y_max = self.y_limits[channel_idx]
            ax.set_ylim(y_min, y_max)
            
            # Apply stored X-timespan
            ax.set_xlim(0, self.x_timespan)
            
            ax.grid(True)
            
            # Initialize empty lines
            line_target, = ax.plot([], [], 'r--', label='Target')
            line_actual, = ax.plot([], [], 'b-', label='Actual')
            
            # Store handles for future data updates
            self.plot_handles[channel_idx] = {
                'ax': ax,
                'line_target': line_target,
                'line_actual': line_actual
            }

        self.fig.tight_layout()
        self.canvas.draw()

    def open_download_graph_window(self):
        """
        Opens a window to select channels and save their data to a CSV file.
        This feature is only available while paused.
        """
        win = tk.Toplevel(self.root)
        win.title("Download Graph Data")
        win.geometry("400x400")
        
        # 1. Top Label
        lbl = ttk.Label(win, text="Select channels to download:", padding=10)
        lbl.pack(side=tk.TOP, fill=tk.X)
        
        # 2. Bottom Area (Buttons & Input) - Pack BEFORE list to stay at bottom
        bottom_area = ttk.Frame(win, padding=10)
        bottom_area.pack(side=tk.BOTTOM, fill=tk.X)
        
        #   Filename Input
        input_frame = ttk.Frame(bottom_area)
        input_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        
        ttk.Label(input_frame, text="Filename:").pack(side=tk.LEFT)
        ent_filename = ttk.Entry(input_frame)
        ent_filename.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        # Default filename
        ent_filename.insert(0, f"graph_data_{time.strftime('%Y%m%d_%H%M%S')}")

        #   Save Button
        btn_save = ttk.Button(bottom_area, text="Save to CSV", width=20)
        btn_save.pack(side=tk.RIGHT)

        # 3. Middle: Checkboxes
        frame_checks = ttk.Frame(win, padding=10)
        frame_checks.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        canvas = tk.Canvas(frame_checks)
        scrollbar = ttk.Scrollbar(frame_checks, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        ch_vars = []
        for i in range(self.num_channels):
            var = tk.BooleanVar(value=False)
            ch_vars.append(var)
            cb = ttk.Checkbutton(scrollable_frame, text=f"Channel {i+1}", variable=var)
            cb.pack(anchor="w", padx=5, pady=2)
            
        # Hook up the save button
        btn_save.config(command=lambda: self.save_graph_data(win, ch_vars, ent_filename.get()))

    def save_graph_data(self, window, ch_vars, filename_input):
        selected_indices = [i for i, var in enumerate(ch_vars) if var.get()]
        
        if not selected_indices:
            messagebox.showwarning("No Selection", "Please select at least one channel to download.")
            return

        filename = filename_input.strip()
        if not filename:
            messagebox.showwarning("Invalid Filename", "Please enter a filename.")
            return
            
        if not filename.lower().endswith(".csv"):
            filename += ".csv"
            
        # Check if we have data
            
        # Check if we have data
        # We need a unified time base. We'll pick the time vector from the first selected channel 
        # that has data.
        
        base_time = None
        base_idx = -1
        
        for idx in selected_indices:
            if idx < len(self.channel_data) and len(self.channel_data[idx]['t']) > 0:
                base_time = self.channel_data[idx]['t']
                base_idx = idx
                break
        
        if base_time is None:
            messagebox.showwarning("No Data", "No recorded data found for selected channels.")
            return

        # Prepare path
        # 'graphs' folder is sibling to 'action_library'
        graphs_dir = os.path.join(os.path.dirname(self.action_lib_path), "graphs")
        if not os.path.exists(graphs_dir):
            try:
                os.makedirs(graphs_dir)
            except OSError as e:
                messagebox.showerror("Error", f"Could not create graphs directory:\n{e}")
                return
        
        filepath = os.path.join(graphs_dir, filename)
        
        try:
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                
                # Build Header
                header = ["Time (s)"]
                for i in selected_indices:
                    header.append(f"Ch{i+1}_Target")
                    header.append(f"Ch{i+1}_Actual")
                writer.writerow(header)
                
                # Write rows
                num_points = len(base_time)
                
                for k in range(num_points):
                    row = [round(base_time[k], 3)]
                    for i in selected_indices:
                        if i < len(self.channel_data) and k < len(self.channel_data[i]['t']):
                            tgt = self.channel_data[i]['target'][k]
                            act = self.channel_data[i]['actual'][k]
                            row.append(tgt)
                            row.append(act)
                        else:
                            row.append("")
                            row.append("")
                    writer.writerow(row)
            
            messagebox.showinfo("Success", f"Data saved successfully to:\n{filepath}")
            window.destroy()

        except Exception as e:
            messagebox.showerror("Error", f"Failed to save file:\n{e}")

    def refresh_ports(self):
        """Scans for available serial ports and updates the dropdown."""
        port_list = []

        # Real serial ports only (Arduino over USB)
        try:
            ports = serial.tools.list_ports.comports()
            port_list.extend([port.device for port in ports])
        except Exception:
            pass

        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
        else:
            self.port_combo.set('')

    def toggle_connection(self):
        """Handles connecting and disconnecting."""
        if self.comm is None:
            # Connect
            selected_port = self.port_var.get()
            
            if not selected_port:
                messagebox.showwarning("Warning", "No port selected.")
                return

            try:
                # Instantiate the communication class with the selected port
                self.comm = SerialCommunication(port=selected_port, baudrate=115200)

                # SerialCommunication.connect() may fail without throwing; validate we are actually connected.
                if not self._is_hardware_connected():
                    raise ConnectionError("Could not open serial port. Is the Arduino connected and the correct COM port selected?")

                # Connection successful -> allow future warnings again
                self.hardware_disconnected_warned = False

                # Send all-off so every channel is dormant until an action starts
                off_cmd = "<" + ",".join(["off"] * self.num_channels) + ">"
                self.comm.send_command(off_cmd)
                
                # Update UI
                self.btn_connect.config(text="Disconnect")
                self.lbl_status.config(text="Connected", foreground="green")
                self.port_combo.config(state="disabled")
                self.btn_refresh.config(state="disabled")
                self.btn_send.config(state="normal")
                
            except Exception as e:
                messagebox.showerror("Connection Error", f"Could not connect:\n{e}")
                self.comm = None
        else:
            # Disconnect
            try:
                self.comm.close()
            except Exception:
                pass
            self.comm = None
            
            # Update UI
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="Disconnected", foreground="red")
            self.port_combo.config(state="readonly")
            self.btn_refresh.config(state="normal")
            self.btn_send.config(state="disabled")

    def toggle_pause(self):
        """Toggles between Pause and Resume states."""
        if not self.is_paused:
            # --- ENTER PAUSE ---
            self.is_paused = True
            self.last_pause_timestamp = time.time()
            self.btn_pause.config(text="RESUME", bg="green", fg="white")
            self.btn_download_graph.config(state="normal")
            
            # Hold Pressure
            currents = self.latest_pressures[:self.num_channels]
            cmd_str = "<" + ",".join([f"{p:.1f}" for p in currents]) + ">"
            if self.comm:
                ok = self.comm.send_command(cmd_str)
                if not ok and getattr(self.comm, 'disconnected', False):
                    self._handle_hardware_disconnected(getattr(self.comm, 'disconnected_reason', None))
            self.parse_and_store_target(cmd_str) # Update target plot to show holding
            print(f"PAUSED: Holding pressures at {cmd_str}")

            # Setup Scrollbar (always show it in pause mode)
            min_t = None
            max_t = None
            max_channels = min(self.num_channels, len(self.channel_data))
            for ch in range(max_channels):
                t_list = self.channel_data[ch]['t']
                if not t_list:
                    continue
                if min_t is None or t_list[0] < min_t:
                    min_t = t_list[0]
                if max_t is None or t_list[-1] > max_t:
                    max_t = t_list[-1]

            if min_t is None or max_t is None:
                min_t = 0.0
                max_t = 0.0

            if (max_t - min_t) > self.x_timespan:
                # Slider controls end_time; keep x-window within [min_t, max_t]
                slider_min = min_t + self.x_timespan
                slider_max = max_t
                self.history_slider.config(
                    from_=slider_min,
                    to=slider_max,
                    state="normal",
                )
                self.history_slider.set(slider_max)
            else:
                # Not enough history to scroll; show disabled scrollbar anyway.
                self.history_slider.config(
                    from_=0.0,
                    to=max(self.x_timespan, 1.0),
                    state="disabled",
                )
                self.history_slider.set(max_t)

            # Pack above the graphs (just above the canvas)
            self.history_frame.pack(fill=tk.X, padx=5, pady=(0, 5), before=self.canvas.get_tk_widget())
        else:
            # --- RESUME ---
            self.is_paused = False
            paused_duration = time.time() - self.last_pause_timestamp
            self.total_paused_time += paused_duration
            
            # Adjust action start time so the action doesn't expire immediately
            if self.current_action:
                self.action_start_time += paused_duration
            
            self.btn_pause.config(text="PAUSE", bg="orange", fg="black")
            self.btn_download_graph.config(state="disabled")
            self.history_frame.pack_forget() # Hide scrollbar
            
            # Reset view to live
            self.update_loop()

    def cancel_current_action(self):
        """Cancels the currently executing action, if any."""
        if self.current_action:
            print(f"Cancelling action: {self.current_action}")
            self.current_action = None
            self.current_action_data = None
            self.current_action_channels = {}
            self.current_action_times = []
            self.current_action_pressures = []
            self.current_action_indices = []

            # Remove from UI Queue (Head is at index 0)
            if self.queue_listbox.size() > 0:
                self.queue_listbox.delete(0)
            
            # Remove from internal list
            if self.action_queue:
                self.action_queue.pop(0)

    def on_history_scroll(self, value):
        """Updates graph view based on scrollbar position during pause."""
        if not self.is_paused: return

        # Determine available time bounds across channels
        min_t = None
        max_t = None
        max_channels = min(self.num_channels, len(self.channel_data))
        for ch in range(max_channels):
            t_list = self.channel_data[ch]['t']
            if not t_list:
                continue
            if min_t is None or t_list[0] < min_t:
                min_t = t_list[0]
            if max_t is None or t_list[-1] > max_t:
                max_t = t_list[-1]

        if min_t is None or max_t is None:
            min_t = 0.0
            max_t = 0.0

        end_time = float(value)
        # Clamp end_time so the window stays within available history
        if (max_t - min_t) <= self.x_timespan:
            end_time = max_t
            start_time = max(0.0, end_time - self.x_timespan)
        else:
            min_end = min_t + self.x_timespan
            max_end = max_t
            end_time = max(min_end, min(max_end, end_time))
            start_time = end_time - self.x_timespan
        
        for ch_idx, handles in self.plot_handles.items():
            ax = handles['ax']
            ax.set_xlim(start_time, end_time)
            
        self.canvas.draw_idle()

    def parse_and_store_target(self, cmd):
        """Parses a command string <v1,v2...> and updates current_targets.
        Tokens that are 'off' are stored as NaN so the plotter breaks the target line."""
        if cmd.startswith('<') and cmd.endswith('>'):
            content = cmd[1:-1]
            try:
                parts = content.split(',')
                for i, tok in enumerate(parts):
                    if i < len(self.current_targets):
                        tok = tok.strip()
                        if tok.lower() == 'off':
                            self.current_targets[i] = float('nan')
                        else:
                            self.current_targets[i] = float(tok)
            except:
                pass

    def send_command(self):
        """Sends the text from the entry box."""
        if self.comm:
            cmd = self.cmd_entry.get()
            if cmd:
                success = self.comm.send_command(cmd)
                if success:
                    print(f"Sent: {cmd}")
                    self.parse_and_store_target(cmd) # Update targets for plotting
                    # Try to read immediate response
                    response = self.comm.read_response()
                    if getattr(self.comm, 'disconnected', False):
                        self._handle_hardware_disconnected(getattr(self.comm, 'disconnected_reason', None))
                        return
                    if response:
                        print(f"Received: {response}")
                else:
                    if getattr(self.comm, 'disconnected', False):
                        self._handle_hardware_disconnected(getattr(self.comm, 'disconnected_reason', None))
                        return
                    messagebox.showerror("Error", "Failed to send command.")

    def add_to_queue(self):
        """Adds the selected action from library to the queue."""
        selection = self.action_listbox.curselection()
        if selection:
            action_name = self.action_listbox.get(selection[0])
            self.action_queue.append(action_name)
            self.queue_listbox.insert(tk.END, action_name)
            print(f"Added to queue: {action_name}")

    def emergency_stop(self):
        """Stops everything, clears queue, holds pressure, and shows warning."""
        # 1. Clear Queue
        self.action_queue.clear()
        self.queue_listbox.delete(0, tk.END)
        
        # 2. Stop Current Action
        if self.current_action:
            print(f"Action '{self.current_action}' ABORTED by E-STOP.")
            self.current_action = None
            
        # 3. Hold Pressure (Send current actuals as targets)
        currents = self.latest_pressures[:self.num_channels]
        # Format: <p1,p2,...>
        cmd_str = "<" + ",".join([f"{p:.1f}" for p in currents]) + ">"
        
        if self.comm:
            ok = self.comm.send_command(cmd_str)
            if not ok and getattr(self.comm, 'disconnected', False):
                self._handle_hardware_disconnected(getattr(self.comm, 'disconnected_reason', None))
                print(f"E-STOP TRIGGERED: Hardware disconnected; would hold pressures at {cmd_str}")
            else:
                print(f"E-STOP TRIGGERED: Holding pressures at {cmd_str}")
        else:
            print(f"E-STOP TRIGGERED: Not connected; would hold pressures at {cmd_str}")
            
        self.parse_and_store_target(cmd_str) # Update targets

        # 4. Show Warning Window
        self.show_estop_window()

    def show_estop_window(self):
        """Displays the modal E-Stop warning window."""
        win = tk.Toplevel(self.root)
        win.title("EMERGENCY STOP")
        win.geometry("400x200")
        win.configure(bg="#ffcccc") # Light red background
        
        # Center window
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - 200
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - 100
        win.geometry(f"+{x}+{y}")
        
        lbl = tk.Label(win, text="SYSTEM HALTED\n\nQueue Cleared.\nPressures Held at Current Values.", 
                       fg="red", bg="#ffcccc", font=("Arial", 14, "bold"))
        lbl.pack(expand=True, pady=20)
        
        btn_reset = tk.Button(win, text="RESET SYSTEM (Set All to 0 kPa)", 
                              bg="white", fg="black", font=("Arial", 11),
                              command=lambda: self.reset_system(win))
        btn_reset.pack(pady=20, ipadx=10, ipady=5)
        
        # Make it modal (blocks interaction with main window)
        win.transient(self.root)
        win.grab_set()
        self.root.wait_window(win)

    def reset_system(self, window):
        """Resets all targets to 0 and closes the warning window."""
        # Send 0 pressure to all channels
        zero_cmd = "<" + ",".join(["0.0"] * self.num_channels) + ">"
        
        if self.comm:
            ok = self.comm.send_command(zero_cmd)
            if not ok and getattr(self.comm, 'disconnected', False):
                self._handle_hardware_disconnected(getattr(self.comm, 'disconnected_reason', None))
                print(f"System Reset: Hardware disconnected; would send {zero_cmd}")
            else:
                print(f"System Reset: Sent {zero_cmd}")
        else:
            print(f"System Reset: Not connected; would send {zero_cmd}")
        
        self.parse_and_store_target(zero_cmd) # Update targets
            
        window.destroy()
