import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import tkinter.font as tkfont

# Try importing pyserial, show friendly installer notice if missing
pyserial_installed = True
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    pyserial_installed = False

class CalibApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("TARS Servo Calibration Control Center")
        self.geometry("900x650")
        self.configure(bg="#0F0F13")
        
        # Keep track of serial connection
        self.ser = None
        self.read_thread = None
        self.running = False
        
        # Design Styles & Color Palette (Dark Theme / Catppuccin inspired)
        self.colors = {
            "bg": "#0F0F13",
            "card_bg": "#181825",
            "fg": "#CDD6F4",
            "fg_muted": "#A6ADC8",
            "accent": "#CBA6F7",      # Lavender
            "lh": "#F38BA8",          # Pink/Red
            "rh": "#A6E3A1",          # Green
            "ll": "#89B4FA",          # Blue
            "rl": "#F9E2AF",          # Yellow
            "btn_bg": "#313244",
            "btn_hover": "#45475A",
            "btn_active": "#585B70",
            "border": "#2b2b3c"
        }
        
        # Setup modern font styles
        self.title_font = tkfont.Font(family="Segoe UI", size=16, weight="bold")
        self.header_font = tkfont.Font(family="Segoe UI", size=12, weight="bold")
        self.body_font = tkfont.Font(family="Segoe UI", size=10)
        self.btn_font = tkfont.Font(family="Segoe UI", size=10, weight="bold")
        self.log_font = tkfont.Font(family="Consolas", size=9)

        # Style configuration
        self.style = ttk.Style()
        self.style.theme_use("clam")
        self.style.configure(".", background=self.colors["bg"], foreground=self.colors["fg"])
        self.style.configure("TLabel", background=self.colors["bg"], foreground=self.colors["fg"], font=self.body_font)
        
        self.build_ui()
        
        if not pyserial_installed:
            messagebox.showwarning(
                "Missing Dependency",
                "The 'pyserial' package is not installed.\n\n"
                "Please run the following command in your terminal:\n"
                "pip install pyserial\n\n"
                "Then restart this tool."
            )
            self.write_log("[System Error] pyserial library not found. Connect functionality will be disabled.\n")

    def build_ui(self):
        # Header Title
        title_frame = tk.Frame(self, bg=self.colors["bg"], height=60)
        title_frame.pack(fill="x", padx=20, pady=10)
        
        title_label = tk.Label(title_frame, text="TARS SERVO CALIBRATION CONTROL CENTER", 
                               font=self.title_font, fg=self.colors["accent"], bg=self.colors["bg"])
        title_label.pack(side="left")
        
        subtitle = tk.Label(title_frame, text=" v2.0 (GUI Edition)", font=self.body_font, 
                            fg=self.colors["fg_muted"], bg=self.colors["bg"])
        subtitle.pack(side="left", padx=5, pady=5)
        
        # -------------------------------------------------------------
        # Connection Panel
        # -------------------------------------------------------------
        conn_frame = tk.LabelFrame(self, text=" 1. Serial Connection ", font=self.header_font,
                                    bg=self.colors["card_bg"], fg=self.colors["accent"], 
                                    bd=1, relief="solid", highlightbackground=self.colors["border"])
        conn_frame.pack(fill="x", padx=20, pady=5)
        
        # Inner padding frame
        conn_inner = tk.Frame(conn_frame, bg=self.colors["card_bg"])
        conn_inner.pack(fill="x", padx=15, pady=10)
        
        tk.Label(conn_inner, text="Port:", bg=self.colors["card_bg"], fg=self.colors["fg"]).pack(side="left", padx=5)
        
        self.port_combobox = ttk.Combobox(conn_inner, width=15, state="readonly")
        self.port_combobox.pack(side="left", padx=5)
        self.refresh_ports()
        
        # Refresh ports button
        btn_refresh = tk.Button(conn_inner, text="↻ Refresh", bg=self.colors["btn_bg"], fg=self.colors["fg"],
                                bd=0, activebackground=self.colors["btn_active"], activeforeground=self.colors["fg"],
                                font=self.body_font, command=self.refresh_ports, padx=8, pady=2)
        btn_refresh.pack(side="left", padx=5)
        self.bind_hover(btn_refresh)
        
        tk.Label(conn_inner, text="Baud Rate:", bg=self.colors["card_bg"], fg=self.colors["fg"]).pack(side="left", padx=15)
        self.baud_combobox = ttk.Combobox(conn_inner, width=10, state="readonly", values=["9600", "115200", "921600"])
        self.baud_combobox.set("115200")
        self.baud_combobox.pack(side="left", padx=5)
        
        self.btn_connect = tk.Button(conn_inner, text="Connect", bg=self.colors["accent"], fg="#0F0F13",
                                     bd=0, activebackground=self.colors["btn_active"], activeforeground=self.colors["fg"],
                                     font=self.btn_font, command=self.toggle_connection, padx=15, pady=2)
        self.btn_connect.pack(side="right", padx=5)
        
        # -------------------------------------------------------------
        # Servo Controls Grid Panel
        # -------------------------------------------------------------
        ctrl_frame = tk.LabelFrame(self, text=" 2. Fine-Tuning Calibration Controls ", font=self.header_font,
                                    bg=self.colors["bg"], fg=self.colors["accent"], bd=0)
        ctrl_frame.pack(fill="x", padx=20, pady=10)
        
        # 4 Columns for the 4 servo angles
        servos_config = [
            {"title": "Left Height (LH)", "sub": "Stand Angle", "color": self.colors["lh"], "up_char": "q", "dn_char": "a", "up_lbl": "LH Stand +1 (q)", "dn_lbl": "LH Stand -1 (a)"},
            {"title": "Right Height (RH)", "sub": "Stand Angle", "color": self.colors["rh"], "up_char": "w", "dn_char": "s", "up_lbl": "RH Stand +1 (w)", "dn_lbl": "RH Stand -1 (s)"},
            {"title": "Left Leg (LL)", "sub": "Neutral Angle", "color": self.colors["ll"], "up_char": "e", "dn_char": "d", "up_lbl": "LL Neutral +1 (e)", "dn_lbl": "LL Neutral -1 (d)"},
            {"title": "Right Leg (RL)", "sub": "Neutral Angle", "color": self.colors["rl"], "up_char": "r", "dn_char": "f", "up_lbl": "RL Neutral +1 (r)", "dn_lbl": "RL Neutral -1 (f)"}
        ]
        
        self.buttons = [] 
        for i, config in enumerate(servos_config):
            card = tk.Frame(ctrl_frame, bg=self.colors["card_bg"], bd=1, relief="solid", highlightbackground=self.colors["border"])
            card.grid(row=0, column=i, padx=5, sticky="nsew")
            ctrl_frame.columnconfigure(i, weight=1)
            
            # Card header
            hdr = tk.Frame(card, bg=config["color"], height=4)
            hdr.pack(fill="x")
            
            lbl_title = tk.Label(card, text=config["title"], font=self.header_font, fg=config["color"], bg=self.colors["card_bg"])
            lbl_title.pack(pady=(12, 2))
            
            lbl_sub = tk.Label(card, text=config["sub"], font=self.body_font, fg=self.colors["fg_muted"], bg=self.colors["card_bg"])
            lbl_sub.pack(pady=(0, 10))
            
            # Plus button
            btn_up = tk.Button(card, text="▲  +5", bg=self.colors["btn_bg"], fg=config["color"],
                               bd=0, activebackground=config["color"], activeforeground=self.colors["bg"],
                               font=self.btn_font, command=lambda c=config["up_char"]: self.send_command(c, repeat=5), pady=12)
            btn_up.pack(fill="x", padx=15, pady=5)
            self.bind_hover(btn_up, hover_color=config["color"], text_hover=self.colors["bg"])
            self.buttons.append(btn_up)
            
            # Minus button
            btn_dn = tk.Button(card, text="▼  -5", bg=self.colors["btn_bg"], fg=config["color"],
                               bd=0, activebackground=config["color"], activeforeground=self.colors["bg"],
                               font=self.btn_font, command=lambda c=config["dn_char"]: self.send_command(c, repeat=5), pady=12)
            btn_dn.pack(fill="x", padx=15, pady=(5, 15))
            self.bind_hover(btn_dn, hover_color=config["color"], text_hover=self.colors["bg"])
            self.buttons.append(btn_dn)
            
        # -------------------------------------------------------------
        # Action Buttons Panel (Save / Reset)
        # -------------------------------------------------------------
        action_frame = tk.Frame(self, bg=self.colors["bg"])
        action_frame.pack(fill="x", padx=20, pady=10)
        
        btn_reset = tk.Button(action_frame, text="⟲ Reset / Re-apply Angle (SPACE)", bg="#45475A", fg=self.colors["fg"],
                              bd=0, activebackground=self.colors["btn_active"], activeforeground=self.colors["fg"],
                              font=self.btn_font, command=lambda: self.send_command(" "), padx=20, pady=10)
        btn_reset.pack(side="left", padx=5)
        self.bind_hover(btn_reset)
        self.buttons.append(btn_reset)
        
        btn_save = tk.Button(action_frame, text="💾 SAVE CALIBRATION TO NVS (Shift+S)", bg=self.colors["accent"], fg="#0F0F13",
                             bd=0, activebackground=self.colors["btn_active"], activeforeground=self.colors["fg"],
                             font=self.btn_font, command=lambda: self.send_command("S"), padx=25, pady=10)
        btn_save.pack(side="right", padx=5)
        self.buttons.append(btn_save)
        
        # -------------------------------------------------------------
        # Testing Dashboard
        # -------------------------------------------------------------
        test_frame = tk.LabelFrame(self, text=" 3. Testing Dashboard (Live Movement) ", font=self.header_font,
                                   bg=self.colors["bg"], fg=self.colors["accent"], bd=0)
        test_frame.pack(fill="x", padx=20, pady=10)
        
        test_btn_container = tk.Frame(test_frame, bg=self.colors["bg"])
        test_btn_container.pack(pady=5)
        
        test_buttons = [
            ("Walk Fwd (i)", "i"),
            ("Walk Back (k)", "k"),
            ("Turn Left (j)", "j"),
            ("Turn Right (l)", "l"),
            ("Small Step Fwd (m)", "m"),
            ("Small Step Back (n)", "n"),
            ("Relax Servos (x)", "x")
        ]
        
        for text, char in test_buttons:
            btn = tk.Button(test_btn_container, text=text, bg=self.colors["card_bg"], fg=self.colors["fg"],
                            bd=1, relief="solid", highlightbackground=self.colors["border"],
                            activebackground=self.colors["btn_active"], activeforeground=self.colors["fg"],
                            font=self.btn_font, command=lambda c=char: self.send_command(c), padx=15, pady=8)
            btn.pack(side="left", padx=5)
            self.bind_hover(btn)
            
        # -------------------------------------------------------------
        # Console Log Panel
        # -------------------------------------------------------------
        log_lbl_frame = tk.LabelFrame(self, text=" 4. Live Feedback Console (Raw Serial Input) ", font=self.header_font,
                                       bg=self.colors["card_bg"], fg=self.colors["accent"], 
                                       bd=1, relief="solid", highlightbackground=self.colors["border"])
        log_lbl_frame.pack(fill="both", expand=True, padx=20, pady=(5, 20))
        
        # Text widget for console output
        self.log_text = tk.Text(log_lbl_frame, bg="#0b0b0f", fg="#A6E3A1", font=self.log_font, bd=0, insertbackground="white")
        self.log_text.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        
        scrollbar = ttk.Scrollbar(log_lbl_frame, command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y", pady=10, padx=(0, 10))
        self.log_text.config(yscrollcommand=scrollbar.set)
        
        # Keyboard shortcuts bindings
        self.bind("<KeyPress>", self.handle_key_press)
        
    def refresh_ports(self):
        if not pyserial_installed:
            self.port_combobox["values"] = ["No Serial Library"]
            self.port_combobox.set("No Serial Library")
            return
            
        ports = serial.tools.list_ports.comports()
        port_list = [p.device for p in ports]
        
        if port_list:
            self.port_combobox["values"] = port_list
            # Autoselect COM3 if present, else the first portm (chọn cổng)
            if "COM3" in port_list:
                self.port_combobox.set("COM3")
            else:
                self.port_combobox.set(port_list[0])
            if "COM4" in port_list:
                self.port_combobox.set("COM4")
            else:
                self.port_combobox.set(port_list[0]) 
        else:
            self.port_combobox["values"] = []
            self.port_combobox.set("No Ports Found")

    def toggle_connection(self):
        if not pyserial_installed:
            messagebox.showerror("Error", "pyserial library is not installed.")
            return
            
        if self.ser and self.ser.is_open:
            # Disconnect
            self.running = False
            if self.ser:
                self.ser.close()
            self.ser = None
            self.btn_connect.config(text="Connect", bg=self.colors["accent"], fg="#0F0F13")
            self.write_log("\n[Disconnected]\n")
        else:
            # Connect
            port = self.port_combobox.get()
            if not port or port == "No Ports Found" or port == "No Serial Library":
                messagebox.showwarning("Warning", "Please select a valid COM port.")
                return
            
            try:
                baud = int(self.baud_combobox.get())
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.running = True
                self.btn_connect.config(text="Disconnect", bg="#F38BA8", fg=self.colors["bg"])
                
                # Start read thread
                self.read_thread = threading.Thread(target=self.read_serial_thread, daemon=True)
                self.read_thread.start()
                
                self.write_log(f"[Connected to {port} @ {baud} baud]\n")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to connect: {str(e)}")
                self.ser = None

    def read_serial_thread(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    text = data.decode('utf-8', errors='ignore')
                    self.write_log(text)
            except Exception as e:
                self.write_log(f"\n[Error reading serial: {str(e)}]\n")
                break
            time.sleep(0.01)
            
    def write_log(self, text):
        # Schedule UI update safely on the main thread
        self.after(0, self._append_log, text)
        
    def _append_log(self, text):
        self.log_text.insert("end", text)
        self.log_text.see("end")

    def send_command(self, char, repeat=1):
        if not self.ser or not self.ser.is_open:
            self.write_log(f"[Local Info] Key '{char}' pressed. (Not connected)\n")
            return
        try:
            payload = char * repeat
            self.ser.write(payload.encode('utf-8'))
            self.ser.flush()
            if char == " ":
                self.write_log("[Sent Command: Reset/Apply]\n")
            elif char == "S":
                self.write_log("[Sent Command: SAVE TO NVS]\n")
            elif char in ['i', 'k', 'j', 'l', 'm', 'n']:
                self.write_log(f"[Sent Command: Test Movement '{char}']\n")
        except Exception as e:
            self.write_log(f"\n[Failed to send command '{char}': {str(e)}]\n")

    def handle_key_press(self, event):
        focused_widget = self.focus_get()
        if isinstance(focused_widget, tk.Text):
            return
            
        char = event.char
        if event.keysym == "space":
            self.send_command(" ")
            return
            
        valid_keys = ['q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 'S', 'i', 'k', 'j', 'l', 'm', 'n']
        if char in valid_keys:
            # Send 5 times for visible movement for calibration adjusting keys
            calib_keys = ['q', 'a', 'w', 's', 'e', 'd', 'r', 'f']
            repeat = 5 if char in calib_keys else 1
            self.send_command(char, repeat)

    def bind_hover(self, widget, hover_color=None, text_hover=None):
        bg_orig = widget.cget("background")
        fg_orig = widget.cget("foreground")
        
        h_color = hover_color if hover_color else self.colors["btn_hover"]
        t_color = text_hover if text_hover else fg_orig
        
        def on_enter(e):
            widget.config(background=h_color, foreground=t_color)
        def on_leave(e):
            widget.config(background=bg_orig, foreground=fg_orig)
            
        widget.bind("<Enter>", on_enter)
        widget.bind("<Leave>", on_leave)

if __name__ == "__main__":
    app = CalibApp()
    app.mainloop()
