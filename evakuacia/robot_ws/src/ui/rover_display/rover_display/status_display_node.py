#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import threading
import time
from typing import Iterable

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


def discover_ipv4_addresses() -> list[str]:
    addresses: list[str] = []

    try:
        completed = subprocess.run(
            ['hostname', '-I'],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
        if completed.returncode == 0:
            for token in completed.stdout.split():
                if token and '.' in token and not token.startswith('127.'):
                    if token not in addresses:
                        addresses.append(token)
    except (OSError, subprocess.SubprocessError):
        pass

    if addresses:
        return addresses

    try:
        hostname = socket.gethostname()
        for family, *_rest, sockaddr in socket.getaddrinfo(
            hostname,
            None,
            family=socket.AF_INET,
            type=socket.SOCK_STREAM,
        ):
            if family != socket.AF_INET:
                continue
            ip = sockaddr[0]
            if ip and not ip.startswith('127.') and ip not in addresses:
                addresses.append(ip)
    except socket.gaierror:
        pass

    return addresses


class RoverStatusDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__('rover_status_display_node')

        self.declare_parameter('display_env', ':0')
        self.declare_parameter('xauthority_path', str(Path.home() / '.Xauthority'))
        self.declare_parameter('window_title', 'Rover Display')
        self.declare_parameter('full_screen', True)
        self.declare_parameter('refresh_sec', 1.0)
        self.declare_parameter('background_color', '#07141C')
        self.declare_parameter('panel_color', '#0C202B')
        self.declare_parameter('accent_color', '#16B8F3')
        self.declare_parameter('text_color', '#F4FBFF')
        self.declare_parameter('muted_text_color', '#8CB5C7')
        self.declare_parameter('wifi_interface', 'wlan0')
        self.declare_parameter('wifi_config_script', '/usr/local/sbin/rover-wifi-config.py')
        self.declare_parameter('wifi_netplan_file', '/etc/netplan/50-cloud-init.yaml')
        self.declare_parameter('robot_name_prefix', 'sverk_rover')
        self.declare_parameter('robot_serial', '1')
        self.declare_parameter('right_panel_mode', 'placeholder')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('agent_text_topic', '/agent/text')
        self.declare_parameter('battery_topic', '/battery/state')
        self.declare_parameter('console_shell', '/bin/bash')

        display_env = str(self.get_parameter('display_env').value).strip()
        xauthority_path = str(self.get_parameter('xauthority_path').value).strip()
        if display_env and not os.environ.get('DISPLAY'):
            os.environ['DISPLAY'] = display_env
        if xauthority_path and not os.environ.get('XAUTHORITY'):
            os.environ['XAUTHORITY'] = str(Path(xauthority_path).expanduser())

        try:
            import tkinter as tk
        except Exception as exc:  # pragma: no cover - platform specific
            raise RuntimeError(
                'Tkinter is not available. Install python3-tk or use a desktop image.'
            ) from exc

        self._tk = tk
        self._refresh_ms = max(200, int(float(self.get_parameter('refresh_sec').value) * 1000.0))
        self._window_title = str(self.get_parameter('window_title').value)
        self._full_screen = bool(self.get_parameter('full_screen').value)
        self._background_color = str(self.get_parameter('background_color').value)
        self._panel_color = str(self.get_parameter('panel_color').value)
        self._accent_color = str(self.get_parameter('accent_color').value)
        self._text_color = str(self.get_parameter('text_color').value)
        self._muted_text_color = str(self.get_parameter('muted_text_color').value)
        self._wifi_interface = str(self.get_parameter('wifi_interface').value).strip() or 'wlan0'
        self._wifi_config_script = (
            str(self.get_parameter('wifi_config_script').value).strip()
            or '/usr/local/sbin/rover-wifi-config.py'
        )
        self._wifi_netplan_file = (
            str(self.get_parameter('wifi_netplan_file').value).strip()
            or '/etc/netplan/50-cloud-init.yaml'
        )
        self._robot_name_prefix = (
            str(self.get_parameter('robot_name_prefix').value).strip() or 'sverk_rover'
        )
        self._robot_serial = str(self.get_parameter('robot_serial').value).strip() or '1'
        self._robot_title = f'{self._robot_name_prefix}_{self._robot_serial}'
        self._right_panel_mode = self._normalize_right_panel_mode(
            str(self.get_parameter('right_panel_mode').value)
        )
        self._odom_topic = str(self.get_parameter('odom_topic').value).strip() or '/odom'
        self._agent_text_topic = (
            str(self.get_parameter('agent_text_topic').value).strip() or '/agent/text'
        )
        self._battery_topic = str(self.get_parameter('battery_topic').value).strip() or '/battery/state'
        self._console_shell = str(self.get_parameter('console_shell').value).strip() or '/bin/bash'

        self._networkctl_path = shutil.which('networkctl') or '/usr/bin/networkctl'
        self._sudo_path = shutil.which('sudo') or '/usr/bin/sudo'

        self._root = self._create_window()
        screen_width = max(1, int(self._root.winfo_screenwidth()))
        screen_height = max(1, int(self._root.winfo_screenheight()))
        self._screen_width = screen_width
        self._screen_height = screen_height
        self._ui_scale = max(0.58, min(1.0, screen_width / 1280.0, screen_height / 720.0))
        self._screen_wraplength = max(320, int(screen_width * 0.82))

        self._current_screen = 'main'
        self._apply_in_progress = False
        self._active_entry = None
        self._keyboard_target_var = None
        self._keyboard_shift = False
        self._last_cpu_total: int | None = None
        self._last_cpu_idle: int | None = None
        self._odom_last_monotonic: float | None = None
        self._battery_percent: float | None = None
        self._agent_messages: list[str] = []
        self._console_window = None
        self._console_output = None
        self._console_entry = None

        self._hostname_var = tk.StringVar(value=socket.gethostname())
        self._ip_var = tk.StringVar(value='Поиск адреса...')
        self._top_ip_var = tk.StringVar(value='IP: ...')
        self._ssid_var = tk.StringVar(value='')
        self._password_var = tk.StringVar(value='')
        self._network_status_var = tk.StringVar(value='Экран сетевых настроек активен')
        self._settings_status_var = tk.StringVar(value='Измени параметры сети и нажми применить.')
        self._connected_ssid_var = tk.StringVar(value='SSID: —')
        self._keyboard_title_var = tk.StringVar(value='Ввод')
        self._dashboard_network_var = tk.StringVar(value='SSID: —')
        self._dashboard_ros_state_var = tk.StringVar(value='ROS: проверка...')
        self._dashboard_ros_graph_var = tk.StringVar(value='Ноды: — · Топики: —')
        self._dashboard_cpu_var = tk.StringVar(value='—')
        self._dashboard_memory_var = tk.StringVar(value='—')
        self._dashboard_disk_var = tk.StringVar(value='—')
        self._dashboard_temperature_var = tk.StringVar(value='—')
        self._dashboard_uptime_var = tk.StringVar(value='—')
        self._dashboard_load_var = tk.StringVar(value='—')
        self._odom_position_var = tk.StringVar(value='x: —   y: —')
        self._odom_heading_var = tk.StringVar(value='курс: —')
        self._odom_velocity_var = tk.StringVar(value='скорость: —')
        self._odom_age_var = tk.StringVar(value='odom: нет данных')
        self._battery_var = tk.StringVar(value='—%')

        self._build_layout()
        self._create_ros_subscriptions()
        self._load_current_wifi_config()
        self._show_main_screen()
        self._refresh_status()

    def _scaled(self, value: int, *, minimum: int = 1) -> int:
        return max(minimum, int(round(value * self._ui_scale)))

    def _normalize_right_panel_mode(self, value: str) -> str:
        normalized = value.strip().lower()
        if normalized in {'agent', 'llm', 'hackathon'}:
            return 'agent'
        return 'placeholder'

    def _create_window(self):
        try:
            root = self._tk.Tk()
        except Exception as exc:  # pragma: no cover - platform specific
            raise RuntimeError(
                'Could not open touchscreen window. '
                'Check DISPLAY/XAUTHORITY or launch from the desktop session.'
            ) from exc

        root.title(self._window_title)
        root.configure(bg=self._background_color)
        if self._full_screen:
            root.attributes('-fullscreen', True)
        root.bind('<Escape>', lambda _event: self._shutdown())
        root.protocol('WM_DELETE_WINDOW', self._shutdown)
        return root

    def _build_layout(self) -> None:
        tk = self._tk

        container = tk.Frame(self._root, bg=self._background_color)
        container.pack(
            fill='both',
            expand=True,
            padx=self._scaled(12, minimum=6),
            pady=self._scaled(10, minimum=5),
        )

        top_bar = tk.Frame(
            container,
            bg='#0F2A38',
            highlightbackground='#16495F',
            highlightthickness=1,
            bd=0,
        )
        top_bar.pack(fill='x', pady=(0, self._scaled(8, minimum=4)))

        robot_label = tk.Label(
            top_bar,
            text=self._robot_title,
            bg='#0F2A38',
            fg=self._text_color,
            font=('DejaVu Sans Mono', self._scaled(17, minimum=12), 'bold'),
            anchor='w',
        )
        robot_label.pack(side='left', padx=self._scaled(10, minimum=5), pady=self._scaled(6, minimum=3))

        self._top_ip_button = tk.Button(
            top_bar,
            textvariable=self._top_ip_var,
            command=self._open_settings_screen,
            bg='#0F2A38',
            fg=self._accent_color,
            activebackground='#16495F',
            activeforeground=self._text_color,
            font=('DejaVu Sans Mono', self._scaled(16, minimum=11), 'bold'),
            relief='flat',
            bd=0,
            padx=self._scaled(10, minimum=5),
            pady=self._scaled(6, minimum=3),
            cursor='hand2',
        )
        self._top_ip_button.pack(side='right', padx=self._scaled(8, minimum=4), pady=self._scaled(4, minimum=2))

        panel = tk.Frame(
            container,
            bg=self._panel_color,
            highlightbackground='#12384A',
            highlightthickness=1,
            bd=0,
        )
        panel.pack(fill='both', expand=True)

        self._screen_container = tk.Frame(panel, bg=self._panel_color)
        self._screen_container.pack(fill='both', expand=True)

        self._settings_screen = tk.Frame(self._screen_container, bg=self._panel_color)
        self._menu_screen = tk.Frame(self._screen_container, bg=self._panel_color)
        self._keyboard_screen = tk.Frame(self._screen_container, bg=self._panel_color)

        for frame in (self._settings_screen, self._menu_screen, self._keyboard_screen):
            frame.place(relx=0.0, rely=0.0, relwidth=1.0, relheight=1.0)

        self._build_settings_screen()
        self._build_main_menu_screen()
        self._build_keyboard_screen()

    def _build_settings_screen(self) -> None:
        tk = self._tk

        top_row = tk.Frame(self._settings_screen, bg=self._panel_color)
        top_row.pack(fill='x', padx=self._scaled(14, minimum=8), pady=(self._scaled(12, minimum=6), self._scaled(8, minimum=4)))

        title = tk.Label(
            top_row,
            text='Настройки Wi-Fi',
            bg=self._panel_color,
            fg=self._text_color,
            font=('DejaVu Sans', self._scaled(24, minimum=16), 'bold'),
            anchor='w',
        )
        title.pack(side='left', fill='x', expand=True)

        back_button = tk.Button(
            top_row,
            text='Назад',
            command=self._show_main_screen,
            bg='#12384A',
            fg=self._text_color,
            activebackground=self._accent_color,
            activeforeground=self._background_color,
            font=('DejaVu Sans', self._scaled(14, minimum=10), 'bold'),
            relief='flat',
            bd=0,
            padx=self._scaled(14, minimum=8),
            pady=self._scaled(9, minimum=5),
            cursor='hand2',
        )
        back_button.pack(side='right')

        form = tk.Frame(self._settings_screen, bg=self._panel_color)
        form.pack(fill='x', padx=self._scaled(20, minimum=10))

        ssid_label = tk.Label(
            form,
            text='SSID',
            bg=self._panel_color,
            fg=self._muted_text_color,
            font=('DejaVu Sans', self._scaled(16, minimum=10), 'bold'),
            anchor='w',
        )
        ssid_label.pack(fill='x', pady=(0, self._scaled(4, minimum=2)))

        self._ssid_entry = tk.Entry(
            form,
            textvariable=self._ssid_var,
            font=('DejaVu Sans', self._scaled(18, minimum=11)),
            relief='flat',
            bd=0,
            insertbackground=self._text_color,
            bg='#0F2A38',
            fg=self._text_color,
            justify='left',
        )
        self._ssid_entry.pack(fill='x', ipady=self._scaled(12, minimum=7), pady=(0, self._scaled(12, minimum=6)))
        self._ssid_entry.bind('<FocusIn>', lambda _event: self._show_keyboard(self._ssid_entry))

        password_label = tk.Label(
            form,
            text='Пароль',
            bg=self._panel_color,
            fg=self._muted_text_color,
            font=('DejaVu Sans', self._scaled(16, minimum=10), 'bold'),
            anchor='w',
        )
        password_label.pack(fill='x', pady=(0, self._scaled(4, minimum=2)))

        self._password_entry = tk.Entry(
            form,
            textvariable=self._password_var,
            font=('DejaVu Sans', self._scaled(18, minimum=11)),
            relief='flat',
            bd=0,
            insertbackground=self._text_color,
            bg='#0F2A38',
            fg=self._text_color,
            justify='left',
        )
        self._password_entry.pack(fill='x', ipady=self._scaled(12, minimum=7))
        self._password_entry.bind('<FocusIn>', lambda _event: self._show_keyboard(self._password_entry))

        apply_button = tk.Button(
            self._settings_screen,
            text='Применить',
            command=self._apply_wifi_settings,
            bg=self._accent_color,
            fg=self._background_color,
            activebackground=self._text_color,
            activeforeground=self._background_color,
            font=('DejaVu Sans', self._scaled(18, minimum=11), 'bold'),
            relief='flat',
            bd=0,
            padx=self._scaled(18, minimum=10),
            pady=self._scaled(12, minimum=8),
            cursor='hand2',
        )
        apply_button.pack(pady=(self._scaled(14, minimum=8), self._scaled(10, minimum=6)))

        settings_status = tk.Label(
            self._settings_screen,
            textvariable=self._settings_status_var,
            bg=self._panel_color,
            fg=self._muted_text_color,
            font=('DejaVu Sans', self._scaled(14, minimum=9)),
            wraplength=self._screen_wraplength,
            justify='center',
        )
        settings_status.pack(fill='x', padx=self._scaled(18, minimum=8))

    def _build_info_row(self, parent, title: str, variable, *, monospace: bool = True) -> None:
        row = self._tk.Frame(parent, bg=self._panel_color)
        row.pack(fill='x', pady=self._scaled(4, minimum=2))

        label = self._tk.Label(
            row,
            text=title,
            bg=self._panel_color,
            fg=self._muted_text_color,
            font=('DejaVu Sans', self._scaled(12, minimum=8), 'bold'),
            anchor='w',
            width=11,
        )
        label.pack(side='left')

        value = self._tk.Label(
            row,
            textvariable=variable,
            bg=self._panel_color,
            fg=self._text_color,
            font=('DejaVu Sans Mono' if monospace else 'DejaVu Sans', self._scaled(14, minimum=9), 'bold'),
            anchor='w',
            justify='left',
            wraplength=max(150, int(self._screen_width * 0.42)),
        )
        value.pack(side='left', fill='x', expand=True)

    def _build_placeholder_panel(self, parent) -> None:
        placeholder = self._tk.Label(
            parent,
            text='Правая панель свободна\n\nрежим: placeholder',
            bg=self._panel_color,
            fg=self._muted_text_color,
            font=('DejaVu Sans', self._scaled(18, minimum=12), 'bold'),
            justify='center',
            wraplength=max(180, int(self._screen_width * 0.38)),
        )
        placeholder.pack(fill='both', expand=True)

    def _build_agent_panel(self, parent) -> None:
        top = self._tk.Frame(parent, bg=self._panel_color)
        top.pack(fill='x', pady=(0, self._scaled(8, minimum=4)))

        self._battery_canvas = self._tk.Canvas(
            top,
            width=self._scaled(160, minimum=112),
            height=self._scaled(68, minimum=48),
            bg=self._panel_color,
            highlightthickness=0,
        )
        self._battery_canvas.pack(side='left')

        battery_label = self._tk.Label(
            top,
            textvariable=self._battery_var,
            bg=self._panel_color,
            fg=self._text_color,
            font=('DejaVu Sans Mono', self._scaled(20, minimum=13), 'bold'),
            anchor='w',
        )
        battery_label.pack(side='left', padx=self._scaled(8, minimum=4))

        messenger_frame = self._tk.Frame(
            parent,
            bg='#0F2A38',
            highlightbackground='#16495F',
            highlightthickness=1,
            bd=0,
        )
        messenger_frame.pack(fill='both', expand=True)

        self._agent_text = self._tk.Text(
            messenger_frame,
            bg='#0F2A38',
            fg=self._text_color,
            insertbackground=self._text_color,
            relief='flat',
            bd=0,
            wrap='word',
            font=('DejaVu Sans', self._scaled(13, minimum=9)),
            padx=self._scaled(10, minimum=6),
            pady=self._scaled(10, minimum=6),
            height=8,
        )
        self._agent_text.pack(fill='both', expand=True)
        self._agent_text.insert(
            'end',
            f'Ожидание сообщений из {self._agent_text_topic}...\n',
        )
        self._agent_text.configure(state='disabled')
        self._redraw_battery_icon()

    def _build_main_menu_screen(self) -> None:
        tk = self._tk

        content = tk.Frame(self._menu_screen, bg=self._panel_color)
        content.pack(fill='both', expand=True, padx=self._scaled(12, minimum=6), pady=self._scaled(10, minimum=5))
        content.columnconfigure(0, weight=1, uniform='main')
        content.columnconfigure(1, weight=1, uniform='main')
        content.rowconfigure(0, weight=1)

        left = tk.Frame(content, bg=self._panel_color)
        left.grid(row=0, column=0, sticky='nsew', padx=(0, self._scaled(8, minimum=4)))

        right = tk.Frame(content, bg=self._panel_color)
        right.grid(row=0, column=1, sticky='nsew', padx=(self._scaled(8, minimum=4), 0))

        tech_title = tk.Label(
            left,
            text='Техническая информация',
            bg=self._panel_color,
            fg=self._accent_color,
            font=('DejaVu Sans', self._scaled(17, minimum=11), 'bold'),
            anchor='w',
        )
        tech_title.pack(fill='x', pady=(0, self._scaled(8, minimum=4)))

        self._build_info_row(left, 'IP', self._ip_var)
        self._build_info_row(left, 'Hostname', self._hostname_var, monospace=False)
        self._build_info_row(left, 'Сеть', self._dashboard_network_var, monospace=False)
        self._build_info_row(left, 'ROS', self._dashboard_ros_state_var, monospace=False)
        self._build_info_row(left, 'Graph', self._dashboard_ros_graph_var)
        self._build_info_row(left, 'Odom xy', self._odom_position_var)
        self._build_info_row(left, 'Odom yaw', self._odom_heading_var)
        self._build_info_row(left, 'Odom vel', self._odom_velocity_var)
        self._build_info_row(left, 'Odom age', self._odom_age_var, monospace=False)
        self._build_info_row(left, 'CPU', self._dashboard_cpu_var)
        self._build_info_row(left, 'RAM', self._dashboard_memory_var)
        self._build_info_row(left, 'Диск', self._dashboard_disk_var)
        self._build_info_row(left, 'Темп.', self._dashboard_temperature_var)
        self._build_info_row(left, 'Аптайм', self._dashboard_uptime_var)
        self._build_info_row(left, 'Load', self._dashboard_load_var)

        spacer = tk.Frame(left, bg=self._panel_color)
        spacer.pack(fill='both', expand=True)

        console_button = tk.Button(
            left,
            text='Консоль',
            command=self._open_console_window,
            bg='#12384A',
            fg=self._text_color,
            activebackground=self._accent_color,
            activeforeground=self._background_color,
            font=('DejaVu Sans', self._scaled(12, minimum=9), 'bold'),
            relief='flat',
            bd=0,
            padx=self._scaled(12, minimum=6),
            pady=self._scaled(8, minimum=4),
            cursor='hand2',
        )
        console_button.pack(anchor='sw')

        if self._right_panel_mode == 'agent':
            self._build_agent_panel(right)
        else:
            self._build_placeholder_panel(right)

    def _build_keyboard_screen(self) -> None:
        tk = self._tk

        top_frame = tk.Frame(self._keyboard_screen, bg=self._panel_color)
        top_frame.pack(fill='x', padx=self._scaled(14, minimum=8), pady=(self._scaled(14, minimum=8), self._scaled(10, minimum=6)))

        self._keyboard_title_label = tk.Label(
            top_frame,
            textvariable=self._keyboard_title_var,
            bg=self._panel_color,
            fg=self._muted_text_color,
            font=('DejaVu Sans', self._scaled(15, minimum=10), 'bold'),
            anchor='w',
        )
        self._keyboard_title_label.pack(fill='x', pady=(0, self._scaled(6, minimum=3)))

        input_row = tk.Frame(top_frame, bg=self._panel_color)
        input_row.pack(fill='x')

        self._keyboard_entry = tk.Entry(
            input_row,
            font=('DejaVu Sans', self._scaled(20, minimum=12)),
            relief='flat',
            bd=0,
            insertbackground=self._text_color,
            bg='#F2F7FA',
            fg='#111111',
            justify='left',
        )
        self._keyboard_entry.pack(side='left', fill='x', expand=True, ipady=self._scaled(10, minimum=7), padx=(0, self._scaled(10, minimum=6)))

        close_button = tk.Button(
            input_row,
            text='Выход',
            command=self._close_keyboard,
            bg='#C9CDD2',
            fg='#111111',
            activebackground='#E5E8EC',
            activeforeground='#111111',
            font=('DejaVu Sans', self._scaled(14, minimum=10), 'bold'),
            relief='flat',
            bd=0,
            padx=self._scaled(16, minimum=8),
            pady=self._scaled(12, minimum=7),
            cursor='hand2',
        )
        close_button.pack(side='right')

        self._keyboard_frame = tk.Frame(self._keyboard_screen, bg=self._panel_color)
        self._keyboard_frame.pack(fill='both', expand=True, padx=self._scaled(10, minimum=6), pady=(0, self._scaled(10, minimum=6)))

        rows = [
            list('1234567890'),
            list('qwertyuiop'),
            list('asdfghjkl'),
            ['SHIFT'] + list('zxcvbnm') + ['BACK'],
            ['@', '.', '-', '_', '/', ':', 'SPACE', 'CLEAR', 'DONE'],
        ]

        for row in rows:
            row_frame = self._tk.Frame(self._keyboard_frame, bg=self._panel_color)
            row_frame.pack(fill='both', expand=True, pady=self._scaled(3, minimum=2))
            for key in row:
                if key == 'SPACE':
                    label = 'Пробел'
                elif key == 'BACK':
                    label = '⌫'
                elif key == 'SHIFT':
                    label = '⇧'
                elif key == 'DONE':
                    label = 'Готово'
                elif key == 'CLEAR':
                    label = 'Очист.'
                else:
                    label = key
                button = self._tk.Button(
                    row_frame,
                    text=label,
                    command=lambda value=key: self._on_keyboard_key(value),
                    bg='#12384A',
                    fg=self._text_color,
                    activebackground=self._accent_color,
                    activeforeground=self._background_color,
                    font=('DejaVu Sans', self._scaled(15, minimum=10), 'bold'),
                    relief='flat',
                    bd=0,
                    padx=self._scaled(8, minimum=4),
                    pady=self._scaled(12, minimum=8),
                    cursor='hand2',
                )
                expand = True
                if key in {'SPACE', 'DONE', 'CLEAR'}:
                    expand = False
                button.pack(side='left', fill='both', expand=expand, padx=self._scaled(2, minimum=1))
                if key == 'SPACE':
                    button.configure(width=max(8, int(14 * self._ui_scale)))
                elif key in {'DONE', 'CLEAR'}:
                    button.configure(width=max(5, int(8 * self._ui_scale)))

    def _run_command(self, command: list[str], timeout: float = 5.0) -> subprocess.CompletedProcess[str]:
        try:
            return subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except FileNotFoundError:
            return subprocess.CompletedProcess(command, 127, '', 'command not found')
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout if isinstance(exc.stdout, str) else ''
            stderr = exc.stderr if isinstance(exc.stderr, str) else ''
            if not stderr:
                stderr = f'timeout after {timeout:.1f}s'
            return subprocess.CompletedProcess(command, 124, stdout, stderr)

    def _create_ros_subscriptions(self) -> None:
        self.create_subscription(Odometry, self._odom_topic, self._handle_odom, 10)
        self.create_subscription(String, self._agent_text_topic, self._handle_agent_text, 10)
        self.create_subscription(BatteryState, self._battery_topic, self._handle_battery_state, 10)

    def _handle_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        speed = math.hypot(float(linear.x), float(linear.y))

        self._odom_position_var.set(f'x: {position.x:.2f}   y: {position.y:.2f}')
        self._odom_heading_var.set(f'{math.degrees(yaw):.1f}°')
        self._odom_velocity_var.set(f'{speed:.2f} м/с')
        self._odom_last_monotonic = time.monotonic()
        self._odom_age_var.set('odom: сейчас')

    def _handle_agent_text(self, message: String) -> None:
        text = str(message.data).strip()
        if not text:
            return
        self._agent_messages.append(text)
        self._agent_messages = self._agent_messages[-30:]
        self._append_agent_text(text)

    def _handle_battery_state(self, message: BatteryState) -> None:
        percentage = float(message.percentage)
        if math.isnan(percentage) or percentage < 0.0:
            self._battery_percent = None
            self._battery_var.set('—%')
        else:
            if percentage <= 1.0:
                percentage *= 100.0
            self._battery_percent = max(0.0, min(100.0, percentage))
            self._battery_var.set(f'{self._battery_percent:.0f}%')
        self._redraw_battery_icon()

    def _append_agent_text(self, text: str) -> None:
        widget = getattr(self, '_agent_text', None)
        if widget is None:
            return
        widget.configure(state='normal')
        if not self._agent_messages[:-1]:
            widget.delete('1.0', 'end')
        widget.insert('end', f'{text}\n\n')
        widget.see('end')
        widget.configure(state='disabled')

    def _redraw_battery_icon(self) -> None:
        canvas = getattr(self, '_battery_canvas', None)
        if canvas is None:
            return
        canvas.delete('all')
        width = max(52, int(canvas.cget('width')))
        height = max(22, int(canvas.cget('height')))
        pad = max(3, self._scaled(3, minimum=2))
        body_right = width - self._scaled(10, minimum=7)
        outline_width = max(2, self._scaled(4, minimum=2))
        canvas.create_rectangle(
            pad,
            pad,
            body_right,
            height - pad,
            outline=self._accent_color,
            width=outline_width,
        )
        canvas.create_rectangle(
            body_right,
            height * 0.34,
            width - pad,
            height * 0.66,
            outline=self._accent_color,
            width=outline_width,
        )
        if self._battery_percent is None:
            return
        fill_width = (body_right - pad * 2) * (self._battery_percent / 100.0)
        color = '#3DDC84' if self._battery_percent >= 30.0 else '#FFB020'
        if self._battery_percent < 15.0:
            color = '#FF4D4D'
        canvas.create_rectangle(
            pad + 3,
            pad + 3,
            pad + 3 + fill_width,
            height - pad - 3,
            fill=color,
            outline='',
        )

    def _append_console_output(self, text: str) -> None:
        if self._console_output is None:
            return
        self._console_output.configure(state='normal')
        self._console_output.insert('end', text)
        self._console_output.see('end')
        self._console_output.configure(state='disabled')

    def _open_console_window(self) -> None:
        if self._console_window is not None and self._console_window.winfo_exists():
            self._console_window.lift()
            return

        window = self._tk.Toplevel(self._root)
        window.title('Консоль ровера')
        window.configure(bg=self._background_color)
        window.geometry(f'{self._screen_width}x{self._screen_height}+0+0')
        try:
            window.attributes('-fullscreen', True)
        except Exception:
            pass
        window.bind('<Escape>', lambda _event: self._close_console_window())
        window.protocol('WM_DELETE_WINDOW', self._close_console_window)
        self._console_window = window

        top = self._tk.Frame(window, bg=self._background_color)
        top.pack(fill='x', padx=8, pady=(8, 4))
        title = self._tk.Label(
            top,
            text='Консоль',
            bg=self._background_color,
            fg=self._accent_color,
            font=('DejaVu Sans', self._scaled(14, minimum=10), 'bold'),
            anchor='w',
        )
        title.pack(side='left', fill='x', expand=True)
        close = self._tk.Button(
            top,
            text='Закрыть',
            command=self._close_console_window,
            bg='#12384A',
            fg=self._text_color,
            activebackground=self._accent_color,
            activeforeground=self._background_color,
            relief='flat',
            bd=0,
            padx=10,
            pady=6,
        )
        close.pack(side='right')

        self._console_output = self._tk.Text(
            window,
            bg='#050B10',
            fg=self._text_color,
            insertbackground=self._text_color,
            relief='flat',
            bd=0,
            wrap='word',
            font=('DejaVu Sans Mono', self._scaled(12, minimum=9)),
        )
        self._console_output.pack(fill='both', expand=True, padx=8, pady=4)
        self._console_output.configure(state='disabled')

        bottom = self._tk.Frame(window, bg=self._background_color)
        bottom.pack(fill='x', padx=8, pady=(4, 8))
        self._console_entry = self._tk.Entry(
            bottom,
            bg='#0F2A38',
            fg=self._text_color,
            insertbackground=self._text_color,
            relief='flat',
            bd=0,
            font=('DejaVu Sans Mono', self._scaled(12, minimum=9)),
        )
        self._console_entry.pack(side='left', fill='x', expand=True, ipady=8)
        self._console_entry.bind('<Return>', lambda _event: self._run_console_command())
        run = self._tk.Button(
            bottom,
            text='Выполнить',
            command=self._run_console_command,
            bg=self._accent_color,
            fg=self._background_color,
            activebackground=self._text_color,
            activeforeground=self._background_color,
            relief='flat',
            bd=0,
            padx=10,
            pady=7,
        )
        run.pack(side='right', padx=(8, 0))
        self._append_console_output(f'$ {self._console_shell}\n')
        self._console_entry.focus_set()

    def _close_console_window(self) -> None:
        if self._console_window is not None:
            try:
                self._console_window.destroy()
            except Exception:
                pass
        self._console_window = None
        self._console_output = None
        self._console_entry = None

    def _run_console_command(self) -> None:
        if self._console_entry is None:
            return
        command = self._console_entry.get().strip()
        self._console_entry.delete(0, self._tk.END)
        if not command:
            return
        self._append_console_output(f'\n$ {command}\n')
        thread = threading.Thread(
            target=self._run_console_command_worker,
            args=(command,),
            daemon=True,
        )
        thread.start()

    def _run_console_command_worker(self, command: str) -> None:
        completed = self._run_command([self._console_shell, '-lc', command], timeout=30.0)
        output = completed.stdout
        if completed.stderr:
            output += completed.stderr
        if not output:
            output = f'[exit {completed.returncode}]\n'
        elif completed.returncode != 0:
            output += f'[exit {completed.returncode}]\n'
        self._root.after(0, lambda: self._append_console_output(output))

    def _show_main_screen(self) -> None:
        self._current_screen = 'main'
        self._hide_keyboard()
        self._menu_screen.tkraise()

    def _open_settings_screen(self) -> None:
        self._current_screen = 'settings'
        self._hide_keyboard()
        self._settings_screen.tkraise()

    def _show_keyboard(self, entry) -> None:
        self._active_entry = entry
        if entry == self._ssid_entry:
            self._keyboard_target_var = self._ssid_var
            self._keyboard_title_var.set('SSID')
        else:
            self._keyboard_target_var = self._password_var
            self._keyboard_title_var.set('Пароль')

        if self._keyboard_target_var is not None:
            self._keyboard_entry.configure(textvariable=self._keyboard_target_var)
        self._current_screen = 'keyboard'
        self._keyboard_screen.tkraise()
        self._root.after(10, lambda: self._keyboard_entry.focus_force())

    def _hide_keyboard(self) -> None:
        self._active_entry = None
        self._keyboard_target_var = None
        self._keyboard_entry.configure(textvariable='')

    def _close_keyboard(self) -> None:
        self._hide_keyboard()
        self._open_settings_screen()

    def _on_keyboard_key(self, key: str) -> None:
        if self._keyboard_target_var is None:
            return

        if key == 'SHIFT':
            self._keyboard_shift = not self._keyboard_shift
            return
        if key == 'BACK':
            value = self._keyboard_entry.get()
            if value:
                self._keyboard_entry.delete(len(value) - 1, self._tk.END)
            return
        if key == 'SPACE':
            self._keyboard_entry.insert(self._tk.END, ' ')
            return
        if key == 'CLEAR':
            self._keyboard_entry.delete(0, self._tk.END)
            return
        if key == 'DONE':
            self._close_keyboard()
            return

        symbol = key.upper() if self._keyboard_shift else key
        self._keyboard_entry.insert(self._tk.END, symbol)
        if self._keyboard_shift and len(symbol) == 1 and symbol.isalpha():
            self._keyboard_shift = False

    def _networkctl_status_text(self) -> str:
        completed = self._run_command(
            [self._networkctl_path, 'status', self._wifi_interface],
            timeout=3.0,
        )
        if completed.returncode != 0:
            return ''
        return completed.stdout

    def _extract_connected_ssid(self, status_text: str) -> str:
        marker = 'Wi-Fi access point:'
        for line in status_text.splitlines():
            if marker not in line:
                continue
            after = line.split(marker, 1)[1].strip()
            if not after:
                return ''
            if ' (' in after:
                return after.split(' (', 1)[0].strip()
            return after
        return ''

    def _format_gib(self, value_bytes: int | float) -> str:
        return f'{float(value_bytes) / (1024.0 ** 3):.1f} ГБ'

    def _read_cpu_text(self) -> str:
        try:
            fields = Path('/proc/stat').read_text(encoding='utf-8').splitlines()[0].split()
            values = [int(item) for item in fields[1:]]
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            total = sum(values)
        except (OSError, IndexError, ValueError):
            return '—'

        if self._last_cpu_total is None or self._last_cpu_idle is None:
            self._last_cpu_total = total
            self._last_cpu_idle = idle
            return 'измерение...'

        total_delta = max(0, total - self._last_cpu_total)
        idle_delta = max(0, idle - self._last_cpu_idle)
        self._last_cpu_total = total
        self._last_cpu_idle = idle
        if total_delta <= 0:
            return '—'
        usage = 100.0 * (1.0 - idle_delta / total_delta)
        return f'{usage:.0f}%'

    def _read_memory_text(self) -> str:
        try:
            values: dict[str, int] = {}
            for line in Path('/proc/meminfo').read_text(encoding='utf-8').splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    values[parts[0].rstrip(':')] = int(parts[1])
            total_kib = values['MemTotal']
            available_kib = values.get('MemAvailable', values.get('MemFree', 0))
        except (OSError, KeyError, ValueError):
            return '—'

        used_kib = max(0, total_kib - available_kib)
        percent = used_kib / total_kib * 100.0 if total_kib else 0.0
        used_gib = used_kib * 1024.0
        total_gib = total_kib * 1024.0
        return f'{percent:.0f}% · {self._format_gib(used_gib)}/{self._format_gib(total_gib)}'

    def _read_disk_text(self) -> str:
        try:
            usage = shutil.disk_usage('/')
        except OSError:
            return '—'
        percent = usage.used / usage.total * 100.0 if usage.total else 0.0
        return f'{percent:.0f}% · свободно {self._format_gib(usage.free)}'

    def _read_temperature_text(self) -> str:
        for path in (
            Path('/sys/class/thermal/thermal_zone0/temp'),
            Path('/sys/class/hwmon/hwmon0/temp1_input'),
        ):
            try:
                raw = path.read_text(encoding='utf-8').strip()
                if raw:
                    return f'{float(raw) / 1000.0:.1f} °C'
            except (OSError, ValueError):
                continue
        return '—'

    def _read_uptime_text(self) -> str:
        try:
            seconds = int(float(Path('/proc/uptime').read_text(encoding='utf-8').split()[0]))
        except (OSError, IndexError, ValueError):
            return '—'
        days, seconds = divmod(seconds, 24 * 3600)
        hours, seconds = divmod(seconds, 3600)
        minutes = seconds // 60
        if days:
            return f'{days}д {hours}ч {minutes}м'
        if hours:
            return f'{hours}ч {minutes}м'
        return f'{minutes}м'

    def _read_load_text(self) -> str:
        try:
            one, five, fifteen = os.getloadavg()
            return f'{one:.2f} · {five:.2f} · {fifteen:.2f}'
        except OSError:
            return '—'

    def _refresh_ros_dashboard(self) -> None:
        if not rclpy.ok():
            self._dashboard_ros_state_var.set('не работает')
            self._dashboard_ros_graph_var.set('Ноды: — · Топики: —')
            return

        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception:
            pass

        try:
            nodes = self.get_node_names()
            topics = self.get_topic_names_and_types()
            services = self.get_service_names_and_types()
        except Exception as exc:
            self._dashboard_ros_state_var.set('ошибка ROS')
            self._dashboard_ros_graph_var.set(str(exc))
            return

        self._dashboard_ros_state_var.set('работает')
        self._dashboard_ros_graph_var.set(
            f'Ноды: {len(nodes)} · Топики: {len(topics)} · Сервисы: {len(services)}'
        )

    def _refresh_dashboard_status(self, connected_ssid: str) -> None:
        self._dashboard_network_var.set(f'SSID: {connected_ssid or "—"}')
        if self._odom_last_monotonic is None:
            self._odom_age_var.set('odom: нет данных')
        else:
            self._odom_age_var.set(
                f'odom: {time.monotonic() - self._odom_last_monotonic:.1f} сек назад'
            )
        self._dashboard_cpu_var.set(self._read_cpu_text())
        self._dashboard_memory_var.set(self._read_memory_text())
        self._dashboard_disk_var.set(self._read_disk_text())
        self._dashboard_temperature_var.set(self._read_temperature_text())
        self._dashboard_uptime_var.set(self._read_uptime_text())
        self._dashboard_load_var.set(self._read_load_text())
        self._refresh_ros_dashboard()

    def _load_current_wifi_config(self) -> None:
        if not Path(self._wifi_config_script).exists():
            self._settings_status_var.set('Не найден helper для чтения текущей Wi-Fi конфигурации.')
            return

        completed = self._run_command(
            [
                self._sudo_path,
                '-n',
                self._wifi_config_script,
                'current',
                self._wifi_interface,
                self._wifi_netplan_file,
            ],
            timeout=5.0,
        )
        if completed.returncode != 0:
            details = completed.stderr.strip() or completed.stdout.strip()
            if 'password is required' in details.lower():
                self._settings_status_var.set('Нужен sudoers NOPASSWD для rover-wifi-config.py.')
            elif details:
                self._settings_status_var.set(details)
            return

        ssid = ''
        password = ''
        for line in completed.stdout.splitlines():
            if line.startswith('SSID='):
                ssid = line.split('=', 1)[1]
            elif line.startswith('PASSWORD='):
                password = line.split('=', 1)[1]

        if ssid:
            self._ssid_var.set(ssid)
        if password:
            self._password_var.set(password)

    def _apply_wifi_settings(self) -> None:
        if self._apply_in_progress:
            return

        ssid = self._ssid_var.get().strip()
        password = self._password_var.get()
        if not ssid:
            self._settings_status_var.set('SSID не должен быть пустым.')
            return
        if len(password) < 8:
            self._settings_status_var.set('Пароль Wi-Fi должен быть не короче 8 символов.')
            return

        self._apply_in_progress = True
        self._settings_status_var.set('Применение Wi-Fi настроек...')
        self._network_status_var.set('Применение Wi-Fi настроек...')
        self._close_keyboard()
        self._top_ip_button.configure(state='disabled')

        thread = threading.Thread(
            target=self._apply_wifi_settings_worker,
            args=(ssid, password),
            daemon=True,
        )
        thread.start()

    def _apply_wifi_settings_worker(self, ssid: str, password: str) -> None:
        result = self._run_command(
            [
                self._sudo_path,
                '-n',
                self._wifi_config_script,
                'apply',
                self._wifi_interface,
                self._wifi_netplan_file,
                ssid,
                password,
            ],
            timeout=40.0,
        )

        success = result.returncode == 0
        if success:
            message = f'Сеть {ssid} применена.'
        else:
            details = result.stderr.strip() or result.stdout.strip() or 'Ошибка применения Wi-Fi.'
            if 'password is required' in details.lower():
                message = 'Нужен sudoers NOPASSWD для rover-wifi-config.py.'
            else:
                message = details

        self._root.after(
            0,
            lambda: self._finish_apply_wifi_settings(success=success, message=message),
        )

    def _finish_apply_wifi_settings(self, *, success: bool, message: str) -> None:
        self._apply_in_progress = False
        self._top_ip_button.configure(state='normal')
        self._settings_status_var.set(message)
        self._network_status_var.set(message)
        if success:
            self._show_main_screen()
        self._refresh_status(reschedule=False)

    def _refresh_status(self, *, reschedule: bool = True) -> None:
        addresses = discover_ipv4_addresses()
        self._ip_var.set('\n'.join(addresses) if addresses else 'Сеть не подключена')
        self._top_ip_var.set(addresses[0] if addresses else 'нет сети')

        status_text = self._networkctl_status_text()
        connected_ssid = self._extract_connected_ssid(status_text)
        self._connected_ssid_var.set(f'SSID: {connected_ssid or "—"}')
        self._refresh_dashboard_status(connected_ssid)

        if self._current_screen != 'main' and not self._apply_in_progress:
            if connected_ssid:
                self._network_status_var.set(f'Подключено к сети {connected_ssid}')
            else:
                self._network_status_var.set('Сеть не подключена')

        if reschedule and rclpy.ok():
            self._root.after(self._refresh_ms, self._refresh_status)

    def _shutdown(self) -> None:
        try:
            self._root.quit()
            self._root.destroy()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    def run(self) -> None:
        self.get_logger().info('Touchscreen display started')
        self._root.mainloop()


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RoverStatusDisplayNode | None = None
    try:
        node = RoverStatusDisplayNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
