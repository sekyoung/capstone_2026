import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Workspace limits (metres) — adjust to match your arm
X_MIN, X_MAX = 0.00, 0.25
Y_MIN, Y_MAX = -0.15, 0.15
Z_MIN, Z_MAX = 0.05, 0.30

HOME_X, HOME_Y, HOME_Z = 0.12, 0.0, 0.18
CANVAS_W, CANVAS_H = 400, 400
PUBLISH_HZ = 30


def to_canvas(x, y):
    """World (x, y) → canvas pixel (cx, cy)."""
    cx = (x - X_MIN) / (X_MAX - X_MIN) * CANVAS_W
    cy = (1.0 - (y - Y_MIN) / (Y_MAX - Y_MIN)) * CANVAS_H
    return cx, cy


def from_canvas(cx, cy):
    """Canvas pixel (cx, cy) → world (x, y), clamped."""
    x = cx / CANVAS_W * (X_MAX - X_MIN) + X_MIN
    y = (1.0 - cy / CANVAS_H) * (Y_MAX - Y_MIN) + Y_MIN
    return max(X_MIN, min(X_MAX, x)), max(Y_MIN, min(Y_MAX, y))


class TeleopNode(Node):
    def __init__(self):
        super().__init__('mouse_teleop_node')
        self._pub = self.create_publisher(PoseStamped, '/target_hand_pose', 10)
        self.get_logger().info('Mouse teleop node started')

    def publish(self, x: float, y: float, z: float) -> None:
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)


class TeleopGUI:
    DOT_R = 8

    def __init__(self, node: TeleopNode):
        self._node = node
        self._x = HOME_X
        self._y = HOME_Y
        self._z = HOME_Z

        self._root = tk.Tk()
        self._root.title('Hand Teleop — click/drag to move')
        self._root.resizable(False, False)

        self._build_ui()
        self._redraw()
        self._schedule_publish()

    # ------------------------------------------------------------------ build
    def _build_ui(self):
        frm = ttk.Frame(self._root, padding=8)
        frm.grid()

        # XY canvas
        ttk.Label(frm, text='XY plane  (click or drag)').grid(
            row=0, column=0, columnspan=2, sticky='w')

        self._canvas = tk.Canvas(
            frm, width=CANVAS_W, height=CANVAS_H,
            bg='#1e1e2e', cursor='crosshair',
        )
        self._canvas.grid(row=1, column=0, padx=(0, 8))
        self._canvas.bind('<Button-1>',        self._on_click)
        self._canvas.bind('<B1-Motion>',       self._on_click)

        self._draw_grid()
        self._dot = self._canvas.create_oval(0, 0, 1, 1, fill='#f38ba8', outline='white', width=2)

        # Z slider
        z_frm = ttk.Frame(frm)
        z_frm.grid(row=1, column=1, sticky='ns')

        ttk.Label(z_frm, text='Z').pack()
        self._z_var = tk.DoubleVar(value=HOME_Z)
        self._z_slider = ttk.Scale(
            z_frm, from_=Z_MAX, to=Z_MIN,
            orient='vertical', length=CANVAS_H,
            variable=self._z_var, command=self._on_z,
        )
        self._z_slider.pack(fill='y', expand=True)
        ttk.Label(z_frm, textvariable=self._z_var).pack()

        # Coordinate readout
        self._info = tk.StringVar(value=self._coord_text())
        ttk.Label(frm, textvariable=self._info, font=('Courier', 10)).grid(
            row=2, column=0, columnspan=2, sticky='w', pady=(4, 0))

        # Reset button
        ttk.Button(frm, text='Reset to home', command=self._reset).grid(
            row=3, column=0, columnspan=2, pady=(4, 0), sticky='ew')

    def _draw_grid(self):
        step_x = CANVAS_W / 5
        step_y = CANVAS_H / 5
        for i in range(1, 5):
            self._canvas.create_line(i * step_x, 0, i * step_x, CANVAS_H, fill='#313244')
            self._canvas.create_line(0, i * step_y, CANVAS_W, i * step_y, fill='#313244')
        # axis labels
        for i in range(6):
            xv = X_MIN + i * (X_MAX - X_MIN) / 5
            yv = Y_MAX - i * (Y_MAX - Y_MIN) / 5
            cx = i * CANVAS_W / 5
            cy = i * CANVAS_H / 5
            self._canvas.create_text(cx + 2, CANVAS_H - 12, text=f'{xv:.2f}',
                                     fill='#6c7086', anchor='w', font=('Courier', 7))
            self._canvas.create_text(2, cy + 2, text=f'{yv:.2f}',
                                     fill='#6c7086', anchor='w', font=('Courier', 7))

    # ----------------------------------------------------------------- events
    def _on_click(self, event):
        self._x, self._y = from_canvas(event.x, event.y)
        self._redraw()

    def _on_z(self, _=None):
        self._z = round(self._z_var.get(), 4)
        self._info.set(self._coord_text())

    def _reset(self):
        self._x, self._y, self._z = HOME_X, HOME_Y, HOME_Z
        self._z_var.set(HOME_Z)
        self._redraw()

    # ----------------------------------------------------------------- render
    def _redraw(self):
        cx, cy = to_canvas(self._x, self._y)
        r = self.DOT_R
        self._canvas.coords(self._dot, cx - r, cy - r, cx + r, cy + r)
        self._info.set(self._coord_text())

    def _coord_text(self):
        return f'x={self._x:.3f}  y={self._y:.3f}  z={self._z:.3f}'

    # ----------------------------------------------------------- publish loop
    def _schedule_publish(self):
        self._node.publish(self._x, self._y, self._z)
        self._root.after(int(1000 / PUBLISH_HZ), self._schedule_publish)

    # -------------------------------------------------------------------- run
    def run(self):
        self._root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    # Spin ROS in a background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        TeleopGUI(node).run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
