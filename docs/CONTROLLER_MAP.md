# Controller Input Map

Reference for controller teleop with `ros2 launch lunabot_2425 joy_teleop.launch.py`. Luna uses holonomic drive (forward/back + strafe + rotate). **No input = no motion:** hold the enable button to drive; release to stop.

---

## Input Map (used by `joy_teleop.yaml`)

| Input | Index | Used for |
|---|---:|---|
| Left stick Y | axis `1` | `linear.x` (forward/back) |
| Left stick X | axis `0` | `linear.y` (holonomic strafe) |
| Right stick X | axis `3` | `angular.z` (yaw/rotate) |
| A (enable) | button `0` | deadman enable (hold to drive) |

---

## How to find your indices

Controller axis/button indices vary by vendor (Xbox, PS, generic). To discover yours:

1. Plug in the controller and start the stack (Gazebo + RTAB-Map, then `ros2 launch lunabot_2425 joy_teleop.launch.py`).
2. In another terminal:
   ```bash
   ros2 topic echo /joy
   ```
3. Move each stick and press each button. Note which `axes` or `buttons` index changes:
   - `axes[0]`, `axes[1]` = left stick X, Y
   - `axes[2]`, `axes[3]` = often right stick or triggers
   - `buttons[0]` = typically A / X
4. Update `lunabot_2425/config/joy_teleop.yaml` and this table if your controller differs.

---

## Printouts (joy_echo)

Enabled by default. You should see `[JoyEcho]` lines when the controller sends input.

To disable: `ros2 launch lunabot_2425 joy_teleop.launch.py joy_echo:=false`

Output examples: `[JoyEcho] Forward`, `[JoyEcho] Strafe left`, `[JoyEcho] Enable (A) pressed — you can drive with sticks`, `[JoyEcho] Enable (A) released — robot should stop`.

If you see **no** `[JoyEcho]` lines after a few seconds, the controller may not be publishing (`joy_node`). Check `ls /dev/input/js*` and `ros2 topic echo /joy` inside the container.
