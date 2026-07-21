#!/usr/bin/env python3
"""
LLM Navigation node — plain-English commands → Nav2 goal.

Pipeline:
  text topic/input → ollama LLM → NavigateToPose action

Usage
─────
  ros2 run diff_drive_robot llm_nav.py

Deps:
  ollama must be running: `ollama serve`
"""

import json
import math
import os
import re
import threading
import time
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ── helpers — do not modify ───────────────────────────────────────────────────

def _load_locations(share_dir: str) -> dict:
    candidates = [
        os.path.join(share_dir, 'config', 'locations.yaml'),
        os.path.join(os.path.expanduser('~'), 'rosnav', 'locations.yaml'),
    ]
    try:
        import yaml
    except ImportError:
        return {}
    for p in candidates:
        if os.path.isfile(p):
            with open(p) as f:
                data = yaml.safe_load(f) or {}
            return data.get('locations', {})
    return {}


def _yaw_to_quat(yaw_deg: float):
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ── TODO 1 — System prompt ────────────────────────────────────────────────────

# Define _SYSTEM as a module-level string that will be used by _parse_command().
#
# The prompt must instruct the LLM to return ONLY a JSON object — no preamble,
# no explanation, no markdown. It should handle four cases:
#   - Go to a named location  → {"action":"go","location":"<name>"}
#   - Go to coordinates       → {"action":"go","x":<float>,"y":<float>,"yaw":0.0}
#   - Stop                    → {"action":"stop"}
#   - Anything else           → {"action":"unknown","reason":"<why>"}
#
# The prompt receives two format placeholders at call time:
#   {locations} — comma-separated list of known location names
#   {command}   — the raw text the user typed
#
# Include at least two examples so the model has concrete output patterns to
# follow. The quality of your prompt directly determines whether the LLM
# returns clean JSON or unparseable text — experiment with it.

_SYSTEM = """
You are a robot navigation assistant.

You MUST return ONLY ONE valid JSON object.

Do not write explanations.
Do not write markdown.
Do not write code fences.
Do not write any extra text.

Available locations:
{locations}

User command:
{command}

Rules:

1. If the command is:

go to <location>

and <location> is one of the available locations, return EXACTLY

{{"action":"go","location":"<location>"}}

2. If the command is:

go to <number> <number>

return EXACTLY

{{"action":"go","x":<number>,"y":<number>,"yaw":0.0}}

Correct example:

{{"action":"go","x":1.0,"y":2.0,"yaw":0.0}}

WRONG:

{{"action":"go","location":"1.0 2.0"}}

WRONG:

{{"action":"go","location":{{"x":1.0,"y":2.0}}}}

Never put coordinates inside "location".

3. If the command is:

stop

return EXACTLY

{{"action":"stop"}}

4. If the command is unknown return EXACTLY

{{"action":"unknown","reason":"unknown command"}}
"""

# ── TODO 2 — Ollama API call ──────────────────────────────────────────────────

def call_ollama(model: str, prompt: str, base_url: str = 'http://localhost:11434') -> str:
    """
    Send a completion request to a locally running Ollama instance and return
    the model's response string.

    Endpoint: POST {base_url}/api/generate
    Request body (JSON):
      model   — the model name (e.g. "tinyllama")
      prompt  — the full prompt string
      stream  — False (we want a single complete response, not a stream)
      format  — "json" (instructs Ollama to constrain output to valid JSON)

    The response body is JSON. The model's text is in response["response"].
    Return that string.
    """

    body = json.dumps({
        "model": model,
        "prompt": prompt,
        "stream": False,
        "format": "json"
    }).encode("utf-8")

    req = urllib.request.Request(
        f"{base_url}/api/generate",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST"
    )

    with urllib.request.urlopen(req) as resp:
        response = json.loads(resp.read().decode("utf-8"))

    return response["response"]



# ── TODO 3 — JSON extraction ──────────────────────────────────────────────────

def _extract_json(text: str) -> dict | None:
    start = text.find("{")
    end = text.rfind("}")

    if start == -1 or end == -1 or end <= start:
        return None

    try:
        return json.loads(text[start:end + 1])
    except json.JSONDecodeError:
        return None


#def _extract_json(text: str) -> dict | None:
    """
    LLMs often wrap their JSON in prose or markdown. This function defensively
    extracts the first {...} block from the raw output string and parses it.

    Return the parsed dict if a valid JSON object is found, None otherwise.
    """
#    match = re.search(r'\{.*?\}', text, re.DOTALL)
#
#   if not match:
#        return None
#
#   try:
#        return json.loads(match.group(0))
#    except json.JSONDecodeError:
#        return None


# ── ROS node ──────────────────────────────────────────────────────────────────

class LLMNavigator(Node):
    def __init__(self):
        super().__init__('llm_navigator')

        self.declare_parameter('ollama_model', 'phi3:mini')
        self.declare_parameter('ollama_url',   'http://localhost:11434')
        self.declare_parameter('nav_action',   'navigate_to_pose')
        self.declare_parameter('frame_id',     'map')

        g = self.get_parameter
        self._ollama_model = g('ollama_model').value
        self._ollama_url   = g('ollama_url').value
        self._frame_id     = g('frame_id').value

        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('diff_drive_robot')
        except Exception:
            share = os.path.join(
                os.path.expanduser('~'), 'rosnav', 'src', 'diff_drive_robot-main')
        self._locations = _load_locations(share)
        self.get_logger().info(f'Loaded locations: {list(self._locations.keys())}')

        self._nav_client = ActionClient(self, NavigateToPose, g('nav_action').value)

        self.create_subscription(String, '/llm_nav/command', self._text_cmd_cb, 10)

        self._current_pose: tuple[float, float] | None = None
        self._goal_xy: tuple[float, float] | None = None
        self._nav_start_time: float | None = None
        self._recovery_count = 0
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        self._busy = False
        self._busy_lock = threading.Lock()

        self.get_logger().info(f'LLM nav ready.  ollama={self._ollama_model}')
        self.get_logger().info('Type a command in this terminal or publish to /llm_nav/command')

    # ── LLM parse — do not modify ─────────────────────────────────────────────

    def _parse_command(self, text: str) -> dict | None:
        location_list = ', '.join(self._locations.keys()) if self._locations else 'none'
        prompt = _SYSTEM.format(locations=location_list, command=text)
        try:
            raw = call_ollama(self._ollama_model, prompt, self._ollama_url)
        except (urllib.error.URLError, TimeoutError) as e:
            self.get_logger().error(f'ollama error: {e}')
            return None
        parsed = _extract_json(raw)
        if parsed is None:
            self.get_logger().error(f'LLM returned unparseable: {raw[:200]}')
        return parsed

    # ── TODO 4 — Goal resolution ──────────────────────────────────────────────

    def _resolve_goal(self, parsed: dict) -> tuple[float, float, float] | None:
        """
        Convert the LLM's parsed JSON into an (x, y, yaw_deg) tuple.

        The parsed dict will have one of these shapes:
          {"action": "stop"}
          {"action": "unknown", "reason": "..."}
          {"action": "go", "location": "<name>"}
          {"action": "go", "x": <float>, "y": <float>, "yaw": <float>}

        For "stop": cancel all goals via self._nav_client and return None.
        For "unknown" or unrecognised actions: log a warning and return None.
        For "go" with a location name: look it up in self._locations. Each
          entry is [x, y] or [x, y, yaw]. Return None if the name is missing.
        For "go" with raw coordinates: read x, y, and optional yaw directly.
        Return None for any malformed input.
        """
        action = parsed.get("action")

        # If coordinates are present but action is omitted, assume "go".
        if action is None and "x" in parsed and "y" in parsed:
            action = "go"

        if action == "stop":
            self._nav_client._cancel_goal_async()
            return None

        if action == "unknown":
            self.get_logger().warning(parsed.get("reason", "Unknown command"))
            return None

        if action != "go":
            self.get_logger().warning(f"Unknown action: {action}")
            return None

        if "location" in parsed:

            name = parsed["location"]

            # ---- TinyLlama sometimes returns coordinates as a string ----
            if isinstance(name, str):
                parts = name.strip().split()

                if len(parts) == 2:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        return (x, y, 0.0)
                    except ValueError:
                        pass

            # ---- TinyLlama sometimes returns coordinates as an object ----
            if isinstance(name, dict):
                if "x" in name and "y" in name:
                    x = float(name["x"])
                    y = float(name["y"])
                    yaw = float(name.get("yaw", 0.0))
                    return (x, y, yaw)

            if name not in self._locations:
                self.get_logger().warning(f"Unknown location: {name}")
                return None

            loc = self._locations[name]

            if len(loc) == 2:
                return (loc[0], loc[1], 0.0)

            if len(loc) >= 3:
                return (loc[0], loc[1], loc[2])

            return None

        if "x" in parsed and "y" in parsed:

            x = float(parsed["x"])
            y = float(parsed["y"])
            yaw = float(parsed.get("yaw", 0.0))

            return (x, y, yaw)

        self.get_logger().warning("No valid goal found")
        return None

    # ── TODO 5 — Goal dispatch ────────────────────────────────────────────────

    def _send_goal(self, x: float, y: float, yaw_deg: float):
        """
        Dispatch (x, y, yaw_deg) to Nav2 in a background thread so the ROS
        executor is never blocked.
        """
        threading.Thread(
            target=self._send_goal_thread,
            args=(x, y, yaw_deg),
            daemon=True
        ).start()

    def _send_goal_thread(self, x: float, y: float, yaw_deg: float):
        """
        Wait for the NavigateToPose action server (up to 60s), build a
        PoseStamped goal using self._frame_id and _yaw_to_quat(), and
        send it via self._nav_client.

        Store self._goal_xy, self._nav_start_time, and reset
        self._recovery_count before sending. Register
        self._goal_accepted_cb as the done callback and
        self._feedback_cb as the feedback callback.

        If the server never comes up, log an error, clear self._busy,
        and return.
        """
        if not self._nav_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("NavigateToPose action server not available.")
            self._busy = False
            return

        self._goal_xy = (x, y)
        self._nav_start_time = time.time()
        self._recovery_count = 0

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = self._frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        qz, qw = _yaw_to_quat(yaw_deg)

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )

        future.add_done_callback(self._goal_accepted_cb)

    # ── TODO 6 — Result handling ──────────────────────────────────────────────

    def _result_cb(self, future):
        """
        Called when Nav2 finishes (success or failure).

        Read future.result().status and compare against GoalStatus constants.
        Log whether the goal succeeded or failed, how long it took
        (self._nav_start_time), and how many recoveries Nav2 triggered
        (self._recovery_count).

        If successful and both self._goal_xy and self._current_pose are
        available, compute the Euclidean distance between them and log it
        as the accuracy of the navigation.

        Always clear self._busy at the end (acquire self._busy_lock).
        """
        result = future.result()
        status = result.status

        elapsed = time.time() - self._nav_start_time

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Goal succeeded in {elapsed:.2f} s "
                f"with {self._recovery_count} recoveries."
            )

            if self._goal_xy is not None and self._current_pose is not None:
                dx = self._goal_xy[0] - self._current_pose[0]
                dy = self._goal_xy[1] - self._current_pose[1]
                error = math.hypot(dx, dy)

                self.get_logger().info(
                    f"Navigation accuracy: {error:.3f} m"
                )

        else:
            self.get_logger().warning(
                f"Goal failed with status {status}. "
                f"Time: {elapsed:.2f} s, "
                f"Recoveries: {self._recovery_count}"
            )

        with self._busy_lock:
            self._busy = False

    # ── callbacks — do not modify ─────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self._current_pose = (p.x, p.y)

    def _feedback_cb(self, fb):
        dist = fb.feedback.distance_remaining
        self._recovery_count = fb.feedback.number_of_recoveries
        if dist > 0.0:
            self.get_logger().info(
                f'  distance remaining: {dist:.2f}m', throttle_duration_sec=3.0)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            with self._busy_lock:
                self._busy = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _text_cmd_cb(self, msg: String):
        self._process(msg.data.strip())

    def handle_typed(self, text: str):
        with self._busy_lock:
            busy = self._busy
        if busy:
            print('Still navigating — wait or type "stop".', flush=True)
            return
        self._process(text)

    def _process(self, text: str):
        self.get_logger().info(f'Command: "{text}"')
        print(f'   Asking {self._ollama_model}…', flush=True)
        parsed = self._parse_command(text)
        if parsed is None:
            return
        self.get_logger().info(f'LLM parsed: {parsed}')
        goal = self._resolve_goal(parsed)
        if goal:
            with self._busy_lock:
                self._busy = True
            self._send_goal(*goal)


# ── main — do not modify ──────────────────────────────────────────────────────

def _ui_loop(node: LLMNavigator):
    print('\n─────────────────────────────────────────', flush=True)
    print(' LLM Navigator  |  ctrl-C to quit', flush=True)
    print(' Type a command → send as text', flush=True)
    print('─────────────────────────────────────────\n', flush=True)
    while rclpy.ok():
        try:
            line = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line:
            node.handle_typed(line)


def main(args=None):
    rclpy.init(args=args)
    node = LLMNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        _ui_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
