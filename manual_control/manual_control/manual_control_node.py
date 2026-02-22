import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool

from .auto_mode_sync import AutoModeSync
from robot_common.command_protocol import format_command
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter

class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control")
        self.log = LogAdapter(self.get_logger(), "manual_control")

        config = ConfigManager("manual_control", logger=self.log).load()
        topics_cfg = config.get("topics", {})
        service_cfg = config.get("service", {})

        self.base_speed = int(config.get("base_speed", 6))
        self.auto_sync_retry_period = float(config.get("auto_mode_service_retry_period", 0.2))
        self.auto_sync_max_attempts = int(config.get("auto_mode_service_max_attempts", 30))
        self.autoMode = False
        self._auto_sync = AutoModeSync(max_attempts=self.auto_sync_max_attempts)

        cmd_topic = topics_cfg.get("cmd_vel", "/cmd_vel")
        auto_topic = topics_cfg.get("auto_mode", "/auto_mode")
        vr_topic = topics_cfg.get("vr_control", "/VR_control")
        pick_topic = topics_cfg.get("pick_robot", "/pick_robot")
        auto_service = service_cfg.get("set_auto_mode", "/set_auto_mode")

        self.cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self.auto_pub = self.create_publisher(Bool, auto_topic, 10)
        self.auto_client = self.create_client(SetBool, auto_service)

        self.create_subscription(String, vr_topic, self._manual_cb, 10)
        self.create_subscription(String, pick_topic, self._pick_cb, 10)
        self._sync_timer = self.create_timer(self.auto_sync_retry_period, self._sync_auto_mode_timer_cb)

    def _manual_cb(self, msg: String):
        if self.autoMode:
            return
        cmd = msg.data.strip()
        msg = self._command_to_msg(cmd, self.base_speed)
        if msg:
            self.cmd_pub.publish(msg)

    def _pick_cb(self, msg: String):
        data = msg.data.strip()
        self.autoMode = data == "1"
        self.auto_pub.publish(Bool(data=self.autoMode))
        self._auto_sync.queue(self.autoMode)

    def _sync_auto_mode_timer_cb(self):
        action, enabled = self._auto_sync.step(self.auto_client.service_is_ready())
        if action == "idle":
            return
        if action == "wait":
            return
        if action == "give_up":
            self.log.warning(f"set_auto_mode sync dropped after retries: {enabled}", event="AUTO_SYNC")
            return
        req = SetBool.Request()
        req.data = enabled
        future = self.auto_client.call_async(req)
        future.add_done_callback(lambda fut, target=enabled: self._on_auto_mode_sync_result(fut, target))

    def _on_auto_mode_sync_result(self, future, target):
        try:
            response = future.result()
        except Exception as exc:
            self.log.warning(f"set_auto_mode request failed: {exc}", event="AUTO_SYNC")
            self._auto_sync.queue(target)
            return

        if not response.success:
            self.log.warning(f"set_auto_mode rejected: {response.message}", event="AUTO_SYNC")
            self._auto_sync.queue(target)

    def _command_to_msg(self, command, speed):
        payload = format_command(command.strip(), speed)
        if payload is None:
            return None
        return String(data=payload)



def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
