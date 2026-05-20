"""Small retry state machine for syncing auto-mode requests to a ROS service."""


class AutoModeSync:
    """Track one pending auto-mode change until the target service accepts it.

    The ROS service `/set_auto_mode` may not be ready when manual_control
    starts (the line_follower node may still be initializing). Rather than
    blocking startup, we queue the desired state and retry until the service
    becomes available — or give up after max_attempts (so we don't retry
    forever if line_follower is dead).
    """

    def __init__(self, max_attempts=30):
        self.max_attempts = int(max(1, max_attempts))
        # `pending_mode` is the latest desired state (last writer wins). If
        # the operator flips auto on, then off, before the first request goes
        # through, only the "off" state needs to be sent.
        self.pending_mode = None
        self.attempts = 0

    def queue(self, enabled):
        # Reset attempts so a fresh request always gets the full retry budget,
        # even if a previous queued request was about to give up.
        self.pending_mode = bool(enabled)
        self.attempts = 0

    def step(self, service_ready):
        if self.pending_mode is None:
            return "idle", None

        if not service_ready:
            self.attempts += 1
            if self.attempts >= self.max_attempts:
                # Cap reached. Drop the pending request and log at the caller.
                # 30 attempts × 200ms tick ≈ 6s — long enough for a normal
                # node startup, short enough that operator notices something
                # is wrong with line_follower.
                dropped_mode = self.pending_mode
                self.pending_mode = None
                self.attempts = 0
                return "give_up", dropped_mode
            return "wait", self.pending_mode

        # Service ready — clear pending and let caller send the request.
        # If the request fails async, caller re-queues via queue().
        mode = self.pending_mode
        self.pending_mode = None
        self.attempts = 0
        return "send", mode
