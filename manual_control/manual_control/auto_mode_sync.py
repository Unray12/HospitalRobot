class AutoModeSync:
    def __init__(self, max_attempts=30):
        self.max_attempts = int(max(1, max_attempts))
        self.pending_mode = None
        self.attempts = 0

    def queue(self, enabled):
        self.pending_mode = bool(enabled)
        self.attempts = 0

    def step(self, service_ready):
        if self.pending_mode is None:
            return "idle", None

        if not service_ready:
            self.attempts += 1
            if self.attempts >= self.max_attempts:
                dropped_mode = self.pending_mode
                self.pending_mode = None
                self.attempts = 0
                return "give_up", dropped_mode
            return "wait", self.pending_mode

        mode = self.pending_mode
        self.pending_mode = None
        self.attempts = 0
        return "send", mode
