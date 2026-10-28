"""Application mode selection and transport configuration."""
from bullet_msgs import load_transport

from .runtime import DetectorRuntime, MissionRunner


class Application:
    """Run one configured detector-only or mission-enabled application."""

    def __init__(self, options):
        self.options = options

    def run(self) -> int:
        transport = load_transport(self.options.transport_config)
        if self.options.bbox_endpoint in transport.values():
            raise ValueError("bbox endpoint must differ from transport endpoints")
        if self.options.mission_config is not None:
            return MissionRunner(self.options, transport).run()
        return DetectorRuntime(self.options, transport).run()
