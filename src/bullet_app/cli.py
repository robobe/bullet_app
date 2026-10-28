"""Click parsing for the Bullet application command line."""
from dataclasses import dataclass
from pathlib import Path

import click


DEFAULT_BBOX_ENDPOINT = "tcp://127.0.0.1:5558"
DEFAULT_IMAGE_TOPIC = "camera/front/image"


@dataclass(frozen=True)
class AppOptions:
    transport_config: Path
    topic: str
    debug: bool
    bbox_endpoint: str
    mission_config: Path | None
    hover_check: bool


@click.command()
@click.option("--transport-config", type=click.Path(path_type=Path), required=True)
@click.option("--topic", default=DEFAULT_IMAGE_TOPIC, show_default=True)
@click.option("--debug", is_flag=True, help="Show camera frames and target bounding box")
@click.option("--bbox-endpoint", default=DEFAULT_BBOX_ENDPOINT, show_default=True,
              help="Loopback TCP endpoint to bind for bbox results")
@click.option("--mission-config", type=click.Path(path_type=Path),
              help="Enable app-owned takeoff/hover/LOS using this navigator profile")
@click.option("--hover-check", is_flag=True, help="Stop after a stable 20-second hover")
def command(transport_config, topic, debug, bbox_endpoint, mission_config, hover_check):
    """Receive Bullet camera images, detect red targets, and publish bounding boxes."""
    if hover_check and mission_config is None:
        raise click.UsageError("--hover-check requires --mission-config")
    return AppOptions(transport_config, topic, debug, bbox_endpoint, mission_config, hover_check)


class CliParser:
    """Parse command-line arguments without starting application services."""

    def parse(self, argv=None) -> AppOptions | int:
        return command.main(args=argv, standalone_mode=False)
