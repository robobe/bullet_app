"""Bullet application console entry point."""
import sys

import click
from loguru import logger
import zmq

from .application import Application
from .cli import CliParser


def configure_logging():
    """Configure process diagnostics once at the application boundary."""
    logger.remove()
    logger.add(sys.stdout, format="{message}")


def main(argv=None):
    configure_logging()
    try:
        options = CliParser().parse(argv)
        return options if isinstance(options, int) else Application(options).run()
    except click.ClickException as error:
        error.show()
        return error.exit_code
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, zmq.ZMQError) as error:
        logger.error("Bullet app failed: {}", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
