"""Command-line application entry point."""

from __future__ import annotations

import argparse
import logging
import signal
from pathlib import Path

from .camera import CameraStream
from .config import load_config
from .controller import FruitSortingController
from .dashboard import DashboardServer, DashboardStore
from .detector import FruitDetector
from .exceptions import FruitSorterError
from .logging_setup import configure_logging
from .robot_client import RobotClient, SimulatedRobotClient


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="UNO Q YOLOv8 fruit sorter")
    parser.add_argument(
        "--config",
        default="config/robot.yaml",
        help="Path to robot YAML configuration",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run vision, calibration, IK, and planning without real servo commands",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Disable the local OpenCV preview window",
    )
    parser.add_argument(
        "--no-web",
        action="store_true",
        help="Disable the browser dashboard",
    )
    return parser


def cli() -> None:
    args = build_parser().parse_args()
    config = load_config(Path(args.config))
    if args.no_display:
        config.application.display = False
    if args.no_web:
        config.web.enabled = False

    configure_logging(config.logging)
    log = logging.getLogger("fruit_sorter")

    camera = CameraStream(config.camera)
    detector = FruitDetector(config.detector)
    robot = SimulatedRobotClient(config) if args.dry_run else RobotClient(config)

    dashboard_store = DashboardStore(config, dry_run=args.dry_run)
    controller = FruitSortingController(
        config,
        camera,
        detector,
        robot,
        dashboard=dashboard_store,
    )
    dashboard_server = DashboardServer(
        config.web,
        dashboard_store,
        emergency_stop_callback=controller.emergency_stop,
    )

    def handle_signal(signum: int, _frame) -> None:
        log.warning("Received signal %d; requesting controlled shutdown", signum)
        controller.request_stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    dashboard_server.start()

    try:
        controller.run()
    except FruitSorterError as exc:
        log.critical("Fruit sorter stopped: %s", exc)
        dashboard_store.set_error(str(exc))
        raise SystemExit(1) from exc
    except Exception as exc:
        log.exception("Unexpected fatal error")
        dashboard_store.set_error(str(exc))
        controller.emergency_stop()
        raise SystemExit(2)
    finally:
        dashboard_server.stop()


if __name__ == "__main__":
    cli()
