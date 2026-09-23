"""Console and rotating-file logging setup."""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

from .config import LoggingConfig


def configure_logging(config: LoggingConfig) -> logging.Logger:
    level = getattr(logging, config.level.upper(), logging.INFO)
    log_path = Path(config.file)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    root = logging.getLogger()
    root.setLevel(level)
    root.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
    )

    stream = logging.StreamHandler()
    stream.setLevel(level)
    stream.setFormatter(formatter)
    root.addHandler(stream)

    rotating = RotatingFileHandler(
        log_path,
        maxBytes=config.max_bytes,
        backupCount=config.backup_count,
        encoding="utf-8",
    )
    rotating.setLevel(level)
    rotating.setFormatter(formatter)
    root.addHandler(rotating)

    return logging.getLogger("fruit_sorter")
