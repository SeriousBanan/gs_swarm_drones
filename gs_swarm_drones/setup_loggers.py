"""
This module create and configure loggers.

`logger` - base logger which logging everything that level upper than `logging.DEBUG` to
file `logs/logs.log` and everything that level upper than `logging.INFO` to concole.

`error_logger` - logger which logging everything that level upper than `logging.ERROR` to
file `logs/errors.log`
"""

import logging
import logging.config
import logging.handlers
import os


__all__ = ["logger", "error_logger"]


if not os.path.exists("logs"):
    os.mkdir("logs")

del os


_logging_config = {
    "version": 1,
    "formatters": {
        "formatter": {
            "class": "logging.Formatter",
            "format": "{asctime} | {levelname:8} | {message}",
            "style": "{"
        },
        "extended_formatter": {
            "class": "logging.Formatter",
            "format": "{asctime} | {levelname:8} | {module}:{lineno} | {message}",
            "style": "{"
        }
    },
    "handlers": {
        "console": {
            "class": "logging.StreamHandler",
            "level": "INFO",
            "formatter": "formatter"
        },
        "error": {
            "class": "logging.handlers.RotatingFileHandler",
            "level": "ERROR",
            "formatter": "extended_formatter",
            "filename": "logs/errors.log",
            "maxBytes": 1048576,
            "backupCount": 10
        },
        "file": {
            "class": "logging.handlers.RotatingFileHandler",
            "level": "DEBUG",
            "formatter": "extended_formatter",
            "filename": "logs/logs.log",
            "maxBytes": 1048576,
            "backupCount": 10
        }
    },
    "loggers": {
        "logger": {
            "level": "DEBUG",
            "handlers": ["console", "file"]
        },
        "error_logger": {
            "level": "ERROR",
            "handlers": ["error"]
        }
    }
}

logging.config.dictConfig(_logging_config)


logger = logging.getLogger("logger")
error_logger = logging.getLogger("error_logger")
