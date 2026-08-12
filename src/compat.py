"""Compatibility helpers for the unpublished Rigel 0.3 plugin API.

Rigel 0.3 was never published and its upstream repository is no longer public.
Keeping this small adapter lets existing Rigel installations load the plugin while
also allowing the orchestration package to be used on its own.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Mapping

try:  # pragma: no cover - exercised only when the unpublished Rigel 0.3 is installed
    from rigel.loggers import get_logger
    from rigel.models.builder import ModelBuilder as _ModelBuilder
    from rigel.plugins import Plugin as _PluginBase
except ImportError:

    def get_logger() -> logging.Logger:
        """Return the package logger when Rigel is not installed."""
        return logging.getLogger("rigel_orchestration")

    class _ModelBuilder:  # type: ignore[no-redef]
        """Subset of Rigel's model builder used by this plugin."""

        def __init__(self, model_type: type[Any]) -> None:
            self.model_type = model_type

        def build(self, args: list[Any], kwargs: Mapping[str, Any]) -> Any:
            return self.model_type(*args, **dict(kwargs))

    class _PluginBase:  # type: ignore[no-redef]
        """Minimal implementation of the Rigel 0.3 plugin lifecycle contract."""

        def __init__(
            self,
            raw_data: Mapping[str, Any],
            global_data: Mapping[str, Any] | None,
            application: Any,
            providers_data: Mapping[str, Any] | None,
            shared_data: Mapping[str, Any] | None = None,
        ) -> None:
            self.raw_data = dict(raw_data)
            self.global_data = dict(global_data or {})
            self.application = application
            self.providers_data = dict(providers_data or {})
            self.shared_data = dict(shared_data or {})


ModelBuilder = _ModelBuilder
PluginBase = _PluginBase
