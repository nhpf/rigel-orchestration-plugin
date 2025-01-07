from __future__ import annotations

from typing import TYPE_CHECKING, Any

from rigel.loggers import get_logger
from rigel.plugins import Plugin as PluginBase

if TYPE_CHECKING:
    from rigel.models.application import Application
    from rigel.models.plugin import PluginRawData
    from rigel.models.rigelfile import RigelfileGlobalData

LOGGER = get_logger()


class Plugin(PluginBase):
    """A plugin is a class that encapsulates a specific functionality.

    It inherits from the `PluginBase` class and provides a constructor that takes the required
    arguments for a plugin.

    Attributes:
        raw_data (PluginRawData): Dedicated raw data for the plugin (extracted from Rigelfile)
        global_data (RigelfileGlobalData): The entire Rigelfile raw data
        application (Application): Raw data about the overall application (e.g., ROS distro - extracted from Rigelfile)
        providers_data (Dict[str, Any]): Raw data concerning providers
        shared_data (Optional[Dict[str, Any]]): Data shared between plugins in a pipeline

    """

    def __init__(
        self,
        raw_data: PluginRawData,  # Dict declared in "with" section of Rigelfile related to the plugin
        global_data: RigelfileGlobalData,  # Dict of whole Rigelfile
        application: Application,  # Dict with application section of Rigelfile
        providers_data: dict[str, Any],  # Dict with credentials for providers
        shared_data: dict[str, Any] | None = None,  # Dict of shared data between plugins
    ) -> None:
        """Create a new plugin instance."""
        self.data = raw_data

        if shared_data is None:
            shared_data = {}

        super().__init__(raw_data, global_data, application, providers_data, shared_data)

    # NOTE: for more information on execution flows refer to section
    # "Executors, job sequences and execution flows" (from a previous thesis using Rigel):
    # https://github.com/Kazadhum/file_intro_plugin/issues/4

    def setup(self) -> None:
        """Use this function to allocate plugin resoures."""
        # Parse args, setup resources, etc.
        LOGGER.info("[SETUP]")

    def start(self) -> None:
        """Use this function to start executing business logic of your plugin."""
        # Launch kubernetes jobs, start processing, etc.
        LOGGER.info("[START]")

    def process(self) -> None:
        """Use this function to perform any evaluation of your plugin execution."""
        # Evaluate results, observability, check for errors, etc.
        LOGGER.info("[PROCESS]")

    def stop(self) -> None:
        """Use this function to gracefully clean plugin resources."""
        LOGGER.info("[STOP]")
