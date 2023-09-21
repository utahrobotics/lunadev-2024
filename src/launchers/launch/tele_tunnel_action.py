from typing import List, Optional
from launch import Action, Condition, LaunchContext, LaunchDescriptionEntity
import sys

sys.path.insert(1, '/root/lunadev-2024/lunadev/')

import start_telemetry_tunnel


class StartTelemetryTunnel(Action):
    def __init__(self, *, condition: Optional[Condition] = None) -> None:
        """
        Starts a telemetry tunnel over SSH
        """
        super().__init__(condition=condition)

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """
        Execute the action.

        Should be overridden by derived class, but by default does nothing.
        """
        start_telemetry_tunnel.main()
