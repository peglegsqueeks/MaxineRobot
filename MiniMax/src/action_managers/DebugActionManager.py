from .ActionManager import ActionManager
from typing import Any


class DebugActionManager(ActionManager[Any]):
    """
    Debug action manager that logs actions instead of performing them.
    Used for testing and debugging purposes when hardware is not available.
    Enhanced compatibility for all robot systems.
    """

    def __init__(self, name: str) -> None:
        """
        Initialize the debug action manager
        
        Args:
            name: Name of the manager for identification
        """
        super().__init__(name)
        self.debug_mode = True
        self.action_count = 0

    def perform_action(self, config: Any):
        """
        Debug implementation - silently handles action requests without hardware interaction
        
        Args:
            config: Action configuration (ignored in debug mode)
        """
        # In debug mode, actions are silently handled without hardware interaction
        # This prevents errors when hardware is not available during testing
        self.action_count += 1
        pass

    def shutdown(self):
        """Debug implementation of shutdown - no hardware to shutdown"""
        self.debug_mode = False
        self.action_count = 0

    def get_status(self):
        """Get debug manager status"""
        return {
            'name': self.name,
            'debug_mode': self.debug_mode,
            'actions_performed': self.action_count,
            'type': 'debug'
        }

    def reset(self):
        """Reset debug manager state"""
        self.action_count = 0

    def is_debug_mode(self):
        """Check if in debug mode"""
        return self.debug_mode

    def __str__(self) -> str:
        return f"DebugActionManager({self.name})"

    def __repr__(self) -> str:
        return self.__str__()