from fastapi import Request

from src.application.protocols import DriveControllerProtocol


def get_drive_controller(request: Request) -> DriveControllerProtocol:
    """Получение контроллера для управления ездой машинки."""
    if request.app.state.drive_controller is None:
        raise NotImplementedError("The controller for driving the car is not initialized!")
    return request.app.state.drive_controller
