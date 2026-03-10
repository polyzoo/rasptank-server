from fastapi import Request

from src.application.protocols import DriveControllerProtocol


def get_drive_controller(request: Request) -> DriveControllerProtocol:
    """Получает контроллер для управления движением RaspTank."""
    if request.app.state.drive_controller is None:
        raise NotImplementedError("The controller for driving the RaspTank is not initialized!")
    return request.app.state.drive_controller
