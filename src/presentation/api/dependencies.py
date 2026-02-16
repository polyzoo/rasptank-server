from fastapi import Request

from src.application.protocols import HealthCheckProtocol, DriveControllerProtocol


def get_health_controller(request: Request) -> HealthCheckProtocol:
    """Получение контроллера для проверки работоспособности приложения."""
    if request.app.state.health_controller is None:
        raise NotImplementedError("The controller for healthcheck the app is not initialized!")
    return request.app.state.health_controller


def get_drive_controller(request: Request) -> DriveControllerProtocol:
    """Получение контроллера для управления ездой машинки."""
    if request.app.state.drive_controller is None:
        raise NotImplementedError("The controller for driving the car is not initialized!")
    return request.app.state.drive_controller
