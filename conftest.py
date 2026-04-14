import os

import pytest
from pymavlink import mavutil

from vehicle import Vehicle

_MAVLINK_CONN = os.environ.get("MAVLINK_CONN", "tcp:localhost:5760")


@pytest.fixture(scope="session")
def vehicle() -> Vehicle:
    connection = mavutil.mavlink_connection(_MAVLINK_CONN)
    connection.wait_heartbeat(timeout=30)
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )
    yield Vehicle(connection)
    connection.close()


@pytest.fixture
def guided_vehicle(vehicle: Vehicle) -> Vehicle:
    vehicle.flight.wait_for_ready_to_arm()
    vehicle.flight.set_mode("GUIDED")
    yield vehicle


@pytest.fixture
def armed_vehicle(guided_vehicle: Vehicle) -> Vehicle:
    guided_vehicle.flight.arm()
    yield guided_vehicle
    if guided_vehicle.status.is_armed():
        guided_vehicle.flight.disarm(force=True)
