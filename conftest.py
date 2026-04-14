import pytest
from pymavlink import mavutil

from vehicle import Vehicle


@pytest.fixture(scope="session")
def vehicle() -> Vehicle:
    connection = mavutil.mavlink_connection("tcp:localhost:5760")
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
