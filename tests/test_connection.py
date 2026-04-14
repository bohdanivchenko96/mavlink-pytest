def test_heartbeat(vehicle):
    """Verify ArduCopter is alive by receiving a heartbeat."""
    heartbeat = vehicle.receive_heartbeat()
    assert heartbeat.get_type() == "HEARTBEAT"


def test_copter_type(vehicle):
    """Verify the vehicle reports itself as a quadrotor."""
    assert vehicle.vehicle_type() == "MAV_TYPE_QUADROTOR", (
        f"Expected MAV_TYPE_QUADROTOR, got {vehicle.vehicle_type()}"
    )


def test_connection_stable_for_5_seconds(vehicle):
    """Verify the MAVLink connection remains stable over a 5-second window."""
    stats = vehicle.connection_stats(duration=5.0)

    assert stats.missed == 0, (
        f"Connection dropped: {stats.missed} heartbeat(s) missed, {stats.received} received"
    )
    assert stats.received > 0, "No heartbeats received during 5-second stability window"


def test_pre_arm_check(vehicle):
    """Verify the vehicle is in a safe state before arming."""
    assert vehicle.status.battery_remaining() > 20, "Battery too low to arm"
    assert vehicle.gps.satellites_visible() >= 3, "Less than 3 satellites visible"
    assert vehicle.attitude.is_level(), "Pitch or roll is too high"


def test_arm_vehicle(guided_vehicle):
    """Verify the vehicle can be armed."""
    guided_vehicle.flight.arm()
    assert guided_vehicle.status.is_armed(), "Vehicle did not arm"


def test_disarm_vehicle(armed_vehicle):
    """Verify the vehicle can be disarmed."""
    armed_vehicle.flight.disarm(force=True)
    assert not armed_vehicle.status.is_armed(), "Vehicle did not disarm"
