def test_take_height(vehicle):
    """Verify the vehicle can take off to 10m and land autonomously."""
    vehicle.flight.wait_for_ready_to_arm()
    vehicle.flight.set_mode("GUIDED")
    vehicle.flight.arm()
    vehicle.flight.takeoff(altitude_m=20)

    assert vehicle.gps.altitude() >= 19 and vehicle.gps.altitude() <= 21, (
        f"Expected 20m altitude +/- 1m , got {vehicle.gps.altitude():.1f}m"
    )
