def test_take_height(armed_vehicle):
    """Verify the vehicle can take off to 20m."""
    armed_vehicle.flight.takeoff(altitude_m=20)

    altitude = armed_vehicle.gps.altitude()
    assert 19 <= altitude <= 21, f"Expected 20m altitude ±1m, got {altitude:.1f}m"

    armed_vehicle.flight.land()
