"""The dynamometer.

What it absorbs, what it weighs, and where it couples to the vehicle. Nothing
about the vehicle lives here -- see vehicle.py for that.

The dyno applies road resistance ``A + B*v + C*v^2`` and nothing else, so speed
is an output and whoever works the pedal is the only authority over it. There is
deliberately no speed-controlled mode: a dyno servo holding a speed fights a
driver tracking the same speed, and the pedal then parks on whatever the driver's
integrator happened to hold rather than on what the physics require. Measured,
reaching 15 m/s three different ways parked the throttle at 0.040, 0.206 and
0.696; under road resistance it is 0.0283 every time.
"""

__all__ = ['Dyno']


class Dyno(object):
    """``mode='chassis'`` puts the vehicle on rollers, so its rotating parts
    refer to the road through ``r^2``. ``mode='axle'`` bolts hub units to the
    hubs -- nothing moves, so the body's inertia has to be added electrically.
    """

    def __init__(self, mode='chassis', road_A_N=111.0, road_B_Npms=0.99,
                 road_C_Npms2=0.45, roller_inertia_kgm2=40.0,
                 hub_inertia_kgm2=0.9, grade_rad=0.0):
        if mode not in ('chassis', 'axle'):
            raise ValueError("mode must be 'chassis' or 'axle', got %r" % (mode,))
        self.mode = mode
        self.road_A_N = road_A_N
        self.road_B_Npms = road_B_Npms
        self.road_C_Npms2 = road_C_Npms2
        #: A real dyno's rollers are heavy and this term is not small.
        self.roller_inertia_kgm2 = roller_inertia_kgm2
        self.hub_inertia_kgm2 = hub_inertia_kgm2
        self.grade_rad = grade_rad

    def resistance(self, speed):
        """Road resistance, signed so it always opposes motion."""
        sign = 1.0 if speed >= 0.0 else -1.0
        return sign * (self.road_A_N + self.road_B_Npms * abs(speed)
                       + self.road_C_Npms2 * speed * speed)
