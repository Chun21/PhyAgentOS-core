import math

import pytest

from PhyAgentOS.skill_runtime.g1d_trajectory import JointPath, plan_stop, quintic_duration


def test_continuous_path_bounds_and_stop_at_each_segment():
    points = ((0.0,), (.35,), (-.35,), (.35,), (0.0,))
    durations = tuple(quintic_duration(a, b, minimum_duration_s=1,
        max_velocity=.5, max_acceleration=2, max_jerk=10)
        for a, b in zip(points[:-1], points[1:]))
    path = JointPath(points, durations)
    boundary = 0.0
    for index, duration in enumerate(durations):
        for fraction in (.1, .3, .5, .8):
            t = boundary + fraction * duration
            q, dq, ddq = path.sample(t)
            assert -.35 <= q[0] <= .35
            assert 0 < abs(dq[0]) <= .5 + 1e-9
            assert abs(ddq[0]) <= 2 + 1e-9
            dt = 1e-5
            assert (path.sample(t + dt)[0][0] - path.sample(t - dt)[0][0]) / (2*dt) == pytest.approx(dq[0], abs=1e-7)
            stop = plan_stop(q, dq, ddq, limits=[(-.5, .5)], velocity=.5,
                             acceleration=2, jerk=10, budget_s=2)
            assert stop.sample(0)[0] == pytest.approx(q)
            assert stop.sample(0)[1] == pytest.approx(dq)
            assert stop.sample(stop.duration_s)[1] == (0.0,)
        boundary += duration
        at_knot = path.sample(boundary)
        assert at_knot[0] == pytest.approx(points[index + 1])
        assert abs(at_knot[1][0]) < 1e-9
        assert abs(at_knot[2][0]) < 1e-9
    assert math.isclose(boundary, path.duration_s)
    assert path.sample(path.duration_s + 1) == ((0.0,), (0.0,), (0.0,))
