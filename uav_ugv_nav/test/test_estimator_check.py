from uav_ugv_nav.uav.crazyflie import check_estimate

MOCAP = (1.0, 2.0, 0.4)


def run(estimates, mocap=MOCAP, **limits):
    samples = iter(estimates)
    return check_estimate(lambda: next(samples), lambda: mocap, sleep=lambda seconds: None,
                          **limits)


def test_passes_when_the_estimate_matches_mocap():
    passed, detail = run([(1.01, 2.0, 0.4)] * 30)
    assert passed, detail


def test_fails_when_the_estimate_is_far_from_mocap():
    passed, detail = run([(1.2, 2.0, 0.4)] * 30)
    assert not passed and 'from MoCap' in detail


def test_fails_when_the_estimate_is_stuck_at_the_origin():
    passed, detail = run([(0.0, 0.0, 0.0)] * 30)
    assert not passed and 'origin' in detail


def test_fails_when_the_estimate_jumps():
    passed, detail = run([(1.0, 2.0, 0.4), (1.4, 2.0, 0.4)] + [MOCAP] * 30, max_error=1.0)
    assert not passed and 'jumped' in detail


def test_fails_without_data():
    assert run([None] * 30)[0] is False
    assert run([MOCAP] * 30, mocap=None)[0] is False
