"""
The few cflib calls this framework needs, for one Crazyflie.

The Crazyflie runs its Kalman filter with position-only MoCap updates
(send_extpos) and flies with the high-level commander. Notes from our setup
(firmware 2026.04, cflib 0.1.32):

* Parameters written right after connecting were silently dropped, so
  connect() waits a few seconds before configuring the Crazyflie.
* With send_extpose (position + orientation) the pre-flight check found the
  estimate meters away from MoCap, so only the position is streamed. MoCap
  then does not correct yaw: the Crazyflie must start facing world +x (the
  UGV parks at a fixed heading for that).
* A log block holds at most 26 bytes (6 floats).

cflib is imported lazily so that check_estimate() can be tested without it.
"""

import math
import os
import threading
import time
from typing import Callable, Optional, Tuple

Point = Tuple[float, float, float]


class Crazyflie:
    """One radio link to one Crazyflie."""

    def __init__(self, uri: str, cache_dir: Optional[str] = None,
                 log: Callable[[str], None] = print):
        self.uri = uri
        self._cache_dir = os.path.expanduser(cache_dir) if cache_dir else None
        self._log = log
        self._scf = None
        self._cf = None
        self._lock = threading.Lock()
        self._estimate = None  # (x, y, z) of the onboard Kalman filter
        self._battery = None   # battery voltage [V]

    def connect(self, startup_delay: float = 3.0, position_std_dev: float = 0.01) -> None:
        """Open the link, select the Kalman filter and start logging the state."""
        import cflib.crtp
        from cflib.crazyflie import Crazyflie as CflibCrazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        cflib.crtp.init_drivers()
        if self._cache_dir:
            os.makedirs(self._cache_dir, exist_ok=True)
        self._log(f'connecting to {self.uri}')
        self._scf = SyncCrazyflie(self.uri, cf=CflibCrazyflie(rw_cache=self._cache_dir))
        try:
            self._scf.open_link()
        except Exception as error:  # noqa: B902 - cflib raises bare Exceptions
            self._scf = None
            raise RuntimeError(
                f'cannot connect to {self.uri}: {error}. Is the Crazyradio plugged in, the '
                'Crazyflie switched on, and the channel/address in the URI right?') from None
        self._cf = self._scf.cf

        time.sleep(startup_delay)  # parameter writes sent earlier are lost (firmware 2026.04)
        self._cf.param.set_value('stabilizer.estimator', '2')  # 2 = Kalman filter
        self._cf.param.set_value('locSrv.extPosStdDev', str(position_std_dev))
        time.sleep(0.5)
        estimator = self._cf.param.get_value('stabilizer.estimator')
        if str(estimator) != '2':
            raise RuntimeError(
                f'{self.uri}: could not select the Kalman filter (stabilizer.estimator='
                f'{estimator}); power-cycle the Crazyflie and try again')
        self._start_logging()
        self._log(f'connected to {self.uri} (Kalman filter, MoCap std dev {position_std_dev} m)')

    def _start_logging(self) -> None:
        from cflib.crazyflie.log import LogConfig

        config = LogConfig(name='uav_ugv_nav', period_in_ms=100)
        for variable in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'pm.vbat'):
            config.add_variable(variable, 'float')
        config.data_received_cb.add_callback(self._on_log)
        self._cf.log.add_config(config)
        config.start()

    def _on_log(self, timestamp, data, config) -> None:
        with self._lock:
            self._estimate = (
                data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])
            self._battery = data['pm.vbat']

    # -- state ---------------------------------------------------------------------

    def estimate(self) -> Optional[Point]:
        """Return the latest position estimate of the onboard Kalman filter."""
        with self._lock:
            return self._estimate

    def battery_voltage(self) -> Optional[float]:
        """Return the latest battery voltage, or None before the first log packet."""
        with self._lock:
            return self._battery

    def send_position(self, x: float, y: float, z: float) -> None:
        """Feed one MoCap position to the Kalman filter."""
        self._cf.extpos.send_extpos(x, y, z)

    def reset_estimator(self) -> None:
        """Restart the Kalman filter from the current MoCap position."""
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')

    # -- flight --------------------------------------------------------------------

    def arm(self, armed: bool = True) -> None:
        """Arm (or disarm) the motors; needed by recent firmware before takeoff."""
        supervisor = getattr(self._cf, 'supervisor', None)  # cflib >= 0.1.28
        if supervisor is not None:
            supervisor.send_arming_request(armed)
        else:
            self._cf.platform.send_arming_request(armed)

    def takeoff(self, z: float, duration: float) -> None:
        """Climb to the absolute height z [m]."""
        self._cf.high_level_commander.takeoff(z, duration)

    def go_to(self, x: float, y: float, z: float, duration: float) -> None:
        """Fly to an absolute position, facing +x."""
        self._cf.high_level_commander.go_to(x, y, z, 0.0, duration)

    def land(self, z: float, duration: float) -> None:
        """Descend to the absolute height z [m] (the deck or the floor)."""
        self._cf.high_level_commander.land(z, duration)

    def stop(self) -> None:
        """Stop the motors."""
        self._cf.high_level_commander.stop()

    def close(self) -> None:
        """Close the radio link."""
        if self._scf is not None:
            self._scf.close_link()
            self._scf = None


def check_estimate(estimate: Callable[[], Optional[Point]],
                   mocap: Callable[[], Optional[Point]],
                   sleep: Callable[[float], None] = time.sleep,
                   max_error: float = 0.05, max_jump: float = 0.3,
                   duration: float = 3.0, period: float = 0.1) -> Tuple[bool, str]:
    """
    Compare the Crazyflie's Kalman estimate with MoCap before takeoff.

    Samples both every period seconds for duration seconds. Fails if the
    estimate is more than max_error from MoCap, jumps by more than max_jump
    between samples, or sits at the origin (filter just reset, or no MoCap
    positions arriving). Returns (passed, explanation).
    """
    previous, worst = None, 0.0
    for _ in range(max(1, round(duration / period))):
        sleep(period)
        est, ref = estimate(), mocap()
        if est is None:
            return False, 'no state estimate from the Crazyflie yet'
        if ref is None:
            return False, 'no MoCap pose'
        if all(abs(v) < 1e-3 for v in est):
            return False, 'estimate is stuck at the origin (are MoCap positions arriving?)'
        error = math.dist(est, ref)
        worst = max(worst, error)
        if error > max_error:
            return False, (f'estimate {_fmt(est)} is {error:.3f} m from MoCap {_fmt(ref)} '
                           f'(limit {max_error} m)')
        if previous is not None and math.dist(est, previous) > max_jump:
            return False, f'estimate jumped from {_fmt(previous)} to {_fmt(est)}'
        previous = est
    return True, f'estimate within {worst:.3f} m of MoCap for {duration:g} s'


def _fmt(point) -> str:
    return '(' + ', '.join(f'{v:.2f}' for v in point) + ')'
