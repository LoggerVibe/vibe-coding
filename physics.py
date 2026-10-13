"""EV longitudinal dynamics and distance-domain quadrature. SI units throughout."""
from dataclasses import dataclass, asdict
import math

G = 9.81
RHO = 1.225
SPEEDS = tuple(float(v) for v in range(0, 21, 2))


@dataclass(frozen=True)
class Vehicle:
    mass: float = 1700
    cd: float = .28
    area: float = 2.3
    crr: float = .010
    efficiency: float = .90
    regen: float = .65
    auxiliary: float = 700
    wind: float = 0  # positive is headwind, m/s
    max_power: float = 85000  # wheel power, W
    regen_power: float = 45000  # mechanical capture limit, W
    acceleration: float = 2.0
    braking: float = 3.0

    @classmethod
    def parse(cls, data=None):
        data = {} if data is None else data
        if not isinstance(data, dict):
            raise ValueError('차량 설정은 JSON 객체여야 합니다.')
        bounds = {'mass': (900, 3000), 'cd': (.15, .6), 'area': (1.5, 4),
                  'crr': (.005, .03), 'efficiency': (.65, .98), 'regen': (0, .9),
                  'auxiliary': (0, 3000), 'wind': (-10, 15)}
        values = {}
        for key, (lo, hi) in bounds.items():
            value = float(data.get(key, getattr(cls(), key)))
            if not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f'{key}: {lo}~{hi} 범위의 값을 입력하세요.')
            values[key] = value
        return cls(**values)

    def json(self):
        return asdict(self)


def forces(v, a, grade, car):
    """grade = dh/ds = sin(theta), where s is road arc length."""
    relative = v + car.wind
    return {'inertia': car.mass * a,
            'grade': car.mass * G * grade,
            'rolling': car.mass * G * car.crr * math.sqrt(1 - grade * grade),
            'drag': .5 * RHO * car.cd * car.area * relative * abs(relative)}


def battery_power(wheel_power, car):
    if wheel_power >= 0:
        return wheel_power / car.efficiency + car.auxiliary
    return -min(-wheel_power, car.regen_power) * car.regen + car.auxiliary


def transition(ds, dh, v0, v1, car, limit=100):
    """Constant acceleration. Integrate power with composite midpoint quadrature.

    Eight panels; signed battery energy in Wh. Negative means net charging.
    Mechanical power maxima include endpoints and all interior stationary points.
    """
    if ds <= 0 or abs(dh) >= ds or min(v0, v1) < 0 or v0 + v1 <= 0:
        return None
    if max(v0, v1) > limit + 1e-9:
        return None
    a = (v1 * v1 - v0 * v0) / (2 * ds)
    if not -car.braking - 1e-9 <= a <= car.acceleration + 1e-9:
        return None
    dt = 2 * ds / (v0 + v1)
    grade = dh / ds
    def power(v):
        return sum(forces(v, a, grade, car).values()) * v
    # P(v)=B*v + k*(v+w)*abs(v+w)*v; derivative is quadratic
    candidates = [v0, v1]
    lo, hi = sorted(candidates)
    k = .5 * RHO * car.cd * car.area
    b = sum(value for key, value in forces(0, a, grade, car).items() if key != 'drag')
    for sign in (-1, 1):
        aa, bb, cc = 3*k*sign, 4*k*sign*car.wind, b+k*sign*car.wind**2
        disc = bb*bb - 4*aa*cc
        if disc >= 0:
            for root in ((-bb + math.sqrt(disc))/(2*aa), (-bb - math.sqrt(disc))/(2*aa)):
                if lo < root < hi and (root + car.wind)*sign >= 0:
                    candidates.append(root)
    if lo < -car.wind < hi:
        candidates.append(-car.wind)
    if max(map(power, candidates)) > car.max_power + 1e-6:
        return None
    totals = dict(draw=0., recovered=0., auxiliary=car.auxiliary*dt/3600,
                  drag=0., rolling=0., grade=0., inertia=0., friction=0.)
    n = 8
    for j in range(n):
        v = v0 + (v1 - v0) * (j + .5) / n
        f = forces(v, a, grade, car)
        p = sum(f.values()) * v
        h = dt / (n * 3600)
        totals['draw'] += max(p, 0) / car.efficiency * h
        totals['recovered'] += min(max(-p, 0), car.regen_power) * car.regen * h
        totals['friction'] += max(-p - car.regen_power, 0) * h
        for key in ('drag', 'rolling', 'grade', 'inertia'):
            totals[key] += f[key] * v * h
    energy = totals['draw'] + totals['auxiliary'] - totals['recovered']
    return {'dt': dt, 'energy': energy, 'a': a, 'power': energy*3600/dt,
            'grade_angle': math.degrees(math.asin(grade)), **totals}


def steady_optimum(car):
    """Flat road, no wind, constant efficiency: derivative of energy per metre."""
    if car.auxiliary == 0:
        return 0.
    return (car.auxiliary * car.efficiency / (RHO * car.cd * car.area)) ** (1/3)
