"""UoA Aerospace Society - quick script to calculate CoM vs CoL for the drone by Dave Riley.

All distances are in mm, all masses are in kg.
Distance axis is one dimensional, measureing from the tip of the propeller at x=0, increasing as
go further back on the drone.

This is missing some data, and might probably buggy, someone please update and check it. Its using
minimisation to do a brute force of the tail distance, which isn't elegant but was quick to
implement.
"""
from dataclasses import dataclass, field
from pprint import pp

import numpy as np
from scipy import optimize


@dataclass(frozen=True, kw_only=True)
class MassDist:
    """Holds a single centre of mass, distance pair."""

    mass: float
    dist: float

    def product(self) -> float:
        """Mass multiplied by distance."""
        return self.mass * self.dist


@dataclass(frozen=True, kw_only=True)
class DroneData:
    """Data measured for the drone components; mass and distance to CoM when assembled.

    Data is currently for drone as measured on 21/10/2026.
    """

    nose_assembly: MassDist = MassDist(mass=583, dist=65)
    tube: MassDist = MassDist(mass=518, dist=275)  # 330mm overall length
    battery: MassDist = MassDist(mass=865, dist=195)
    motor_speed_contr: MassDist = MassDist(mass=98, dist=195)
    flight_contr: MassDist = MassDist(mass=76, dist=315)
    rear_tube_plug: MassDist = MassDist(
        mass=0, dist=0
    )  # WARNING! not got measurement to hand
    wings: MassDist = MassDist(mass=1920, dist=240)
    wheels: MassDist = MassDist(mass=230, dist=160)
    tail_mass: float = 395  # new lighter foam PLA one
    tail_rods_g_per_mm: float = 0  # WARNING! not got measurement to hand; both rods combined
    desired_centre_of_mass: float = 190
    centre_of_lift: float = 222
    length_tubenose: float = 330 + 110  # combined length of tube and nose assembly when assembed
    sum_masses_exc_tail_tailrods: float = field(
        init=False
    )  # sum of masses excluding tail and tailrods
    sum_massesxdists_exc_tail_tailrods: float = field(
        init=False
    )  # sum of masses x distances excluding tail and tailrods

    def __post_init__(self) -> None:
        """Calculate properties needed optimisation so calcs only done once."""
        super().__setattr__(
            "sum_masses_exc_tail_tailrods",
            sum(
                [
                    self.nose_assembly.mass,
                    self.tube.mass,
                    self.battery.mass,
                    self.motor_speed_contr.mass,
                    self.flight_contr.mass,
                    self.rear_tube_plug.mass,
                    self.wings.mass,
                    self.wheels.mass,
                ]
            ),
        )
        super().__setattr__("sum_massesxdists_exc_tail_tailrods", 
            sum(
                [
                    self.nose_assembly.product(),
                    self.tube.product(),
                    self.battery.product(),
                    self.motor_speed_contr.product(),
                    self.flight_contr.product(),
                    self.rear_tube_plug.product(),
                    self.wings.product(),
                    self.wheels.product(),
                ]
            ),
        )


def main() -> None:
    """Calculated the required tail position based on selected values."""
    data: DroneData = DroneData()

    bnds = optimize.Bounds(
        ub=[np.inf],
        lb=[100],
    )
    constr: tuple[optimize.NonlinearConstraint] = (
        optimize.NonlinearConstraint(  # equality constraint on desired CoM
            fun=lambda x: calc_com(tail_com=x, drone=data),
            ub=[data.desired_centre_of_mass],
            lb=[data.desired_centre_of_mass],
        ),
    )
    res: optimize.OptimizeResult = optimize.minimize(
        fun=lambda x: x,  # minimise distance of tail CoM
        x0=300,  # initial guess
        method="SLSQP",
        bounds=bnds,
        constraints=constr,
        options={"maxiter": 1000},
    )
    pp(res)
    msg: str = f"{np.round(res.x[0], 1)}mm" if res.success else "Optimisation failed"
    print(f"\nTail CoM: {msg}\n")


def calc_com(tail_com: float, drone: DroneData) -> float:
    """Calculates the centre of mass using sum(masses*dists)/sum(masses).

    tail_com: distance from propeller to tail CoM
    """
    ROD_EXTRA: float = 50  # extra rod length for fixing etc
    sum_m: float = drone.sum_masses_exc_tail_tailrods
    sum_mr: float = drone.sum_massesxdists_exc_tail_tailrods
    tail_m: float = drone.tail_mass
    tn_l: float = drone.length_tubenose
    rods_l: float = tail_com - tn_l + ROD_EXTRA
    rods_m: float = rods_l * drone.tail_rods_g_per_mm
    rods_com: float = (rods_l / 2) + tn_l

    sum_masses_dists: float = sum_mr + (tail_m * tail_com) + (rods_m * rods_com)
    sum_masses: float = sum_m + tail_m + rods_m
    return sum_masses_dists / sum_masses


if __name__ == "__main__":
    main()
