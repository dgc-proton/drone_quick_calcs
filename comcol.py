"""UoA Aerospace Society - quick script to calculate CoM vs CoL for the drone by Dave Riley.

All distances are in mm, all masses are in kg.
Distance axis is one dimensional, measureing from the tip of the propeller at x=0, increasing as
go further back on the drone.

This is missing some data, someone please update and check it. Its using minimisation to do a
brute force of the tail distance, which isn't elegant but was quick to
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
class DroneDataFindTailCoM:
    """Data measured for the drone components; mass and distance to CoM when assembled.

    This is for finding the tail CoM.
    """

    nose_assembly: MassDist = MassDist(mass=583, dist=65)
    fuselage: MassDist = MassDist(mass=518, dist=275)  # 330mm overall length
    battery: MassDist = MassDist(mass=865, dist=195)
    motor_speed_contr: MassDist = MassDist(mass=98, dist=195)
    flight_contr: MassDist = MassDist(mass=76, dist=315)
    rear_fuselage_plug: MassDist = MassDist(
        mass=0, dist=0
    )  # WARNING! not got measurement to hand
    wings: MassDist = MassDist(mass=1920, dist=240)
    wheels: MassDist = MassDist(mass=230, dist=160)
    tail_skid: MassDist = MassDist(mass=0, dist=0)  # WARNING! not got details for this yet
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
                    self.fuselage.mass,
                    self.battery.mass,
                    self.motor_speed_contr.mass,
                    self.flight_contr.mass,
                    self.rear_fuselage_plug.mass,
                    self.wings.mass,
                    self.wheels.mass,
                    self.tail_skid.mass,
                ]
            ),
        )
        super().__setattr__("sum_massesxdists_exc_tail_tailrods", 
            sum(
                [
                    self.nose_assembly.product(),
                    self.fuselage.product(),
                    self.battery.product(),
                    self.motor_speed_contr.product(),
                    self.flight_contr.product(),
                    self.rear_fuselage_plug.product(),
                    self.wings.product(),
                    self.wheels.product(),
                    self.tail_skid.product(),
                ]
            ),
        )


@dataclass(frozen=True, kw_only=True)
class DroneDataFindFuseLength:
    """Data measured for the drone components; mass and distance to CoM when assembled.

    This is for finding the fuselage length.
    """

    nose_assembly: MassDist = MassDist(mass=583, dist=65)
    fuselage_mass: float = 518
    fuselage_min_length: float = 200  # minimum fuselage length to fit all the stuff in
    battery: MassDist = MassDist(mass=865, dist=195+244)
    motor_speed_contr: MassDist = MassDist(mass=98, dist=195+244)
    flight_contr: MassDist = MassDist(mass=76, dist=315+244)
    rear_fuselage_plug: MassDist = MassDist(
        mass=0, dist=0
    )  # WARNING! not got measurement to hand
    wings: MassDist = MassDist(mass=1920, dist=240+244)
    wheels: MassDist = MassDist(mass=230, dist=160+244)
    tail_skid: MassDist = MassDist(mass=0, dist=0)  # WARNING! not got details for this yet
    tail: MassDist = MassDist(mass=395, dist=700+244)  # new lighter foam PLA one
    tail_rods: MassDist = MassDist(mass=0, dist=0)  # WARNING! not got measurement to hand; both rods combined
    desired_centre_of_mass: float = 190 + 244
    centre_of_lift: float = 222 + 244
    length_nose_assembly: float = 110
    sum_masses_exc_fuse: float = field(
        init=False
    )  # sum of masses excluding fuselage
    sum_massesxdists_exc_fuse: float = field(
        init=False
    )  # sum of masses x distances excluding fuselage

    def __post_init__(self) -> None:
        """Calculate properties needed optimisation so calcs only done once."""
        super().__setattr__(
            "sum_masses_exc_fuse",
            sum(
                [
                    self.nose_assembly.mass,
                    self.battery.mass,
                    self.motor_speed_contr.mass,
                    self.flight_contr.mass,
                    self.rear_fuselage_plug.mass,
                    self.wings.mass,
                    self.wheels.mass,
                    self.tail_skid.mass,
                    self.tail.mass,
                    self.tail_rods.mass,
                ]
            ),
        )
        super().__setattr__("sum_massesxdists_exc_fuse", 
            sum(
                [
                    self.nose_assembly.product(),
                    self.battery.product(),
                    self.motor_speed_contr.product(),
                    self.flight_contr.product(),
                    self.rear_fuselage_plug.product(),
                    self.wings.product(),
                    self.wheels.product(),
                    self.tail_skid.product(),
                    self.tail.product(),
                    self.tail_rods.product(),
                ]
            ),
        )


def main() -> None:
    """Run the function you want here. Make sure the data is set in the correct class first!"""
    FIND_FUSELAGE_LENGTH: bool = True
    FIND_TAIL_COM: bool = False

    if FIND_TAIL_COM:
        find_tail_com()
    if FIND_FUSELAGE_LENGTH:
        find_fuselage_length()


def find_fuselage_length() -> None:
    """Find fuselage length, all other parameters are fixed."""
    data: DroneDataFindFuseLength = DroneDataFindFuseLength()
    bnds = optimize.Bounds(
        ub=[np.inf],  # not enforcing an upper bound, just be sensible when evaluating the results
        lb=[data.fuselage_min_length],
    )
    constr: tuple[optimize.NonlinearConstraint] = (
        optimize.NonlinearConstraint(  # equality constraint on desired CoM
            fun=lambda x: com_from_fuse_length(fuse_l=x, drone=data),
            ub=[data.desired_centre_of_mass],
            lb=[data.desired_centre_of_mass],
        ),
    )
    res: optimize.OptimizeResult = optimize.minimize(
        fun=lambda x: x,  # minimise length of fuselage
        x0=data.fuselage_min_length,  # initial guess
        method="SLSQP",
        bounds=bnds,
        constraints=constr,
        options={"maxiter": 1500},
    )
    pp(res)
    if res.success:
        msg: str = f"{np.round(res.x[0], 1)}mm"
    else:
        msg: str = "Optimisation failed, design possibly infeasible"
    print(f"\nFuselage length: {msg}\n")


def find_tail_com() -> None:
    """Find CoM of tail, all other parameters are fixed."""
    data: DroneDataFindTailCoM = DroneDataFindTailCoM()
    bnds = optimize.Bounds(
        ub=[np.inf],
        lb=[data.length_tubenose + 100],  # tail can't be in the fuselage
    )
    constr: tuple[optimize.NonlinearConstraint] = (
        optimize.NonlinearConstraint(  # equality constraint on desired CoM
            fun=lambda x: calc_tail_com(tail_com=x, drone=data),
            ub=[data.desired_centre_of_mass],
            lb=[data.desired_centre_of_mass],
        ),
    )
    res: optimize.OptimizeResult = optimize.minimize(
        fun=lambda x: x,  # minimise distance of tail CoM
        x0=data.length_tubenose + 150,  # initial guess
        method="SLSQP",
        bounds=bnds,
        constraints=constr,
        options={"maxiter": 1000},
    )
    pp(res)
    if res.success:
        msg: str = f"{np.round(res.x[0], 1)}mm"
    else:
        msg: str = "Optimisation failed, design possibly infeasible"
    print(f"\nTail CoM: {msg}\n")


def com_from_fuse_length(fuse_l: float, drone:DroneDataFindFuseLength) -> float:
    """Calculates the fuselage length using sum(masses*dists)/sum(masses).

    Returns current overall CoM
    """
    sum_m: float = drone.sum_masses_exc_fuse
    sum_mr: float = drone.sum_massesxdists_exc_fuse
    fuse_m: float = drone.fuselage_mass
    nose_l: float = drone.length_nose_assembly

    sum_masses_dists: float = sum_mr + (fuse_m * ((fuse_l/2)+nose_l))
    sum_masses: float = sum_m + fuse_m
    return sum_masses_dists / sum_masses


def calc_tail_com(tail_com: float, drone: DroneDataFindTailCoM) -> float:
    """Calculates the centre of mass using sum(masses*dists)/sum(masses).

    tail_com: distance from propeller to tail CoM
    """
    ROD_EXTRA: float = 50  # extra rod length for fixing it in place etc
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
