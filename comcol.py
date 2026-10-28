"""UoA Aerospace Society - quick script to calculating various CoM vs CoL stuff for the drone.

Script originally written by Dave Riley.

All distances are in mm, all masses are in kg.
Distance axis is one dimensional, measureing from the tip of the propeller at x=0, increasing as
go further back on the drone.
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
    debug_name: str | None = None

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
    tail_skid: MassDist = MassDist(
        mass=0, dist=0
    )  # WARNING! not got details for this yet
    tail_mass: float = 395  # new lighter foam PLA one
    tail_rods_g_per_mm: float = (
        0  # WARNING! not got measurement to hand; both rods combined
    )
    desired_centre_of_mass: float = 190
    centre_of_lift: float = 222
    length_tubenose: float = (
        330 + 110
    )  # combined length of tube and nose assembly when assembed
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
        super().__setattr__(
            "sum_massesxdists_exc_tail_tailrods",
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
class DroneDataGeneral:
    """Data measured for the drone components; mass and distance to CoM when assembled."""

    nose_assembly: MassDist = MassDist(mass=583, dist=65, debug_name="nose assembly")
    fuselage: MassDist = MassDist(mass=680, dist=360)
    fuselage_min_length: float = 200  # minimum fuselage length to fit all the stuff in
    battery: MassDist = MassDist(mass=865, dist=225, debug_name="battery")
    motor_speed_contr: MassDist = MassDist(
        mass=98, dist=225, debug_name="motor speed controller"
    )
    flight_contr: MassDist = MassDist(mass=76, dist=275, debug_name="flight controller")
    rear_fuselage_plug: MassDist = MassDist(
        mass=173, dist=650, debug_name="rear fuselage plug"
    )
    wings: MassDist = MassDist(mass=1920, dist=447, debug_name="wings")
    wheels: MassDist = MassDist(mass=257, dist=130, debug_name="wheels")
    tail_skid: MassDist = MassDist(mass=28, dist=1145, debug_name="tail skid")
    tail: MassDist = MassDist(
        mass=395, dist=1145, debug_name="tail"
    )  # new lighter foam PLA one
    tail_rods: MassDist = MassDist(mass=60, dist=800, debug_name="tail rods")
    desired_centre_of_mass: float = 397
    centre_of_lift: float = 429
    length_nose_assembly: float = 110
    sum_masses_exc_fuse: float = field(init=False)  # sum of masses excluding fuselage
    sum_massesxdists_exc_fuse: float = field(
        init=False
    )  # sum of masses x distances excluding fuselage

    def __post_init__(self) -> None:
        """Calculate properties needed optimisation so calcs only done once."""
        super().__setattr__("fuse_com_unknown_fuselage_mass", self.fuselage.mass)
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
        super().__setattr__(
            "sum_massesxdists_exc_fuse",
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
    FIND_FUSELAGE_LENGTH: bool = False
    FIND_TAIL_COM: bool = False
    CHECK_COM: bool = False
    COM_WINGS_UNKNOWN: bool = True

    if FIND_TAIL_COM:
        find_tail_com()
    if FIND_FUSELAGE_LENGTH:
        find_fuselage_length()
    if CHECK_COM:
        # simple get user input for length, no checks done on it so be careful
        # fuseelage CoM = (fuse_length/2) + nose_assembly_length
        check_com(
            fuse_com_mm=float(
                input("Enter fuselage CoM in mm for checking overall CoM: ")
            )
        )
    if COM_WINGS_UNKNOWN:
        com_wings_unknown(dist_wingcom2overallcom=50)


def com_wings_unknown(*, dist_wingcom2overallcom: float) -> None:
    """Find the required wing CoM for a design.

    Argument is how far in front of the wing CoM we want the overall CoM.
    """
    data: DroneDataGeneral = DroneDataGeneral()
    sum_m: float = np.sum(
        [
            data.nose_assembly.mass,
            data.fuselage.mass,
            data.battery.mass,
            data.motor_speed_contr.mass,
            data.flight_contr.mass,
            data.rear_fuselage_plug.mass,
            data.wings.mass,
            data.wheels.mass,
            data.tail_skid.mass,
            data.tail.mass,
            data.tail_rods.mass,
        ]
    )
    sum_mr_exc_wings: float = np.sum(
        [
            data.nose_assembly.product(),
            data.fuselage.product(),
            data.battery.product(),
            data.motor_speed_contr.product(),
            data.flight_contr.product(),
            data.rear_fuselage_plug.product(),
            data.wheels.product(),
            data.tail_skid.product(),
            data.tail.product(),
            data.tail_rods.product(),
        ]
    )

    print(f"sum of masses = {sum_m}")
    print(f"sum of (masses * distance) = {sum_mr_exc_wings}")

    num: float = sum_mr_exc_wings + (dist_wingcom2overallcom * sum_m)
    den: float = sum_m - data.wings.mass
    wings_com: float = num / den

    print(f"Wings CoM = {wings_com:.2f}mm")


def check_com(*, fuse_com_mm: float) -> None:
    """Calculate actual CoM using DroneDataFindFuseLength data and distance to fuselage CoM."""
    data: DroneDataGeneral = DroneDataGeneral()

    points: list[MassDist] = [
        data.nose_assembly,
        MassDist(mass=data.fuse_com_unknown_fuselage_mass, dist=fuse_com_mm, debug_name="fuselage"),
        data.battery,
        data.motor_speed_contr,
        data.flight_contr,
        data.rear_fuselage_plug,
        data.wings,
        data.wheels,
        data.tail_skid,
        data.tail,
        data.tail_rods,
    ]

    sum_mr: float = 0
    sum_m: float = 0
    print("\nCom calculations:")
    for point in points:
        print(
            f"{point.debug_name}: {point.mass}g @{point.dist}mm = {point.product()} g*mm"
        )
        sum_mr += point.product()
        sum_m += point.mass

    print(f"sum of masses = {sum_m}")
    print(f"sum of (masses * distance) = {sum_mr}")
    com: float = sum_mr / sum_m

    if com < data.centre_of_lift:
        msg_start: str = "CoM is in front of CoL, design possibly stable:\n"
    else:
        msg_start: str = "CoM is not in front of CoL, design unstable:\n"

    print(msg_start + (f"CoM: {com:.2f}mm\nCoL: {data.centre_of_lift:.2f}mm"))


def find_fuselage_length() -> None:
    """Find fuselage length, all other parameters are fixed.

    Because there is only one design varibale (no interdependence) this could and probably should
    be rewritten as a simple bit of maths instead of an optimisation problem.
    """
    data: DroneDataGeneral = DroneDataGeneral()
    bnds = optimize.Bounds(
        ub=[
            np.inf
        ],  # not enforcing an upper bound, just be sensible when evaluating the results
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


def com_from_fuse_length(fuse_l: float, drone: DroneDataGeneral) -> float:
    """Calculates the fuselage length using sum(masses*dists)/sum(masses).

    Returns current overall CoM
    """
    sum_m: float = drone.sum_masses_exc_fuse
    sum_mr: float = drone.sum_massesxdists_exc_fuse
    fuse_m: float = drone.fuse_com_unknown_fuselage_mass
    nose_l: float = drone.length_nose_assembly

    sum_masses_dists: float = sum_mr + (fuse_m * ((fuse_l / 2) + nose_l))
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
