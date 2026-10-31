"""UoA Aerospace Society - quick script to calculating various CoM vs CoL stuff for the drone.

Script originally written by Dave Riley.

All distances are in mm, all masses are in kg.
Distance axis is one dimensional, measuring from the tip of the propeller at x=0, increasing as
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
class DroneData:
    """Data measured for the drone components; mass and distance to CoM when assembled."""

    nose_assembly: MassDist = MassDist(mass=583, dist=65, debug_name="nose assembly")
    length_nose_assembly: float = 110
    fuselage: MassDist = MassDist(mass=680, dist=360)
    fuselage_g_per_mm: float = 518 / 330  # used in unknown fuselage length calcs
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
    desired_offset_com_wingcom: float = 35  # how far we want CoM in front of the wing CoM
    centre_of_lift: float = 397


def main() -> None:
    """Run the function you want here. Make sure the data is set in the correct class first!"""
    CHECK_COM: bool = True
    COM_WINGS_UNKNOWN: bool = False

    if CHECK_COM:
        check_com()
    if COM_WINGS_UNKNOWN:
        com_wings_unknown(dist_wingcom2overallcom=50)


def com_wings_unknown(*, dist_wingcom2overallcom: float) -> None:
    """Find the required wing CoM for a design.

    Argument is how far in front of the wing CoM we want the overall CoM.
    """
    data: DroneData = DroneData()
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


def check_com() -> None:
    """Calculate actual CoM using DroneDataFindFuseLength data and distance to fuselage CoM."""
    data: DroneData = DroneData()

    points: list[MassDist] = [
        data.nose_assembly,
        data.fuselage,
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


if __name__ == "__main__":
    main()
