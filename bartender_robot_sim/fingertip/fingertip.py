#!/usr/bin/env python3
"""Parametric neck-hooking fingertip for a Robotiq 2F-85 gripper.

Lets a UR5e pick a glass bottle up by the neck: the tip hooks under the
bottle's collar ring so the bottle hangs on a mechanical ledge instead of
being held by pad friction.

Built in three geometry increments, each checked against hand arithmetic
before the next was cut into it: neck pocket, then collar ledge, then the
lead-in tapers. The mounting plate is last because its dimensions are not
known yet -- see MOUNT_PLACEHOLDERS.

GEOMETRY CONVENTION
    origin  centre of the mounting face, on the gripper side
    +X      away from the mounting face, toward the bottle (closing axis)
    +Y      across the finger
    +Z      up, parallel to the bottle axis when the bottle is upright
so the part occupies X in [0, mount_plate_t + body_t]. A "left" part is
modelled and the right is its mirror in the XZ plane.

USAGE
    python fingertip.py --neck-d 30 --collar-d 36 --out ./build
    python fingertip.py --mount-check --out ./build   # bolt-pattern test piece
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, fields, replace
from pathlib import Path

from build123d import (Align, Axis, Box, Cone, Cylinder, Plane, Pos, chamfer,
                       export_step, export_stl, mirror)


# --------------------------------------------------------------------------
# MOUNTING INTERFACE PLACEHOLDERS -- NOT REAL ROBOTIQ DIMENSIONS.
#
# These numbers are invented so the model has something to build. They are
# NOT from Robotiq's drawings, NOT measured, and must not be trusted. Replace
# every one of them with a value measured off the real fingertip, or read out
# of Robotiq's official 2F-85 STEP file, before printing anything that has to
# bolt on. validate() warns loudly while any of them still match these values.
#
# Print the --mount-check test piece and offer it up to the gripper before
# committing to a full tip.
# --------------------------------------------------------------------------
MOUNT_PLACEHOLDERS = {
    "mount_hole_spacing": 20.0,
    "mount_hole_d": 4.5,
    "mount_face_w": 22.0,
    "mount_face_h": 37.0,
    "mount_plate_t": 5.0,
}


# --------------------------------------------------------------------------
# BOTTLE FINISHES
#
# The "finish" is the neck and mouth of a glass bottle, and it is the most
# standardised part of one: closures have to fit it, so it is specified where
# the body is not. The feature this fingertip hooks under is usually the
# TRANSFER BEAD (or transfer ring), the raised ring just below the finish --
# which exists so that factory conveyors can carry bottles by the neck. This
# tip is a small version of equipment that already exists.
#
# So the sane way to drive this generator is to name a finish rather than
# measure each bottle. The names below are real; THE NUMBERS ARE NOT FILLED
# IN, because they have to come from the finish drawing for the bottle you are
# actually running, not from memory. Get them from the glass supplier's finish
# spec, the GPI/SPI or CETIE standard sheet, or a caliper, then record where
# they came from in `source` so the next person can check.
#
#     neck_d   outside diameter of the neck BELOW the bead
#     collar_d outside diameter of the bead itself
#
# Using a finish whose numbers are still None fails loudly rather than
# guessing. --finish is a convenience over --neck-d/--collar-d, not a
# substitute for knowing the dimensions.
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Finish:
    """A standard bottle finish. None means "not filled in yet"."""

    name: str
    neck_d: float | None = None      # mm, neck OD below the transfer bead
    collar_d: float | None = None    # mm, transfer bead OD
    source: str = ""                 # where those numbers came from
    note: str = ""


FINISHES = {
    f.name: f for f in (
        Finish("crown", note="beer, 26mm crown cap (GPI/CETIE crown finish)"),
        Finish("bvs30h60", note="wine/spirits screwcap, BVS 30H60"),
        Finish("wine-cork", note="still wine, CETIE cork finish"),
        Finish("gpi-400", note="GPI/SPI 400 continuous thread"),
    )
}


class ValidationError(Exception):
    """Geometry that would produce a part that cannot work. Never clamped."""


@dataclass(frozen=True)
class Params:
    """Every dimension of the fingertip. Units: mm and degrees."""

    # ---- bottle: measure these off the bottles you actually run ----------
    neck_d: float = 30.0          # mm  neck outside diameter, below the collar
    collar_d: float = 36.0        # mm  collar ring outside diameter
    clearance: float = 0.6        # mm  radial gap between neck and pocket wall

    # ---- neck pocket -----------------------------------------------------
    pocket_arc: float = 120.0     # deg angular span of the pocket, per tip
    # Defaults to the full body height, so the "pocket floor" is the bottom
    # face of the tip and the bottom lead-in becomes a true entry funnel.
    # Set it shorter to leave a closed floor under the pocket.
    pocket_h: float = 34.0        # mm  pocket height, floor to top of the tip

    # ---- collar ledge ----------------------------------------------------
    # The ledge is the load-bearing surface: the collar's underside sits on it
    # and the bottle hangs there. ledge_h splits the pocket -- below it the
    # wall hugs the neck and stops the bottle swinging, above it the wall is
    # cut back by ledge_depth to clear the collar. So ledge_h wants to be most
    # of pocket_h, leaving just enough above for the collar and the mouth.
    ledge_h: float = 16.0         # mm  ledge height above the pocket floor
    # Must be big enough that the upper pocket actually CLEARS the collar,
    # i.e. pocket_r + ledge_depth >= collar_r + clearance. See validate() and
    # the note in the README about the direction of this check.
    ledge_depth: float = 3.2      # mm  radial step outward above the ledge

    # ---- lead-in taper ---------------------------------------------------
    # Measured from the closing axis (X). This cannot be as shallow as it
    # looks: the pocket wall's own tangent at the entry already makes
    # (90 - pocket_arc/2) degrees with X, so at the default 120deg arc a 30deg
    # lead-in is exactly degenerate and cuts nothing. validate() enforces it.
    leadin_angle: float = 45.0    # deg taper angle at pocket bottom and entry
    leadin_h: float = 3.0         # mm  depth of the lead-in, along Z and X

    # ---- body ------------------------------------------------------------
    body_t: float = 16.0          # mm  thickness, front of plate to front face
    # Wide enough that the COLLAR pocket, which reaches round further than the
    # neck pocket, still closes inside the body. Too narrow and the upper
    # pocket breaks out of the sides and truncates the load-bearing ledge --
    # validate() computes the minimum and refuses.
    body_w: float = 40.0          # mm  width across the finger (Y)
    body_h: float = 34.0          # mm  overall height (Z)
    wall_min: float = 2.0         # mm  thinnest wall left behind the pocket

    # ---- mounting interface: PLACEHOLDERS, see MOUNT_PLACEHOLDERS above --
    mount_hole_spacing: float = MOUNT_PLACEHOLDERS["mount_hole_spacing"]
    mount_hole_d: float = MOUNT_PLACEHOLDERS["mount_hole_d"]
    mount_face_w: float = MOUNT_PLACEHOLDERS["mount_face_w"]
    mount_face_h: float = MOUNT_PLACEHOLDERS["mount_face_h"]
    mount_plate_t: float = MOUNT_PLACEHOLDERS["mount_plate_t"]

    # ---- fasteners -------------------------------------------------------
    countersink: bool = True      # -    False gives plain through-holes
    csk_head_d: float = 8.4       # mm  M4 countersunk head diameter (ISO 10642)
    csk_angle: float = 90.0       # deg included angle of an M4 csk head

    # ---- gripper limits --------------------------------------------------
    max_stroke: float = 85.0      # mm  2F-85 maximum opening between pads
    min_opening: float = 40.0     # mm  opening that must remain with tips on

    # ---- derived: bottle and pocket --------------------------------------
    @property
    def neck_r(self) -> float:
        return self.neck_d / 2.0

    @property
    def collar_r(self) -> float:
        return self.collar_d / 2.0

    @property
    def pocket_r(self) -> float:
        """Radius of the lower pocket, the part that hugs the neck."""
        return self.neck_r + self.clearance

    @property
    def collar_pocket_r(self) -> float:
        """Radius of the upper pocket, the part that clears the collar."""
        return self.pocket_r + self.ledge_depth

    @property
    def half_arc(self) -> float:
        return math.radians(self.pocket_arc / 2.0)

    @property
    def front_x(self) -> float:
        """X of the front face: the whole protrusion from the mounting face."""
        return self.mount_plate_t + self.body_t

    @property
    def axis_x(self) -> float:
        """X of the bottle axis.

        The pocket is a vertical cylinder centred AHEAD of the front face. For
        the cut to subtend exactly pocket_arc, the axis has to sit at
        front_x + r*cos(arc/2), which leaves a bite of r*(1 - cos(arc/2)).
        Both pockets share this axis -- it is the bottle's axis.
        """
        return self.front_x + self.pocket_r * math.cos(self.half_arc)

    def pocket_depth(self, radius: float) -> float:
        """How far a pocket of this radius bites into the front face."""
        return radius - (self.axis_x - self.front_x)

    # ---- derived: ledge --------------------------------------------------
    @property
    def collar_overhang(self) -> float:
        """How far the collar sticks out past the pocket wall below it."""
        return self.collar_r - self.pocket_r

    @property
    def ledge_bearing_width(self) -> float:
        """Radial width of the annulus the collar actually rests on.

        The ledge annulus runs from pocket_r out to collar_pocket_r; the
        collar's underside runs from neck_r out to collar_r. The overlap --
        the bit that carries the bottle -- is from pocket_r to whichever of
        the two outer radii is the smaller.
        """
        return min(self.collar_pocket_r, self.collar_r) - self.pocket_r

    @property
    def pocket_floor_z(self) -> float:
        return self.body_h - self.pocket_h

    @property
    def ledge_z(self) -> float:
        return self.pocket_floor_z + self.ledge_h

    # ---- derived: widths -------------------------------------------------
    @property
    def collar_half_angle(self) -> float:
        """Half-angle the collar pocket subtends: wider, being a bigger circle
        cut by the same front-face plane."""
        d = self.axis_x - self.front_x
        return math.acos(min(1.0, d / self.collar_pocket_r))

    @property
    def collar_half_width(self) -> float:
        """Half the chord the collar pocket cuts in the front face."""
        return self.collar_pocket_r * math.sin(self.collar_half_angle)

    @property
    def entry_flat_width(self) -> float:
        """Flat front face left beside the collar pocket, per side.

        This is the horn the entry chamfer is cut into, so it bounds how big
        that chamfer can be. The COLLAR pocket sets it, not the neck pocket,
        because the collar pocket reaches round further.
        """
        return self.body_w / 2.0 - self.collar_half_width

    @property
    def min_body_w(self) -> float:
        """Narrowest body that still contains the collar pocket, plus walls."""
        return 2.0 * (self.collar_half_width + self.wall_min)

    # ---- derived: lead-ins -----------------------------------------------
    @property
    def entry_tangent_angle(self) -> float:
        """Angle the pocket wall already makes with X at the pocket entry."""
        return 90.0 - self.pocket_arc / 2.0

    @property
    def leadin_entry_b(self) -> float:
        """Chamfer leg along the front face, for a leadin_angle entry taper.

        Legs a (along the pocket wall) and b (along the front face) give a
        face at angle alpha to X when
            tan(alpha) = (b + a*cos(A)) / (a*sin(A)),   A = pocket_arc/2
        so b = a*(sin(A)*tan(alpha) - cos(A)). That is <= 0 exactly when
        alpha <= 90 - A, which is the degenerate case guarded in validate().
        """
        A = self.half_arc
        a = self.leadin_h
        return a * (math.sin(A) * math.tan(math.radians(self.leadin_angle))
                    - math.cos(A))

    @property
    def leadin_bottom_r(self) -> float:
        """Radius the bottom flare opens out to at the pocket floor."""
        return self.pocket_r + self.leadin_h * math.tan(
            math.radians(self.leadin_angle))

    # ---- derived: mount --------------------------------------------------
    @property
    def hole_z(self) -> tuple[float, float]:
        """Z of the two bolt holes, centred on the body height."""
        mid = self.body_h / 2.0
        return (mid - self.mount_hole_spacing / 2.0,
                mid + self.mount_hole_spacing / 2.0)

    @property
    def csk_depth(self) -> float:
        """Axial depth of an M4 countersink of csk_head_d at csk_angle."""
        if not self.countersink:
            return 0.0
        return ((self.csk_head_d - self.mount_hole_d) / 2.0
                / math.tan(math.radians(self.csk_angle / 2.0)))

    def wall_x_at_z(self, z: float) -> float:
        """X of the pocket wall on the centreline (Y=0) at this height.

        This is how much material a bolt at Y=0 has to sit in, and where its
        countersink has to start.
        """
        if z < self.pocket_floor_z or self.pocket_h <= 0.0:
            return self.front_x
        r = self.pocket_r if z <= self.ledge_z else self.collar_pocket_r
        return self.front_x - self.pocket_depth(r)

    # ---- derived: stroke -------------------------------------------------
    @property
    def opening_with_tips(self) -> float:
        """Gap left between the two front faces at full open."""
        return self.max_stroke - 2.0 * self.front_x

    def mount_is_placeholder(self) -> list[str]:
        """Which mounting dimensions are still at their invented values."""
        return [k for k, v in MOUNT_PLACEHOLDERS.items()
                if math.isclose(getattr(self, k), v, rel_tol=0, abs_tol=1e-9)]


PARAMS = Params()


# --------------------------------------------------------------------------
# validation
# --------------------------------------------------------------------------
def validate(p: Params) -> list[str]:
    """Check the geometry can actually work. Raise on anything fatal.

    Returns a list of warnings. Nothing is ever silently clamped: if a
    dimension makes the part unbuildable or useless, this raises with the
    numbers that failed and what to change.

    This catches everything that can be predicted from the numbers alone.
    build_tip() raises the same ValidationError for the few limits that only
    the CAD kernel knows about, so callers should guard both.
    """
    errors: list[str] = []
    warnings: list[str] = []

    if p.pocket_r <= 0.0:
        errors.append(
            f"pocket radius is {p.pocket_r:.2f} mm, must be > 0 "
            f"(neck_d={p.neck_d}, clearance={p.clearance})")
    if p.collar_d <= p.neck_d:
        errors.append(
            f"collar_d {p.collar_d} must exceed neck_d {p.neck_d}: with no "
            f"collar overhang there is nothing for the ledge to hook under")

    # The ledge has to clear the collar, or the collar cannot get above it.
    # NOTE ON DIRECTION: the brief asked to assert ledge_depth < collar
    # overhang. That is the opposite of what the mechanism needs and it is
    # asserted the other way here -- see the README section "The ledge check".
    need = p.collar_r + p.clearance
    if p.collar_pocket_r < need:
        errors.append(
            f"upper pocket radius {p.collar_pocket_r:.2f} mm does not clear "
            f"the collar, which needs {need:.2f} mm "
            f"(collar_r {p.collar_r:.2f} + clearance {p.clearance}). "
            f"Raise ledge_depth to at least "
            f"{need - p.pocket_r:.2f} mm")
    if p.ledge_bearing_width <= 0.0:
        errors.append(
            f"ledge bearing width is {p.ledge_bearing_width:.2f} mm: the "
            f"collar would not touch the ledge at all")
    elif p.ledge_bearing_width < 1.0:
        warnings.append(
            f"ledge bearing width is only {p.ledge_bearing_width:.2f} mm; "
            f"that is a thin shelf to hang a bottle on")
    if not 0.0 < p.ledge_h < p.pocket_h:
        errors.append(
            f"ledge_h {p.ledge_h} must sit inside the pocket "
            f"(0 .. pocket_h {p.pocket_h})")

    # Lead-ins.
    if p.leadin_h > 0.0 and p.leadin_angle <= p.entry_tangent_angle:
        errors.append(
            f"leadin_angle {p.leadin_angle} deg is degenerate: at "
            f"pocket_arc {p.pocket_arc} deg the pocket wall already leaves "
            f"the entry at {p.entry_tangent_angle:.1f} deg from the closing "
            f"axis, so the taper would cut nothing. Use more than "
            f"{p.entry_tangent_angle:.1f} deg")
    elif p.leadin_h > 0.0:
        # The entry chamfer is cut into the flat horn beside the collar
        # pocket. Ask for a leg wider than that horn and OCCT does not fail
        # gracefully -- it throws "Failed creating a chamfer, try a smaller
        # length value(s)" from deep inside the kernel, which says nothing
        # about which parameter caused it. Caught here instead.
        need = p.leadin_entry_b
        have = p.entry_flat_width
        if need > have - 0.1:
            errors.append(
                f"entry lead-in needs {need:.2f} mm of flat front face but "
                f"only {have:.2f} mm is left beside the collar pocket. "
                f"Reduce leadin_angle (currently {p.leadin_angle} deg) or "
                f"leadin_h ({p.leadin_h} mm), or widen body_w to at least "
                f"{2.0 * (p.collar_half_width + need + 0.1):.2f} mm")

    # Body has to contain the deeper of the two pockets, and the wider one.
    deepest = max(p.pocket_depth(p.collar_pocket_r), p.leadin_bottom_r -
                  (p.axis_x - p.front_x))
    wall = p.body_t - deepest
    if wall < p.wall_min:
        errors.append(
            f"only {wall:.2f} mm of wall left behind the pocket, minimum is "
            f"{p.wall_min} mm. Raise body_t to at least "
            f"{deepest + p.wall_min:.2f} mm")
    if p.body_w < p.min_body_w:
        errors.append(
            f"body_w {p.body_w} mm is too narrow: the collar pocket spans "
            f"{2 * p.collar_half_width:.2f} mm and would break out of the "
            f"sides, truncating the ledge. Use at least "
            f"{p.min_body_w:.2f} mm")

    # Gripper stroke. The two front faces are what the bottle has to pass.
    if p.opening_with_tips < p.min_opening:
        errors.append(
            f"tips leave only {p.opening_with_tips:.2f} mm of opening "
            f"({p.max_stroke} mm stroke less 2 x {p.front_x:.2f} mm of tip), "
            f"below min_opening {p.min_opening} mm. Reduce body_t or "
            f"mount_plate_t by {(p.min_opening - p.opening_with_tips) / 2.0:.2f} mm each")

    # Mounting interface.
    stale = p.mount_is_placeholder()
    if stale:
        warnings.append(
            "MOUNTING DIMENSIONS ARE STILL PLACEHOLDERS, not real Robotiq "
            "values: " + ", ".join(sorted(stale)) +
            ". Measure them off the fingertip or take them from Robotiq's "
            "official 2F-85 STEP file, then print --mount-check and offer it "
            "up before printing a full tip.")
    if p.mount_hole_d <= 0.0:
        errors.append(f"mount_hole_d {p.mount_hole_d} must be > 0")
    if p.mount_hole_spacing + p.mount_hole_d > p.mount_face_h:
        errors.append(
            f"bolt holes ({p.mount_hole_spacing} mm apart, "
            f"{p.mount_hole_d} mm across) do not fit inside mount_face_h "
            f"{p.mount_face_h} mm")
    over = []
    if p.mount_face_w > p.body_w:
        over.append(f"width {p.mount_face_w} > {p.body_w}")
    if p.mount_face_h > p.body_h:
        over.append(f"height {p.mount_face_h} > {p.body_h}")
    if over:
        warnings.append(
            "mounting plate overhangs the body (" + "; ".join(over) +
            " mm), so the plate sticks out past the tip")
    if p.countersink:
        for z in p.hole_z:
            avail = p.wall_x_at_z(z)
            if p.csk_depth >= avail:
                errors.append(
                    f"countersink at z={z:.1f} is {p.csk_depth:.2f} mm deep "
                    f"but there is only {avail:.2f} mm of material there; the "
                    f"head would break through. Reduce csk_head_d or move the "
                    f"hole")
        if p.csk_head_d <= p.mount_hole_d:
            errors.append(
                f"csk_head_d {p.csk_head_d} must exceed mount_hole_d "
                f"{p.mount_hole_d}")

    if errors:
        raise ValidationError(
            "fingertip geometry is not buildable:\n  - " +
            "\n  - ".join(errors))
    return warnings


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------
def _pocket_cutter(p: Params, radius: float, z0: float, height: float):
    """A vertical part-cylinder cut into the front face.

    Both pockets share the bottle's axis, so they take the same axis_x, which
    is fixed by the LOWER pocket's arc. The upper one is wider, so it reaches
    round further -- deliberately: it has to clear the collar over at least as
    much arc as the neck pocket grips.
    """
    cutter = Cylinder(radius=radius, height=height,
                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    return Pos(p.axis_x, 0.0, z0) * cutter


def build_mount_plate(p: Params):
    """The flat boss that beds against the gripper finger, with its bolts.

    Kept separate from the tip so --mount-check can export it on its own as a
    five-minute test print.
    """
    plate = Box(p.mount_plate_t, p.mount_face_w, p.mount_face_h,
                align=(Align.MIN, Align.CENTER, Align.MIN))
    plate = Pos(0.0, 0.0, (p.body_h - p.mount_face_h) / 2.0) * plate
    return plate - _bolt_holes(p, through_to=p.mount_plate_t)


def _bolt_holes(p: Params, through_to: float):
    """Through-holes along X, countersunk from the bottle side.

    The bolt goes in from the pocket side and threads into the finger, so the
    head is countersunk into whatever surface is frontmost at that height --
    which inside the pocket is the pocket wall, not the front face. wall_x_at_z
    is what finds it, and validate() checks the head does not break through.
    """
    holes = None
    for z in p.hole_z:
        shaft = Cylinder(radius=p.mount_hole_d / 2.0, height=through_to + 2.0,
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
        cut = Pos(-1.0, 0.0, z) * Plane.YZ.location * shaft
        holes = cut if holes is None else holes + cut
        if p.countersink and p.csk_depth > 0.0:
            face_x = min(p.wall_x_at_z(z), through_to)
            # Run the cone a little PAST the surface it countersinks into.
            # Ending it exactly on the pocket wall leaves the cone's rim
            # tangent to that cylinder, which OCCT resolves into an invalid
            # solid (is_valid goes False while the volume still looks right).
            # Overshooting makes the intersection transversal; the extra cone
            # is outside the part, so the countersink is unchanged.
            over = 0.5
            slope = math.tan(math.radians(p.csk_angle / 2.0))
            cone = Cone(bottom_radius=p.mount_hole_d / 2.0,
                        top_radius=p.csk_head_d / 2.0 + over * slope,
                        height=p.csk_depth + over,
                        align=(Align.CENTER, Align.CENTER, Align.MIN))
            cone = Plane.YZ.location * cone
            cone = Pos(face_x - p.csk_depth, 0.0, z) * cone
            holes = holes + cone
    return holes


def build_tip(p: Params):
    """The whole fingertip: mount plate, body, pocket, ledge and lead-ins."""
    plate = Box(p.mount_plate_t, p.mount_face_w, p.mount_face_h,
                align=(Align.MIN, Align.CENTER, Align.MIN))
    plate = Pos(0.0, 0.0, (p.body_h - p.mount_face_h) / 2.0) * plate
    body = Box(p.body_t, p.body_w, p.body_h,
               align=(Align.MIN, Align.CENTER, Align.MIN))
    part = plate + Pos(p.mount_plate_t, 0.0, 0.0) * body

    # Below the ledge: hugs the neck. Above it: stepped out to clear the
    # collar, cut a little past the top so the pocket opens out of the top.
    part -= _pocket_cutter(p, p.pocket_r, p.pocket_floor_z, p.ledge_h)
    part -= _pocket_cutter(p, p.collar_pocket_r, p.ledge_z,
                           p.pocket_h - p.ledge_h + 1.0)

    # Bottom lead-in: flare the pocket out towards the floor, so a neck that
    # meets the tip low is cammed up into the pocket instead of stopping it.
    if p.leadin_h > 0.0:
        flare = Cone(bottom_radius=p.leadin_bottom_r, top_radius=p.pocket_r,
                     height=p.leadin_h,
                     align=(Align.CENTER, Align.CENTER, Align.MIN))
        part -= Pos(p.axis_x, 0.0, p.pocket_floor_z) * flare

    # Entry lead-in: chamfer the vertical corners where the pockets break out
    # of the front face, so a neck that is off-centre slides in rather than
    # butting against a sharp horn.
    b = p.leadin_entry_b
    if b > 1e-6:
        edges = [e for e in part.edges().filter_by(Axis.Z)
                 if abs(e.center().X - p.front_x) < 1e-6
                 and abs(e.center().Y) < p.body_w / 2.0 - 1e-6]
        if edges:
            try:
                part = chamfer(edges, length=b, length2=p.leadin_h)
            except ValueError as exc:
                # validate() catches the chamfer sizes it can predict, but the
                # kernel has its own limits and reports them as a bare
                # "try a smaller length value(s)" that names no parameter.
                # Re-raise with the numbers, so the failure is actionable.
                raise ValidationError(
                    f"the entry lead-in could not be cut: legs "
                    f"{b:.2f} mm along the front face and {p.leadin_h:.2f} mm "
                    f"along the pocket wall, into the {p.entry_flat_width:.2f} "
                    f"mm of flat beside the collar pocket. Reduce "
                    f"leadin_angle ({p.leadin_angle} deg) or leadin_h "
                    f"({p.leadin_h} mm), or widen body_w ({p.body_w} mm). "
                    f"Kernel said: {exc}") from exc

    return part - _bolt_holes(p, through_to=p.front_x)


def build_pair(p: Params):
    left = build_tip(p)
    return left, mirror(left, about=Plane.XZ)


# --------------------------------------------------------------------------
# export / CLI
# --------------------------------------------------------------------------
def export(part, stem: str, out: Path, p: Params | None = None) -> None:
    out.mkdir(parents=True, exist_ok=True)
    export_step(part, str(out / f"{stem}.step"))
    export_stl(part, str(out / f"{stem}.stl"))
    if p is not None:
        # Sidecar of the derived dimensions, so anything that has to POSITION
        # this mesh (the Gazebo/MoveIt description, for one) does not have to
        # re-derive them or keep a second copy in step with this file.
        (out / f"{stem}.json").write_text(json.dumps({
            "body_w": p.body_w, "body_h": p.body_h, "body_t": p.body_t,
            "mount_plate_t": p.mount_plate_t, "front_x": p.front_x,
            "neck_d": p.neck_d, "collar_d": p.collar_d,
            "pocket_r": p.pocket_r, "pocket_arc": p.pocket_arc,
            "axis_x": p.axis_x, "ledge_z": p.ledge_z,
            "ledge_bearing_width": p.ledge_bearing_width,
            "opening_with_tips": p.opening_with_tips,
        }, indent=2) + "\n")
    bb = part.bounding_box()
    print(f"  {stem:22s} volume {part.volume:9.1f} mm^3   "
          f"bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm")


def _summary(p: Params) -> str:
    return (
        f"  neck {p.neck_d} / collar {p.collar_d} mm, clearance {p.clearance}\n"
        f"  pocket r {p.pocket_r:.2f} mm over {p.pocket_arc}deg, "
        f"{p.pocket_depth(p.pocket_r):.2f} mm deep\n"
        f"  collar pocket r {p.collar_pocket_r:.2f} mm over "
        f"{2 * math.degrees(p.collar_half_angle):.1f}deg\n"
        f"  ledge at z {p.ledge_z:.1f} mm, bearing width "
        f"{p.ledge_bearing_width:.2f} mm\n"
        f"  tip protrudes {p.front_x:.1f} mm, leaving "
        f"{p.opening_with_tips:.1f} mm of the {p.max_stroke} mm stroke")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Parametric neck-hooking fingertip for a Robotiq 2F-85.")
    ap.add_argument("--neck-d", type=float, help="neck outside diameter, mm")
    ap.add_argument("--collar-d", type=float, help="collar outside diameter, mm")
    ap.add_argument("--out", type=Path, default=Path("./build"),
                    help="output directory (default ./build)")
    ap.add_argument("--mount-check", action="store_true",
                    help="export only the mounting plate, as a quick test "
                         "print to check the bolt pattern lines up")
    ap.add_argument("--no-countersink", action="store_true",
                    help="plain through-holes instead of M4 countersinks")
    ap.add_argument("--finish", choices=sorted(FINISHES),
                    help="take neck_d and collar_d from a standard bottle "
                         "finish instead of giving them individually")
    ap.add_argument("--list-finishes", action="store_true",
                    help="show the known finishes and whether their "
                         "dimensions have been filled in")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override any other Params field, e.g. --set "
                         "body_w=42. Repeatable.")
    args = ap.parse_args(argv)

    if args.list_finishes:
        print("bottle finishes (the neck/mouth region, which IS standardised;")
        print("the hook catches the transfer bead just below it):\n")
        for f in FINISHES.values():
            filled = (f"neck {f.neck_d} / bead {f.collar_d} mm"
                      if f.neck_d is not None else
                      "NOT FILLED IN -- add it from the finish drawing")
            print(f"  {f.name:12s} {filled}")
            print(f"  {'':12s} {f.note}")
            if f.source:
                print(f"  {'':12s} source: {f.source}")
            print()
        return 0

    p = PARAMS
    if args.finish:
        f = FINISHES[args.finish]
        if f.neck_d is None or f.collar_d is None:
            print(f"ERROR: finish {f.name!r} has no dimensions filled in yet.\n"
                  f"       {f.note}\n"
                  f"       Add neck_d and collar_d to FINISHES from the finish\n"
                  f"       drawing or a caliper, and record `source`. Nothing is\n"
                  f"       guessed here. Meanwhile pass --neck-d and --collar-d.",
                  file=sys.stderr)
            return 2
        p = replace(p, neck_d=f.neck_d, collar_d=f.collar_d)
    if args.neck_d is not None:
        p = replace(p, neck_d=args.neck_d)
    if args.collar_d is not None:
        p = replace(p, collar_d=args.collar_d)
    if args.no_countersink:
        p = replace(p, countersink=False)
    known = {f.name: f.type for f in fields(Params)}
    for item in args.set:
        name, _, value = item.partition("=")
        name = name.strip()
        if name not in known:
            print(f"ERROR: no such parameter {name!r}. Known: "
                  f"{', '.join(sorted(known))}", file=sys.stderr)
            return 2
        try:
            coerced = (value.strip().lower() in ("1", "true", "yes")
                       if known[name] == "bool" else float(value))
        except ValueError:
            print(f"ERROR: {name}={value!r} is not a number", file=sys.stderr)
            return 2
        p = replace(p, **{name: coerced})

    try:
        warnings = validate(p)
    except ValidationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    for w in warnings:
        print(f"WARNING: {w}\n", file=sys.stderr)

    # build_tip() can also raise ValidationError, for the kernel limits
    # validate() cannot predict, so the build is guarded the same way.
    try:
        if args.mount_check:
            print("mount-check test piece (mounting plate only):")
            export(build_mount_plate(p), "mount_check", args.out)
            print(f"wrote STEP + STL to {args.out}")
            return 0

        print(_summary(p))
        left, right = build_pair(p)
    except ValidationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    export(left, "fingertip_left", args.out, p)
    export(right, "fingertip_right", args.out, p)
    print(f"wrote STEP + STL to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
