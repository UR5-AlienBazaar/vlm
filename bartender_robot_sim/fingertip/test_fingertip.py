"""Tests for the fingertip generator.

Two things are covered: that validate() refuses bad geometry loudly instead of
quietly clamping it, and that what comes out of the modeller is a real closed
solid rather than a shell with holes in it.
"""
import math
import struct
from collections import defaultdict
from dataclasses import replace

import pytest

from fingertip import (FINISHES, MOUNT_PLACEHOLDERS, PARAMS,
                       ValidationError, build_mount_plate, build_pair,
                       build_tip, main, validate)


# --------------------------------------------------------------------------
# validation
# --------------------------------------------------------------------------
def test_defaults_are_buildable():
    validate(PARAMS)  # must not raise


def test_defaults_warn_about_placeholder_mount():
    warnings = validate(PARAMS)
    assert any("PLACEHOLDER" in w.upper() for w in warnings), warnings


def test_placeholder_warning_clears_once_measured():
    measured = replace(PARAMS, **{k: v + 1.0
                                  for k, v in MOUNT_PLACEHOLDERS.items()})
    assert measured.mount_is_placeholder() == []
    warnings = validate(measured)
    assert not any("PLACEHOLDER" in w.upper() for w in warnings), warnings


def test_partly_measured_mount_still_warns():
    """One value left at its invented default is still a trap."""
    half = replace(PARAMS, mount_hole_spacing=23.7)
    assert half.mount_is_placeholder()
    assert any("PLACEHOLDER" in w.upper() for w in validate(half))


def test_pocket_radius_must_be_positive():
    bad = replace(PARAMS, neck_d=0.0, clearance=0.0)
    with pytest.raises(ValidationError, match="pocket radius"):
        validate(bad)


def test_collar_must_be_wider_than_neck():
    bad = replace(PARAMS, neck_d=30.0, collar_d=30.0)
    with pytest.raises(ValidationError, match="collar_d"):
        validate(bad)


def test_ledge_must_step_out_far_enough_to_clear_the_collar():
    """The step has to reach past the collar, or the collar never gets above
    the ledge. See the README note on the direction of this check."""
    bad = replace(PARAMS, ledge_depth=0.5)
    with pytest.raises(ValidationError, match="does not clear"):
        validate(bad)


def test_ledge_must_sit_inside_the_pocket():
    with pytest.raises(ValidationError, match="ledge_h"):
        validate(replace(PARAMS, ledge_h=PARAMS.pocket_h + 1.0))


def test_leadin_shallower_than_the_wall_tangent_is_refused():
    """At a 120 deg arc the wall already leaves the entry at 30 deg, so a
    30 deg lead-in cuts nothing at all."""
    bad = replace(PARAMS, pocket_arc=120.0, leadin_angle=30.0)
    with pytest.raises(ValidationError, match="degenerate"):
        validate(bad)
    assert validate(replace(bad, leadin_angle=45.0)) is not None


def test_entry_leadin_wider_than_the_horn_is_refused():
    """The chamfer is cut into the flat beside the collar pocket. Ask for more
    than is there and the kernel throws a message naming no parameter, so it
    is caught before it gets that far."""
    bad = replace(PARAMS, leadin_angle=60.0)
    assert bad.leadin_entry_b > bad.entry_flat_width - 0.1
    with pytest.raises(ValidationError, match="entry lead-in"):
        validate(bad)


def test_thick_tips_that_eat_the_gripper_stroke_are_refused():
    bad = replace(PARAMS, body_t=40.0)
    with pytest.raises(ValidationError, match="opening"):
        validate(bad)


def test_narrow_body_that_truncates_the_ledge_is_refused():
    bad = replace(PARAMS, body_w=30.0)
    with pytest.raises(ValidationError, match="too narrow"):
        validate(bad)


def test_thin_wall_behind_the_pocket_is_refused():
    bad = replace(PARAMS, body_t=11.0)
    with pytest.raises(ValidationError, match="wall"):
        validate(bad)


def test_countersink_that_would_break_through_is_refused():
    bad = replace(PARAMS, csk_head_d=40.0)
    with pytest.raises(ValidationError, match="countersink|csk_head_d"):
        validate(bad)


def test_validation_never_clamps():
    """A refused value must come back out of the dataclass unchanged."""
    bad = replace(PARAMS, ledge_depth=0.5)
    with pytest.raises(ValidationError):
        validate(bad)
    assert bad.ledge_depth == 0.5


def test_error_message_names_the_numbers():
    with pytest.raises(ValidationError) as exc:
        validate(replace(PARAMS, body_w=30.0))
    msg = str(exc.value)
    assert "30" in msg and "at least" in msg


# --------------------------------------------------------------------------
# derived geometry
# --------------------------------------------------------------------------
def test_pocket_subtends_the_requested_arc():
    """The axis is placed so the cut subtends pocket_arc exactly."""
    for arc in (90.0, 120.0, 150.0):
        p = replace(PARAMS, pocket_arc=arc)
        half = math.acos((p.axis_x - p.front_x) / p.pocket_r)
        assert math.degrees(2 * half) == pytest.approx(arc, abs=1e-6)


def test_bearing_width_is_the_collar_overhang_once_the_step_clears():
    p = PARAMS
    assert p.collar_pocket_r >= p.collar_r
    assert p.ledge_bearing_width == pytest.approx(p.collar_overhang, abs=1e-9)


# --------------------------------------------------------------------------
# solids
# --------------------------------------------------------------------------
@pytest.fixture(scope="module")
def tip():
    return build_tip(PARAMS)


def test_solid_has_non_zero_volume(tip):
    assert tip.volume > 0.0


def test_solid_is_valid(tip):
    assert tip.is_valid


def test_solid_fits_the_declared_envelope(tip):
    bb = tip.bounding_box()
    assert bb.size.X == pytest.approx(PARAMS.front_x, abs=1e-6)
    assert bb.size.Y == pytest.approx(PARAMS.body_w, abs=1e-6)


def test_pocket_actually_removed_material(tip):
    solid_block = PARAMS.front_x * PARAMS.body_w * PARAMS.body_h
    assert tip.volume < solid_block


def test_mount_plate_builds_and_is_drilled():
    plate = build_mount_plate(PARAMS)
    assert plate.is_valid and plate.volume > 0.0
    undrilled = (PARAMS.mount_plate_t * PARAMS.mount_face_w
                 * PARAMS.mount_face_h)
    assert plate.volume < undrilled


def test_pair_is_mirrored_not_duplicated():
    left, right = build_pair(PARAMS)
    assert left.is_valid and right.is_valid
    assert left.volume == pytest.approx(right.volume, rel=1e-9)


# --------------------------------------------------------------------------
# exported mesh
# --------------------------------------------------------------------------
def _read_binary_stl(path):
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    tris = []
    for i in range(count):
        base = 84 + i * 50 + 12
        tris.append(tuple(struct.unpack("<fff", data[base + k * 12:
                                                     base + k * 12 + 12])
                          for k in range(3)))
    return tris


def test_exported_stl_is_manifold(tmp_path, tip):
    """Every edge of a closed surface is shared by exactly two triangles."""
    from build123d import export_stl
    path = tmp_path / "tip.stl"
    export_stl(tip, str(path))
    tris = _read_binary_stl(path)
    assert tris, "no triangles exported"

    def key(v):
        return (round(v[0], 4), round(v[1], 4), round(v[2], 4))

    edges = defaultdict(int)
    for tri in tris:
        k = [key(v) for v in tri]
        for a, b in ((k[0], k[1]), (k[1], k[2]), (k[2], k[0])):
            edges[tuple(sorted((a, b)))] += 1

    bad = {e: n for e, n in edges.items() if n != 2}
    assert not bad, f"{len(bad)} non-manifold edges out of {len(edges)}"


def test_exported_stl_has_volume(tmp_path, tip):
    """Signed volume of the mesh, to catch an inside-out or open surface."""
    from build123d import export_stl
    path = tmp_path / "tip.stl"
    export_stl(tip, str(path))
    total = 0.0
    for a, b, c in _read_binary_stl(path):
        total += (a[0] * (b[1] * c[2] - b[2] * c[1])
                  - a[1] * (b[0] * c[2] - b[2] * c[0])
                  + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    assert total > 0.0
    assert total == pytest.approx(tip.volume, rel=0.02)


# --------------------------------------------------------------------------
# robustness: these parameters are meant to be edited, so editing them must
# either produce a part or say why, never a kernel stack trace
# --------------------------------------------------------------------------
@pytest.mark.parametrize("neck", [18.0, 26.0, 30.0, 38.0])
@pytest.mark.parametrize("arc", [90.0, 120.0, 140.0])
@pytest.mark.parametrize("leadin_angle", [45.0, 60.0])
def test_sweep_either_builds_or_refuses_cleanly(neck, arc, leadin_angle):
    p = replace(PARAMS, neck_d=neck, collar_d=neck + 6.0, pocket_arc=arc,
                leadin_angle=leadin_angle)
    # The contract is: either a valid solid, or a ValidationError that says
    # what to change. validate() catches what it can predict up front, and
    # build_tip() raises the same exception type for the kernel limits it
    # cannot. What must never happen is a raw OCCT error reaching the user.
    try:
        validate(p)
        part = build_tip(p)
    except ValidationError as exc:
        assert str(exc).strip()
        return
    assert part.is_valid
    assert part.volume > 0.0


def test_step_round_trips(tmp_path, tip):
    """Re-import the exported STEP and check it is the same solid.

    Catches an export that writes a file a CAD package cannot use, which an
    STL check would not notice.
    """
    from build123d import export_step, import_step
    path = tmp_path / "tip.step"
    export_step(tip, str(path))
    assert path.stat().st_size > 0
    back = import_step(str(path))
    assert back.is_valid
    assert back.volume == pytest.approx(tip.volume, rel=1e-6)
    bb_a, bb_b = tip.bounding_box(), back.bounding_box()
    assert bb_b.size.X == pytest.approx(bb_a.size.X, abs=1e-6)
    assert bb_b.size.Y == pytest.approx(bb_a.size.Y, abs=1e-6)
    assert bb_b.size.Z == pytest.approx(bb_a.size.Z, abs=1e-6)


# --------------------------------------------------------------------------
# bottle finishes
# --------------------------------------------------------------------------
def test_finish_dimensions_are_complete_or_absent():
    """Half a finish is worse than none: it would build a wrong part."""
    for f in FINISHES.values():
        assert (f.neck_d is None) == (f.collar_d is None), f.name


def test_a_filled_finish_records_where_it_came_from():
    for f in FINISHES.values():
        if f.neck_d is not None:
            assert f.source, (
                f"finish {f.name!r} has dimensions but no source; record the "
                f"drawing or standard they came from")


def test_every_finish_says_what_it_is():
    for f in FINISHES.values():
        assert f.note


def test_unfilled_finish_refuses_rather_than_guessing(tmp_path, capsys):
    unfilled = [n for n, f in FINISHES.items() if f.neck_d is None]
    if not unfilled:
        pytest.skip("all finishes have been filled in")
    rc = main(["--finish", unfilled[0], "--out", str(tmp_path)])
    assert rc == 2
    assert "no dimensions filled in" in capsys.readouterr().err
    assert not list(tmp_path.glob("*.step"))
