#!/usr/bin/env python3
"""Generate the real bar's bottles that have no mesh yet, as primitive-shape SDF models.

    python3 make_real_bar_bottles.py [--models ../../../../models]

The real bar serves six drinks: whiskey (Jack Daniel's, models/jack_daniels_bottle),
beer (Heineken, models/beer_bottle), and the four made here: vodka (Zubrowka),
liqueur (Jagermeister), gin (Tenjaku) and wine (Frontera white). Ballantine's
stands on the real bar too, as a distractor, so it is made here as well.

Sizes are the 0.7/0.75 l bottles' published heights and widths, rounded;
colours are read off the real-bar photos (vlm/photos) where the bottle is in
them, and are placeholders for Tenjaku and Frontera, which are not. Replace a
bottle with a mesh when one exists; the segmentation label is set where the
world includes the model, not here.
"""
import argparse
from pathlib import Path

GLASS_SPEC = '<specular>0.6 0.6 0.6 1</specular>'
CLEAR = (0.85, 0.92, 0.90, 0.35)
JAGER_GREEN = (0.05, 0.15, 0.07, 1)
WINE_GREEN = (0.25, 0.40, 0.18, 0.6)
AMBER = (0.25, 0.08, 0.03, 1)

# name: (description, shapes). A shape is (kind, size, z of its centre above the
# base, rgba, collides): kind 'cyl' takes (radius, length), 'box' (x, y, z).
# Labels and caps are visual only: collisions are the body and the neck. A
# visual-only box is a flat label on the body's front (+y) face.
BOTTLES = {
    'zubrowka_bottle': (
        'Zubrowka vodka 0.7 l: round clear bottle, cream label with a bison', [
            ('cyl', (0.037, 0.19), 0.095, CLEAR, True),
            ('cyl', (0.0375, 0.08), 0.10, (0.93, 0.92, 0.84, 1), False),
            ('cyl', (0.025, 0.03), 0.205, CLEAR, True),
            ('cyl', (0.014, 0.06), 0.25, CLEAR, True),
            ('cyl', (0.016, 0.03), 0.295, (0.92, 0.92, 0.92, 1), False),
        ]),
    'jagermeister_bottle': (
        'Jagermeister 0.7 l: square dark green bottle, orange label with a stag', [
            ('box', (0.085, 0.085, 0.19), 0.095, JAGER_GREEN, True),
            ('box', (0.06, 0.004, 0.10), 0.10, (0.88, 0.52, 0.18, 1), False),
            ('box', (0.07, 0.07, 0.03), 0.205, JAGER_GREEN, True),
            ('cyl', (0.016, 0.05), 0.245, JAGER_GREEN, True),
            ('cyl', (0.018, 0.03), 0.285, (0.75, 0.35, 0.10, 1), False),
        ]),
    'tenjaku_bottle': (
        'Tenjaku gin 0.7 l: PLACEHOLDER shape and colours, not in the photos', [
            ('cyl', (0.040, 0.20), 0.10, CLEAR, True),
            ('cyl', (0.0405, 0.07), 0.10, (0.85, 0.80, 0.70, 1), False),
            ('cyl', (0.015, 0.07), 0.235, CLEAR, True),
            ('cyl', (0.017, 0.03), 0.285, (0.10, 0.10, 0.10, 1), False),
        ]),
    'frontera_bottle': (
        'Frontera white wine 0.75 l: PLACEHOLDER colours, Bordeaux shape, not in the photos', [
            ('cyl', (0.037, 0.20), 0.10, WINE_GREEN, True),
            ('cyl', (0.0375, 0.08), 0.09, (0.95, 0.95, 0.92, 1), False),
            ('cyl', (0.025, 0.03), 0.215, WINE_GREEN, True),
            ('cyl', (0.014, 0.08), 0.27, WINE_GREEN, True),
            ('cyl', (0.0145, 0.04), 0.29, (0.85, 0.80, 0.60, 1), False),
        ]),
    'ballantines_bottle': (
        "Ballantine's 0.7 l: flat dark amber bottle, white label (a distractor, not served)", [
            ('box', (0.09, 0.055, 0.19), 0.095, AMBER, True),
            ('box', (0.065, 0.004, 0.09), 0.08, (0.95, 0.94, 0.90, 1), False),
            ('cyl', (0.015, 0.06), 0.22, AMBER, True),
            ('cyl', (0.017, 0.035), 0.2675, (0.90, 0.88, 0.80, 1), False),
        ]),
}
MASS = 1.2  # a full 0.7 l bottle: ~0.5 kg of glass, ~0.7 kg of drink


def _geometry(kind, size):
    if kind == 'cyl':
        return f'<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>'
    return f'<box><size>{size[0]} {size[1]} {size[2]}</size></box>'


def model_sdf(name, shapes):
    body_r = max(size[0] if kind == 'cyl' else max(size[:2]) / 2 for kind, size, *_ in shapes)
    height = max(z + (size[1] if kind == 'cyl' else size[2]) / 2 for kind, size, z, *_ in shapes)
    bodies = [size for kind, size, _, _, collides in shapes if kind == 'box' and collides]
    front = max((size[1] / 2 for size in bodies), default=0.0)
    # A solid cylinder of the bottle's size: close enough for standing and tipping.
    ixx = MASS * (3 * body_r ** 2 + height ** 2) / 12
    izz = MASS * body_r ** 2 / 2
    parts = []
    for k, (kind, size, z, rgba, collides) in enumerate(shapes):
        y = round(front + size[1] / 2, 4) if kind == 'box' and not collides else 0.0
        colour = ' '.join(str(c) for c in rgba)
        dark = ' '.join(str(round(c * 0.3, 3)) for c in rgba[:3])
        alpha = rgba[3]
        transparency = f'<transparency>{round(1 - alpha, 2)}</transparency>' if alpha < 1 else ''
        geometry = f'<geometry>{_geometry(kind, size)}</geometry>'
        parts.append(
            f'      <visual name="v{k}"><pose>0 {y} {z} 0 0 0</pose>{geometry}'
            f'<material><ambient>{dark} 1</ambient><diffuse>{colour}</diffuse>{GLASS_SPEC}'
            f'</material>{transparency}</visual>')
        if collides:
            parts.append(
                f'      <collision name="c{k}"><pose>0 0 {z} 0 0 0</pose>{geometry}'
                '<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>'
                '</collision>')
    inertia = (f'<ixx>{ixx:.6f}</ixx><iyy>{ixx:.6f}</iyy><izz>{izz:.6f}</izz>'
               '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>')
    return ('<?xml version="1.0" ?>\n'
            '<!-- GENERATED by ros2_ws/src/bartender_gazebo/scripts/make_real_bar_bottles.py.\n'
            '     Edit that script and re-run it; edits here are overwritten. -->\n'
            f'<sdf version="1.9">\n  <model name="{name}">\n    <link name="link">\n'
            f'      <inertial><pose>0 0 {round(height / 2, 4)} 0 0 0</pose><mass>{MASS}</mass>'
            f'<inertia>{inertia}</inertia></inertial>\n'
            # Same damping the cola and beer needed: an upright cylinder spins on a flat top.
            '      <velocity_decay><linear>0.05</linear><angular>0.30</angular></velocity_decay>\n'
            + '\n'.join(parts) + '\n    </link>\n  </model>\n</sdf>\n')


def model_config(name, description):
    return (f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
            '  <sdf version="1.9">model.sdf</sdf>\n'
            '  <author><name>Bartender Robot Project</name></author>\n'
            f'  <description>{description}. Generated by make_real_bar_bottles.py.</description>\n'
            '</model>\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--models', type=Path,
                        default=Path(__file__).resolve().parents[4] / 'models',
                        help='models/ directory to write into')
    opts = parser.parse_args()
    for name, (description, shapes) in BOTTLES.items():
        out = opts.models / name
        out.mkdir(parents=True, exist_ok=True)
        (out / 'model.sdf').write_text(model_sdf(name, shapes))
        (out / 'model.config').write_text(model_config(name, description))
        print('wrote', out)


if __name__ == '__main__':
    main()
