#!/usr/bin/env python3
"""Results database for sim captures and VLM training runs, plus a page to browse it.

    python results.py ingest outputs/results.db data/vlm_raw     # after a capture
    python results.py serve  outputs/results.db --port 8081      # http://127.0.0.1:8081

simulation -> scenario -> state is what the sim produced (a capture run, its
scenes, their per-camera frames with ground truth). experiment -> metric and
experiment -> prediction is what training did with it. Stdlib only, so the
capture box can ingest without the training stack installed.
"""
import argparse
import html
import json
import sqlite3
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

SCHEMA = """
CREATE TABLE IF NOT EXISTS simulations (
    id          INTEGER PRIMARY KEY,
    raw_dir     TEXT NOT NULL UNIQUE,   -- capture_vlm_frames.py output directory
    seeds       TEXT NOT NULL,          -- JSON list of capture seeds seen
    ingested_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS scenarios (
    id            INTEGER PRIMARY KEY,
    simulation_id INTEGER NOT NULL REFERENCES simulations(id),
    name          TEXT NOT NULL,        -- scene directory, e.g. s07_00003
    kind          TEXT NOT NULL,        -- static | pour
    params        TEXT,                 -- JSON: shown, yaw, glass, distractor (static)
    outcome       TEXT,                 -- JSON: success, message, phases, upright_after (pour)
    frames        INTEGER,
    seconds       REAL,
    UNIQUE (simulation_id, name)
);
CREATE TABLE IF NOT EXISTS states (
    id          INTEGER PRIMARY KEY,
    scenario_id INTEGER NOT NULL REFERENCES scenarios(id),
    frame       INTEGER NOT NULL,
    camera      TEXT NOT NULL,          -- overhead | stand | wrist
    image       TEXT NOT NULL,          -- path relative to raw_dir
    split       TEXT,                   -- train | val
    bucket      TEXT,
    label       TEXT NOT NULL,          -- JSON ground-truth answer (vlm_labels.scene_label)
    UNIQUE (scenario_id, frame, camera)
);
CREATE TABLE IF NOT EXISTS experiments (
    id          INTEGER PRIMARY KEY,
    name        TEXT NOT NULL,
    stage       TEXT NOT NULL,          -- sft | grpo | eval
    parent_id   INTEGER REFERENCES experiments(id),  -- e.g. the SFT run a GRPO run starts from
    git_sha     TEXT,
    config      TEXT NOT NULL,          -- JSON: model, data, hyperparameters, reward weights
    status      TEXT NOT NULL,          -- running | done | failed
    started_at  REAL NOT NULL,
    finished_at REAL,
    notes       TEXT
);
CREATE TABLE IF NOT EXISTS metrics (
    experiment_id INTEGER NOT NULL REFERENCES experiments(id),
    step          INTEGER NOT NULL,
    key           TEXT NOT NULL,        -- loss, reward, reward_std, decision_acc, brier, ...
    value         REAL NOT NULL,
    PRIMARY KEY (experiment_id, step, key)
);
CREATE TABLE IF NOT EXISTS predictions (
    experiment_id INTEGER NOT NULL REFERENCES experiments(id),
    state_id      INTEGER NOT NULL REFERENCES states(id),
    answer        TEXT NOT NULL,        -- raw model output
    reward        REAL,
    scores        TEXT,                 -- JSON: field_acc, iou, decision, brier
    PRIMARY KEY (experiment_id, state_id)
);
"""


def connect(db):
    Path(db).parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(db)
    con.execute('PRAGMA foreign_keys = ON')
    con.executescript(SCHEMA)
    return con


def ingest(con, raw, pour_stride=1):
    """Record a capture directory's scenes and labelled frames. Re-running updates in place."""
    import numpy as np
    from PIL import Image

    from vlm_labels import blocked, bucket, is_val, scene_label

    raw = Path(raw).resolve()
    seeds = set()
    with con:
        con.execute('INSERT OR IGNORE INTO simulations (raw_dir, seeds, ingested_at) VALUES (?, ?, ?)',
                    (str(raw), '[]', time.time()))
        sim_id = con.execute('SELECT id FROM simulations WHERE raw_dir = ?', (str(raw),)).fetchone()[0]
        for scene in sorted(p for p in raw.iterdir() if (p / 'scene.json').exists()):
            meta = json.loads((scene / 'scene.json').read_text())
            seeds.add(meta.get('seed'))
            con.execute(
                'INSERT INTO scenarios (simulation_id, name, kind, params, outcome, frames, seconds) '
                'VALUES (?, ?, ?, ?, ?, ?, ?) ON CONFLICT (simulation_id, name) DO UPDATE SET '
                'kind = excluded.kind, params = excluded.params, outcome = excluded.outcome, '
                'frames = excluded.frames, seconds = excluded.seconds',
                (sim_id, scene.name, meta.get('kind', 'static'), _json(meta.get('params')),
                 _json(meta.get('pour')), meta.get('frames'), meta.get('seconds')))
            scenario_id = con.execute('SELECT id FROM scenarios WHERE simulation_id = ? AND name = ?',
                                      (sim_id, scene.name)).fetchone()[0]
            stride = pour_stride if meta.get('kind') == 'pour' else 1
            for labels_png in sorted(scene.glob('*_labels.png')):
                frame, camera = labels_png.name.split('_')[:2]
                if int(frame) % stride:
                    continue
                in_gripper = json.loads((scene / f'{frame}.json').read_text())['in_gripper']
                labels = np.asarray(Image.open(labels_png))
                label = scene_label(labels, meta['bottles'], meta['glasses'], in_gripper,
                                    blocked(meta.get('params') or {}))
                rgb = labels_png.with_name(labels_png.name.replace('_labels', '_rgb'))
                con.execute(
                    'INSERT OR REPLACE INTO states (scenario_id, frame, camera, image, split, bucket, label) '
                    'VALUES (?, ?, ?, ?, ?, ?, ?)',
                    (scenario_id, int(frame), camera, rgb.relative_to(raw).as_posix(),
                     'val' if is_val(scene.name) else 'train', bucket(meta, label, labels), json.dumps(label)))
        con.execute('UPDATE simulations SET seeds = ?, ingested_at = ? WHERE id = ?',
                    (json.dumps(sorted(s for s in seeds if s is not None)), time.time(), sim_id))
    return sim_id


def _json(value):
    return None if value is None else json.dumps(value)


def git_sha():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None  # not a checkout, e.g. rsynced to Brev without .git


def start_experiment(con, name, stage, config, parent_id=None, notes=None):
    with con:
        cur = con.execute(
            'INSERT INTO experiments (name, stage, parent_id, git_sha, config, status, started_at, notes) '
            "VALUES (?, ?, ?, ?, ?, 'running', ?, ?)",
            (name, stage, parent_id, git_sha(), json.dumps(config), time.time(), notes))
    return cur.lastrowid


def finish_experiment(con, experiment_id, status='done'):
    with con:
        con.execute('UPDATE experiments SET status = ?, finished_at = ? WHERE id = ?',
                    (status, time.time(), experiment_id))


def log_metrics(con, experiment_id, step, values):
    with con:
        con.executemany('INSERT OR REPLACE INTO metrics VALUES (?, ?, ?, ?)',
                        [(experiment_id, step, k, float(v)) for k, v in values.items()
                         if isinstance(v, (int, float))])


def log_prediction(con, experiment_id, state_id, answer, reward=None, scores=None):
    with con:
        con.execute('INSERT OR REPLACE INTO predictions VALUES (?, ?, ?, ?, ?)',
                    (experiment_id, state_id, answer, reward, _json(scores)))


def _table(rows, headers, link=None):
    head = ''.join(f'<th>{html.escape(h)}</th>' for h in headers)
    body = ''
    for row in rows:
        cells = [html.escape('' if v is None else str(v)) for v in row]
        if link:
            cells[0] = f'<a href="{link}{cells[0]}">{cells[0]}</a>'
        body += '<tr>' + ''.join(f'<td>{c}</td>' for c in cells) + '</tr>'
    return f'<table><tr>{head}</tr>{body}</table>'


def _chart(points, title):
    """One metric as an inline SVG line."""
    w, h, pad = 420, 160, 30
    steps = [p[0] for p in points]
    values = [p[1] for p in points]
    lo, hi = min(values), max(values)
    x0, x1 = min(steps), max(steps)

    def xy(step, value):
        x = pad + (w - 2 * pad) * ((step - x0) / (x1 - x0) if x1 > x0 else 0.5)
        y = h - pad - (h - 2 * pad) * ((value - lo) / (hi - lo) if hi > lo else 0.5)
        return f'{x:.1f},{y:.1f}'

    line = ' '.join(xy(s, v) for s, v in points)
    return (f'<figure><figcaption>{html.escape(title)} (last {values[-1]:.4g})</figcaption>'
            f'<svg width="{w}" height="{h}"><polyline fill="none" stroke="currentColor" '
            f'stroke-width="1.5" points="{line}"/>'
            f'<text x="2" y="{pad}">{hi:.3g}</text><text x="2" y="{h - pad}">{lo:.3g}</text>'
            f'<text x="{pad}" y="{h - 8}">step {x0}</text><text x="{w - pad - 50}" y="{h - 8}">{x1}</text>'
            '</svg></figure>')


STYLE = ('<style>body{font-family:sans-serif;margin:16px;background:#1d1d1f;color:#ddd}'
         'a{color:#8cf}table{border-collapse:collapse;margin:8px 0 24px}'
         'td,th{border:1px solid #444;padding:4px 8px;font-size:13px;text-align:left}'
         'figure{display:inline-block;margin:8px}svg text{fill:#999;font-size:10px}</style>')


def page_index(con):
    experiments = con.execute(
        "SELECT id, name, stage, parent_id, git_sha, status, datetime(started_at, 'unixepoch'), "
        "(SELECT COUNT(DISTINCT step) FROM metrics m WHERE m.experiment_id = e.id) "
        'FROM experiments e ORDER BY id DESC').fetchall()
    simulations = con.execute(
        "SELECT s.id, raw_dir, seeds, datetime(ingested_at, 'unixepoch'), "
        '(SELECT COUNT(*) FROM scenarios c WHERE c.simulation_id = s.id), '
        '(SELECT COUNT(*) FROM states t JOIN scenarios c ON t.scenario_id = c.id WHERE c.simulation_id = s.id) '
        'FROM simulations s ORDER BY s.id DESC').fetchall()
    return ('<h1>VLM RL results</h1><h2>Experiments</h2>'
            + _table(experiments, ['id', 'name', 'stage', 'parent', 'git', 'status', 'started (UTC)', 'steps'],
                     '/experiment?id=')
            + '<h2>Simulations</h2>'
            + _table(simulations, ['id', 'raw dir', 'seeds', 'ingested (UTC)', 'scenarios', 'states'],
                     '/simulation?id='))


def page_experiment(con, experiment_id):
    exp = con.execute('SELECT name, stage, status, config, notes FROM experiments WHERE id = ?',
                      (experiment_id,)).fetchone()
    if not exp:
        return '<p>No such experiment.</p>'
    series = {}
    for key, step, value in con.execute(
            'SELECT key, step, value FROM metrics WHERE experiment_id = ? ORDER BY key, step', (experiment_id,)):
        series.setdefault(key, []).append((step, value))
    by_bucket = con.execute(
        'SELECT t.bucket, COUNT(*), AVG(p.reward) FROM predictions p JOIN states t ON p.state_id = t.id '
        'WHERE p.experiment_id = ? GROUP BY t.bucket ORDER BY t.bucket', (experiment_id,)).fetchall()
    name, stage, status, config, notes = exp
    return (f'<p><a href="/">all runs</a></p><h1>{html.escape(name)}</h1>'
            f'<p>{html.escape(stage)} · {html.escape(status)} · {html.escape(notes or "")}</p>'
            + ''.join(_chart(points, key) for key, points in series.items())
            + '<h2>Eval by bucket</h2>'
            + _table([(b, n, None if r is None else round(r, 3)) for b, n, r in by_bucket],
                     ['bucket', 'frames', 'mean reward'])
            + f'<h2>Config</h2><pre>{html.escape(json.dumps(json.loads(config), indent=1))}</pre>')


def page_simulation(con, simulation_id):
    rows = con.execute(
        'SELECT c.name, c.kind, c.frames, c.seconds, c.params, c.outcome, COUNT(t.id) '
        'FROM scenarios c LEFT JOIN states t ON t.scenario_id = c.id WHERE c.simulation_id = ? '
        'GROUP BY c.id ORDER BY c.name', (simulation_id,)).fetchall()
    return ('<p><a href="/">all runs</a></p><h1>Simulation ' + html.escape(str(simulation_id)) + '</h1>'
            + _table(rows, ['scenario', 'kind', 'frames', 'seconds', 'params', 'outcome', 'states']))


def serve(db, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            url = urlparse(self.path)
            ident = parse_qs(url.query).get('id', ['0'])[0]
            con = connect(db)
            try:
                if url.path == '/experiment':
                    body = page_experiment(con, ident)
                elif url.path == '/simulation':
                    body = page_simulation(con, ident)
                else:
                    body = page_index(con)
            finally:
                con.close()
            data = f'<!doctype html><meta charset="utf-8"><title>VLM RL results</title>{STYLE}{body}'.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(data)

    print(f'serving {db} on http://127.0.0.1:{port}')
    ThreadingHTTPServer(('127.0.0.1', port), Handler).serve_forever()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('ingest', help='record a capture directory')
    p.add_argument('db')
    p.add_argument('raw')
    p.add_argument('--pour-stride', type=int, default=20)
    p = sub.add_parser('serve', help='browse the database')
    p.add_argument('db')
    p.add_argument('--port', type=int, default=8081)
    opts = parser.parse_args()
    if opts.cmd == 'ingest':
        con = connect(opts.db)
        print('simulation', ingest(con, opts.raw, opts.pour_stride))
    else:
        serve(opts.db, opts.port)


if __name__ == '__main__':
    main()
