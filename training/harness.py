#!/usr/bin/env python3
"""Self-improving eval harness for the workcell scene VLM: prompt candidates evolve against sim scenes.

    python harness.py run --dev data/workcell_raw/dev --test data/workcell_raw/test \\
        --pool data/workcell_raw/pool --generations 5 --endpoint http://127.0.0.1:8092
    python harness.py serve --port 8093

Each generation mutates the best candidate (a reflective prompt rewrite from
its failures, and a few-shot example from its weakest bucket), scores every
candidate on dev, keeps the best, reports it on test, logs everything to
results.db and JOURNAL.md, and writes a scene-params request that steers the
sim capture (capture_workcell_scenes.py --params) toward what the model gets wrong.
Selection only ever sees dev; test is reported, never selected on.
"""
import argparse
import base64
import html
import json
import random
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import requests

from results import (_table, STYLE, connect, finish_experiment, log_metrics, log_prediction,
                     page_experiment, start_experiment)
from workcell import FIELDS, gold_answer, read_answer, scene_truth, score

SEED_PROMPT = (
    'You are the eyes of a UR5 robot bartender looking at its work table. The table may hold six '
    'bottles: whiskey (a square Jack Daniel\'s bottle with a black label), vodka (a round clear '
    'Zubrowka bottle), liqueur (a square dark green Jagermeister bottle), beer (a green Heineken '
    'bottle with a cap), gin (a Tenjaku bottle) and wine (a Frontera white wine bottle). Any other '
    'object, including a Ballantine\'s bottle, a box, a can or a thermos, is a distractor, not one of these.\n'
    'The order is: pick {order}.\n'
    'Answer with JSON only, listing all six bottles:\n'
    '{"bottles": [{"name": "whiskey|vodka|liqueur|beer|gin|wine", "visible": true|false, "upright": true|false|null, '
    '"blocked": true|false|null, "bbox": [x0, y0, x1, y1] or null}], '
    '"next_action": {"action": "pick|clear_path|report_fallen|not_found", "target": "{order}"}}\n'
    'bbox is in 0-1000 image coordinates. upright is false when a bottle lies on its side. blocked is '
    'true when another object stands between the robot arm and the bottle. next_action: not_found if '
    'the ordered bottle is not visible, report_fallen if it is lying down, clear_path if it is blocked, '
    'otherwise pick.')
REWRITE_TEMPERATURE = 0.8
FAILURES_SHOWN = 8


def load_scenes(root):
    """Truth for every finished scene under root (scene.json is written last)."""
    return [dict(scene_truth(p.parent), image=str(p.parent / '0000_cam_rgb.png'))
            for p in sorted(Path(root).glob('*/scene.json'))]


def _image_part(path):
    data = base64.b64encode(Path(path).read_bytes()).decode()
    return {'type': 'image_url', 'image_url': {'url': f'data:image/png;base64,{data}'}}


def build_messages(candidate, item, pool):
    messages = []
    for name in candidate['fewshot']:
        shot = pool[name]
        messages += [
            {'role': 'user', 'content': [_image_part(shot['image']),
                                         {'type': 'text', 'text': candidate['prompt'].replace('{order}', shot['order'])}]},
            {'role': 'assistant', 'content': json.dumps(gold_answer(shot))},
        ]
    messages.append({'role': 'user', 'content': [
        _image_part(item['image']), {'type': 'text', 'text': candidate['prompt'].replace('{order}', item['order'])}]})
    return messages


class Model:
    """An OpenAI-compatible chat endpoint (vLLM serving Qwen3-VL)."""

    def __init__(self, endpoint, name, workers=16):
        self.url = endpoint.rstrip('/') + '/v1/chat/completions'
        self.name = name
        self.workers = workers

    def chat(self, messages, temperature=0.0, max_tokens=600):
        response = requests.post(self.url, timeout=300, json={
            'model': self.name, 'messages': messages, 'temperature': temperature, 'max_tokens': max_tokens})
        response.raise_for_status()
        return response.json()['choices'][0]['message']['content']

    def answer_all(self, candidate, items, pool):
        with ThreadPoolExecutor(self.workers) as ex:
            return list(ex.map(lambda item: self.chat(build_messages(candidate, item, pool)), items))


def summarize(items, answers):
    """Overall and per-bucket mean scores; returns (metrics, per-item scores)."""
    scores = [score(a, t) for a, t in zip(answers, items)]
    keys = ('total', 'valid', *FIELDS)
    metrics = {k: sum(s[k] for s in scores) / len(scores) for k in keys}
    groups = {}
    for item, s in zip(items, scores):
        for dim, value in item['buckets'].items():
            groups.setdefault(f'{dim}={value}', []).append(s)
    for name, group in groups.items():
        metrics[f'bucket/{name}/total'] = sum(s['total'] for s in group) / len(group)
        metrics[f'bucket/{name}/action'] = sum(s['action'] for s in group) / len(group)
        metrics[f'bucket/{name}/n'] = len(group)
    return metrics, scores


def weakest_buckets(metrics, k=3, min_n=2):
    buckets = [(v, name.split('/')[1]) for name, v in metrics.items()
               if name.startswith('bucket/') and name.endswith('/total')
               and metrics[name.replace('/total', '/n')] >= min_n]
    return [name for _, name in sorted(buckets)[:k]]


def better(child, parent):
    """Whether a candidate's metrics beat another's: the next action first, the total only
    as a tie-break. Selecting on the total alone kept a prompt that saw the bottles better
    but chose the action worse (dev total 0.53 -> 0.65 while action fell 0.61 -> 0.49)."""
    return (child['action'], child['total']) > (parent['action'], parent['total'])


def failure_report(items, answers, scores):
    worst = sorted(zip(scores, items, answers), key=lambda x: x[0]['total'])[:FAILURES_SHOWN]
    lines = []
    for s, item, answer in worst:
        truth = {n: {k: b[k] for k in ('visible', 'upright', 'blocked')} for n, b in item['bottles'].items()}
        lines.append(f"- order pick {item['order']}: right action {item['next_action']['action']}, truth {json.dumps(truth)}; "
                     f"model answered {answer.strip()[:400]!r}; scores {json.dumps({k: round(v, 2) for k, v in s.items()})}")
    return '\n'.join(lines)


def reflect(model, candidate, report):
    """Ask the model to rewrite the prompt so the listed failures would not recur."""
    text = model.chat([{'role': 'user', 'content': (
        'You improve instructions for a vision model that looks at a robot work table and must answer '
        'in a fixed JSON format. Here is the current instruction:\n<prompt>\n' + candidate['prompt'] +
        '\n</prompt>\nHere are cases it got wrong (truth vs what the model answered):\n' + report +
        '\n\nWrite an improved instruction that fixes these mistakes: add concrete visual cues and '
        'decision rules. Keep the same JSON keys and values, keep the literal placeholder {order}, and '
        'keep it under 350 words. Reply with the new instruction between <prompt> and </prompt> only.')}],
        temperature=REWRITE_TEMPERATURE, max_tokens=900)
    start, end = text.find('<prompt>'), text.rfind('</prompt>')
    new = text[start + len('<prompt>'):end].strip() if 0 <= start < end else ''
    return new if '{order}' in new and 'next_action' in new and 'bottles' in new else None


def add_fewshot(candidate, pool, weakest, rng):
    """A few-shot example from the pool that falls in one of the weakest buckets."""
    taken = set(candidate['fewshot'])
    matches = [name for name, t in pool.items() if name not in taken
               and any(f'{d}={v}' in weakest for d, v in t['buckets'].items())]
    return candidate['fewshot'] + [rng.choice(matches)] if matches else None


def scene_request(metrics, weakest):
    """capture_workcell_scenes.py --params overrides aimed at the weakest buckets."""
    params = {}
    if metrics['upright'] < 0.8 or 'action=report_fallen' in weakest:
        params['fallen_p'] = 0.25
    if metrics['blocked'] < 0.8 or 'action=clear_path' in weakest:
        params['block_p'] = 0.7
        params['distractors'] = [0.2, 0.4, 0.4]
    cameras = [w.split('=')[1] for w in weakest if w.startswith('camera=')]
    if cameras:
        params['camera'] = {c: (0.5 if c in cameras else 0.5 / 3) for c in ('front', 'side', 'overhead', 'corner')}
    return params


def ingest_scenes(con, root, items):
    """Record the scenes as simulation -> scenario -> state rows so predictions can point at them."""
    root = str(Path(root).resolve())
    with con:
        con.execute('INSERT OR IGNORE INTO simulations (raw_dir, seeds, ingested_at) VALUES (?, ?, ?)',
                    (root, '[]', time.time()))
        sim = con.execute('SELECT id FROM simulations WHERE raw_dir = ?', (root,)).fetchone()[0]
        ids = {}
        for item in items:
            con.execute('INSERT OR IGNORE INTO scenarios (simulation_id, name, kind, params, frames) '
                        "VALUES (?, ?, 'workcell', ?, 1)", (sim, item['scene'], json.dumps(item['buckets'])))
            scenario = con.execute('SELECT id FROM scenarios WHERE simulation_id = ? AND name = ?',
                                   (sim, item['scene'])).fetchone()[0]
            label = {k: item[k] for k in ('order', 'bottles', 'next_action')}
            con.execute('INSERT INTO states (scenario_id, frame, camera, image, split, bucket, label) '
                        "VALUES (?, 0, 'cam', ?, ?, ?, ?) ON CONFLICT (scenario_id, frame, camera) DO UPDATE SET "
                        'label = excluded.label, bucket = excluded.bucket',
                        (scenario, item['image'], Path(root).name, json.dumps(item['buckets']), json.dumps(label)))
            ids[item['scene']] = con.execute('SELECT id FROM states WHERE scenario_id = ?', (scenario,)).fetchone()[0]
    return ids


def evaluate(con, model, candidate, split, items, pool, state_ids, generation):
    answers = model.answer_all(candidate, items, pool)
    metrics, scores = summarize(items, answers)
    exp = start_experiment(con, f"g{generation}-{candidate['id']}-{split}", 'harness',
                           {**{k: candidate[k] for k in ('id', 'prompt', 'fewshot', 'origin')},
                            'model': model.name, 'split': split, 'generation': generation},
                           parent_id=candidate.get('experiment'))
    log_metrics(con, exp, generation, metrics)
    for item, answer, s in zip(items, answers, scores):
        log_prediction(con, exp, state_ids[item['scene']], answer, s['total'], s)
    finish_experiment(con, exp)
    return exp, metrics, answers, scores


def journal(out, generation, lines):
    out.mkdir(parents=True, exist_ok=True)
    entry = f'\n## Generation {generation} ({time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime())})\n\n' + '\n'.join(lines) + '\n'
    with open(out / 'JOURNAL.md', 'a') as f:
        f.write(entry)


def run(opts):
    rng = random.Random(opts.seed)
    model = Model(opts.endpoint, opts.model, opts.workers)
    out = Path(opts.out)
    con = connect(out / 'results.db')
    dev, test = load_scenes(opts.dev), load_scenes(opts.test)
    pool = {t['scene']: t for t in load_scenes(opts.pool)} if opts.pool else {}
    state_ids = {**ingest_scenes(con, opts.dev, dev), **ingest_scenes(con, opts.test, test)}
    best = {'id': 'c0', 'prompt': SEED_PROMPT, 'fewshot': [], 'origin': 'seed'}
    best['experiment'], best_metrics, answers, scores = evaluate(con, model, best, 'dev', dev, pool, state_ids, 0)
    journal(out, 0, [f"Seed prompt on dev ({len(dev)} scenes): total {best_metrics['total']:.3f}, "
                     f"action {best_metrics['action']:.3f}."])
    for generation in range(1, opts.generations + 1):
        weakest = weakest_buckets(best_metrics)
        report = failure_report(dev, answers, scores)
        children = []
        for i in range(opts.rewrites):
            prompt = reflect(model, best, report)
            if prompt:
                children.append({'id': f'c{generation}.{i}', 'prompt': prompt, 'fewshot': best['fewshot'],
                                 'origin': f"reflect from {best['id']}", 'experiment': best['experiment']})
        shots = add_fewshot(best, pool, weakest, rng) if len(best['fewshot']) < opts.max_shots else None
        if shots:
            children.append({'id': f'c{generation}.shot', 'prompt': best['prompt'], 'fewshot': shots,
                             'origin': f"few-shot for {', '.join(weakest)} from {best['id']}",
                             'experiment': best['experiment']})
        lines = [f"Parent {best['id']} dev total {best_metrics['total']:.3f}; weakest buckets: {', '.join(weakest) or 'none'}."]
        winner = None
        for child in children:
            child['experiment'], metrics, c_answers, c_scores = evaluate(
                con, model, child, 'dev', dev, pool, state_ids, generation)
            lines.append(f"- {child['id']} ({child['origin']}): dev total {metrics['total']:.3f}, action {metrics['action']:.3f}")
            if better(metrics, best_metrics) and (winner is None or better(metrics, winner[1])):
                winner = (child, metrics, c_answers, c_scores)
        if winner:
            best, best_metrics, answers, scores = winner
            lines.append(f"Selected {best['id']}.")
        else:
            lines.append(f"No child beat {best['id']}; it stays.")
        _, test_metrics, _, _ = evaluate(con, model, best, 'test', test, pool, state_ids, generation)
        lines.append(f"Best on test ({len(test)} scenes): total {test_metrics['total']:.3f}, action {test_metrics['action']:.3f}, "
                     f"visible {test_metrics['visible']:.3f}, upright {test_metrics['upright']:.3f}, "
                     f"blocked {test_metrics['blocked']:.3f}, iou {test_metrics['iou']:.3f}.")
        request = scene_request(best_metrics, weakest_buckets(best_metrics))
        if request:
            path = out / 'requests' / f'gen_{generation:03d}.json'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(request, indent=1))
            lines.append(f"Scene request for the sim capture: `{path}` {json.dumps(request)}")
        journal(out, generation, lines)
        (out / 'best_prompt.txt').write_text(best['prompt'])
        print('\n'.join(lines), flush=True)


def page_harness(con, out):
    rows = con.execute(
        "SELECT e.id, e.name, e.parent_id, json_extract(e.config, '$.origin'), "
        "(SELECT round(value, 3) FROM metrics m WHERE m.experiment_id = e.id AND m.key = 'total'), "
        "(SELECT round(value, 3) FROM metrics m WHERE m.experiment_id = e.id AND m.key = 'action'), "
        "datetime(e.started_at, 'unixepoch') FROM experiments e WHERE e.stage = 'harness' ORDER BY e.id DESC").fetchall()
    journal_path = Path(out) / 'JOURNAL.md'
    prompt_path = Path(out) / 'best_prompt.txt'
    return ('<h1>VLM harness</h1><h2>Candidates (newest first)</h2>'
            + _table(rows, ['id', 'name', 'parent', 'origin', 'total', 'action', 'started (UTC)'], '/experiment?id=')
            + '<h2>Best prompt</h2><pre style="white-space:pre-wrap">'
            + html.escape(prompt_path.read_text() if prompt_path.exists() else SEED_PROMPT) + '</pre>'
            + '<h2>Journal</h2><pre style="white-space:pre-wrap">'
            + html.escape(journal_path.read_text() if journal_path.exists() else 'No generations yet.') + '</pre>')


def serve(out, port):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from urllib.parse import parse_qs, urlparse

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            url = urlparse(self.path)
            con = connect(Path(out) / 'results.db')
            try:
                if url.path == '/experiment':
                    body = page_experiment(con, parse_qs(url.query).get('id', ['0'])[0])
                else:
                    body = page_harness(con, out)
            finally:
                con.close()
            data = f'<!doctype html><meta charset="utf-8"><title>VLM harness</title>{STYLE}{body}'.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(data)

    print(f'serving {out} on http://127.0.0.1:{port}')
    ThreadingHTTPServer(('127.0.0.1', port), Handler).serve_forever()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('run')
    p.add_argument('--dev', required=True)
    p.add_argument('--test', required=True)
    p.add_argument('--pool', help='scenes few-shot examples may be drawn from (never dev or test)')
    p.add_argument('--generations', type=int, default=5)
    p.add_argument('--rewrites', type=int, default=2)
    p.add_argument('--max-shots', type=int, default=3)
    p.add_argument('--endpoint', default='http://127.0.0.1:8092')
    p.add_argument('--model', default='qwen3-vl-4b')
    p.add_argument('--workers', type=int, default=16)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--out', default='outputs/harness')
    p = sub.add_parser('serve')
    p.add_argument('--out', default='outputs/harness')
    p.add_argument('--port', type=int, default=8093)
    opts = parser.parse_args()
    if opts.cmd == 'run':
        run(opts)
    else:
        serve(opts.out, opts.port)


if __name__ == '__main__':
    main()
