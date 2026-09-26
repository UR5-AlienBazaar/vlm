import json

import numpy as np
from PIL import Image

from results import (connect, finish_experiment, ingest, log_metrics, log_prediction, page_experiment,
                     page_index, page_simulation, start_experiment)
from vlm_labels import LABEL_IDS


def _capture(raw):
    labels = np.zeros((100, 200), np.uint8)
    labels[10:60, 20:40] = LABEL_IDS['whiskey']
    for name, kind, frames in (('s07_00000', 'static', 1), ('s07_00001', 'pour', 3)):
        scene = raw / name
        scene.mkdir(parents=True)
        meta = {'bottles': ['whiskey'], 'glasses': ['glass'], 'kind': kind, 'seed': 7, 'frames': frames}
        if kind == 'pour':
            meta['pour'] = {'success': True, 'upright_after': {'whiskey': True}}
        else:
            meta['params'] = {'shown': ['whiskey'], 'distractor': None}
        (scene / 'scene.json').write_text(json.dumps(meta))
        for i in range(frames):
            (scene / f'{i:04d}.json').write_text(json.dumps({'in_gripper': None}))
            Image.fromarray(labels).save(scene / f'{i:04d}_overhead_labels.png')


def test_ingest_is_idempotent_and_pages_show_it(tmp_path):
    raw = tmp_path / 'raw'
    _capture(raw)
    con = connect(tmp_path / 'results.db')

    sim = ingest(con, raw, pour_stride=2)
    assert ingest(con, raw, pour_stride=2) == sim
    assert con.execute('SELECT COUNT(*) FROM scenarios').fetchone()[0] == 2
    assert con.execute('SELECT COUNT(*) FROM states').fetchone()[0] == 1 + 2
    outcome = con.execute("SELECT outcome FROM scenarios WHERE kind = 'pour'").fetchone()[0]
    assert json.loads(outcome)['success']

    exp = start_experiment(con, 'sft-smoke', 'sft', {'lr': 1e-4})
    log_metrics(con, exp, 1, {'loss': 2.0, 'epoch': 0.1, 'note': 'ignored'})
    log_metrics(con, exp, 2, {'loss': 1.5})
    state = con.execute('SELECT id FROM states LIMIT 1').fetchone()[0]
    log_prediction(con, exp, state, '{}', reward=0.8, scores={'iou': 1.0})
    finish_experiment(con, exp)

    assert 'sft-smoke' in page_index(con)
    detail = page_experiment(con, exp)
    assert 'loss (last 1.5)' in detail and '<svg' in detail and 'static/none/whiskey' in detail
    assert 's07_00001' in page_simulation(con, sim)
