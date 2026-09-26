import json
from types import SimpleNamespace

import harness
from harness import (SEED_PROMPT, add_fewshot, better, build_messages, load_scenes, reflect, scene_request,
                     summarize, weakest_buckets)
from test_workcell import make_scene
from workcell import gold_answer


class FakeModel:
    """Answers gold for scenes it is told to, a wrong action otherwise; rewrites prompts on request."""

    name = 'fake'

    def __init__(self, right=()):
        self.right = set(right)

    def chat(self, messages, temperature=0.0, max_tokens=600):
        return '<prompt>' + SEED_PROMPT + '\nLook for bottles lying on their side.</prompt>'

    def answer_all(self, candidate, items, pool):
        answers = []
        for item in items:
            gold = gold_answer(item)
            if item['scene'] not in self.right and not candidate['fewshot']:
                gold['next_action']['action'] = 'pick'
            answers.append(json.dumps(gold))
        return answers


def _scenes(root, n, camera='front'):
    for i in range(n):
        make_scene(root, f'w900_{i:05d}', camera=camera)
    return load_scenes(root)


def test_messages_carry_fewshot_turns_and_the_order(tmp_path):
    items = _scenes(tmp_path / 'dev', 2)
    pool = {t['scene']: t for t in items}
    candidate = {'prompt': SEED_PROMPT, 'fewshot': [items[0]['scene']]}
    messages = build_messages(candidate, items[1], pool)
    assert [m['role'] for m in messages] == ['user', 'assistant', 'user']
    assert f"pick {items[1]['order']}" in messages[-1]['content'][1]['text']
    assert messages[-1]['content'][0]['image_url']['url'].startswith('data:image/png;base64,')


def test_summary_finds_the_weak_bucket_and_asks_the_sim_for_it(tmp_path):
    items = _scenes(tmp_path / 'dev', 2) + _scenes(tmp_path / 'side', 2, camera='side')
    answers = [json.dumps(gold_answer(t)) for t in items[:2]] + ['nonsense'] * 2
    metrics, _ = summarize(items, answers)
    assert metrics['bucket/camera=side/total'] == 0.0 and metrics['bucket/camera=front/total'] > 0.9
    weakest = weakest_buckets(metrics)
    assert 'camera=side' in weakest
    assert scene_request(metrics, weakest)['camera']['side'] == 0.5


def test_reflection_keeps_only_prompts_that_keep_the_contract():
    candidate = {'prompt': SEED_PROMPT}
    assert 'lying on their side' in reflect(FakeModel(), candidate, '- a failure')
    broken = SimpleNamespace(chat=lambda *a, **k: '<prompt>just describe the image</prompt>')
    assert reflect(broken, candidate, '- a failure') is None


def test_fewshot_comes_from_a_weak_bucket(tmp_path):
    import random
    pool = {t['scene']: t for t in _scenes(tmp_path / 'pool', 3, camera='side')}
    assert add_fewshot({'fewshot': []}, pool, ['camera=side'], random.Random(0))[0] in pool
    assert add_fewshot({'fewshot': []}, pool, ['camera=front'], random.Random(0)) is None


def test_selection_puts_the_action_before_the_total():
    parent = {'action': 0.6, 'total': 0.5}
    assert not better({'action': 0.5, 'total': 0.9}, parent)
    assert better({'action': 0.6, 'total': 0.51}, parent)
    assert better({'action': 0.7, 'total': 0.4}, parent)


def test_a_run_selects_on_dev_logs_lineage_and_writes_the_journal(tmp_path, monkeypatch):
    dev = tmp_path / 'dev'
    _scenes(dev, 3)
    pool = tmp_path / 'pool'
    _scenes(pool, 2)
    monkeypatch.setattr(harness, 'Model', lambda *a: FakeModel())
    out = tmp_path / 'out'
    harness.run(SimpleNamespace(seed=0, endpoint='', model='fake', workers=1, out=str(out), dev=str(dev),
                                test=str(dev), pool=str(pool), generations=1, rewrites=1, max_shots=3))
    con = harness.connect(out / 'results.db')
    rows = con.execute("SELECT name, parent_id FROM experiments WHERE stage = 'harness' ORDER BY id").fetchall()
    assert rows[0] == ('g0-c0-dev', None) and all(parent for _, parent in rows[1:])
    assert con.execute('SELECT COUNT(*) FROM predictions').fetchone()[0] == 3 * len(rows)
    journal = (out / 'JOURNAL.md').read_text()
    assert 'Generation 1' in journal and 'Best on test' in journal
    assert 'harness' in harness.page_harness(con, out)
