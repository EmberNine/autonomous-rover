"""Tests for run_bridge_mutants.py. Run from the repo root: python3 -m pytest tools/

Running the mutants takes ~15 minutes with ROS, so it isn't part of pytest; this only checks
that the mutant list still fits the code: each mutant's text occurs exactly once in its file (so
it plants exactly the bug it names), and its test target exists.
"""
import pytest

import run_bridge_mutants as runner


def test_mutant_ids_are_unique():
    ids = [mutant.id for mutant in runner.MUTANTS]
    assert len(ids) == len(set(ids))


@pytest.mark.parametrize('mutant', runner.MUTANTS, ids=lambda mutant: mutant.id)
def test_mutant_text_occurs_exactly_once(mutant):
    text = (runner.REPOSITORY_ROOT / mutant.path).read_text(encoding='utf-8')
    assert text.count(mutant.old) == 1
    assert mutant.new != mutant.old


@pytest.mark.parametrize('mutant', runner.MUTANTS, ids=lambda mutant: mutant.id)
def test_mutant_has_its_tests_and_they_are_copied(mutant):
    assert (runner.REPOSITORY_ROOT / runner.test_target(mutant)).exists()
    assert any(mutant.path.startswith(part + '/') for part in runner.COPIED)


def test_the_requested_mutants_are_present():
    ids = {mutant.id for mutant in runner.MUTANTS}
    assert {'A1', 'B9', 'D5', 'T1', 'T2', 'T3'} <= ids          # old text dedup, resume gap, fake echo, wall-clock timers


def test_only_failing_tests_kill_a_collection_error_is_broken():
    assert runner.verdict(1) == 'KILLED' and runner.verdict(0) == 'SURVIVED'
    assert {runner.verdict(code) for code in (2, 3, 4, 5)} == {'BROKEN'}


def test_the_shutdown_mutants_are_present_and_the_watcher_ones_use_its_tests():
    ids = {mutant.id: mutant for mutant in runner.MUTANTS}
    assert {f'V{n}' for n in range(1, 16)} <= set(ids)
    assert runner.test_target(ids['V1']) == 'tools/test_battery_watch.py'
    assert runner.test_target(ids['V12']) == runner.BRIDGE_TESTS


def test_the_deadman_mutants_are_present():
    ids = {mutant.id for mutant in runner.MUTANTS}
    assert {f'K{n}' for n in range(1, 12)} <= ids


def test_the_steering_mutants_and_the_control_are_present():
    ids = {mutant.id for mutant in runner.MUTANTS}
    assert {f'R{n}' for n in range(1, 19)} <= ids
    assert runner.CONTROLS == {'X0'} and runner.CONTROLS <= ids


@pytest.mark.parametrize('mutant', [m for m in runner.MUTANTS if m.id in runner.CONTROLS], ids=lambda m: m.id)
def test_a_control_changes_only_a_comment(mutant):
    """Each line of the change is the same up to its '#': the control can't change behaviour."""
    old_lines, new_lines = mutant.old.splitlines(), mutant.new.splitlines()
    assert len(old_lines) == len(new_lines)
    for old, new in zip(old_lines, new_lines):
        assert '#' in old and '#' in new
        assert old.split('#', 1)[0] == new.split('#', 1)[0]
