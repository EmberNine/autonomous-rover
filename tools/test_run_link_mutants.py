"""Tests for run_link_mutants.py. Run from the repo root: python3 -m pytest tools/

Running the mutants takes minutes, so it isn't part of pytest; this only
checks that the mutant list still fits the code: each mutant's text occurs
exactly once in its file (so it would plant exactly the bug it names), and
each C mutant reaches at least one suite.
"""
import pytest

import run_link_mutants as runner


def test_mutant_ids_are_unique():
    ids = [mutant.id for mutant in runner.MUTANTS]
    assert len(ids) == len(set(ids))


@pytest.mark.parametrize('mutant', runner.MUTANTS, ids=lambda mutant: mutant.id)
def test_mutant_text_occurs_exactly_once(mutant):
    text = (runner.REPOSITORY_ROOT / mutant.path).read_text(encoding='utf-8')
    assert text.count(mutant.old) == 1
    assert mutant.new != mutant.old


@pytest.mark.parametrize('mutant', [m for m in runner.MUTANTS
                                    if m.path.startswith('firmware/') and m.path != 'firmware/Core/Src/main.c'],
                         ids=lambda mutant: mutant.id)
def test_c_mutant_reaches_a_suite(mutant):
    names = runner.suites_for(mutant.path)
    assert names and all(name in runner.SUITES for name in names)


@pytest.mark.parametrize('mutant', [m for m in runner.MUTANTS
                                    if m.path.startswith('tools/') or m.path == 'firmware/Core/Src/main.c'],
                         ids=lambda mutant: mutant.id)
def test_python_mutant_has_a_test_file(mutant):
    assert (runner.REPOSITORY_ROOT / runner.python_tests_for(mutant.path)).is_file()


def test_user_requested_mutants_are_present():
    ids = {mutant.id for mutant in runner.MUTANTS}
    assert {'protocol_arm_honoured_with_speed', 'rx_drive0_rearms_without_arm', 'rx_hello_in_binary_does_not_latch',
            'protocol_round_ties_away_from_zero'} <= ids
