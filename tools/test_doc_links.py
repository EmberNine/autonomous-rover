"""The README's numbers against their sources: every line link (`path?plain=1#L<n>`) in README.md and
docs/design-decisions.md that cites a number must land on a line that has that number. A doc
edit that shifts the cited lines, or a number changed on one side only, fails here.

- A link whose text has numbers, e.g. [6.6 V for 3 s](firmware/README.md?plain=1#L496): each
  number must be on the target line.
- A source link, e.g. [kalman-filter.md:295](docs/kalman-filter.md?plain=1#L295): the text's line
  number must be the link's, and in a table row every number in the row's other cells must be
  on the target line (the row is the citation). "Week N" is a label, not a cited number.
"""
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DOCS = ['README.md', 'docs/design-decisions.md']
LINK = re.compile(r'\[([^\]]+)\]\(([^)?#\s]+)\?plain=1#L(\d+)\)')
SOURCE_TEXT = re.compile(r'^[\w./-]+\.md:(\d+)$')
NUMBER = re.compile(r'\d+(?:\.\d+)?')


def numbers(text):
    return NUMBER.findall(re.sub(r'Week \d+', '', text))


def has_number(line, number):
    """number appears in line as a whole number: 7.2 is not in 17.25, 1 is not in 0.10."""
    return re.search(rf'(?<![\d.]){re.escape(number)}(?![\d]|\.\d)', line) is not None


def links(doc):
    """(doc line number, doc line, link text, target path, target line number) for each line link."""
    for number, line in enumerate((ROOT / doc).read_text(encoding='utf-8').splitlines(), 1):
        for text, target, linked in LINK.findall(line):
            yield number, line, text, target, int(linked)


def cited_numbers(line, text):
    """The numbers a link cites: its own text's, or, for a source link in a table row, the row's."""
    source = SOURCE_TEXT.match(text)
    if not source:
        return numbers(text)
    if not line.lstrip().startswith('|'):
        return []                                # a source link in prose cites a statement
    row = LINK.sub('', line)
    return [n for cell in row.split('|') for n in numbers(cell)]


CASES = [(doc, *link) for doc in DOCS for link in links(doc)]


def test_the_readme_has_line_links():
    assert sum(1 for case in CASES if case[0] == 'README.md') >= 30


@pytest.mark.parametrize('doc, number, line, text, target, linked', CASES,
                         ids=[f'{case[0]}:{case[1]}:{case[3]}' for case in CASES])
def test_a_cited_number_is_on_the_linked_line(doc, number, line, text, target, linked):
    lines = (ROOT / target).read_text(encoding='utf-8').splitlines()
    assert 1 <= linked <= len(lines), f'{doc}:{number}: {target} has {len(lines)} lines, not {linked}'
    source = SOURCE_TEXT.match(text)
    if source:
        assert int(source.group(1)) == linked, f'{doc}:{number}: [{text}] links line {linked}'
    target_line = lines[linked - 1]
    missing = [n for n in cited_numbers(line, text) if not has_number(target_line, n)]
    assert not missing, (f'{doc}:{number}: [{text}] cites {missing}, not on {target}:{linked}: '
                         f'{target_line.strip()[:160]}')
