import unittest

from pika_foot_pedal.debounce import PedalDebouncer


MS = 1_000_000


def test_hold_release_and_repeat():
    switch = PedalDebouncer(30)
    switch.initialize(False, 0)
    assert switch.tick(100 * MS) is None
    switch.event(1, 100 * MS)
    assert switch.tick(129 * MS) is None
    assert switch.tick(130 * MS) == 'press'
    switch.event(2, 140 * MS)
    switch.event(1, 150 * MS)
    assert switch.tick(1000 * MS) is None
    switch.event(0, 1000 * MS)
    assert switch.tick(1030 * MS) == 'release'
    assert switch.tick(1100 * MS) is None


def test_short_bounce_is_ignored():
    switch = PedalDebouncer(30)
    switch.initialize(False, 0)
    switch.event(1, 10 * MS)
    switch.event(0, 20 * MS)
    assert switch.tick(60 * MS) is None
    assert not switch.pressed


def test_boot_held_requires_release_then_fresh_press():
    switch = PedalDebouncer(30)
    switch.initialize(True, 0)
    assert switch.pressed
    assert switch.tick(100 * MS) is None
    switch.event(0, 100 * MS)
    assert switch.tick(130 * MS) is None
    assert switch.armed
    switch.event(1, 140 * MS)
    assert switch.tick(170 * MS) == 'press'


def test_disconnect_disarms_and_clears_level():
    switch = PedalDebouncer(30)
    switch.initialize(False, 0)
    switch.event(1, 10 * MS)
    assert switch.tick(40 * MS) == 'press'
    assert switch.disconnect()
    assert not switch.pressed and not switch.armed
    switch.initialize(True, 50 * MS)
    assert switch.tick(100 * MS) is None


class TestDebounce(unittest.TestCase):
    def test_edges_and_repeat(self):
        test_hold_release_and_repeat()

    def test_bounce(self):
        test_short_bounce_is_ignored()

    def test_boot_held(self):
        test_boot_held_requires_release_then_fresh_press()

    def test_disconnect(self):
        test_disconnect_disarms_and_clears_level()
