import unittest
from classic_bags import Bag
import os
import shutil
import tempfile


class TestBags:
    def __init__(self, *args, **kwargs):
        super(TestBags, self).__init__(*args, **kwargs)
        self.bag_dir = tempfile.mkdtemp(prefix='rosbag_tests')

    def tearDown(self):
        shutil.rmtree(self.bag_dir)

    def _write_simple_bag(self, name):
        from std_msgs.msg import Int32, String

        with Bag(name, 'w', format=self.STORAGE_ID) as bag:
            s = String(data='foo')
            i = Int32(data=42)

            bag.write('chatter', s)
            bag.write('numbers', i)

    def _fname(self, name):
        return os.path.join(self.bag_dir, name)

    def test_value_equality(self):
        fname = self._fname('test_value_equality.bag')

        self._write_simple_bag(fname)

        with Bag(fname, format=self.STORAGE_ID) as bag:
            numbers = list(bag.read_messages('numbers'))
            chatter = list(bag.read_messages('chatter'))

        self.assertEqual(len(numbers), 1)
        self.assertEqual(len(chatter), 1)

        numbers = numbers[0]
        chatter = chatter[0]

        # channel names
        self.assertEqual(numbers[0], 'numbers')
        self.assertEqual(chatter[0], 'chatter')

        # values
        self.assertEqual(numbers[1].data, 42)
        self.assertEqual(chatter[1].data, 'foo')

    def test_type_equality(self):
        fname = self._fname('test_type_equality.bag')

        from std_msgs.msg import Int32, String

        self._write_simple_bag(fname)

        with Bag(fname, format=self.STORAGE_ID) as bag:
            numbers = next(bag.read_messages('numbers'))
            chatter = next(bag.read_messages('chatter'))

        self.assertEqual(numbers[1].__class__, Int32)
        self.assertEqual(chatter[1].__class__, String)

    def test_type_isinstance(self):
        fname = self._fname('test_type_isinstance.bag')

        from std_msgs.msg import Int32, String

        self._write_simple_bag(fname)

        with Bag(fname, format=self.STORAGE_ID) as bag:
            numbers = next(bag.read_messages('numbers'))
            chatter = next(bag.read_messages('chatter'))

        self.assertIsInstance(numbers[1], Int32)
        self.assertIsInstance(chatter[1], String)


class TestSQL(TestBags, unittest.TestCase):
    STORAGE_ID = 'sqlite3'


class TestMCAP(TestBags, unittest.TestCase):
    STORAGE_ID = 'mcap'


if __name__ == '__main__':
    unittest.main()
