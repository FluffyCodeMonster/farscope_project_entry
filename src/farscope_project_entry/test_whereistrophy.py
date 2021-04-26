import unittest
from farscope_robot_utils import where_is_this_trophy

class TestWhereIsTrophy(unittest.TestCase):

    # Tests that the location is correctly determined for a trophy on shelf 4
    def test_shelf4_0(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((1.0, -4.0, 0.5))
        self.assertEqual(shelf_id, 4)
        self.assertEqual(level_id, 1)
        self.assertAlmostEqual(offset_pc, 0, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 4
    def test_shelf4_1(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((1.2, -4.15, 0.5))
        self.assertEqual(shelf_id, 4)
        self.assertEqual(level_id, 1)
        self.assertAlmostEqual(offset_pc, -0.4545454545454544, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 4
    def test_shelf4_2(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((0.8, -4.15, 0.5))
        self.assertEqual(shelf_id, 4)
        self.assertEqual(level_id, 1)
        self.assertAlmostEqual(offset_pc, 0.4545454545454544, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 2
    def test_shelf2_0(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((2.0, -1.5, 0.8))
        self.assertEqual(shelf_id, 2)
        self.assertEqual(level_id, 2)
        self.assertAlmostEqual(offset_pc, 0.0, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 2
    def test_shelf2_1(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((2.0, -1.7, 0.8))
        self.assertEqual(shelf_id, 2)
        self.assertEqual(level_id, 2)
        self.assertAlmostEqual(offset_pc, 0.4545454545454544, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 2
    def test_shelf2_2(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((2.0, -1.3, 0.8))
        self.assertEqual(shelf_id, 2)
        self.assertEqual(level_id, 2)
        self.assertAlmostEqual(offset_pc, -0.4545454545454544, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 7
    def test_shelf7_0(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((-3.0, -3.0, 0.1))
        self.assertEqual(shelf_id, 7)
        self.assertEqual(level_id, 0)
        self.assertAlmostEqual(offset_pc, 0.0, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 7
    def test_shelf7_1(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((-3.0, -3.2, 0.1))
        self.assertEqual(shelf_id, 7)
        self.assertEqual(level_id, 0)
        self.assertAlmostEqual(offset_pc, -0.4545454545454544, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 7
    def test_shelf7_2(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((-3.0, -2.8, 0.1))
        self.assertEqual(shelf_id, 7)
        self.assertEqual(level_id, 0)
        self.assertAlmostEqual(offset_pc, 0.4545454545454544, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 8
    def test_shelf8_0(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((-2.0, -1.0, 1.2))
        self.assertEqual(shelf_id, 8)
        self.assertEqual(level_id, 3)
        self.assertAlmostEqual(offset_pc, 0.0, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 8
    def test_shelf8_1(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((-2.2, -1.0, 1.2))
        self.assertEqual(shelf_id, 8)
        self.assertEqual(level_id, 3)
        self.assertAlmostEqual(offset_pc, -0.4545454545454544, places=7, msg=None, delta=None)

    # Tests that the location is correctly determined for a trophy on shelf 8
    def test_shelf8_2(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((-1.8, -1.0, 1.2))
        self.assertEqual(shelf_id, 8)
        self.assertEqual(level_id, 3)
        self.assertAlmostEqual(offset_pc, 0.4545454545454544, places=7, msg=None, delta=None)

    # Tests that a location outside of any shelves is detected correctly
    def test_shelf9_0(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((5.0, 5.0, 1.2))
        self.assertEqual(shelf_id, 0)
        self.assertEqual(level_id, 3)
        self.assertAlmostEqual(offset_pc, 0, places=7, msg=None, delta=None)
        
    # Tests that a location outside of any shelves is detected correctly
    def test_shelf9_1(self):
        (shelf_id, level_id, offset_pc, _) = where_is_this_trophy((5.0, 5.0, 0))
        self.assertEqual(shelf_id, 0)
        self.assertEqual(level_id, 0)
        self.assertAlmostEqual(offset_pc, 0, places=7, msg=None, delta=None)

if __name__ == '__main__':
    unittest.main()
