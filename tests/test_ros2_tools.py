import unittest
from tools.ros2_tools import list_topics, launch_turtlebot3_simulation, record_bag

class TestRos2Tools(unittest.TestCase):
    def test_list_topics(self):
        result = list_topics()
        self.assertIsInstance(result, str)

    def test_record_bag(self):
        result = record_bag("/camera")
        self.assertIsInstance(result, str)

if __name__ == "__main__":
    unittest.main()