"""CPU inference configuration checks; no ROS graph or hardware access."""

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sac_driver.inference_engine import InferenceEngine


class InferenceThreads(unittest.TestCase):
    @patch('sac_driver.inference_engine.load_policy')
    @patch('sac_driver.inference_engine.torch.set_num_threads')
    def test_requested_pool_is_set_before_loading_policy(self, set_threads, load):
        events = []
        set_threads.side_effect = lambda n: events.append(('threads', n))
        def load_fake(*args, **kwargs):
            events.append(('load', None))
            return Mock(parameters=Mock(return_value=iter([Mock(device='cpu')])))
        load.side_effect = load_fake
        InferenceEngine('unused.pth', cpu_threads=1)
        self.assertEqual(events, [('threads', 1), ('load', None)])

    @patch('sac_driver.inference_engine.load_policy')
    @patch('sac_driver.inference_engine.torch.set_num_threads')
    def test_invalid_pool_fails_before_loading(self, set_threads, load):
        with self.assertRaises(ValueError):
            InferenceEngine('unused.pth', cpu_threads=0)
        set_threads.assert_not_called()
        load.assert_not_called()


if __name__ == '__main__':
    unittest.main()
