import stm_control
import unittest


class TestStm(unittest.TestCase):
    """Unit tests for the STM controller class."""

    def setUp(self):
        """Set up the test fixture before every test method.

        This method initializes an instance of the STM controller
        and opens the connection to the device.
        """
        self.stm = stm_control.STM()
        self.stm.open()

    # def test_move_motor(self):
    #     self.stm.move_motor(100)

    def test_get_status(self):
        """Test getting the status from the STM.

        This test calls the get_status() method and prints the result.
        """
        print(self.stm.get_status())

    # def test_iv_curve(self):
    #     print(self.stm.get_iv_curve())
    # def test_scan(self):
    #     self.stm.start_scan()


if __name__ == '__main__':
    unittest.main()
