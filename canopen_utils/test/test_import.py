#    Copyright 2026 ros2_canopen contributors
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0

import unittest


class TestImport(unittest.TestCase):
    def test_import_package(self):
        import canopen_utils  # noqa: F401


if __name__ == "__main__":
    unittest.main()
