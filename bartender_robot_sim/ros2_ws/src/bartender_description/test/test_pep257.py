# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


# ament_pep257's own defaults, plus D213.
#
# The list has to be spelled out because --ignore REPLACES ament's default set
# rather than extending it; passing just D213 silently turns D100-D107, D203,
# D212 and D404 back ON, which is how this was first written and why it still
# failed.
#
# D213 ("multi-line docstring summary should start at the second line") is the
# one addition, and it is excluded because it is the opposite of the
# convention every other file in this repo follows -- pour_action_server,
# render_bartender_urdf and the fingertip generator all open on line one.
# D212 and D213 are a mutually exclusive pair, and ament already ignores D212,
# so as shipped it silently requires the style the repo does not use.
#
# Everything else still applies: D205, D209, D400, D401, D403 and D415 all
# caught real problems in this package and were fixed rather than excluded.
IGNORE = [
    'D100', 'D101', 'D102', 'D103', 'D104', 'D105', 'D106', 'D107',
    'D203', 'D212', 'D404',
    'D213',
]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--ignore'] + IGNORE)
    assert rc == 0, 'Found code style errors / warnings'
