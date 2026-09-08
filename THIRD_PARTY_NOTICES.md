# Third-party notices

The repository-level MIT license applies only to material whose copyright is
held by this project's contributors. Third-party components retain their own
licenses and attribution requirements.

## Gazebo model assets

The `person_standing`, `table_marble`, and `stop_sign` directories under
`dev/turtlebot3/ros2_ws/src/tb3_frontier_exploration/models/` originate from
the OSRF Gazebo model database and are used as scene/conversion inputs.

- Source: <https://github.com/osrf/gazebo_models>
- License: Creative Commons Attribution 3.0 Unported
- Repository notice: Copyright 2012 Nathan Koenig
- Model authors named by the source metadata include Marina Kollmitz, Ian Chen,
  Cole Biesemeyer, and Nate Koenig.

The source repository's license and attribution must remain in effect for
these assets. See <https://creativecommons.org/licenses/by/3.0/>.

## OpenAI CLIP

`dev/isaac_sim/vendor/clip-1.0-py3-none-any.whl` is a reproducible wheel built
from OpenAI CLIP commit `d05afc436d78f1c48dc0dbf8e5980a9d471f35f6`.

- Source: <https://github.com/openai/CLIP>
- License: MIT
- Copyright: 2021 OpenAI

The complete MIT license is embedded in the wheel.

## Vendored semantic memory source

`dev/turtlebot3/external/semantic-nav-memory/` records its source as
`https://github.com/ilumn/semantic-nav-memory.git` at commit
`c5ad423425a0ba2e8a2007d8368eaa4825949435`. No license file was present in the
vendored snapshot at publication preparation time. Repository maintainers must
confirm that they have permission to share this snapshot before making the Git
repository public. This notice does not grant additional rights.

## Runtime software and model checkpoints

NVIDIA Isaac Sim, ROS packages, CUDA/NVIDIA drivers, and downloaded model
checkpoints are not relicensed by this repository. Isaac Sim is installed
separately. YOLO and CLIP checkpoints are downloaded locally by
`dev/isaac_sim/fetch_models.sh`, are excluded from Git, and remain subject to
their upstream terms.
