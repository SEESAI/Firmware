# PX4 v1.17 Build and SITL

> These steps apply to the **v1.17 tree**. For the v1.13 tree, see [build_and_sitl_v1.13.md](build_and_sitl_v1.13.md).

Follow these instructions to
- Run PX4 SITL natively on Linux with minimal external dependencies
- Build PX4 v1.17 for CubeOrange autopilot hardware

Run all commands from the repository root. Before building anything, make sure the submodules are present:

```sh
git submodule update --init --recursive
```

## PX4 SITL with minimal external dependencies

This workflow runs PX4 SITL natively on Linux, using PX4's built-in Software-in-the-Loop (SIH) simulator. SIH runs the vehicle physics inside the PX4 process, so it does not require an external simulator such as Gazebo.

### Host and Python dependencies

Install the native build tools for your distribution: `make`, CMake, Ninja, a C++ compiler, and Python 3. Then create a virtual environment and install PX4's Python requirements:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r Tools/setup/requirements.txt
python -m pip install "setuptools<81"
```

The requirements file constrains EmPy to `>=3.3,<4`. EmPy 4.x is incompatible with PX4's template generators; in particular, configuration can fail with `AttributeError: module 'em' has no attribute 'RAW_OPT'`. Using the requirements file in the active environment avoids accidentally selecting an incompatible system or user-installed EmPy.

The `setuptools<81` pin keeps `pkg_resources` available. Newer setuptools releases remove it, and the DroneCAN DSDL compiler still imports it.

### Build and run SIH

With the virtual environment active:

```sh
make px4_sitl sihsim_quadx
```

This builds and starts PX4 SITL with SIH's quadrotor-X model. The simulator runs headless by default. QGroundControl can connect over UDP port 14550 and show the vehicle on its map.

Other SIH models and their support status are listed in the [SIH documentation](../en/sim_sih/index.md). The quadrotor is the stable model; the other listed vehicle types are experimental.

## Build for CubeOrange

Build NuttX targets inside the PX4 CI container, not with the host toolchain:

- GCC 14 (the `arm-none-eabi-gcc` shipped with recent distributions) fails in Micro-XRCE-DDS-Client, because an implicit `getaddrinfo` declaration is an error in GCC 14.
- The older `px4io/px4-dev-nuttx-focal` image used for v1.13 does not work either: its glibc is too old for the in-tree NuttX host tools.

The CI image has GCC 13.2. Its tag is the `default_container` in [`Tools/ci/generate_board_targets_json.py`](../../Tools/ci/generate_board_targets_json.py); if CI moves to a newer image, use that tag below.

```sh
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -e CI=true \
  -v $PWD:$PWD -w $PWD \
  ghcr.io/px4/px4-dev:v1.16.0-rc1-258-g0369abd556 \
  bash -c 'git config --global --add safe.directory "*"; CMAKE_BUILD_PARALLEL_LEVEL=8 make cubepilot_cubeorange_default'
```

The firmware is written to `build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4`.

- Other variants of the board are `cubepilot_cubeorange_console` and `cubepilot_cubeorange_bootloader`. For CubeOrange+, use `cubepilot_cubeorangeplus_default`.
- `CMAKE_BUILD_PARALLEL_LEVEL` caps the number of build jobs. Raise or lower it to suit your machine's memory, and avoid running several PX4 builds at the same time.

### Flash the CubeOrange

- Disconnect other flight controllers before flashing.
- Connect the autopilot to the host over USB.
- Open QGroundControl's **Vehicle Setup > Firmware**, choose **Advanced settings > Custom firmware file**.
- Select the `.px4` file generated above.
- Flash.
