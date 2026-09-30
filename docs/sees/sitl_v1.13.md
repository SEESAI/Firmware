# PX4 v1.13 SITL

> These steps apply to the **1.13 tree only**. Newer PX4 versions use `Tools/simulation/` and different make targets (eg: `gz_x500`). Refer to official PX4 documentation

## Background

PX4 v1.13 relies on outdated toolchain and components: CMake < 3.5, gcc-11, Gazebo classic. These cannot be set up easily in newer versions of Ubuntu (24.04 +). Follow these instructions to
- Setup a Docker container environment to build PX4 v1.13 on a Linux host
- Run PX4 software-in-the-loop (SITL) with Gazebo classic for physics simulation
- Build PX4 v1.13 for Cubeorange autopilot hardware
- Run PX4 SITL with Airsim for physics simulation and SeesInterface2 for visualisation and control

## Prerequisites

- Install Docker
  ```bash
  sudo apt update
  sudo apt install -y ca-certificates curl gnupg
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt update
  sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

  # Enable and start Docker
  sudo systemctl enable docker
  sudo systemctl start docker

  # Add user to docker group to allow Docker to run as non-root user
  sudo usermod -aG docker $USER

  # Log out and log back in for this to take effect
  ```
- Get Docker image for simulation. From the repo root:
  ```bash
  git submodule update --init --recursive
  mkdir -p ~/.ccache
  # Pull the simulation image (the one CI uses for SITL on this branch)
  docker pull px4io/px4-dev-simulation-focal:2021-09-08
  ```
- **TODO: QGroundControl setup**

## Instructions to build firmware for Cube Orange autopilot

- The v1.13 NuttX setup script pins `gcc-arm-none-eabi` to **9-2020-q2-update**. On an x86_64 Linux host, download that version once so the container build does not depend on the host's Ubuntu compiler package:
  ```bash
  mkdir -p "${HOME}/.local/opt"
  wget -O /tmp/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 \
    https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
  tar -xjf /tmp/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 -C "${HOME}/.local/opt"
  "${HOME}/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc" --version
  ```
- From the repo root on the `v1.13.1_dev` branch, build with the NuttX image and the pinned compiler:
  ```bash
  docker pull px4io/px4-dev-nuttx-focal:2021-09-08
  docker run -it --rm --init --privileged --name px4-cubeorange \
    -e LOCAL_USER_ID=$(id -u) \
    -e CCACHE_DIR=${HOME}/.ccache -v ${HOME}/.ccache:${HOME}/.ccache \
    -v ${HOME}/.local/opt/gcc-arm-none-eabi-9-2020-q2-update:/opt/gcc-arm-none-eabi-9-2020-q2-update:ro \
    -v ${PWD}:${PWD} -w ${PWD} \
    px4io/px4-dev-nuttx-focal:2021-09-08 \
    bash -c 'export PATH=/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH; arm-none-eabi-gcc --version; make -C platforms/nuttx/NuttX/nuttx/tools -f Makefile.host clean && make -C   platforms/nuttx/NuttX/nuttx/tools -f Makefile.host default mkversion && make cubepilot_cubeorange_default'
  ```

  > The NuttX tool cleanup removes generated host utilities (such as `incdir` and `mkconfig`), then rebuilds them against the container's glibc before Ninja starts. This matters if the checkout was previously built on a newer host: otherwise the container can fail with `GLIBC_2.34 not found`. Do not run the cleanup alone before an incremental build.

  > If NuttX then reports `include/arch already exists but is not a symbolic link`, check that `platforms/nuttx/NuttX/nuttx/include/arch` is empty. If it is, run `rmdir platforms/nuttx/NuttX/nuttx/include/arch` from the repo root and rerun the build. Do not remove it if it contains files.

  The firmware is written to `build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4`.
- To flash the CubeOrange autopilot
  - Disconnect other flight controllers before flashing;
  - Connect the autopilot to the host over USB
  - Open QGroundControl's **Vehicle Setup > Firmware**, choose **Advanced settings > Custom firmware file**
  - Select the `.px4` file generated above.
  - Flash

## PX4 SITL simulation with Gazebo (headless)

- From the repo root:
  ```bash
  docker run -it --rm --init --privileged --network host --name px4-sitl \
    -e LOCAL_USER_ID=$(id -u) \
    -e CCACHE_DIR=${HOME}/.ccache -v ${HOME}/.ccache:${HOME}/.ccache \
    -v ${PWD}:${PWD} -w ${PWD} \
    px4io/px4-dev-simulation-focal:2021-09-08 \
    bash -c 'HEADLESS=1 make px4_sitl gazebo'
  ```
  The first build takes a while. When finished, you should get the `pxh>` prompt.
- Start QGroundControl on the host after `pxh>` appears. It should connect automatically on UDP 14550.
- To stop the simulation
  ```bash
  pxh> shutdown
  ```
- If it's hung, from another terminal:
  ```bash
  docker stop px4-sitl
  ```
Notes on the flags:
- `--network host` lets QGroundControl or MAVSDK on the host reach PX4 over UDP (14550 for the GCS link, 14540 for offboard/MAVSDK). Only publishing ports (`-p`) isn't enough, because PX4 sends to `localhost`, which is the container itself.
- The repo is mounted at the **same path** as on the host, so the build directory still works whether you build on the host or in the container.
- `LOCAL_USER_ID` makes the build run as your user, so files in `build/` aren't owned by root.

### Additional notes

- **Other vehicles**: Replace `gazebo` with a model-specific target, e.g.:
  ```bash
  bash -c 'HEADLESS=1 make px4_sitl gazebo_standard_vtol'
  bash -c 'HEADLESS=1 make px4_sitl gazebo_typhoon_h480'
  ```
- **With Gazebo GUI**: Allow the container to use your X server, then drop `HEADLESS=1` and pass the display through:
  ```bash
  xhost +local:docker

  docker run -it --rm --init --privileged --network host --name px4-sitl \
    -e LOCAL_USER_ID=$(id -u) \
    -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e CCACHE_DIR=${HOME}/.ccache -v ${HOME}/.ccache:${HOME}/.ccache \
    -v ${PWD}:${PWD} -w ${PWD} \
    px4io/px4-dev-simulation-focal:2021-09-08 \
    bash -c 'make px4_sitl gazebo'
  ```
- **Interactive shell**: Replace the final `bash -c '...'` with `bash` to get a shell inside the container, then run `make` targets yourself.
- **Alternative**: The repo's helper script `Tools/docker_run.sh` also works, as long as you override the image (it picks the NuttX image for `px4_sitl` by default):
  ```bash
  PX4_DOCKER_REPO=px4io/px4-dev-simulation-focal:2021-09-08 \
    ./Tools/docker_run.sh 'HEADLESS=1 make px4_sitl gazebo'
  ```
  It doesn't use host networking, though. It only publishes UDP 14556, so QGroundControl on the host usually won't connect. Use the `docker run` command above for interactive testing.

## PX4 SITL simulation with Airsim and SeesInterface2

> TODO:
> - Complete this section
> - Put this doc in sees tools?
> - Do we need to run the drone version and gcs version of si2? if so, how
> - Describe how to checkout v1.13 version of the firmware
> - Once done, have all the steps be verified by a peer

Simulation framework configuration files are in <https://github.com/SEESAI/SeesTools/tree/main/CI/SimulationFramework/PX4Framework>

**Terminal 1 (Airsim)**

- Download an Airsim build from Google Drive at [`<DroneSoftware>/Simulation/Airsim Builds/Airsim Builds Linux`](https://drive.google.com/drive/u/0/folders/1yiP4zU01y0jaJWnI9CWtF3XYL1Ut5tR4)
- Extract the `.zip` file to `$HOME/Sees/AirsimBuilds`.
  - For example, for the sample environment `2024-06-25-LandscapeMountains.zip`, we would end up with `$HOME/Sees/AirsimBuilds/LandscapeMountains/`
- Copy Airsim SITL settings file from [`https://github.com/SEESAI/SeesTools/blob/main/CI/SimulationFramework/PX4Framework/settings-sitl.json`](https://github.com/SEESAI/SeesTools/blob/main/CI/SimulationFramework/PX4Framework/settings-sitl.json) into `$HOME/Documents/AirSimSettings/settings.json` where Airsim expects it
- Start Airsim
  ```bash
  cd $HOME/Sees/AirsimBuilds/LandscapeMountains
  chmod +x LandscapeMountains.sh
  ./LandscapeMountains.sh
  ```
  You should see the following, indicating it's waiting for a TCP connection to PX4
  ```bash
  Waiting for TCP connection on port 4560, local IP 127.0.0.1
  PX4
  ```

**Terminal 2 (PX4)**

- From the root of [Firmware](https://github.com/SEESAI/Firmware) repo, checked out at v1.13
  ```bash
  docker run -it --rm --init --privileged --network host --name px4-sitl \
    -e LOCAL_USER_ID=$(id -u) \
    -e CCACHE_DIR=${HOME}/.ccache -v ${HOME}/.ccache:${HOME}/.ccache \
    -v ${PWD}:${PWD} -w ${PWD} \
    px4io/px4-dev-simulation-focal:2021-09-08 \
    bash -c 'HEADLESS=1 make px4_sitl none'
  ```
  You should see the following on the terminal
  ```bash
  INFO  [simulator] Simulator connected on TCP port 4560.
  ```
  Correspondingly, the terminal running Airsim should report
  ```bash
  Connected to SITL over TCP.
  received first heartbeat
  ```

**Terminal 3 (QGroundControl)**

- TODO: setup and run SeesAI QGroundControl as per https://github.com/SEESAI/qgroundcontrol_launcher/blob/vilas/fix/local_user/README.md. select other
- CommsLink -> Add Link -> UDP (port 14550) **TODO: confirm**

**Terminal 4 (SeesInterface)**

- TODO: Setup SI2
- In `CMakeConfig.cmake`, enable Airsim and PX4 plugins
  ```bash
  set(PLUGIN_AIRSIM ON CACHE BOOL "")
  set(PLUGIN_PX4    ON CACHE BOOL "")
  ```
- At the end of `SeesInterface2/Config/MainConfig.yml` set the Airsim config file as follows:
  ```yaml
  Config: Airsim/Airsim_SITL_Drone.yml
  ```
  (**TODO: would `Airsim/AirsimGroundTruthPoseSITL.yml` also work?**)
- You may want to comment out the variables set under `# POSE ESTIMATOR SETTINGS` in `MainConfig.yml`
- Setup SI2

**TODO: Add a how it works section**
- Take relevant bits from SeesInterface2/Notes/RunWithPX4AndAirsim.md
