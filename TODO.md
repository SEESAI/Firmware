# TODO - mavlink fix for v1.17.0

> Remove file before merging to `v1.17.0-dev` branch

- Update [mavlink](https://github.com/SEESAI/mavlink) repo to what's required for upstream firmware v1.17
- Introduce course_accuracy and heading_accuracy to GPS_INPUT message definition (migrates [mavlink#2](https://github.com/SEESAI/mavlink/pull/2/changes))
- Introduce changes to _mavlink_ submodule from [Firmware#81](https://github.com/SEESAI/Firmware/pull/81)
- Introduce GPS_INPUT stream (migrates [Firmware#52](https://github.com/SEESAI/Firmware/pull/52/changes)). Integrate GPS_INPUT related change to it from [Firmware#81](https://github.com/SEESAI/Firmware/pull/81)
- Update mavlink submodule (migrates [Firmware#81](https://github.com/SEESAI/Firmware/pull/81) which integrates #75, 76, 77, 79)
