# can-dbc-nodes

This repo contains the dbc files for the sensors, as well as defines the logic for the single node CAN. This node subscribes to different sensors (and then publishes the data to the ROS2 system?).

## FAQs

- What is CAN?

  - Look here for a good introduction to CAN bus: [https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial](https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial)
  - Side note: it may be confusing, but for this project, we only have 1 ECU (electronic control units, AKA CAN nodes) in the CAN network. This node subscribes to messages published by sensors in the network, which are preconfigured based on the dbc files to publish messages. They don't process any messages don't count as ECUs.

- What is ROS2?

  - Read this: [https://docs.ros.org/en/jazzy/Concepts.html](https://docs.ros.org/en/jazzy/Concepts.html)

- How do CAN and ROS2 interact?

  - (?)

- What is a dbc file?

  - DataBase CAN files define the process for decoding raw CAN bus data into physical values. For example, the following (signal) line defines how to process a segment of the data in a CAN message/frame:

    `SG_Temp1 : 0|8@0+ (1,0) [0|120] "C"Vector__XXX`

    - SG_Temp1: name
    - `0`: start bit
    - `8`: message length (bits)
    - `@0`: endianness (CSO 1), @0 or @1
    - `+`: signed/unsigned, + or -
    - `(1,0)`: scaling / offset real value = raw value \* 1 + 0
    - `[0|120]`: range of valid real values
    - `"C"`: unit (for tooling, not important)
    - `Vector__XXX`: placeholder for any sender/receiver

  - Read [https://docs.openvehicles.com/en/latest/components/vehicle_dbc/docs/dbc-primer.html](https://docs.openvehicles.com/en/latest/components/vehicle_dbc/docs/dbc-primer.html) for more information.

## Contributing Guidelines

For most contributing questions: ask the software lead!

### Installing the stylechecker

1. Install clang-format using your package manager (e.g. brew install clang-format).
2. Download the ROS 2 .clang-format file and place it in the root directory:
   `wget https://raw.githubusercontent.com/ament/ament_lint/master/ament_clang_format/ament_clang_format/configuration/.clang-format`
3. If using vscode, put the following line into `.vscode/settings.json`: `"C_Cpp.clang_format_style": "file"`
4. Reload your editor if necessary.

### other:

from an old readme on this github:

Running raptor_dbw_can with kvaser hardware:

1. make sure kvaser-interface is built and installed first
   clone this repository
2. modify the launch parameters file in raptor_dbw_can/launch/launch_params.yaml -"hardware_id" is the serial number (S/N) for the kvaser hardware. This must match your hardware -"circuit_id" is the can channel number (0-n)
3. in the terminal, with the path set to the base the workspace:
   - `colcon build --packages-up-to raptor_dbw_can`
   - `ros2 launch raptor_dbw_can raptor_dbw_can_launch.py`
