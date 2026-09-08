# ROS checks

The conversion and schema checks need no ROS runtime and never open a device:

```sh
cmake -S ros/tests -B build/ros-tests
cmake --build build/ros-tests
ctest --test-dir build/ros-tests --output-on-failure
python3 ros/tests/test_schema.py
```

The C++ fixtures exercise current-frame IMU conversion, standard missing
covariances, independent INS/GNSS fields, 64-bit presence flags, SI units, and
raw product pressure preservation. Schema checks also reject standard pressure
publishers and obsolete combined temperature/pressure switches.
They are not a substitute for compiling against the real ROS message types.

After building and sourcing a ROS 2 workspace, run the Linux runtime check in
a dedicated ROS domain:

```sh
ROS_DOMAIN_ID=121 python3 ros/tests/test_ros2_runtime.py
```

This uses a pseudo-terminal and a nonexistent CAN interface. It checks port
failure/recovery, partial IMU messages, idle/disconnect diagnostics, and
diagnostics while simulated time stays at zero. No CAN adapter or IMU is used.
Without ROS 2 Python packages it reports a skip, not a runtime pass.

For ROS 1, source the installed Noetic workspace and run:

```sh
python3 ros/tests/test_ros1_runtime.py
```

This starts an isolated ROS master and checks the serial node using a
pseudo-terminal, including failure/recovery and diagnostics with paused
simulation time. Without ROS 1 Python packages it reports a skip.

Build ROS 1 Noetic and ROS 2 Humble/Jazzy/Lyrical in their respective environments.
Then verify installed launch files and schema generation, including a workspace
containing the complete SDK checkout. Hardware receive rates and CAN behavior
still require tests on the intended equipment.
