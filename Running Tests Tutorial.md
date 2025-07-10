### Writing Tests for Micro Autonomy

A GitHub workflow has been set up to run any tests in this repo. To add a test for a ROS node, make a `test` folder in the node with a testing class.

Example structure with `gym_vis` node:

```
└── gym_vis
    ├── assets
    │   └── f1tenth_car.stl
    ├── CMakeLists.txt
    ├── include
    │   └── gym_vis.hpp
    ├── LICENSE
    ├── package.xml
    ├── src
    │   └── gym_vis.cpp
    └── test
        └── test_gym_vis.cpp
```

Use the `#include "gtest/gtest.h"` library for testing.

# Onboarding Steps