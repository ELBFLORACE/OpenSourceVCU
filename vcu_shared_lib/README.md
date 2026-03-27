# EFR Shared Library - Header Overview

This directory provides core header files for the EFR (Elbflorace) shared library, offering reusable types, utility functions, and interfaces for vehicle control and ROS2 parameter management.

## Provided Headers

### `enums.hpp`

Defines several strongly-typed enumerations for vehicle mission, state, and EBS (Emergency Brake System) status.

#### Enums

- `VEHICLE_MISSION`: Mission types (e.g., INSPECTION, ACCELERATION, ENDURANCE).
- `VEHICLE_STATE`: Vehicle operational states (e.g., Idle, Ready, Drive, Emergency).
- `EBS`: Emergency Brake System states (e.g., Activated, Armed, Disabled, Error).

**Usage Example:**

```cpp
#include "vcu_shared_lib/enums.hpp"

VEHICLE_MISSION mission = ACCELERATION;
if (mission == ACCELERATION) {
    // Start acceleration sequence
}
```

---

### `params.hpp`

Provides a type-safe interface for loading ROS2 node parameters, supporting both required and optional parameters.

#### Key Types

- `ParamVariant`: Variant type for pointers to supported parameter types (double, int64_t, bool, std::string, and their optional variants).
- `ParameterTuple`: Tuple of parameter name and storage pointer.

### Parameter loading

- `loadParams(node, paramTuples)`: Loads parameters into the provided pointers.

**Usage Example:**

```cpp
#include "vcu_shared_lib/params.hpp"

double speed;
std::optional<int64_t> mode;
ParameterTuples params = {
    {"speed", &speed},
    {"mode", &mode}
};
aloadParams(node, params);
```

---

### `wheels.hpp`

Defines a templated `Wheels<T>` struct for handling four-wheel values (front-left, front-right, rear-left, rear-right) with arithmetic and utility operations.

#### Features

- Construction from message or values.
- Arithmetic operations: sum, avg, min, max, clamp, scale, add, sub, div.
- Operator overloads for `+`, `-`, `*`, `/` (both with scalars and other `Wheels`).
- Conversion to/from ROS2 message type (`ix_msgs::msg::Wheels`).
- Comparison operators (`==`, `!=`).

**Usage Example:**

```cpp
#include "vcu_shared_lib/wheels.hpp"

Wheels<float> torque(1.0, 1.0, 0.8, 0.8);
torque.scale(0.9);
auto msg = torque.to_msg();
```

---

## How to Use

1. **Add shared lib to CMakeLists.txt**

   ```cmake
   find_package(vcu_shared_lib REQUIRED)

   ament_target_dependencies(your_node
     rclcpp
     ...(other packages)...
     vcu_shared_lib
   )
   ```

2. **Add shared lib dependency to package.xml**

   ```xml
   <depend>vcu_shared_lib</depend>
   ```

3. **Include the relevant header(s):**

   ```cpp
   #include "vcu_shared_lib/enums.hpp"
   #include "vcu_shared_lib/params.hpp"
   #include "vcu_shared_lib/wheels.hpp"
   ```

4. **Use the provided types and functions as shown in the examples above**

<br />
