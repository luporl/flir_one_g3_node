# flir_one_g3_node

## Installation

```
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make install
```

## Running

```
. install/setup.sh
rosrun flir_one_g3 flir_one_g3_node
```

## Published topics

- /flir_one_g3/dbg - debug messages
- /flir_one_g3/optical/image_raw/compressed - images captured by optical camera (jpeg)
- /flir_one_g3/thermal/image_raw - images captured by thermal camera (rgb8)
- /flir_one_g3/thermal/info - information derived from thermal image:
  - Minimum temperature (in °C)
  - Temperature at the image center (in °C)
  - Maximum temperature (in °C)
  - X, Y coordinates of maximum temperature in image (thermal image has a resolution of 80x60)
