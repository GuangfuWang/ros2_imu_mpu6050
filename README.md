# ros2_imu_mpu6050
This is ros2 node for mpu 6050 imu.
## Build
1. first you need to create a workspace folder(in case you already have one, ignore this step):
```shell
sudo mkdir sensors 
cd sensors
sudo mkdir src
cd src
```
2. after create your workspace, download our source file using:
```shell
git clone XXX
```
3. To build you need install ros properly and change the CMakeLists.txt file at line 33 to your ros2 distro(i use foxy so if you use this version, then nothing modification is required literally.)
back to your workspace directory, run:
```shell
rosdep install --from-path src -y
```
to install all nessesary dependencies. (the above **src** can be changed to folders that contains our imu source files.)

run:
```shell
colcon build --packages-select imu
```
to build our executable

run:
```shell
. install/setup.bash
ros2 run imu mpu6050uart
```
to run our node.
