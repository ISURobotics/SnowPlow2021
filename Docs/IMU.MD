Last communication from the previous team:

Issue 24:


Right now, our GPS node has to have our starting orientation programmed in in order to know which way is positive X.

Using the magnetometer data from the IMU, we should be able to figure this out on the fly and do it more accurately.

We're also getting a 3-axis compass sensor, which might be more accurate than the IMU for this. Once we have that sensor, we can test it against the IMU and see which one is more accurate for this.

Issue 27: 

This is a more...physical issue, but we should make a turntable that we can mount the IMU on and spin around. This can basically be a block of wood on a swivel. The goal is to see whether the IMU still gets an accurate direction after spinning for 20-30 rotations in a controlled environment. When this is done, I want a measured error value in degrees off the starting position

I agree that the imu, gps, lidar and potentially new 3 axis sensor logic are all closely intertwined and we should consider as a group and clearly define what the goal is for functionality. This logic could spill over into the traversal logic and any startup sequence we are considering.

I left a 3d printed esp32 with a mpu 6050 attached in our workspace. mpu6050: $5.00. It is fully programmed. It works like this:

```
// confirm usb port then:
// Set baud and scan
stty -F /dev/ttyUSB0 115200
cat /dev/ttyUSB0
```
Using the above will output the gyro values over serial and is easy to grab in python.
Previous team used a 9dof gyro. The one we have is a $60.00 sensor. Reading about it on reddit seems to imply cheaper better solutions are available and there are reports of others having issues as well. If we do have issues with it, we could consider the above 6dof. If time permits I can see about similiar code for a 9 dof $22.00 sensor: 

- [Potential New Sensor](https://www.digikey.com/en/products/detail/sparkfun-electronics/15335/10279707?gclsrc=aw.ds&gad_source=1&gad_campaignid=20243136172&gbraid=0AAAAADrbLlgE00iv_QijrDf124GrrtlxN&gclid=CjwKCAjwxrLHBhA2EiwAu9EdM8XNbWB1JnQKSxIMD3vuL56hVKuiMgqUN77YJCvdaj0UnhhbgpUsdRoCoDQQAvD_BwE).
