![meia](./docs/assets/logo.svg)

Pure pursuit demo in PROS

We used this code for the 2021-2022 season and made it to worlds! It is the most thrown-together code I have ever seen, however, it has two key advantages:

1. Normalization - 
    Usually, when using a seperate PID controller for forward/backward and rotational targets, you have to cap the voltage outputted by the forward/backward PID controller so that the rotational voltage is not out of bounds on one side.
    ![image](https://user-images.githubusercontent.com/61331006/183824458-847c8510-6430-4433-ad53-5a1f281fef75.png)
    We use the below technique to normalize the voltages: 
    ```cpp
        std::pair<double, double> normalize(double l_volt, double r_volt, double max) {
        double l_dif = (std::abs(l_volt) > max) ? std::abs(l_volt) - max : 0;
        double r_dif = (std::abs(r_volt) > max) ? std::abs(r_volt) - max : 0;
        // scale down the voltages similarly if one is over 127
        bool greater_dif = l_dif > r_dif;
        double dif = (greater_dif) ? l_dif : r_dif;
        double l_out = (max / (max + dif)) * l_volt;
        double r_out = (max / (max + dif)) * r_volt;
        return {l_out, r_out};
    }
    ```
    As I said, not the best code I’ve ever written.
    It’s a simple innovation, but quite effective.
2. Pure pursuit with heading correction and debt. -
    I came up with debt off the top of my head, and I don’t know the proper term so we are going with it.


    Pure pursuit works by setting a PID target for each side of the drive (in our case, using the `chassis_controller` class), and then slowly incrementing it for both sides. by doing this, you can ensure both wheels go the same amount, and can get reasonably good heading maintenance without a compass, GPS, or IMU. Of course, there are always bumps in the field, so we need to use the imu to correct for that. To do this, we define a proportional constant that converts the rotational error of the imu into a delta target for each of the wheels. However, we cannot do so recursively, or else the same distance correction would be continually applied. Instead, we ensure that the difference in error between the wheels is the distance correction.


    These have the following benefits:
    **PID constants for both heading and distance can be kept insanely tight: there is little to no fear of overshooting since we apply a decel curve, so we always hit the same mark**
    Complex motion (think reduced-jerk figure 8s, ellipses, maybe a GELU curve idk), since we can change the target for each side of the drive or the heading and distance every tick.
    
    But to ensure the existence of these benefits, we have to ensure the errors do not exceed a point where full power is sent to the motors, lest we dilute our heading correction in the algorithm from #1. To accomplish this, we also normalize our targets to this maximum using the same algorithm and assign our shortcomings to a debt that is made up before the end of the movement.
    


Created by Harvard Westlake Student Michael Barr under team 462a
