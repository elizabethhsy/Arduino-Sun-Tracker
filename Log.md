# C2 Project Log
## Description
As part of the Arduino microcontroller group, our project used the Arduino to calculate and point to the location the sun in the sky.

The ideal completed project would be able to take the current time, latitude and longitude and control the motors to point to different objects in the sky using the calculated elevation and azimuth angles.

## Log
### Week 1
Came up with the idea of the project, started experimenting with the Arduino and came up with the things we needed for the project: GPS module, stepper motors etc.

### Week 2
Set up a Github repository to store our code, wrote the code to calculate the elevation and azimuth of the sun given a specific latitude, longitude and time. Also wrote the code to test the motors. We managed to get one motor working. The GPS module wasn't receiving any signal even outside (possibly due to antenna problems), hence a replacement was ordered.

### Week 3
Focused on getting the motors to work together to point to the right direction of the Sun. Between the two sessions, the components for the motor and the pointer were 3D printed, the code for calculating the position of the Sun was cleaned up, and replacements for the GPS module were ordered.

During the session, we managed to get both of the motors to work and coordinate with each other. We also found the maximum angle that the pointer could point (140 degrees downwards) and set up safety nets to ensure that the motor doesn't overexert itself. After the motors were put together and tested, we combined the code for the motor with the code for calculating the position of the Sun. We managed to get the pointer to point to the current position of the Sun. We also tried to manually set the time of day (from 12am onwards, stepping by 1 hour each iteration), and the code worked up until 1pm, but the stepper motor ended up moving in the opposite direction after that.

### Week 4
Obtained a replacement for the GPS antenna, now the GPS is able to get time data. There is still no location data, but that is most likely due to the lack of GPS signal indoors rather than an issue with the GPS module itself. Managed to get the time function on the GPS work with the current code to get the angle of elevation and azimuth. The motors also work to point to the correct direction. In addition, we also fixed the issue of the latter half of the day not working by subtracting the azimuth angle from 180 when it was past noon.

### Week 5
Everything worked fine. Set up the UI for calibrating the direction of North in the beginning, and displaying the elevation and azimuth of the sun in addition to the current time. Tested the system by leaving it there for 2 hours, and it worked as expected.

### Week 6 (final week)
Changed the stepper size from 1.8 degrees (1 full step) to 0.1135 degrees (1/16th of a step), which made the stepper motor run more smoothly. We brought the setup out to the balcony, where you could see the sun. Dr Savage came to have a look at our project, and we showed the process of calibration and pointing (even though it didn't work the first time). The rest of the time was spent filming a time lapse (which didn't really work), and I drew a circuit diagram while Niklas documented and cleaned the code by adding comments.
