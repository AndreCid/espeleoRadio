# espeleo_radio:
Package responsible for issuing an alert when the robot loses contact with the base and deal with radio quality information.

The package publishes a warning when the quality status gets too low, or the difference between noise and signal gets too low, so that the noise disturbs the signal way too much.

## Dependencies:

To run this package you're going to need Python libraries Selenium and Beautiful Soup installed in your machine.
If using pip, follow the commands:
```
sudo pip install selenium
sudo pip install beautifulsoup4
sudo pip install requests
```

If you don't have pip installed you can run:
```
python get-pip.py
pip install -U pip
```

## Scripts included on this package:

- status_catcher.py: access the online radio monitoring url and publishes the interesting statuses in a topic.

- radio_notification.py: subscribes on the topic with the statuses and checks if the condition to publish the warnings are true.

## How to interact:

Before running the nodes you need to set some things up. Go to the config directory and open the `param.yaml` file.
This file contains all the parameters the package needs to run in the correct way.

The parameter "ws_path" is the path to the current catkin workspace where the package is.
The parameter "radio_ip" is the ip of the radio by which you want to access the connection details.
The parameter "borderline_quality" defines the minimum value of connection quality status. If the value gets below that, the code makes a warning boolean variable true.
The parameter "borderline_noise" defines the minimum value of the difference between signal and noise. If the value gets below that, the code makes another warning boolean variable true.

With everything set, now you just need to run the following launch file:

`roslaunch espeleo_radio radio_status.launch`

**Topics**

- `/radio/status`: a Float32MultiArray message. The first value of the data vector carries the quality status, the second the signal status and the third the noisef status.

- `/radio/warning`: a ByteMultiArray message. The first value of the data vector becomes '1' when the quality status is too low and the second becomes '1' when the difference between noise and signal is too low.

## Running this package with simulations:

This package have a functionality to simulate Radio Signal Strength (RSS), which is done using a linear or logarithmic model of intensity decay based on distance between the robotic platform and control base.

There's a launch to run it:

`roslaunch espeleo_radio simulation.launch`

After starting the launch the emulated status will be in the usual `/radio/status` topic.

But for the scripts to run correctly it's needed to set some parameters. Open the `param.yaml` file and check the following variables.

The parameters "xb" and "yb" are the control base position in 'x' and 'y' ccordinates respectively.
The parameter "linear_model" defines which of the decayment models are being used. If set to 'True' the linear model will be used, if 'False' the logarithmic model will be used.
The parameter "odom_topic" defines the topic with odometry information that will be used to calculate the distance.
