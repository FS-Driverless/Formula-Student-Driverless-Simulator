// NOLINTNEXTLINE
#include <ros/ros.h>
#include <sys/stat.h>
#include <string>
#include "iostream"

#include <sensor_msgs/Joy.h>
#include <ros_interface/ControlCommand.h>

/*
 This node gets input from a joystick xbox controller (see http://wiki.ros.org/joy)
 and sends the values to the lowlevel controls (hardware) on topic
/airsim_node/FSCar/control_command
Below variables specify the value mappings between xbox
controller and joy interface.

 The right trigger (RT) controls the acceleration (gas)
 The left trigger (LT) controls brake (negative acceleration).
 The x-axis of the left stick controlls the steering angle.

 When the joy driver initializes it sometimes sends very high values. This would cause the
 car to move unpredictably. Therefore the controller is 'locked' when it is starts and when the
controller is re-plugged in. To unlock both triggers must be fully pressed (gas and brake) after
which commands will be sent.

 This node checks if the controller is plugged in by polling the linux device filename. If the
controller is unplugged the file disapears and this node will start sending break.

 When the controller is locked or the controller is unplugged the node will continuously sending
break setpoints at 10hz.


== RUNNING ==
 To run this node together with the joystick drivers, use the joystick.launch file:
    > roslaunch car mission/joystick.launch

== HELP IT DOESNT WORK ==
 Make sure you run this on a computer that has the xbox controller attached!
 You can check if the controller is available by running the below command.
    > sudo ls /dev/input/js0
    crwxrwxrwx 1 root 993 13, 0 Nov  8 14:43 /dev/input/js0

 If you get permisison erros you have to give ros more permissions. Run
    > sudo chmod 777 /dev/input/js0

 If the device is connected but not available at js0 it might be mapped to another device.
 Find the correct device by running
    > sudo ls -al /dev/input/js*
 When you find the correct device mapping, update the launchfile accordingly.

 The message
    > [ERROR] Couldn't open joystick force feedback!
 is normal. Nothing to worry about. It is a warning that always happens with wired xbox controllers.

== TESTING ==
 To test this node on your computer just attach an xbox controller and run it as described above.
 Now chek the /airsim_node/FSCar/control_command topic and you should see values corresponding to your controller
movements. > `rostopic echo /airsim_node/FSCar/control_command` You can debug the input
values from the joy driver by checking the `/joy` topic.
*/

// acceleration (in m/s²) sent to TrajectorySetpoints when the gas, right trigger, is fully pressed.
#define MAXTHROTTLE 0.6

// Same as MAXTHROTTLE, but when the boost button (B) is pressed.
#define MAXTHROTTLE_BOOSTED 1

// acceleration (in m/s²) sent to TrajectorySetpoints when the brake, left trigger, is fully
// pressed.
#define MAXBRAKE 1

// maximum steering angle (in rad/s) sent to the TrajectorySetpoints when the left stick is pushed
// fully to the right. The negative value of this parameter will be sent when the left stick is
// pushed fully to the left.
#define MAXSTEERANGLE 1

// Same as MAXsteering angle, but when the boost button (B) is pressed.
#define MAXSTEERANGLE_BOOSTED 0.5

ros::Publisher CONTROL_PUBLISHER;
ros::Subscriber JOY_SUBSCRIBER;
ros::Subscriber MOTORVEL_SUBSCRIBER;
ros::Timer TIME_SUBSCRIBER;

bool CONTROLLER_LOCKED = true;
bool CONTROLLER_LOCKED_WARNING_SENT = false;
bool CONTROLLER_PLUGGEDIN = false;

double PREVIOUS_STEERANGLE;
std::string JOYSTICK_PATH;

inline bool FileExists(const std::string &devicefilename)
{
  struct stat buffer
  {
  };
  return (stat(devicefilename.c_str(), &buffer) == 0);
}

void SendMaxBreak()
{
  ros_interface::ControlCommand cmd = ros_interface::ControlCommand();
  cmd.header.stamp = ros::Time::now();
  cmd.throttle = 0;
  cmd.steering = PREVIOUS_STEERANGLE;
  cmd.brake = 1;
  CONTROL_PUBLISHER.publish(cmd);
}

void CheckJoystickPluggedin(const ros::TimerEvent & /*event*/)
{
  bool pluggedin_now = FileExists(JOYSTICK_PATH);
  if (CONTROLLER_PLUGGEDIN && !pluggedin_now)
  {
    // Since last check the controller has been unplugged
    CONTROLLER_LOCKED = true;
    CONTROLLER_LOCKED_WARNING_SENT = false;
    ROS_INFO("ManualJoyControl:: Joystick unplugged. Sending max breaking");
  }
  else if (!CONTROLLER_PLUGGEDIN && pluggedin_now)
  {
    // Since last check the controller has been plugged in
    ROS_INFO("ManualJoyControl:: Joystick plugged in! It is still locked.");
  }

  CONTROLLER_PLUGGEDIN = pluggedin_now;

  if (!CONTROLLER_PLUGGEDIN || CONTROLLER_LOCKED)
  {
    // 10 times a second we send a max break if unplugged or locked.
    SendMaxBreak();
  }
}

void JoyCallback(const sensor_msgs::Joy &joystickmsg)
{
  float left_trigger = joystickmsg.axes[2];

  if (!CONTROLLER_PLUGGEDIN)
  {
    // already breaking 10 times a second if joystick not conneced (see check_joystick_pluggedin).
    return;
  }
  float right_trigger = joystickmsg.axes[5];

  if (CONTROLLER_LOCKED)
  {
    if (left_trigger == 1 && right_trigger == 1)
    {
      CONTROLLER_LOCKED = false;
      ROS_INFO("ManualJoyControl:: Controller unlocked!");
    }
    else
    {
      if (!CONTROLLER_LOCKED_WARNING_SENT)
      {
        ROS_INFO(
            "ManualJoyControl:: Controller locked. Release both triggers simultaneously after "
            "moving them around a bit to unlock.");
        CONTROLLER_LOCKED_WARNING_SENT = true;
      }
      return;
    }
  }

  ros_interface::ControlCommand cmd = ros_interface::ControlCommand();

  bool boost_button_pressed = joystickmsg.buttons[1] != 0;
  float max_acceleration = -1;
  float max_steerangle = 0;

  if (boost_button_pressed)
  {
    max_acceleration = MAXTHROTTLE_BOOSTED;
    max_steerangle = MAXSTEERANGLE_BOOSTED;
  }
  else
  {
    max_acceleration = MAXTHROTTLE;
    max_steerangle = MAXSTEERANGLE;
  }

  if (left_trigger != 1)
  {
    // if leftTrigger is not neutral, brake
    cmd.throttle = 0;
    cmd.brake = (-1 * left_trigger + 1) * (MAXBRAKE / 2.0);
  }
  else if (right_trigger != 1)
  {
    // if rightTrigger is not neutral, accelerate
    cmd.throttle = (-1 * right_trigger + 1) * (max_acceleration / 2);
  }

  cmd.steering = -joystickmsg.axes[0] * max_steerangle;
  PREVIOUS_STEERANGLE = cmd.steering;

  if (cmd.throttle > max_acceleration)
  {
    ROS_INFO(
        "ManualJoyControl:: Out of bounds acceleration (%f) detected. Reset acceleration to 0.",
        cmd.throttle);
    cmd.throttle = 0;
  }

  if (cmd.steering > (MAXSTEERANGLE + 0.00001) || cmd.steering < -1 * (MAXSTEERANGLE + 0.00001))
  {
    // We need to do the weird 0.00001 because float comparison is a freak and this makes it
    // failsafe.

    ROS_INFO("ManualJoyControl:: Out of bounds Steer Angle (%f) detected. Reset acceleration to 0.",
             cmd.steering);
    cmd.steering = PREVIOUS_STEERANGLE;
  }
  cmd.header.stamp = ros::Time::now();

  CONTROL_PUBLISHER.publish(cmd);
}


int main(
    int argc,
    char *argv[]) // NOLINT(readability-identifier-naming) because this name must not be renamed
{
  ROS_INFO("JoyControl:: Initializing Node");
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node_handle;
  node_handle.getParam("/joy/dev", JOYSTICK_PATH);
  CONTROL_PUBLISHER = node_handle.advertise<ros_interface::ControlCommand>(
      "/airsim_node/FSCar/control_command", 1);
  // we only care about the last message so queue size set to 1 deletes any older messages.
  JOY_SUBSCRIBER = node_handle.subscribe("/joy", 1, JoyCallback);

  TIME_SUBSCRIBER = node_handle.createTimer(ros::Duration(0.1), CheckJoystickPluggedin);
  ROS_INFO("JoyControl:: Waiting for neutral trigger state.");

  ros::spin();
  return 0;
}
