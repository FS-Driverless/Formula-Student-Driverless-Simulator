// NOLINTNEXTLINE
#include <ros/ros.h>
#include <sys/stat.h>
#include <string>
#include "iostream"

#include <sensor_msgs/Joy.h>
#include <fsds_ros_bridge/ControlCommand.h>

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
  fsds_ros_bridge::ControlCommand cmd = fsds_ros_bridge::ControlCommand();
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
    ROS_INFO("Joystick:: Joystick unplugged. Sending max breaking");
  }
  else if (!CONTROLLER_PLUGGEDIN && pluggedin_now)
  {
    // Since last check the controller has been plugged in
    ROS_INFO("Joystick:: Joystick plugged in! It is still locked.");
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
      ROS_INFO("Joystick:: Controller unlocked!");
    }
    else
    {
      if (!CONTROLLER_LOCKED_WARNING_SENT)
      {
        ROS_INFO(
            "Joystick:: Controller locked. Release both triggers simultaneously after "
            "moving them around a bit to unlock.");
        CONTROLLER_LOCKED_WARNING_SENT = true;
      }
      return;
    }
  }

  fsds_ros_bridge::ControlCommand cmd = fsds_ros_bridge::ControlCommand();

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
        "Joystick:: Out of bounds acceleration (%f) detected. Reset acceleration to 0.",
        cmd.throttle);
    cmd.throttle = 0;
  }

  if (cmd.steering > (MAXSTEERANGLE + 0.00001) || cmd.steering < -1 * (MAXSTEERANGLE + 0.00001))
  {
    // We need to do the weird 0.00001 because float comparison is a freak and this makes it
    // failsafe.

    ROS_INFO("Joystick:: Out of bounds Steer Angle (%f) detected. Reset acceleration to 0.",
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
  ROS_INFO("Joystick:: Initializing Node");
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node_handle;
  node_handle.getParam("/joy/dev", JOYSTICK_PATH);
  CONTROL_PUBLISHER = node_handle.advertise<fsds_ros_bridge::ControlCommand>(
      "/fsds/control_command", 1);
  // we only care about the last message so queue size set to 1 deletes any older messages.
  JOY_SUBSCRIBER = node_handle.subscribe("/joy", 1, JoyCallback);

  TIME_SUBSCRIBER = node_handle.createTimer(ros::Duration(0.1), CheckJoystickPluggedin);
  ROS_INFO("Joystick:: Waiting for neutral trigger state.");

  ros::spin();
  return 0;
}
