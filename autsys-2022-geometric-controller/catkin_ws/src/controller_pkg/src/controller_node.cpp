#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define PI M_PI

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 0 |  Autonomous Systems - Fall 2022  - Lab 3 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab 3 Handout for more details).
//
//  The include below is strongly suggested [but not mandatory if you have
//  better alternatives in mind :)]. Eigen is a C++ library for linear algebra
//  that will help you significantly with the implementation. Check the
//  quick reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                                 end part 0
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class controllerNode{
  ros::NodeHandle nh;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 |  Declare ROS callback handlers
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // In this section, you need to declare:
  //   1. two subscribers (for the desired and current UAVStates)
  //   2. one publisher (for the propeller speeds)
  //   3. a timer for your main control loop

  ros::Subscriber des_state;
  ros::Subscriber cur_state;
  ros::Publisher prop_speed; 
  ros::Timer time;

  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::Matrix4d F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop


  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 2 |  Initialize ROS callback handlers
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // In this section, you need to initialize your handlers from part 1.
      // Specifically:
      //  - bind controllerNode::onDesiredState() to the topic "desired_state"
      //  - bind controllerNode::onCurrentState() to the topic "current_state"
      //  - bind controllerNode::controlLoop() to the created timer, at frequency
      //    given by the "hz" variable
      //
      // Hints: 
      //  - use the nh variable already available as a class member
      //  - read the lab 3 handout to fnd the message type
    
      des_state = nh.subscribe("desired_state",1000,&controllerNode::onDesiredState,this);
      cur_state = nh.subscribe("current_state",1000,&controllerNode::onCurrentState,this);
      time = nh.createTimer(ros::Duration(1/hz),&controllerNode::controlLoop,this);

      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 2
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 6 [NOTE: save this for last] |  Tune your gains!
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // Live the life of a control engineer! Tune these parameters for a fast
      // and accurate controller. To get around running catkin build every time
      // you changed parameters, please move them to the dedicated controller
      // parameter yaml file in /config.
      // import them using ROS node handle getParam:
      // nh.getParam(ros::this_node()::getName() + ..., ...)

      // Controller gains
      
      nh.getParam(ros::this_node::getName() +"/kx", kx);
      nh.getParam(ros::this_node::getName() +"/kv", kv);
      nh.getParam(ros::this_node::getName() +"/kr", kr);
      nh.getParam(ros::this_node::getName() +"/komega", komega);

      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 6
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

      // Initialize constants
      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
      //funzione di callback
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 3 | Objective: fill in xd, vd, ad, yawd
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // 3.1 Get the desired position, velocity and acceleration from the in-
      //     coming ROS message and fill in the class member variables xd, vd
      //     and ad accordingly. You can ignore the angular acceleration.
      //
      // Hint: use "v << vx, vy, vz;" to fill in a vector with Eigen.

      xd << des_state.transforms[0].translation.x, des_state.transforms[0].translation.y, des_state.transforms[0].translation.z;
      vd << des_state.velocities[0].linear.x, des_state.velocities[0].linear.y, des_state.velocities[0].linear.z;
      ad << des_state.accelerations[0].linear.x, des_state.accelerations[0].linear.y, des_state.accelerations[0].linear.z;

      // 3.2 Extract the yaw component from the quaternion in the incoming ROS
      //     message and store in the yawd class member variable
      //
      //  Hints:
      //    - use the methods tf::getYaw(...)
      //    - maybe you want to use also tf::quaternionMsgToTF(...)

      tf::Quaternion q;
      tf::quaternionMsgToTF(des_state.transforms[0].rotation, q);
      yawd = tf::getYaw(q);

      //ROS_INFO("yaw_desired = %f",yawd);

      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 3
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      //funzione di callback
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 4 | Objective: fill in x, v, R and omega
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // Get the current position and velocity from the incoming ROS message and
      // fill in the class member variables x, v, R and omega accordingly.
      //
      //  CAVEAT: cur_state.twist.twist.angular is in the world frame, while omega
      //          needs to be in the body frame!

      x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
      v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;

      double q1 = cur_state.pose.pose.orientation.x;
      double q2 = cur_state.pose.pose.orientation.y;
      double q3 = cur_state.pose.pose.orientation.z;
      double q4 = cur_state.pose.pose.orientation.w;
      
      R(0,0) = pow(q1,2) - pow(q2,2) - pow(q3,2) + pow(q4,2);
      R(0,1) = 2 * ((q1 * q2) - (q3 * q4));
      R(0,2) = 2 * ((q1 * q3) + (q2 * q4));
      R(1,0) = 2 * ((q1 * q2) + (q3 * q4));
      R(1,1) = - pow(q1,2) + pow(q2,2) - pow(q3,2) + pow(q4,2);
      R(1,2) = 2 * ((q2 * q3) - (q1 * q4));
      R(2,0) = 2 * ((q1 * q3) - (q2 * q4));
      R(2,1) = 2 * ((q2 * q3) + (q1 * q4));
      R(2,2) = - pow(q1,2) - pow(q2,2) + pow(q3,2) + pow(q4,2);

      Eigen::Vector3d omega_world;
      omega_world << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y, cur_state.twist.twist.angular.z;

      omega = R.inverse()*omega_world;

      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 4
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 5 | Objective: Implement the controller!
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // 5.1 Compute position and velocity errors. Objective: fill in ex, ev.
    //  Hint: [1], eq. (6), (7)

    ex = x -xd;
    ev = v - vd;

    // 5.2 Compute the Rd matrix.
    //
    //  Hint: break it down in 3 parts:
    //    - b3d vector = z-body axis of the quadrotor, [1] eq. (12)
    //    - check out [1] fig. (3) for the remaining axes [use cross product]
    //    - assemble the Rd matrix, eigen offers: "MATRIX << col1, col2, col3"
    //
    //  CAVEATS:
    //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term and
    //        ii) the overall sign (in front of the fraction) in equation (12)
    //            of the paper
    //    - remember to normalize your axes!

    Eigen::Vector3d b3d, b2d, b1d;
    b3d = (-kx*ex -kv*ev +m*g*e3 + m*ad).normalized();
    b1d << cos(yawd), sin(yawd), 0;
    b2d = b3d.cross(b1d).normalized();
    b1d = b2d.cross(b3d).normalized();
    Eigen::Matrix3d Rd;
    Rd << b1d, b2d, b3d;

    // 5.3 Compute the orientation error (er) and the rotation-rate error (eomega)
    //  Hints:
    //     - [1] eq. (10) and (11)
    //     - you can use the Vee() static method implemented above
    //
    //  CAVEAT: feel free to ignore the second addend in eq (11), since it
    //          requires numerical differentiation of Rd and it has negligible
    //          effects on the closed-loop dynamics.

    er = 0.5*controllerNode::Vee(Rd.transpose()*R-R.transpose()*Rd);
    eomega = omega;

    // 5.4 Compute the desired wrench (force + torques) to control the UAV.
    //  Hints:
    //     - [1] eq. (15), (16)

    // CAVEATS:
    //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term
    //        ii) the overall sign (in front of the bracket) in equation (15)
    //            of the paper
    //
    //    - feel free to ignore all the terms involving \Omega_d and its time
    //      derivative as they are of the second order and have negligible
    //      effects on the closed-loop dynamics.

    double F;           // thrust
    Eigen::Vector3d M;  // torques
    F = (-kx*ex -kv*ev +m*g*e3 +m*ad).dot(R*e3);
    M = -kr*er -komega*eomega + omega.cross(J*omega);

    // 5.5 Recover the rotor speeds from the wrench computed above
    //
    //  Hints:
    //     - [1] eq. (1)
    //
    // CAVEATs:
    //     - we have different conventions for the aerodynamic coefficients,
    //       Namely: C_{\tau f} = c_d / c_f
    //               (LHS paper [1], RHS our conventions [lecture notes])
    //
    //     - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //       paper. In the paper [1], the x-body axis [b1] is aligned with a
    //       quadrotor arm, whereas for us, it is 45° from it (i.e., "halfway"
    //       between b1 and b2). To resolve this, check out equation 6.9 in the
    //       lecture notes!
    //
    //     - The thrust forces are **in absolute value** proportional to the
    //       square of the propeller speeds. Negative propeller speeds - although
    //       uncommon - should be a possible outcome of the controller when
    //       appropriate. Note that this is the case in unity but not in real
    //       life, where propellers are aerodynamically optimized to spin in one
    //       direction!

    Eigen::Vector4d sprop;      //propeller speed
    Eigen::Vector4d action;     //Applied forces and torques 

    action << M, F;
    //F2W << -d*cf/sqrt(2),  d*cf/sqrt(2),   d*cf/sqrt(2),  -d*cf/sqrt(2),
    //       -d*cf/sqrt(2), -d*cf/sqrt(2),   d*cf/sqrt(2),   d*cf/sqrt(2),
    //               -cd,            cd,            -cd,              cd,
    //                 cf,            cf,             cf,              cf; 

    F2W <<  cf*d/sqrt(2.0),  cf*d/sqrt(2.0),  -cf*d/sqrt(2.0),  -cf*d/sqrt(2.0),
           -cf*d/sqrt(2.0),  cf*d/sqrt(2.0),   cf*d/sqrt(2.0),  -cf*d/sqrt(2.0),
                        cd,             -cd,               cd,              -cd,
                        cf,              cf,               cf,               cf;

    //std::cout << "The determinant of F2W is " << F2W.determinant() << std::endl;
    sprop= F2W.inverse()*action;

    // 5.6 Populate and publish the control message
    //
    // Hint: do not forget that the propeller speeds are signed (maybe you want
    // to use signed_sqrt function).

    //ROS_INFO("Thrust= %f",F);

    mav_msgs::Actuators speed_msg;
    prop_speed = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds",1000);
    //speed_msg.angular_velocities = {v1,v2,v3,v4};
    speed_msg.angular_velocities.resize(4);

    for (int i = 0; i < 4; i++) {
      speed_msg.angular_velocities[i] = signed_sqrt(sprop(i));
    }
    prop_speed.publish(speed_msg);

    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //           end part 5, congrats! Start tuning your gains (part 6)
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ros::spin();
}