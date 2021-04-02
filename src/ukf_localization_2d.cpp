#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <golfi/ukf_states.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

using namespace Eigen;
using namespace std;

sensor_msgs::Imu imu; // to store IMU data
nav_msgs::Odometry utm; //to store UTM data from GNSS
golfi::ukf_states golfi_msg; // custom message to store UKF state values
nav_msgs::Odometry odom; // published odometry message

int N = 5.; // number of states
int N_sigma = 2 * N + 1; // number of sigma points
int kappa; // parameter to compute sigma points

typedef Matrix<long double, Dynamic, Dynamic> Mat;
typedef Matrix<long double, Dynamic, 1> Vec;
Mat X(5,1); // state variables
Vec u(3); // input variables: ax, ay, vyaw
Mat Q(5,5); // process noise covariance
long double y_tach = 0; // mean of predicted velocity measurements from tachometer
long double y_imu = 0; // mean of predicted yaw measurements from imu
Vec y_gnss(2); // mean of predicted x and y measurements from GNSS
long double P_tach = 0; // covariance of predicted tachometer measurement
long double P_imu = 0; // covariance of predicted IMU measurement
Mat P_gnss(2,2); // covariance of predicted GNSS measurement
long double vel; // magnitude velocity measurement from tachometer
long double yaw;  // yaw measurement from imu
Vec utm_gnss(2); // UTM measurement from GNSS
Vec y_tach_estimate(N_sigma); //predicted x and y measurements from tachometer
Vec y_imu_estimate(N_sigma); // predicted x and y measurements from IMU
Mat y_gnss_estimate(2,N_sigma); // predicted x and y measurements from GNSS
double R_tach; // measurement noise covariance for tachometer
double R_imu; // measurement noise covariance for IMU
Mat R_gnss(2,2); // measurement noise covariance for GNSS
Mat P(5,5); // state covariance
Mat sigma_points(5,11); // to store sigma points
long double dt; // delta time every 1 loop
Mat F(5,5); // motion model matrix
Mat u_mat(5,3); // input matrix in motion model
Vec weights(N_sigma);
Vec Pxy_tach(5); // cross covariance from predicted tachometer measurements
Vec Pxy_imu(5); // cross covariance from predicted IMU measurements
Mat Pxy_gnss(5,2); // cross covariance from predicted GNSS measurements
Vec K_tach(5); // Kalman gain for tachometer
Vec K_imu(5); // Kalman gain for IMU
Mat K_gnss(5,2); // Kalman gain for GNSS
Mat rotation_mat(2,2); // to transform between world/map frame and imu frame
Vec u_accel(2); // temporary variable
Vec u_accel_temp(2);
long double x_init, y_init, yaw_init; // to store the first x and y measurement from GNSS as the initial point
int i, j; // for iteration
long double pi = 3.14159265359;
long double yaw_odom;
long double yaw_gnss = 0;
long double x_gnss_old, y_gnss_old, x_gnss_new, y_gnss_new, vx_gnss, vy_gnss, v_gnss;
long double dy,dx;
long double v_tach_old = 0;
long double v_tach_new;
bool first_tach = false;

void setProcessNoiseCovarianceQ(const Mat &get_Q) {
  // sets the process noise covariance Q (used when getting Q from ROS param server such as yaml file)
  Q = get_Q;
}

void setRgnss (const Mat &get_R_gnss) {
  // sets the measurement noise covariance for GNSS (used when getting R_gnss from ROS param server such as yaml file)
  R_gnss = (Mat) get_R_gnss;
}

void setInitialCovarianceP (const Mat &get_P) {
  // sets the initial state covariance (used when getting P from ROS param server such as yaml file)
  P = get_P;
}

long double wrap_angle(long double yaw) {
  while (yaw > pi) {
    yaw -= 2 * pi;
  }
  while (yaw < -pi) {
    yaw += 2 * pi;
  }
  return (yaw);
}

void tachCallback(const geometry_msgs::Twist &tach_msg) {
  vel = tach_msg.linear.x;
  if (vel <= 4) {

    // Compute Cholesky decomposition of matrix P
    Mat L = P.llt().matrixL();

    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
    }

    // predict measurement(s)
    y_tach_estimate.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_tach_estimate[i] = sqrt(pow(sigma_points.col(i)[2],2) + pow(sigma_points.col(i)[3],2));
    }

    // calculate the mean of predicted measurements
    y_tach = 0;
    j = 0;
    for(i = 0; i < N_sigma; i++) {
      if(abs(y_tach_estimate[i]) <= 6) {
          j++;
          y_tach += y_tach_estimate[i];
      }
    }
    if (j == 0) {
      y_tach = 2;
    }
    else {
      y_tach = (long double) y_tach / j;
    }

    // calculate the covariance of predicted measurements
    P_tach = 0;
    for(i = 0; i < N_sigma; i++) {
      P_tach += (long double) weights[i] * (y_tach_estimate[i] - y_tach) * (y_tach_estimate[i] - y_tach);
    }
    P_tach += R_tach;

    // calculate the cross-covariance of predicted measurements
    Pxy_tach.setZero();
    for(i = 0; i < N_sigma; i++) {
      if(abs(y_tach_estimate[i]) <= 10) {
        Pxy_tach += (Vec) ((long double) weights[i] * (sigma_points.col(i) - X) * (y_tach_estimate[i] - y_tach));
      }
    }

    // calculate the Kalman gain of predicted measurements
    K_tach = (Vec) Pxy_tach / P_tach;

    // update states
    X = (Mat) X + K_tach * (vel - y_tach);

    // update state covariances
    P = (Mat) P - K_tach * P_tach * K_tach.transpose();
  }
}

void imuCallback(const sensor_msgs::Imu &imu_msg) {
    imu = imu_msg;
    // Get yaw from quaternion measurement
    yaw = tf::getYaw(imu.orientation);

    // Compute Cholesky decomposition of matrix P
    Mat L = P.llt().matrixL();

    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
    }

    // predict measurement(s)
    y_imu_estimate.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_imu_estimate[i] = sigma_points.col(i)[4];
    }

    // calculate the mean of predicted measurements
    y_imu = 0;
    for(i = 0; i < N_sigma; i++) {
      y_imu += (long double) weights[i] * y_imu_estimate[i];
    }

    // calculate the covariance of predicted measurements
    P_imu = 0;
    for(i = 0; i < N_sigma; i++) {
      P_imu += (long double) weights[i] * (y_imu_estimate[i] - y_imu) * (y_imu_estimate[i] - y_imu);
    }
    P_imu += R_imu;

    // calculate the cross-covariance of predicted measurements
    Pxy_imu.setZero();
    for(i = 0; i < N_sigma; i++) {
      Pxy_imu += (Vec) ((long double) weights[i] * (sigma_points.col(i) - X) * (y_imu_estimate[i] - y_imu));
    }

    // calculate the Kalman gain of predicted measurements
    K_imu = (Vec) Pxy_imu / P_imu;

    // update states
    X = (Mat) X + K_imu * (yaw - y_imu);

    // update state covariances
    P = (Mat) P - K_imu * P_imu * K_imu.transpose();
}

void gnssCallback(const nav_msgs::Odometry &utm_msg) {
    utm = utm_msg;
    if(utm.header.seq > 0) {
      x_gnss_new = utm.pose.pose.position.x;
      y_gnss_new = utm.pose.pose.position.y;
      dx = x_gnss_new - x_gnss_old;
      dy = y_gnss_new - y_gnss_old;
      yaw_gnss = atan2(dy,dx);
      vx_gnss = dx / 0.2;
      vy_gnss = dy / 0.2;
      v_gnss = sqrt(vx_gnss * vx_gnss + vy_gnss * vy_gnss);
    }

    // Compute Cholesky decomposition of matrix P
    Mat L = P.llt().matrixL();

    // Redrawn sigma points
    sigma_points.col(0) = X;
    for (i = 1; i <= N; i++) {
      sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
      sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
    }

    // predict measurement(s)
    y_gnss_estimate.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_gnss_estimate.col(i)[0] = sigma_points.col(i)[0];
      y_gnss_estimate.col(i)[1] = sigma_points.col(i)[1];
    }

    // calculate the mean of predicted measurements
    y_gnss.setZero();
    for(i = 0; i < N_sigma; i++) {
      y_gnss = (Vec) y_gnss + weights[i] * y_gnss_estimate.col(i);
    }

    // calculate the covariance of predicted measurements
    P_gnss.setZero();
    for(i = 0; i < N_sigma; i++) {
      P_gnss = (Mat) P_gnss + weights[i] * (y_gnss_estimate.col(i) - y_gnss) * (y_gnss_estimate.col(i) - y_gnss).transpose();
    }
    P_gnss = P_gnss + R_gnss;

    // calculate the cross-covariance of predicted measurements
    Pxy_gnss.setZero();
    for(i = 0; i < N_sigma; i++) {
      Pxy_gnss += weights[i] * (sigma_points.col(i) - X) * (y_gnss_estimate.col(i) - y_gnss).transpose();
    }

    // calculate the Kalman gain of predicted measurements
    K_gnss = (Mat) Pxy_gnss * P_gnss.inverse();

    // assign the actual measurement value
    utm_gnss << utm.pose.pose.position.x, utm.pose.pose.position.y;

    // update the states
    X = (Mat) X + K_gnss * (utm_gnss - y_gnss);

    // update the state covariances
    P = (Mat) P - K_gnss * P_gnss * K_gnss.transpose();

    x_gnss_old = x_gnss_new;
    y_gnss_old = y_gnss_new;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "ukf_ule");
  ros::NodeHandle n;
  ros::Subscriber tach_sub = n.subscribe("/sensor_velocity", 10, tachCallback);
  ros::Subscriber imu_sub = n.subscribe("/imu", 10, imuCallback);
  ros::Subscriber gnss_sub = n.subscribe("/utm", 10, gnssCallback);
  ros::Publisher ukf_pub = n.advertise<golfi::ukf_states>("ukf_states", 10);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
  ros::Rate loop_rate(50); // UKF frequency
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster broadcaster;

  // get parameters from ROS param server
  n.getParam("kappa", kappa);
  n.getParam("R_tach", R_tach);
  n.getParam("R_imu", R_imu);

  // get matrix parameters from ROS param server
  Mat get_R_gnss(2,2);
  get_R_gnss.setZero();
  XmlRpc::XmlRpcValue param_R_gnss;
  if (n.hasParam("R_gnss"))
  {
    try
    {
      n.getParam("R_gnss", param_R_gnss);

      ROS_ASSERT(param_R_gnss.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for (i = 0; i < 2; i++)
      {
        for (j = 0; j < 2; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << param_R_gnss[2 * i + j];
            std::istringstream istr(ostr.str());
            istr >> get_R_gnss(i, j);
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch(...) {}
    setRgnss(get_R_gnss);
  }

  Mat get_Q(5,5);
  get_Q.setZero();
  XmlRpc::XmlRpcValue param_Q;
  if (n.hasParam("process_noise_covariance_Q"))
  {
    try
    {
      n.getParam("process_noise_covariance_Q", param_Q);

      ROS_ASSERT(param_Q.getType() == XmlRpc::XmlRpcValue::TypeArray);

      for (i = 0; i < 5; i++)
      {
        for (j = 0; j < 5; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << param_Q[5 * i + j];
            std::istringstream istr(ostr.str());
            istr >> get_Q(i, j);
          }
          catch(...) {}
        }
      }
    }
    catch(...) {
      throw;
    }
    setProcessNoiseCovarianceQ(get_Q);
  }

  bool initialized = false;
  ros::Time current_time;
  ros::Time last_time;
  last_time = ros::Time::now();

  // compute weights
  weights[0] = (long double) kappa / (N + kappa);
  for (i = 1; i < N_sigma; i++) {
    weights[i] = (long double) 1 / (2 * (N + kappa));
  }

  while(ros::ok()) {
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    //********** INITIALIZATION **********//

    if(!initialized && (utm.header.seq > 0) && (imu.header.seq > 0)) {
      x_init = utm.pose.pose.position.x;
      y_init = utm.pose.pose.position.y;
      yaw_init = yaw;
      X << x_init,
          y_init,
          0,
          0,
          yaw_init;

      // Get initial state covariance P from param server
      Mat get_P(5,5);
      get_P.setZero();
      XmlRpc::XmlRpcValue param_P;
      if (n.hasParam("initial_estimate_covariance_P"))
      {
        try
        {
          n.getParam("initial_estimate_covariance_P", param_P);

          ROS_ASSERT(param_P.getType() == XmlRpc::XmlRpcValue::TypeArray);

          for (i = 0; i < 5; i++)
          {
            for (j = 0; j < 5; j++)
            {
              try
              {
                // These matrices can cause problems if all the types
                // aren't specified with decimal points. Handle that
                // using string streams.
                std::ostringstream ostr;
                ostr << param_P[5 * i + j];
                std::istringstream istr(ostr.str());
                istr >> get_P(i, j);
              }
              catch(...) {}
            }
          }
        }
        catch(...) {
          throw;
        }
        setInitialCovarianceP(get_P);
        rotation_mat << (long double) cos(wrap_angle(yaw + pi/2)), (long double) -sin(wrap_angle(yaw + pi/2)),
                        (long double) sin(wrap_angle(yaw + pi/2)), (long double) cos(wrap_angle(yaw + pi/2));
        u_accel << imu.linear_acceleration.x, -imu.linear_acceleration.y;
        u_accel = rotation_mat * u_accel;
        u << u_accel[0], u_accel[1], -imu.angular_velocity.z;
      }

      initialized = true;
      odom.header.frame_id  = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = x_init;
      odom.pose.pose.position.y = y_init;
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(wrap_angle(yaw_init + pi / 2));
    }

    else if (initialized) {
      //********** PREDICTION **********//

      F << 1, 0, dt, 0, 0,
          0, 1, 0, dt, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
      u_mat << pow(dt, 2) / 2., 0., 0.,
              0., pow(dt, 2) / 2., 0.,
              dt, 0., 0.,
              0, dt, 0.,
              0, 0., dt;

      // Compute Cholesky decomposition of matrix P
      Mat L = P.llt().matrixL();

      // Compute sigma points
      sigma_points.col(0) = X;
      for (i = 1; i <= N; i++) {
        sigma_points.col(i) = (Mat) X + sqrt(N + kappa) * L.col(i-1);
        sigma_points.col(i + N) = (Mat) X - sqrt(N + kappa) * L.col(i-1);
      }

      for(i = 0; i < N_sigma; i++) {
        sigma_points.col(i) = (Mat) F * sigma_points.col(i) + u_mat * u;
      }

      // Compute predicted mean and covariance
      X.setZero();
      for (i = 0; i < N_sigma; i++) {
        X = (Mat) X + weights[i] * sigma_points.col(i);
      }

      P.setZero();
      for(i = 0; i < N_sigma; i++) {
        P += weights[i] * (sigma_points.col(i) - X) * (sigma_points.col(i) - X).transpose();
      }
      P += Q;

    // }

    X(4,0) = wrap_angle(X(4,0));
    yaw_odom = wrap_angle(X(4,0) + pi/2);

    golfi_msg.stamp = ros::Time::now();
    golfi_msg.x = X(0,0);
    golfi_msg.y = X(1,0);
    golfi_msg.vx = X(2,0);
    golfi_msg.vy = X(3,0);
    golfi_msg.vx_gnss = vx_gnss;
    golfi_msg.vy_gnss = vy_gnss;
    golfi_msg.v_gnss = v_gnss;
    golfi_msg.v_tach = vel;
    golfi_msg.yaw_est = yaw_odom;
    golfi_msg.yaw_imu = yaw;
    golfi_msg.yaw_dydx = yaw_gnss;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.stamp  = current_time;
    odom_trans.transform.translation.x = X(0,0) - x_init;
    odom_trans.transform.translation.y = X(1,0) - y_init;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_odom);

    // odom.header.seq = 0;
    odom.header.frame_id  = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = X(0,0) - x_init;
    odom.pose.pose.position.y = X(1,0) - y_init;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_odom);
    odom.twist.twist.linear.x = X(2,0);
    odom.twist.twist.linear.y = X(3,0);
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = u[2];

    u_mat << pow(dt, 2) / 2., 0., 0.,
            0., pow(dt, 2) / 2., 0.,
            dt, 0., 0.,
            0, dt, 0.,
            0, 0., dt;
    rotation_mat << (long double) cos(wrap_angle(yaw + pi/2)), (long double) -sin(wrap_angle(yaw + pi/2)),
                    (long double) sin(wrap_angle(yaw + pi/2)), (long double) cos(wrap_angle(yaw + pi/2));
    u_accel << imu.linear_acceleration.x, -imu.linear_acceleration.y;
    u_accel = rotation_mat * u_accel;
    u << u_accel[0], u_accel[1], -imu.angular_velocity.z;
  }
    ukf_pub.publish(golfi_msg);
    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);
    ros::spinOnce();
    loop_rate.sleep();
    last_time = current_time;
  }
  return 0;
}
