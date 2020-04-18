/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <Eigen/Array>
#include <Eigen/SVD>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <pr2_mechanism_controllers/base_kinematics.h>
#include <angles/angles.h>

#include <boost/scoped_ptr.hpp>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

namespace controller
{

typedef Eigen::Matrix<float, 3, 1> OdomMatrix3x1;
typedef Eigen::Matrix<float, 5, 1> OdomMatrix5x1;
typedef Eigen::Matrix<float, 5, 3> OdomMatrix5x3;
typedef Eigen::Matrix<float, 5, 5> OdomMatrix5x5;


  class PassiveWheel
  {
    public:

      /*!
       * \brief default offset from the parent caster before any transformations
       */
      geometry_msgs::Point position_;

      /*!
       * \brief name of the link
       */
      std::string link_name_;

      /*!
       * \brief Loads wheel's information from the xml description file and param server
       * @param robot_state The robot's current state
       * @param config Tiny xml element pointing to this wheel
       */
      bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node, std::string link_name);

    double steer_angle_;
  };

  /*! \class
  \brief This class inherits from Controller and computes the base odometry
  */
  class TexasOdometry : public pr2_controller_interface::Controller
  {
    public:

    /*!
    * \brief Constructor for the odometry
    */
    TexasOdometry();

    /*!
    * \brief Destructor for the odometry
    */
    ~TexasOdometry();

    /*!
    * \brief Initializes and loads odometry information from the param server
    * @param robot_state The robot's current state
    * @param config Tiny xml element pointing to this controller
    * @return Successful init
    */
    bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node);

    /*
    * \brief  The starting method is called by the realtime thread just before
    * the first call to update.  Overrides Controller.staring().
    * @return Successful start
    */
    void starting();

    /*!
    * \brief (a) Updates positions of the caster and wheels.
    * Called every timestep in realtime
    */
    void update();

    /*!
    * \brief Publishes the currently computed odometry information
    */
    void publish();


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    ros::NodeHandle node_;

    /*!
    * \brief class where the robot's information is computed and stored
    */
    BaseKinematics base_kin_;

    /*!
    * \brief Finds and stores the latest odometry information
    */
    void updateOdometry();

    /*!
    * \brief Builds the odometry message and prepares it for sending
    * @param msg The nav_msgs::Odometry into which the odometry values are placed
    */
    void getOdometryMessage(nav_msgs::Odometry &msg);
    
    /*!
    * \brief Takes the current odometery information and stores it into the six double parameters
    * @param x X component of the current odom position
    * @param y Y component of the current odom position
    * @param yaw Yaw (theta) component of the current odom position
    * @param vx X component of the current odom velocity
    * @param vy Y component of the current odom velocity
    * @param vw Angular velocity (omega) component of the current odom velocity
    */
    void getOdometry(double &x, double &y, double &yaw, double &vx, double &vy, double &vw);

    /*!
    * \brief Takes the current odometry information and stores it into the Point and Twist parameters
    * @param odom Point into which the odometry position is placed (z is theta)
    * @param odom_vel into which the odometry velocity is placed
    */
    void getOdometry(geometry_msgs::Point &odom, geometry_msgs::Twist &odom_vel);

    /*!
    * \brief Computes the base velocity from the caster positions and wheel speeds
    */
    void computeBaseVelocity();

    /*!
    * \brief Computes the wheel's speed adjusted for the attached caster's rotational velocity
    * @param index The index of the wheel
    */
    double getCorrectedWheelSpeed(const int &index);

    /*!
    * \brief Function used to compute the most likely solution to the odometry using iterative least squares
    */
    OdomMatrix3x1 iterativeLeastSquares(const OdomMatrix5x3 &lhs, const OdomMatrix5x1 &rhs, const std::string &weight_type, const int &max_iter);
    
    /*!
    * \brief Finds the weight matrix from the iterative least squares residuals
    */
    OdomMatrix5x5 findWeightMatrix(const OdomMatrix5x1 &residual, const std::string &weight_type);

    /*!
    * \brief Total distance traveled by the base as computed by the odometer
    */
    double odometer_distance_;

    /*!
    * \brief Total angular distance traveled by the base as computed by the odometer
    */
    double odometer_angle_;

    /*!
    * \brief Stores the last update time and the current update time
    */
    ros::Time last_time_,current_time_;

    /*!
    * \brief Matricies used in the computation of the iterative least squares and related functions
    */
    //    Eigen::MatrixXf cbv_rhs_, fit_rhs_, fit_residual_, odometry_residual_, cbv_lhs_, fit_lhs_, cbv_soln_,fit_soln_,  weight_matrix_;

    /*!
    * \brief Point that stores the current translational position (x,y) and angular position (z)
    */
    geometry_msgs::Point odom_;

    /*!
    * \brief Twist that remembers the current translational velocities (vel.vx, vel.vy) and angular position (ang_vel.vz)
    */
    geometry_msgs::Twist odom_vel_;

    /*!
    * \brief The type of weighting used in findWeightMatrix
    */
    std::string ils_weight_type_;

    /*!
    * \brief Number of iterations used during iterative least squares
    */
    int ils_max_iterations_;

    /*!
    * \brief Maximum residual from the iteritive least squares
    */
    double odometry_residual_max_;

    /*!
    * \brief The topic name of the published odometry
    */
    std::string odom_frame_;

    /*!
    * \brief The topic name of the base link frame
    */
    std::string base_link_frame_;

    /*!
    * \brief The topic name of the base footprint frame
    */
    std::string base_footprint_frame_;

    /*!
    * \brief The last time the odometry information was published
    */
    ros::Time last_publish_time_;

    /*!
    * \brief The time that the odometry is expected to be published next
    */
    double expected_publish_time_;

    /*!
    * \brief The RealtimePublisher that does the realtime publishing of the odometry
    */
    boost::scoped_ptr<realtime_tools::RealtimePublisher <nav_msgs::Odometry> > odometry_publisher_ ;  

    /*!
    * \brief Publishes the transform between the odometry frame and the base frame
    */
    boost::scoped_ptr<realtime_tools::RealtimePublisher <tf::tfMessage> > transform_publisher_ ;  

    double sigma_x_,sigma_y_,sigma_theta_,cov_x_y_,cov_x_theta_,cov_y_theta_;

    double base_link_floor_z_offset_;

    /*!
    * \brief populate the covariance part of the odometry message
    */
    void populateCovariance(const double &residual, nav_msgs::Odometry &msg);

    int sequence_;

    bool isInputValid();

    bool verbose_, publish_odom_;

    double odom_publish_rate_;

    std::vector<PassiveWheel> passive_wheels_;

    double num_passive_wheels_;

    bool initializePassiveWheels(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node);

    OdomMatrix5x3  cbv_lhs_, fit_lhs_;
    OdomMatrix5x1  cbv_rhs_, fit_residual_, odometry_residual_;
    OdomMatrix5x1  fit_rhs_;
    OdomMatrix5x5  weight_matrix_, w_fit;
    OdomMatrix3x1  cbv_soln_, fit_soln_;

    double caster_calibration_multiplier_;
  };
}
