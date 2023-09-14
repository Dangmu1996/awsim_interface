#pragma once

#include "rclcpp/rclcpp.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

using autoware_auto_control_msgs::msg::AckermannControlCommand;

struct VehicleCmdFilterParam
{
    double wheel_base;
    double vel_lim;
    double lon_acc_lim;
    double lon_jerk_lim;
    double lat_acc_lim;
    double lat_jerk_lim;
    double actual_steer_diff_lim;
};

class VehicleCmdFilter
{
public:
    VehicleCmdFilter();
    virtual ~VehicleCmdFilter() = default;

    void setWheelBase(double v) { param_.wheel_base = v; }
    void setVelLim(double v) { param_.vel_lim = v; }
    void setLonAccLim(double v) { param_.lon_acc_lim = v; }
    void setLonJerkLim(double v) { param_.lon_jerk_lim = v; }
    void setLatAccLim(double v) { param_.lat_acc_lim = v; }
    void setLatJerkLim(double v) { param_.lat_jerk_lim = v; }
    void setActualSteerDiffLim(double v) { param_.actual_steer_diff_lim = v; }
    void setParam(const VehicleCmdFilterParam & p) { param_ = p; }
    void setPrevCmd(const AckermannControlCommand & v) { prev_cmd_ = v; }

    void limitLongitudinalWithVel(AckermannControlCommand & input) const;
    void limitLongitudinalWithAcc(const double dt, AckermannControlCommand & input) const;
    void limitLongitudinalWithJerk(const double dt, AckermannControlCommand & input) const;
    void limitLateralWithLatAcc(const double dt, AckermannControlCommand & input) const;
    void limitLateralWithLatJerk(const double dt, AckermannControlCommand & input) const;
    void limitActualSteerDiff(
      const double current_steer_angle, AckermannControlCommand & input) const;
    void filterAll(
      const double dt, const double current_steer_angle, AckermannControlCommand & input) const;
private:
    VehicleCmdFilterParam param_;
    AckermannControlCommand prev_cmd_;

    double calcLatAcc(const AckermannControlCommand& cmd) const;
    double calcSteerFromLatacc(const double v, const double latacc) const;
    double limitDiff(const double curr, const double prev, const double diff_lim) const;
};