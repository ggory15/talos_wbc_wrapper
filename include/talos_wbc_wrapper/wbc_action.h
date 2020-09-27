
#ifndef WBC_ACTION_H
#define WBC_ACTION_H

#include <pal_locomotion/biped_controller.h>
#include <pal_locomotion/state_machine/walking_action_base.h>

namespace pal_locomotion
{
struct WBCActionParameters : public ariles::ConfigurableBase
{
  WBCActionParameters()
  {
    setDefaults();
  }

  void setDefaults()
  {
    sin_freq_z_ = 1.0;
    sin_amp_z_ = 0.0;
    sin_freq_y_ = 1.0;
    sin_amp_y_ = 0.0;
    sin_freq_x_ = 1.0;
    sin_amp_x_ = 0.0;
  }

#define ARILES_SECTION_ID "WBCActionParameters"
#define ARILES_CONSTRUCTOR WBCActionParameters
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(sin_freq_x)                                                    \
  ARILES_ENTRY_(sin_freq_y)                                                     \
  ARILES_ENTRY_(sin_freq_z)                                                             \
  ARILES_ENTRY_(sin_amp_x)                                                             \
  ARILES_ENTRY_(sin_amp_y)                                                             \
  ARILES_ENTRY_(sin_amp_z)
#include ARILES_INITIALIZE

  double sin_freq_z_, sin_amp_z_;
  double sin_freq_y_, sin_amp_y_;
  double sin_freq_x_, sin_amp_x_;
};

class WBCActions : public WalkingActionBase
{
public:
  WBCActions() : time_(0.0)
  {
  }

  WBCActions(ros::NodeHandle &nh, BController *bController);
  virtual ~WBCActions();

  bool configure(ros::NodeHandle &nh, BController *bController,
                 const property_bag::PropertyBag &parameters) override;

  bool enterHook(const ros::Time &time);

  /// This function is called every cycle of the loop, until the end of the Action()
  bool cycleHook(const ros::Time &time);

  /// Return "true" if the Action has to be stopped. The default implementation use time
  /// to stop the action;
  bool isOverHook(const ros::Time &time);

  /// when isOver()=true, this function is called and the action is removed from the
  /// queue.
  bool endHook(const ros::Time &time);

private:
  eVector2 computePcmp(double K, double omega, const eVector2 &icp_act,
                       const eVector2 &icp_des, const eVector2 &icp_des_vel);

  Eigen::Vector2d computeActICP(double omega, const Eigen::Vector2d &com,
                                const Eigen::Vector2d &com_d,
                                const Eigen::Vector2d &local_coord);

  ros::NodeHandle nh_;
  BController *bc_;
  eVector3 ini_target_;
  ddynamic_reconfigure::DDynamicReconfigurePtr ddr_;
  WBCActionParameters params_;
  double time_;
};
}

#endif  // WBC_ACTION_H
