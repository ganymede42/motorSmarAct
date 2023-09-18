#ifndef SMARACT_MCS_MOTOR_DRIVER_H
#define SMARACT_MCS_MOTOR_DRIVER_H

/* Motor driver support for smarAct MCS RS-232 Controller   */

/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28 */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 9/11 */

#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>
#include <epicsTypes.h>

/** drvInfo strings for extra parameters that the MCS2 controller supports */
#define MCSPtypString "PTYP"
#define MCSPtypRbString "PTYP_RB"
#define MCSAutoZeroString "AUTO_ZERO"
#define MCSHoldTimeString "HOLD"
#define MCSSclfString "MCLF"
#define MCSCalString "CAL"


class SmarActMCSAxis : public asynMotorAxis
{
public:
  SmarActMCSAxis(class SmarActMCSController *cnt_p, int axis, int channel);

  //asynMotorAxis methods
  asynStatus move(double position, int relative, double min_vel, double max_vel, double accel);
  asynStatus moveVelocity(double min_vel, double max_vel, double accel);
  asynStatus home(double min_vel, double max_vel, double accel, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving_p);
  asynStatus setPosition(double position);

protected:
  //own methods
  void checkType();
  asynStatus getVal(const char *parm, int *val);
  asynStatus getAngle(int *val);
  int getVel() const { return vel_; }
  asynStatus setSpeed(double velocity);

private:
  SmarActMCSController *pC_; // pointer to asynMotorController for this axis
  epicsInt32 vel_;
  int channel_;
  enum DevType {DT_NO_SENSOR, DT_LIN, DT_ROT};
  enum DevType devType_;
  friend class SmarActMCSController;
};


class SmarActMCSController : public asynMotorController
{
public:
  SmarActMCSController(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int disableSpeed = 0);

  asynStatus cmdWriteRead(bool dbg,const char *fmt, ...);
  int parse2Val(int *val1,int *val2);
  int parse3Val(int *val1,int *val2,int *val3);

  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *asynUser, epicsInt32 value);

protected:
  SmarActMCSAxis **pAxes_;

private:
  asynUser *pAsynUserMot_;

  int ptyp_; /**< positioner type */
#define FIRST_MCS_PARAM ptyp_
  int ptyprb_; /**< positioner type readback */
  int autoZero_;
  int holdTime_;
  int sclf_; /**< set maximum closed loop frequency */
  int cal_;  /**< calibration command */
#define LAST_MCS_PARAM cal_
#define NUM_MCS_PARAMS (&LAST_MCS_PARAM - &FIRST_MCS_PARAM + 1)

  friend class SmarActMCSAxis;
};

#endif // _cplusplus
#endif // SMARACT_MCS_MOTOR_DRIVER_H
