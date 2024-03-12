/*--------------------------------------------------------------*
 * Filename     smarActMCSMotorDriver.h                         *
 * Usage        Motor driver support for smarAct MCS Controller *
 * Author       Thierry Zamofing <thierry.zamofing@psi.ch>      *
 *              Adapted from David Vine Jan 19, 2019            *
 *              Till Straumann <strauman@slac.stanford.edu>     *
 *--------------------------------------------------------------*/

#ifndef SMARACT_MCS_MOTOR_DRIVER_H
#define SMARACT_MCS_MOTOR_DRIVER_H

#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>
#include <epicsTypes.h>

/** drvInfo strings for extra parameters that the MCS2 controller supports */
#define MCSPtypString     "PTYP"
#define MCSPtypRbString   "PTYP_RB"
//#define MCSPstatString    "PSTAT"
#define MCSMclfString     "MCLF"
#define MCSHoldString     "HOLD"
#define MCSCalString      "CAL"
#define MCSAutoZeroString "AUTOZERO"

extern "C" void MCSExtra(int argc, char **argv); //forward declaration for 'friend'

class MCSController : public asynMotorController
{
public:
  MCSController(const char *asynPort, const char *ctrlPort, int numAxes, double movingPollPeriod, double idlePollPeriod, int dbgLvl);

  // functions override from asynMotorDriver
  asynStatus writeInt32(asynUser *asynUser, epicsInt32 value);

  // own member functions
  asynStatus cmdWriteRead(bool dbg,const char *fmt, ...);
  int parse2Val(int *val1,int *val2);
  int parse3Val(int *val1,int *val2,int *val3);

private:
  int ptyp_;     // positioner type
  int ptyprb_;   // positioner type readback
  //int pstatrb_;  // positoner status word readback (only MCS2, MCS1 does not have this feature)
  int mclf_;     // MCL frequency
  int hold_;     // hold time
  int cal_;      // calibration command
  int autoZero_; // set to 0 position after calibration
#define FIRST_MCS_PARAM ptyp_
#define LAST_MCS_PARAM autoZero_
#define NUM_MCS_PARAMS (&LAST_MCS_PARAM - &FIRST_MCS_PARAM + 1)

  friend class MCSAxis;
  friend void MCSExtra(int argc, char **argv);
};


class MCSAxis : public asynMotorAxis
{
public:
  MCSAxis(class MCSController *cnt_p, int axis, int dbgLvl);

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
  asynStatus getVal(bool dbg, const char *parm, int *val);
  asynStatus getAngle(bool dbg, int *val);
  asynStatus setSpeed(double velocity);

private:
  MCSController *pC_; // pointer to asynMotorController for this axis
  int dbgLvl_;
  epicsInt32 vel_;
  enum DevType {DT_NO_SENSOR, DT_LIN, DT_ROT};
  enum DevType devType_;
  friend class MCSController;
  friend void MCSExtra(int argc, char **argv);
};

#endif // _cplusplus
#endif // SMARACT_MCS_MOTOR_DRIVER_H
