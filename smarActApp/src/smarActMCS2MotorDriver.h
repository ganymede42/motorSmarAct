/*---------------------------------------------------------------*
 * Filename     smarActMCS2MotorDriver.h                         *
 * Usage        Motor driver support for smarAct MCS2 Controller *
 * Author       Thierry Zamofing <thierry.zamofing@psi.ch>       *
 *              Adapted from David Vine Jan 19, 2019             *
 *              Till Straumann <strauman@slac.stanford.edu>      *
 *---------------------------------------------------------------*

Note:
The MCS2 controller uses 64-bit int for the encoder and target positions.
The motor record is limited to 32 bit int for RMP (https://github.com/epics-modules/motor/issues/8,
https://epics.anl.gov/tech-talk/2018/msg00087.php) which effectively limits the travel
range to +/- 2.1mm.
Since it doesn't seem the motor record will update to using 64bit int the choices I can see are:
* 1 - using a non-standard motor support
* 2 - rescaling the minimum resolution to 1nm to effectively increase the range to 2.1m

I chose option 2.
1 step = 1nm

Someone with more experience may have a better solution.

Note on controller capability:
The controller supports many more sophisticated features than are supported in this driver.
The two that may be of significant interest are:
  * TTL triggering at specified positions
  * "scan" mode where the piezo stick slip can flex up to 1.6micron to give
     very precise and fast motion
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

/* This is the same for lin and rot positioners
 * lin: controller pm --> driver nm. Because of this the user can use the positioner for mm ranges
 * rot: controller ndeg --> driver udeg. Because of this the user can use the positioner for deg ranges
 * If this scaling was not implemented the maximum range would be ~2.147 mm/deg, now it's ~2147 mm/deg */
#define PULSES_PER_STEP 1000

/** MCS2 Axis status flags **/
#define  ACTIVELY_MOVING          0x0001
#define  CLOSED_LOOP_ACTIVE       0x0002
#define  CALIBRATING              0x0004
#define  REFERENCING              0x0008
#define  MOVE_DELAYED             0x0010
#define  SENSOR_PRESENT           0x0020
#define  IS_CALIBRATED            0x0040
#define  IS_REFERENCED            0x0080
#define  END_STOP_REACHED         0x0100
#define  RANGE_LIMIT_REACHED      0x0200
#define  FOLLOWING_LIMIT_REACHED  0x0400
#define  MOVEMENT_FAILED          0x0800
#define  STREAMING                0x1000
#define  OVERTEMP                 0x4000
#define  REFERENCE_MARK           0x8000

/** MCS2 Axis reference options **/
#define START_DIRECTION        0x0001
#define REVERSE_DIRECTION      0x0002
#define AUTO_ZERO              0x0004
#define ABORT_ON_END_STOP      0x0008
#define CONTINUE_ON_REF_FOUND  0x0010
#define STOP_ON_REF_FOUND      0x0020

/** drvInfo strings for extra parameters that the MCS2 controller supports */
#define MCS2PtypString     "PTYP"
#define MCS2PtypRbString   "PTYP_RB"
#define MCS2PstatString    "PSTAT"
#define MCS2MclfString     "MCLF"
#define MCS2HoldString     "HOLD"
#define MCS2CalString      "CAL"
#define MCS2AutoZeroString "AUTOZERO"

extern "C" void MCS2Extra(int argc, char **argv); //forward declaration for 'friend'
class MCS2Axis;

class MCS2Controller : public asynMotorController
{
public:
  MCS2Controller(const char *asynPort, const char *ctrlPort, int numAxes, double movingPollPeriod, double idlePollPeriod, int dbgLvl);
  // functions override from asynMotorDriver
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  // These are the methods that we override from asynMotorDriver
  void report(FILE *fp, int level);

  // own member functions
  asynStatus clearErrors();
  asynStatus cmdWrite(bool dbg,const char *fmt, ...);
  asynStatus cmdWriteRead(bool dbg,const char *fmt, ...);

protected:
  int ptyp_;     // positioner type
  int ptyprb_;   // positioner type readback
  int pstatrb_;  // positoner status word readback (only MCS2, MCS1 does not have this feature)
  int mclf_;     // MCL frequency
  int hold_;     // hold time
  int cal_;      // calibration command
  int autoZero_; // set to 0 position after calibration
#define FIRST_MCS2_PARAM ptyp_
#define LAST_MCS2_PARAM autoZero_
#define NUM_MCS2_PARAMS (&LAST_MCS2_PARAM - &FIRST_MCS2_PARAM + 1)

  friend class MCS2Axis;
};

class MCS2Axis : public asynMotorAxis
{
public:
  MCS2Axis(class MCS2Controller *pC, int axis, int dbgLvl);
  // These are the methods we override from the base class
  void report(FILE *fp, int level);
  asynStatus poll(bool *moving);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus setPosition(double position);

private:
  MCS2Controller *pC_; // Pointer to the asynMotorController to which this axis belongs. Abbreviated because it is used very frequently
  int dbgLvl_;

  friend class MCS2Controller;
  friend void MCS2Extra(int argc, char **argv);
};

