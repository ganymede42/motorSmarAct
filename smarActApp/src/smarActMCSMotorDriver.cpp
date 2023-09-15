
/* Motor driver support for smarAct MCS RS-232 Controller   */

/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28 */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 9/11 */

#include <iocsh.h>

#include <asynOctetSyncIO.h>
#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <smarActMCSMotorDriver.h>
#include <errlog.h>

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <exception>

#include <math.h>

#include <epicsString.h>
#include <epicsExport.h>

/* Static configuration parameters (compile-time constants) */
#ifdef DEBUG
  #define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
  #define DBG_PRINTF(...)
#endif

#define CMD_LEN 50
#define REP_LEN 50
#define DEFLT_TIMEOUT 2.0

#define FAR_AWAY_LIN 1000000000 /*nm*/
#define FAR_AWAY_ROT 32767  /*revolutions*/
#define UDEG_PER_REV 360000000

enum SmarActMCSStatus {
  Stopped     = 0,
  Stepping    = 1,
  Scanning    = 2,
  Holding     = 3,
  Targeting   = 4,
  MoveDelay   = 5,
  Calibrating = 6,
  FindRefMark = 7,
  Locked = 9
};

//-----------------------------------------------------------

SmarActMCSAxis::SmarActMCSAxis(class SmarActMCSController *cnt_p, int axis, int channel)
  : asynMotorAxis(cnt_p, axis), c_p_(cnt_p)
{
  asynStatus ast;
  int val;
  channel_ = channel;

  asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "SmarActMCSAxis::SmarActMCSAxis -- creating axis %u\n", axis);

  if (c_p_->disableSpeed_)
    ast = asynSuccess;
  else
    ast = getVal("GCLS", &vel_);
  DBG_PRINTF("SmarActMCSAxis::SmarActMCSAxis: GCLS %u returned %i\n", axis, ast);
  if (ast)
    goto bail;
  if ((ast = getVal("GS", &val)))
    goto bail;

  setIntegerParam(c_p_->autoZero_, 1);
  setIntegerParam(c_p_->holdTime_, 0);

  checkType();

bail:
  setIntegerParam(c_p_->motorStatusProblem_, ast ? 1 : 0);
  setIntegerParam(c_p_->motorStatusCommsError_, ast ? 1 : 0);

  callParamCallbacks();

  if (ast) {
    asynPrint(c_p_->pasynUserSelf, ASYN_TRACE_ERROR, "SmarActMCSAxis::SmarActMCSAxis -- channel %u ASYN error %i", axis, ast);
  }
}

asynStatus SmarActMCSAxis::move(double position, int relative, double min_vel, double max_vel, double accel)
{
  asynStatus ast;
  int holdTime;
  const char *fmt;
  long int rpos;
  epicsInt32 angle;
  int rev;

  DBG_PRINTF("SmarActMCSAxis::move: pos:%g rel:%d min_vel:%g max_vel:%g\n", position, relative, min_vel, max_vel);

  c_p_->getIntegerParam(axisNo_, c_p_->holdTime_, &holdTime);
  if ((ast = setSpeed(max_vel)))
    goto bail;

  rpos = lround(position);

  switch(devType_){
  case DT_NO_SENSOR:
    break;
  case DT_LIN:
    fmt = relative ? ":MPR%u,%ld,%d" : ":MPA%u,%ld,%d";
    ast = moveCmd(fmt, channel_, rpos, holdTime);
    break;
  case DT_ROT:
    fmt = relative ? ":MAR%u,%ld,%d,%d" : ":MAA%u,%ld,%d,%d";
    angle = (epicsInt32)rpos % UDEG_PER_REV;
    rev = (int)(rpos / UDEG_PER_REV);
    if (angle < 0){
      angle += UDEG_PER_REV;
      rev -= 1;
    }
    ast = moveCmd(fmt, channel_, angle, rev, holdTime);
    break;
  }

bail:
  if (ast){
    setIntegerParam(c_p_->motorStatusProblem_, 1);
    setIntegerParam(c_p_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

asynStatus SmarActMCSAxis::moveVelocity(double min_vel, double max_vel, double accel)
{
  asynStatus ast;
  epicsInt32 speed = (epicsInt32)rint(fabs(max_vel));
  epicsInt32 tgt_pos;
  signed char dir = 1;

  /* No MCS command we an use directly. Just use a 'relative move' to
   * very far target.
   */

  DBG_PRINTF("SmarActMCSAxis::moveVelocity: min_vel:%g max_vel:%g\n", min_vel, max_vel);

  if (0 == speed){
    /* Here we are in a dilemma. If we set the MCS' speed to zero
     * then it will move at unlimited speed which is so fast that
     * 'JOG' makes no sense.
     * Just 'STOP' the motion - hope that works...
     */
    setIntegerParam(c_p_->motorStop_, 1);
    callParamCallbacks();
    return asynSuccess;
  }

  if (max_vel < 0){
    dir = -1;
  }

  if ((ast = setSpeed(max_vel)))
    goto bail;

  switch(devType_){
  case DT_NO_SENSOR:
    break;
  case DT_LIN:
    tgt_pos = FAR_AWAY_LIN * dir;
    ast = moveCmd(":MPR%u,%ld,0", channel_, tgt_pos);
    break;
  case DT_ROT:
    tgt_pos = FAR_AWAY_ROT * dir;
    ast = moveCmd(":MAR%u,0,%ld,0", channel_, tgt_pos);
    break;
  }

bail:
  if (ast) {
    setIntegerParam(c_p_->motorStatusProblem_, 1);
    setIntegerParam(c_p_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

asynStatus SmarActMCSAxis::home(double min_vel, double max_vel, double accel, int forwards)
{
  asynStatus ast;
  int holdTime;
  int autoZero;
  DBG_PRINTF("SmarActMCSAxis::home: forward:%u\n", forwards);

  if ((ast = setSpeed(max_vel)))
    goto bail;

  c_p_->getIntegerParam(axisNo_, c_p_->autoZero_, &autoZero);
  c_p_->getIntegerParam(axisNo_, c_p_->holdTime_, &holdTime);

  ast = moveCmd(":FRM%u,%u,%d,%d", channel_, forwards ? 0 : 1, holdTime, autoZero);

bail:
  if (ast) {
    setIntegerParam(c_p_->motorStatusProblem_, 1);
    setIntegerParam(c_p_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

asynStatus SmarActMCSAxis::stop(double acceleration)
{
  asynStatus ast;
  DBG_PRINTF("SmarActMCSAxis::stop:\n");
  ast = moveCmd(":S%u", channel_);

  if (ast) {
    setIntegerParam(c_p_->motorStatusProblem_, 1);
    setIntegerParam(c_p_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

asynStatus SmarActMCSAxis::poll(bool *moving_p)
{
  asynStatus ast;
  epicsInt32 pos=0;
  epicsInt32 val;
  epicsInt32 angle;
  int rev;
  enum SmarActMCSStatus status;

  switch(devType_){
  case DT_NO_SENSOR:
    break;
  case DT_LIN:
    if ((ast = getVal("GP", (int *)&pos)))
    break;
  case DT_ROT:
    if ((ast = getAngle(&angle, &rev)))
      goto bail;
    // Convert angle and revs to total angle
    pos = rev * UDEG_PER_REV + angle;
    break;
  }

  setDoubleParam(c_p_->motorEncoderPosition_, (double)pos);
  setDoubleParam(c_p_->motorPosition_, (double)pos);

  if ((ast = getVal("GS", &val)))
    goto bail;

  status = (enum SmarActMCSStatus)val;

  switch (status) {
  case Stepping:
  case Scanning:
  case Targeting:
  case MoveDelay:
  case Calibrating:
  case FindRefMark:
    *moving_p = true;
    break;

  case Holding:
  default:
    *moving_p = false;
    break;
  }

  setIntegerParam(c_p_->motorStatusDone_, !*moving_p);

  /* Check if the sensor 'knows' absolute position and
   * update the MSTA 'HOMED' bit.
   */
  if ((ast = getVal("GPPK", &val)))
    goto bail;

  setIntegerParam(c_p_->motorStatusHomed_, val ? 1 : 0);

  // Get currently set positioner type
  if ((ast = getVal("GST", &val)))
    goto bail;
  setIntegerParam(c_p_->ptyprb_, val);

bail:
  setIntegerParam(c_p_->motorStatusProblem_, ast ? 1 : 0);
  setIntegerParam(c_p_->motorStatusCommsError_, ast ? 1 : 0);

  callParamCallbacks();
  //DBG_PRINTF("SmarActMCSAxis::poll: position:%d status:%u", pos,status);

  return ast;
}

asynStatus SmarActMCSAxis::setPosition(double position)
{
  asynStatus ast=asynSuccess;
  double rpos;

  rpos = rint(position);

  switch(devType_){
  case DT_NO_SENSOR:
    break;
  case DT_LIN:
    ast = moveCmd(":SP%u,%d", channel_, (epicsInt32)rpos);
    break;
  case DT_ROT:
    // For rotation stages the revolution will always be set to zero
    // Only set position if it is between zero an 360 degrees
    if (rpos >= 0.0 && rpos < (double)UDEG_PER_REV) {
      ast = moveCmd(":SP%u,%d", channel_, (epicsInt32)rpos);
    } else {
      ast = asynError;
    }
    break;
  }

  if (ast) {
    setIntegerParam(c_p_->motorStatusProblem_, 1);
    setIntegerParam(c_p_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

//--------------- own methods ---------------

/* Check if the positioner type set on the controller
 * is linear or rotary and set the isRot_ parameter correctly.
 */
void SmarActMCSAxis::checkType()
{
  int val,rev;
  // Attempt to check linear position, if we receive
  // an error, we're a rotary motor.
  if (asynSuccess ==  getVal("GP", &val)) {
    devType_=DT_LIN;
  }
  else if (asynSuccess ==  getAngle(&val, &rev)) {
    devType_=DT_ROT;
    setIntegerParam(c_p_->motorStatusHasEncoder_, 1);
    setIntegerParam(c_p_->motorStatusGainSupport_, 1);
  }
  else {
    devType_=DT_NO_SENSOR;
    setIntegerParam(c_p_->motorStatusHasEncoder_, 1);
    setIntegerParam(c_p_->motorStatusGainSupport_, 1);
  }
  return;
}

/* Read a parameter from the MCS (nothing to do with asyn's parameter
 * library).
 *
 * parm_cmd: MCS command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the MCS
 *
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus SmarActMCSAxis::getVal(const char *parm_cmd, int *val_p)
{
  asynStatus ast;
  char rep[REP_LEN];
  int ax;
  // asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "getVal() cmd=:%s%u", parm_cmd, this->channel_);
  ast = c_p_->sendCmd1(rep, sizeof(rep), ":%s%u", parm_cmd, this->channel_);
  if (ast)
    return ast;
  return c_p_->parseReply(rep, &ax, val_p) ? asynError : asynSuccess;
}

/* Read the position of rotation stage
 *
 * parm_cmd: MCS command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the MCS
 *
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus SmarActMCSAxis::getAngle(int *val_p, int *rev_p)
{
  asynStatus ast;
  char rep[REP_LEN];
  int ax;

  // asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "getAngle() cmd=:%s%u", parm_cmd, this->channel_);

  ast = c_p_->sendCmd1(rep, sizeof(rep), ":GA%u", this->channel_);
  if (ast)
    return ast;
  return c_p_->parseAngle(rep, &ax, val_p, rev_p) ? asynError : asynSuccess;
}

asynStatus SmarActMCSAxis::moveCmd(const char *fmt, ...)
{
  asynStatus ast;
  int val, ax;
  char rep[REP_LEN];
  size_t got;
  double tout = DEFLT_TIMEOUT;
  va_list ap;

  va_start(ap, fmt);
  ast = c_p_->sendCmd3(&got, rep, sizeof(rep), tout, fmt, ap);
  va_end(ap);

  if (ast)
    goto bail;

  if (c_p_->parseReply(rep, &ax, &val)) {
    ast = asynError;
    goto bail;
  }

bail:

  return ast;
}

asynStatus SmarActMCSAxis::setSpeed(double velocity)
{
  epicsInt32 vel;
  asynStatus status;

  // ignore set speed commands if flag is set
  if (c_p_->disableSpeed_)
    return asynSuccess;

  if ((vel = (epicsInt32)rint(fabs(velocity))) != vel_) {
    /* change speed */
    if (asynSuccess == (status = moveCmd(":SCLS%u,%ld", channel_, vel))) {
      vel_ = vel;
    }
    return status;
  }
  return asynSuccess;
}

//------------------------------------------------------------------

SmarActMCSController::SmarActMCSController(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int disableSpeed)
    : asynMotorController(portName, numAxes,
                          0, // parameters
                          0, // interface mask
                          0, // interrupt mask
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0), // default priority and stack size
       asynUserMot_p_(0)
{
  asynStatus status;
  char junk[100];
  size_t got_junk;
  int eomReason;
  pAxes_ = (SmarActMCSAxis **)(asynMotorController::pAxes_);
  disableSpeed_ = disableSpeed;
  if (disableSpeed_)
    epicsPrintf("SmarActMCSController(%s): WARNING - The speed set commands have been disabled for this controller\n", portName);

  createParam(MCSPtypString, asynParamInt32, &this->ptyp_);
  createParam(MCSPtypRbString, asynParamInt32, &this->ptyprb_);
  createParam(MCSAutoZeroString, asynParamInt32, &this->autoZero_);
  createParam(MCSHoldTimeString, asynParamInt32, &this->holdTime_);
  createParam(MCSSclfString, asynParamInt32, &this->sclf_);
  createParam(MCSCalString, asynParamInt32, &this->cal_);

  status = pasynOctetSyncIO->connect(IOPortName, 0, &asynUserMot_p_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "SmarActMCSController:SmarActMCSController: cannot connect to MCS controller\n");
  }

  // slurp away any initial telnet negotiation; there is no guarantee that
  // the other end will not send some telnet chars in the future. The terminal
  // server should really be configured to 'raw' mode!
  pasynOctetSyncIO->read(asynUserMot_p_, junk, sizeof(junk), 2.0, &got_junk, &eomReason);
  if (got_junk) {
    epicsPrintf("SmarActMCSController(%s): WARNING - detected unexpected characters on link (%s); make sure you have a RAW (not TELNET) connection\n", portName, IOPortName);
  }

  pasynOctetSyncIO->setInputEos(asynUserMot_p_, "\n", 1);
  pasynOctetSyncIO->setOutputEos(asynUserMot_p_, "\n", 1);

  // Create axes
  /*  for ( ax=0; ax<numAxes; ax++ ) {
      //axis_p = new SmarActMCSAxis(this, ax);
      pAxes_[ax] = new SmarActMCSAxis(this, ax);
    }
  */
  // move to iocsh function smarActMCSCreateAxis()

  // FIXME the 'forcedFastPolls' may need to be set if the 'sleep/wakeup' feature
  //       of the sensor/readback is used.
  startPoller(movingPollPeriod, idlePollPeriod, 0);
}

/* Parse reply from MCS and return the value converted to a number.
 *
 * If the string cannot be parsed, i.e., is not in the format
 *  ':' , <string_of_upper_case_letters> , <number1> , ',' , <number2>
 *
 * then the routine returns '-1'.
 *
 * Otherwise, if <string_of_upper_case_letters> starts with 'E'
 * (which means an 'Error' code) then the (always non-negative)
 * error code is returned (may be zero in case of an 'acknowledgement'
 * message in synchronous mode).
 *
 * If the string is parsed successfully then <number2> is passed up
 * in *val_p.
 *
 * Hence - return code nonzero -> error (error code if > 0, parse error
 *                                otherwise)
 *       - return code zero    -> successful ACK or command reply; value
 *                                in *val_p.
 */
int SmarActMCSController::parseReply(const char *reply, int *ax_p, int *val_p)
{
  char cmd[10];
  if (3 != sscanf(reply, ":%10[A-Z]%i,%i", cmd, ax_p, val_p))
    return -1;
  return 'E' == cmd[0] ? *val_p : 0;
}

/* Parse angle from MCS and return the value converted to a number.
 *
 * If the string cannot be parsed, i.e., is not in the format
 *  ':' , <string_of_upper_case_letters> , <number1> , ',' , <number2> , ',' , <number3>
 *
 * then the routine returns '-1'.
 *
 * Otherwise, if <string_of_upper_case_letters> starts with 'E'
 * (which means an 'Error' code) then the (always non-negative)
 * error code is returned (may be zero in case of an 'acknowledgement'
 * message in synchronous mode).
 *
 * If the string is parsed successfully then <number2> is passed up
 * in *val_p.
 *
 * Hence - return code nonzero -> error (error code if > 0, parse error
 *                                otherwise)
 *       - return code zero    -> successful ACK or command reply; value
 *                                in *val_p.
 */
int SmarActMCSController::parseAngle(const char *reply, int *ax_p, int *val_p, int *rot_p)
{
  char cmd[10];
  if (4 != sscanf(reply, ":%10[A-Z]%i,%i,%i", cmd, ax_p, val_p, rot_p))
    return -1;
  // Will this ever get called? An error response fewer values than an angle response
  return 'E' == cmd[0] ? *val_p : 0;
}

asynStatus SmarActMCSController::sendCmd4(size_t *got_p, char *rep, int len, double timeout, const char *fmt, va_list ap)
{
  char buf[CMD_LEN];
  size_t nwrite;
  int eomReason;
  asynStatus status;

  epicsVsnprintf(buf, sizeof(buf), fmt, ap);

  status = pasynOctetSyncIO->writeRead(asynUserMot_p_, buf, strlen(buf), rep, len, timeout, &nwrite, got_p, &eomReason);

  //DBG_PRINTF("SmarActMCSController::sendCmd: %s -> %d\n", buf,status);
  // asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "sendCmd()=%s", buf);
  return status;
}

asynStatus SmarActMCSController::sendCmd3(size_t *got_p, char *rep, int len, double timeout, const char *fmt, ...)
{
  va_list ap;
  asynStatus status;
  va_start(ap, fmt);
  status = sendCmd4(got_p, rep, len, timeout, fmt, ap);
  va_end(ap);
  return status;
}

asynStatus SmarActMCSController::sendCmd2(size_t *got_p, char *rep, int len, const char *fmt, ...)
{
  va_list ap;
  asynStatus status;
  va_start(ap, fmt);
  status = sendCmd4(got_p, rep, len, DEFLT_TIMEOUT, fmt, ap);
  va_end(ap);
  return status;
}

asynStatus SmarActMCSController::sendCmd1(char *rep, int len, const char *fmt, ...)
{
  va_list ap;
  asynStatus status;
  size_t got;
  va_start(ap, fmt);
  status = sendCmd4(&got, rep, len, DEFLT_TIMEOUT, fmt, ap);
  va_end(ap);
  return status;
}

/** Called when asyn clients call pasynInt32->write().
 * Extracts the function and axis number from pasynUser.
 * Sets the value in the parameter library.
 * For all other functions it calls asynMotorController::writeInt32.
 * Calls any registered callbacks for this pasynUser->reason and address.
 * \param[in] pasynUser asynUser structure that encodes the reason and address.
 * \param[in] value     Value to write. */
asynStatus SmarActMCSController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  char rep[REP_LEN];
  int val, ax;
  SmarActMCSAxis *pAxis = static_cast<SmarActMCSAxis *>(getAxis(pasynUser));

  if (!pAxis) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "SmarActMCSController:writeInt32: error, function: %i. Invalid axis number.\n", function);
    return asynError;
  }

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);

  if (function == ptyp_) {
    // set positioner type
    status = sendCmd1(rep, sizeof(rep), ":SST%i,%i", pAxis->channel_, value);
    if (status) return status;
    if (parseReply(rep, &ax, &val)) return asynError;
    pAxis->checkType();
  }
  else if (function == cal_) {
    // send calibration command
    status = sendCmd1(rep, sizeof(rep), ":CS%i", pAxis->channel_);
    if (status) return status;
    if (parseReply(rep, &ax, &val)) return asynError;
  }
  else if (function == sclf_) {
    // set piezo MaxClockFreq
    status = sendCmd1(rep, sizeof(rep), ":SCLF%i,%i", pAxis->channel_, value);
    if (status) return status;
    if (parseReply(rep, &ax, &val)) return asynError;
  }
  else {
    /* Call base class method */
    status = asynMotorController::writeInt32(pasynUser, value);
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->channel_);
  if (status)
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
          "SmarActMCSController:writeInt32: error, status=%d function=%d, value=%d\n",
          status, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
          "SmarActMCSController:writeInt32: function=%d, value=%d\n",
          function, value);
  return status;
}

//------------------------------------------------------------------

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
static const iocshArg cc_a0 = {"Port name [string]", iocshArgString};
static const iocshArg cc_a1 = {"I/O port name [string]", iocshArgString};
static const iocshArg cc_a2 = {"Number of axes [int]", iocshArgInt};
static const iocshArg cc_a3 = {"Moving poll period (s) [double]", iocshArgDouble};
static const iocshArg cc_a4 = {"Idle poll period (s) [double]", iocshArgDouble};
static const iocshArg cc_a5 = {"Disable speed cmds [int]", iocshArgInt};

static const iocshArg *const cc_as[] = {&cc_a0, &cc_a1, &cc_a2, &cc_a3, &cc_a4, &cc_a5};

static const iocshFuncDef cc_def = {"smarActMCSCreateController", sizeof(cc_as) / sizeof(cc_as[0]), cc_as};

extern "C" void *
smarActMCSCreateController(
  const char *motorPortName,
  const char *ioPortName,
  int numAxes,
  double movingPollPeriod,
  double idlePollPeriod,
  int disableSpeed)
{
  void *rval = 0;
  // the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
  // if constructing a controller (or axis) incurs an exception even if its
  // caught (IMHO asyn should behave as if the controller/axis never existed...)
#ifdef ASYN_CANDO_EXCEPTIONS
  try {
#endif
    rval = new SmarActMCSController(motorPortName, ioPortName, numAxes, movingPollPeriod, idlePollPeriod, disableSpeed);
#ifdef ASYN_CANDO_EXCEPTIONS
  } catch (SmarActMCSException &e) {
    epicsPrintf("smarActMCSCreateController failed (exception caught):\n%s\n", e.what());
    rval = 0;
  }
#endif

  return rval;
}

static void cc_fn(const iocshArgBuf *args)
{
  smarActMCSCreateController(
    args[0].sval,
    args[1].sval,
    args[2].ival,
    args[3].dval,
    args[4].dval,
    args[5].ival);
}

static const iocshArg ca_a0 = {"Controller Port name [string]", iocshArgString};
static const iocshArg ca_a1 = {"Axis number [int]", iocshArgInt};
static const iocshArg ca_a2 = {"Channel [int]", iocshArgInt};

static const iocshArg *const ca_as[] = {&ca_a0, &ca_a1, &ca_a2};

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
/* smarActMCSCreateAxis called to create each axis of the smarActMCS controller*/
static const iocshFuncDef ca_def = {"smarActMCSCreateAxis", 3, ca_as};

extern "C" void *
smarActMCSCreateAxis(
  const char *controllerPortName,
  int axisNumber,
  int channel)
{
  void *rval = 0;

  SmarActMCSController *pC;
  //SmarActMCSAxis *pAxis;
  asynMotorAxis *pAsynAxis;

  // the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
  // if constructing a controller (or axis) incurs an exception even if its
  // caught (IMHO asyn should behave as if the controller/axis never existed...)
#ifdef ASYN_CANDO_EXCEPTIONS
  try {
#endif
    //		rval = new SmarActMCSAxis(, axisNumber, channel);
    pC = (SmarActMCSController *)findAsynPortDriver(controllerPortName);
    if (!pC) {
      printf("smarActMCSCreateAxis: Error port %s not found\n", controllerPortName);
      rval = 0;
      return rval;
    }
    // check if axis number already exists
    pAsynAxis = pC->getAxis(axisNumber);
    if (pAsynAxis != NULL) { // axis already exists
      epicsPrintf("SmarActMCSCreateAxis failed:axis %u already exists\n", axisNumber);
#ifdef ASYN_CANDO_EXCEPTIONS
      THROW_(SmarActMCSException(MCSCommunicationError, "axis %u already exists", axisNumber));
#endif
      rval = 0;
      return rval;
    }
    pC->lock();
    /*pAxis =*/ new SmarActMCSAxis(pC, axisNumber, channel);
    //pAxis = NULL;
    pC->unlock();

#ifdef ASYN_CANDO_EXCEPTIONS
  } catch (SmarActMCSException &e) {
    epicsPrintf("SmarActMCSAxis failed (exception caught):\n%s\n", e.what());
    rval = 0;
  }
#endif

  return rval;
}

static void ca_fn(const iocshArgBuf *args)
{
  smarActMCSCreateAxis(
    args[0].sval,
    args[1].ival,
    args[2].ival);
}

static void smarActMCSMotorRegister(void)
{
  iocshRegister(&cc_def, cc_fn); // smarActMCSCreateController
  iocshRegister(&ca_def, ca_fn); // smarActMCSCreateAxis
}

extern "C" {
  epicsExportRegistrar(smarActMCSMotorRegister);
}
