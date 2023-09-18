
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

SmarActMCSAxis::SmarActMCSAxis(class SmarActMCSController *pC, int axis, int channel)
  : asynMotorAxis(pC, axis), pC_(pC)
{
  asynStatus ast;
  int val;
  channel_ = channel;

  asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "SmarActMCSAxis::SmarActMCSAxis -- creating axis %u\n", axis);

  ast = getVal("GCLS", &vel_);
  if (ast)
    goto bail;
  if ((ast = getVal("GS", &val)))
    goto bail;

  setIntegerParam(pC_->autoZero_, 1);
  setIntegerParam(pC_->holdTime_, 0);

  checkType();

bail:
  setIntegerParam(pC_->motorStatusProblem_, ast ? 1 : 0);
  setIntegerParam(pC_->motorStatusCommsError_, ast ? 1 : 0);

  callParamCallbacks();

  if (ast) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "SmarActMCSAxis::SmarActMCSAxis -- channel %u ASYN error %i", axis, ast);
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

  pC_->getIntegerParam(axisNo_, pC_->holdTime_, &holdTime);
  if ((ast = setSpeed(max_vel)))
    goto bail;

  rpos = lround(position);

  switch(devType_){
  case DT_NO_SENSOR:
    //TODO: support sensorless positioners
    printf("SmarActMCSAxis::move()@%d:sensorless positioners not supported\n",channel_);
    break;
  case DT_LIN:
    fmt = relative ? ":MPR%u,%ld,%d" : ":MPA%u,%ld,%d";
    ast = pC_->cmdWriteRead(1,fmt, channel_, rpos, holdTime);
    break;
  case DT_ROT:
    fmt = relative ? ":MAR%u,%ld,%d,%d" : ":MAA%u,%ld,%d,%d";
    angle = (epicsInt32)rpos % UDEG_PER_REV;
    rev = (int)(rpos / UDEG_PER_REV);
    if (angle < 0){
      angle += UDEG_PER_REV;
      rev -= 1;
    }
    ast = pC_->cmdWriteRead(1,fmt, channel_, angle, rev, holdTime);
    break;
  }
bail:
  if (ast){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
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
    setIntegerParam(pC_->motorStop_, 1);
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
    //TODO: support sensorless positioners
    printf("SmarActMCSAxis::moveVelocity()@%d:sensorless positioners not supported\n",channel_);
    break;
  case DT_LIN:
    tgt_pos = FAR_AWAY_LIN * dir;
    ast = pC_->cmdWriteRead(1,":MPR%u,%ld,0", channel_, tgt_pos);
    break;
  case DT_ROT:
    tgt_pos = FAR_AWAY_ROT * dir;
    ast = pC_->cmdWriteRead(1,":MAR%u,0,%ld,0", channel_, tgt_pos);
    break;
  }

bail:
  if (ast) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
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

  pC_->getIntegerParam(axisNo_, pC_->autoZero_, &autoZero);
  pC_->getIntegerParam(axisNo_, pC_->holdTime_, &holdTime);

  ast = pC_->cmdWriteRead(1,":FRM%u,%u,%d,%d", channel_, forwards ? 0 : 1, holdTime, autoZero);
bail:
  if (ast) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

asynStatus SmarActMCSAxis::stop(double acceleration)
{
  asynStatus ast;
  DBG_PRINTF("SmarActMCSAxis::stop:\n");
  ast = pC_->cmdWriteRead(1,":S%u", channel_);

  if (ast) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

asynStatus SmarActMCSAxis::poll(bool *moving_p)
{
  asynStatus ast;
  int val,pos=0;
  enum SmarActMCSStatus status;

  switch(devType_){
  case DT_NO_SENSOR:
    //TODO: support sensorless positioners
    break;
  case DT_LIN:
    if ((ast=getVal("GP", &pos)))
      goto bail;
    break;
  case DT_ROT:
    if ((ast=getAngle(&pos)))
      goto bail;
    break;
  }

  setDoubleParam(pC_->motorEncoderPosition_, (double)pos);
  setDoubleParam(pC_->motorPosition_, (double)pos);

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

  setIntegerParam(pC_->motorStatusDone_, !*moving_p);

  /* Check if the sensor 'knows' absolute position and
   * update the MSTA 'HOMED' bit.
   */
  if ((ast = getVal("GPPK", &val)))
    goto bail;

  setIntegerParam(pC_->motorStatusHomed_, val ? 1 : 0);

  // Get currently set positioner type
  if ((ast = getVal("GST", &val)))
    goto bail;
  setIntegerParam(pC_->ptyprb_, val);

bail:
  setIntegerParam(pC_->motorStatusProblem_, ast ? 1 : 0);
  setIntegerParam(pC_->motorStatusCommsError_, ast ? 1 : 0);

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
    //TODO: support sensorless positioners
    printf("SmarActMCSAxis::setPosition()@%d:sensorless positioners not supported\n",channel_);
    break;
  case DT_LIN:
    ast = pC_->cmdWriteRead(1,":SP%u,%d", channel_, (epicsInt32)rpos);

    break;
  case DT_ROT:
    // For rotation stages the revolution will always be set to zero
    // Only set position if it is between zero an 360 degrees
    if (rpos >= 0.0 && rpos < (double)UDEG_PER_REV) {
      ast = pC_->cmdWriteRead(1,":SP%u,%d", channel_, (epicsInt32)rpos);
    } else {
      ast = asynError;
    }
    break;
  }

  if (ast) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
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
  int val;
  // Attempt to check linear position, if we receive
  // an error, we're a rotary motor.
  if (asynSuccess ==  getVal("GP", &val)) {
    devType_=DT_LIN;
    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);
  }
  else if (asynSuccess ==  getAngle(&val)) {
    devType_=DT_ROT;
    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);
  }
  else {
    devType_=DT_NO_SENSOR;
  }
  DBG_PRINTF("SmarActMCSAxis::checkType() @%d -> %d\n", channel_,devType_);
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
  int ax;
  // asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "getVal() cmd=:%s%u", parm_cmd, this->channel_);
  ast = pC_->cmdWriteRead(0, ":%s%u", parm_cmd, this->channel_);
  if (ast)
    return ast;
  return pC_->parse2Val(&ax, val_p) ? asynError : asynSuccess;
}

/* Read the position of rotation stage
 *
 * parm_cmd: MCS command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the MCS
 *
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus SmarActMCSAxis::getAngle(int *val_p)
{
  asynStatus ast;
  int ax,val,rev,res;
  ast = pC_->cmdWriteRead(0, ":GA%u", this->channel_);
  if (ast)
    return ast;
  res=pC_->parse3Val(&ax, &val, &rev);
  if (res)
    ast=asynError;
  else
    *val_p = val+ rev * UDEG_PER_REV;
  return ast;
}

asynStatus SmarActMCSAxis::setSpeed(double velocity)
{
  printf("SmarActMCSAxis::setSpeed %d -> %g\n",channel_,velocity);
  epicsInt32 ax,vel;
  asynStatus ast;
  vel = (epicsInt32)rint(fabs(velocity));
  if (vel == vel_)
    return asynSuccess; //nothing to do
  ast = pC_->cmdWriteRead(1,":SCLS%u,%ld", channel_, vel);
  if (ast!=asynSuccess)
    return ast;
  if (pC_->parse2Val(&ax, &vel))
    return asynError;
  vel_ = vel;
  return ast;
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
       pAsynUserMot_(0)
{
  asynStatus status;
  char junk[100];
  size_t got_junk;
  int eomReason;
  pAxes_ = (SmarActMCSAxis **)(asynMotorController::pAxes_);
  if (disableSpeed)
    epicsPrintf("SmarActMCSController(%s): WARNING - The disableSpeed is not supported any more\n", portName);

  createParam(MCSPtypString, asynParamInt32, &this->ptyp_);
  createParam(MCSPtypRbString, asynParamInt32, &this->ptyprb_);
  createParam(MCSAutoZeroString, asynParamInt32, &this->autoZero_);
  createParam(MCSHoldTimeString, asynParamInt32, &this->holdTime_);
  createParam(MCSSclfString, asynParamInt32, &this->sclf_);
  createParam(MCSCalString, asynParamInt32, &this->cal_);

  status = pasynOctetSyncIO->connect(IOPortName, 0, &pAsynUserMot_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "SmarActMCSController:SmarActMCSController: cannot connect to MCS controller\n");
  }

  // slurp away any initial telnet negotiation; there is no guarantee that
  // the other end will not send some telnet chars in the future. The terminal
  // server should really be configured to 'raw' mode!
  pasynOctetSyncIO->read(pAsynUserMot_, junk, sizeof(junk), 2.0, &got_junk, &eomReason);
  if (got_junk) {
    epicsPrintf("SmarActMCSController(%s): WARNING - detected unexpected characters on link (%s); make sure you have a RAW (not TELNET) connection\n", portName, IOPortName);
  }

  pasynOctetSyncIO->setInputEos(pAsynUserMot_, "\n", 1);
  pasynOctetSyncIO->setOutputEos(pAsynUserMot_, "\n", 1);

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


// Parse reply from MCS and return the value converted to a number.
// expected format:
// :<COMMAND><val1>,<val2>
// If the string cannot be parsed, i.e., is not in the format
// then the routine returns '-1'
// if <COMMAND> starts with 'E'
// (which means an 'Error' code) then the (always non-negative)
// error code is returned (may be zero in case of an 'acknowledgement'
// If the string is parsed successfully then <value> is passed up
// in *val and 0 is returned
int SmarActMCSController::parse2Val(int *val1, int *val2)
{
  char cmd[10];
  if (3 != sscanf(outString_, ":%10[A-Z]%i,%i", cmd, val1, val2))
    return -1;
  return 'E' == cmd[0] ? *val2 : 0;
}

// Parse reply from MCS and return the value converted to a number.
// expected format:
// :<COMMAND><val1>,<val2>,<val3>
// If the string cannot be parsed, i.e., is not in the format
// then the routine returns '-1'
// if <COMMAND> starts with 'E'
// (which means an 'Error' code) then the (always non-negative)
// error code is returned (may be zero in case of an 'acknowledgement'
// If the string is parsed successfully then <value> is passed up
// in *val and 0 is returned
int SmarActMCSController::parse3Val(int *val1, int *val2, int *val3)
{
  char cmd[10];
  if (4 != sscanf(outString_, ":%10[A-Z]%i,%i,%i", cmd, val1, val2, val3))
    return -1;
  return 'E' == cmd[0] ? *val2 : 0;
}

//formatted write string to member outString_
//reads returned string in outString_
//if dbg !=0 the strings are printed to the console
asynStatus SmarActMCSController::cmdWriteRead(bool dbg,const char *fmt, ...)
{
  asynStatus ast;
  va_list ap;
  va_start(ap, fmt);
  epicsVsnprintf(outString_, sizeof(outString_), fmt, ap);
  va_end(ap);
#ifdef DEBUG
  if(dbg)
    printf("SmarActMCSController::cmdWriteRead: %s -> ",outString_);
#endif
  {
    size_t nwrite,nread;
    int eomReason;
    ast = pasynOctetSyncIO->writeRead(pAsynUserMot_, outString_, strlen(outString_), outString_,  sizeof(outString_), DEFLT_TIMEOUT, &nwrite, &nread, &eomReason);
  }
  //ast = this->writeReadController();
#ifdef DEBUG
  if(dbg)
    puts(outString_);
#endif
  return ast;
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
  int val, ax;
  SmarActMCSAxis *pAxis = static_cast<SmarActMCSAxis *>(getAxis(pasynUser));

  if (!pAxis) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "SmarActMCSController:writeInt32: error, function: %i. Invalid axis number.\n", function);
    return asynError;
  }

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  //status = setIntegerParam(pAxis->axisNo_, function, value);

  if (function == ptyp_) {
    // set positioner type
    status = cmdWriteRead(1,":SST%i,%i", pAxis->channel_, value);
    if (status) return status;
    if (parse2Val(&ax, &val)) return asynError;
    pAxis->checkType();
  }
  else if (function == cal_) {
    // send calibration command
    status = cmdWriteRead(1, ":CS%i", pAxis->channel_);
    if (status) return status;
    if (parse2Val(&ax, &val)) return asynError;
  }
  else if (function == sclf_) {
    // set piezo MaxClockFreq
    status = cmdWriteRead(1, ":SCLF%i,%i", pAxis->channel_, value);
    if (status) return status;
    if (parse2Val(&ax, &val)) return asynError;
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
  rval = new SmarActMCSController(motorPortName, ioPortName, numAxes, movingPollPeriod, idlePollPeriod, disableSpeed);
  return rval;
}

static void cc_fn(const iocshArgBuf *args)
{
  smarActMCSCreateController( args[0].sval, args[1].sval, args[2].ival, args[3].dval, args[4].dval, args[5].ival);
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
  // rval = new SmarActMCSAxis(, axisNumber, channel);
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
    rval = 0;
    return rval;
  }
  pC->lock();
  /*pAxis =*/ new SmarActMCSAxis(pC, axisNumber, channel);
  //pAxis = NULL;
  pC->unlock();

  return rval;
}

static void ca_fn(const iocshArgBuf *args)
{
  smarActMCSCreateAxis( args[0].sval, args[1].ival, args[2].ival);
}

static void smarActMCSMotorRegister(void)
{
  iocshRegister(&cc_def, cc_fn); // smarActMCSCreateController
  iocshRegister(&ca_def, ca_fn); // smarActMCSCreateAxis
}

extern "C" {
  epicsExportRegistrar(smarActMCSMotorRegister);
}
