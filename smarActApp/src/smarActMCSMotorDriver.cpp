/*--------------------------------------------------------------*
 * Filename     smarActMCSMotorDriver.cpp                       *
 * Usage        Motor driver support for smarAct MCS Controller *
 * Author       Thierry Zamofing <thierry.zamofing@psi.ch>      *
 *              Adapted from David Vine Jan 19, 2019            *
 *              Till Straumann <strauman@slac.stanford.edu>     *
 *--------------------------------------------------------------*/

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

#ifdef DEBUG
  #define DBG_PRINTF(dbg,...) if(dbg) printf(__VA_ARGS__)
#else
  #define DBG_PRINTF(dbg,...)
#endif

#define CMD_LEN 50
#define REP_LEN 50
#define DEFLT_TIMEOUT 2.0

#define FAR_AWAY_LIN 1000000000 /*nm*/
#define FAR_AWAY_ROT 32767  /*revolutions*/
#define UDEG_PER_REV 360000000

#define _countof(arr) sizeof(arr) / sizeof(arr[0])

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

//------------------------------------------------------------------

// Creates a new controller object.
// \param[in] ctrlPort             name of the port that will be created for this driver
// \param[in] asynPort             name of the drvAsynIPPPort that was created previously
// \param[in] numAxes              number of axes that this controller supports
// \param[in] movingPollPeriod     time in ms between polls when any axis is moving
// \param[in] idlePollPeriod       time in ms between polls when no axis is moving
MCSController::MCSController(const char *ctrlPort, const char *asynPort, int numAxes, double movingPollPeriod, double idlePollPeriod, int dbgLvl)
    : asynMotorController(ctrlPort, numAxes,
                          0, // parameters
                          0, // interface mask
                          0, // interrupt mask
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // default priority and stack size
{
  asynStatus status;
  int axis;

  createParam(MCSPtypString,     asynParamInt32, &this->ptyp_);
  createParam(MCSPtypRbString,   asynParamInt32, &this->ptyprb_);
  createParam(MCSMclfString,     asynParamInt32, &this->mclf_);
  createParam(MCSHoldString,     asynParamInt32, &this->hold_);
  createParam(MCSCalString,      asynParamInt32, &this->cal_);
  createParam(MCSAutoZeroString, asynParamInt32, &this->autoZero_);

  status = pasynOctetSyncIO->connect(asynPort, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "MCSController:MCSController: cannot connect to MCS controller\n");
  }
  pasynOctetSyncIO->setInputEos(pasynUserController_, "\n", 1);
  pasynOctetSyncIO->setOutputEos(pasynUserController_, "\n", 1);

  if(dbgLvl&0x01)
  {
    cmdWriteRead(1,":GSI"); //Get System Id
    cmdWriteRead(1,":GIV"); //Get Interface Version
    cmdWriteRead(1,":GNC"); //Get Number of Channels
  }

  // Create the axis objects
  for(axis=0; axis<numAxes; axis++){
    new MCSAxis(this, axis,dbgLvl);
  }

  //User specifies poll periods in milliseconds, starPoller expects seconds
  startPoller(movingPollPeriod/1000., idlePollPeriod/1000., 0);
}

// Called when asyn clients call pasynInt32->write().
// Extracts the function and axis number from pasynUser.
// Sets the value in the parameter library.
// For all other functions it calls asynMotorController::writeInt32.
// Calls any registered callbacks for this pasynUser->reason and address.
// \param[in] pasynUser asynUser structure that encodes the reason and address.
// \param[in] value     Value to write. */
asynStatus MCSController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus ast = asynSuccess;
  int val, ax;
  MCSAxis *pAxis = static_cast<MCSAxis *>(getAxis(pasynUser));

  if (!pAxis) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "MCSController:writeInt32: error, function: %i. Invalid axis number.\n", function);
    return asynError;
  }

  // Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
  // status at the end, but that's OK
  ast = setIntegerParam(pAxis->axisNo_, function, value);

  if (function == ptyp_) {
    // set positioner type
    int curVal;
    ast = pAxis->getVal(pAxis->dbgLvl_&0x01,"GST", &curVal);
    if(curVal!=value)
    { // only set value if it changed -> :SST sets GPPK to 0, so homing is lost -> this avoids to loose homing if ioc is restarted
      ast = cmdWriteRead(pAxis->dbgLvl_&0x08,":SST%i,%i", pAxis->axisNo_, value);
      if (ast) return ast;
      if (parse2Val(&ax, &val)) return asynError;
      pAxis->checkType();
    }
  }
  else if (function == cal_) {
    // send calibration command
    ast = cmdWriteRead(pAxis->dbgLvl_&0x08, ":CS%i", pAxis->axisNo_);
    if (ast) return ast;
    if (parse2Val(&ax, &val)) return asynError;
  }
  else if (function == mclf_) {
    // set piezo MaxClockFreq
    ast= cmdWriteRead(pAxis->dbgLvl_&0x08, ":SCLF%i,%i", pAxis->axisNo_, value);
    if (ast) return ast;
    if (parse2Val(&ax, &val)) return asynError;
  }
  else if (function == autoZero_) {
  }
  else {
    // Call base class method
    ast = asynMotorController::writeInt32(pasynUser, value);
  }

  // Do callbacks so higher layers see any changes
  callParamCallbacks(pAxis->axisNo_);
  if (ast)
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
          "MCSController:writeInt32: error, status=%d function=%d, value=%d\n",
          ast, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
          "MCSController:writeInt32: function=%d, value=%d\n",
          function, value);
  return ast;
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
int MCSController::parse2Val(int *val1, int *val2)
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
int MCSController::parse3Val(int *val1, int *val2, int *val3)
{
  char cmd[10];
  if (4 != sscanf(outString_, ":%10[A-Z]%i,%i,%i", cmd, val1, val2, val3))
    return -1;
  return 'E' == cmd[0] ? *val2 : 0;
}

//formatted write string to member outString_
//reads returned string in outString_
//if dbg !=0 the strings are printed to the console
asynStatus MCSController::cmdWriteRead(bool dbg,const char *fmt, ...)
{
  asynStatus ast;
  va_list ap;
  va_start(ap, fmt);
  epicsVsnprintf(outString_, sizeof(outString_), fmt, ap);
  va_end(ap);
  DBG_PRINTF(dbg,"MCSController::cmdWriteRead: %s -> ",outString_);
  {
    size_t nwrite,nread;
    int eomReason;
    ast = pasynOctetSyncIO->writeRead(pasynUserController_, outString_, strlen(outString_), outString_,  sizeof(outString_), DEFLT_TIMEOUT, &nwrite, &nread, &eomReason);
  }
  //ast = this->writeReadController();
#ifdef DEBUG
  if(dbg) puts(outString_);
#endif
  return ast;
}

//-----------------------------------------------------------

MCSAxis::MCSAxis(class MCSController *pC, int axis, int dbgLvl)
  : asynMotorAxis(pC, axis), pC_(pC), dbgLvl_(dbgLvl)
{
  asynStatus ast;
  int val;

  asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCSAxis::MCSAxis -- creating axis %u\n", axis);

  ast = getVal(dbgLvl_&0x01,"GCLS", &vel_);
  if (ast)
    goto bail;
  if ((ast = getVal(dbgLvl_&0x01,"GS", &val)))
    goto bail;

  setIntegerParam(pC_->autoZero_, 0);
  setIntegerParam(pC_->hold_,     0);

  checkType();

bail:
  setIntegerParam(pC_->motorStatusProblem_, ast ? 1 : 0);
  setIntegerParam(pC_->motorStatusCommsError_, ast ? 1 : 0);

  callParamCallbacks();

  if (ast) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "MCSAxis::MCSAxis -- channel %u ASYN error %i", axis, ast);
  }
}

asynStatus MCSAxis::move(double position, int relative, double min_vel, double max_vel, double accel)
{
  asynStatus ast;
  int holdTime;
  const char *fmt;
  long int rpos;
  epicsInt32 angle;
  int rev;

  DBG_PRINTF(dbgLvl_&0x02,"MCSAxis::move@%u(pos:%.12g rel:%d min_vel:%.12g max_vel:%.12g)\n", axisNo_, position, relative, min_vel, max_vel);

  pC_->getIntegerParam(axisNo_, pC_->hold_, &holdTime);
  if ((ast = setSpeed(max_vel)))
    goto bail;

  rpos = lround(position);

  switch(devType_){
  case DT_NO_SENSOR:
    //TODO: support sensorless positioners
    printf("MCSAxis::move@%d():sensorless positioners not supported\n",axisNo_);
    break;
  case DT_LIN:
    fmt = relative ? ":MPR%u,%ld,%d" : ":MPA%u,%ld,%d";
    ast = pC_->cmdWriteRead(dbgLvl_&0x02,fmt, axisNo_, rpos, holdTime);
    break;
  case DT_ROT:
    fmt = relative ? ":MAR%u,%ld,%d,%d" : ":MAA%u,%ld,%d,%d";
    angle = (epicsInt32)rpos % UDEG_PER_REV;
    rev = (int)(rpos / UDEG_PER_REV);
    if (angle < 0){
      angle += UDEG_PER_REV;
      rev -= 1;
    }
    ast = pC_->cmdWriteRead(dbgLvl_&0x02,fmt, axisNo_, angle, rev, holdTime);
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

asynStatus MCSAxis::moveVelocity(double min_vel, double max_vel, double accel)
{
  asynStatus ast;
  epicsInt32 speed = (epicsInt32)rint(fabs(max_vel));
  epicsInt32 tgt_pos;
  signed char dir = 1;

  // No MCS command we an use directly. Just use a 'relative move' to very far target.
  DBG_PRINTF(dbgLvl_&0x02,"MCSAxis::moveVelocity@%d(min_vel:%.12g max_vel:%.12g\n", axisNo_, min_vel, max_vel);

  if (speed==0){
    //Here we are in a dilemma. If we set the 'MCS' speed to zero then it will move at unlimited speed which is so fast that
    //'JOG' makes no sense. Just 'STOP' the motion - hope that works...
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
    printf("MCSAxis::moveVelocity@%u():sensorless positioners not supported\n",axisNo_);
    break;
  case DT_LIN:
    tgt_pos = FAR_AWAY_LIN * dir;
    ast = pC_->cmdWriteRead(dbgLvl_&0x02,":MPR%u,%d,0", axisNo_, tgt_pos);
    break;
  case DT_ROT:
    tgt_pos = FAR_AWAY_ROT * dir;
    ast = pC_->cmdWriteRead(dbgLvl_&0x02,":MAR%u,0,%ld,0", axisNo_, tgt_pos);
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

asynStatus MCSAxis::home(double min_vel, double max_vel, double accel, int forwards)
{
  asynStatus ast;
  int holdTime;
  int autoZero;
  DBG_PRINTF(dbgLvl_&0x02,"MCSAxis::home@%u(forward:%u)\n",axisNo_,forwards);

  if ((ast = setSpeed(max_vel)))
    goto bail;

  pC_->getIntegerParam(axisNo_, pC_->autoZero_, &autoZero);
  pC_->getIntegerParam(axisNo_, pC_->hold_,     &holdTime);

  ast = pC_->cmdWriteRead(dbgLvl_&0x02,":FRM%u,%u,%d,%d", axisNo_, forwards ? 0 : 1, holdTime, autoZero);
bail:
  if (ast) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

asynStatus MCSAxis::stop(double acceleration)
{
  asynStatus ast;
  DBG_PRINTF(dbgLvl_&0x02,"MCSAxis::stop@%u()\n",axisNo_);
  ast = pC_->cmdWriteRead(dbgLvl_&0x02,":S%u", axisNo_);

  if (ast) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return ast;
}

asynStatus MCSAxis::poll(bool *moving_p)
{
  asynStatus ast;
  int val,pos=0;
  enum SmarActMCSStatus status;

  switch(devType_){
  case DT_NO_SENSOR:
    //TODO: support sensorless positioners
    break;
  case DT_LIN:
    if ((ast=getVal(dbgLvl_&0x100,"GP", &pos)))
      goto bail;
    break;
  case DT_ROT:
    if ((ast=getAngle(dbgLvl_&0x100,&pos)))
      goto bail;
    break;
  }

  setDoubleParam(pC_->motorEncoderPosition_, (double)pos);
  setDoubleParam(pC_->motorPosition_, (double)pos);

  if ((ast = getVal(dbgLvl_&0x100,"GS", &val)))
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

  // Check if the sensor 'knows' absolute position and update the MSTA 'HOMED' bit.
  if ((ast = getVal(dbgLvl_&0x100,"GPPK", &val)))
    goto bail;

  setIntegerParam(pC_->motorStatusHomed_, val ? 1 : 0);

  // Get currently set positioner type
  if ((ast = getVal(dbgLvl_&0x100,"GST", &val)))
    goto bail;
  setIntegerParam(pC_->ptyprb_, val);

bail:
  setIntegerParam(pC_->motorStatusProblem_, ast ? 1 : 0);
  setIntegerParam(pC_->motorStatusCommsError_, ast ? 1 : 0);

  callParamCallbacks();
  return ast;
}

asynStatus MCSAxis::setPosition(double position)
{
  asynStatus ast=asynSuccess;
  double rpos;

  rpos = rint(position);

  switch(devType_){
  case DT_NO_SENSOR:
    //TODO: support sensorless positioners
    printf("MCSAxis::setPosition()@%d:sensorless positioners not supported\n",axisNo_);
    break;
  case DT_LIN:
    ast = pC_->cmdWriteRead(dbgLvl_&0x02,":SP%u,%d", axisNo_, (epicsInt32)rpos);

    break;
  case DT_ROT:
    // For rotation stages the revolution will always be set to zero
    // Only set position if it is between zero an 360 degrees
    if (rpos >= 0.0 && rpos < (double)UDEG_PER_REV) {
      ast = pC_->cmdWriteRead(dbgLvl_&0x02,":SP%u,%d", axisNo_, (epicsInt32)rpos);
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

// Check if the positioner type set on the controller is linear or rotary or without encoder
void MCSAxis::checkType()
{
  int val;
  // Attempt to check linear sensor
  if (asynSuccess ==  getVal(dbgLvl_&0x04,"GP", &val)) {
    devType_=DT_LIN;
    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);
  }
  // Attempt to check rotary sensor
  else if (asynSuccess ==  getAngle(dbgLvl_&0x04,&val)) {
    devType_=DT_ROT;
    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);
  }
  else { // up to now we could not test with such sensors on MCS1
    devType_=DT_NO_SENSOR;
  }
  DBG_PRINTF(dbgLvl_&0x04,"MCSAxis::checkType@%u() -> %d\n", axisNo_,devType_);
  return;
}

// Read a parameter from the MCS (nothing to do with asyn's parameter library).
//
// parm_cmd: MCS command (w/o ':' char) to read parameter
// val:    where to store the value returned by the MCS
// RETURNS:  asynError if an error occurred, asynSuccess otherwise.
asynStatus MCSAxis::getVal(bool dbg,const char *parm_cmd, int *val)
{
  asynStatus ast;
  int ax;
  // asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "getVal() cmd=:%s%u", parm_cmd, this->axisNo_);
  ast = pC_->cmdWriteRead(dbg, ":%s%u", parm_cmd, this->axisNo_);
  if (ast)
    return ast;
  return pC_->parse2Val(&ax, val) ? asynError : asynSuccess;
}

// Read the position of rotation stage
//
// parm_cmd: MCS command (w/o ':' char) to read parameter
// val:    where to store the value returned by the MCS
// RETURNS:  asynError if an error occurred, asynSuccess otherwise.
///
asynStatus MCSAxis::getAngle(bool dbg,int *val)
{
  asynStatus ast;
  int ax,pos,rev,res;
  ast = pC_->cmdWriteRead(dbg, ":GA%u", this->axisNo_);
  if (ast)
    return ast;
  res=pC_->parse3Val(&ax, &pos, &rev);
  if (res)
    ast=asynError;
  else
    *val = pos+ rev * UDEG_PER_REV;
  return ast;
}

asynStatus MCSAxis::setSpeed(double velocity)
{
  DBG_PRINTF(dbgLvl_&0x02,"MCSAxis::setSpeed@%u(%.12g)\n",axisNo_,velocity);
  epicsInt32 ax,vel;
  asynStatus ast;
  vel = (epicsInt32)rint(fabs(velocity));
  if (vel == vel_)
    return asynSuccess; //nothing to do
  ast = pC_->cmdWriteRead(dbgLvl_&0x02,":SCLS%u,%ld", axisNo_, vel);
  if (ast!=asynSuccess)
    return ast;
  if (pC_->parse2Val(&ax, &vel))
    return asynError;
  vel_ = vel;
  return ast;
}

//------------------------------------------------------------------

// Code for iocsh registration
static const iocshArg MCSCreateControllerArgLst[] = {
  {"asyn port name",          iocshArgString},
  {"MCS port name",           iocshArgString},
  {"Number of axes",          iocshArgInt},
  {"Moving poll period (ms)", iocshArgInt},
  {"Idle poll period (ms)",   iocshArgInt},
  {"debug level",             iocshArgInt},
};
#define ARG MCSCreateControllerArgLst
static const iocshArg *const MCSCreateControllerArgs[] = {&ARG[0], &ARG[1], &ARG[2], &ARG[3], &ARG[4], &ARG[5]};
#undef ARG
static const iocshFuncDef MCSCreateControllerFuncDef = {"MCSCreateController", _countof(MCSCreateControllerArgs), MCSCreateControllerArgs};

static void MCSCreateControllerFunc(const iocshArgBuf *args)
{
  assert(new MCSController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival));
}

// Code for iocsh registration
static const iocshArg     MCSExtraArg0= {"<variable arguments>", iocshArgArgv};
static const iocshArg    *MCSExtraArgs[] = {&MCSExtraArg0};
static const iocshFuncDef MCSExtraDef = {"MCSExtra", 1, MCSExtraArgs};
void MCSExtra(int argc, char **argv);
static void MCSExtraFunc(const iocshArgBuf *args) {
  MCSExtra(args[0].aval.ac, args[0].aval.av);
}

void MCSExtra(int argc, char **argv)
{
  const char* usage[]={
         "MCSExtra report <ASYNPORT>",
         "MCSExtra setDbgLvlAxis <ASYNPORT> <AX> <HEX_DBGLVL>\n"\
         "Debug bits Axis (default value 0x00ff):\n"\
         "  0x001 : constructor\n"\
         "  0x002 : move, moveVelocity, stop, homing, setPosition\n"\
         "  0x004 :  checkType()\n"\
         "  0x008 : user asyn reason\n"\
         "  0x010 : \n"\
         "  0x020 : \n"\
         "  0x100 : polling\n",
      };
  uint32_t i;
  if (argc<2)
  {
    for(i=0;i<_countof(usage);i++)
      puts(usage[i]);
  }
  else if(!strcmp("report",argv[1]))
  {
    if(argc < 3)
      puts(usage[0]);
    else
    {
      MCSController* pC = (MCSController*) findAsynPortDriver(argv[2]);
      if(!pC)
      {
        puts(usage[0]);
        return;
      }
      for(i=0;i<32;i++)
      {
        MCSAxis* ax=(MCSAxis*)pC->getAxis(i);
        if(!ax)
          break;
        printf("axis:%d dbgVerb %x\n",i,ax->dbgLvl_);
      }
    }
  }
  else if(!strcmp("setDbgLvlAxis",argv[1]))
  {
    if(argc < 5)
      puts(usage[1]);
    else
    {
      MCSController* pC = (MCSController*) findAsynPortDriver(argv[2]);
      if(!pC)
      {
        puts(usage[1]);
        return;
      }
      MCSAxis* ax=(MCSAxis*)pC->getAxis(atoi(argv[3]));
      printf("MCSExtra %s:%s:%s = %p\n",argv[1],argv[2],argv[3],ax);
      if(ax)
      {
        uint32_t oldVal=ax->dbgLvl_,newVal=strtol(argv[4], NULL, 0);
        ax->dbgLvl_=newVal;
        printf(" dbgVerb 0x%x->0x%x\n",oldVal,newVal);
      }
    }
  }
  else
  {
    for(uint32_t i=0;i<_countof(usage);i++)
      puts(usage[i]);
  }
}

static void MCSMotorRegister(void)
{
  iocshRegister(&MCSCreateControllerFuncDef, MCSCreateControllerFunc); // MCSCreateController
  iocshRegister(&MCSExtraDef, MCSExtraFunc);
  //iocshRegister(&ca_def, ca_fn); // smarActMCSCreateAxis
}

extern "C" {
  epicsExportRegistrar(MCSMotorRegister);
}


/*
//smarActMCSCreateAxis not needed any more, as the controller creates by default all axes automatically
static const iocshArg ca_a0 = {"Controller Port name [string]", iocshArgString};
static const iocshArg ca_a1 = {"Axis number [int]", iocshArgInt};
static const iocshArg ca_a2 = {"dbgLvl [int]", iocshArgInt};

static const iocshArg *const ca_as[] = {&ca_a0, &ca_a1, &ca_a2};

// iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp)
// smarActMCSCreateAxis called to create each axis of the smarActMCS controller
static const iocshFuncDef ca_def = {"smarActMCSCreateAxis", 3, ca_as};

extern "C" void *
smarActMCSCreateAxis( const char *controllerPortName, int axisNumber, int dbgLvl)
{
  void *rval = 0;

  MCSController *pC;
  //MCSAxis *pAxis;
  asynMotorAxis *pAsynAxis;

  // the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
  // if constructing a controller (or axis) incurs an exception even if its
  // caught (IMHO asyn should behave as if the controller/axis never existed...)
  // rval = new MCSAxis(, axisNumber, channel);
  pC = (MCSController *)findAsynPortDriver(controllerPortName);
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
  //pAxis =
  new MCSAxis(pC, axisNumber, dbgLvl);
  //pAxis = NULL;
  pC->unlock();

  return rval;
}

static void ca_fn(const iocshArgBuf *args)
{
  smarActMCSCreateAxis( args[0].sval, args[1].ival, args[2].ival);
}
*/