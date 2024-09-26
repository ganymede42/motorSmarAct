/*---------------------------------------------------------------*
 * Filename     smarActMCS2MotorDriver.cpp                       *
 * Usage        Motor driver support for smarAct MCS2 Controller *
 * Author       Thierry Zamofing <thierry.zamofing@psi.ch>       *
 *              Adapted from David Vine Jan 19, 2019             *
 *              Till Straumann <strauman@slac.stanford.edu>      *
 *---------------------------------------------------------------*
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "smarActMCS2MotorDriver.h"

#ifdef DEBUG
  #define DBG_PRINTF(dbg,...) if(dbg) printf(__VA_ARGS__)
#else
  #define DBG_PRINTF(dbg,...)
#endif

#define _countof(arr) sizeof(arr) / sizeof(arr[0])

// Creates a new controller object.
// \param[in] ctrlPort             name of the port that will be created for this driver
// \param[in] asynPort             name of the drvAsynIPPPort that was created previously
// \param[in] numAxes              number of axes that this controller supports
// \param[in] movingPollPeriod     time in ms between polls when any axis is moving
// \param[in] idlePollPeriod       time in ms between polls when no axis is moving
MCS2Controller::MCS2Controller(const char *ctrlPort, const char *asynPort, int numAxes, double movingPollPeriod, double idlePollPeriod, int dbgLvl)
    : asynMotorController(ctrlPort, numAxes, NUM_MCS2_PARAMS,
                          0, 0,
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
  int axis;
  asynStatus ast;
  // Create controller-specific parameters
  createParam(MCS2PtypString,     asynParamInt32, &this->ptyp_);
  createParam(MCS2PtypRbString,   asynParamInt32, &this->ptyprb_);
  createParam(MCS2PstatString,    asynParamInt32, &this->pstatrb_);
  createParam(MCS2MclfString,     asynParamInt32, &this->mclf_);
  createParam(MCS2MclfRbString,   asynParamInt32, &this->mclfrb_);
  createParam(MCS2HoldString,     asynParamInt32, &this->hold_);
  createParam(MCS2HoldRbString,   asynParamInt32, &this->holdrb_);
  createParam(MCS2CalString,      asynParamInt32, &this->cal_);
  createParam(MCS2AutoZeroString, asynParamInt32, &this->autoZero_);

  // Connect to MCS2 controller
  ast = pasynOctetSyncIO->connect(asynPort, 0, &pasynUserController_, NULL);
  pasynOctetSyncIO->setInputEos(pasynUserController_, "\r\n", 2);
  pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r\n", 2);

  ast = cmdWriteRead(dbgLvl&0x01, ":DEV:SNUM?");
  if (ast) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"MCS2Controller::MCS2Controller: cannot connect to MCS2 controller\n");
  }
  ast = cmdWriteRead(dbgLvl&0x01, ":DEV:NOCH?");
  axis=atoi(this->inString_);
  if (numAxes>axis) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"MCS2Controller::MCS2Controller: allocate more axes than available!\n");
  }

  // Create the axis objects
  for(axis=0; axis<numAxes; axis++){
    new MCS2Axis(this, axis, dbgLvl);
  }

  //User specifies poll periods in milliseconds, starPoller expects seconds
  startPoller(movingPollPeriod/1000., idlePollPeriod/1000., 2);
}

// Called when asyn clients call pasynInt32->write().
// Extracts the function and axis number from pasynUser.
// Sets the value in the parameter library.
// For all other functions it calls asynMotorController::writeInt32.
// Calls any registered callbacks for this pasynUser->reason and address.
// \param[in] pasynUser asynUser structure that encodes the reason and address.
// \param[in] value     Value to write. */
asynStatus MCS2Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus ast = asynSuccess;
  MCS2Axis *pAxis = (MCS2Axis*)getAxis(pasynUser);

  // Check if axis exists
  if(!pAxis) return asynError;

  // Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
  // status at the end, but that's OK
  ast = setIntegerParam(pAxis->axisNo_, function, value);

  if (function == ptyp_) {
    // set positioner type
    if(!value)
      return asynSuccess; // value 0 is illegal but used to not change at IOC start
    ast=cmdWrite(pAxis->dbgLvl_&0x08, ":CHAN%d:PTYP %d", pAxis->axisNo_, value);
  }
  else if (function == mclf_) {
    // set piezo MaxClockFreq
    ast=cmdWrite(pAxis->dbgLvl_&0x08, ":CHAN%d:MCLF:CURR %d", pAxis->axisNo_, value);
  }
  else if (function == cal_) {
    // send calibration command
    ast=cmdWrite(pAxis->dbgLvl_&0x08, ":CAL%d", pAxis->axisNo_);
  }
  else if (function == hold_) {
    // set holding time
    ast=cmdWrite(pAxis->dbgLvl_&0x08, ":CHAN%d:HOLD %d", pAxis->axisNo_, value);
  }
  else if (function == autoZero_) {
    // autozero: just stores the value and returns success
    //ast = setIntegerParam(pAxis->axisNo_, function, value);
    //ast=asynSuccess
  }
  else {
    // Call base class method
    ast = asynMotorController::writeInt32(pasynUser, value);
  }

  // Do callbacks so higher layers see any changes
  callParamCallbacks(pAxis->axisNo_);
  if (ast)
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "MCS2Controller::writeInt32: error, status=%d function=%d, value=%d\n", ast, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "MCS2Controller::writeInt32: function=%d, value=%d\n", function, value);
  return ast;
}

// Reports on status of the driver
// \param[in] fp The file pointer on which report information will be written
// \param[in] level The level of report detail desired
//
// If details > 0 then information is printed about each axis.
// After printing controller-specific information calls asynMotorController::report()
void MCS2Controller::report(FILE *fp, int level)
{
  fprintf(fp, "MCS2 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

asynStatus MCS2Controller::clearErrors()
{

  asynStatus ast;
  int numErrMsg;

  // Read out error messages
  //ast=cmdWriteRead(1,":SYST:ERR:COUN?");
  sprintf(outString_,":SYST:ERR:COUN?");
  ast = writeReadController();
  if (ast) goto skip;
  numErrMsg = atoi(inString_);
  for (int i=0; i<numErrMsg; i++){
    sprintf(outString_,":SYST:ERR?");
    ast = writeReadController();
    if (ast) goto skip;
    printf("MCS2Controller::clearErrors(): %s\n",inString_);
    //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "YMCS2Controller::clearErrors: %d:%s\n", errCode, errMsg);
  }

skip:
  setIntegerParam(this->motorStatusProblem_, ast ? 1 : 0);
  callParamCallbacks();
  return ast;
}

//formatted write string to member outString_
//if dbg !=0 the strings are printed to the console
asynStatus MCS2Controller::cmdWrite(bool dbg,const char *fmt, ...)
{
  asynStatus ast;
  va_list ap;
  va_start(ap, fmt);
  epicsVsnprintf(outString_, sizeof(outString_), fmt, ap);
  va_end(ap);
  DBG_PRINTF(dbg,"MCS2Controller::cmdWrite: %s\n",outString_);
  ast = this->writeController();
  return ast;
}
//formatted write string to member outString_
//reads returned string in outString_
//if dbg !=0 the strings are printed to the console
asynStatus MCS2Controller::cmdWriteRead(bool dbg,const char *fmt, ...)
{
  asynStatus ast;
  va_list ap;
  va_start(ap, fmt);
  epicsVsnprintf(outString_, sizeof(outString_), fmt, ap);
  va_end(ap);
  DBG_PRINTF(dbg,"MCS2Controller::cmdWriteRead: %s -> ",outString_);
  ast = this->writeReadController();
#ifdef DEBUG
  if(dbg) puts(inString_);
#endif
  return ast;
}

// These are the MCS2Axis methods

/** Creates a new MCS2Axis object.
 * \param[in] pC Pointer to the ACRController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
 *
 * Initializes register numbers, etc.
 */
MCS2Axis::MCS2Axis(MCS2Controller *pC, int axisNo, int dbgLvl)
    : asynMotorAxis(pC, axisNo),
      pC_(pC),
      dbgLvl_(dbgLvl)
{
  asynPrint(pC->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Axis::MCS2Axis: Creating axis %u\n", axisNo);

  setIntegerParam(pC_->autoZero_, 0); // set initial value

  callParamCallbacks();
}

/** Reports on status of the driver
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * If details > 0 then information is printed about each axis.
 * After printing controller-specific information calls asynMotorController::report()
 */
void MCS2Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    int pcode;
    char pname[256];
    int channelState;
    int vel;
    int acc;
    int mclf;
    int followError;
    int error;
    int temp;

    asynStatus ast;

    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:PTYP?", axisNo_);
    pcode = atoi(pC_->inString_);
    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:PTYP:NAME?", axisNo_);
    strcpy(pC_->inString_, pname);
    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:STAT?", axisNo_);
    channelState = atoi(pC_->inString_);
    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:VEL?", axisNo_);
    vel = atoi(pC_->inString_);
    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:ACC?", axisNo_);
    acc = atoi(pC_->inString_);
    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:MCLF?", axisNo_);
    mclf = atoi(pC_->inString_);
    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:FERR?", axisNo_);
    followError = atoi(pC_->inString_);
    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:ERR?", axisNo_);
    error = atoi(pC_->inString_);
    ast=pC_->cmdWriteRead(dbgLvl_&0x08, ":CHAN%d:TEMP?", axisNo_);
    temp = atoi(pC_->inString_);

    if (ast)
      fprintf(fp, "  axis %d -> asyn status error %d\n", axisNo_, ast);
    fprintf(fp, "  axis %d\n"
                "    positioner type %d\n"
                "    positioner name %s\n"
                "    state %d\n"
                "    velocity %d\n"
                "    acceleration %d\n"
                "    max closed loop frequency %d\n"
                "    following error %d\n"
                "    error %d\n"
                "    temp %d\n",
            axisNo_, pcode, pname, channelState, vel,
            acc, mclf, followError, error, temp);
    pC_->clearErrors();
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus MCS2Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus ast = asynSuccess;

  // MCS2 MMOD (move mode) values:
  // 0: absolute (close loop)
  // 1 relative (close loop)
  // 4 relative steps (open loop)
  int hasEnc;
  pC_->getIntegerParam(axisNo_, pC_->motorStatusHasEncoder_, &hasEnc);
  if(hasEnc)
  {
    DBG_PRINTF(dbgLvl_&0x02, "MCS2Axis::move: closeloop: position: %g rel: %d\n",position,relative);
    ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:MMOD %d", axisNo_, relative > 0 ? 1 : 0);
    // Set acceleration
    ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:ACC %f", axisNo_, acceleration * PULSES_PER_STEP);
    // Set velocity
    ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:VEL %f", axisNo_, maxVelocity * PULSES_PER_STEP);
    // Do move
    ast=pC_->cmdWrite(dbgLvl_&0x02, ":MOVE%d %f", axisNo_, position * PULSES_PER_STEP);
  }
  else
  { // move relative in open loop
    double curPos;//,mRes;
    pC_->getDoubleParam(axisNo_, pC_->motorEncoderPosition_,&curPos);
    //pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_,&mRes);
    if(!relative)
      position=position-curPos;
    curPos+=position;
    DBG_PRINTF(dbgLvl_&0x02, "MCS2Axis::move: openloop: new_pos: %g rel_move: %g\n",curPos,position);
    //:STEP:FREQuency 1..20000, default:1000 Hz.
    //:STEP:AMPLitude 1..65535, default:65535 (100 V).
    setDoubleParam(pC_->motorEncoderPosition_, curPos);
    setDoubleParam(pC_->motorPosition_, curPos);

    ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:MMOD %d", axisNo_, 4);
    //ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:FREQ %f", axisNo_, mRes*maxVelocity);
    ast=pC_->cmdWrite(dbgLvl_&0x02, ":MOVE%d %f", axisNo_, position);
  }
  return ast;
}

asynStatus MCS2Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  //this is used for jogging: just do a big enough move
  DBG_PRINTF(dbgLvl_&0x02,"MCSAxis::moveVelocity@%d(min_vel:%.12g max_vel:%.12g\n", axisNo_, minVelocity, maxVelocity);
  float pos=(maxVelocity>0?1:-1)*1E8;
  maxVelocity=abs(maxVelocity);
  return MCS2Axis::move(pos, 1, minVelocity, maxVelocity, acceleration);
}

asynStatus MCS2Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  // MCS2ProgrammersGuide.pdf p.47 : 2.8.1 Reference Marks
  // do not use autozero for positioners with multiple reference mark!
  // this will not give consistent home position with multiple reference marks !
  asynStatus ast;
  int autoZero;
  pC_->getIntegerParam(axisNo_, pC_->autoZero_, &autoZero);
  unsigned short refOpt = 0;
  if (forwards==0)
    refOpt |= START_DIRECTION;
  if(autoZero)
    refOpt |= AUTO_ZERO;
  ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:REF:OPT %d", axisNo_, refOpt);
  // Set acceleration
  ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:ACC %f", axisNo_, acceleration * PULSES_PER_STEP);
  // Set velocity
  ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:VEL %f", axisNo_, maxVelocity * PULSES_PER_STEP);
  // Begin move
  ast=pC_->cmdWrite(dbgLvl_&0x02, ":REF%d", axisNo_);
  return ast;
}

asynStatus MCS2Axis::stop(double acceleration)
{
  asynStatus ast;
  DBG_PRINTF(dbgLvl_&0x02,"MCSAxis::stop@%u()\n",axisNo_);
  ast=pC_->cmdWrite(dbgLvl_&0x02, ":STOP%d", axisNo_);
  return ast;
}

asynStatus MCS2Axis::setPosition(double position)
{
  asynStatus ast;
  DBG_PRINTF(dbgLvl_&0x02,"MCS2Axis::setPosition@%u(%.12g)",axisNo_,position);
  ast=pC_->cmdWrite(dbgLvl_&0x02, ":CHAN%d:POS %f", axisNo_, position * PULSES_PER_STEP);
  return ast;
}

/** Polls the axis.
 * This function reads the controller position, encoder position, the limit status, the moving status,
 * the drive power-on status and positioner type. It does not current detect following error, etc.
 * but this could be added.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus MCS2Axis::poll(bool *moving)
{
  int done;
  int chanState;
  int closedLoop;
  int sensorPresent;
  int isCalibrated;
  int isReferenced;
  int endStopReached;
  int followLimitReached;
  int movementFailed;
  int refMark;
  int positionerType;
  double encoderPosition;
  double theoryPosition;
  int driveOn;
  int mclf;
  int hold;
  asynStatus ast;

  // Read the channel state
  ast=pC_->cmdWriteRead(dbgLvl_&0x100, ":CHAN%d:STAT?", axisNo_);
  if (ast) goto skip;
  chanState = atoi(pC_->inString_);
  setIntegerParam(pC_->pstatrb_, chanState);
  done = (chanState & ACTIVELY_MOVING) ? 0 : 1;
  closedLoop = (chanState & CLOSED_LOOP_ACTIVE) ? 1 : 0;
  sensorPresent = (chanState & SENSOR_PRESENT) ? 1 : 0;
  isCalibrated = (chanState & IS_CALIBRATED) ? 1 : 0;
  isReferenced = (chanState & IS_REFERENCED) ? 1 : 0;
  endStopReached = (chanState & END_STOP_REACHED) ? 1 : 0;
  followLimitReached = (chanState & FOLLOWING_LIMIT_REACHED) ? 1 : 0;
  movementFailed = (chanState & MOVEMENT_FAILED) ? 1 : 0;
  refMark = (chanState & REFERENCE_MARK) ? 1 : 0;
  //if (axisNo_==0)
  //{ //debug code
  //  int hasEnc; pC_->getIntegerParam(axisNo_, pC_->motorStatusHasEncoder_, &hasEnc);
  //  printf("chanState:0x%x hasEnc:%d closedLoop:%d\n",chanState,hasEnc,closedLoop);
  //}
  *moving = done ? false : true;
  //https://epics.anl.gov/bcda/synApps/motor/R7-2-1/motorRecord.html#Fields_status
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorStatusHasEncoder_, sensorPresent);
  setIntegerParam(pC_->motorStatusGainSupport_, closedLoop);
  setIntegerParam(pC_->motorStatusHomed_, isReferenced);
  setIntegerParam(pC_->motorStatusHighLimit_, endStopReached);
  setIntegerParam(pC_->motorStatusLowLimit_, endStopReached);
  setIntegerParam(pC_->motorStatusFollowingError_, followLimitReached);
  setIntegerParam(pC_->motorStatusProblem_, movementFailed);
  setIntegerParam(pC_->motorStatusAtHome_, refMark);

  if (sensorPresent)
  {
    // avoid pollong POS? if there is no sensor present. This would not return a value and block the polling for 1-2 sec
  // Read the current encoder position
    ast=pC_->cmdWriteRead(dbgLvl_&0x100, ":CHAN%d:POS?", axisNo_);
    if (ast) goto skip;
    encoderPosition = (double)strtod(pC_->inString_, NULL);
    encoderPosition /= PULSES_PER_STEP;
    setDoubleParam(pC_->motorEncoderPosition_, encoderPosition);

    // Read the current theoretical position
    ast=pC_->cmdWriteRead(dbgLvl_&0x100, ":CHAN%d:POS:TARG?", axisNo_);
    if (ast) goto skip;
    theoryPosition = (double)strtod(pC_->inString_, NULL);
    theoryPosition /= PULSES_PER_STEP;
    setDoubleParam(pC_->motorPosition_, theoryPosition);
  }

  // Read the drive power on status
  ast=pC_->cmdWriteRead(dbgLvl_&0x100, ":CHAN%d:AMPL?", axisNo_);
  if (ast) goto skip;
  driveOn = atoi(pC_->inString_) ? 1 : 0;
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);

  // Read the currently selected positioner type
  ast=pC_->cmdWriteRead(dbgLvl_&0x100, ":CHAN%d:PTYP?", axisNo_);
  if (ast) goto skip;
  positionerType = atoi(pC_->inString_);
  setIntegerParam(pC_->ptyprb_, positionerType);

  // Read CAL status MCLF HOLD when idle
  if (done)
  {
    setIntegerParam(pC_->cal_, isCalibrated);
    ast=pC_->cmdWriteRead(dbgLvl_&0x100, ":CHAN%d:MCLF?", axisNo_);
    if (ast) goto skip;
    mclf = atoi(pC_->inString_);
    setIntegerParam(pC_->mclfrb_, mclf);
    ast=pC_->cmdWriteRead(dbgLvl_&0x100, ":CHAN%d:HOLD?", axisNo_);
    if (ast) goto skip;
    hold = atoi(pC_->inString_);
    setIntegerParam(pC_->holdrb_, hold);
  }

skip:
  if(axisNo_==0) //when axis 0 is in idle mode, just gather any error that occured
    pC_->clearErrors(); // this just clears/prints error at a decent rate
  setIntegerParam(pC_->motorStatusProblem_, ast ? 1 : 0);
  callParamCallbacks();
  return ast ? asynError : asynSuccess;
}

//------------------------------------------------------------------

// Code for iocsh registration
static const iocshArg MCS2CreateControllerArgLst[] = {
  {"Port name",               iocshArgString},
  {"MCS2 port name",          iocshArgString},
  {"Number of axes",          iocshArgInt},
  {"Moving poll period (ms)", iocshArgInt},
  {"Idle poll period (ms)",   iocshArgInt},
  {"debug level",             iocshArgInt},
};
#define ARG MCS2CreateControllerArgLst
static const iocshArg *const MCS2CreateControllerArgs[] = {&ARG[0], &ARG[1], &ARG[2], &ARG[3], &ARG[4], &ARG[5]};
static const iocshFuncDef MCS2CreateControllerDef = {"MCS2CreateController", _countof(MCS2CreateControllerArgs), MCS2CreateControllerArgs};
#undef ARG
static void MCS2CreateContollerFunc(const iocshArgBuf *args)
{
  assert(new MCS2Controller(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival));
}

// Code for iocsh registration
static const iocshArg     MCS2ExtraArg0= {"<variable arguments> (type 'MCS2Extra' for usage)", iocshArgArgv};
static const iocshArg    *MCS2ExtraArgs[] = {&MCS2ExtraArg0};
static const iocshFuncDef MCS2ExtraDef = {"MCS2Extra", 1, MCS2ExtraArgs};
void MCS2Extra(int argc, char **argv);
static void MCS2ExtraFunc(const iocshArgBuf *args) {
  MCS2Extra(args[0].aval.ac, args[0].aval.av);
}

void MCS2Extra(int argc, char **argv)
{
  const char* usage[]={
         "MCS2Extra report <ASYNPORT>",
         "MCS2Extra setDbgLvlAxis <ASYNPORT> <AX> <HEX_DBGLVL>\n"\
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
      MCS2Controller* pC = (MCS2Controller*) findAsynPortDriver(argv[2]);
      if(!pC)
      {
        puts(usage[0]);
        return;
      }
      for(i=0;i<32;i++)
      {
        MCS2Axis* ax=(MCS2Axis*)pC->getAxis(i);
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
      MCS2Controller* pC = (MCS2Controller*) findAsynPortDriver(argv[2]);
      if(!pC)
      {
        puts(usage[1]);
        return;
      }
      MCS2Axis* ax=(MCS2Axis*)pC->getAxis(atoi(argv[3]));
      printf("MCS2Extra %s:%s:%s = %p\n",argv[1],argv[2],argv[3],ax);
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


static void MCS2MotorRegister(void)
{
  iocshRegister(&MCS2CreateControllerDef, MCS2CreateContollerFunc);
  iocshRegister(&MCS2ExtraDef, MCS2ExtraFunc);
}

extern "C" {
  epicsExportRegistrar(MCS2MotorRegister);
}
