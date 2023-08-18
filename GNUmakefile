include /ioc/tools/driver.makefile

MODULE = motorSmarAct

EXCLUDE_VERSIONS = 3
BUILDCLASSES = Linux WIN32

SOURCES += smarActApp/src/smarActMCS2MotorDriver.cpp
DBDS += smarActApp/src/devSmarActMCS2Motor.dbd

SOURCES += smarActApp/src/smarActMCSMotorDriver.cpp
DBDS += smarActApp/src/devSmarActMCSMotor.dbd

SOURCES += smarActApp/src/smarActSCUMotorDriver.cpp
DBDS += smarActApp/src/devSmarActSCUMotor.dbd


TEMPLATES+=$(wildcard ./smarActApp/Db/*.db)
TEMPLATES+=$(wildcard ./smarActApp/Db/*.template)
TEMPLATES+=$(wildcard ./dsmarActApp/Dbb/*.subs)
