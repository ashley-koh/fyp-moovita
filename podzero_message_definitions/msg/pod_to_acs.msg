## POD to ACS v1.4   #12/4/18 21:48

Header header

uint8 FrontLeftWheelCnt
uint8 FrontRightWheelCnt
uint8 RearLeftWheelCnt
uint8 RearRightWheelCnt

uint8 PodState
int16 Velocity
int8 SteeringPercent
uint8 StateOfCharge
int16 McuBatteryCurrent
int16 TotalBatteryCurrent
int16 VSupply12
int16 VBat48
int16 McuTemp
int16 MotorTemp
uint8 VmsFaultCode
uint8 McuFaultCode


uint8 ParkBrakeState
bool ParkBrakePressureAchieved
uint8 MotorBrakeState
bool HeadlampsAreOn
bool InteriorLightsAreOn
bool LeftDiIsActive
bool RightDiIsActive

bool EStopTriggered
bool SafetyLidarTriggered
bool RfEstopTriggered

bool LeftDoorButtonPressed
bool LeftDoorClosed
bool RightDoorButtonPressed
bool RightDoorClosed

uint8 FrontLeftOuterUltrasonic
uint8 FrontLeftCentreUltrasonic
uint8 FrontRightCentreUltrasonic
uint8 FrontRightOuterUltrasonic

uint8 RearLeftOuterUltrasonic
uint8 RearLeftCentreUltrasonic
uint8 RearRightCentreUltrasonic
uint8 RearRightOuterUltrasonic

