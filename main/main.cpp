// Do not remove the include below
#include "main.h"

int throttlePin = 0;
int motorPins[] = {9,10,11,12}; // motor 1 is + y, motor 2 is -x, motor 3 is - y, motor 4 is + x
int csnPin = 53;
int cePin = 8;

int gyrorange = 1; // + or - .5
double motormultiplier[4];

const int sampTime = 16; //milliseconds
int loopits;
unsigned long starttime;


ControlState state;
Servo esc[4];

MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
Quaternion lastq;
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int32_t gyro[3];       // [x, y, z]

int radioNumber = 1;
RF24 radio(cePin,csnPin);


byte addresses[][6] = {"1Node","2Node"};

double motorspeed[4];

uint32_t m_z = 1;
uint32_t m_w = 1;

//10/6.5
//11.5

//calculated pid, p = .9, i = 0.00936, d = 21.635
// k_u is where it start oscillating
// P_u is period of oscillation = 0.5s / 2.6ms = 192 sampling intervals
// p = 0.6 * k_u, i = 2k_p / (P_u), d = (K_p * P_u )/ 8

PIDs posPID;
PIDs veloPID;

double currentgain = .8;

unsigned long lasttime;
unsigned long timesq;
unsigned long ltime;

const int hoversetpoint = 105;

PID xPosControl(&posPID.x.input, &(posPID.x.output), &posPID.x.setpoint, posPID.x.p,posPID.x.i,posPID.x.d, DIRECT);
PID yPosControl(&posPID.y.input, &posPID.y.output, &posPID.y.setpoint, posPID.y.p,posPID.y.i,posPID.y.d, DIRECT);
PID zPosControl(&posPID.z.input, &posPID.z.output, &posPID.z.setpoint, posPID.z.p,posPID.z.i,posPID.z.d, DIRECT);

PID xVeloControl(&veloPID.x.input, &(veloPID.x.output), &veloPID.x.setpoint, veloPID.x.p,veloPID.x.i,veloPID.x.d, DIRECT);
PID yVeloControl(&veloPID.y.input, &veloPID.y.output, &veloPID.y.setpoint, veloPID.y.p,veloPID.y.i,veloPID.y.d, DIRECT);
PID zVeloControl(&veloPID.z.input, &veloPID.z.output, &veloPID.z.setpoint, veloPID.z.p,veloPID.z.i,veloPID.z.d, DIRECT);

PID vertControl(&posPID.vert.input, &posPID.vert.output, &posPID.vert.setpoint, posPID.vert.p,posPID.vert.i,posPID.vert.d, DIRECT);

void PIDCompute(){
	//possibly need to scale these back
	veloPID.x.input = 100 * (q.x - lastq.x)/ double(sampTime);//double(gyro[0]) / 50000;
	veloPID.y.input = 100 * (q.y - lastq.y) / double(sampTime);//double(gyro[1]) / 50000;
	veloPID.z.input = 100 * (q.z - lastq.z) / double(sampTime);//double(gyro[2]) / 50000;
	xPosControl.Compute();
	yPosControl.Compute();
	zPosControl.Compute();
	if(xVeloControl.Compute()) lastq.x = q.x;
	if(yVeloControl.Compute()) lastq.y = q.y;
	if(zVeloControl.Compute()) lastq.z = q.z;
	//vertControl.Compute();
}

void setPID(){
	xPosControl.SetTunings(posPID.x.p, posPID.x.i, posPID.x.d);
	yPosControl.SetTunings(posPID.y.p, posPID.y.i, posPID.y.d);
	zPosControl.SetTunings(posPID.z.p, posPID.z.i, posPID.z.d);

	xVeloControl.SetTunings(veloPID.x.p, veloPID.x.i, veloPID.x.d);
	yVeloControl.SetTunings(veloPID.y.p, veloPID.y.i, veloPID.y.d);
	zVeloControl.SetTunings(veloPID.z.p, veloPID.z.i, veloPID.z.d);

	vertControl.SetTunings(posPID.vert.p, posPID.vert.i, posPID.vert.d);
}
char pidcharchar(ControlState state){
	switch(pidchar(state)){
	case p:	 return 'p';
	case i:	 return 'i';
	case d:  return 'd';
	default: return 'B';
	}
}

char quancharchar(ControlState state){
	switch(quanchar(state)){
	case position: return 'P';
	case velocity: return 'V';
	default: return 'E';
	}
}

char axischarchar(ControlState state){
	switch(axischar(state)){
	case x:    return 'x';
	case y:    return 'y';
	case z:	   return 'z';
	case vert: return 'v';
	default:   return 'B';
	}
}

void printgains(ControlState state){
	Serial.print("[");
	Serial.print(quancharchar(state));
	Serial.print(axischarchar(state));
	Serial.print(pidcharchar(state));
	Serial.print("] pos x ");
	Serial.print(posPID.x.p); Serial.print(" ");
	Serial.print(posPID.x.i); Serial.print(" ");
	Serial.print(posPID.x.d);
	Serial.print(" pos y ");
	Serial.print(posPID.y.p); Serial.print(" ");
	Serial.print(posPID.y.i); Serial.print(" ");
	Serial.print(posPID.y.d);
	Serial.print(" pos z ");
	Serial.print(posPID.z.p); Serial.print(" ");
	Serial.print(posPID.z.i); Serial.print(" ");
	Serial.print(posPID.z.d);
	Serial.print(" pos v ");
	Serial.print(posPID.vert.p); Serial.print(" ");
	Serial.print(posPID.vert.i); Serial.print(" ");
	Serial.print(posPID.vert.d);
	Serial.print(" velo x ");
	Serial.print(veloPID.x.p); Serial.print(" ");
	Serial.print(veloPID.x.i); Serial.print(" ");
	Serial.print(veloPID.x.d);
	Serial.print(" velo y ");
	Serial.print(veloPID.y.p); Serial.print(" ");
	Serial.print(veloPID.y.i); Serial.print(" ");
	Serial.print(veloPID.y.d);
	Serial.print(" velo z ");
	Serial.print(veloPID.z.p); Serial.print(" ");
	Serial.print(veloPID.z.i); Serial.print(" ");
	Serial.print(veloPID.z.d);
	Serial.println();
}

uint32_t GetUint()
{
    m_z = 36969 * (m_z & 65535) + (m_z >> 16);
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    return (m_z << 16) + m_w;
}
double GetUniform()
{
    // 0 <= u < 2^32
    uint32_t u = GetUint();
    // The magic number below is 1/(2^32 + 2).
    // The result is strictly between 0 and 1.
    return (u + 1.0) * 2.328306435454494e-10;
}
int randRound(double x){
	double frac_part, int_part;
	frac_part = modf(x, &int_part);
	if(GetUniform() < frac_part){
		int_part++;
	}
	return int_part;
}
// looking from above, clockwise it goes red, white , white, red, and motor 1, motor 2, motor 3, and motor 4.
// x axis runs from red to white with a positive value being a rotation towards motor 3 and 4
// y axis runs from motor 1 and 2 side to motor 3 and 4 side, with a positive value being a rotation towards the motor 2 and 3 side.

void torqueToMotorVelo(double vaccel, double tauX, double tauY, double tauZ, double motors[]){
	/*
	 * Input is torque about all three axes and vertical thrust, and outputs to motors array
	 * constants based on matrix ainv in matrices.txt, which is inverse of matrix transformation
	 * from motor velocity squared to torque
	*/
	motors[0] = -1 * 0.8694517 * tauX - 1 * tauY + 0.15910966 * tauZ + 0.172 * (vaccel / 100);
	motors[1] = -1 * 0.8694517 * tauX + 1 * tauY - 0.17389034 * tauZ + 0.161 * (vaccel / 100);
	motors[2] = 0.8694517 * tauX + 1 * tauY + 0.17389034 * tauZ + 0.161 * (vaccel / 100);
	motors[3] = 0.8694517 * tauX - 1 * tauY - 0.15910966 * tauZ + 0.172 * (vaccel / 100);
	for (int i=0; i<4; i++){
		motors[i] = ((motors[i] > 1) ? 1 : ((motors[i] < -1) ? -1 : motors[i]));
		motors[i] = randRound(hoversetpoint + motors[i] * (hoversetpoint - 30)); //scale
	}

}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void printDebugs(){
		//			for(int i=0; i<4;i++){
		//				state.pwmset[i] = motorspeed[i];
		//			}
	//				state.setretx = xsetpoint;
	//				state.setrety = ysetpoint;
	//				state.vertset = vertaccelset;
	state.pwmoutx = veloPID.x.output;
	state.pwmouty = veloPID.y.output;
	state.pwmoutvert = posPID.vert.output;
	Serial.println("still writing");
	radio.stopListening();
	radio.write(&state, sizeof(ControlState));
	radio.startListening() ;
}
void updateMotors(){
	gyroToMotor(posPID.vert.setpoint, veloPID.x.output, veloPID.y.output, veloPID.z.output, motorspeed);
	for (int i=0; i<4; i++){
		esc[i].write(motorspeed[i]); //* motormultiplier[i]);
	}
	PIDCompute();
}
void calcIterTimes(){
	if(loopits == 0){
		starttime = millis();
		ltime = starttime;
		timesq = 0;
	}
	loopits++;
	unsigned long now = millis();
	timesq += sq(now - ltime);
	ltime = now;
}

void printIterTimes(){
	if(loopits!= 0){
		unsigned long time = millis() - starttime;
		float meanperiod = float(time) / float(loopits);
		Serial.println();
		Serial.print("Iterations: ");
		Serial.print(loopits);
		Serial.print("\t");
		Serial.print("Time in ms: ");
		Serial.print(time);
		Serial.print("\t");
		Serial.print("frequency (Hz): ");
		Serial.print(float(loopits) * 1000.0 / float(time));
		Serial.print("\t period (ms): ");
		Serial.print(meanperiod);
		Serial.print("std deviation: ");
		Serial.print(sqrt(float(timesq) / float(loopits) - sq(meanperiod)));
	}
	loopits = 0;
}

void zeroMotors(){
	for (int i=0; i<4; i++){
		esc[i].write(30);
	}
}

void resetIs(){
	xPosControl.ResetITerm();
	yPosControl.ResetITerm();
	zPosControl.ResetITerm();
	xVeloControl.ResetITerm();
	yVeloControl.ResetITerm();
	zVeloControl.ResetITerm();
	vertControl.ResetITerm();
}

void updateGains(){
	if(state.currentgain != currentgain ){
//			Serial.println("state.currentgain");
//			Serial.println(state.currentgain);
		currentgain = state.currentgain;
		switch(quanchar(state)){
		case position:
			switch(axischar(state)){
			case x:
				switch(pidchar(state)){
				case p:
					posPID.x.p = currentgain;
					break;
				case i:
					posPID.x.i = currentgain;
					break;
				case d:
					posPID.x.d = currentgain;
					break;
				}
				break;
			case y:
				switch(pidchar(state)){
				case p:
					posPID.y.p = currentgain;
					break;
				case i:
					posPID.y.i = currentgain;
					break;
				case d:
					posPID.y.d = currentgain;
					break;
				}
				break;
			case z:
				switch(pidchar(state)){
				case p:
					posPID.z.p = currentgain;
					break;
				case i:
					posPID.z.i = currentgain;
					break;
				case d:
					posPID.z.d = currentgain;
					break;
				}
				break;
			case vert:
				switch(pidchar(state)){
				case p:
					posPID.vert.p = currentgain;
					break;
				case i:
					posPID.vert.i = currentgain;
					break;
				case d:
					posPID.vert.d = currentgain;
					break;
				}
				break;
			}
			break;
		case velocity:
			switch(axischar(state)){
			case x:
				switch(pidchar(state)){
				case p:
					veloPID.x.p = currentgain;
					break;
				case i:
					veloPID.x.i = currentgain;
					break;
				case d:
					veloPID.x.d = currentgain;
					break;
				}
				break;
			case y:
				switch(pidchar(state)){
				case p:
					veloPID.y.p = currentgain;
					break;
				case i:
					veloPID.y.i = currentgain;
					break;
				case d:
					veloPID.y.d = currentgain;
					break;
				}
				break;
			case z:
				switch(pidchar(state)){
				case p:
					veloPID.z.p = currentgain;
					break;
				case i:
					veloPID.z.i = currentgain;
					break;
				case d:
					veloPID.z.d = currentgain;
					break;
				}
				break;
			}
		}
		setPID();
	}
}

void updateControlVars(){
	posPID.x.input = q.x;
	posPID.y.input = q.y;
	posPID.z.input = q.z;

	posPID.vert.input = aaReal.z / 100;

	posPID.vert.setpoint = state.vertaccelset;
	posPID.x.setpoint = state.xset;
	posPID.y.setpoint = state.yset;
	posPID.z.setpoint = state.zset;

	veloPID.x.setpoint = posPID.x.output;
	veloPID.y.setpoint = posPID.y.output;
	veloPID.z.setpoint = posPID.z.output;
}

void printDebugging(){
	//		if(millis() > lasttime + 50){
	//		//		Serial.print("x pid output: ");
	////		Serial.print(xgyroout);
	////		Serial.print("  x pid input: ");
	//		Serial.println();
	////			Serial.print(xVeloControl.GetLastInput());
	////			Serial.print("\t");
	////
	////			Serial.print(yVeloControl.GetLastInput());
	////			Serial.print("\t");
	////			Serial.print(zVeloControl.GetLastInput());
	////			Serial.print("\t");
	////
	//			Serial.print(posPID.x.input);
	//			Serial.print("\t");
	//
	//			Serial.print(posPID.y.input);
	//			Serial.print("\t");
	//			Serial.print(posPID.z.input);
	//			Serial.print("\t");
	////			Serial.print(veloPID.x.setpoint);
	////			Serial.print("\t");
	////
	////			Serial.print(veloPID.y.setpoint);
	////			Serial.print("\t");
	////			Serial.print(veloPID.z.setpoint);
	////			Serial.print("\t");
	////
	//			//printgains(state);
	////			Serial.print(vertaccelset);
	//			Serial.println();
	//			lasttime = millis();
	////		Serial.println("state.currentgain");
	////		Serial.println(state.currentgain);
	////		Serial.println(currentgain);
	//		}
}

void handleMPU(){
	//Serial.print("mpuInterrupt = "); Serial.print(mpuInterrupt);
	//Serial.print(". fifoCount = "); Serial.println(fifoCount);
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize){
			Serial.println("stuck in loop");
			fifoCount = mpu.getFIFOCount();
			Serial.println(fifoCount);
		}
		//Serial.println("mpuing");
		//Serial.println(fifoCount);
		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		mpu.dmpGetQuaternion(&q, fifoBuffer);
//		mpu.dmpGetAccel(&aa, fifoBuffer);
//		mpu.dmpGetGravity(&gravity, &q);
//		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		//mpu.dmpGetGyro(gyro, fifoBuffer);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
		//Serial.print("After mpu, expect fifoCount = "); Serial.print(fifoCount);
		//Serial.print(", but mpu.getFIFOCount() = "); Serial.println(mpu.getFIFOCount());
		}
	if(fifoCount > packetSize){
		mpuInterrupt = false;
		mpu.resetFIFO();
		fifoCount = 0;
	}
}

void setup() {
	posPID.x.p = 1; //.4//original 1.2 blue
	posPID.x.i = 0;//0.05;//0.35; //1//.5 blue
	posPID.x.d = 0;//.2; //.1//.5 blue
	posPID.y.p = 0;//.4;//1.2 blue
	posPID.y.i = 0; //.8 blue
	posPID.y.d = 0;//0.2; // .3 blue
	posPID.z.p = 0;
	posPID.z.i = 0;
	posPID.z.d = 0;
	posPID.vert.p = 0;
	posPID.vert.i = 0;
	posPID.vert.d = 0;
	veloPID.x.p = 1;
	veloPID.x.i = 0;
	veloPID.x.d = 0;
	veloPID.y.p = 0;
	veloPID.z.p = 0;
	lasttime = millis();
  Serial.begin(57600);
  Serial.println(F("Setup"));

  radio.begin();
  radio.setPALevel(RF24_PA_HIGH); // might want to change this later


  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }

  radio.startListening();

  for (int i=0; i < 4; i++){
	  esc[i].attach(motorPins[i], 700, 2000);
  }
  // Gyro setup
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	  Wire.begin();
	  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	  Fastwire::setup(400, true);
  #endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
	  // turn on the DMP, now that it's ready
	  Serial.println(F("Enabling DMP..."));
	  mpu.setDMPEnabled(true);

	  // enable Arduino interrupt detection
	  Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
	  attachInterrupt(0, dmpDataReady, RISING);
	  mpuIntStatus = mpu.getIntStatus();

	  // set our DMP Ready flag so the main loop() function knows it's okay to use it
	  Serial.println(F("DMP ready! Waiting for first interrupt..."));
	  dmpReady = true;

	  // get expected DMP packet size for later comparison
	  packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
	  // ERROR!
	  // 1 = initial memory load failed
	  // 2 = DMP configuration updates failed
	  // (if it's going to break, usually the code will be 1)
	  Serial.print(F("DMP Initialization failed (code "));
	  Serial.print(devStatus);
	  Serial.println(F(")"));
  }

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(105); //-407 1715
  mpu.setYGyroOffset(-60); //266
  mpu.setZGyroOffset(-13); //53
  mpu.setZAccelOffset(1000); // 1688 factory default for my test chip

  posPID.x.setpoint = 0;
  posPID.y.setpoint = 0;
  posPID.z.setpoint = 0;
  xPosControl.SetMode(AUTOMATIC);
  yPosControl.SetMode(MANUAL);//AUTOMATIC);
  zPosControl.SetMode(MANUAL);//AUTOMATIC);
  xVeloControl.SetMode(AUTOMATIC);
  yVeloControl.SetMode(AUTOMATIC);
  zVeloControl.SetMode(AUTOMATIC);
  vertControl.SetMode(AUTOMATIC);
  xPosControl.SetOutputLimits(-1,1);
  yPosControl.SetOutputLimits(-1,1);
  zPosControl.SetOutputLimits(-1,1);
  xVeloControl.SetOutputLimits(-1,1);
  yVeloControl.SetOutputLimits(-1,1);
  zVeloControl.SetOutputLimits(-1,1);
  vertControl.SetOutputLimits(-100,100);
  xPosControl.SetSampleTime(sampTime);
  yPosControl.SetSampleTime(sampTime);
  zPosControl.SetSampleTime(sampTime);
  xVeloControl.SetSampleTime(sampTime);
  yVeloControl.SetSampleTime(sampTime);
  zVeloControl.SetSampleTime(sampTime);
  vertControl.SetSampleTime(sampTime);
  for (int i=0; i<4; i++){
  				esc[i].write(30);
  }
  //delay(5000);

}
void loop() {
	while(!mpuInterrupt && fifoCount < packetSize){
		if( radio.available()){
			radio.read( &state, sizeof(ControlState) );
			if(printDebugbool(state)){
				printDebugs();
			}
		}
		if(state.turnOn){
			calcIterTimes();
			updateMotors();
		}else{
			zeroMotors();
			printIterTimes();
			resetIs();
			updateGains();
		}
		updateControlVars();
		printDebugging();
	}
	handleMPU();
}
