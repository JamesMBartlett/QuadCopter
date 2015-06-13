// Do not remove the include below
#include "main.h"

int throttlePin = 0;
int motorPins[] = {9,10,11,12}; // motor 1 is + y, motor 2 is -x, motor 3 is - y, motor 4 is + x
int csnPin = 53;
int cePin = 8;

int gyrorange = 1; // + or - .5
double motormultiplier[4];

const int sampTime = 2; //milliseconds
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gyro;       // [x, y, z]

int radioNumber = 1;
RF24 radio(cePin,csnPin);


byte addresses[][6] = {"1Node","2Node"};
double xgyroin;
double ygyroin;
double zgyroin;
double vertaccelin;
double xgyroout = 0;
double ygyroout = 0;
double zgyroout;
double vertaccelout = 0;
double xsetpoint;
double ysetpoint;
double zsetpoint;
double vertaccelset;
double motorspeed[4];

uint32_t m_z = 1;
uint32_t m_w = 1;

//10/6.5
//11.5

//calculated pid, p = .9, i = 0.00936, d = 21.635
// k_u is where it start oscillating
// P_u is period of oscillation = 0.5s / 2.6ms = 192 sampling intervals
// p = 0.6 * k_u, i = 2k_p / (P_u), d = (K_p * P_u )/ 8

double currentgain = .8;
double px = 1; //.4//original 1.2 blue
double ix = 0;//0.05;//0.35; //1//.5 blue
double dx = .2; //.1//.5 blue
//double dx = 0.5;
double py = 1.3;//.4;//1.2 blue
double iy = 0; //.8 blue
double dy = 0.2; // .3 blue
double pz = 1;
double iz = 0;
double dz = 0;
double pvert = 1;
double ivert = 0;
double dvert = 0;
bool gainchangeflag = false;

unsigned long lasttime;

const int hoversetpoint = 105;


PID xControl(&xgyroin, &xgyroout, &xsetpoint, px,ix,dx, DIRECT);
PID yControl(&ygyroin, &ygyroout, &ysetpoint, py,iy,dy, DIRECT);
PID zControl(&zgyroin, &zgyroout, &zsetpoint, pz, iz,dz, DIRECT);
PID vertControl(&vertaccelin, &vertaccelout, &vertaccelset, pvert, ivert,dvert, DIRECT);

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

void gyroToMotor(double vaccel, double x, double y, double z, double motors[]){
	motors[0] = vaccel / 100 + x + y + z;
	motors[1] = vaccel / 100 + x - y - z;
	motors[2] = vaccel / 100 - x - y + z;
	motors[3] = vaccel / 100 - x + y - z;
	for (int i=0; i<4; i++){
		motors[i] = randRound(hoversetpoint + motors[i] * (hoversetpoint - 30) / 4); //scale
	}

}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
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

  xsetpoint = 0;
  ysetpoint = 0;
  zsetpoint = 0;
  xControl.SetMode(AUTOMATIC);
  yControl.SetMode(AUTOMATIC);
  zControl.SetMode(AUTOMATIC);
  vertControl.SetMode(AUTOMATIC);
  xControl.SetOutputLimits(-1,1);
  yControl.SetOutputLimits(-1,1);
  zControl.SetOutputLimits(-1,1);
  vertControl.SetOutputLimits(-100,100);
  xControl.SetSampleTime(sampTime);
  yControl.SetSampleTime(sampTime);
  zControl.SetSampleTime(sampTime);
  vertControl.SetSampleTime(sampTime);
  for (int i=0; i<4; i++){
  				esc[i].write(30);
  }
  //delay(5000);

}
void loop() {
	while(!mpuInterrupt && fifoCount < packetSize){
		if( radio.available()){
																		// Variable for the received timestamp
		  //while (radio.available()) {                                   // While there is data ready
			//Serial.println("reading radio");
			radio.read( &state, sizeof(ControlState) );             // Get the payload
		  //}

//		  radio.stopListening();                                        // First, stop listening so we can talk
//		  radio.write( &state, sizeof(ControlState) );              // Send the final one back.
//		  radio.startListening();                                       // Now, resume listening so we catch the next packets.
//		  Serial.println(F("Sent response "));
//		  Serial.print(state.axes[0]);
//		  Serial.print(", ");
//		  Serial.print(state.axes[1]);
			//Serial.println(state.printDebug);
			if(printDebugbool(state)){
	//			for(int i=0; i<4;i++){
	//				state.pwmset[i] = motorspeed[i];
	//			}
//				state.setretx = xsetpoint;
//				state.setrety = ysetpoint;
//				state.vertset = vertaccelset;
				state.pwmoutx = xgyroout;
				state.pwmouty = ygyroout;
				state.pwmoutvert = vertaccelout;
				Serial.println("still writing");
				radio.stopListening();
				radio.write(&state, sizeof(ControlState));
				radio.startListening() ;
			}
		}



		//int throttle = map(state.vertaccelset, -100, 100, 30, 179);

		vertaccelset = state.vertaccelset;
		xsetpoint = state.xset;
		ysetpoint = state.yset;
		zsetpoint = state.zset;
		gyroToMotor(vertaccelout, xgyroout, ygyroout, zgyroout, motorspeed); //vertaccelout


		if(state.turnOn){ //millis() < 12000){
			if(loopits == 0){
				starttime = millis();
			}
			loopits++;
//			Serial.println();
//			for(int i= 0; i<4; i++){
//				Serial.print(motorspeed[i]);
//				Serial.print("\t");
//			}
//			Serial.println();
			for (int i=0; i<4; i++){
				esc[i].write(motorspeed[i]); //* motormultiplier[i]);
			}
			xControl.Compute();
			yControl.Compute();
			zControl.Compute();
			vertControl.Compute();
		}else{
			if(loopits!= 0){
				unsigned long time = millis() - starttime;
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
				Serial.print(float(time) / float(loopits));
			}
			loopits = 0;
			xControl.ResetITerm();
			yControl.ResetITerm();
			zControl.ResetITerm();
			vertControl.ResetITerm();
			for (int i=0; i<4; i++){
				esc[i].write(30);
			}
		}
		xgyroin = q.x;
		ygyroin = q.y;
		zgyroin = q.z;
		vertaccelin = aaReal.z / 100;


		if(millis() > lasttime + 50){
		//		Serial.print("x pid output: ");
//		Serial.print(xgyroout);
//		Serial.print("  x pid input: ");
//		Serial.println();
//			Serial.print(xsetpoint);
//			Serial.print("\t");
//
//			Serial.print(ysetpoint);
//			Serial.print("\t");
//			Serial.print(zsetpoint);
//			Serial.print("\t");
//
//			Serial.print(vertaccelset);
//			Serial.println();
			lasttime = millis();
//		Serial.println("state.currentgain");
//		Serial.println(state.currentgain);
//		Serial.println(currentgain);
		}
		if(state.currentgain != currentgain ){
//			Serial.println("state.currentgain");
//			Serial.println(state.currentgain);
			currentgain = state.currentgain;
			switch(axischar(state)){
			case x:
				switch(pidchar(state)){
				case p:
					px = currentgain;
					break;
				case i:
					ix = currentgain;
					break;
				case d:
					dx = currentgain;
					break;
				}
				xControl.SetTunings(px,ix,dx);
				break;
			case y:
				switch(pidchar(state)){
				case p:
					py = currentgain;
					break;
				case i:
					iy = currentgain;
					break;
				case d:
					dy = currentgain;
					break;
				}
				yControl.SetTunings(py,iy,dy);
				break;
			case z:
				switch(pidchar(state)){
				case p:
					pz = currentgain;
					break;
				case i:
					iz = currentgain;
					break;
				case d:
					dz = currentgain;
					break;
				}
				zControl.SetTunings(pz,iz,dz);
				break;
			case vert:
				switch(pidchar(state)){
				case p:
					pvert = currentgain;
					break;
				case i:
					ivert = currentgain;
					break;
				case d:
					dvert = currentgain;
					break;
				}
				vertControl.SetTunings(pvert,ivert,dvert);
				break;
			}
		}
//		Serial.println();
//		Serial.print("Kp");
//		Serial.print(xControl.GetKp());
	}
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
		}

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		mpu.dmpGetGyro(&gyro, fifoBuffer);
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
//		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
	}

}
