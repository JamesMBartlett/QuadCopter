// Do not remove the include below
#include "mini.h"

int csnPin = 10;
int cePin = 9;


int buttonpressed;
const double aspsb = 0.8; //accelsetpointscaleback
const double pcgincr = .1;
const double icgincr = .1;
const double dcgincr = .01;

double px = 6.9; //.4//original 1.2 blue
double ix = 0;//0.05;//0.35; //1//.5 blue
double dx = 0;//.2; //.1//.5 blue
double py = 1.3;//.4;//1.2 blue
double iy = 0; //.8 blue
double dy = 0;//0.2; // .3 blue
double pz = 1;
double iz = 0;
double dz = 0;
double pvert = 1;
double ivert = 0;
double dvert = 0;

ControlState state;

bool radioNumber = 0;
RF24 radio(cePin, csnPin);
byte addresses[][6] = {"1Node","2Node"};

char pidcharchar(ControlState state){
	switch(pidchar(state)){
	case p:	 return 'p';
	case i:	 return 'i';
	case d:  return 'd';
	default: return 'B';
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
	Serial.print(axischarchar(state));
	Serial.print(pidcharchar(state));
	Serial.print("] x ");
	Serial.print(px); Serial.print(" ");
	Serial.print(ix); Serial.print(" ");
	Serial.print(dx);
	Serial.print(" y ");
	Serial.print(py); Serial.print(" ");
	Serial.print(iy); Serial.print(" ");
	Serial.print(dy);
	Serial.print(" z ");
	Serial.print(pz); Serial.print(" ");
	Serial.print(iz); Serial.print(" ");
	Serial.print(dz);
	Serial.print(" v ");
	Serial.print(pvert); Serial.print(" ");
	Serial.print(ivert); Serial.print(" ");
	Serial.print(dvert);
	Serial.println();
}

void saveCurrentGain(ControlState state){
	switch(axischar(state)){
	case x:
		switch(pidchar(state)){
		case p:
			px = state.currentgain;
			break;
		case i:
			ix = state.currentgain;
			break;
		case d:
			dx = state.currentgain;
			break;
		}
	break;
	case y:
		switch(pidchar(state)){
		case p:
			py = state.currentgain;
			break;
		case i:
			iy = state.currentgain;
			break;
		case d:
			dy = state.currentgain;
			break;
		}
	break;
	case z:
		switch(pidchar(state)){
		case p:
			pz = state.currentgain;
			break;
		case i:
			iz = state.currentgain;
			break;
		case d:
			dz = state.currentgain;
			break;
		}
	break;
	case vert:
		switch(pidchar(state)){
		case p:
			pvert = state.currentgain;
			break;
		case i:
			ivert = state.currentgain;
			break;
		case d:
			dvert = state.currentgain;
			break;
		}
	break;
	}
}
double getNewCurrentGain(ControlState state){
	switch(axischar(state)){
		case x:
			switch(pidchar(state)){
			case p:
				return px;
			case i:
				return ix;
			case d:
				return dx;
			}
		break;
		case y:
			switch(pidchar(state)){
			case p:
				return py;
			case i:
				return iy;
			case d:
				return dy;
			}
		break;
		case z:
			switch(pidchar(state)){
			case p:
				return pz;
			case i:
				return iz;
			case d:
				return dz;
			}
		break;
		case vert:
			switch(pidchar(state)){
			case p:
				return pvert;
			case i:
				return ivert;
			case d:
				return dvert;
			}
		break;
		}
	return 0;
}

void setup() {
	state.currentgain = px;
	state.turnOn = false;
	state.vertaccelset = -20;
  Serial.begin(57600);
  Serial.println(F("Transmitter On"));
  radio.begin();

  radio.setPALevel(RF24_PA_HIGH); //might turn up if need to at long range

  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }

  radio.startListening();
}
void loop() {
    radio.stopListening();
    if (!radio.write( &state, sizeof(ControlState) )){
       Serial.println(F("failed"));
    }
    if(printDebugbool(state)){
		radio.startListening();

		unsigned long started_waiting_at = micros();
		boolean timeout = false;

		while ( ! radio.available() ){
		  if (micros() - started_waiting_at > 200000 ){
			  timeout = true;
			  break;
		  }
		}

		if ( timeout ){
			//Serial.println(F("Failed, response timed out."));
		}else{
			ControlState state_ret;
			radio.read( &state_ret, sizeof(ControlState) );
//			Serial.println();
//			Serial.print("state.printDebug");
//			Serial.print(state.printDebug);
//			Serial.print(F("pwmsets"));
//			Serial.print("\t");
//			Serial.print(state_ret.pwmset[0]);
//			Serial.print("\t");
//			Serial.print(state_ret.pwmset[1]);
//			Serial.print("\t");
//			Serial.print(state_ret.pwmset[2]);
//			Serial.print("\t");
//			Serial.print(state_ret.pwmset[3]);
			Serial.println();
			Serial.print(F("pwms"));
//			Serial.print("\t");
//			Serial.print(state_ret.setretx);
//			Serial.print("\t");
//			Serial.print(state_ret.setrety);
//			Serial.print("\t");
//			Serial.print(state_ret.vertset);
			Serial.print("\t");
			Serial.print(state_ret.pwmoutx);
			Serial.print("\t");
			Serial.print(state_ret.pwmouty);
			Serial.print("\t");
			Serial.print(state_ret.pwmoutvert);
			Serial.println();
	//        Serial.print(F("Sent "));
	//        Serial.print(state.axes[0]);
	//        Serial.print(",");
	//        Serial.print(state.axes[1]);
	//        Serial.print(F(", Got response "));
	//        Serial.print(state_ret.currentgain);
	//        Serial.print(",");
	//        Serial.print(state_ret.axes[1]);
	//	    Serial.println();
		}
    }
    printgains(state);

    if(Serial.available()){
    	char c = toupper(Serial.read());
    	//Serial.println("Serial received");
    	//Serial.println(c);
    	if (c == 'A'){ //data being given about axis
    		int ind = Serial.parseInt();
    		if (ind == 0){
    			state.zset = double(Serial.parseInt()) / 500;
    		}
    		else if (ind == 1){
    			state.vertaccelset = (double(Serial.parseInt()) + 100) * aspsb - 100;
    			//state.vertaccelset = Serial.parseInt();
    		}else if (ind == 2){
    			state.xset = double(Serial.parseInt()) / 500;
    			Serial.println(state.xset);
    		}else if (ind == 3){
    			state.yset = double(Serial.parseInt()) / 500;
    			Serial.println(state.yset);
    		}
//    		Serial.println("axis value read from serial: ");
//    		Serial.print(state.axes[ind]);
//    		Serial.println();
    	}else if (c == 'B') {//data being given about buttons
    		int ind = Serial.parseInt();
    		Serial.println("button value read from serial: ");
    		buttonpressed =  Serial.parseInt();
    		Serial.println(buttonpressed);
    		if((ind == 4 || ind == 6) && buttonpressed == 1){
    			switch(pidchar(state)){
    			case p:
    				state.currentgain = state.currentgain + ((ind == 4) ? pcgincr : -pcgincr);
    				break;
    			case i:
    				state.currentgain = state.currentgain + ((ind == 4) ? icgincr : -icgincr);
    				break;
    			case d:
    				state.currentgain = state.currentgain + ((ind == 4) ? dcgincr : -dcgincr);
    				break;
    			}
    			saveCurrentGain(state);
    		}else if(ind == 11){
    			state.turnOn = (buttonpressed == 1 ? true:false);
    			Serial.println("turnOn");
    			Serial.println(state.turnOn);
    		}else if(ind == 10 && buttonpressed == 1){
    			state.turnOn = false;
    			Serial.println("turnOn");
    			Serial.println(state.turnOn);
    		}//else if(ind == 13 && buttonpressed == 1){
    			//setprintDebug(state, true);
    		else if(ind == 13 && buttonpressed == 0){
    			setprintDebug(state, false);
    		}else if(ind == 14 && buttonpressed == 1){
    			saveCurrentGain(state);
    			incrementaxis(state);
    			state.currentgain = getNewCurrentGain(state);
    		}else if(ind == 12 && buttonpressed == 1){
    			saveCurrentGain(state);
    			incrementpid(state);
    			state.currentgain = getNewCurrentGain(state);
    		}
    	}else{
    		Serial.println("Unknown Character");
    		Serial.println(c);
    	}
    }
}

