// Do not remove the include below
#include "mini.h"

int csnPin = 10;
int cePin = 9;


int buttonpressed;
const double aspsb = 0.8; //accelsetpointscaleback
double cgincr = 1;

PIDs posPID;
PIDs veloPID;

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

void saveCurrentGain(ControlState state){
	switch(quanchar(state)){
	case position:
		switch(axischar(state)){
		case x:
			switch(pidchar(state)){
			case p:
				posPID.x.p = state.currentgain;
				break;
			case i:
				posPID.x.i = state.currentgain;
				break;
			case d:
				posPID.x.d = state.currentgain;
				break;
			}
		break;
		case y:
			switch(pidchar(state)){
			case p:
				posPID.y.p = state.currentgain;
				break;
			case i:
				posPID.y.i = state.currentgain;
				break;
			case d:
				posPID.y.d = state.currentgain;
				break;
			}
		break;
		case z:
			switch(pidchar(state)){
			case p:
				posPID.z.p = state.currentgain;
				break;
			case i:
				posPID.z.i = state.currentgain;
				break;
			case d:
				posPID.z.d = state.currentgain;
				break;
			}
		break;
		case vert:
			switch(pidchar(state)){
			case p:
				posPID.vert.p = state.currentgain;
				break;
			case i:
				posPID.vert.i = state.currentgain;
				break;
			case d:
				posPID.vert.d = state.currentgain;
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
				veloPID.x.p = state.currentgain;
				break;
			case i:
				veloPID.x.i = state.currentgain;
				break;
			case d:
				veloPID.x.d = state.currentgain;
				break;
			}
		break;
		case y:
			switch(pidchar(state)){
			case p:
				veloPID.y.p = state.currentgain;
				break;
			case i:
				veloPID.y.i = state.currentgain;
				break;
			case d:
				veloPID.y.d = state.currentgain;
				break;
			}
		break;
		case z:
			switch(pidchar(state)){
			case p:
				veloPID.z.p = state.currentgain;
				break;
			case i:
				veloPID.z.i = state.currentgain;
				break;
			case d:
				veloPID.z.d = state.currentgain;
				break;
			}
		break;
		}
	}
}

double getNewCurrentGain(ControlState state){
	switch(quanchar(state)){
	case position:
		switch(axischar(state)){
			case x:
				switch(pidchar(state)){
				case p:
					return posPID.x.p;
				case i:
					return posPID.x.i;
				case d:
					return posPID.x.d;
				}
			break;
			case y:
				switch(pidchar(state)){
				case p:
					return posPID.y.p;
				case i:
					return posPID.y.i;
				case d:
					return posPID.y.d;
				}
			break;
			case z:
				switch(pidchar(state)){
				case p:
					return posPID.z.p;
				case i:
					return posPID.z.i;
				case d:
					return posPID.z.d;
				}
			break;
			case vert:
				switch(pidchar(state)){
				case p:
					return posPID.vert.p;
				case i:
					return posPID.vert.i;
				case d:
					return posPID.vert.d;
				}
			break;
			}
	case velocity:
		switch(axischar(state)){
			case x:
				switch(pidchar(state)){
				case p:
					return veloPID.x.p;
				case i:
					return veloPID.x.i;
				case d:
					return veloPID.x.d;
				}
			break;
			case y:
				switch(pidchar(state)){
				case p:
					return veloPID.y.p;
				case i:
					return veloPID.y.i;
				case d:
					return veloPID.y.d;
				}
			break;
			case z:
				switch(pidchar(state)){
				case p:
					return veloPID.z.p;
				case i:
					return veloPID.z.i;
				case d:
					return veloPID.z.d;
				}
			break;
			case vert:
				switch(pidchar(state)){
				case p:
					return veloPID.vert.p;
				case i:
					return veloPID.vert.i;
				case d:
					return veloPID.vert.d;
				}
			break;
			}
	}
	return 0;
}

void setup() {
	veloPID.x.p = 3; //.4//original 1.2 blue
	veloPID.x.i = 0;//0.05;//0.35; //1//.5 blue
	veloPID.x.d = 4;//.2; //.1//.5 blue
	posPID.y.p = 1;//.4;//1.2 blue
	posPID.y.i = 0; //.8 blue
	posPID.y.d = 0;//0.2; // .3 blue
	posPID.z.p = 1;
	posPID.z.i = 0;
	posPID.z.d = 0;
	posPID.vert.p = 1;
	posPID.vert.i = 0;
	posPID.vert.d = 0;
	veloPID.x.p = 1;
	state.currentgain = posPID.x.p;
	state.turnOn = false;
	state.vertaccelset = -20;
  Serial.begin(57600);
  Serial.println(F("Transmitter On"));
  radio.begin();

  radio.setPALevel(RF24_PA_LOW); //might turn up if need to at long range

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
    		setReset(state, false);
    		int ind = Serial.parseInt();
    		Serial.println("button value read from serial: ");
    		buttonpressed =  Serial.parseInt();
    		Serial.println(buttonpressed);
    		if((ind == 4 || ind == 6) && buttonpressed == 1){
    			state.currentgain = state.currentgain + ((ind == 4) ? cgincr : -cgincr);
    			saveCurrentGain(state);
    		}else if(ind == 11){
    			state.turnOn = (buttonpressed == 1 ? true:false);
    			Serial.println("turnOn");
    			Serial.println(state.turnOn);
    		}else if(ind == 10 && buttonpressed == 1){
    			state.turnOn = false;
    			Serial.println("turnOn");
    			Serial.println(state.turnOn);
    		}else if(ind == 13 && buttonpressed == 1){
    			setprintDebug(state, true);
    		}else if(ind == 13 && buttonpressed == 0){
    			setprintDebug(state, false);
    		}else if(ind == 14 && buttonpressed == 1){
    			saveCurrentGain(state);
    			incrementaxis(state);
    			state.currentgain = getNewCurrentGain(state);
    		}else if(ind == 15 && buttonpressed == 1){
    			saveCurrentGain(state);
    			incrementquantity(state);
    			state.currentgain = getNewCurrentGain(state);
    		}
    		else if(ind == 12 && buttonpressed == 1){
    			saveCurrentGain(state);
    			incrementpid(state);
    			state.currentgain = getNewCurrentGain(state);
    		}else if((ind == 5 || ind == 7) && buttonpressed == 1){
    			cgincr *= ((ind==7)? 0.1 : 10);
    		}else if((ind == 3) && buttonpressed == 1){
    			setReset(state, true);
    		}
    	}else{
    		Serial.println("Unknown Character");
    		Serial.println(c);
    	}
    }
}

