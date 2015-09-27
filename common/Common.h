#include "Arduino.h"

#define printDebug B00000001
#define xyzvmask   B00000110
#define xcg        B00000000
#define ycg        B00000010
#define zcg        B00000100
#define vertcg     B00000110
#define pidmask    B00011000
#define pcg        B00000000
#define icg        B00001000
#define dcg        B00010000
#define quantity   B00100000 // 1 = position, 0 = velocity
#define RESET	   B01000000

struct ControlState {
	double xset;
	double yset;
	double zset;
	int vertaccelset;
	bool turnOn;
	//int buttons[2];
	double currentgain;
	byte kitchensink;
//	double setretx;
//	double setrety;
//	double vertset;
	double pwmoutx;
	double pwmouty;
	double pwmoutvert;
};

struct PIDStruct{
	double p;
	double i;
	double d;
	double input;
	double output;
	double setpoint;
};

struct PIDs{
	PIDStruct x;
	PIDStruct y;
	PIDStruct z;
	PIDStruct vert;
};

enum axes {
	x,
	y,
	z,
	vert
};

enum pid{
	p,
	i,
	d
};
enum quantities{
	position,
	velocity
};

quantities quanchar(ControlState state){
	return ((quantity & state.kitchensink) == quantity) ?  position : velocity;
}

void incrementquantity(ControlState &state){
	bool value = ((state.kitchensink & quantity) == quantity);
	state.kitchensink &= ~quantity;
	state.kitchensink |= (value ? B0 : quantity);
}

axes axischar(ControlState state){
	byte masked = xyzvmask & state.kitchensink;
	switch(masked){
	case xcg:
		return x;
	case ycg:
		return y;
	case zcg:
		return z;
	case vertcg:
		return vert;
	default: return x;
	}
}

void incrementaxis(ControlState &state){
	int xyzv = (int(xyzvmask & state.kitchensink)) + 2;
//	Serial.print(state.kitchensink, BIN);	Serial.println("\t");
//	Serial.print(xyzv, BIN); 	Serial.println("\t");
	state.kitchensink &= ~xyzvmask;
//	Serial.print(state.kitchensink, BIN);	Serial.println("\t");
	state.kitchensink |= (((byte)(xyzv)) & xyzvmask);
//	Serial.print(state.kitchensink, BIN);
//	Serial.println();
}
void incrementpid(ControlState &state){
	int incremented = (int(pidmask & state.kitchensink)) + 8;
	if(incremented > int(dcg))
		incremented = pcg;
	state.kitchensink &= ~pidmask;
	state.kitchensink |= (((byte) incremented) & pidmask);
}
void setprintDebug(ControlState &state, bool value){
	state.kitchensink |= (value ? printDebug : B0);
}

void setReset(ControlState &state, bool value){
	state.kitchensink |= (value ? RESET : B0);
}
bool getReset(ControlState &state){
	return (RESET & state.kitchensink) == RESET;
}

pid pidchar(ControlState state){
	byte masked = pidmask & state.kitchensink;
	switch(masked){
	case pcg:
		return p;
	case icg:
		return i;
	case dcg:
		return d;
	default: return p;
	}
}

bool printDebugbool(ControlState state){
	return (printDebug & state.kitchensink)  == printDebug;
}
