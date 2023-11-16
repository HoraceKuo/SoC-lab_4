#include "fir.h"

void __attribute__ ( ( section ( ".mprjram" ) ) ) initfir() {
	for(int a = 0; a < N; a++) {
		inputbuffer[a] = 0;
		outputsignal[a] = 0;
	}
}

int* __attribute__ ( ( section ( ".mprjram" ) ) ) fir(){
	initfir();
	
	int outTemp;		//儲存output輸出outTemp
	int n32Data;
	int inTemp;		//儲存input做fir
	int n32Loop;
	int inSigLen;


//XFER_LOOP:
	for (inSigLen = 0; inSigLen < N; inSigLen++) {
		outTemp = 0;
		inTemp = inputsignal[inSigLen];
//SHIFT_ACC_LOOP:
		for (n32Loop = N - 1; n32Loop >= 0; n32Loop--) {
			if (n32Loop == 0) {
				inputbuffer[0] = inTemp;
				n32Data = inTemp;
			} else {
				inputbuffer[n32Loop] = inputbuffer[n32Loop - 1];
				n32Data = inputbuffer[n32Loop];
			}
			outTemp += n32Data * taps[n32Loop];
		}
		outputsignal[inSigLen] = outTemp;
	}
		


	return outputsignal;
}
		
