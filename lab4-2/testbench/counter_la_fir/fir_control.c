#include "fir_control.h"
#include <defs.h>
#include <stub.c>
#include <caravel.h>

#define reg_fir_control (*(volatile uint32_t*)0x30000000)
#define reg_fir_len (*(volatile uint32_t*)0x30000010)
#define reg_fir_coeff_0 (*(volatile uint32_t*)0x30000040)
#define reg_fir_coeff_1 (*(volatile uint32_t*)0x30000044)
#define reg_fir_coeff_2 (*(volatile uint32_t*)0x30000048)
#define reg_fir_coeff_3 (*(volatile uint32_t*)0x3000004C)
#define reg_fir_coeff_4 (*(volatile uint32_t*)0x30000050)
#define reg_fir_coeff_5 (*(volatile uint32_t*)0x30000054)
#define reg_fir_coeff_6 (*(volatile uint32_t*)0x30000058)
#define reg_fir_coeff_7 (*(volatile uint32_t*)0x3000005C)
#define reg_fir_coeff_8 (*(volatile uint32_t*)0x30000060)
#define reg_fir_coeff_9 (*(volatile uint32_t*)0x30000064)
#define reg_fir_coeff_10 (*(volatile uint32_t*)0x30000068)
#define reg_fir_x (*(volatile uint32_t*)0x30000080)
#define reg_fir_y (*(volatile uint32_t*)0x30000084)

//==============================================================
///////////////這邊是Ex4-1，刪掉跑比較快///////////////////////
//=============================================================

// void __attribute__ ( ( section ( ".mprjram" ) ) ) initfir() {
// 	for(int a = 0; a < N; a++) {
// 		inputbuffer[a] = 0;
// 		outputsignal[a] = 0;
// 	}
// }

// int* __attribute__ ( ( section ( ".mprjram" ) ) ) fir(){
// 	initfir();
	
// 	int outTemp;		//儲存output輸出outTemp
// 	int n32Data;
// 	int inTemp;		//儲存input做fir
// 	int n32Loop;
// 	int inSigLen;


// //XFER_LOOP:
// 	for (inSigLen = 0; inSigLen < N; inSigLen++) {
// 		outTemp = 0;
// 		inTemp = inputsignal[inSigLen];
// //SHIFT_ACC_LOOP:
// 		for (n32Loop = N - 1; n32Loop >= 0; n32Loop--) {
// 			if (n32Loop == 0) {
// 				inputbuffer[0] = inTemp;
// 				n32Data = inTemp;
// 			} else {
// 				inputbuffer[n32Loop] = inputbuffer[n32Loop - 1];
// 				n32Data = inputbuffer[n32Loop];
// 			}
// 			outTemp += n32Data * tap[n32Loop];
// 		}
// 		outputsignal[inSigLen] = outTemp;
// 	}
// 	return outputsignal;
// }
//===========================================================================
int* __attribute__ ( ( section ( ".mprjram" ) ) ) fircontrol(){

	// while(reg_fir_control != 0x00000006){}
	while(reg_fir_control != 0x00000004){}
	// reg_mprj_datal = 0x00A50000;//ap_start & tap parameter

	reg_fir_len = data_length;
	reg_fir_coeff_0 = tap[0];
	reg_fir_coeff_1 = tap[1];
	reg_fir_coeff_2 = tap[2];
	reg_fir_coeff_3 = tap[3];
	reg_fir_coeff_4 = tap[4];
	reg_fir_coeff_5 = tap[5];
	reg_fir_coeff_6 = tap[6];
	reg_fir_coeff_7 = tap[7];
	reg_fir_coeff_8 = tap[8];
	reg_fir_coeff_9 = tap[9];
	reg_fir_coeff_10 = tap[10];

	reg_mprj_datal = 0x00A50000;//ap_start & tap parameter
	reg_fir_control = 0x00000001;//ap_stert = 1

	for(int n=0; n<data_length; n++){

		while(reg_fir_control != 0x00000010) {}
		reg_fir_x = n;
		while(reg_fir_control != 0x00000020) {
			if(reg_fir_control == 0x00000024) {
				break;
			}
		}
		y[n] = reg_fir_y;
	}
	reg_mprj_datal = (0x005A00 | (y[63]<<16)) << 8;	

	return 0 ;
}