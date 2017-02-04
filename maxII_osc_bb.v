// megafunction wizard: %MAX II oscillator%VBB%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: altufm_osc 

// ============================================================
// File Name: maxII_osc.v
// Megafunction Name(s):
// 			altufm_osc
//
// Simulation Library Files(s):
// 			maxii
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 9.1 Build 350 03/24/2010 SP 2 SJ Full Version
// ************************************************************

//Copyright (C) 1991-2010 Altera Corporation
//Your use of Altera Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Altera Program License 
//Subscription Agreement, Altera MegaCore Function License 
//Agreement, or other applicable license agreement, including, 
//without limitation, that your use is for the sole purpose of 
//programming logic devices manufactured by Altera and sold by 
//Altera or its authorized distributors.  Please refer to the 
//applicable agreement for further details.

module maxII_osc (
	oscena,
	osc)/* synthesis synthesis_clearbox = 1 */;

	input	  oscena;
	output	  osc;

endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "MAX II"
// Retrieval info: PRIVATE: INTENDED_DEVICE_PART STRING ""
// Retrieval info: PRIVATE: INTERFACE_CHOICE NUMERIC "4"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: VERSION_NUMBER NUMERIC "0"
// Retrieval info: CONSTANT: OSC_FREQUENCY NUMERIC "180000"
// Retrieval info: USED_PORT: osc 0 0 0 0 OUTPUT NODEFVAL osc
// Retrieval info: USED_PORT: oscena 0 0 0 0 INPUT NODEFVAL oscena
// Retrieval info: CONNECT: @oscena 0 0 0 0 oscena 0 0 0 0
// Retrieval info: CONNECT: osc 0 0 0 0 @osc 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL maxII_osc.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL maxII_osc.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL maxII_osc.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL maxII_osc.bsf TRUE FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL maxII_osc_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL maxII_osc_bb.v TRUE
// Retrieval info: LIB_FILE: maxii
