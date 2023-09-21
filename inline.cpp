/*########################################################################################################*/
// cd /nfs/iil/ptl/bt/ghaber1/pin/pin-2.10-45467-gcc.3.4.6-ia32_intel64-linux/source/tools/SimpleExamples
// make
//  ../../../pin -t obj-intel64/rtn-translation.so -- ~/workdir/tst
/*########################################################################################################*/
/*BEGIN_LEGAL 
Intel Open Source License 

Copyright (c) 2002-2011 Intel Corporation. All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.  Redistributions
in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.  Neither the name of
the Intel Corporation nor the names of its contributors may be used to
endorse or promote products derived from this software without
specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL OR
ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
END_LEGAL */
/* ===================================================================== */

/* ===================================================================== */
/*! @file
 * This probe pintool generates translated code of routines, places them in an allocated TC 
 * and patches the orginal code to jump to the translated routines.
 */

#include "pin.H"
extern "C" {
#include "xed-interface.h"
}
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>
#include <errno.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <values.h>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <unordered_set>
#include <chrono>


using namespace std;
using std::cerr;
using std::endl;

typedef struct bbl_struct {
	ADDRINT routine_address;
	ADDRINT head_address;
	ADDRINT tail_address;
	ADDRINT FT_target;
	ADDRINT target;
	ADDRINT next_ins;
	
	unsigned long instructions_num; // number of dynamic instructions in the bbl 
	unsigned long taken_count;
	unsigned long not_taken_count;
	
	//for probe mode
	bool was_reordered;
} bbl_st;

//In this struct every routine's data is saved 
typedef struct rtn_struct {
	unsigned int routine_address;

	unsigned long instructions_num;
	unsigned long calls_num ;
	
	map<ADDRINT,bbl_st> rtn_bbls_map;
	map <ADDRINT, unsigned long> calls_map;

} rtn_st;




/*======================================================================*/
/* commandline switches                                                 */
/*======================================================================*/
KNOB<BOOL>   KnobVerbose(KNOB_MODE_WRITEONCE,    "pintool",
    "verbose", "0", "Verbose run");

KNOB<BOOL>   KnobDumpTranslatedCode(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_tc", "0", "Dump Translated Code");

KNOB<BOOL>   KnobDoNotCommitTranslatedCode(KNOB_MODE_WRITEONCE,    "pintool",
    "no_tc_commit", "0", "Do not commit translated code");

KNOB<BOOL>   KnobProf(KNOB_MODE_WRITEONCE,    "pintool",
    "prof", "0", "run profiler");

KNOB<BOOL>   KnobOpt(KNOB_MODE_WRITEONCE,    "pintool",
    "opt", "0", "run inst");

/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */
std::ofstream* out = 0;

map <ADDRINT,rtn_st*> rtn_map;
map <ADDRINT,bbl_st*> bbl_map;
vector<pair<ADDRINT,ADDRINT>> call_vector;
vector<ADDRINT> hot_calls_vector;


// For XED:
#if defined(TARGET_IA32E)
    xed_state_t dstate = {XED_MACHINE_MODE_LONG_64, XED_ADDRESS_WIDTH_64b};
#else
    xed_state_t dstate = { XED_MACHINE_MODE_LEGACY_32, XED_ADDRESS_WIDTH_32b};
#endif

//For XED: Pass in the proper length: 15 is the max. But if you do not want to
//cross pages, you can pass less than 15 bytes, of course, the
//instruction might not decode if not enough bytes are provided.
const unsigned int max_inst_len = XED_MAX_INSTRUCTION_BYTES;

ADDRINT lowest_sec_addr = 0;
ADDRINT highest_sec_addr = 0;

#define MAX_PROBE_JUMP_INSTR_BYTES  14

// tc containing the new code:
char *tc;	
int tc_cursor = 0;
int tmp_tc_cursor = 0;
int inline_counter = 0;

// instruction map with an entry for each new instruction:
typedef struct { 
	ADDRINT orig_ins_addr;
	ADDRINT new_ins_addr;
	ADDRINT orig_targ_addr;
	bool hasNewTargAddr;
	char encoded_ins[XED_MAX_INSTRUCTION_BYTES];
	xed_category_enum_t category_enum;
	unsigned int size;
	int targ_map_entry;
	int inline_index; //OUR ADDITION
} instr_map_t;

typedef struct { 
	ADDRINT orig_ins_addr;
	ADDRINT orig_targ_addr;
	//xed_decoded_inst_t xedd;
} tmp_instr_map_t;

instr_map_t *instr_map = NULL;
tmp_instr_map_t *tmp_instr_map = NULL;
int num_of_instr_map_entries = 0;
int num_of_instr_map_entries_tmp = 0;
int max_ins_count = 0;


// total number of routines in the main executable module:
int max_rtn_count = 0;

// Tables of all candidate routines to be translated:
typedef struct { 
	ADDRINT rtn_addr; 
	USIZE rtn_size;
	int instr_map_entry;   // negative instr_map_entry means routine does not have a translation.
	bool isSafeForReplacedProbe;	
} translated_rtn_t;

translated_rtn_t *translated_rtn;
int translated_rtn_num = 0;
map <ADDRINT,pair<int,int> > tmp_rtn_map;// map that has the rtn_address of each rtn as key and as value it has a pair: < first index in the temp inst_map_entry,last index in the temp inst_map_entry >



/* ============================================================= */
/* Service dump routines                                         */
/* ============================================================= */


/* ===================  FUNCTIONS================================================== */

// a compare function to sort the map by ins_count
bool compare_rtn_by_inst (const pair<unsigned int, rtn_st*>& a , const pair<unsigned int, rtn_st*>& b)
{
	return a.second->instructions_num > b.second->instructions_num;
}

bool compare_rtn_by_calls (const pair<unsigned int, rtn_st*>& a , const pair<unsigned int, rtn_st*>& b)
{
	return a.second->calls_num > b.second->calls_num;
}


bool compare_bbl (const pair<unsigned int, bbl_st*>& a , const pair<unsigned int, bbl_st*>& b)
{
	return a.second->instructions_num > b.second->instructions_num;
}

bool compare_bbl_by_tail (const pair<unsigned int, bbl_st*>& a , const pair<unsigned int, bbl_st*>& b)
{
	return a.second->tail_address > b.second->tail_address;
}


/* ===================================================================== */

/* ===================================================================== */
/* =====================READ AND WRITE BINARY FILES======================================= */

/* ===================================================================== */
void writeBinary(const std::string& filePath) {
    std::ofstream outFile(filePath, std::ios::binary);

    // Write the number of routines
    size_t numRoutines = rtn_map.size();
    outFile.write(reinterpret_cast<char*>(&numRoutines), sizeof(numRoutines));

    for (auto it = rtn_map.begin(); it != rtn_map.end(); ++it) {
        // Write routine data
        outFile.write(reinterpret_cast<const char*>(&it->second->routine_address), sizeof(it->second->routine_address));
        outFile.write(reinterpret_cast<const char*>(&it->second->instructions_num), sizeof(it->second->instructions_num));
        outFile.write(reinterpret_cast<const char*>(&it->second->calls_num), sizeof(it->second->calls_num));

        // Write number of BBLs in this routine
        size_t numBBLs = it->second->rtn_bbls_map.size();
        outFile.write(reinterpret_cast<const char*>(&numBBLs), sizeof(numBBLs));

        for (auto bblIt = it->second->rtn_bbls_map.begin(); bblIt != it->second->rtn_bbls_map.end(); ++bblIt) {
            // Write BBL address (key)
            ADDRINT bblAddr = bblIt->first;
            outFile.write(reinterpret_cast<const char*>(&bblAddr), sizeof(bblAddr));
            
            // Write BBL data
            bbl_st bblData = bblIt->second;
            outFile.write(reinterpret_cast<const char*>(&bblData), sizeof(bblData));
        }

        // Write number of calls in calls_map
        size_t numCalls = it->second->calls_map.size();
        outFile.write(reinterpret_cast<const char*>(&numCalls), sizeof(numCalls));

        // Write calls_map data
        for(auto callsIt = it->second->calls_map.begin(); callsIt != it->second->calls_map.end(); ++callsIt) {
            ADDRINT callAddr = callsIt->first;
            unsigned long callCount = callsIt->second;
            outFile.write(reinterpret_cast<const char*>(&callAddr), sizeof(callAddr));
            outFile.write(reinterpret_cast<const char*>(&callCount), sizeof(callCount));
        }
    }
	
	size_t numHotCalls = hot_calls_vector.size();
	outFile.write(reinterpret_cast<const char*>(&numHotCalls), sizeof(numHotCalls));

	// Write hot_calls_vector data
	for(const auto& callAddr : hot_calls_vector) {
		outFile.write(reinterpret_cast<const char*>(&callAddr), sizeof(callAddr));
	}
	
    outFile.close();
}

void readBinary(const std::string& filePath) {
     std::ifstream inFile(filePath, std::ios::binary);

    // Read the number of routines
    size_t numRoutines;
    inFile.read(reinterpret_cast<char*>(&numRoutines), sizeof(numRoutines));

    for (size_t i = 0; i < numRoutines; ++i) {
        rtn_st* routineData = new(rtn_st);

        // Read routine data
        inFile.read(reinterpret_cast<char*>(&routineData->routine_address), sizeof(routineData->routine_address));
        inFile.read(reinterpret_cast<char*>(&routineData->instructions_num), sizeof(routineData->instructions_num));
        inFile.read(reinterpret_cast<char*>(&routineData->calls_num), sizeof(routineData->calls_num));

        // Read number of BBLs in this routine
        size_t numBBLs;
        inFile.read(reinterpret_cast<char*>(&numBBLs), sizeof(numBBLs));

        for (size_t j = 0; j < numBBLs; ++j) {
            ADDRINT bblAddr;
            bbl_st bblData;

            // Read BBL address (key)
            inFile.read(reinterpret_cast<char*>(&bblAddr), sizeof(bblAddr));
            
            // Read BBL data
            inFile.read(reinterpret_cast<char*>(&bblData), sizeof(bblData));

            // Add BBL data to map
            routineData->rtn_bbls_map[bblAddr] = bblData;
        }

        // Read number of calls in calls_map
        size_t numCalls;
        inFile.read(reinterpret_cast<char*>(&numCalls), sizeof(numCalls));

        // Read calls_map data
        for(size_t k = 0; k < numCalls; ++k) {
            ADDRINT callAddr;
            unsigned long callCount;
            inFile.read(reinterpret_cast<char*>(&callAddr), sizeof(callAddr));
            inFile.read(reinterpret_cast<char*>(&callCount), sizeof(callCount));

            // Add data to calls_map
            routineData->calls_map[callAddr] = callCount;
        }

        // Add routine data to map
        rtn_map[routineData->routine_address] = routineData;
    }
	
// Read number of elements in hot_calls_vector
	size_t numHotCalls;
	inFile.read(reinterpret_cast<char*>(&numHotCalls), sizeof(numHotCalls));

	// Read hot_calls_vector data
	hot_calls_vector.resize(numHotCalls); // Ensure there's enough space in the vector
	for(size_t i = 0; i < numHotCalls; ++i) {
		ADDRINT callAddr;
		inFile.read(reinterpret_cast<char*>(&callAddr), sizeof(callAddr));

		// Add data to hot_calls_vector
		hot_calls_vector[i] = callAddr;
	}
	
    inFile.close();
}
/*************************/
/* invert conditional jumps */
/*************************/
void invert_jmp(xed_decoded_inst_t* xedd){
	/*
	This funtion gets a conditional jump xed inst, and invertes the condition and the target address to FT
	inputs: xed_decoded_inst_t the jump to invert, target_address - Fallthrough address
	*/
	
		xed_category_enum_t category_enum = xed_decoded_inst_get_category(xedd);

		if (category_enum != XED_CATEGORY_COND_BR) 
			return;

		xed_iclass_enum_t iclass_enum = xed_decoded_inst_get_iclass(xedd);

  		if (iclass_enum == XED_ICLASS_JRCXZ)
			return;    // do not revert JRCXZ

		xed_iclass_enum_t 	retverted_iclass;

		switch (iclass_enum) {

			case XED_ICLASS_JB:
				retverted_iclass = XED_ICLASS_JNB;		
				break;

			case XED_ICLASS_JBE:
				retverted_iclass = XED_ICLASS_JNBE;
				break;

			case XED_ICLASS_JL:
				retverted_iclass = XED_ICLASS_JNL;
				break;
		
			case XED_ICLASS_JLE:
				retverted_iclass = XED_ICLASS_JNLE;
				break;

			case XED_ICLASS_JNB: 
			    retverted_iclass = XED_ICLASS_JB;
				break;

			case XED_ICLASS_JNBE: 
				retverted_iclass = XED_ICLASS_JBE;
				break;

			case XED_ICLASS_JNL:
			retverted_iclass = XED_ICLASS_JL;
				break;

			case XED_ICLASS_JNLE:
				retverted_iclass = XED_ICLASS_JLE;
				break;

			case XED_ICLASS_JNO:
				retverted_iclass = XED_ICLASS_JO;
				break;

			case XED_ICLASS_JNP: 
				retverted_iclass = XED_ICLASS_JP;
				break;

			case XED_ICLASS_JNS: 
				retverted_iclass = XED_ICLASS_JS;
				break;

			case XED_ICLASS_JNZ:
				retverted_iclass = XED_ICLASS_JZ;
				break;

			case XED_ICLASS_JO:
				retverted_iclass = XED_ICLASS_JNO;
				break;

			case XED_ICLASS_JP: 
			    retverted_iclass = XED_ICLASS_JNP;
				break;

			case XED_ICLASS_JS: 
				retverted_iclass = XED_ICLASS_JNS;
				break;

			case XED_ICLASS_JZ:
				retverted_iclass = XED_ICLASS_JNZ;
				break;
	
			default:
				return;
		}

		// Converts the decoder request to a valid encoder request:
		xed_encoder_request_init_from_decode (xedd);

		// set the reverted opcode;
		xed_encoder_request_set_iclass	(xedd, retverted_iclass);

		xed_uint8_t enc_buf[XED_MAX_INSTRUCTION_BYTES];
		unsigned int max_size = XED_MAX_INSTRUCTION_BYTES;
		unsigned int new_size = 0;
    
		xed_error_enum_t xed_error = xed_encode (xedd, enc_buf, max_size, &new_size);
		if (xed_error != XED_ERROR_NONE) {
			cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) <<  endl;
			return;
		}		
		

		//print the original and the new reverted cond instructions:
		//
		//cerr << "orig instr: " << hex << INS_Address(ins_tail) << " " << INS_Disassemble(ins_tail) << endl;

		xed_decoded_inst_zero_set_mode(xedd,&dstate);
		
		// set input command to inverted command
		xed_error_enum_t xed_code = xed_decode(xedd, enc_buf, XED_MAX_INSTRUCTION_BYTES);
		if (xed_code != XED_ERROR_NONE) {
		//	cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << INS_Address(ins_tail) << endl;
			cerr << "ERROR: inverted jump decode failed" << endl;
			return;
		}

	
}

/*************************/
/* add final jmp after reorder */
/*************************/
void add_final_jmp(xed_decoded_inst_t* xedd){
	
	unsigned int max_size = XED_MAX_INSTRUCTION_BYTES;
	unsigned int new_size = 0;
	xed_uint8_t enc_buf2[XED_MAX_INSTRUCTION_BYTES];		
		xed_int32_t disp = xed_decoded_inst_get_branch_displacement(xedd);
		xed_encoder_instruction_t  enc_instr;

		xed_inst1(&enc_instr, dstate, 
				XED_ICLASS_JMP, 64,
				xed_relbr(disp, 32));
                                
		xed_encoder_request_t enc_req;

		xed_encoder_request_zero_set_mode(&enc_req, &dstate);
		xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
		if (!convert_ok) {
			cerr << "conversion to encode request failed" << endl;
			return;
		}

		xed_error_enum_t xed_error = xed_encode (&enc_req, enc_buf2, max_size, &new_size);
		if (xed_error != XED_ERROR_NONE) {
			cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;				
			return;
		}

		xed_decoded_inst_zero_set_mode(xedd,&dstate);
		xed_error_enum_t xed_code = xed_decode(xedd, enc_buf2, XED_MAX_INSTRUCTION_BYTES);
		if (xed_code != XED_ERROR_NONE) {
			cerr << "ERROR: new final jmp decode failed" << endl;
			return;
		}
		
}


/*************************/
/* dump_all_image_instrs */
/*************************/
void dump_all_image_instrs(IMG img)
{
	for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {   
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {		

			// Open the RTN.
            RTN_Open( rtn );

			cerr << RTN_Name(rtn) << ":" << endl;

			for( INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins) )
            {				
	              cerr << "0x" << hex << INS_Address(ins) << ": " << INS_Disassemble(ins) << endl;
			}

			// Close the RTN.
            RTN_Close( rtn );
		}
	}
}


/*************************/
/* dump_instr_from_xedd */
/*************************/
void dump_instr_from_xedd (xed_decoded_inst_t* xedd, ADDRINT address)
{
	// debug print decoded instr:
	char disasm_buf[2048];

    xed_uint64_t runtime_address = static_cast<UINT64>(address);  // set the runtime adddress for disassembly 	

    xed_format_context(XED_SYNTAX_INTEL, xedd, disasm_buf, sizeof(disasm_buf), static_cast<UINT64>(runtime_address), 0, 0);	

    cerr << hex << address << ": " << disasm_buf <<  endl;
}


/************************/
/* dump_instr_from_mem */
/************************/
void dump_instr_from_mem (ADDRINT *address, ADDRINT new_addr)
{
  char disasm_buf[2048];
  xed_decoded_inst_t new_xedd;

  xed_decoded_inst_zero_set_mode(&new_xedd,&dstate); 
   
  xed_error_enum_t xed_code = xed_decode(&new_xedd, reinterpret_cast<UINT8*>(address), max_inst_len);				   

  BOOL xed_ok = (xed_code == XED_ERROR_NONE);
  if (!xed_ok){
	  cerr << "invalid opcode" << endl;
	  return;
  }
 
  xed_format_context(XED_SYNTAX_INTEL, &new_xedd, disasm_buf, 2048, static_cast<UINT64>(new_addr), 0, 0);

  cerr << "0x" << hex << new_addr << ": " << disasm_buf <<  endl;  
 
}


/****************************/
/*  dump_entire_instr_map() */
/****************************/
void dump_entire_instr_map()
{	
	for (int i=0; i < num_of_instr_map_entries; i++) {
		for (int j=0; j < translated_rtn_num; j++) {
			if (translated_rtn[j].instr_map_entry == i) {

				RTN rtn = RTN_FindByAddress(translated_rtn[j].rtn_addr);

				if (rtn == RTN_Invalid()) {
					cerr << "Unknwon"  << ":" << endl;
				} else {
				  cerr << RTN_Name(rtn) << ":" << endl;
				}
			}
		}
		dump_instr_from_mem ((ADDRINT *)instr_map[i].new_ins_addr, instr_map[i].new_ins_addr);		
	}
}


/**************************/
/* dump_instr_map_entry */
/**************************/
void dump_instr_map_entry(int instr_map_entry)
{
	cerr << dec << instr_map_entry << ": ";
	cerr << " orig_ins_addr: " << hex << instr_map[instr_map_entry].orig_ins_addr;
	cerr << " new_ins_addr: " << hex << instr_map[instr_map_entry].new_ins_addr;
	cerr << " orig_targ_addr: " << hex << instr_map[instr_map_entry].orig_targ_addr;

	ADDRINT new_targ_addr;
	if (instr_map[instr_map_entry].targ_map_entry >= 0)
		new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;
	else
		new_targ_addr = instr_map[instr_map_entry].orig_targ_addr;

	cerr << " new_targ_addr: " << hex << new_targ_addr;
	cerr << "    new instr:";
	dump_instr_from_mem((ADDRINT *)instr_map[instr_map_entry].encoded_ins, instr_map[instr_map_entry].new_ins_addr);
}


/*************/
/* dump_tc() */
/*************/
void dump_tc()
{
  char disasm_buf[2048];
  xed_decoded_inst_t new_xedd;
  ADDRINT address = (ADDRINT)&tc[0];
  unsigned int size = 0;

  while (address < (ADDRINT)&tc[tc_cursor]) {

      address += size;

	  xed_decoded_inst_zero_set_mode(&new_xedd,&dstate); 
   
	  xed_error_enum_t xed_code = xed_decode(&new_xedd, reinterpret_cast<UINT8*>(address), max_inst_len);				   

	  BOOL xed_ok = (xed_code == XED_ERROR_NONE);
	  if (!xed_ok){
		  cerr << "invalid opcode" << endl;
		  return;
	  }
 
	  xed_format_context(XED_SYNTAX_INTEL, &new_xedd, disasm_buf, 2048, static_cast<UINT64>(address), 0, 0);

	  cerr << "0x" << hex << address << ": " << disasm_buf <<  endl;

	  size = xed_decoded_inst_get_length (&new_xedd);	
  }
}


/* ============================================================= */
/* Translation routines                                         */
/* ============================================================= */


/*************************/
/* add_new_instr_entry() */
/*************************/
//iggy
int add_new_instr_entry(xed_decoded_inst_t *xedd, ADDRINT pc, unsigned int size , int inline_counter , ADDRINT orig_targ_addr)
{

	// copy orig instr to instr map:
	if (xed_decoded_inst_get_length (xedd) != size) {
		cerr << "Invalid instruction decoding" << endl;
		return -1;
	}

	if(orig_targ_addr==0){
		xed_uint_t disp_byts = xed_decoded_inst_get_branch_displacement_width(xedd);
		xed_int32_t disp;
		if (disp_byts > 0) { // there is a branch offset.
			disp = xed_decoded_inst_get_branch_displacement(xedd);
			orig_targ_addr = pc + xed_decoded_inst_get_length (xedd) + disp;	
		}
	}

	// Converts the decoder request to a valid encoder request:
	if (xedd != NULL) {
		xed_encoder_request_init_from_decode(xedd);
	} else {
		// Handle the error or print a warning.
	}
	
	
	xed_encoder_request_init_from_decode (xedd);
    unsigned int new_size = 0;
	
	xed_error_enum_t xed_error = xed_encode (xedd, reinterpret_cast<UINT8*>(instr_map[num_of_instr_map_entries].encoded_ins), max_inst_len , &new_size);
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;		
		return -1;
	}	
	// add a new entry in the instr_map:
	
	instr_map[num_of_instr_map_entries].orig_ins_addr = pc;
	instr_map[num_of_instr_map_entries].new_ins_addr = (ADDRINT)&tc[tc_cursor];  // set an initial estimated addr in tc
	instr_map[num_of_instr_map_entries].orig_targ_addr = orig_targ_addr; 
    instr_map[num_of_instr_map_entries].hasNewTargAddr = false;
	instr_map[num_of_instr_map_entries].targ_map_entry = -1;
	instr_map[num_of_instr_map_entries].size = new_size;	
    instr_map[num_of_instr_map_entries].category_enum = xed_decoded_inst_get_category(xedd);
	instr_map[num_of_instr_map_entries].inline_index = inline_counter;//OUR ADDITION
	num_of_instr_map_entries++;

	// update expected size of tc:
	tc_cursor += new_size;    	     

	if (num_of_instr_map_entries >= max_ins_count) {
		cerr << "out of memory for map_instr" << endl;
		return -1;
	}
	
    // debug print new encoded instr:
	if (KnobVerbose) {
		cerr << "    new instr:";
		dump_instr_from_mem((ADDRINT *)instr_map[num_of_instr_map_entries-1].encoded_ins, instr_map[num_of_instr_map_entries-1].new_ins_addr);
	}

	return new_size;
}
/*************************************************/
/* add_new_instr_entry() TO TMP MAP (TO BE USED BETWEEN REORDER AND INLINE PHASES) */
/*************************************************/
/*
void add_new_instr_entry_to_tmp_map( ADDRINT pc , ADDRINT orig_targ_addr)
{

}
*/
/*************************************************/
/* chain_all_direct_br_and_call_target_entries() */
/*************************************************/
int chain_all_direct_br_and_call_target_entries()
{
	for (int i=0; i < num_of_instr_map_entries; i++) {			    

		if (instr_map[i].orig_targ_addr == 0)
			continue;

		if (instr_map[i].hasNewTargAddr)
			continue;

        for (int j = 0; j < num_of_instr_map_entries; j++) {

            if (j == i)
			   continue;
	
            if ((instr_map[j].orig_ins_addr == instr_map[i].orig_targ_addr) && ( instr_map[j].inline_index == instr_map[i].inline_index)  ) {
                instr_map[i].hasNewTargAddr = true; 
	            instr_map[i].targ_map_entry = j;
                break;
			}
		}
	}
   
	return 0;
}


/**************************/
/* fix_rip_displacement() */
/**************************/
int fix_rip_displacement(int instr_map_entry) 
{
	//debug print:
	//dump_instr_map_entry(instr_map_entry);

	xed_decoded_inst_t xedd;
	xed_decoded_inst_zero_set_mode(&xedd,&dstate); 
				   
	xed_error_enum_t xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
	if (xed_code != XED_ERROR_NONE) {
		cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << instr_map[instr_map_entry].new_ins_addr << endl;
		return -1;
	}

	unsigned int memops = xed_decoded_inst_number_of_memory_operands(&xedd);

	if (instr_map[instr_map_entry].orig_targ_addr != 0)  // a direct jmp or call instruction.
		return 0;

	//cerr << "Memory Operands" << endl;
	bool isRipBase = false;
	xed_reg_enum_t base_reg = XED_REG_INVALID;
	xed_int64_t disp = 0;
	for(unsigned int i=0; i < memops ; i++)   {

		base_reg = xed_decoded_inst_get_base_reg(&xedd,i);
		disp = xed_decoded_inst_get_memory_displacement(&xedd,i);

		if (base_reg == XED_REG_RIP) {
			isRipBase = true;
			break;
		}
		
	}

	if (!isRipBase)
		return 0;

			
	//xed_uint_t disp_byts = xed_decoded_inst_get_memory_displacement_width(xedd,i); // how many byts in disp ( disp length in byts - for example FFFFFFFF = 4
	xed_int64_t new_disp = 0;
	xed_uint_t new_disp_byts = 4;   // set maximal num of byts for now.

	unsigned int orig_size = xed_decoded_inst_get_length (&xedd);

	// modify rip displacement. use direct addressing mode:	
	new_disp = instr_map[instr_map_entry].orig_ins_addr + disp + orig_size; // xed_decoded_inst_get_length (&xedd_orig);
	xed_encoder_request_set_base0 (&xedd, XED_REG_INVALID);

	//Set the memory displacement using a bit length 
	xed_encoder_request_set_memory_displacement (&xedd, new_disp, new_disp_byts);

	unsigned int size = XED_MAX_INSTRUCTION_BYTES;
	unsigned int new_size = 0;
			
	// Converts the decoder request to a valid encoder request:
	xed_encoder_request_init_from_decode (&xedd);
	
	xed_error_enum_t xed_error = xed_encode (&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), size , &new_size); // &instr_map[i].size
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
		dump_instr_map_entry(instr_map_entry); 
		return -1;
	}				

	if (KnobVerbose) {
		dump_instr_map_entry(instr_map_entry);
	}

	return new_size;
}


/************************************/
/* fix_direct_br_call_to_orig_addr */
/************************************/
int fix_direct_br_call_to_orig_addr(int instr_map_entry)
{
	//bla
	xed_decoded_inst_t xedd;
	xed_decoded_inst_zero_set_mode(&xedd,&dstate); 
				   
	xed_error_enum_t xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
	if (xed_code != XED_ERROR_NONE) {
		cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << instr_map[instr_map_entry].new_ins_addr << endl;
		return -1;
	}
	
	xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);

	if (category_enum != XED_CATEGORY_CALL && category_enum != XED_CATEGORY_UNCOND_BR) {
		cerr << "ERROR: Invalid direct jump from translated code to original code, " 
			  << " instruction_entry: "<< instr_map_entry
			  << " original address: "<<instr_map[instr_map_entry].orig_ins_addr
			  << " original target address: "<<instr_map[instr_map_entry].orig_targ_addr
			 
			  <<endl;

			  
		cerr<< "the instruction map is: " << endl;
		
		for(int i = 0; i<num_of_instr_map_entries;i++){
			const char* category_name = xed_category_enum_t2str(instr_map[i].category_enum);

			cerr << " entry_num: " << i 
				 <<std::hex <<	" orig_ins_addr: " << instr_map[i].orig_ins_addr 
				 << " new_ins_addr: "  << instr_map[i].new_ins_addr
				 << " hasNewTargAddr: "<< instr_map[i].hasNewTargAddr
				 << " orig_targ_addr: "<< instr_map[i].orig_targ_addr
				  << " category: " << category_name
				 <<endl;
		}
		
		//dump_instr_map_entry(instr_map_entry);
		return -1;
				cout << "l2" << endl;

		/*
			instr_map[num_of_instr_map_entries].orig_ins_addr = pc;
	instr_map[num_of_instr_map_entries].new_ins_addr = (ADDRINT)&tc[tc_cursor];  // set an initial estimated addr in tc
	instr_map[num_of_instr_map_entries].orig_targ_addr = orig_targ_addr; 
    instr_map[num_of_instr_map_entries].hasNewTargAddr = false;
	instr_map[num_of_instr_map_entries].targ_map_entry = -1;
	instr_map[num_of_instr_map_entries].size = new_size;	
    instr_map[num_of_instr_map_entries].category_enum = xed_decoded_inst_get_category(xedd);
	instr_map[num_of_instr_map_entries].inline_index = inline_counter;//OUR ADDITION
		*/
	}

	// check for cases of direct jumps/calls back to the orginal target address:
	if (instr_map[instr_map_entry].targ_map_entry >= 0) {
		cerr << "ERROR: Invalid jump or call instruction" << endl;
		return -1;
	}

	unsigned int ilen = XED_MAX_INSTRUCTION_BYTES;
	unsigned int olen = 0;
				

	xed_encoder_instruction_t  enc_instr;

	ADDRINT new_disp = (ADDRINT)&instr_map[instr_map_entry].orig_targ_addr - 
		               instr_map[instr_map_entry].new_ins_addr - 
					   xed_decoded_inst_get_length (&xedd);

	if (category_enum == XED_CATEGORY_CALL)
			xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_CALL_NEAR, 64,
			xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));

	if (category_enum == XED_CATEGORY_UNCOND_BR)
			xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_JMP, 64,
			xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));

	xed_encoder_request_t enc_req;

	xed_encoder_request_zero_set_mode(&enc_req, &dstate);
	xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
	if (!convert_ok) {
		cerr << "conversion to encode request failed" << endl;
		return -1;
	}
   

	xed_error_enum_t xed_error = xed_encode(&enc_req, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), ilen, &olen);
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
	    dump_instr_map_entry(instr_map_entry); 
        return -1;
    }
	// handle the case where the original instr size is different from new encoded instr:
	if (olen != xed_decoded_inst_get_length (&xedd)) {
		
		new_disp = (ADDRINT)&instr_map[instr_map_entry].orig_targ_addr - 
	               instr_map[instr_map_entry].new_ins_addr - olen;

		if (category_enum == XED_CATEGORY_CALL)
			xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_CALL_NEAR, 64,
			xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));

		if (category_enum == XED_CATEGORY_UNCOND_BR)
			xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_JMP, 64,
			xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));


		xed_encoder_request_zero_set_mode(&enc_req, &dstate);
		xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
		if (!convert_ok) {
			cerr << "conversion to encode request failed" << endl;
			return -1;
		}

		xed_error = xed_encode (&enc_req, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), ilen , &olen);
		if (xed_error != XED_ERROR_NONE) {
			cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
			dump_instr_map_entry(instr_map_entry);
			return -1;
		}		
	}
	
	// debug prints:
	if (KnobVerbose) {
		dump_instr_map_entry(instr_map_entry); 
	}
		
	instr_map[instr_map_entry].hasNewTargAddr = true;
	return olen;	
}


/***********************************/
/* fix_direct_br_call_displacement */
/***********************************/
int fix_direct_br_call_displacement(int instr_map_entry) 
{					
	xed_decoded_inst_t xedd;
	xed_decoded_inst_zero_set_mode(&xedd,&dstate); 
				   
	xed_error_enum_t xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
	if (xed_code != XED_ERROR_NONE) {
		cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << instr_map[instr_map_entry].new_ins_addr << endl;
		return -1;
	}

	xed_int32_t  new_disp = 0;	
	unsigned int size = XED_MAX_INSTRUCTION_BYTES;
	unsigned int new_size = 0;


	xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
	
	if (category_enum != XED_CATEGORY_CALL && category_enum != XED_CATEGORY_COND_BR && category_enum != XED_CATEGORY_UNCOND_BR) {
		cerr << "ERROR: unrecognized branch displacement" << endl;
		return -1;
	}

	// fix branches/calls to original targ addresses:
	if (instr_map[instr_map_entry].targ_map_entry < 0) {
	   int rc = fix_direct_br_call_to_orig_addr(instr_map_entry);
	   return rc;
	}

	ADDRINT new_targ_addr;		
	new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;
		
	new_disp = (new_targ_addr - instr_map[instr_map_entry].new_ins_addr) - instr_map[instr_map_entry].size; // orig_size;

	xed_uint_t   new_disp_byts = 4; // num_of_bytes(new_disp);  ???

	// the max displacement size of loop instructions is 1 byte:
	xed_iclass_enum_t iclass_enum = xed_decoded_inst_get_iclass(&xedd);
	if (iclass_enum == XED_ICLASS_LOOP ||  iclass_enum == XED_ICLASS_LOOPE || iclass_enum == XED_ICLASS_LOOPNE) {
	  new_disp_byts = 1;
	}

	// the max displacement size of jecxz instructions is ???:
	xed_iform_enum_t iform_enum = xed_decoded_inst_get_iform_enum (&xedd);
	if (iform_enum == XED_IFORM_JRCXZ_RELBRb){
	  new_disp_byts = 1;
	}

	// Converts the decoder request to a valid encoder request:
	xed_encoder_request_init_from_decode (&xedd);

	//Set the branch displacement:
	xed_encoder_request_set_branch_displacement (&xedd, new_disp, new_disp_byts);

	xed_uint8_t enc_buf[XED_MAX_INSTRUCTION_BYTES];
	unsigned int max_size = XED_MAX_INSTRUCTION_BYTES;

	xed_error_enum_t xed_error = xed_encode (&xedd, enc_buf, max_size , &new_size);
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) <<  endl;
		char buf[2048];		
		xed_format_context(XED_SYNTAX_INTEL, &xedd, buf, 2048, static_cast<UINT64>(instr_map[instr_map_entry].orig_ins_addr), 0, 0);
	    cerr << " instr: " << "0x" << hex << instr_map[instr_map_entry].orig_ins_addr << " : " << buf <<  endl;
  		return -1;
	}		

	new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;

	new_disp = new_targ_addr - (instr_map[instr_map_entry].new_ins_addr + new_size);  // this is the correct displacemnet.

	//Set the branch displacement:
	xed_encoder_request_set_branch_displacement (&xedd, new_disp, new_disp_byts);

	xed_error = xed_encode (&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), size , &new_size); // &instr_map[i].size
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
		dump_instr_map_entry(instr_map_entry);
		return -1;
	}				

	//debug print of new instruction in tc:
	if (KnobVerbose) {
		dump_instr_map_entry(instr_map_entry);
	}

	return new_size;
}				


/************************************/
/* fix_instructions_displacements() */
/************************************/
int fix_instructions_displacements()
{
   // fix displacemnets of direct branch or call instructions:

    int size_diff = 0;	

	do {
		
		size_diff = 0;

		if (KnobVerbose) {
			cerr << "starting a pass of fixing instructions displacements: " << endl;
		}

		for (int i=0; i < num_of_instr_map_entries; i++) {

			instr_map[i].new_ins_addr += size_diff;
				   
			int new_size = 0;

			// fix rip displacement:	
			new_size = fix_rip_displacement(i);
			if (new_size < 0)
				return -1;

			if (new_size > 0) { // this was a rip-based instruction which was fixed.

				if (instr_map[i].size != (unsigned int)new_size) {
				   size_diff += (new_size - instr_map[i].size); 					
				   instr_map[i].size = (unsigned int)new_size;								
				}

				continue;   
			}

			// check if it is a direct branch or a direct call instr:
			if (instr_map[i].orig_targ_addr == 0) {
				continue;  // not a direct branch or a direct call instr.
			}


			// fix instr displacement:		
			new_size = fix_direct_br_call_displacement(i);
			if (new_size < 0)
				return -1;
			if (instr_map[i].size != (unsigned int)new_size) {
			   size_diff += (new_size - instr_map[i].size);
			   instr_map[i].size = (unsigned int)new_size;
			}
		}  // end int i=0; i ..

	} while (size_diff != 0);

   return 0;
 }


/*****************************************/
/* find_candidate_rtns_for_translation() */
/*****************************************/
int find_candidate_rtns_for_translation(IMG img)
{
    map<ADDRINT, xed_decoded_inst_t> local_instrs_map;
    local_instrs_map.clear();
	map<ADDRINT, xed_decoded_inst_t> rtn_local_instrs_map;
	map<ADDRINT, xed_decoded_inst_t> global_xedd_map;
	ADDRINT jmp_counter=1;
    // go over routines and check if they are candidates for translation and mark them for translation:
    for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {   
        if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec))
            continue;
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {    
            if (rtn == RTN_Invalid()) {
              cerr << "Warning: invalid routine " << RTN_Name(rtn) << endl;
                continue;
            }
			//  our check - if rtn not in profile, skip it
			if(rtn_map.find(RTN_Address(rtn))==rtn_map.end() ){
				cerr << "Warning: skipping routine " <<std::hex << RTN_Address(rtn) << endl;
				continue;
			}
			if(rtn_map[RTN_Address(rtn)]->rtn_bbls_map.empty()==true){
				continue;
			}
			if((((RTN_Name(rtn)=="_real_fini") || (RTN_Name(rtn)=="_fini"))) ){ //RTN_Name(rtn)!= "hasSuffix" && 
				continue;
			}
		// PRINT RTN's NAME
		//		cout << "RTN_NAME: " << RTN_Name(rtn)<<endl;
			//	cout << "Address: " << std::hex<<RTN_Address(rtn)<<endl;
				//cout << "address from map: " << rtn_map[RTN_Address(rtn)]->routine_address << endl;
				//cout << "ins_num from map: " << rtn_map[RTN_Address(rtn)]->instructions_num << endl;



            // Open the RTN.
            RTN_Open( rtn ); 
			
			// fill a local map for the current rtn. later it will be reordered and inserted into the instruction map
				rtn_local_instrs_map.clear();
				for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins)) {
					
					ADDRINT addr = INS_Address(ins);
					//debug print of orig instruction:
					if (KnobVerbose) {
						cerr << "1:old instr: ";
						cerr << "0x" << hex << INS_Address(ins) << ": " << INS_Disassemble(ins) <<  endl;
						//xed_print_hex_line(reinterpret_cast<UINT8*>(INS_Address (ins)), INS_Size(ins));                               
					}        
					
					xed_decoded_inst_t xedd;
					xed_error_enum_t xed_code;                            
					
					xed_decoded_inst_zero_set_mode(&xedd,&dstate); 
					xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(addr), max_inst_len);
					if (xed_code != XED_ERROR_NONE) {
						cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << addr << endl;
						break;
					}
					
					// Save xed and addr into a map to be used later.
					rtn_local_instrs_map[addr] = xedd;
					
				} // end for INS...
				map<ADDRINT, xed_decoded_inst_t> test_map=rtn_local_instrs_map;
				
				 //---------------DELETE THIS-----------------------------------------------------
/*
					for (const auto& pair : rtn_local_instrs_map) {
						ADDRINT addr = pair.first;
						xed_decoded_inst_t xedd = pair.second;           

					          //debug print of orig instruction:
					   if (KnobVerbose) {
						 char disasm_buf[2048];
						 xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(addr), 0, 0);               
						 cerr << "0x" << hex << addr << ": " << disasm_buf  <<  endl; 
					   }
					   //insert 
					  // add_new_instr_entry_to_tmp_map(&xedd, addr, xed_decoded_inst_get_length(&xedd),0,0);
					   	tmp_instr_map[num_of_instr_map_entries_tmp].orig_ins_addr = addr;
						tmp_instr_map[num_of_instr_map_entries_tmp].orig_targ_addr = 0;
						num_of_instr_map_entries_tmp++;
						test_map.erase(addr);
						global_xedd_map[addr] = xedd;
					}
			*/	
				//--------------REORDERING-----------------------------------------------------------------------------------	
				map<ADDRINT,bbl_st> tmp_rtn_bbls_map = rtn_map[RTN_Address(rtn)]->rtn_bbls_map;

				INS first_ins = RTN_InsHead(rtn);		
				bbl_st curr_bbl=tmp_rtn_bbls_map[INS_Address(first_ins)];
				bbl_st next_bbl;
				ADDRINT head_address=curr_bbl.head_address;
				ADDRINT tail_address=curr_bbl.tail_address;
				int rtn_first_entry=num_of_instr_map_entries_tmp;
				int rtn_last_entry=0;
				
				bool  changed_FT=false;
				while(!tmp_rtn_bbls_map.empty()){
					ADDRINT original_target_of_tail=0;
					auto start_it=rtn_local_instrs_map.find(head_address);
					auto end_it=rtn_local_instrs_map.find(tail_address);
					//insert all instruction of the BLL to the instruction map (without the tail)
					for (auto it = start_it; it != end_it; ++it) {
						ADDRINT addr = it->first;
						xed_decoded_inst_t xedd = it->second;           

					          //debug print of orig instruction:
					   if (KnobVerbose) {
						 char disasm_buf[2048];
						 xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(addr), 0, 0);               
						 cerr << "0x" << hex << addr << ": " << disasm_buf  <<  endl; 
					   }
					   //insert 
					  // add_new_instr_entry_to_tmp_map(&xedd, addr, xed_decoded_inst_get_length(&xedd),0,0);
					   	tmp_instr_map[num_of_instr_map_entries_tmp].orig_ins_addr = addr;
						tmp_instr_map[num_of_instr_map_entries_tmp].orig_targ_addr = 0;
						num_of_instr_map_entries_tmp++;
						test_map.erase(addr);
						global_xedd_map[addr] = xedd;
					}
					
					tmp_rtn_bbls_map.erase(head_address); //remove the BBL and find the next one
					//info for entering the tail into the global map (might be changed if reordering is needed)
					ADDRINT addr =tail_address ; 
					xed_decoded_inst_t xedd = rtn_local_instrs_map[tail_address];
					
					//add either the original or an inverted tail depending of taken/not_taken profile
					if(curr_bbl.FT_target!=0 && curr_bbl.target!=0){//this is a branch and we need to choose the next bbl depending on taken/not taken frequency
						if(curr_bbl.taken_count<curr_bbl.not_taken_count){		//insert the regular jump
							if(tmp_rtn_bbls_map.find(curr_bbl.next_ins)!=tmp_rtn_bbls_map.end()){//check if the new FT target was already added
								next_bbl=tmp_rtn_bbls_map[curr_bbl.next_ins];// declare next bbl to be the original FT
							}
							else{ // if the wanted FT was already inserted, just take another unrelated bbl from the map.
								next_bbl=tmp_rtn_bbls_map.begin()->second;
								changed_FT=true;
								//FUTURE OPTIMIZATION: TRY TO USE THE JUMP TARGET BBL AS THE NEXT BBL 
							}
							
						}
						else{ //REORDER
						
							//insert an INVERTED jump 
							
							if(tmp_rtn_bbls_map.find(curr_bbl.target)!=tmp_rtn_bbls_map.end()){//check if the new FT target was not already added
								next_bbl=tmp_rtn_bbls_map[curr_bbl.target];// declare next bbl to be the jump target FT
								invert_jmp(&xedd);// OUR FUNCTION
								original_target_of_tail=curr_bbl.next_ins;
					//			cerr << "REORDERING!! head address is:  "<<std::hex<< head_address << " tail address is: " << tail_address<< endl;
								//rtn_map[RTN_Address(rtn)]->rtn_bbls_map[curr_bbl.target].was_reordered=1;// this symbolizes that the BBL is not in it's original place and might need a jmp at the end to it;s FT
							}
							else{
								next_bbl=tmp_rtn_bbls_map.begin()->second;
								changed_FT=true;
								}
							

							
						}
					}
					else {
						if(RTN_FindNameByAddress(tail_address)==RTN_FindNameByAddress(curr_bbl.next_ins)){//check if we are the the last BBL of the routine
							//if the next instruction is in the same routine, we want to take the next BBL
							if(tmp_rtn_bbls_map.find(curr_bbl.next_ins)!=tmp_rtn_bbls_map.end()){//check if the new FT target was already added
								next_bbl=tmp_rtn_bbls_map[curr_bbl.next_ins];// declare next bbl to be the original FT
							}
							else{
								next_bbl=tmp_rtn_bbls_map.begin()->second;//if the wanted FT was already inserted, just take another unrelated bbl from the map.
								changed_FT=true;
							}
						}
						else{
							//if the next instruction is NOT in the same routine, we want to take the first BBL from the top
							
							next_bbl=tmp_rtn_bbls_map.begin()->second;
							changed_FT=true;
						}
					
					
					}
					//insert the tail to the map
				   if (KnobVerbose) {
					 char disasm_buf[2048];
					 xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(addr), 0, 0);               
					 cerr << "0x" << hex << addr << ": " << disasm_buf  <<  endl; 
				   }
				   //insert the inverted branch inst:
					//add_new_instr_entry_to_tmp_map(&xedd, addr, xed_decoded_inst_get_length(&xedd),0,original_target_of_tail);
					tmp_instr_map[num_of_instr_map_entries_tmp].orig_ins_addr = addr;
					tmp_instr_map[num_of_instr_map_entries_tmp].orig_targ_addr = original_target_of_tail;
					num_of_instr_map_entries_tmp++;
					test_map.erase(addr);
					global_xedd_map[addr] = xedd;
					// check if the next BBL is the original FT. if not, add a jmp instruction to the original FT 
					if(curr_bbl.next_ins != next_bbl.head_address && changed_FT){
					
						add_final_jmp(&xedd); // OUR FUNCTION
						
						if (KnobVerbose) {
						 char disasm_buf[2048];
						 xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(addr), 0, 0);               
						 cerr << "0x" << hex << addr << ": " << disasm_buf  <<  endl; 
					   }
					   //insert the jmp inst:
						//add_new_instr_entry_to_tmp_map(&xedd, 0, xed_decoded_inst_get_length(&xedd),0,curr_bbl.next_ins);
						tmp_instr_map[num_of_instr_map_entries_tmp].orig_ins_addr = jmp_counter;
						tmp_instr_map[num_of_instr_map_entries_tmp].orig_targ_addr = curr_bbl.next_ins;
						global_xedd_map[jmp_counter] = xedd;
						jmp_counter++;
						num_of_instr_map_entries_tmp++;

					}
					// if curr BBL was reordered or the FT was, we need to add a a jump instruction	
					//CHOOSE THE NEXT BBL 
					curr_bbl=next_bbl;
					first_ins = RTN_InsHead(rtn);			
					head_address=curr_bbl.head_address;
					tail_address=curr_bbl.tail_address;
					changed_FT=false;
				}
				
				//PRINT test_map to see if there are any instructions left at the end
				// and insert them into the entry map
				//cerr << "THESE ARE THE ADDRESSES IN THE LOCAL RTN_MAP after the insertion FOR RTN:  " << RTN_Name(rtn)<< endl;
				for(const auto& pair : test_map) {
					//cerr << std::hex<<pair.first << endl;
					ADDRINT addr = pair.first;
					xed_decoded_inst_t xedd = pair.second; 
					if (KnobVerbose) {
					 char disasm_buf[2048];
					 xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(addr), 0, 0);               
					 cerr << "0x" << hex << addr << ": " << disasm_buf  <<  endl; 
				   }
				   // add the last instructions
					//add_new_instr_entry_to_tmp_map(&xedd, addr, xed_decoded_inst_get_length(&xedd),0,0);
					tmp_instr_map[num_of_instr_map_entries_tmp].orig_ins_addr = addr;
					tmp_instr_map[num_of_instr_map_entries_tmp].orig_targ_addr = 0;
					num_of_instr_map_entries_tmp++;
					global_xedd_map[addr] = xedd;
				}
				rtn_last_entry=num_of_instr_map_entries_tmp;
				tmp_rtn_map[INS_Address(first_ins)]=make_pair(rtn_first_entry,rtn_last_entry);
				//cout<< " hi " << endl;
				
		
            // Close the RTN.
            RTN_Close( rtn );
        } // end for RTN..
    } // end for SEC...


// add insts from tmp map to real map with inlining


//eyal
//


	ADDRINT hot_call=hot_calls_vector[1];
	//cout << "HOT CALL: " <<std::hex << hot_call << endl;
	for(int i=0;i<num_of_instr_map_entries_tmp;i++){
		int rc=0;
		ADDRINT curr_orig_ins_addr=tmp_instr_map[i].orig_ins_addr;
		ADDRINT curr_orig_targ_addr=tmp_instr_map[i].orig_targ_addr;
		xed_decoded_inst_t xedd=global_xedd_map[curr_orig_ins_addr];
		RTN rtn = RTN_FindByAddress(curr_orig_ins_addr);

		//update translated_rtn_map:
		if(RTN_Valid(rtn)){
			RTN_Open(rtn);
			if(curr_orig_ins_addr==INS_Address(RTN_InsHead(rtn)) ){
//cout << RTN_Name(rtn) << endl;

				translated_rtn[translated_rtn_num].rtn_addr = RTN_Address(rtn);			
				translated_rtn[translated_rtn_num].rtn_size = RTN_Size(rtn);
				translated_rtn[translated_rtn_num].instr_map_entry = num_of_instr_map_entries;
				translated_rtn[translated_rtn_num].isSafeForReplacedProbe = true;	
				translated_rtn_num++;

			}
		RTN_Close(rtn);
		}
		//cout<<"curr_address: " << curr_orig_ins_addr << endl;
		if(curr_orig_ins_addr!=hot_call)// check if the instruction is not a hot call that should be inlined 
		//if (find(hot_calls_vector.begin(), hot_calls_vector.end(), curr_orig_ins_addr) == hot_calls_vector.end())
		{
		// in case that it is not a hot call, just insert the instruction:

			rc = add_new_instr_entry(&xedd, curr_orig_ins_addr, xed_decoded_inst_get_length(&xedd),0,curr_orig_targ_addr);
			if (rc < 0) {
				cerr << "ERROR: failed during instructon translation." << endl;
				translated_rtn[translated_rtn_num].instr_map_entry = -1;
				break;
			}

		}
		else{ // THIS IS THE INLINING PART:
			xed_int64_t disp = xed_decoded_inst_get_branch_displacement(&xedd);
			ADDRINT target_addr = curr_orig_ins_addr + xed_decoded_inst_get_length (&xedd) + disp;// a call's target address should be a head of a routine
			//std::cout <<std::hex<< target_addr << " " << endl;
			
			if(tmp_rtn_map.find(target_addr)==tmp_rtn_map.end()){// do not inline if we can't find the target routine of this call
				rc = add_new_instr_entry(&xedd, curr_orig_ins_addr, xed_decoded_inst_get_length(&xedd),0,curr_orig_targ_addr);
				if (rc < 0) {
					cerr << "ERROR: failed during instructon translation." << endl;
					translated_rtn[translated_rtn_num].instr_map_entry = -1;
					break;
				}
				continue;			
			}
			
			inline_counter++;
			int first_entry=tmp_rtn_map[target_addr].first;
			int last_entry=tmp_rtn_map[target_addr].second;
			ADDRINT after_call_addr=tmp_instr_map[i+1].orig_ins_addr;
			
			for(int j=first_entry;j<=last_entry;j++){ // all the instructions inside the inlined rtn
				curr_orig_ins_addr=tmp_instr_map[j].orig_ins_addr;
				curr_orig_targ_addr=tmp_instr_map[j].orig_targ_addr;
				xedd=global_xedd_map[curr_orig_ins_addr];
				ADDRINT inline_targ_addr = 0;
				xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
				xed_uint_t disp_byts = xed_decoded_inst_get_branch_displacement_width(&xedd);//check if the jump target is inside the routine or not
				xed_int32_t disp;
				if(category_enum==XED_CATEGORY_RET){ // if the inst is ret, make it a jump to the next instruction after the original call

					add_final_jmp(&xedd);
					rc = add_new_instr_entry(&xedd, curr_orig_ins_addr, xed_decoded_inst_get_length(&xedd),0,after_call_addr);
					if (rc < 0) {
						cerr << "ERROR: failed during instructon translation." << endl;
						translated_rtn[translated_rtn_num].instr_map_entry = -1;
						break;
					}
					continue;
				}
				else if (disp_byts > 0) {

					//check if the instruction is is a branch
					disp = xed_decoded_inst_get_branch_displacement(&xedd);
					inline_targ_addr = curr_orig_ins_addr + xed_decoded_inst_get_length (&xedd) + disp;
					if(RTN_FindByAddress(inline_targ_addr) == RTN_FindByAddress(curr_orig_ins_addr)){
						// check if the target of a jmp is inside the rtn, if it is insert the instruction with inline counter!=0 so that it jumps inside the inlined rtn
						rc = add_new_instr_entry(&xedd, curr_orig_ins_addr, xed_decoded_inst_get_length(&xedd),inline_counter,curr_orig_targ_addr);
						if (rc < 0) {
							cerr << "ERROR: failed during instructon translation." << endl;
							translated_rtn[translated_rtn_num].instr_map_entry = -1;
							break;
						}
					}
					else{ // if the jmp target is outside of the inlined routine, giive it inline_counter=0 so that it jumps as it used to in the original code
						rc = add_new_instr_entry(&xedd, curr_orig_ins_addr, xed_decoded_inst_get_length(&xedd),0,curr_orig_targ_addr);
						if (rc < 0) {
							cerr << "ERROR: failed during instructon translation." << endl;
							translated_rtn[translated_rtn_num].instr_map_entry = -1;
							break;
						}
					}
				}
				else{
					rc = add_new_instr_entry(&xedd, curr_orig_ins_addr, xed_decoded_inst_get_length(&xedd),inline_counter,curr_orig_targ_addr);

					if (rc < 0) {
						cerr << "ERROR: failed during instructon translation." << endl;
						translated_rtn[translated_rtn_num].instr_map_entry = -1;

						break;
					}
				}
				//cout << "last entry1: " <<  num_of_instr_map_entries << endl;	
			}
			//cout << "last entry2: " <<  num_of_instr_map_entries << endl;	

		}	
	}
	



    return 0;
}
/***************************/
/* int copy_instrs_to_tc() */
/***************************/
int copy_instrs_to_tc()
{
	int cursor = 0;

	for (int i=0; i < num_of_instr_map_entries; i++) {

	  if ((ADDRINT)&tc[cursor] != instr_map[i].new_ins_addr) {
		  cerr << "ERROR: Non-matching instruction addresses: " << hex << (ADDRINT)&tc[cursor] << " vs. " << instr_map[i].new_ins_addr << endl;
	      return -1;
	  }	  

	  memcpy(&tc[cursor], &instr_map[i].encoded_ins, instr_map[i].size);

	  cursor += instr_map[i].size;
	}

	return 0;
}


/*************************************/
/* void commit_translated_routines() */
/*************************************/
inline void commit_translated_routines() 
{
	// Commit the translated functions: 
	// Go over the candidate functions and replace the original ones by their new successfully translated ones:

	for (int i=0; i < translated_rtn_num; i++) {

		//replace function by new function in tc
	
		if (translated_rtn[i].instr_map_entry >= 0) {
				    
			if (translated_rtn[i].rtn_size > MAX_PROBE_JUMP_INSTR_BYTES && translated_rtn[i].isSafeForReplacedProbe) {						

				RTN rtn = RTN_FindByAddress(translated_rtn[i].rtn_addr);

				//debug print:				
				if (rtn == RTN_Invalid()) {
					cerr << "committing rtN: Unknown";
				} else {
					cerr << "committing rtN: " << RTN_Name(rtn);
				}
				cerr << " from: 0x" << hex << RTN_Address(rtn) << " to: 0x" << hex << instr_map[translated_rtn[i].instr_map_entry].new_ins_addr << endl;

						
				if (RTN_IsSafeForProbedReplacement(rtn)) {

					AFUNPTR origFptr = RTN_ReplaceProbed(rtn,  (AFUNPTR)instr_map[translated_rtn[i].instr_map_entry].new_ins_addr);							

					if (origFptr == NULL) {
						cerr << "RTN_ReplaceProbed failed.";
					} else {
						cerr << "RTN_ReplaceProbed succeeded. ";
					}
					cerr << " orig routine addr: 0x" << hex << translated_rtn[i].rtn_addr
							<< " replacement routine addr: 0x" << hex << instr_map[translated_rtn[i].instr_map_entry].new_ins_addr << endl;	

					dump_instr_from_mem ((ADDRINT *)translated_rtn[i].rtn_addr, translated_rtn[i].rtn_addr);												
				}												
			}
		}
	}
}


/****************************/
/* allocate_and_init_memory */
/****************************/ 
int allocate_and_init_memory(IMG img) 
{
	// Calculate size of executable sections and allocate required memory:
	//
	for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {   
		if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec))
			continue;


		if (!lowest_sec_addr || lowest_sec_addr > SEC_Address(sec))
			lowest_sec_addr = SEC_Address(sec);

		if (highest_sec_addr < SEC_Address(sec) + SEC_Size(sec))
			highest_sec_addr = SEC_Address(sec) + SEC_Size(sec);

		// need to avouid using RTN_Open as it is expensive...
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {		

			if (rtn == RTN_Invalid())
				continue;

			max_ins_count += RTN_NumIns  (rtn);
			max_rtn_count++;
		}
	}

	max_ins_count *= 4; // estimating that the num of instrs of the inlined functions will not exceed the total nunmber of the entire code.
	
	// Allocate memory for the instr map needed to fix all branch targets in translated routines:
	instr_map = (instr_map_t *)calloc(max_ins_count, sizeof(instr_map_t));
	if (instr_map == NULL) {
		perror("calloc");
		return -1;
	}
	//tmp_instr_map_t *tmp_instr_map = NULL;

	tmp_instr_map = (tmp_instr_map_t *)calloc(max_ins_count, sizeof(tmp_instr_map_t));
	if (tmp_instr_map == NULL) {
		perror("calloc");
		return -1;
	}


	// Allocate memory for the array of candidate routines containing inlineable function calls:
	// Need to estimate size of inlined routines.. ???
	translated_rtn = (translated_rtn_t *)calloc(max_rtn_count, sizeof(translated_rtn_t));
	if (translated_rtn == NULL) {
		perror("calloc");
		return -1;
	}


	// get a page size in the system:
	int pagesize = sysconf(_SC_PAGE_SIZE);
    if (pagesize == -1) {
      perror("sysconf");
	  return -1;
	}

	ADDRINT text_size = (highest_sec_addr - lowest_sec_addr) * 2 + pagesize * 4;

    int tclen = 2 * text_size + pagesize * 4;   // need a better estimate???

	// Allocate the needed tc with RW+EXEC permissions and is not located in an address that is more than 32bits afar:		
	char * addr = (char *) mmap(NULL, tclen, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
	if ((ADDRINT) addr == 0xffffffffffffffff) {
		cerr << "failed to allocate tc" << endl;
        return -1;
	}
	
	tc = (char *)addr;
	return 0;
}



/* ============================================ */
/* Main translation routine                     */
/* ============================================ */
VOID ImageLoad(IMG img, VOID *v)
{
	// debug print of all images' instructions
	//dump_all_image_instrs(img);


    // Step 0: Check the image and the CPU:
	if (!IMG_IsMainExecutable(img))
		return;

	int rc = 0;

	// step 1: Check size of executable sections and allocate required memory:	
	rc = allocate_and_init_memory(img);
	if (rc < 0)
		return;

	cout << "after memory allocation" << endl;

	// step 1.5: read the data from the profiling phase (count.bin)
	std::string filePath = "counts.bin";
	readBinary(filePath);
	
	// Step 2: go over all routines and identify candidate routines and copy their code into the instr map IR:
	rc = find_candidate_rtns_for_translation(img);
	if (rc < 0)
		return;

	cout << "after identifying candidate routines" << endl;	 
	
	// Step 3: Chaining - calculate direct branch and call instructions to point to corresponding target instr entries:
	rc = chain_all_direct_br_and_call_target_entries();
	if (rc < 0 )
		return;
	
	cout << "after calculate direct br targets" << endl;

	// Step 4: fix rip-based, direct branch and direct call displacements:
	rc = fix_instructions_displacements();
	if (rc < 0 )
		return;
	
	cout << "after fix instructions displacements" << endl;


	// Step 5: write translated routines to new tc:
	rc = copy_instrs_to_tc();
	if (rc < 0 )
		return;

	cout << "after write all new instructions to memory tc" << endl;

   if (KnobDumpTranslatedCode) {
	   cerr << "Translation Cache dump:" << endl;
       dump_tc();  // dump the entire tc

	 //  cerr << endl << "instructions map dump:" << endl;
	   dump_entire_instr_map();     // dump all translated instructions in map_instr
   }


	// Step 6: Commit the translated routines:
	//Go over the candidate functions and replace the original ones by their new successfully translated ones:
    if (!KnobDoNotCommitTranslatedCode) {
	  commit_translated_routines();	
	  cout << "after commit translated routines" << endl;
    }
	////////////////////////////////////////////////////////////////////////////////////////
	//print testing stuff here
}



/* ===================================================================== */
/* Print Help Message                                                    */
/* ===================================================================== */
INT32 Usage()
{
    cerr << "This tool translated routines of an Intel(R) 64 binary"
         << endl;
    cerr << KNOB_BASE::StringKnobSummary();
    cerr << endl;
    return -1;
}

/* ===================================================================== */

VOID docount(unsigned int *counter_pointer) { (*counter_pointer)++; }
VOID call_counter (ADDRINT call_addr,ADDRINT targ_address){ call_vector.push_back(make_pair(call_addr,targ_address));}
VOID Taken_count(unsigned int *taken_count , unsigned int *not_taken_count ,INT32 is_taken) { 
	if(is_taken){
		(*taken_count)++;
	}
	else{
		(*not_taken_count)++;
	}
 }
/* ===================================================================== */

/* ===================================================================== */

VOID ROUTINE(RTN my_rtn, VOID* v) { 
	
	
		
	rtn_st* rtn = new rtn_st;
	//give each routine struct the names and address of the routine and of the image
	ADDRINT my_addr=RTN_Address(my_rtn); // this is the current rtn's address
	IMG  img = IMG_FindByAddress(my_addr);// the image of the rtn can be found from the rtn's address
	rtn->routine_address = my_addr;//rtn's address
	
	rtn->instructions_num = 0;
	rtn->calls_num = 0;
		
	//check that thr routine is valid and that it is in the main executable image
	if (RTN_Valid(my_rtn) && IMG_IsMainExecutable(img)) {
		RTN_Open(my_rtn);
		//count the amount of calls to this routine
		RTN_InsertCall(my_rtn, IPOINT_BEFORE, (AFUNPTR)docount,IARG_PTR, &(rtn->calls_num), IARG_END);
		//count the amount of instructions in this routine
		for (INS ins =RTN_InsHead(my_rtn); INS_Valid(ins); ins = INS_Next(ins)) {
			INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)docount,IARG_PTR, &(rtn->instructions_num), IARG_END);	
			if(INS_IsCall(ins) && INS_IsDirectControlFlow(ins)){
				//ADDRINT targ_addr=INS_DirectControlFlowTargetAddress(ins);
				INS_InsertCall(ins,IPOINT_BEFORE,AFUNPTR(call_counter)
				,IARG_ADDRINT,INS_Address(ins)
				,IARG_BRANCH_TARGET_ADDR
				,IARG_END);
			}
		}
	
	//add the routine to the map
	rtn_map[my_addr] = rtn;
	RTN_Close(my_rtn);
	}		
}
/* ===================================================================== */

/* ===================================================================== */

VOID TRACE_INSTRUMENTATION(TRACE trace, VOID *v) {
    for (BBL bbl = TRACE_BblHead(trace); BBL_Valid(bbl); bbl = BBL_Next(bbl)) {
		
		RTN rtn = INS_Rtn(BBL_InsHead(bbl));
		if(!RTN_Valid(rtn)){// if rtn is not valid, do not do anything
			continue;
		}
			bbl_st* my_bbl = new bbl_st;
			my_bbl->routine_address=RTN_Address(rtn);
			//my_bbl->routine_name = RTN_Name(rtn);
			
			INS head=BBL_InsHead(bbl);
			my_bbl->head_address=INS_Address(head);

			INS tail =BBL_InsTail(bbl);
			my_bbl->tail_address=INS_Address(tail);
			
			
			//find fall-through target
			if(INS_HasFallThrough(tail)){
				my_bbl->FT_target=INS_NextAddress(tail);
			}
			else{
				my_bbl->FT_target=0;
			}
			
			my_bbl->next_ins = INS_NextAddress(tail);
			my_bbl->was_reordered=false;

			
			
			//find jump target
			if(INS_IsDirectControlFlow(tail)){
				my_bbl->target=INS_DirectControlFlowTargetAddress(tail);
			}
			
			my_bbl->instructions_num=0;
			my_bbl->taken_count=0;
			my_bbl->not_taken_count=0;
			
			// check the heat of the bbl by counting whenever it starts
			INS_InsertCall(head, IPOINT_BEFORE,(AFUNPTR)docount, IARG_PTR, &(my_bbl->instructions_num), IARG_END );
			//count the amount of "taken" jumps 
			INS_InsertCall(tail, IPOINT_BEFORE, (AFUNPTR)Taken_count,IARG_PTR, &(my_bbl->taken_count) , IARG_PTR, &(my_bbl->not_taken_count),IARG_BRANCH_TAKEN, IARG_END);				
			/*
			for (INS ins = BBL_InsHead(bbl); INS_Valid(ins); ins = INS_Next(ins)){
				INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)docount,IARG_PTR, &(my_bbl->instructions_num), IARG_END);				
			}
			*/
			
				bbl_map[INS_Address(BBL_InsHead(bbl))]=my_bbl;	

    }
}



/* ===================================================================== */

/* ===================================================================== */

VOID Fini(INT32 code, VOID* v) { 
	
	
	// count the "call" instruction in each routine
	/*
		for(const auto& pair: call_vector){
		if(rtn_map.find(pair.second)==rtn_map.end()) {continue;}
		else{
			if(rtn_map[pair.second]->calls_map.find(pair.first)==rtn_map[pair.second]->calls_map.end()){
				rtn_map[pair.second]->calls_map[pair.first]=1;
			}
			else{
				rtn_map[pair.second]->calls_map[pair.first]+=1;
			}
		}
	}
	*/
	
	//count the bbls in each routine 
	vector<pair<unsigned int, bbl_st*>> bbl_vec(bbl_map.begin(), bbl_map.end());
// insert the bbl's to their routines 
	unsigned long sum_taken_count=0;
	unsigned long sum_not_taken_count=0;
	
	for (const pair<const unsigned int, bbl_st*>& pair : bbl_vec) {
		const bbl_st& current_bbl = *(pair.second);
		ADDRINT current_rtn_addr = current_bbl.routine_address;		
		if(rtn_map.find(current_rtn_addr)==rtn_map.end()){continue;}//check if routine has been translated
		//insert all the bbls to their respective rtns. in case of 2 bbls with the same tail, take the one that contains the other (smallest head address)
		if(rtn_map[current_rtn_addr]->rtn_bbls_map.find(current_bbl.tail_address)==rtn_map[current_rtn_addr]->rtn_bbls_map.end()){
			rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address]=current_bbl;
		}
		else{
			sum_taken_count		=	rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address].taken_count 		  +	current_bbl.taken_count;
			sum_not_taken_count	=	rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address].not_taken_count + current_bbl.not_taken_count;
			if(rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address].head_address > current_bbl.head_address){
				rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address]=current_bbl;
			}
			rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address].taken_count=sum_taken_count;
			rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address].not_taken_count=sum_not_taken_count;
		}
			
			
	}
	
	/*for (const pair<const unsigned int, bbl_st*>& pair : bbl_vec) {
		const bbl_st& current_bbl = *(pair.second);
		ADDRINT current_rtn_addr = current_bbl.routine_address;		
		if(rtn_map.find(current_rtn_addr)==rtn_map.end()){continue;}//check if routine has been translated
		//insert all the bbls to their respective rtns. in case of 2 bbls with the same tail, take the one that contains the other (smallest head address)
		if(rtn_map[current_rtn_addr]->rtn_bbls_map.find(current_bbl.head_address)==rtn_map[current_rtn_addr]->rtn_bbls_map.end()){
			rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.head_address]=current_bbl;
		}
		else{
			sum_taken_count		=	rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.head_address].taken_count 		  +	current_bbl.taken_count;
			sum_not_taken_count	=	rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.head_address].not_taken_count + current_bbl.not_taken_count;
			if(rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.head_address].head_address > current_bbl.head_address){
				rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.head_address]=current_bbl;
			}
			rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.head_address].taken_count=sum_taken_count;
			rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.head_address].not_taken_count=sum_not_taken_count;
		}
			
			
	}*/

	// change the bbl map to keys with head address instead of tail address:
	
	for(auto& pair : rtn_map){
		map<ADDRINT, bbl_st> bbl_map_key_is_head;
		for(const auto& pair2:pair.second->rtn_bbls_map){
			const bbl_st& bbl=pair2.second;
			bbl_map_key_is_head[bbl.head_address]=bbl;
		}
		pair.second->rtn_bbls_map=std::move(bbl_map_key_is_head);
	}
	
	
		/////////////////////PRINT ROUTINE MAP//////////////////////////////////////////

	// sort the map by ins_count
	// create a vector from the pairs of the map
	vector<pair<unsigned int, rtn_st*>> sorted_vec(rtn_map.begin(), rtn_map.end());
	unsigned long total_inst_sum=0;
	unsigned long total_calls_num=0;
	unsigned long mean_calls=0;
	//use std::sort and my comapre function to sort the vector by instruction_num 
	sort(sorted_vec.begin(), sorted_vec.end(), compare_rtn_by_inst);
	//print the sorted map
	ofstream output("loop-count.csv");
	for (const pair<const unsigned int, rtn_st*>& pair : sorted_vec) {
		const rtn_st& data = *(pair.second); 
		total_inst_sum+=data.instructions_num;
		total_calls_num+=data.calls_num;
		if(data.calls_num != 0) { 

			output
			
			<< std::hex	
			<< "0x" 			   <<data.routine_address <<" , "	
			//<< data.routine_name   << " , " 
			//<<data.image_name      << " , " 
			<< std::hex
			//<< "0x"				   << data.image_address  << " , "			
			<< std::dec		
			<<data.instructions_num<< " , " 	
			<< data.calls_num
			<<std::hex;
// print all the tails of the bbls in the routine:
			for(const auto& pair2: data.calls_map ){
				output << " , " <<  std::hex << "0x" << pair2.first << " , " << std::dec <<pair2.second ;
			}
			output<<endl;
		}
	}
	output.close();

	
	/////////////////FIND HOT CALLS///////////////////////////////////////////////
	vector<ADDRINT> hot_rtns_vec;
	map<pair<ADDRINT,ADDRINT>,unsigned long> call_pairs_count_map;
	unsigned int dominant_threshold=2;
	mean_calls=total_calls_num/sorted_vec.size();
	mean_calls=total_calls_num/sorted_vec.size();
	cout <<"MEAN calls: " << mean_calls << endl;
	sort(sorted_vec.begin(), sorted_vec.end(), compare_rtn_by_calls);
	unsigned int k=0;
	while((sorted_vec[k].second->calls_num 	>	 static_cast<unsigned long>(mean_calls)) && (k<(sorted_vec.size()))){// find the hot routines
		hot_rtns_vec.push_back(sorted_vec[k].first);
		k++;
	}
	cout << "HOT_RTNS_NUM= " << hot_rtns_vec.size() << endl;
	
	for(const auto& pair : call_vector) { //count the amount of calls to a routine from a specific address
	
	call_pairs_count_map[pair]++;
    }
	
	for(const auto& pair : call_pairs_count_map) {
		ADDRINT call_address=pair.first.first;
		ADDRINT targ_addr = pair.first.second;
		unsigned long curr_call_count=pair.second;
		if(rtn_map.find(targ_addr)==rtn_map.end()){continue;}
		if((std::find(hot_rtns_vec.begin(), hot_rtns_vec.end(), targ_addr) != hot_rtns_vec.end()) && (curr_call_count > rtn_map[targ_addr]->calls_num/dominant_threshold)){
			hot_calls_vector.push_back(call_address);
			//cout << "HOT CALL: " << std::hex << call_address << endl;
		}
    }

	
///////////////////////////PRINT BBL MAP//////////////////////////////////////////

	
	sort(bbl_vec.begin(), bbl_vec.end(), compare_bbl);
	
	ofstream output2("bbl_prof.csv");
	
	output2 << "rtn_name , rtn_addr , head_addr , tail_addr , FT , next_ins , target , ins_count , taken_count , not_taken_count " << endl;
	for (const pair<const unsigned int, bbl_st*>& pair : bbl_vec) {
		const bbl_st& data = *(pair.second); 

			output2
			//<< data.routine_name << " , "
			<< std::hex	
			<< "0x"<<data.routine_address<<" , "	
			<< "0x"<<data.head_address	 <<" , "
			<< "0x"<<data.tail_address	 <<" , " 
			<< "0x"<<data.FT_target		 <<" , " 
			<< "0x"<<data.next_ins		 <<" , " 
			<< "0x"<<data.target		 <<" , " 
			<< std::dec 
			<<data.instructions_num 	 <<" , "
			<<data.taken_count		 	 <<" , "
			<<data.not_taken_count		 
			<<endl;
		
		delete pair.second;
	}
	output2.close();
///////////////////////////// WRITE BINARY/////////////////////////////////////////////////////
    std::string filePath = "counts.bin";
	writeBinary(filePath);

}
/* ===================================================================== */
/* Main                                                                  */
/* ===================================================================== */

int main(int argc, char * argv[])
{

    // Initialize pin & symbol manager
    //out = new std::ofstream("xed-print.out");

    if( PIN_Init(argc,argv) )
        return Usage();

    PIN_InitSymbols();
    if(KnobOpt){

	IMG_AddInstrumentFunction(ImageLoad, 0);
	PIN_StartProgramProbed();
    }
    else if(KnobProf){
		RTN_AddInstrumentFunction(ROUTINE, 0);
		TRACE_AddInstrumentFunction(TRACE_INSTRUMENTATION, 0);
		PIN_AddFiniFunction(Fini, 0);
		PIN_StartProgram();
	}
    else{


	PIN_StartProgramProbed();
	}

    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */
