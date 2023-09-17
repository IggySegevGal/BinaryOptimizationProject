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
	int calls_num ;
	
	map<ADDRINT,bbl_st> rtn_bbls_map;
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

KNOB<BOOL>   KnobInst(KNOB_MODE_WRITEONCE,    "pintool",
    "inst", "0", "run inst");

/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */
std::ofstream* out = 0;

map <ADDRINT,rtn_st*> rtn_map;
map <ADDRINT,bbl_st*> bbl_map;
vector<ADDRINT> hot_rtn_addr;
int rtn_num = 0;


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


instr_map_t *instr_map = NULL;
int num_of_instr_map_entries = 0;
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



/* ============================================================= */
/* Service dump routines                                         */
/* ============================================================= */


/* ===================COMPARE FUNCTIONS================================================== */

// a compare function to sort the map by ins_count
bool compare_rtn (const pair<unsigned int, rtn_st*>& a , const pair<unsigned int, rtn_st*>& b)
{
	return a.second->instructions_num > b.second->instructions_num;
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

        // Add routine data to map
        rtn_map[routineData->routine_address] = routineData;
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

	xed_decoded_inst_t xedd;
	xed_decoded_inst_zero_set_mode(&xedd,&dstate); 
				   
	xed_error_enum_t xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
	if (xed_code != XED_ERROR_NONE) {
		cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << instr_map[instr_map_entry].new_ins_addr << endl;
		return -1;
	}
	
	xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
	
	if (category_enum != XED_CATEGORY_CALL && category_enum != XED_CATEGORY_UNCOND_BR) {

		cerr << "ERROR: Invalid direct jump from translated code to original code in rotuine: " 
			  << RTN_Name(RTN_FindByAddress(instr_map[instr_map_entry].orig_ins_addr)) << endl;
		dump_instr_map_entry(instr_map_entry);
		return -1;
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
    // go over routines and check if they are candidates for translation and mark them for translation:
cerr << "before for" << endl;
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
			if(RTN_Name(rtn)!= "hasSuffix"){
				continue;
			}
            translated_rtn[translated_rtn_num].rtn_addr = RTN_Address(rtn);            
            translated_rtn[translated_rtn_num].rtn_size = RTN_Size(rtn);

            // Open the RTN.
            RTN_Open( rtn ); 
			//eyal
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
						translated_rtn[translated_rtn_num].instr_map_entry = -1;
						break;
					}
					
					// Save xed and addr into a map to be used later.
					rtn_local_instrs_map[addr] = xedd;
					
				} // end for INS...
				
				//--------------REORDERING-----------------------------------------------------------------------------------
								
				map<ADDRINT,bbl_st> tmp_rtn_bbls_map = rtn_map[RTN_Address(rtn)]->rtn_bbls_map;

				INS first_ins = RTN_InsHead(rtn);			
				bbl_st curr_bbl=tmp_rtn_bbls_map[INS_Address(first_ins)];
				bbl_st next_bbl;
				ADDRINT head_address=curr_bbl.head_address;
				ADDRINT tail_address=curr_bbl.tail_address;
				while(!tmp_rtn_bbls_map.empty()){
					ADDRINT original_target_of_tail=0;
					auto start_it=rtn_local_instrs_map.find(head_address);
					auto end_it=rtn_local_instrs_map.find(tail_address);
					//insert all instruction of the BLL to the instruction map (without the tail)
					for (auto it = start_it; it != end_it; ++it) {
						ADDRINT addr = it->first;
						xed_decoded_inst_t xedd = it->second;           

					   // Check if we are at a routine header:
					   if (translated_rtn[rtn_num].rtn_addr == addr) {
						   translated_rtn[rtn_num].instr_map_entry = num_of_instr_map_entries;
						   translated_rtn[rtn_num].isSafeForReplacedProbe = true;
						   rtn_num++;
					   }
					          //debug print of orig instruction:
					   if (KnobVerbose) {
						 char disasm_buf[2048];
						 xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(addr), 0, 0);               
						 cerr << "0x" << hex << addr << ": " << disasm_buf  <<  endl; 
					   }
					   //insert 
					   	int rc = add_new_instr_entry(&xedd, addr, xed_decoded_inst_get_length(&xedd),0,0);
						if (rc < 0) {
							cerr << "ERROR: failed during instructon translation." << endl;
							translated_rtn[rtn_num].instr_map_entry = -1;
							break;
						}
					}
					
					tmp_rtn_bbls_map.erase(head_address); //remove the BBL and find the next one
					//info for entering the tail into the global map (might be changed if reordering is needed)
					ADDRINT addr =tail_address ; 
					xed_decoded_inst_t xedd = rtn_local_instrs_map[tail_address];
					
					//add either the original or an inverted tail depending of taken/not_taken profile
					if(curr_bbl.FT_target!=0 && curr_bbl.target!=0){//this is a branch and we need to choose the next bbl depending on taken/not taken frequency
						if(curr_bbl.taken_count<curr_bbl.not_taken_count){
							//insert the regular jump
							if(tmp_rtn_bbls_map.find(curr_bbl.next_ins)!=tmp_rtn_bbls_map.end()){//check if the new FT target was already added
								next_bbl=tmp_rtn_bbls_map[curr_bbl.next_ins];// declare next bbl to be the original FT
							}
							else{ // if the wanted FT was already inserted, just take another unrelated bbl from the map.
								next_bbl=tmp_rtn_bbls_map.begin()->second;
								//FUTURE OPTIMIZATION: TRY TO USE THE JUMP TARGET BBL AS THE NEXT BBL 
							}
							
						}
						else{ //REORDER
						
							//insert an INVERTED jump 
							invert_jmp(&xedd);
							original_target_of_tail=curr_bbl.next_ins;
							cerr << "REORDERING!! head address is:  "<<std::hex<< head_address << " tail address is: " << tail_address<< endl;
							//xedd = rtn_local_instrs_map[tail_address];//*************FIX THIS to inverted jmp*************************	
							if(tmp_rtn_bbls_map.find(curr_bbl.target)!=tmp_rtn_bbls_map.end()){//check if the new FT target was already added
								next_bbl=tmp_rtn_bbls_map[curr_bbl.target];// declare next bbl to be the jump target FT
								rtn_map[RTN_Address(rtn)]->rtn_bbls_map[curr_bbl.target].was_reordered=1;// this symbolizes that the BBL is not in it's original place and might need a jmp at the end to it;s FT
							}
							else{
								next_bbl=tmp_rtn_bbls_map.begin()->second;
								//tmp_rtn_bbls_map.begin()
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
							}
						}
						else{
							//if the next instruction is NOT in the same routine, we want to take the first BBL from the top
							
							next_bbl=tmp_rtn_bbls_map.begin()->second;
						}
					
					
					}
					//insert the tail to the map

				   if (KnobVerbose) {
					 char disasm_buf[2048];
					 xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(addr), 0, 0);               
					 cerr << "0x" << hex << addr << ": " << disasm_buf  <<  endl; 
				   }
				   //insert 
					int rc = add_new_instr_entry(&xedd, addr, xed_decoded_inst_get_length(&xedd),0,original_target_of_tail);
					if (rc < 0) {
						cerr << "ERROR: failed during instructon translation." << endl;
						translated_rtn[rtn_num].instr_map_entry = -1;
						break;
					}
					
				//	if(rtn_map[RTN_Address(rtn)]->rtn_bbls_map[head_address].was_reordered==1 || rtn_map[RTN_Address(rtn)]->rtn_bbls_map[curr_bbl.next_ins].was_reordered==1){
					
					if(curr_bbl.next_ins != next_bbl.head_address){// check if the next BBL is the original FT. if not, add a jmp instruction to the original FT 
					// *****************ADD JUMP INSTRUCTION*******************************************
					// *****************ADD JUMP INSTRUCTION*******************************************
					// *****************ADD JUMP INSTRUCTION*******************************************
					// *****************ADD JUMP INSTRUCTION*******************************************
					// *****************ADD JUMP INSTRUCTION*******************************************
					}
					// if curr BBL was reordered or the FT was, we need to add a a jump instruction	
					//CHOOSE THE NEXT BBL 
					curr_bbl=next_bbl;
					first_ins = RTN_InsHead(rtn);			
					head_address=curr_bbl.head_address;
					tail_address=curr_bbl.tail_address;
				}
					

					
					
					
		
				
			/*
            if(RTN_Name=="myMalloc"){
				INS first_ins = RTN_InsHead(rtn);
				
				map<ADDRINT,bbl_st> tmp_rtn_bbls_map = *(rtn_map[RTN_Address(rtn)]->rtn_bbls_map);
				bbl_st curr_bbl=tmp_rtn_bbls_map[first_ins];
				ADDRINT head_address=curr_bbl.head_address;
				ADDRINT tail_address=curr_bbl.tail_address;
				//iterate over all bbls and insert them to the map
				while(!tmp_rtn_bbls_map.empty()){
					// insert all of the bbl's instructions (besides the tail that might change)
					for (INS ins = head_address; INS_Address(ins)<tail_address; ins = INS_Next(ins)) { 
						ADDRINT addr = INS_Address(ins);
						
						//debug print of orig instruction:
						if (KnobVerbose) {
							cerr << "old instr: ";
							cerr << "0x" << hex << INS_Address(ins) << ": " << INS_Disassemble(ins) <<  endl;
							//xed_print_hex_line(reinterpret_cast<UINT8*>(INS_Address (ins)), INS_Size(ins));                               
						}        
						
						xed_decoded_inst_t xedd;
						xed_error_enum_t xed_code;                            
						
						xed_decoded_inst_zero_set_mode(&xedd,&dstate); 

						xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(addr), max_inst_len);
						if (xed_code != XED_ERROR_NONE) {
							cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << addr << endl;
							translated_rtn[translated_rtn_num].instr_map_entry = -1;
							break;
						}
						
						// Save xed and addr into a map to be used later.
						local_instrs_map[addr] = xedd;
					}
					//INSERT THE CORRECT TAIL (depends on the bbl's head of taken/fallthrough)
					if(curr_bbl.taken_count > curr_bbl.not_taken_count){
						//insert inverted jump
					}
					else{
						//insert regular jump
						
					}
					
					
				}
			}
			*/
		// test for no reordering	
		/*
    for(const auto& pair : rtn_local_instrs_map) {
        local_instrs_map.insert(pair);
    }*/



            // debug print of routine name:
            if (KnobVerbose) {
                cerr <<   "rtn name: " << RTN_Name(rtn) << " : " << dec << translated_rtn_num << endl;
            }            

            // Close the RTN.
            RTN_Close( rtn );

            translated_rtn_num++;

        } // end for RTN..
    } // end for SEC...


    // Go over the local_instrs_map map and add each instruction to the instr_map:
/*    
    for (map<ADDRINT, xed_decoded_inst_t>::iterator iter = local_instrs_map.begin(); iter != local_instrs_map.end(); iter++) {
       ADDRINT addr = iter->first;
       xed_decoded_inst_t xedd = iter->second;           

       // Check if we are at a routine header:
       if (translated_rtn[rtn_num].rtn_addr == addr) {
           translated_rtn[rtn_num].instr_map_entry = num_of_instr_map_entries;
           translated_rtn[rtn_num].isSafeForReplacedProbe = true;
           rtn_num++;
       }
    
       //debug print of orig instruction:
       if (KnobVerbose) {
         char disasm_buf[2048];
         xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(addr), 0, 0);               
         cerr << "0x" << hex << addr << ": " << disasm_buf  <<  endl; 
       }
      */
       
       /* ///////////////////////////// delete hereeeeeeeeeee
       // Check if this is a direct call instr:    
		xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
		xed_int64_t disp = xed_decoded_inst_get_branch_displacement(&xedd);
		ADDRINT target_addr = addr + xed_decoded_inst_get_length (&xedd) + disp;
		RTN rtn = RTN_FindByAddress(target_addr);
		
		if ((category_enum == XED_CATEGORY_CALL ) && ((RTN_Name(rtn) == "add_pair_to_block") || RTN_Name(rtn) == "BZ2_blockSort")) { 
			inline_counter++;
			cerr<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			cerr<<RTN_Name(rtn)<<endl;
			cerr<< " caller address: " <<addr<<endl;
			cerr<<" caller_rtn: "<<RTN_Name(RTN_FindByAddress(addr))<<endl;
			cerr<<" inline counter: "<< inline_counter<<endl;
			cerr<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

			//THIS IS THE INLINING PART
			RTN_Open(rtn);  	  
			for (INS ins = RTN_InsHead(rtn); !INS_IsRet(ins); ins = INS_Next(ins)) {
                ADDRINT addr = INS_Address(ins);
			    xed_decoded_inst_t inline_xedd =local_instrs_map[addr]; // TAKE THE XEDD OF THE INST FROM THE LOCAL MAP
			   	xed_uint_t disp_byts = xed_decoded_inst_get_branch_displacement_width(&inline_xedd);//check if the jump target is inside the routine or not
				xed_int32_t disp;
				ADDRINT inline_targ_addr = 0;
				if (disp_byts > 0) { // there is a branch offset.
					disp = xed_decoded_inst_get_branch_displacement(&inline_xedd);
					inline_targ_addr = addr + xed_decoded_inst_get_length (&inline_xedd) + disp;
					if(RTN_FindByAddress(inline_targ_addr)==rtn){ // the jump target is inside the inlined function
						//insert with inline_counter != 0 (to match with inlined code and not original code)
						int rc = add_new_instr_entry(&inline_xedd, addr, xed_decoded_inst_get_length(&inline_xedd),inline_counter,0);//ADD THE INST TO THE GLOBAL MAP
						if (rc < 0) {
							cerr << "ERROR: failed during instructon translation." << endl;
							translated_rtn[rtn_num].instr_map_entry = -1;
							break;
						}
					}
					else { // jump is outside of the inlined code (current rtn)
						//insert with inline_counter == 0 (to match with original code)
						int rc = add_new_instr_entry(&inline_xedd, addr, xed_decoded_inst_get_length(&inline_xedd),0,0);//ADD THE INST TO THE GLOBAL MAP
						if (rc < 0) {
							cerr << "ERROR: failed during instructon translation." << endl;
							translated_rtn[rtn_num].instr_map_entry = -1;
							break;
						} 
					}
				  
				} else{
				int rc = add_new_instr_entry(&inline_xedd, addr, xed_decoded_inst_get_length(&inline_xedd),inline_counter,0);//ADD THE INST TO THE GLOBAL MAP
				if (rc < 0) {
					cerr << "ERROR: failed during instructon translation." << endl;
					translated_rtn[rtn_num].instr_map_entry = -1;
					break;
				}
				cerr << " Callee addr: " << std::hex << addr << "\n";
				}
			}
			RTN_Close(rtn);
	
			
		}
		*/ ///////////////////////////// delete hereeeeeeeeeee
		//else{ ///////////////////////////// delete hereeeeeeeeeee
			// Add instr into global instr_map:
			/*
			int rc = add_new_instr_entry(&xedd, addr, xed_decoded_inst_get_length(&xedd),0,0);
			if (rc < 0) {
				cerr << "ERROR: failed during instructon translation." << endl;
				translated_rtn[rtn_num].instr_map_entry = -1;
				break;
			}
			*/
		//}	   ///////////////////////////// delete hereeeeeeeeeee
      
       
     // end for map<...

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

	   cerr << endl << "instructions map dump:" << endl;
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
		
	//checkt that thr routine is valid and that it is in the main executable image
	if (RTN_Valid(my_rtn) && IMG_IsMainExecutable(img)) {
		RTN_Open(my_rtn);
		//count the amount of calls to this routine
		RTN_InsertCall(my_rtn, IPOINT_BEFORE, (AFUNPTR)docount,IARG_PTR, &(rtn->calls_num), IARG_END);
		//count the amount of instructions in this routine
		for (INS ins =RTN_InsHead(my_rtn); INS_Valid(ins); ins = INS_Next(ins)) {
			INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)docount,IARG_PTR, &(rtn->instructions_num), IARG_END);	
		}
	
	//add the routine to the map
	rtn_map[my_addr] = rtn;
	RTN_Close(my_rtn);
	}		
}
/* ===================================================================== */
VOID Taken_count(unsigned int *taken_count , unsigned int *not_taken_count ,INT32 is_taken) { 
	if(is_taken){
		(*taken_count)++;
	}
	else{
		(*not_taken_count)++;
	}
 }
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
	
	
	vector<pair<unsigned int, bbl_st*>> bbl_vec(bbl_map.begin(), bbl_map.end());
// insert the bbl's to their routines 
	for (const pair<const unsigned int, bbl_st*>& pair : bbl_vec) {
		const bbl_st& current_bbl = *(pair.second);
		ADDRINT current_rtn_addr = current_bbl.routine_address;		
		if(rtn_map.find(current_rtn_addr)==rtn_map.end()){continue;}//check if routine has been translated
			//insert all the bbls to theire respective rtns. in case of 2 bbls with the same tail, take the one the contains the other (smalles head address)
			if(rtn_map[current_rtn_addr]->rtn_bbls_map.find(current_bbl.tail_address)==rtn_map[current_rtn_addr]->rtn_bbls_map.end()){
				rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address]=current_bbl;
			}
			else{
				if(rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address].head_address > current_bbl.head_address){
					rtn_map[current_rtn_addr]->rtn_bbls_map[current_bbl.tail_address]=current_bbl;
				}
			}
			
			
	}

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
	//use std::sort and my comapre function to sort the vector by instruction_num 
	sort(sorted_vec.begin(), sorted_vec.end(), compare_rtn);
	//print the sorted map
	ofstream output("loop-count.csv");
	for (const pair<const unsigned int, rtn_st*>& pair : sorted_vec) {
		const rtn_st& data = *(pair.second); 
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
			for (const auto& pair2 : data.rtn_bbls_map) {
				ADDRINT head_addr=pair2.first;
				output << " , " << "0x" << head_addr << " ";
			}
			output<<endl;
		}
	}
	output.close();
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
    if(KnobInst){
	//read the csv here 
/*
	//V2
	FILE* file =fopen("loop-count.csv" ,"r");
	if(file){
		ADDRINT addr;
		int count=0;
		//take only the first column and drop the rest until a newline
		while(std::fscanf(file,"0x%lx%*[^\n]%*c",&addr) !=EOF &&count<10){
			hot_rtn_addr.push_back(addr);
			count++;
		}
		fclose(file);
	} else {
		cout<<"could not open file"<< endl;
	}
	//for( size_t i=0; i<hot_rtn_addr.size();i++){
	//	cout<<hot_rtn_addr[i]<<endl;
	//}
	//
	 //Register ImageLoad
	 */
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
/*
	ifstream infile("loop-count.csv");
	string line;
	string field;
	for(int i =0; i<10;i++){
		getline(infile, line);
		istringstream s(line);//create a string stream of each line
		getline(s,field,',');//take the first field of each line
 		//cout << stoul(field,nullptr,16) <<endl;
		ADDRINT addr=std::stoull(field,nullptr,16);
		hot_rtn_addr.push_back(addr);
	}
*/

	PIN_StartProgramProbed();
	}

    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */
