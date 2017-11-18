/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

/***************************************************************************
 * Modified the vanilla McPAT application to perform batch Runtime dynamic
 * Power (RT-DP) computation by accepting as input a folder containing the
 * batch of input XML files. The batch RT-DP computation is tailored for the
 * ARM_AtomicSimpleCPU_template.xml processor description template. This work
 * was done as part of the COSSIM project. http://www.cossim.org/
 * 
 * Modified by: Prajith R G
 * 		Chalmers University of Technology
 * 		Date: 17/11/2017
 *
 ***************************************************************************/

#include "io.h"
#include <iostream>
#include "xmlParser.h"
#include "XML_Parse.h"
#include "processor.h"
#include "globalvar.h"
#include "version.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <chrono>
#include <fstream>

#define inVectorSize 64
#define outVectorSize 48
#define is_debug false

using namespace std;

void print_usage(char * argv0);
void generate_input_data(Processor proc, ParseXML *, int batch_index, float *dataIn);
void batch_RT_DP_compute(Processor proc, float* dataIn, float* dataOut, int batchSize);
void dump_RT_DP_Output_CSV(float *dataOutm, int);

int main(int argc,char *argv[])
{
	char * fb ;
	bool infile_specified = false;
	bool is_batch_size_specified = false;
	int  plevel = 2;
	opt_for_clk	= true;
	int batchSize; 

	if (argc <= 2 || argv[1] == string("-h") || argv[1] == string("--help"))
	{
		print_usage(argv[0]);
	}

	for (int32_t i = 0; i < argc; i++)
	{
		if (argv[i] == string("-infile"))
		{
			infile_specified = true;
			i++;
			fb = argv[i];
		}

		if (argv[i] == string("-batchSize"))
		{
			is_batch_size_specified = true;
			i++;
			batchSize = atoi(argv[i]);
		}

		if (argv[i] == string("-print_level"))
		{
			i++;
			plevel = atoi(argv[i]);
		}

		if (argv[i] == string("-opt_for_clk"))
		{
			i++;
			opt_for_clk = (bool)atoi(argv[i]);
		}
	}

	if (infile_specified == false || is_batch_size_specified == false)
	{
		print_usage(argv[0]);
	}

	cout<<"--------------------------------------------------\n";
	cout<<"  Batch-McPAT (version 1.0 of Nov 2017)"<<endl;
	cout<<"--------------------------------------------------\n";

	//Init data input and output
	size_t inSizeBytes = inVectorSize * batchSize *  sizeof(float);
	size_t outSizeBytes = outVectorSize * batchSize *  sizeof(float);
	float *dataIn = (float*) malloc(inSizeBytes);
	float *dataOut = (float*) malloc(outSizeBytes);
	for(int i = 0; i < (batchSize * inVectorSize); i++) {
		dataIn[i] = i;
	}
	for(int i = 0; i < (outVectorSize * batchSize); i++) {
		dataOut[i] = 0;
	}

	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();

	//Init XML Parser
	ParseXML *myParser= new ParseXML();
	char buf[20], str[200];
	strcpy(str, fb);
	sprintf(buf, "%d.xml", 0);
	/*Parse the first mcpatIn0.xml*/
	myParser->parse(strcat(str, buf));

	/*Populate Seed processor from mcpatIn0.xml*/
	Processor seedProc = Processor(myParser);
	seedProc.displayEnergy(2, plevel);
	strcpy(str, fb);

	/*Each input XML file in the batch is parsed to collect
	  the component activity factors as reported from cGEM5
	  and input data is generated to feed the RT-DP computation
	 */
	cout<<"      : input XML files parsed..";
	for (int i=0; i<batchSize; i++){
		sprintf(buf, "%d.xml", i);
		cout<<"\r"<< i + 1;
		myParser->parse(strcat(str, buf));
		generate_input_data(seedProc, myParser, i, dataIn);
		strcpy(str, fb);
	}
	cout<<endl;
	t2 = std::chrono::high_resolution_clock::now();
	auto parseDuration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
	cout.precision(12);
	if(is_debug) cout << "Time taken for parsing and generating input data = " << (double)parseDuration/1E9 <<"s" <<endl;

	t1 = std::chrono::high_resolution_clock::now();
	if(is_debug) cout << "Calling batch RT-DP Compute function.."<< endl;
	batch_RT_DP_compute(seedProc, dataIn, dataOut, batchSize);
	t2 = std::chrono::high_resolution_clock::now();
	auto batch_rt_dp_duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
	cout.precision(12);
	if(is_debug) cout << "Time taken for batch RT-DP computation (including scalar init) = " << (double)batch_rt_dp_duration/1E9 <<"s" <<endl;

	t1 = std::chrono::high_resolution_clock::now();
	dump_RT_DP_Output_CSV(dataOut, batchSize);
	t2 = std::chrono::high_resolution_clock::now();
	auto csvDuration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
	cout.precision(12);
	if(is_debug) cout << "Time taken for dumping batch RT-DP output into CSV file = " << (double)csvDuration/1E9 <<"s" <<endl;

	delete myParser, dataIn, dataOut, seedProc;
	return 0;
}

void dump_RT_DP_Output_CSV(float *dataOut, int batchSize){

	if(is_debug)  cout<< "Dumping CSV output.."<<endl;
	ofstream myFile;
	myFile.open ("Batch_McPAT_RT_DP_Output.csv");
	myFile << "IFU-ICache"<< "," << "IFU-Branch Target Buffer"<< "," << "IFU-Instruction Buffer"<< "," << "IFU-BPT-Global Predictor"<< ","
			<< "IFU-BPT-L1_Local Predictor"<< "," << "IFU-BPT-L2_Local Predictor"<< "," << "IFU-BPT-Chooser"<< "," << "IFU-BPT-RAS"<< ","
			<< "IFU-BPT"<< "," << "IFU-Instruction Decoder"<< "," << "Instruction Fetch Unit (IFU)"<< "," << "Load Store Unit"<< ","
			<< "LSU-Data Cache"<< "," << "LSU-Load/Store Queue"<< "," << "MMU-Itlb"<< "," << "MMU-Dtlb"<< "," << "MMU"<< ","
			<< "Execution Unit"<< "," << "EXU-Register Files"<< "," << "EXU-Integer RF"<< "," << "EXU-Floating Point RF" << ","
			<< "Instruction Scheduler" << "," << "Int Inst Window" << "," << "FP Inst Window" << "," << "Integer ALUs" << ","
			<< "FPU" << "," << "MUL" << ","	<< "Bypass" << "," << "iFRAT" << "," << "fFRAT" << "," << "ifreeL" << "," << "ffreeL" << ","
			<< "RNU" << "," << "unicache" << "," << "BUSES" << "," << "Core" << "," << "Processor" << endl;

	for (int i = 0; i < batchSize; i++) {
		for (int j=0; j < outVectorSize; j++){
			long index = i * outVectorSize + j;
			if (j > 36) continue;
			else if (j == 36) myFile << dataOut[index] << endl;
			else myFile << dataOut[index] << ",";
		}
	}

	myFile.close();
	cout<<"Runtime Dynamic Power results written to Batch_McPAT_RT_DP_Output.csv. DONE!"<< endl;
}

void print_usage(char * argv0)
{
	cerr<<"--------------------------------------------------\n";
	cerr<<"  Batch-McPAT (version 1.0 of Nov 2017)"<<endl;
	cerr<<"--------------------------------------------------\n";
	cerr << "Usage: ./batch_mcpat -infile < absolute directory path + batch XML file prefix >  -batchSize < No of input XML files > " << endl;
	//-print_level < level of details 0~5 >  -opt_for_clk < 0 (optimize for ED^2P only)/1 (optimzed for target clock rate)>"<< endl;
	cerr << "Example: ./batch_mcpat -infile /home/demo/COSSIM_Maxeler_DEMO/COSSIM/gem5/McPat/node0/mcpatIn -batchSize 1000" << endl;
	cerr << "This takes as input, a batch of XML files (mcpatIn0.xml ... mcpatIn999.xml) and outputs the Batch_McPAT_RT_DP_Output.csv file in the current working directory." << endl;
	//cerr << "Note:default print level is at processor level, please increase it to see the details" << endl;
	cerr<<"--------------------------------------------------\n";
	cerr << "Exiting!" << endl;
	exit(1);
}

/**
 * Generates batch input data containing the component activity factors as reported from cGEM5
 */
void generate_input_data(Processor proc, ParseXML *XML, int i, float *dataIn)
{
	int ithCore =0;
	//IFU
	dataIn[0 + i*inVectorSize] = XML->sys.core[ithCore].icache.read_accesses;
	dataIn[1 + i*inVectorSize] = XML->sys.core[ithCore].icache.read_misses;
	dataIn[2 + i*inVectorSize] = XML->sys.core[ithCore].total_instructions;
	dataIn[3 + i*inVectorSize] = XML->sys.core[ithCore].total_instructions;
	dataIn[4 + i*inVectorSize] = XML->sys.core[ithCore].BTB.read_accesses;
	dataIn[5 + i*inVectorSize] = XML->sys.core[ithCore].BTB.write_accesses;
	dataIn[6 + i*inVectorSize] = XML->sys.core[ithCore].total_instructions;
	dataIn[7 + i*inVectorSize] = XML->sys.core[ithCore].branch_instructions;
	dataIn[8 + i*inVectorSize] = XML->sys.core[ithCore].branch_mispredictions;
	dataIn[9 + i*inVectorSize] = XML->sys.core[ithCore].function_calls;

	///Local Power
	////Icache
	//Core Pipe
	dataIn[10 + i*inVectorSize] = proc.cores[0]->coredynp.pipeline_duty_cycle;
	dataIn[11 + i*inVectorSize] = proc.cores[0]->coredynp.IFU_duty_cycle;
	dataIn[12 + i*inVectorSize] = XML->sys.total_cycles;
	dataIn[13 + i*inVectorSize] = XML->sys.number_of_cores;
	dataIn[14 + i*inVectorSize] = proc.cores[0]->coredynp.num_pipelines;

	//LSU
	dataIn[15+ i*inVectorSize] = XML->sys.core[ithCore].dcache.read_accesses;
	dataIn[16+ i*inVectorSize] = XML->sys.core[ithCore].dcache.read_misses;
	dataIn[17+ i*inVectorSize] = XML->sys.core[ithCore].dcache.write_accesses;
	dataIn[18+ i*inVectorSize] = XML->sys.core[ithCore].dcache.write_misses;
	dataIn[19+ i*inVectorSize] = XML->sys.core[ithCore].load_instructions;
	dataIn[20+ i*inVectorSize] = XML->sys.core[ithCore].store_instructions;
	dataIn[21+ i*inVectorSize]=proc.cores[0]->coredynp.LSU_duty_cycle;

	//MMU
	dataIn[22+ i*inVectorSize] = XML->sys.core[ithCore].itlb.total_accesses;
	dataIn[23+ i*inVectorSize] = XML->sys.core[ithCore].itlb.total_misses;
	dataIn[24+ i*inVectorSize] = XML->sys.core[ithCore].dtlb.total_accesses;
	dataIn[25+ i*inVectorSize] = XML->sys.core[ithCore].dtlb.total_misses;

	//EXU
	///RFU
	dataIn[26+ i*inVectorSize] = XML->sys.core[ithCore].int_regfile_reads;
	dataIn[27+ i*inVectorSize] = XML->sys.core[ithCore].int_regfile_writes;
	dataIn[28+ i*inVectorSize] = XML->sys.core[ithCore].float_regfile_reads;
	dataIn[29+ i*inVectorSize] = XML->sys.core[ithCore].float_regfile_writes;

	///SHEU
	dataIn[30+ i*inVectorSize] = XML->sys.core[ithCore].inst_window_reads;
	dataIn[31+ i*inVectorSize] = XML->sys.core[ithCore].inst_window_writes;
	dataIn[32+ i*inVectorSize] = XML->sys.core[ithCore].inst_window_wakeup_accesses;
	dataIn[33+ i*inVectorSize] = XML->sys.core[ithCore].fp_inst_window_reads;
	dataIn[34+ i*inVectorSize] = XML->sys.core[ithCore].fp_inst_window_writes;
	dataIn[35+ i*inVectorSize] = XML->sys.core[ithCore].fp_inst_window_wakeup_accesses;

	///EXEU
	dataIn[36+ i*inVectorSize] = XML->sys.core[ithCore].ialu_accesses;

	///FPU
	dataIn[37+ i*inVectorSize] = XML->sys.core[ithCore].fpu_accesses;

	//MUL
	dataIn[38+ i*inVectorSize] = XML->sys.core[ithCore].mul_accesses;

	///bypass
	dataIn[39+ i*inVectorSize] = XML->sys.core[ithCore].cdb_alu_accesses;

	////MUL
	dataIn[40+ i*inVectorSize] = XML->sys.core[ithCore].cdb_mul_accesses;

	////FP
	dataIn[41+ i*inVectorSize] = XML->sys.core[ithCore].cdb_fpu_accesses;
	dataIn[42+ i*inVectorSize] = proc.cores[0]->coredynp.ALU_duty_cycle;

	//RNU
	dataIn[43+ i*inVectorSize] = XML->sys.core[ithCore].rename_reads;
	dataIn[44+ i*inVectorSize] = XML->sys.core[ithCore].rename_writes;
	dataIn[45+ i*inVectorSize] = XML->sys.core[ithCore].fp_rename_reads;
	dataIn[46+ i*inVectorSize] = XML->sys.core[ithCore].fp_rename_writes;

	//First Level Directory TODO x2
	//Commenting #COSSIM_MAX Mcpat template does not have L1 Dir
	//dataIn[47+ i*inVectorSize] = XML->sys.L1Directory[0].read_accesses;
	//dataIn[48+ i*inVectorSize] = XML->sys.L1Directory[0].write_accesses;

	//NOC - Type BUS (0)
	dataIn[49+ i*inVectorSize] = XML->sys.NoC[0].total_accesses;

	//Execution Time
	dataIn[50 + i*inVectorSize] = XML->sys.total_cycles/proc.cores[0]->coredynp.clockRate;//proc.cores[0]->executionTime;

}


/**
 * Batch RT-DP compute method
 */
void batch_RT_DP_compute(Processor proc, float* dataIn, float* dataOut, int batchSize){

	/* One time init of constant base energies using the first seed processor (proc) input mcpatIn0.xml file */
	float busBasePower = 0;//proc.nocs[0]->link_bus->power.readOp.dynamic;
	float corepipePowerReadOpDynamic = proc.cores[0]->corepipe->power.readOp.dynamic;
	float dcacheCachesLocal_resultPowerReadOpDynamic = proc.cores[0]->lsu->dcache.caches->local_result.power.readOp.dynamic;
	float dcacheCachesLocal_resultPowerWriteOpDynamic = proc.cores[0]->lsu->dcache.caches->local_result.power.writeOp.dynamic;
	float dcacheCachesLocal_resultTag_array2PowerReadOpDynamic = proc.cores[0]->lsu->dcache.caches->local_result.tag_array2->power.readOp.dynamic;
	float dcacheIFBLocal_resultPowerSearchOpDynamic = proc.cores[0]->lsu->dcache.ifb->local_result.power.searchOp.dynamic;
	float dcacheIFBLocal_resultPowerWriteOpDynamic = proc.cores[0]->lsu->dcache.ifb->local_result.power.writeOp.dynamic;
	float dcacheLSQLocal_resultPowerReadOpDynamic = proc.cores[0]->lsu->LSQ->local_result.power.readOp.dynamic;
	float dcacheLSQLocal_resultPowerSearchOpDynamic = proc.cores[0]->lsu->LSQ->local_result.power.searchOp.dynamic;
	float dcacheLSQLocal_resultPowerWriteOpDynamic = proc.cores[0]->lsu->LSQ->local_result.power.writeOp.dynamic;
	float dcacheMissbLocal_resultPowerSearchOpDynamic = proc.cores[0]->lsu->dcache.missb->local_result.power.searchOp.dynamic;
	float dcacheMissbLocal_resultPowerWriteOpDynamic = proc.cores[0]->lsu->dcache.missb->local_result.power.writeOp.dynamic;
	float dcachePrefetchBLocal_resultPowerSearchOpDynamic = proc.cores[0]->lsu->dcache.prefetchb->local_result.power.searchOp.dynamic;
	float dcachePrefetchBLocal_resultPowerWriteOpDynamic = proc.cores[0]->lsu->dcache.prefetchb->local_result.power.writeOp.dynamic;
	float dcacheWBBLocal_resultPowerSearchOpDynamic = proc.cores[0]->lsu->dcache.wbb->local_result.power.searchOp.dynamic;
	float dcacheWBBLocal_resultPowerWriteOpDynamic = proc.cores[0]->lsu->dcache.wbb->local_result.power.writeOp.dynamic;
	float dtlbLocal_resultPowerSearchOpDynamic = proc.cores[0]->mmu->dtlb->local_result.power.searchOp.dynamic;
	float dtlbLocal_resultPowerWriteOpDynamic = proc.cores[0]->mmu->dtlb->local_result.power.writeOp.dynamic;
	float exuPerAccessEnergy = proc.cores[0]->exu->exeu->per_access_energy;
	float exubase_energy =  proc.cores[0]->exu->exeu->base_energy;
	float fFRATLocal_resultPowerReadOpDynamic = 0;//proc.cores[0]->rnu->fFRAT->local_result.power.readOp.dynamic;
	float fFRATLocal_resultPowerWriteOpDynamic = 0;//proc.cores[0]->rnu->fFRAT->local_result.power.writeOp.dynamic;
	float fdclPowerReadOpDynamic = 0;//proc.cores[0]->rnu->fdcl->power.readOp.dynamic;
	float ffreeLLocal_resultPowerReadOpDynamic = 0;//proc.cores[0]->rnu->ifreeL->local_result.power.readOp.dynamic;
	float ffreeLLocal_resultPowerWriteOpDynamic = 0;//proc.cores[0]->rnu->ffreeL->local_result.power.writeOp.dynamic;
	float fpTagBypassPowerReadOpDynamic = proc.cores[0]->exu->fpTagBypass->power.readOp.dynamic;
	float fp_BypassPowerReadOpDynamic = proc.cores[0]->exu->fp_bypass->power.readOp.dynamic;
	float fp_inst_windowLocal_resultPowerReadOpDynamic = 0;//proc.cores[0]->exu->scheu->fp_inst_window->local_result.power.readOp.dynamic;
	float fp_inst_windowLocal_resultPowerSearchOpDynamic = 0;//proc.cores[0]->exu->scheu->fp_inst_window->local_result.power.searchOp.dynamic;
	float fp_inst_windowLocal_resultPowerWriteOpDynamic = 0;//proc.cores[0]->exu->scheu->fp_inst_window->local_result.power.writeOp.dynamic;
	float fpuPerAccessEnergy = proc.cores[0]->exu->fp_u->per_access_energy;
	float fpuSckRation = proc.cores[0]->exu->fp_u->getG_tpSckt_co_eff();
	float fpubase_energy = proc.cores[0]->exu->fp_u->base_energy;
	float frfLocal_resultPowerReadOpDynamic = proc.cores[0]->exu->rfu->FRF->local_result.power.readOp.dynamic;
	float frfLocal_resultPowerWriteOpDynamic = proc.cores[0]->exu->rfu->FRF->local_result.power.writeOp.dynamic;
	float iFRATLocal_resultPowerReadOpDynamic = 0;//proc.cores[0]->rnu->iFRAT->local_result.power.readOp.dynamic;
	float iFRATLocal_resultPowerWriteOpDynamic = 0;//proc.cores[0]->rnu->iFRAT->local_result.power.writeOp.dynamic;
	float idclPowerReadOpDynamic = 0;// proc.cores[0]->rnu->idcl->power.readOp.dynamic;
	float ifreeLLocal_resultPowerReadOpDynamic = 0;//proc.cores[0]->rnu->ffreeL->local_result.power.readOp.dynamic;
	float ifreeLLocal_resultPowerWriteOpDynamic = 0;// proc.cores[0]->rnu->ifreeL->local_result.power.writeOp.dynamic;
	float ifuBPTChooserLocal_resultPowerReadOpDynamic = proc.cores[0]->ifu->BPT->chooser->local_result.power.readOp.dynamic;
	float ifuBPTChooserLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->BPT->chooser->local_result.power.writeOp.dynamic;
	float ifuBPTGlobalBPTLocal_resultPowerReadOpDynamic = proc.cores[0]->ifu->BPT->globalBPT->local_result.power.readOp.dynamic;
	float ifuBPTGlobalBPTLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->BPT->globalBPT->local_result.power.writeOp.dynamic;
	float ifuBPTL1LocalBPTLocal_resultPowerReadOpDynamic = proc.cores[0]->ifu->BPT->L1_localBPT->local_result.power.readOp.dynamic;
	float ifuBPTL1LocalBPTLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->BPT->L1_localBPT->local_result.power.writeOp.dynamic;
	float ifuBPTL2LocalBPTLocal_resultPowerReadOpDynamic = proc.cores[0]->ifu->BPT->L2_localBPT->local_result.power.readOp.dynamic;
	float ifuBPTL2LocalBPTLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->BPT->L2_localBPT->local_result.power.writeOp.dynamic;
	float ifuBPTRASLocal_resultPowerReadOpDynamic = proc.cores[0]->ifu->BPT->RAS->local_result.power.readOp.dynamic;
	float ifuBPTRASLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->BPT->RAS->local_result.power.writeOp.dynamic;
	float ifuBTBLocal_resultPowerReadOpDynamic = proc.cores[0]->ifu->BTB->local_result.power.readOp.dynamic;
	float ifuBTBLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->BTB->local_result.power.writeOp.dynamic;
	float ifuIBLocal_resultPowerReadOpDynamic = proc.cores[0]->ifu->IB->local_result.power.readOp.dynamic;
	float ifuIBLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->IB->local_result.power.writeOp.dynamic;
	float ifuIDInstrt_powerReadOpDynamic = proc.cores[0]->ifu->ID_inst->power_t.readOp.dynamic;
	float ifuIDMisc_powerReadOpDynamic = proc.cores[0]->ifu->ID_misc->power_t.readOp.dynamic;
	float ifuIDOperand_powerReadOpDynamic = proc.cores[0]->ifu->ID_operand->power_t.readOp.dynamic;
	float ifuIcacheCachesLocal_resultPowerReadOpDynamic = proc.cores[0]->ifu->icache.caches->local_result.power.readOp.dynamic;
	float ifuIcacheCachesLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->icache.caches->local_result.power.writeOp.dynamic;
	float ifuIcacheIFBLocal_resultPowerSearchOpDynamic = proc.cores[0]->ifu->icache.ifb->local_result.power.searchOp.dynamic;
	float ifuIcacheIFBLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->icache.ifb->local_result.power.writeOp.dynamic;
	float ifuIcacheMissbLocal_resultPowerSearchOpDynamic = proc.cores[0]->ifu->icache.missb->local_result.power.searchOp.dynamic;
	float ifuIcacheMissbLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->icache.missb->local_result.power.writeOp.dynamic;
	float ifuIcachePrefetchBLocal_resultPowerSearchOpDynamic = proc.cores[0]->ifu->icache.prefetchb->local_result.power.searchOp.dynamic;
	float ifuIcachePrefetchBLocal_resultPowerWriteOpDynamic = proc.cores[0]->ifu->icache.prefetchb->local_result.power.writeOp.dynamic;
	float instruction_selectionPowerReadOpDynamic = 0;//proc.cores[0]->exu->scheu->instruction_selection->power.readOp.dynamic;
	float intTagBypassPowerReadOpDynamic = proc.cores[0]->exu->intTagBypass->power.readOp.dynamic;
	float intTag_mul_BypassPowerReadOpDynamic = proc.cores[0]->exu->intTag_mul_Bypass->power.readOp.dynamic;
	float int_bypassPowerReadOpDynamic = proc.cores[0]->exu->int_bypass->power.readOp.dynamic;
	float int_inst_windowLocal_resultPowerReadOpDynamic = 0;//proc.cores[0]->exu->scheu->int_inst_window->local_result.power.readOp.dynamic;
	float int_inst_windowLocal_resultPowerSearchOpDynamic = 0;// proc.cores[0]->exu->scheu->int_inst_window->local_result.power.searchOp.dynamic;
	float int_inst_windowLocal_resultPowerWriteOpDynamic = 0;//proc.cores[0]->exu->scheu->int_inst_window->local_result.power.writeOp.dynamic;
	float int_mul_bypassPowerReadOpDynamic = proc.cores[0]->exu->int_mul_bypass->power.readOp.dynamic;
	float irfLocal_resultPowerReadOpDynamic = proc.cores[0]->exu->rfu->IRF->local_result.power.readOp.dynamic;
	float irfLocal_resultPowerWriteOpDynamic = proc.cores[0]->exu->rfu->IRF->local_result.power.writeOp.dynamic;
	float itlbLocal_resultPowerSearchOpDynamic = proc.cores[0]->mmu->itlb->local_result.power.searchOp.dynamic;
	float itlbLocal_resultPowerWriteOpDynamic = proc.cores[0]->mmu->itlb->local_result.power.writeOp.dynamic;
	float mulPerAccessEnergy = proc.cores[0]->exu->mul->per_access_energy;
	float mulSckRation = proc.cores[0]->exu->mul->getG_tpSckt_co_eff();
	float mulbase_energy = proc.cores[0]->exu->mul->base_energy;
	float sckRation = proc.cores[0]->exu->exeu->getG_tpSckt_co_eff();
	float unicacheCachesLocal_resultPowerSearchOpDynamic = 0.0 /*l1dirarray->unicache.caches->local_result.power.searchOp.dynamic*/;
	float unicacheCachesLocal_resultPowerWriteOpDynamic = 0.0 /*l1dirarray->unicache.caches->local_result.power.writeOp.dynamic*/;

	//Defining another pointer to the dataIn for easy transformation to DFE version
	float* inVector = dataIn;

	//Define place holders for component RT-DP computation
	float ifuIcacheCachesStats_tReadAcAccess,
			ifuIcacheCachesStats_tReadAcMiss, ifuIBStats_tReadAcAccess,
			ifuIBStats_tWriteAcAccess, ifuBTBStats_tReadAcAccess,
			ifuBTBStats_tWriteAcAccess, ifuID_Inst_Operand_MiscStats_tReadAcAccess,
			ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tReadAcAccess,
			ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccess,
			ifuBPTRASStats_tReadWriteAcAccess, coredynpPipeline_duty_cycle,
			coredynpIFU_duty_cycle, XMLSysTotalCycles, XMLSysNumber_of_cores,
			coredynpNumPipelines, executionTime, ifuIcacheCachesStats_tReadAcHit,
			ifuIcachePower_tReadOpDynamic, ifuBTBPower_tReadOpDynamic,
			ifuIBPower_tReadOpDynamic,
			ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccessCorrection,
			ifuBPTGlobalBPTPower_tReadOpDynamic, ifuBPTL1_localBPTPower_tReadOpDynamic,
			ifuBPTL2_localBPTPower_tReadOpDynamic, ifuBPTChooserPower_tReadOpDynamic,
			ifuBPTRASPower_tReadOpDynamic, ifuBPTPower_tReadOpDynamic,
			ifuIDPower_tReadOpDynamic, rtp_pipeline_coe, num_units, powerFactor,
			ifuRTReadOpDynamicPower, dcacheCachesStats_tReadAcAccess,
			dcacheCachesStats_tReadAcMiss, dcacheCachesStats_tReadAcHit,
			dcacheCachesStats_tWriteAcAccess, dcacheCachesStats_tWriteAcMiss,
			xmlSysCoreLoadInstructions, xmlSysCoreStoreInstructions,
			lsqStats_tReadWriteAcAccess, dcachePower_tReadOpDynamic,
			lsqPower_tReadOpDynamic, coredynpLSU_duty_cycle, lsuRtp_pipeline_coe,
			lsuPowerFactor, lsuRTReadOpDynamicPower, itlbStats_tReadAcAccess,
			itlbStats_tReadAcMiss, dtlbStats_tReadAcAccess, dtlbStats_tReadAcMiss,
			itlbPower_tReadOpDynamic, dtlbPower_tReadOpDynamic, mmuRtp_pipeline_coe,
			mmuPowerFactor, mmuRTReadOpDynamicPower, irfStats_tReadAcAccess,
			irfStats_tWriteAcAccess, frfStats_tReadAcAccess, frfStats_tWriteAcAccess,
			irfPower_tReadOpDynamic, frfPower_tReadOpDynamic, rfuRTReadOpDynamicPower,
			int_inst_windowStats_tReadAcAccess, int_inst_windowStats_tWriteAcAccess,
			int_inst_windowStats_tSearchAcAccess, fp_inst_windowStats_tReadAcAccess,
			fp_inst_windowStats_tWriteAcAccess, fp_inst_windowStats_tSearchAcAccess,
			int_inst_windowPower_tReadOpDynamic, fp_inst_windowPower_tReadOpDynamic,
			sheuRTReadOpDynamicPower, exuStats_tReadAcAccess, exuRt_powerReadOpDynamic,
			fpuStats_tReadAcAccess, fpuRt_powerReadOpDynamic, mulStats_tReadAcAccess,
			mulRt_powerReadOpDynamic, cdbALUAccess, bypassRt_powerReadOpDynamic,
			cdbMULAccess, cdbfpuAccess, coredynpALU_duty_cycle, exuRtp_pipeline_coe,
			exuPowerFactor, exuRTReadOpDynamicPower, iFRATStats_tReadAcAccess,
			iFRATStats_tWriteAcAccess, fFRATStats_tReadAcAccess, fFRATStats_tWriteAcAccess,
			ifreeLStats_tReadAcAccess, ifreeLStats_tWriteAcAccess,
			ffreeLStats_tReadAcAccess, ffreeLStats_tWriteAcAccess,
			iFRATPower_tReadOpDynamic, fFRATPower_tReadOpDynamic,
			ifreeLPower_tReadOpDynamic, ffreeLPower_tReadOpDynamic, rnuPowerFactor,
			rnuRTReadOpDynamicPower, unicacheCachesStats_tReadAcAccess,
			unicacheCachesStats_tWriteAcAccess, unicachePower_tReadOpDynamic,
			busStats_tReadAcAccess, BusRTPower, coreRTPower, processorRTPower;

	cout << "Starting batch RT-DP Computation for batch Size = " << batchSize << endl;
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

	//Batch RT-DP computation
	for (int i=0; i< batchSize; i++){

		///IFU
		////ICache
		ifuIcacheCachesStats_tReadAcAccess = inVector[0 + i*inVectorSize];
		ifuIcacheCachesStats_tReadAcMiss = inVector[1 + i*inVectorSize];
		////IB
		ifuIBStats_tReadAcAccess = inVector[2 + i*inVectorSize];
		ifuIBStats_tWriteAcAccess = inVector[3 + i*inVectorSize];
		////BTB
		ifuBTBStats_tReadAcAccess = inVector[4 + i*inVectorSize];
		ifuBTBStats_tWriteAcAccess = inVector[5 + i*inVectorSize];
		////ID
		ifuID_Inst_Operand_MiscStats_tReadAcAccess = inVector[6 + i*inVectorSize];
		////BPT
		ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tReadAcAccess = inVector[7 + i*inVectorSize];
		ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccess  = inVector[8 + i*inVectorSize];
		ifuBPTRASStats_tReadWriteAcAccess = inVector[9 + i*inVectorSize];
		///Local power result
		////Icache

		//Core Pipeline
		coredynpPipeline_duty_cycle = inVector[10 + i*inVectorSize];
		coredynpIFU_duty_cycle = inVector[11 + i*inVectorSize];
		XMLSysTotalCycles = inVector[12 + i*inVectorSize];
		XMLSysNumber_of_cores = inVector[13 + i*inVectorSize];
		coredynpNumPipelines = inVector[14 + i*inVectorSize];

		//Execution Time
		executionTime = inVector[50 + i*inVectorSize];

		///Power Calculation
		ifuIcacheCachesStats_tReadAcHit = ifuIcacheCachesStats_tReadAcAccess - ifuIcacheCachesStats_tReadAcMiss;
		///Icache

		//optimization.pushPipeliningFactor(0.75);
		ifuIcachePower_tReadOpDynamic = ifuIcacheCachesStats_tReadAcHit * ifuIcacheCachesLocal_resultPowerReadOpDynamic +
				ifuIcacheCachesStats_tReadAcMiss * (ifuIcacheCachesLocal_resultPowerReadOpDynamic + //assume tag data accessed in parallel
						ifuIcacheCachesLocal_resultPowerWriteOpDynamic +//read miss in Icache cause a write to Icache
						ifuIcacheMissbLocal_resultPowerSearchOpDynamic +
						ifuIcacheMissbLocal_resultPowerWriteOpDynamic +
						ifuIcacheIFBLocal_resultPowerSearchOpDynamic +
						ifuIcacheIFBLocal_resultPowerWriteOpDynamic +
						ifuIcachePrefetchBLocal_resultPowerSearchOpDynamic +
						ifuIcachePrefetchBLocal_resultPowerWriteOpDynamic );

		///BTB
		ifuBTBPower_tReadOpDynamic =  ifuBTBStats_tReadAcAccess * ifuBTBLocal_resultPowerReadOpDynamic +
				ifuBTBStats_tWriteAcAccess * ifuBTBLocal_resultPowerWriteOpDynamic;
		///IB
		ifuIBPower_tReadOpDynamic = ifuIBLocal_resultPowerReadOpDynamic * ifuIBStats_tReadAcAccess +
				ifuIBLocal_resultPowerWriteOpDynamic * ifuIBStats_tWriteAcAccess;
		///BPT
		ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccessCorrection = ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccess +
				ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tReadAcAccess * 0.1; //10% of BR will flip internal bits//0

		ifuBPTGlobalBPTPower_tReadOpDynamic = ifuBPTGlobalBPTLocal_resultPowerReadOpDynamic*ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tReadAcAccess +
				ifuBPTGlobalBPTLocal_resultPowerWriteOpDynamic * ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccessCorrection;

		ifuBPTL1_localBPTPower_tReadOpDynamic =  ifuBPTL1LocalBPTLocal_resultPowerReadOpDynamic * ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tReadAcAccess +
				ifuBPTL1LocalBPTLocal_resultPowerWriteOpDynamic * ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccessCorrection;

		ifuBPTL2_localBPTPower_tReadOpDynamic =  ifuBPTL2LocalBPTLocal_resultPowerReadOpDynamic * ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tReadAcAccess +
				ifuBPTL2LocalBPTLocal_resultPowerWriteOpDynamic * ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccessCorrection;

		ifuBPTChooserPower_tReadOpDynamic =  ifuBPTChooserLocal_resultPowerReadOpDynamic * ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tReadAcAccess +
				ifuBPTChooserLocal_resultPowerWriteOpDynamic * ifuBPTGlobalBPTL1L2LocalBPTChooserStats_tWriteAcAccessCorrection;

		ifuBPTRASPower_tReadOpDynamic =  ifuBPTRASLocal_resultPowerReadOpDynamic * ifuBPTRASStats_tReadWriteAcAccess +
				ifuBPTRASLocal_resultPowerWriteOpDynamic * ifuBPTRASStats_tReadWriteAcAccess;

		ifuBPTPower_tReadOpDynamic = ifuBPTGlobalBPTPower_tReadOpDynamic +
				ifuBPTL1_localBPTPower_tReadOpDynamic + ifuBPTL2_localBPTPower_tReadOpDynamic +
				ifuBPTChooserPower_tReadOpDynamic + ifuBPTRASPower_tReadOpDynamic;
		///ID
		ifuIDPower_tReadOpDynamic =  (ifuIDInstrt_powerReadOpDynamic +
				ifuIDOperand_powerReadOpDynamic +
				ifuIDMisc_powerReadOpDynamic) * ifuID_Inst_Operand_MiscStats_tReadAcAccess;

		// Assume XML->sys.homogeneous_cores==1 for ARM 2Hz Template
		rtp_pipeline_coe = coredynpPipeline_duty_cycle * coredynpIFU_duty_cycle	*
				XMLSysTotalCycles*
				XMLSysNumber_of_cores;
		num_units = 4; // ARM atomic simple CPU is INORDER; Defined in McPAT Core::computeEnergy(bool is_tdp) : double num_units = 4.0;
		powerFactor = coredynpNumPipelines*rtp_pipeline_coe/num_units;

		ifuRTReadOpDynamicPower = ifuIcachePower_tReadOpDynamic +
				ifuBTBPower_tReadOpDynamic +
				ifuIBPower_tReadOpDynamic +
				ifuBPTPower_tReadOpDynamic +
				ifuIDPower_tReadOpDynamic +
				corepipePowerReadOpDynamic*powerFactor;

		//LSU
		//init stats for Runtime Dynamic (RTP)
		dcacheCachesStats_tReadAcAccess = inVector[15 + i*inVectorSize];
		dcacheCachesStats_tReadAcMiss = inVector[16 + i*inVectorSize];
		dcacheCachesStats_tReadAcHit = dcacheCachesStats_tReadAcAccess - dcacheCachesStats_tReadAcMiss;

		dcacheCachesStats_tWriteAcAccess = inVector[17 + i*inVectorSize];

		dcacheCachesStats_tWriteAcMiss = inVector[18 + i*inVectorSize];
		xmlSysCoreLoadInstructions = inVector[19 + i*inVectorSize];
		xmlSysCoreStoreInstructions = inVector[20 + i*inVectorSize];

		lsqStats_tReadWriteAcAccess = (xmlSysCoreLoadInstructions + xmlSysCoreStoreInstructions) * 2;

		dcachePower_tReadOpDynamic = dcacheCachesStats_tReadAcHit*dcacheCachesLocal_resultPowerReadOpDynamic+
				dcacheCachesStats_tReadAcMiss * dcacheCachesLocal_resultPowerReadOpDynamic +
				dcacheCachesStats_tWriteAcMiss * dcacheCachesLocal_resultTag_array2PowerReadOpDynamic +
				dcacheCachesStats_tWriteAcAccess * dcacheCachesLocal_resultPowerWriteOpDynamic +
				dcacheCachesStats_tWriteAcMiss * dcacheCachesLocal_resultPowerWriteOpDynamic +
				dcacheCachesStats_tWriteAcMiss *
				(dcacheMissbLocal_resultPowerSearchOpDynamic +
						dcacheMissbLocal_resultPowerWriteOpDynamic +
						dcacheIFBLocal_resultPowerSearchOpDynamic +
						dcacheIFBLocal_resultPowerWriteOpDynamic +
						dcachePrefetchBLocal_resultPowerSearchOpDynamic +
						dcachePrefetchBLocal_resultPowerWriteOpDynamic +
						dcacheWBBLocal_resultPowerSearchOpDynamic +
						dcacheWBBLocal_resultPowerWriteOpDynamic);

		lsqPower_tReadOpDynamic = lsqStats_tReadWriteAcAccess * (dcacheLSQLocal_resultPowerSearchOpDynamic +
				dcacheLSQLocal_resultPowerReadOpDynamic+
				dcacheLSQLocal_resultPowerWriteOpDynamic);

		coredynpLSU_duty_cycle = inVector[21 + i*inVectorSize];

		// Assume XML->sys.homogeneous_cores==1 for ARM 2Hz Template
		lsuRtp_pipeline_coe = coredynpPipeline_duty_cycle * coredynpLSU_duty_cycle *
				XMLSysTotalCycles*
				XMLSysNumber_of_cores;
		lsuPowerFactor = coredynpNumPipelines*lsuRtp_pipeline_coe/num_units;

		lsuRTReadOpDynamicPower = dcachePower_tReadOpDynamic + lsqPower_tReadOpDynamic +
				corepipePowerReadOpDynamic*lsuPowerFactor;

		//************************************************
		//MMU
		//init stats for Runtime Dynamic (RTP)
		itlbStats_tReadAcAccess  = inVector[22 + i*inVectorSize];
		itlbStats_tReadAcMiss  = inVector[23 + i*inVectorSize];

		dtlbStats_tReadAcAccess  = inVector[24 + i*inVectorSize];
		dtlbStats_tReadAcMiss  = inVector[25 + i*inVectorSize];

		itlbPower_tReadOpDynamic = itlbStats_tReadAcAccess * itlbLocal_resultPowerSearchOpDynamic +
				itlbStats_tReadAcMiss * itlbLocal_resultPowerWriteOpDynamic;

		dtlbPower_tReadOpDynamic = dtlbStats_tReadAcAccess * dtlbLocal_resultPowerSearchOpDynamic +
				dtlbStats_tReadAcMiss * dtlbLocal_resultPowerWriteOpDynamic;

		// Assume XML->sys.homogeneous_cores==1 for ARM 2Hz Template
		mmuRtp_pipeline_coe = coredynpPipeline_duty_cycle *
				(0.5 + 0.5 * coredynpLSU_duty_cycle) *
				XMLSysTotalCycles*
				XMLSysNumber_of_cores;
		mmuPowerFactor = coredynpNumPipelines*mmuRtp_pipeline_coe/num_units;

		mmuRTReadOpDynamicPower= itlbPower_tReadOpDynamic + dtlbPower_tReadOpDynamic
				+ corepipePowerReadOpDynamic*mmuPowerFactor;;


		//************************************************
		//EXU

		///RFU (No register window)
		//init stats for Runtime Dynamic (RTP)

		irfStats_tReadAcAccess  = inVector[26 + i*inVectorSize];
		irfStats_tWriteAcAccess  = inVector[27 + i*inVectorSize];

		frfStats_tReadAcAccess  = inVector[28 + i*inVectorSize];
		frfStats_tWriteAcAccess  = inVector[29 + i*inVectorSize];


		irfPower_tReadOpDynamic = irfStats_tReadAcAccess * irfLocal_resultPowerReadOpDynamic +
				irfStats_tWriteAcAccess * irfLocal_resultPowerWriteOpDynamic;


		frfPower_tReadOpDynamic = frfStats_tReadAcAccess * frfLocal_resultPowerReadOpDynamic +
				frfStats_tWriteAcAccess * frfLocal_resultPowerWriteOpDynamic;

		rfuRTReadOpDynamicPower = irfPower_tReadOpDynamic + frfPower_tReadOpDynamic;

		///SHEU
		int_inst_windowStats_tReadAcAccess  = inVector[30 + i*inVectorSize]; // XML->sys.core[ithCore].inst_window_reads;
		int_inst_windowStats_tWriteAcAccess  = inVector[31 + i*inVectorSize];// XML->sys.core[ithCore].inst_window_writes;
		int_inst_windowStats_tSearchAcAccess  = inVector[32 + i*inVectorSize];// XML->sys.core[ithCore].inst_window_wakeup_accesses;
		fp_inst_windowStats_tReadAcAccess  = inVector[33 + i*inVectorSize]; // XML->sys.core[ithCore].fp_inst_window_reads;
		fp_inst_windowStats_tWriteAcAccess  = inVector[34 + i*inVectorSize];//  XML->sys.core[ithCore].fp_inst_window_writes;
		fp_inst_windowStats_tSearchAcAccess  = inVector[35 + i*inVectorSize];// XML->sys.core[ithCore].fp_inst_window_wakeup_accesses;

		int_inst_windowPower_tReadOpDynamic  =
				int_inst_windowLocal_resultPowerReadOpDynamic * int_inst_windowStats_tReadAcAccess
				+ int_inst_windowLocal_resultPowerSearchOpDynamic * int_inst_windowStats_tSearchAcAccess
				+ int_inst_windowLocal_resultPowerWriteOpDynamic  * int_inst_windowStats_tWriteAcAccess
				+ instruction_selectionPowerReadOpDynamic * int_inst_windowStats_tReadAcAccess;

		fp_inst_windowPower_tReadOpDynamic  =
				fp_inst_windowLocal_resultPowerReadOpDynamic * fp_inst_windowStats_tReadAcAccess
				+ fp_inst_windowLocal_resultPowerSearchOpDynamic * fp_inst_windowStats_tSearchAcAccess
				+ fp_inst_windowLocal_resultPowerWriteOpDynamic  * fp_inst_windowStats_tWriteAcAccess
				+ instruction_selectionPowerReadOpDynamic * fp_inst_windowStats_tWriteAcAccess;

		sheuRTReadOpDynamicPower  = int_inst_windowPower_tReadOpDynamic + fp_inst_windowPower_tReadOpDynamic;

		///EXEU
		exuStats_tReadAcAccess = inVector[36 + i*inVectorSize];//XML->sys.core[ithCore].ialu_accesses;
		exuRt_powerReadOpDynamic = (exuPerAccessEnergy*exuStats_tReadAcAccess + exubase_energy*executionTime)*sckRation;

		///FPU
		fpuStats_tReadAcAccess = inVector[37 + i*inVectorSize];//XML->sys.core[ithCore].fpu_accesses;

		fpuRt_powerReadOpDynamic =
				(fpuPerAccessEnergy*fpuStats_tReadAcAccess
						+ fpubase_energy*executionTime)*fpuSckRation; //fp_u->

		//MUL
		mulStats_tReadAcAccess = inVector[38 + i*inVectorSize];//XML->sys.core[ithCore].mul_accesses;

		mulRt_powerReadOpDynamic =
				(mulPerAccessEnergy*mulStats_tReadAcAccess
						+ mulbase_energy*executionTime)*mulSckRation; //mul->


		///bypass
		cdbALUAccess = inVector[39 + i*inVectorSize]; //XML->sys.core[ithCore].cdb_alu_accesses

		bypassRt_powerReadOpDynamic = (intTagBypassPowerReadOpDynamic + int_bypassPowerReadOpDynamic) * cdbALUAccess;


		cdbMULAccess = inVector[40 + i*inVectorSize]; //XML->sys.core[ithCore].cdb_mul_accesses


		bypassRt_powerReadOpDynamic = bypassRt_powerReadOpDynamic +
				(intTag_mul_BypassPowerReadOpDynamic + int_mul_bypassPowerReadOpDynamic) * cdbMULAccess;


		cdbfpuAccess = inVector[41 + i*inVectorSize]; //XML->sys.core[ithCore].cdb_fpu_accesses

		bypassRt_powerReadOpDynamic = bypassRt_powerReadOpDynamic +
				(fp_BypassPowerReadOpDynamic + fpTagBypassPowerReadOpDynamic) * cdbfpuAccess;

		// Assume XML->sys.homogeneous_cores==1 for ARM 2Hz Template
		coredynpALU_duty_cycle = inVector[42 + i*inVectorSize]; // coredynp.ALU_duty_cycle

		exuRtp_pipeline_coe = coredynpPipeline_duty_cycle *
				coredynpALU_duty_cycle *
				XMLSysTotalCycles*
				XMLSysNumber_of_cores;

		exuPowerFactor = coredynpNumPipelines*exuRtp_pipeline_coe/num_units;

		exuRTReadOpDynamicPower = rfuRTReadOpDynamicPower +
				sheuRTReadOpDynamicPower +
				exuRt_powerReadOpDynamic +
				fpuRt_powerReadOpDynamic+
				mulRt_powerReadOpDynamic +
				bypassRt_powerReadOpDynamic +
				corepipePowerReadOpDynamic*exuPowerFactor;

		//RNU
		//init stats for Runtime Dynamic (RTP)

		// Assume coredynp.core_ty==OOO, coredynp.scheu_ty==PhysicalRegFile (instruction_window_scheme), coredynp.rm_ty ==RAMbased (rename_scheme)
		iFRATStats_tReadAcAccess  = inVector[43 + i*inVectorSize]; // XML->sys.core[ithCore].rename_reads;
		iFRATStats_tWriteAcAccess  = inVector[44 + i*inVectorSize];// XML->sys.core[ithCore].rename_writes;


		fFRATStats_tReadAcAccess  = inVector[45 + i*inVectorSize]; // XML->sys.core[ithCore].fp_rename_reads;
		fFRATStats_tWriteAcAccess  = inVector[46 + i*inVectorSize];// XML->sys.core[ithCore].fp_rename_writes;

		ifreeLStats_tReadAcAccess  = iFRATStats_tReadAcAccess; // XML->sys.core[ithCore].fp_rename_reads;
		ifreeLStats_tWriteAcAccess  = 2*iFRATStats_tWriteAcAccess;// XML->sys.core[ithCore].fp_rename_writes;

		ffreeLStats_tReadAcAccess  = fFRATStats_tReadAcAccess;
		ffreeLStats_tWriteAcAccess  = 2*fFRATStats_tWriteAcAccess;


		//coredynpdecodeW = inVector[117 + i*inVectorSize]; // coredynp.decodeW
		//idclLStats_tReadAcAccess  = 3* coredynpdecodeW* coredynpdecodeW * iFRATStats_tReadAcAccess;

		//coredynpfp_issueW = inVector[118 + i*inVectorSize]; // coredynp.fp_issueW
		//fdclLStats_tReadAcAccess  = 3* coredynpfp_issueW * coredynpfp_issueW * fFRATStats_tWriteAcAccess;

		iFRATPower_tReadOpDynamic  = iFRATStats_tReadAcAccess * (iFRATLocal_resultPowerReadOpDynamic
				+ idclPowerReadOpDynamic)
				+iFRATStats_tWriteAcAccess*iFRATLocal_resultPowerWriteOpDynamic;


		fFRATPower_tReadOpDynamic  = fFRATStats_tReadAcAccess *
				(fFRATLocal_resultPowerReadOpDynamic + fdclPowerReadOpDynamic) +
				fFRATStats_tWriteAcAccess * fFRATLocal_resultPowerWriteOpDynamic ;

		ifreeLPower_tReadOpDynamic = ifreeLStats_tReadAcAccess * ifreeLLocal_resultPowerReadOpDynamic +
				ifreeLStats_tWriteAcAccess * ifreeLLocal_resultPowerWriteOpDynamic;


		ffreeLPower_tReadOpDynamic = ffreeLStats_tReadAcAccess * ffreeLLocal_resultPowerReadOpDynamic +
				ffreeLStats_tWriteAcAccess * ffreeLLocal_resultPowerWriteOpDynamic;

		rnuPowerFactor = coredynpPipeline_duty_cycle *
				XMLSysTotalCycles*
				XMLSysNumber_of_cores/num_units;

		rnuRTReadOpDynamicPower = iFRATPower_tReadOpDynamic +
				fFRATPower_tReadOpDynamic +
				ifreeLPower_tReadOpDynamic+
				ffreeLPower_tReadOpDynamic; /* +
            	rnuPowerFactor*corepipePowerReadOpDynamic;*/ // Removing as ARM atomic simple CPU is INORDER

		//First Level Directory TODO x2
		unicacheCachesStats_tReadAcAccess  = inVector[47 + i*inVectorSize]; // XML->sys.L1Directory[ithCache].read_accesses;
		unicacheCachesStats_tWriteAcAccess  = inVector[48 + i*inVectorSize]; // XML->sys.L1Directory[ithCache].write_accesses;
		unicachePower_tReadOpDynamic	= unicacheCachesStats_tReadAcAccess*unicacheCachesLocal_resultPowerSearchOpDynamic+
				unicacheCachesStats_tWriteAcAccess*unicacheCachesLocal_resultPowerWriteOpDynamic;

		//NOC - Type BUS (0)
		busStats_tReadAcAccess  = inVector[49 + i*inVectorSize]; // XML->sys.NoC[ithNoC].total_accesses;
		BusRTPower = busStats_tReadAcAccess * busBasePower;

		coreRTPower = ifuRTReadOpDynamicPower +
				rnuRTReadOpDynamicPower +
				lsuRTReadOpDynamicPower+
				mmuRTReadOpDynamicPower +
				exuRTReadOpDynamicPower;

		processorRTPower = coreRTPower +
				unicachePower_tReadOpDynamic +
				BusRTPower;

		dataOut[0 + i * outVectorSize] =ifuIcachePower_tReadOpDynamic/executionTime;
		dataOut[1 + i * outVectorSize] =ifuBTBPower_tReadOpDynamic/executionTime;
		dataOut[2 + i * outVectorSize] =ifuIBPower_tReadOpDynamic/executionTime;
		dataOut[3 + i * outVectorSize] =ifuBPTGlobalBPTPower_tReadOpDynamic/executionTime;
		dataOut[4 + i * outVectorSize] =ifuBPTL1_localBPTPower_tReadOpDynamic/executionTime;
		dataOut[5 + i * outVectorSize] =ifuBPTL2_localBPTPower_tReadOpDynamic/executionTime;
		dataOut[6 + i * outVectorSize] =ifuBPTChooserPower_tReadOpDynamic/executionTime;
		dataOut[7 + i * outVectorSize] =ifuBPTRASPower_tReadOpDynamic/executionTime;
		dataOut[8 + i * outVectorSize] =ifuBPTPower_tReadOpDynamic/executionTime;
		dataOut[9 + i * outVectorSize] =ifuIDPower_tReadOpDynamic/executionTime;
		dataOut[10 + i * outVectorSize] =ifuRTReadOpDynamicPower/executionTime;
		dataOut[11 + i * outVectorSize] = lsuRTReadOpDynamicPower/executionTime;
		dataOut[12 + i * outVectorSize] = dcachePower_tReadOpDynamic/executionTime;
		dataOut[13 + i * outVectorSize] = lsqPower_tReadOpDynamic/executionTime;
		dataOut[14 + i * outVectorSize] = itlbPower_tReadOpDynamic/executionTime;
		dataOut[15 + i * outVectorSize] = dtlbPower_tReadOpDynamic/executionTime;
		dataOut[16 + i * outVectorSize] = mmuRTReadOpDynamicPower/executionTime;
		dataOut[17 + i * outVectorSize] = exuRTReadOpDynamicPower/executionTime;
		dataOut[18 + i * outVectorSize] =rfuRTReadOpDynamicPower/executionTime;
		dataOut[19 + i * outVectorSize] =irfPower_tReadOpDynamic/executionTime;
		dataOut[20 + i * outVectorSize] =frfPower_tReadOpDynamic/executionTime;
		dataOut[21 + i * outVectorSize] =sheuRTReadOpDynamicPower/executionTime;
		dataOut[22 + i * outVectorSize] =int_inst_windowPower_tReadOpDynamic/executionTime;
		dataOut[23 + i * outVectorSize] =fp_inst_windowPower_tReadOpDynamic/executionTime;
		dataOut[24 + i * outVectorSize] =exuRt_powerReadOpDynamic/executionTime;
		dataOut[25 + i * outVectorSize] =fpuRt_powerReadOpDynamic/executionTime;
		dataOut[26 + i * outVectorSize] =mulRt_powerReadOpDynamic/executionTime;
		dataOut[27 + i * outVectorSize] =bypassRt_powerReadOpDynamic/executionTime;
		dataOut[28 + i * outVectorSize] =iFRATPower_tReadOpDynamic/executionTime;
		dataOut[29 + i * outVectorSize] =fFRATPower_tReadOpDynamic/executionTime;
		dataOut[30 + i * outVectorSize] =ifreeLPower_tReadOpDynamic/executionTime;
		dataOut[31 + i * outVectorSize] =ffreeLPower_tReadOpDynamic/executionTime;
		dataOut[32 + i * outVectorSize] =rnuRTReadOpDynamicPower/executionTime;
		dataOut[33 + i * outVectorSize] = unicachePower_tReadOpDynamic/executionTime;
		dataOut[34 + i * outVectorSize] = BusRTPower/executionTime;
		dataOut[35 + i * outVectorSize] = coreRTPower/executionTime;
		dataOut[36 + i * outVectorSize] = processorRTPower/executionTime;
		dataOut[37 + i * outVectorSize] = 0;
		dataOut[38 + i * outVectorSize] = 0;
		dataOut[39 + i * outVectorSize] = 0;
		dataOut[40 + i * outVectorSize] = 0;
		dataOut[41 + i * outVectorSize] = 0;
		dataOut[42 + i * outVectorSize] = 0;
		dataOut[43 + i * outVectorSize] = 0;
		dataOut[44 + i * outVectorSize] = 0;
		dataOut[45 + i * outVectorSize] = 0;
		dataOut[46 + i * outVectorSize] = 0;
		dataOut[47 + i * outVectorSize] = 0;
	}

	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
	cout.precision(12);
	if(is_debug) cout << "Batch RT Dynamic Power Computation time = " << (double) duration/1E9 <<"s" <<endl;

}



