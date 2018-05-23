#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdbool.h>
#include "llsim.h"

#define sp_printf(a...)						\
	do {							\
		llsim_printf("sp: clock %d: ", llsim->clock);	\
		llsim_printf(a);				\
	} while (0)

int nr_simulated_instructions = 0;
FILE *inst_trace_fp = NULL, *cycle_trace_fp = NULL;

typedef struct sp_registers_s {
	// 6 32 bit registers (r[0], r[1] don't exist)
	int r[8];

	// 32 bit cycle counter
	int cycle_counter;

	// fetch0
	int fetch0_active; // 1 bit
	int fetch0_pc; // 16 bits

	// fetch1
	int fetch1_active; // 1 bit
	int fetch1_pc; // 16 bits

	// dec0
	int dec0_active; // 1 bit
	int dec0_pc; // 16 bits
	int dec0_inst; // 32 bits

	// dec1
	int dec1_active; // 1 bit
	int dec1_pc; // 16 bits
	int dec1_inst; // 32 bits
	int dec1_opcode; // 5 bits
	int dec1_src0; // 3 bits
	int dec1_src1; // 3 bits
	int dec1_dst; // 3 bits
	int dec1_immediate; // 32 bits

	// exec0
	int exec0_active; // 1 bit
	int exec0_pc; // 16 bits
	int exec0_inst; // 32 bits
	int exec0_opcode; // 5 bits
	int exec0_src0; // 3 bits
	int exec0_src1; // 3 bits
	int exec0_dst; // 3 bits
	int exec0_immediate; // 32 bits
	int exec0_alu0; // 32 bits
	int exec0_alu1; // 32 bits

	// exec1
	int exec1_active; // 1 bit
	int exec1_pc; // 16 bits
	int exec1_inst; // 32 bits
	int exec1_opcode; // 5 bits
	int exec1_src0; // 3 bits
	int exec1_src1; // 3 bits
	int exec1_dst; // 3 bits
	int exec1_immediate; // 32 bits
	int exec1_alu0; // 32 bits
	int exec1_alu1; // 32 bits
	int exec1_aluout;
} sp_registers_t;

/*
 * Master structure
 */
typedef struct sp_s {
	// local srams
#define SP_SRAM_HEIGHT	64 * 1024
	llsim_memory_t *srami, *sramd;

	unsigned int memory_image[SP_SRAM_HEIGHT];
	int memory_image_size;

	int start;

	sp_registers_t *spro, *sprn;
} sp_t;

static void sp_reset(sp_t *sp)
{
	sp_registers_t *sprn = sp->sprn;

	memset(sprn, 0, sizeof(*sprn));
}

/*
 * opcodes
 */
#define ADD 0
#define SUB 1
#define LSF 2
#define RSF 3
#define AND 4
#define OR  5
#define XOR 6
#define LHI 7
#define LD 8
#define ST 9
#define JLT 16
#define JLE 17
#define JEQ 18
#define JNE 19
#define JIN 20
#define DMA 21
#define POL 22
#define HLT 24

 //DMA hardware
int dma_regs[5]; //registers serving the DMA functionality
bool read_into_reg3 = true;	//if false, read into reg4
bool write_reg3 = true;  //if false, write reg4's data
bool dma_opcode_received = false;

// 3 bit control state machine of DMA
int ctl_dma_state;

// control states
#define NO_READ_WRITE		0
#define ONE_READ_NO_WRITE	1
#define ONE_READ_ONE_WRITE	2
#define TWO_WRITE_READY		3
#define ONE_WRITE_READY		4
#define DMA_IDLE_STATE		5

//jump predictors
int jump_predictors[40];



static char opcode_name[32][4] = {"ADD", "SUB", "LSF", "RSF", "AND", "OR", "XOR", "LHI",
				 "LD", "ST", "U", "U", "U", "U", "U", "U",
				 "JLT", "JLE", "JEQ", "JNE", "JIN", "U", "U", "U",
				 "HLT", "U", "U", "U", "U", "U", "U", "U"};

static void dump_sram(sp_t *sp, char *name, llsim_memory_t *sram)
{
	FILE *fp;
	int i;

	fp = fopen(name, "w");
	if (fp == NULL) {
                printf("couldn't open file %s\n", name);
                exit(1);
	}
	for (i = 0; i < SP_SRAM_HEIGHT; i++)
		fprintf(fp, "%08x\n", llsim_mem_extract(sram, i, 31, 0));
	fclose(fp);
}

#define R0 (0)
#define NUM_OF_REGS (8)
#define MAX_STR_LEN (1024)
#define HALT_OPCODE (24)
#define SUCCESS (0)
#define FAIL (-1)
#define TRACE_FILE ("inst_trace.txt")
#define SRAM_OUT ("sram_out.txt")

typedef enum {
	inst_params_imm = 65535,        // 00000000000000001111111111111111
	inst_params_src1 = 458752,      // 00000000000001110000000000000000
	inst_params_src0 = 3670016,     // 00000000001110000000000000000000
	inst_params_dst = 29360128,     // 00000001110000000000000000000000
	inst_params_opcode = 1040187392 // 00111110000000000000000000000000
}inst_params;

typedef enum {
	inst_params_imm_shift = 0,        // 00000000000000001111111111111111
	inst_params_src1_shift = 16,      // 00000000000001110000000000000000
	inst_params_src0_shift = 19,      // 00000000001110000000000000000000
	inst_params_dst_shift = 22,       // 00000001110000000000000000000000
	inst_params_opcode_shift = 25     // 00111110000000000000000000000000
}inst_params_shift;

int data_extracted = 0;
int raw_hazard = 0;
int pc_of_last_inst_executed = -1;
int inst_fetched = 0;
//Functions we use for instruction traces
int end_trace(FILE* file, int cnt, int pc);
int print_line1(FILE* file, int cnt_of_inst, int pc_of_inst);
int print_line2(FILE* file, sp_registers_t* inst_regs);
int print_line3(FILE* file, sp_registers_t* inst_regs);
int print_line4(FILE* file, sp_registers_t* inst_regs);
int print_line5(FILE* file, sp_t* sp);
void print_all_lines(sp_t* sp, int pc_of_inst, int nr_sim_inst);
void init_dma_logic(int source, int dest, int amount);
void perform_dma_logic(bool mem_available, sp_t* sp);
int predict_jump(int current_pc);
void flush_pipeline(sp_registers_t** sprn_address);
bool validate_dma_values(int source, int dest, int amount);


static void sp_ctl(sp_t *sp)
{
	sp_registers_t *spro = sp->spro;
	sp_registers_t *sprn = sp->sprn;
	int i;

	fprintf(cycle_trace_fp, "cycle %d\n", spro->cycle_counter);
	fprintf(cycle_trace_fp, "cycle_counter %08x\n", spro->cycle_counter);
	for (i = 2; i <= 7; i++)
		fprintf(cycle_trace_fp, "r%d %08x\n", i, spro->r[i]);

	fprintf(cycle_trace_fp, "fetch0_active %08x\n", spro->fetch0_active);
	fprintf(cycle_trace_fp, "fetch0_pc %08x\n", spro->fetch0_pc);

	fprintf(cycle_trace_fp, "fetch1_active %08x\n", spro->fetch1_active);
	fprintf(cycle_trace_fp, "fetch1_pc %08x\n", spro->fetch1_pc);

	fprintf(cycle_trace_fp, "dec0_active %08x\n", spro->dec0_active);
	fprintf(cycle_trace_fp, "dec0_pc %08x\n", spro->dec0_pc);
	fprintf(cycle_trace_fp, "dec0_inst %08x\n", spro->dec0_inst); // 32 bits

	fprintf(cycle_trace_fp, "dec1_active %08x\n", spro->dec1_active);
	fprintf(cycle_trace_fp, "dec1_pc %08x\n", spro->dec1_pc); // 16 bits
	fprintf(cycle_trace_fp, "dec1_inst %08x\n", spro->dec1_inst); // 32 bits
	fprintf(cycle_trace_fp, "dec1_opcode %08x\n", spro->dec1_opcode); // 5 bits
	fprintf(cycle_trace_fp, "dec1_src0 %08x\n", spro->dec1_src0); // 3 bits
	fprintf(cycle_trace_fp, "dec1_src1 %08x\n", spro->dec1_src1); // 3 bits
	fprintf(cycle_trace_fp, "dec1_dst %08x\n", spro->dec1_dst); // 3 bits
	fprintf(cycle_trace_fp, "dec1_immediate %08x\n", spro->dec1_immediate); // 32 bits

	fprintf(cycle_trace_fp, "exec0_active %08x\n", spro->exec0_active);
	fprintf(cycle_trace_fp, "exec0_pc %08x\n", spro->exec0_pc); // 16 bits
	fprintf(cycle_trace_fp, "exec0_inst %08x\n", spro->exec0_inst); // 32 bits
	fprintf(cycle_trace_fp, "exec0_opcode %08x\n", spro->exec0_opcode); // 5 bits
	fprintf(cycle_trace_fp, "exec0_src0 %08x\n", spro->exec0_src0); // 3 bits
	fprintf(cycle_trace_fp, "exec0_src1 %08x\n", spro->exec0_src1); // 3 bits
	fprintf(cycle_trace_fp, "exec0_dst %08x\n", spro->exec0_dst); // 3 bits
	fprintf(cycle_trace_fp, "exec0_immediate %08x\n", spro->exec0_immediate); // 32 bits
	fprintf(cycle_trace_fp, "exec0_alu0 %08x\n", spro->exec0_alu0); // 32 bits
	fprintf(cycle_trace_fp, "exec0_alu1 %08x\n", spro->exec0_alu1); // 32 bits

	fprintf(cycle_trace_fp, "exec1_active %08x\n", spro->exec1_active);
	fprintf(cycle_trace_fp, "exec1_pc %08x\n", spro->exec1_pc); // 16 bits
	fprintf(cycle_trace_fp, "exec1_inst %08x\n", spro->exec1_inst); // 32 bits
	fprintf(cycle_trace_fp, "exec1_opcode %08x\n", spro->exec1_opcode); // 5 bits
	fprintf(cycle_trace_fp, "exec1_src0 %08x\n", spro->exec1_src0); // 3 bits
	fprintf(cycle_trace_fp, "exec1_src1 %08x\n", spro->exec1_src1); // 3 bits
	fprintf(cycle_trace_fp, "exec1_dst %08x\n", spro->exec1_dst); // 3 bits
	fprintf(cycle_trace_fp, "exec1_immediate %08x\n", spro->exec1_immediate); // 32 bits
	fprintf(cycle_trace_fp, "exec1_alu0 %08x\n", spro->exec1_alu0); // 32 bits
	fprintf(cycle_trace_fp, "exec1_alu1 %08x\n", spro->exec1_alu1); // 32 bits
	fprintf(cycle_trace_fp, "exec1_aluout %08x\n", spro->exec1_aluout);

	fprintf(cycle_trace_fp, "\n");

	sp_printf("cycle_counter %08x\n", spro->cycle_counter);
	sp_printf("r2 %08x, r3 %08x\n", spro->r[2], spro->r[3]);
	sp_printf("r4 %08x, r5 %08x, r6 %08x, r7 %08x\n", spro->r[4], spro->r[5], spro->r[6], spro->r[7]);
	sp_printf("fetch0_active %d, fetch1_active %d, dec0_active %d, dec1_active %d, exec0_active %d, exec1_active %d\n",
		  spro->fetch0_active, spro->fetch1_active, spro->dec0_active, spro->dec1_active, spro->exec0_active, spro->exec1_active);
	sp_printf("fetch0_pc %d, fetch1_pc %d, dec0_pc %d, dec1_pc %d, exec0_pc %d, exec1_pc %d\n",
		  spro->fetch0_pc, spro->fetch1_pc, spro->dec0_pc, spro->dec1_pc, spro->exec0_pc, spro->exec1_pc);

	sprn->cycle_counter = spro->cycle_counter + 1;

	if (sp->start)
		sprn->fetch0_active = 1;

	bool mem_available = true;
	// fetch0
	sprn->fetch1_active = 0;
	if (spro->fetch0_active) {
		if (raw_hazard == 0)
		{ 
			llsim_mem_read(sp->srami, spro->fetch0_pc);
			sprn->fetch1_pc = spro->fetch0_pc;
			sprn->fetch0_pc++;
		}
		sprn->fetch1_active = 1;
	}

	// fetch1
	sprn->dec0_active = 0;
	if (spro->fetch1_active) {
		if (raw_hazard == 0)
		{
			// When we fetch an instruction in fetch0, we must extract that instruction in fetch1, even if we got a hazard previouly
			if (!inst_fetched)
			{
				sprn->dec0_inst = llsim_mem_extract_dataout(sp->srami, 31, 0);
			}
			else
			{
				sprn->dec0_inst = inst_fetched;
				inst_fetched = 0;
			}
			sprn->dec0_pc = spro->fetch1_pc;
		}
		else
		{
			inst_fetched = llsim_mem_extract_dataout(sp->srami, 31, 0);
		}
		sprn->dec0_active = 1;
	}

	// dec0
	sprn->dec1_active = 0;
	if (spro->dec0_active) {
		if (raw_hazard == 0)
		{
			int opcode = (spro->dec0_inst & inst_params_opcode) >> inst_params_opcode_shift;
			sprn->dec1_opcode = opcode;
			short imm = spro->dec0_inst & inst_params_imm;
			sprn->dec1_immediate = (int)imm;
			sprn->dec1_src1 = (spro->dec0_inst & inst_params_src1) >> inst_params_src1_shift;
			sprn->dec1_src0 = (spro->dec0_inst & inst_params_src0) >> inst_params_src0_shift;
			sprn->dec1_dst = (spro->dec0_inst & inst_params_dst) >> inst_params_dst_shift;
			if (spro->dec1_opcode == LD && ((sprn->dec1_src0 == spro->dec1_dst) || (sprn->dec1_src1 == spro->dec1_dst)))
			{
				raw_hazard = 1;
			}

			sprn->dec1_pc = spro->dec0_pc;
			sprn->dec1_inst = spro->dec0_inst;

			//in case of jumping
			switch (opcode)
			{
				case JLT:
				case JLE:
				case JEQ:
				case JNE:
				case JIN:
					if (predict_jump(spro->dec0_pc))
					{
						sprn->fetch0_pc = (int)imm;
						sprn->r[7] = spro->exec1_pc; //TODO kosta mentioned to check no one overrides r[7], but this is user's responsibility no?
					}

			}
		}
		sprn->dec1_active = 1;
	}

	// dec1
	sprn->exec0_active = 0;
	if (spro->dec1_active) {
		if (spro->dec1_opcode == DMA && !dma_opcode_received && validate_dma_values(spro->r[spro->dec1_dst], spro->r[spro->dec1_src0], spro->dec1_immediate)) //in case DMA is already working, we ignore the new request
		{
			init_dma_logic(spro->r[spro->dec1_dst], spro->r[spro->dec1_src0], spro->dec1_immediate);
		}
		else if (raw_hazard == 0 || (spro->dec1_opcode == LD))
		{
			if (spro->dec1_src0 == 1)
			{
				sprn->exec0_alu0 = spro->dec1_immediate;
			}
			else
			{
				sprn->exec0_alu0 = (spro->dec1_src0) ? spro->r[spro->dec1_src0] : R0;
			}

			if (spro->dec1_src1 == 1)
			{
				sprn->exec0_alu1 = spro->dec1_immediate;
			}
			else
			{
				sprn->exec0_alu1 = (spro->dec1_src1) ? spro->r[spro->dec1_src1] : R0;
			}
			sprn->exec0_pc = spro->dec1_pc;
			sprn->exec0_inst = spro->dec1_inst;
			sprn->exec0_opcode = spro->dec1_opcode;
			sprn->exec0_src0 = spro->dec1_src0;
			sprn->exec0_src1 = spro->dec1_src1;
			sprn->exec0_dst = spro->dec1_dst;
			sprn->exec0_immediate = spro->dec1_immediate;
		}
		sprn->exec0_active = 1;
	}

	// exec0
	sprn->exec1_active = 0;	  //TODO make sure handles polling also. same as ex2.
	if (spro->exec0_active) {

		switch (spro->exec0_opcode)
		{
		case ADD:
			sprn->exec1_aluout = spro->exec0_alu0 + spro->exec0_alu1;
			break;

		case SUB:
			sprn->exec1_aluout = spro->exec0_alu0 - spro->exec0_alu1;
			break;

		case LSF:
			sprn->exec1_aluout = spro->exec0_alu0 << spro->exec0_alu1;
			break;

		case RSF:
			sprn->exec1_aluout = spro->exec0_alu0 >> spro->exec0_alu1;
			break;

		case AND:
			sprn->exec1_aluout = spro->exec0_alu0 & spro->exec0_alu1;
			break;

		case OR:
			sprn->exec1_aluout = spro->exec0_alu0 | spro->exec0_alu1;
			break;

		case XOR:
			sprn->exec1_aluout = spro->exec0_alu0 ^ spro->exec0_alu1;
			break;

		case LHI:
			// we need to only load the imm into the high bits of dst 
			// and not override the lower bits of dst, so we use AND
			if (spro->exec0_dst > 1)
			{
				sprn->exec1_aluout = spro->exec0_alu0 & (spro->exec0_immediate) << 16;
			}
			break;

		case LD:
			mem_available = false;
			if (spro->exec0_alu1 < SP_SRAM_HEIGHT)
			{
				llsim_mem_read(sp->sramd, spro->exec0_alu1);
			}
			break;

		case ST:
			mem_available = false;			
			llsim_mem_set_datain(sp->sramd, spro->exec0_alu0, 31, 0);
			llsim_mem_write(sp->sramd, spro->exec0_alu1);
			break;

		case JLT:
			sprn->exec1_aluout = spro->exec0_alu0 < spro->exec0_alu1;
			break;

		case JLE:
			sprn->exec1_aluout = spro->exec0_alu0 <= spro->exec0_alu1;
			break;

		case JEQ:
			sprn->exec1_aluout = spro->exec0_alu0 == spro->exec0_alu1;
			break;

		case JNE:
			sprn->exec1_aluout = spro->exec0_alu0 != spro->exec0_alu1;
			break;

		case JIN:
			//Check edge case: the address we need to jump to is bigger than the memory
			if (spro->exec0_alu0 < SP_SRAM_HEIGHT)
			{
				sprn->exec1_aluout = 1;
			}
			break;

		case HLT:
			break;
		}
		//checking if we need to forward ALU
		switch (spro->exec0_opcode)
		{
			case ADD:
			case SUB:
			case LSF:
			case RSF:
			case AND:
			case OR:
			case XOR:
			case LHI:
				if(spro->exec0_dst > 1 && spro->exec0_dst < 8)
					{
						if(spro->exec0_dst==spro->dec1_src0) // forwarding ALU to ALU
						{
							sprn->exec0_alu0 = sprn->exec1_aluout;
						}

						if(spro->exec0_dst==spro->dec1_src1) // forwarding ALU to ALU
						{
							sprn->exec0_alu1 = sprn->exec1_aluout;
				 		}
				 	}
				 	break;
			case JLT:
			 case JLE:
			 case JEQ:
			 case JNE:
			 case JIN:
				 if(sprn->exec1_aluout == 1)
				 {
					 if (spro->fetch1_pc != spro->exec0_immediate)
					 {
						 flush_pipeline(&sprn);
					 }
					 if (jump_predictors[(spro->exec0_pc % 40)] < 2)
					 {
						 jump_predictors[(spro->exec0_pc % 40)]++;
					 }
				 }
				 else //if branch is not taken
				 {
					 if (spro->fetch1_pc == spro->exec0_immediate)
					 {
						 flush_pipeline(&sprn);
					 }
					 if (jump_predictors[(spro->exec0_pc % 40)] != 0)
					 {
						 jump_predictors[(spro->exec0_pc % 40)]--;
					 }
				 }
			 case ST:
				 if (sprn->exec0_src0 == spro->exec0_dst)  //FW ALU result to store
				 {
					 sprn->exec0_alu0 = sprn->exec1_aluout;
				 }
				 if (sprn->exec0_src1 == spro->exec0_dst)
				 {
					 sprn->exec0_alu1 = sprn->exec1_aluout;
				 }
				 default:
					
				 	break;
			 }
			 
	 	sprn->exec1_pc = spro->exec0_pc;
		sprn->exec1_inst=spro->exec0_inst;
		sprn->exec1_opcode = spro->exec0_opcode;
		sprn->exec1_src0 = spro->exec0_src0;
		sprn->exec1_src1 = spro->exec0_src1;
		sprn->exec1_dst = spro->exec0_dst;
		sprn->exec1_immediate = spro->exec0_immediate;
		sprn->exec1_alu0 = spro->exec0_alu0;

		if (spro->dec1_src0 == spro->exec0_dst || spro->dec1_src1 == spro->exec0_dst)
		{
			raw_hazard = 0;
		}

		sprn->exec1_alu1 = spro->exec0_alu1;			 
		
		sprn->exec1_active = 1;
	}

	// exec1
	if (spro->exec1_active) {
		sp_printf("In exec1\n");
			switch(spro->exec1_opcode)
			{
			case LD:
				data_extracted = llsim_mem_extract_dataout(sp->sramd, 31, 0);		
				if(spro->exec1_dst > 1 && spro->exec1_dst < 8)
				{
					sprn->r[spro->exec1_dst] = data_extracted;
					// FORWARD: LD -> ALU
					if (spro->exec1_dst == spro->dec1_src0) 
					{
						sprn->exec0_alu0 = data_extracted;
					}
					// FORWARD: LD -> ALU
					if (spro->exec1_dst == spro->dec1_src1) 
					{
						sprn->exec0_alu1 = data_extracted;
					}
				}
		
		 		break;
	 		case ST:
	 			break;
	 		case HLT:
	 			break;
	 		case JLT:
	 		case JLE:
	 		case JEQ:
	 		case JNE:	 		
	 			if(spro->exec1_aluout)
	 			{
	 				sprn->fetch0_pc = spro->exec1_immediate - 1;
	 				sprn->r[7] = spro->exec1_pc;
	 			}
	 			break;
	 		case JIN:
	 			if(spro->exec1_aluout)
	 			{
	 				sprn->fetch0_pc = spro->exec1_alu0 - 1;
	 				sprn->r[7] = spro->exec1_pc;
	 			}
	 			break;
	 		case ADD:
			case SUB:
			case LSF:
			case RSF:
			case AND:
			case OR:
			case XOR:
			case LHI:
			case POL:
				if(spro->exec1_dst >1 && spro->exec1_dst <8)
				{
				sprn->r[spro->exec1_dst] = spro->exec1_aluout;
				}
				// FORWARD: ALU -> ALU
				if(spro->exec1_dst==spro->dec1_src0) 
				{
					sprn->exec0_alu0 = spro->exec1_aluout;
				}
				// FORWARD: ALU -> ALU
				if(spro->exec1_dst==spro->dec1_src1) 
				{
					sprn->exec0_alu1 = spro->exec1_aluout;
				}
				break;
			default:
		 		break;
			}
			sp_printf("In exec1 after switch\n");
		if (pc_of_last_inst_executed != spro->exec1_pc)
		{
			print_all_lines(sp, pc_of_last_inst_executed, nr_simulated_instructions);
			pc_of_last_inst_executed = spro->exec1_pc;
			nr_simulated_instructions++;
		}
		sp_printf("In exec1 after print\n");
		if(spro->exec1_opcode == HLT)
		{
			llsim_stop();
			end_trace(inst_trace_fp, nr_simulated_instructions, pc_of_last_inst_executed);

			ctl_dma_state = DMA_IDLE_STATE;
			dma_opcode_received = false;
			fclose(inst_trace_fp);
			fclose(cycle_trace_fp);
			dump_sram(sp, "srami_out.txt", sp->srami);
			dump_sram(sp, "sramd_out.txt", sp->sramd);
		}

		if (dma_opcode_received)
		{
			//llsim_printf("dma received\n");
			perform_dma_logic(mem_available, sp);
		}
	}
}

static void sp_run(llsim_unit_t *unit)
{
	sp_t *sp = (sp_t *) unit->private;
	//	sp_registers_t *spro = sp->spro;
	//	sp_registers_t *sprn = sp->sprn;

	//	llsim_printf("-------------------------\n");

	if (llsim->reset) {
		sp_reset(sp);
		return;
	}

	sp->srami->read = 0;
	sp->srami->write = 0;
	sp->sramd->read = 0;
	sp->sramd->write = 0;

	sp_ctl(sp);
}

static void sp_generate_sram_memory_image(sp_t *sp, char *program_name)
{
        FILE *fp;
        int addr, i;

        fp = fopen(program_name, "r");
        if (fp == NULL) {
                printf("couldn't open file %s\n", program_name);
                exit(1);
        }
        addr = 0;
        while (addr < SP_SRAM_HEIGHT) {
                fscanf(fp, "%08x\n", &sp->memory_image[addr]);
                //              printf("addr %x: %08x\n", addr, sp->memory_image[addr]);
                addr++;
                if (feof(fp))
                        break;
        }
	sp->memory_image_size = addr;

        fprintf(inst_trace_fp, "program %s loaded, %d lines\n", program_name, addr);

	for (i = 0; i < sp->memory_image_size; i++) {
		llsim_mem_inject(sp->srami, i, sp->memory_image[i], 31, 0);
		llsim_mem_inject(sp->sramd, i, sp->memory_image[i], 31, 0);
	}
}

void sp_init(char *program_name)
{
	llsim_unit_t *llsim_sp_unit;
	llsim_unit_registers_t *llsim_ur;
	sp_t *sp;

	llsim_printf("initializing sp unit\n");

	inst_trace_fp = fopen("inst_trace.txt", "w");
	if (inst_trace_fp == NULL) {
		printf("couldn't open file inst_trace.txt\n");
		exit(1);
	}

	cycle_trace_fp = fopen("cycle_trace.txt", "w");
	if (cycle_trace_fp == NULL) {
		printf("couldn't open file cycle_trace.txt\n");
		exit(1);
	}

	llsim_sp_unit = llsim_register_unit("sp", sp_run);
	llsim_ur = llsim_allocate_registers(llsim_sp_unit, "sp_registers", sizeof(sp_registers_t));
	sp = llsim_malloc(sizeof(sp_t));
	llsim_sp_unit->private = sp;
	sp->spro = llsim_ur->old;
	sp->sprn = llsim_ur->new;

	sp->srami = llsim_allocate_memory(llsim_sp_unit, "srami", 32, SP_SRAM_HEIGHT, 0);
	sp->sramd = llsim_allocate_memory(llsim_sp_unit, "sramd", 32, SP_SRAM_HEIGHT, 0);
	sp_generate_sram_memory_image(sp, program_name);

	sp->start = 1;
	
	// c2v_translate_end
}

int print_line1(FILE* file, int cnt_of_inst, int pc_of_inst)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	check_ret = sprintf(line_to_print,
		"--- instruction %d (%04x) @ PC %d (%04d) -----------------------------------------------------------\n",
		cnt_of_inst,
		cnt_of_inst,
		pc_of_inst,
		pc_of_inst
	);

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

int print_line2(FILE* file, sp_registers_t* inst_regs)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	check_ret = sprintf(line_to_print,
		"pc = %04d, inst = %08x, opcode = %d (%s), dst = %d, src0 = %d, src1 = %d, immediate = %08x\n",
		inst_regs->exec1_pc,
		inst_regs->exec1_inst,
		inst_regs->exec1_opcode,
		opcode_name[inst_regs->exec1_opcode],
		inst_regs->exec1_dst,
		inst_regs->exec1_src0,
		inst_regs->exec1_src1,
		inst_regs->exec1_immediate

	);

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

int print_line3(FILE* file, sp_registers_t* inst_regs)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	check_ret = sprintf(line_to_print,
		"r[0] = %08x r[1] = %08x r[2] = %08x r[3] = %08x\n",
		inst_regs->r[0],
		inst_regs->exec1_immediate,
		inst_regs->r[2],
		inst_regs->r[3]
	);

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

int print_line4(FILE* file, sp_registers_t* inst_regs)//, int* old_regs)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	check_ret = sprintf(line_to_print,
		"r[4] = %08x r[5] = %08x r[6] = %08x r[7] = %08x\n\n",
		inst_regs->r[4],//old_regs[4],
		inst_regs->r[5],//old_regs[5],
		inst_regs->r[6],//old_regs[6],
		inst_regs->r[7]//old_regs[7]
	);

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

int print_line5(FILE* file, sp_t* sp)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	switch (sp->spro->exec1_opcode)
	{
		case ADD:
		case SUB:
		case LSF:
		case RSF:
		case AND:
		case OR:
		case XOR:
			check_ret = sprintf(line_to_print,
				">>>> EXEC: R[%d] = %d %s %d <<<<\n\n",
				sp->spro->exec1_dst,
				sp->spro->exec1_alu0,
				opcode_name[sp->spro->exec1_opcode],
				sp->spro->exec1_alu1
			);
			break;

		case LHI:
			check_ret = sprintf(line_to_print,
				">>>> EXEC: R[%d] %s %d <<<<\n\n",
				sp->spro->exec1_dst,
				opcode_name[sp->spro->exec1_opcode],
				sp->spro->exec1_immediate
			);
			break;
		case LD:
			check_ret = sprintf(line_to_print,
				">>>> EXEC: R[%d] = MEM[%d] = %08x <<<<\n\n",
				sp->spro->exec1_dst,
				sp->spro->exec1_alu1,// the value of the memory address
				sp->sprn->r[sp->spro->exec1_dst] //the value in the memory address
			);
			break;

		case ST:
			check_ret = sprintf(line_to_print,
				">>>> EXEC: MEM[%d] = R[%d] = %08x <<<<\n\n",
				sp->spro->exec1_alu1,// the value of the memory address
				sp->spro->exec1_src0, // the register whose value we save 
				sp->spro->exec1_alu0 //the value in the memory address
			);
			break;

		case JLT:
		case JLE:
		case JEQ:
		case JNE:
			check_ret = sprintf(line_to_print,
				">>>> EXEC: %s %d, %d, %d <<<<\n\n",
				opcode_name[sp->spro->exec1_opcode],
				sp->spro->exec1_alu0,
				sp->spro->exec1_alu1,
				sp->sprn->exec1_pc
			);
			break;

		case JIN:
			check_ret = sprintf(line_to_print,
				">>>> EXEC: %s %d <<<<\n\n",
				opcode_name[sp->spro->exec1_opcode],
				sp->sprn->exec1_pc
			);
			break;

		case HLT:
			check_ret = sprintf(line_to_print,
				">>>> EXEC: HALT at PC %04x <<<<\n",
				sp->spro->exec1_pc
			);
			break;
		default:
			break;
	}

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

void print_all_lines(sp_t* sp, int pc_of_inst, int nr_sim_inst)
{
	print_line1(inst_trace_fp, nr_sim_inst, pc_of_inst);
	//sp_printf("In print_all_lines after 1\n");
	print_line2(inst_trace_fp, sp->spro);
	//sp_printf("In print_all_lines after 2\n");
	print_line3(inst_trace_fp, sp->spro);
	//sp_printf("In print_all_lines after 3\n");
	print_line4(inst_trace_fp, sp->spro);
	//sp_printf("In print_all_lines after 4\n");
	print_line5(inst_trace_fp, sp);
	//sp_printf("In print_all_lines after 5\n");
	fflush(inst_trace_fp);
	//sp_printf("In print_all_lines after flush\n");
}

int end_trace(FILE* file, int cnt, int pc)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	check_ret = sprintf(line_to_print,
		"sim finished at pc %d, %d instructions",
		pc + 1,
		cnt + 1
	);

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

void init_dma_logic(int source, int dest, int amount)
{
	dma_regs[0] = source;
	dma_regs[1] = dest;
	dma_regs[2] = amount;
	dma_opcode_received = true;
	//printf("dma_regs[0]: %d, dma_regs[1]: %d, and dma_regs[2]: %d\n", dma_regs[0], dma_regs[1], dma_regs[2]);
}


void perform_dma_logic(bool mem_available, sp_t* sp)
{
	//llsim_printf("dma state %d and sp is %d\n", ctl_dma_state, sp->spro->ctl_state);
	//llsim_printf("dma_regs[2] is %d and mem_available is %d\n", dma_regs[2], mem_available);
	//llsim_printf("read_into_reg3 is %d and write_reg3 is %d. dma_opcode_received is %d\n\n", read_into_reg3, write_reg3, dma_opcode_received);

	// 3 bit control state machine of DMA
	switch (ctl_dma_state)
	{
	case(NO_READ_WRITE):
		if (dma_regs[2] == 0)
		{
			//llsim_printf("dma finished\n");
			dma_opcode_received = false;
			ctl_dma_state = DMA_IDLE_STATE;
		}

		else if (mem_available)
		{
			llsim_mem_read(sp->sramd, dma_regs[0]); //fetch MEM[dma_regs[0]]
			dma_regs[0]++;
			ctl_dma_state = ONE_READ_NO_WRITE;
		}
		else
		{
			ctl_dma_state = NO_READ_WRITE;
		}
		break;

	case(ONE_READ_NO_WRITE):
		if (read_into_reg3)
		{
			dma_regs[3] = llsim_mem_extract_dataout(sp->sramd, 31, 0);
			//llsim_printf("ONE_READ_NO_WRITE: dma_regs[3] after extract is: %d\n", dma_regs[3]);
		}
		else
		{
			dma_regs[4] = llsim_mem_extract_dataout(sp->sramd, 31, 0);
			//llsim_printf("ONE_READ_NO_WRITE: dma_regs[4] after extract is: %d\n", dma_regs[4]);

		}
		read_into_reg3 = !read_into_reg3; //next, data will be loaded to other register
		dma_regs[2]--;

		if (dma_regs[2] == 0)  //if length remaining is 0, then no need to keep reading.
		{
			ctl_dma_state = ONE_WRITE_READY;
		}
		else if (mem_available)
		{
			llsim_mem_read(sp->sramd, dma_regs[0]);
			dma_regs[0]++;
			ctl_dma_state = ONE_READ_ONE_WRITE;
		}
		else
		{
			ctl_dma_state = ONE_WRITE_READY;
		}

		break;

	case(ONE_READ_ONE_WRITE):
		if (read_into_reg3)
		{
			dma_regs[3] = llsim_mem_extract_dataout(sp->sramd, 31, 0);
			//llsim_printf("ONE_READ_ONE_WRITE: dma_regs[3] after extract is: %d\n", dma_regs[3]);

		}
		else
		{
			dma_regs[4] = llsim_mem_extract_dataout(sp->sramd, 31, 0);
			//llsim_printf("ONE_READ_ONE_WRITE: dma_regs[4] after extract is: %d\n", dma_regs[4]);
		}
		read_into_reg3 = !read_into_reg3; //next, data will be loaded to other register
		dma_regs[2]--;

		if (mem_available)
		{
			int temp_reg; //simulate mux choosing which register to write
			if (write_reg3)
			{
				temp_reg = dma_regs[3];
			}
			else
			{
				temp_reg = dma_regs[4];
			}
			llsim_mem_set_datain(sp->sramd, temp_reg, 31, 0);
			llsim_mem_write(sp->sramd, dma_regs[1]);
			//llsim_printf("ONE_READ_ONE_WRITE: wrote %d to address %d\n", temp_reg, dma_regs[1]);
			dma_regs[1]++;
			write_reg3 = !write_reg3; //next, data will be loaded to other register
			ctl_dma_state = ONE_WRITE_READY;
		}
		else
		{
			ctl_dma_state = TWO_WRITE_READY;
		}
		break;

	case(TWO_WRITE_READY):
		if (mem_available)
		{
			int temp_reg; //simulate mux choosing which register to write
			if (write_reg3)
			{
				temp_reg = dma_regs[3];
			}
			else
			{
				temp_reg = dma_regs[4];
			}
			llsim_mem_set_datain(sp->sramd, temp_reg, 31, 0);
			llsim_mem_write(sp->sramd, dma_regs[1]);
			//llsim_printf("TWO_WRITE_READY: wrote %d to address %d\n", temp_reg, dma_regs[1]);
			dma_regs[1]++;
			write_reg3 = !write_reg3; //next, data will be loaded to other register
			ctl_dma_state = ONE_WRITE_READY;
		}
		break;
	case(ONE_WRITE_READY):
		if (mem_available)
		{
			int temp_reg; //simulate mux choosing which register to write
			if (write_reg3)
			{
				temp_reg = dma_regs[3];
			}
			else
			{
				temp_reg = dma_regs[4];
			}
			llsim_mem_set_datain(sp->sramd, temp_reg, 31, 0);
			llsim_mem_write(sp->sramd, dma_regs[1]);
			//llsim_printf("ONE_WRITE_READY: wrote %d to address %d\n", temp_reg, dma_regs[1]);
			dma_regs[1]++;
			write_reg3 = !write_reg3; //next, data will be loaded to other register
			if (dma_regs[2] == 0)
			{
				dma_opcode_received = false;
				ctl_dma_state = DMA_IDLE_STATE;
			}
			ctl_dma_state = NO_READ_WRITE;
		}
		break;
	case(DMA_IDLE_STATE):
		//llsim_printf("in DMA_IDLE_STATE. dma_opcode_received is: %d\n", dma_opcode_received);
		if (dma_opcode_received)
		{
			ctl_dma_state = NO_READ_WRITE;
		}
		break;


	}

}

int predict_jump(int current_pc)
{
	int table_value = jump_predictors[(current_pc % 40)]; //check what's the current pc prediction
	return table_value > 1; //simulate 2 bit prediction
}

void flush_pipeline(sp_registers_t** sprn_address)
{
	sp_registers_t* sprn = *sprn_address;
	//flush fetch1
	sprn->fetch1_pc = -1;
	
	//flush dec0
	sprn->dec0_inst = -1;
	sprn->dec0_pc = -1;
	
	//flush dec1
	sprn->dec1_dst = -1;
	sprn->dec1_immediate = -1;
	sprn->dec1_inst = -1;
	sprn->dec1_opcode = -1;
	sprn->dec1_pc = -1;
	sprn->dec1_src0 = -1;
	sprn->dec1_src1 = -1;
}

bool validate_dma_values(int source, int dest, int amount)
{
	bool res = (amount > 0) && (source >= 0) && (dest >= 0) && (source != dest);
	//printf("res: %d", res);
	return res;
}