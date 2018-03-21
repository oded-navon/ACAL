#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "llsim.h"

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

	// 16 bit program counter
	int pc;

	// 32 bit instruction
	int inst;

	// 5 bit opcode
	int opcode;

	// 3 bit destination register index
	int dst;

	// 3 bit source #0 register index
	int src0;

	// 3 bit source #1 register index
	int src1;

	// 32 bit alu #0 operand
	int alu0;

	// 32 bit alu #1 operand
	int alu1;

	// 32 bit alu output
	int aluout;

	// 32 bit immediate field (original 16 bit sign extended)
	int immediate;

	// 32 bit cycle counter
	int cycle_counter;

	// 3 bit control state machine state register
	int ctl_state;

	// control states
	#define CTL_STATE_IDLE		0
	#define CTL_STATE_FETCH0	1
	#define CTL_STATE_FETCH1	2
	#define CTL_STATE_DEC0		3
	#define CTL_STATE_DEC1		4
	#define CTL_STATE_EXEC0		5
	#define CTL_STATE_EXEC1		6
} sp_registers_t;

/*
 * Master structure
 */
typedef struct sp_s {
	// local sram
#define SP_SRAM_HEIGHT	64 * 1024
	llsim_memory_t *sram;

	unsigned int memory_image[SP_SRAM_HEIGHT];
	int memory_image_size;

	sp_registers_t *spro, *sprn;
	
	int start;
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
#define HLT 24

static char opcode_name[32][4] = {"ADD", "SUB", "LSF", "RSF", "AND", "OR", "XOR", "LHI",
				 "LD", "ST", "U", "U", "U", "U", "U", "U",
				 "JLT", "JLE", "JEQ", "JNE", "JIN", "U", "U", "U",
				 "HLT", "U", "U", "U", "U", "U", "U", "U"};

static void dump_sram(sp_t *sp)
{
	FILE *fp;
	int i;

	fp = fopen("sram_out.txt", "w");
	if (fp == NULL) {
                printf("couldn't open file sram_out.txt\n");
                exit(1);
	}
	for (i = 0; i < SP_SRAM_HEIGHT; i++)
		fprintf(fp, "%08x\n", llsim_mem_extract(sp->sram, i, 31, 0));
	fclose(fp);
}


static void sp_ctl(sp_t *sp)
{
	sp_registers_t *spro = sp->spro;
	sp_registers_t *sprn = sp->sprn;
	int i;

	// sp_ctl

	fprintf(cycle_trace_fp, "cycle %d\n", spro->cycle_counter);
	for (i = 2; i <= 7; i++)
		fprintf(cycle_trace_fp, "r%d %08x\n", i, spro->r[i]);
	fprintf(cycle_trace_fp, "pc %08x\n", spro->pc);
	fprintf(cycle_trace_fp, "inst %08x\n", spro->inst);
	fprintf(cycle_trace_fp, "opcode %08x\n", spro->opcode);
	fprintf(cycle_trace_fp, "dst %08x\n", spro->dst);
	fprintf(cycle_trace_fp, "src0 %08x\n", spro->src0);
	fprintf(cycle_trace_fp, "src1 %08x\n", spro->src1);
	fprintf(cycle_trace_fp, "immediate %08x\n", spro->immediate);
	fprintf(cycle_trace_fp, "alu0 %08x\n", spro->alu0);
	fprintf(cycle_trace_fp, "alu1 %08x\n", spro->alu1);
	fprintf(cycle_trace_fp, "aluout %08x\n", spro->aluout);
	fprintf(cycle_trace_fp, "cycle_counter %08x\n", spro->cycle_counter);
	fprintf(cycle_trace_fp, "ctl_state %08x\n\n", spro->ctl_state);

	sprn->cycle_counter = spro->cycle_counter + 1;

	int temp0 = 0;
	int temp1 = 0;
	int mem_adr_to_read_from;
	int src0_for_jumps;
	int src1_for_jumps;
	int mem_adr_to_write_to;
	int data_to_write_in_st;

	switch (spro->ctl_state) {
	case CTL_STATE_IDLE:
		sprn->pc = 0;
		if (sp->start)
			sprn->ctl_state = CTL_STATE_FETCH0;
		break;

	case CTL_STATE_FETCH0:
		llsim_mem_read(sp->sram, spro->pc);
		sprn->ctl_state = CTL_STATE_FETCH1; 
		sprn->pc = (spro->pc);
		break;

	case CTL_STATE_FETCH1:
		sprn->inst = llsim_mem_extract_dataout(sp->sram, 31, 0);
		sprn->ctl_state = CTL_STATE_DEC0; 
		sprn->pc = (spro->pc);
		break;

	case CTL_STATE_DEC0:
		sprn->opcode = (spro->inst & inst_params_opcode) >> inst_params_opcode_shift;
		short imm = spro->inst & inst_params_imm;
		sprn->immediate = (int)imm;
		sprn->src1 = (spro->inst & inst_params_src1) >> inst_params_src1_shift;
		sprn->src0 = (spro->inst & inst_params_src0) >> inst_params_src0_shift;
		sprn->dst = (spro->inst & inst_params_dst) >> inst_params_dst_shift;

		sprn->ctl_state = CTL_STATE_DEC1; 
		sprn->pc = (spro->pc);
		break;

	case CTL_STATE_DEC1:
		if (spro->src0 == 1)
		{
			sprn->alu0 = spro->immediate;
		}
		else
		{
			sprn->alu0 = spro->r[spro->src0];
		}

		if (spro->src1 == 1)
		{
			sprn->alu1 = spro->immediate;
		}
		else
		{
			sprn->alu1 = spro->r[spro->src1];
		}

		sprn->ctl_state = CTL_STATE_EXEC0; 
		sprn->pc = (spro->pc);
		break;

	case CTL_STATE_EXEC0:
	//in this step we change the pc to the pc of the next instruction(pc++ or with jump instruction)

	switch (spro->opcode)
	{
		case ADD:
			sprn->aluout = spro->alu0 + spro->alu1;
			sprn->pc = (spro->pc)++;
			break;

		case SUB:
			sprn->aluout = spro->alu0 - spro->alu1;
			sprn->pc = (spro->pc)++;
			break;

		case LSF:
			sprn->aluout = spro->alu0 << spro->alu1;
			sprn->pc = (spro->pc)++;
			break;

		case RSF:
			sprn->aluout = spro->alu0 >> spro->alu1;
			sprn->pc = (spro->pc)++;
			break;

		case AND:
			sprn->aluout = spro->alu0 & spro->alu1;
			sprn->pc = (spro->pc)++;
			break;

		case OR:
			sprn->aluout = spro->alu0 | spro->alu1;
			sprn->pc = (spro->pc)++;
			break;

		case XOR:
			sprn->aluout = spro->alu0 ^ spro->alu1;
			sprn->pc = (spro->pc)++;
			break;

		case LHI:
			temp0 = (spro->immediate) << 16;
			// we need to only load the imm into the high bits of dst 
			// and not override the lower bits of dst, so we use AND
			if (spro->dst > 1)
			{
				sprn->aluout = spro->alu0 & temp0;
			}
			sprn->pc = (spro->pc)++;
			break;

		case LD:
			//if src1 is R1, we use the imm
			if (spro->src1 == 1)
			{
				mem_adr_to_read_from = spro->immediate;
			}
			else
			{
				mem_adr_to_read_from = spro->r[spro->src1];
			}
			//now start the load
			if (mem_adr_to_read_from < SP_SRAM_HEIGHT)
			{
				llsim_mem_read(sp->sram, mem_adr_to_read_from);
			}
			sprn->pc = (spro->pc)++;
			break;

		case ST:
			// In EXEC0 we don't execute write operations, they're executed in EXEC1
			// We just advance the pc, like we do for other operations in this cycle
			sprn->pc = (spro->pc)++;
			break;

		case JLT:
			//if src0 is R1, we use the imm for src0
			if (spro->src0 == 1)
			{
				src0_for_jumps = spro->immediate;
			}
			else
			{
				src0_for_jumps = spro->r[spro->src0];
			}
			//if src1 is R1, we use the imm for src1
			if (spro->src1 == 1)
			{
				src1_for_jumps = spro->immediate;
			}
			else
			{
				src1_for_jumps = spro->r[spro->src1];
			}

			// now check the jump condition
			if (src0_for_jumps < src1_for_jumps)
			{
				sprn->r[7] = spro->pc;
				sprn->pc = spro->immediate;
			}
			else
			{
				sprn->pc = (spro->pc)++;
			}
			break;

		case JLE:
			//if src0 is R1, we use the imm for src0
			if (spro->src0 == 1)
			{
				src0_for_jumps = spro->immediate;
			}
			else
			{
				src0_for_jumps = spro->r[spro->src0];
			}
			//if src1 is R1, we use the imm for src1
			if (spro->src1 == 1)
			{
				src1_for_jumps = spro->immediate;
			}
			else
			{
				src1_for_jumps = spro->r[spro->src1];
			}

			// now check the jump condition
			if (src0_for_jumps <= src1_for_jumps)
			{
				sprn->r[7] = spro->pc;
				sprn->pc = spro->immediate;
			}
			else
			{
				sprn->pc = (spro->pc)++;
			}
			break;

		case JEQ:
			//if src0 is R1, we use the imm for src0
			if (spro->src0 == 1)
			{
				src0_for_jumps = spro->immediate;
			}
			else
			{
				src0_for_jumps = spro->r[spro->src0];
			}

			//if src1 is R1, we use the imm for src1
			if (spro->src1 == 1)
			{
				src1_for_jumps = spro->immediate;
			}
			else
			{
				src1_for_jumps = spro->r[spro->src1];
			}

			// now check the jump condition
			if (src0_for_jumps == src1_for_jumps)
			{
				sprn->r[7] = spro->pc;
				sprn->pc = spro->immediate;
			}
			else
			{
				sprn->pc = (spro->pc)++;
			}
			break;

		case JNE:
			//if src0 is R1, we use the imm for src0
			if (spro->src0 == 1)
			{
				src0_for_jumps = spro->immediate;
			}
			else
			{
				src0_for_jumps = spro->r[spro->src0];
			}
			
			//if src1 is R1, we use the imm for src1
			if (spro->src1 == 1)
			{
				src1_for_jumps = spro->immediate;
			}
			else
			{
				src1_for_jumps = spro->r[spro->src1];
			}

			// now check the jump condition
			if (src0_for_jumps != src1_for_jumps)
			{
				sprn->r[7] = spro->pc;
				sprn->pc = spro->immediate;
			}
			else
			{
				sprn->pc = (spro->pc)++;
			}
			break;

		case JIN:
			//if src0 is R1, we use the imm for src0
			if (spro->src0 == 1)
			{
				src0_for_jumps = spro->immediate;
			}
			else
			{
				src0_for_jumps = spro->r[spro->src0];
			}

			// now check the jump condition
			if (src0_for_jumps < SP_SRAM_HEIGHT)
			{
				sprn->r[7] = spro->pc;
				sprn->pc = spro->immediate;
			}
			else
			{
				sprn->pc = (spro->pc)++;
			}
			break;

		case HLT:
			sprn->pc = (spro->pc)++;
			break;
	}
		sprn->ctl_state = CTL_STATE_EXEC1; 
		
		break;

	case CTL_STATE_EXEC1:

	switch (spro->opcode)
	{
		case ADD:
		case SUB:
		case LSF:
		case RSF:
		case AND:
		case OR:
		case XOR:
		case LHI:
			sprn->r[spro->dst] = spro->aluout;
			break;

		case LD:
			// check that the dst register is in the correct range
			if (spro->dst > 1 && spro->dst < 8)
			{
				sprn->r[spro->dst] = llsim_mem_extract_dataout(sp->sram, 31, 0);
			}
			break;

		case ST:
			//if src0 is R1, the data we write is the imm
			if (spro->src0 == 1)
			{
				data_to_write_in_st = spro->immediate;
			}
			else
			{
				data_to_write_in_st = spro->r[spro->src0];
			}

			// if src1 is R1, the memory address we write to is the imm
			if (spro->src1 == 1)
			{
				mem_adr_to_write_to = spro->immediate;
			}
			else
			{
				mem_adr_to_write_to = spro->r[spro->src1];
			}
			// now we start the write
			if (mem_adr_to_write_to < SP_SRAM_HEIGHT)
			{
				llsim_mem_set_datain(sp->sram, data_to_write_in_st, 31, 0);
				llsim_mem_write(sp->sram, mem_adr_to_write_to);
			}
			break;

		case JLT:
		case JLE:
		case JEQ:
		case JNE:
		case JIN:
		case HLT:
			break;
	}
		if (spro->opcode == HLT)
		{
			sprn->ctl_state = CTL_STATE_IDLE; 
		}
		else
		{
			sprn->ctl_state = CTL_STATE_FETCH0;
		}
		
		break;
	}
}

static void sp_run(llsim_unit_t *unit)
{
	sp_t *sp = (sp_t *) unit->private;

	if (llsim->reset) {
		sp_reset(sp);
		return;
	}

	sp->sram->read = 0;
	sp->sram->write = 0;

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
                addr++;
                if (feof(fp))
                        break;
        }
	sp->memory_image_size = addr;

        fprintf(inst_trace_fp, "program %s loaded, %d lines\n", program_name, addr);

	for (i = 0; i < sp->memory_image_size; i++)
		llsim_mem_inject(sp->sram, i, sp->memory_image[i], 31, 0);
}

static void sp_register_all_registers(sp_t *sp)
{
	sp_registers_t *spro = sp->spro, *sprn = sp->sprn;

	// registers
	llsim_register_register("sp", "r_0", 32, 0, &spro->r[0], &sprn->r[0]);
	llsim_register_register("sp", "r_1", 32, 0, &spro->r[1], &sprn->r[1]);
	llsim_register_register("sp", "r_2", 32, 0, &spro->r[2], &sprn->r[2]);
	llsim_register_register("sp", "r_3", 32, 0, &spro->r[3], &sprn->r[3]);
	llsim_register_register("sp", "r_4", 32, 0, &spro->r[4], &sprn->r[4]);
	llsim_register_register("sp", "r_5", 32, 0, &spro->r[5], &sprn->r[5]);
	llsim_register_register("sp", "r_6", 32, 0, &spro->r[6], &sprn->r[6]);
	llsim_register_register("sp", "r_7", 32, 0, &spro->r[7], &sprn->r[7]);

	llsim_register_register("sp", "pc", 16, 0, &spro->pc, &sprn->pc);
	llsim_register_register("sp", "inst", 32, 0, &spro->inst, &sprn->inst);
	llsim_register_register("sp", "opcode", 5, 0, &spro->opcode, &sprn->opcode);
	llsim_register_register("sp", "dst", 3, 0, &spro->dst, &sprn->dst);
	llsim_register_register("sp", "src0", 3, 0, &spro->src0, &sprn->src0);
	llsim_register_register("sp", "src1", 3, 0, &spro->src1, &sprn->src1);
	llsim_register_register("sp", "alu0", 32, 0, &spro->alu0, &sprn->alu0);
	llsim_register_register("sp", "alu1", 32, 0, &spro->alu1, &sprn->alu1);
	llsim_register_register("sp", "aluout", 32, 0, &spro->aluout, &sprn->aluout);
	llsim_register_register("sp", "immediate", 32, 0, &spro->immediate, &sprn->immediate);
	llsim_register_register("sp", "cycle_counter", 32, 0, &spro->cycle_counter, &sprn->cycle_counter);
	llsim_register_register("sp", "ctl_state", 3, 0, &spro->ctl_state, &sprn->ctl_state);
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

	sp->sram = llsim_allocate_memory(llsim_sp_unit, "sram", 32, SP_SRAM_HEIGHT, 0);
	sp_generate_sram_memory_image(sp, program_name);

	sp->start = 1;

	sp_register_all_registers(sp);
}
