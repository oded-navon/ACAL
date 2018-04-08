#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
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


#define NUM_OF_REGS (8)
#define MAX_STR_LEN (1024)
#define HALT_OPCODE (24)
#define SUCCESS (0)
#define FAIL (-1)
#define TRACE_FILE ("inst_trace.txt")
#define SRAM_OUT ("sram_out.txt")



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

//Functions we use for instruction traces
int end_trace(FILE* file, int cnt, int pc);
int print_line1(FILE* file, int cnt_of_inst, int pc_of_inst);
int print_line2(FILE* file, sp_registers_t* inst_regs);
int print_line3(FILE* file, sp_registers_t* inst_regs);
int print_line4(FILE* file, sp_registers_t* inst_regs);
int print_line5(FILE* file, sp_t* sp);

//DMA functions
void perform_dma_logic(bool mem_available, sp_t* sp);
void init_dma_logic(int source, int dest, int amount);
bool validate_dma_values(int source, int dest, int amount);
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

static char opcode_name[32][4] = {"ADD", "SUB", "LSF", "RSF", "AND", "OR", "XOR", "LHI",
				 "LD", "ST", "U", "U", "U", "U", "U", "U",
				 "JLT", "JLE", "JEQ", "JNE", "JIN", "DMA", "POL","U",
				 "HLT", "U", "U", "U", "U", "U", "U", "U"};

static void dump_sram(sp_t *sp)	  //comme
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
	bool mem_available = false;;
	// sp_ctl

	fprintf(cycle_trace_fp, "cycle %d\n", spro->cycle_counter);
	for (i = 2; i <= 7; i++)
		fprintf(cycle_trace_fp, "r%d %08x\n", i, spro->r[i]);
	for (i = 0; i <= 4; i++)
		fprintf(cycle_trace_fp, "dma_reg[%d] %08x\n", i, dma_regs[i]);
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

	switch (spro->ctl_state) {
	case CTL_STATE_IDLE:
		mem_available = true;
		sprn->pc = 0;
		if (sp->start)
		{
			sprn->ctl_state = CTL_STATE_FETCH0;
		}
		
		break;

	case CTL_STATE_FETCH0:
		mem_available = false;
		//Trace first line in inst_trace
		print_line1(inst_trace_fp, nr_simulated_instructions, spro->pc);

		llsim_mem_read(sp->sram, spro->pc);
		sprn->ctl_state = CTL_STATE_FETCH1;
		sprn->pc = (spro->pc);
		nr_simulated_instructions++;
		
		break;

	case CTL_STATE_FETCH1:
		mem_available = true;
		sprn->inst = llsim_mem_extract_dataout(sp->sram, 31, 0);
		sprn->ctl_state = CTL_STATE_DEC0; 
		sprn->pc = (spro->pc);
		break;

	case CTL_STATE_DEC0:
		mem_available = true;
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
		mem_available = true;
		if (spro->opcode == DMA && !dma_opcode_received && validate_dma_values(spro->r[spro->dst], spro->r[spro->src0], spro->immediate)) //in case DMA is already working, we ignore the new request
		{
			init_dma_logic(spro->r[spro->dst], spro->r[spro->src0], spro->immediate);
		}
		else
		{
			print_line2(inst_trace_fp, sp->spro);
			print_line3(inst_trace_fp, sp->spro);
			print_line4(inst_trace_fp, sp->spro);
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
		}

		sprn->ctl_state = CTL_STATE_EXEC0; 
		sprn->pc = (spro->pc);

		break;

	case CTL_STATE_EXEC0:
	//in this step we change the pc to the pc of the next instruction(pc++ or with jump instruction)
		mem_available = true;
		switch (spro->opcode)
	{
		case ADD:
			sprn->aluout = spro->alu0 + spro->alu1;
			break;

		case SUB:
			sprn->aluout = spro->alu0 - spro->alu1;
			break;

		case LSF:
			sprn->aluout = spro->alu0 << spro->alu1;
			break;

		case RSF:
			sprn->aluout = spro->alu0 >> spro->alu1;
			break;

		case AND:
			sprn->aluout = spro->alu0 & spro->alu1;
			break;

		case OR:
			sprn->aluout = spro->alu0 | spro->alu1;
			break;

		case XOR:
			sprn->aluout = spro->alu0 ^ spro->alu1;
			break;

		case LHI:
			// we need to only load the imm into the high bits of dst 
			// and not override the lower bits of dst, so we use AND
			if (spro->dst > 1)
			{
				sprn->aluout = spro->alu0 & (spro->immediate) << 16;
			}
			break;

		case LD:
			mem_available = false;
			if (spro->alu1 < SP_SRAM_HEIGHT)
			{
				llsim_mem_read(sp->sram, spro->alu1);
			}
			break;

		case ST:
			// In EXEC0 we don't execute write operations, they're executed in EXEC1
			// We just advance the pc, like we do for other operations in this cycle
			break;

		case JLT:
			sprn->aluout = spro->alu0 < spro->alu1;
			break;

		case JLE:
			sprn->aluout = spro->alu0 <= spro->alu1;
			break;

		case JEQ:
			sprn->aluout = spro->alu0 == spro->alu1;
			break;

		case JNE:
			sprn->aluout = spro->alu0 != spro->alu1;
			break;

		case JIN:
			//Check edge case: the address we need to jump to is bigger than the memory
			if (spro->alu0 < SP_SRAM_HEIGHT)
			{
				sprn->aluout = 1;
			}
			break;
		case POL:
			sprn->aluout = !dma_opcode_received;
			break;

		case HLT:
			break;
	}
		sprn->ctl_state = CTL_STATE_EXEC1; 

		break;

	case CTL_STATE_EXEC1:
		mem_available = true;
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
			case POL:
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
				mem_available = false;
				// now we start the write
				if (spro->alu1 < SP_SRAM_HEIGHT)
				{
					llsim_mem_set_datain(sp->sram, spro->alu0, 31, 0);
					llsim_mem_write(sp->sram, spro->alu1);
				}
				break;

			case JLT:
			case JLE:
			case JEQ:
			case JNE:
			case JIN:
				if (spro->aluout)
				{
					sprn->r[7] = spro->pc;
					sprn->pc = spro->immediate - 1;
				}
				break;
			case HLT:
				//llsim_printf("HLT received\n");
				break;
		}
		

		//else (default)
		sprn->pc++;
		print_line5(inst_trace_fp, sp);
		
		if (spro->opcode == HLT)
		{
			//llsim_printf("received halt\n");
			sprn->ctl_state = CTL_STATE_IDLE;
			ctl_dma_state = DMA_IDLE_STATE;
			dma_opcode_received = false;
			end_trace(inst_trace_fp, nr_simulated_instructions - 1, sp->spro->pc - 1);
			fclose(inst_trace_fp);
			fclose(cycle_trace_fp);
			dump_sram(sp);
			llsim_stop();
		}
		else
		{
			sprn->ctl_state = CTL_STATE_FETCH0;
		}
		break;
	}

	if (dma_opcode_received)
	{
		//llsim_printf("dma received\n");
		perform_dma_logic(mem_available, sp);
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
		int res = 0;
        fp = fopen(program_name, "r");
        if (fp == NULL) {
                printf("couldn't open file %s\n", program_name);
                exit(1);
        }
        addr = 0;
        while (addr < SP_SRAM_HEIGHT) {
				res = fscanf(fp, "%08x\n", &sp->memory_image[addr]);
				if (res > 0)
				{
				}
                addr++;
                if (feof(fp))
                        break;
        }
	sp->memory_image_size = addr;

        fprintf(inst_trace_fp, "program %s loaded, %d lines\n\n", program_name, addr);

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

bool validate_dma_values(int source, int dest, int amount)
{
	bool res = (amount > 0) && (source >= 0) && (dest >= 0) && (source != dest);
	//printf("res: %d", res);
	return res;
}

void init_dma_logic(int source, int dest, int amount)
{
	dma_regs[0] = source;
	dma_regs[1] = dest;
	dma_regs[2] = amount;
	dma_opcode_received = true;
	//printf("dma_regs[0]: %d, dma_regs[1]: %d, and dma_regs[2]: %d\n", dma_regs[0], dma_regs[1], dma_regs[2]);
}
/*
TODO:
how to finish - delete regs values and switch to idle


*/

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
			llsim_mem_read(sp->sram, dma_regs[0]); //fetch MEM[dma_regs[0]]
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
			dma_regs[3] = llsim_mem_extract_dataout(sp->sram, 31, 0);
			//llsim_printf("ONE_READ_NO_WRITE: dma_regs[3] after extract is: %d\n", dma_regs[3]);
		}
		else
		{
			dma_regs[4] = llsim_mem_extract_dataout(sp->sram, 31, 0);
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
			llsim_mem_read(sp->sram, dma_regs[0]);
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
			dma_regs[3] = llsim_mem_extract_dataout(sp->sram, 31, 0);
			//llsim_printf("ONE_READ_ONE_WRITE: dma_regs[3] after extract is: %d\n", dma_regs[3]);

		}
		else
		{
			dma_regs[4] = llsim_mem_extract_dataout(sp->sram, 31, 0);
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
			llsim_mem_set_datain(sp->sram, temp_reg, 31, 0);
			llsim_mem_write(sp->sram, dma_regs[1]);
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
			llsim_mem_set_datain(sp->sram, temp_reg, 31, 0);
			llsim_mem_write(sp->sram, dma_regs[1]);
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
			llsim_mem_set_datain(sp->sram, temp_reg, 31, 0);
			llsim_mem_write(sp->sram, dma_regs[1]);
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

	check_ret = fprintf(file, "%s",line_to_print);
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
		inst_regs->pc,//inst_cmd->pc_of_inst,
		inst_regs->inst,//inst_cmd->original_inst,
		inst_regs->opcode,//inst_cmd->opcode,
		opcode_name[inst_regs->opcode],//opcode_name[inst_cmd->opcode],
		inst_regs->dst,//inst_cmd->dst,
		inst_regs->src0,//inst_cmd->src0,
		inst_regs->src1,//inst_cmd->src1,
		inst_regs->immediate//inst_cmd->imm

	);

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, "%s", line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

int print_line3(FILE* file, sp_registers_t* inst_regs)//inst_trace* inst_cmd, int* old_regs)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	check_ret = sprintf(line_to_print,
		"r[0] = %08x r[1] = %08x r[2] = %08x r[3] = %08x\n",
		inst_regs->r[0],//old_regs[0],
		inst_regs->immediate,//inst_cmd->imm,
		inst_regs->r[2],//old_regs[2],
		inst_regs->r[3]//old_regs[3]
	);

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, "%s",line_to_print);
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

	check_ret = fprintf(file, "%s",line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

int print_line5(FILE* file, sp_t* sp)//, inst_trace* inst_cmd)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	switch (sp->spro->opcode)
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
			sp->spro->dst,
			sp->spro->alu0,
			opcode_name[sp->spro->opcode],
			sp->spro->alu1
		);
		break;

	case LHI:
		check_ret = sprintf(line_to_print,
			">>>> EXEC: R[%d] %s %d <<<<\n\n",
			sp->spro->dst,
			opcode_name[sp->spro->opcode],
			sp->spro->immediate
		);
		break;
	case LD:
		check_ret = sprintf(line_to_print,
			">>>> EXEC: R[%d] = MEM[%d] = %08x <<<<\n\n",
			sp->spro->dst,
			sp->spro->alu1,// the value of the memory address
			sp->sprn->r[sp->spro->dst] //the value in the memory address
		);
		break;

	case ST:
		check_ret = sprintf(line_to_print,
			">>>> EXEC: MEM[%d] = R[%d] = %08x <<<<\n\n",
			sp->spro->alu1,// the value of the memory address
			sp->spro->src0, // the register whose value we save 
			sp->spro->alu0 //the value in the memory address
		);
		break;

	case JLT:
	case JLE:
	case JEQ:
	case JNE:
		check_ret = sprintf(line_to_print,
			">>>> EXEC: %s %d, %d, %d <<<<\n\n",
			opcode_name[sp->spro->opcode],
			sp->spro->alu0,
			sp->spro->alu1,
			sp->sprn->pc
		);
		break;

	case JIN:
		check_ret = sprintf(line_to_print,
			">>>> EXEC: %s %d <<<<\n\n",
			opcode_name[sp->spro->opcode],
			sp->sprn->pc
		);
		break;

	case HLT:
		check_ret = sprintf(line_to_print,
			">>>> EXEC: HALT at PC %04x <<<<\n",
			sp->spro->pc
		);
		break;
	default:
		break;
	}

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, "%s",line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}

int end_trace(FILE* file, int cnt, int pc)
{
	int return_value = SUCCESS;

	int check_ret = 0;
	char line_to_print[MAX_STR_LEN];

	check_ret = sprintf(line_to_print,
		"sim finished at pc %d, %d instructions",
		pc+1,
		cnt + 1
	);

	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	check_ret = fprintf(file, "%s",line_to_print);
	if (check_ret < 0)
	{
		return_value = FAIL;
	}

	return return_value;
}
