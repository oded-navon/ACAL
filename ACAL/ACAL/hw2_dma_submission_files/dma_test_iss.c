static void dma_program(char *program_name)
{
	FILE *fp;
	int addr, i, last_addr;

	for (addr = 0; addr < MEM_SIZE; addr++)
		mem[addr] = 0;

	pc = 0;

	/*
	* this program is based on the add-shift approach to multiplying.
	*/
	asm_cmd(ADD, 2, 1, 0, -1300);
	asm_cmd(ADD, 3, 1, 0, 1400);
	asm_cmd(DMA, 2, 3, 1, 20);// false start of DMA - since values are crooked, will be ignored
	asm_cmd(POL, 2, 0, 0, 0);// check DMA - will return false
	asm_cmd(ADD, 2, 1, 0, 1300);// 0: R2 = Mem[1000], the multiplicand
	asm_cmd(DMA, 2, 3, 1, 20);// start DMA
	asm_cmd(DMA, 2, 3, 1, 30);// try to start another DMA, will not be executed
	asm_cmd(POL, 2, 0, 0, 0);// check DMA
	asm_cmd(LD, 2, 0,1, 1000);// R2 = Mem[1000], the multiplicand
	asm_cmd(ADD, 3, 2, 0, 0); //  R3 will be used as a temporary multiplicand
	asm_cmd(LD, 4,0,1, 1001); //  R4 = Mem[1001], the multiplier
	asm_cmd(ADD, 5, 4, 0, 0); //  R5 will be used as a temporary multiplier
	asm_cmd(ADD, 6, 0, 0, 0); //  R6 - the product
	asm_cmd(JEQ, 0, 5, 0, 20); // if multiplier is zero, then finish
	asm_cmd(AND, 7, 5, 1, 1); // R7 = R5 % 2
	asm_cmd(JEQ, 0, 7, 0, 17); // if current number is even, then we'll not add to product
	asm_cmd(ADD, 6, 6, 3, 0); // else, add multiplicand to product
	asm_cmd(LSF, 3, 3, 1, 1); // R3 << 1
	asm_cmd(RSF, 5, 5, 1, 1); // R5 >> 1
	asm_cmd(JEQ, 0, 0, 0, 13); // Jump to "Loop" label (row 5)
	asm_cmd(ST, 0, 6, 1, 1002); // Mem[1002] = R6
	asm_cmd(POL, 2, 0, 0, 0);//21: check DMA

	//now to the part that actually checks the values were copied correctly

	asm_cmd(ADD, 2, 1, 0, 20);//22: R2 = 20
	asm_cmd(ADD, 3, 1, 0, 1300); //23: R3 = source address
	asm_cmd(ADD, 4, 1, 0, 1400); //24: R4 = Dest address
	asm_cmd(LD, 5, 0, 3, 0); //25:	R5 = Mem[R3]
	asm_cmd(LD, 6, 0, 4, 0); //26: R6 = Mem[R4]
	asm_cmd(JNE, 0, 5, 6, 33); // if R3 != R4, we failed the test.
	asm_cmd(SUB, 2, 2, 1, 1); // R2--
	asm_cmd(JEQ, 0, 2, 0, 35); //29: if R2 == 0, then we're done - the test passed
	asm_cmd(ADD, 3, 3, 1, 1); // R3++
	asm_cmd(ADD, 4, 4, 1, 1); // R4++
	asm_cmd(JEQ, 0, 0, 0, 25); // Jump to start of loop
	asm_cmd(ADD, 2, 0, 0, 0);// 33: R2 = 0 (test failed)
	asm_cmd(JEQ, 0, 0, 0, 36); // Jump to start of loop
	asm_cmd(ADD, 2, 0, 1, 1);// 35: R2 = 1 (test passed)
	asm_cmd(HLT, 0, 0, 0, 0); // 36: halt

	mem[1000] = 5;
	mem[1001] = 9;
	for (i = 0; i < 30; i++)
	{
		mem[1300 + i] = i;
	}

	last_addr = 1422;

	fp = fopen(program_name, "w");
	if (fp == NULL) {
		printf("couldn't open file %s\n", program_name);
		exit(1);
	}
	addr = 0;
	while (addr < last_addr) {
		fprintf(fp, "%08x\n", mem[addr]);
		addr++;
	}
}