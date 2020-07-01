SDCC=/home/me/repos/sdcc/bin/sdcc
EASYPDKPROG=/home/me/repos/easy-pdk-programmer-software/easypdkprog

OUTDIR=build
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
INCLUDE_DIR=$(ROOT_DIR)/../include
PEEP_DIR=$(ROOT_DIR)/../peephole_rules


all: compile-PFS154 compile-PMS150C

compile-PFS154:
	mkdir -p $(OUTDIR)_PFS154
	
	$(SDCC) -DPFS154 -mpdk14 -I$(INCLUDE_DIR) --peep-file $(PEEP_DIR)/peephole_pdk14.def --fverbose-asm --peep-asm -o $(OUTDIR)_PFS154/fancontroller.ihx fancontroller.c
	
compile-PMS150C:
	mkdir -p $(OUTDIR)_PMS150C
	$(SDCC) -DPMS150C -mpdk13 -I$(INCLUDE_DIR)  --peep-file $(PEEP_DIR)/peephole_pdk13.def --fverbose-asm --peep-asm -o $(OUTDIR)_PMS150C/fancontroller.ihx fancontroller.c

clean:
	rm -rf $(OUTDIR)_PFS154
	rm -rf $(OUTDIR)_PMS150C

erase:
	$(EASYPDKPROG) -nPFS154 erase

flash-PFS154:
	$(EASYPDKPROG) --verbose -nPFS154 write $(OUTDIR)_PFS154/fancontroller.ihx

flash-PMS150C:
	$(EASYPDKPROG) --verbose -nPMS150C write $(OUTDIR)_PMS150C/fancontroller.ihx


.PHONY: all clean flash-PFS154 flash-PMS150C erase compile-PMS150C compile-PFS154
