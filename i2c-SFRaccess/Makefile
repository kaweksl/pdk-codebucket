SDCC=/home/me/repos/sdcc/bin/sdcc
EASYPDKPROG=/home/me/share-ja/repos/easy-pdk-programmer-software/easypdkprog

OUTNAME="i2c_SFRaccess"
OUTDIR=build
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
INCLUDE_DIR=$(ROOT_DIR)/../include
PEEP_DIR=$(ROOT_DIR)/../peephole_rules
GEN_SERIAL=$(ROOT_DIR)/../tools/genSerial.py
GENERATE_ID = $(shell ../tools/genSerial.py)
SET_ID = $(eval MY_ID=$(GENERATE_ID))

all: compile-PFS154 compile-PMS150C

compile-PFS154:
	mkdir -p $(OUTDIR)_PFS154
	$(SDCC) -DPFS154 -mpdk14 -I$(INCLUDE_DIR) --peep-file $(PEEP_DIR)/peephole_pdk14.def --fverbose-asm --peep-asm -o $(OUTDIR)_PFS154/$(OUTNAME).ihx main.c
	
	
compile-PMS150C:
	mkdir -p $(OUTDIR)_PMS150C
	$(SDCC) -DPMS150C -mpdk13 -I$(INCLUDE_DIR) --peep-file $(PEEP_DIR)/peephole_pdk13.def --fverbose-asm --peep-asm -o $(OUTDIR)_PMS150C/$(OUTNAME).ihx main.c

clean:
	rm -rf $(OUTDIR)_PFS154
	rm -rf $(OUTDIR)_PMS150C

erase:
	$(EASYPDKPROG) -nPFS154 erase

flash-PFS154:
	$(EASYPDKPROG) --verbose -nPFS154 write $(OUTDIR)_PFS154/$(OUTNAME).ihx

flash-PFS154-serial:
	$(SET_ID)
	$(EASYPDKPROG) --verbose --serial=$(MY_ID)  -nPFS154 write $(OUTDIR)_PFS154/$(OUTNAME).ihx

flash-PMS150C:
	$(EASYPDKPROG) --verbose -nPMS150C write $(OUTDIR)_PMS150C/$(OUTNAME).ihx


.PHONY: all clean flash-PFS154 flash-PMS150C erase compile-PMS150C compile-PFS154
