# Sample RTL for virtual-bus. The Rust demo (examples/rtl-demo) models it with Verilator
#
#   make lint   lint the RTL with verilator -Wall
#   make sim    run the unit testbenches with Icarus Verilog
#   make all    both

RTL := rtl
TB := tb
BUILD := build

.PHONY: all lint sim clean

all: lint sim

lint:
	verilator --lint-only -Wall $(RTL)/i2c_whoami.v --top-module i2c_whoami
	verilator --lint-only -Wall $(RTL)/i2c_whoami.v $(RTL)/i2c_whoami_top.v --top-module i2c_whoami_top
	verilator --lint-only -Wall $(RTL)/i2c_whoami.v $(RTL)/i2c_whoami_top.v $(RTL)/sim/i2c_whoami_sim.v --top-module i2c_whoami_sim
	verilator --lint-only -Wall $(RTL)/spi_counter.v --top-module spi_counter

SIMS := tb_i2c_whoami tb_i2c_whoami_top

sim: $(SIMS:%=$(BUILD)/%.vvp)
	@for s in $(SIMS); do vvp -n $(BUILD)/$$s.vvp | grep -E 'PASS|FAIL'; done

$(BUILD)/tb_i2c_whoami_top.vvp: $(TB)/tb_i2c_whoami.v $(RTL)/i2c_whoami.v $(RTL)/i2c_whoami_top.v | $(BUILD)
	iverilog -g2012 -DUSE_TOP -s tb_i2c_whoami -o $@ $^

$(BUILD)/tb_%.vvp: $(TB)/tb_%.v $(RTL)/%.v | $(BUILD)
	iverilog -g2012 -s tb_$* -o $@ $^

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
