BIN=rusty-teensy
OUTDIR=target/thumbv7em-none-eabi/release
HEX=$(OUTDIR)/$(BIN).hex
ELF=$(OUTDIR)/$(BIN)

all:: $(ELF)

.PHONY: $(ELF)
$(ELF):
	xargo build --target=thumbv7em-none-eabi --release

$(HEX): $(ELF)
	arm-none-eabi-objcopy -O ihex $(ELF) $(HEX)

.PHONY: flash
flash: $(HEX)
	teensy_loader_cli -w --mcu=mk20dx256 $(HEX) -v