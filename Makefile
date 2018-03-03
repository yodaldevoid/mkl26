OBJCOPY=arm-none-eabi-objcopy

OUTDIR=target/thumbv6m-none-eabi/release

BINARIES=blink
OUTPUT=$(addprefix $(OUTDIR)/,$(BINARIES))

all:: $(BINARIES)

.PHONY: flash_teensy
flash_teensy: $(OUTDIR)/$(BIN)
	teensy_loader_cli -w --mcu=mkl26z64 $(OUTDIR)/$(BIN) -v

.PHONY: $(BINARIES)
$(BINARIES): %: $(OUTDIR)/%

.PHONY: $(OUTPUT)
$(OUTPUT):
	xargo build --release --example $(notdir $@)

.PHONY: lib
lib:
	xargo build --release --lib

%.hex: %
	$(OBJCOPY) -O ihex $< $@

%.bin: %
	$(OBJCOPY) -O binary $< $@

%.srec: %
	$(OBJCOPY) -O srec $< $@
