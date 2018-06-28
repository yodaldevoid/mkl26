OBJCOPY = arm-none-eabi-objcopy

OUTDIR = target/thumbv6m-none-eabi/release

OUTPUT = $(addprefix $(OUTDIR)/,$(BINARIES))

BUILD_FLAGS ?= ""

all:: $(BINARIES)

.PHONY: flash_teensy
flash_teensy: $(OUTDIR)/$(BIN)
	teensy_loader_cli -w --mcu=mkl26z64 $(OUTDIR)/$(BIN) -v

.PHONY: %
%: $(OUTDIR)/%

.PHONY: $(OUTDIR)/%
$(OUTDIR)/%:
	xargo build --release --example $(notdir $@) ${BUILD_FLAGS}

.PHONY: lib
lib:
	xargo build --release --lib ${BUILD_FLAGS}

%.hex: %
	$(OBJCOPY) -O ihex $< $@

%.bin: %
	$(OBJCOPY) -O binary $< $@

%.srec: %
	$(OBJCOPY) -O srec $< $@
