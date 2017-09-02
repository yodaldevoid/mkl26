OUTDIR=target/thumbv7em-none-eabi/release

TEENSY_3_1_HEX=$(OUTDIR)/teensy_3_1.hex

all:: teensy_3_1

.PHONY: teensy_3_1
teensy_3_1: $(TEENSY_3_1_HEX)

.PHONY: flash_teensy
flash_teensy: $(TEENSY_3_1_HEX)
	teensy_loader_cli -w --mcu=mk20dx256 $< -v

.PHONY: *.elf
$(OUTDIR)/teensy_3_1:
	xargo build --target=thumbv7em-none-eabi --release

%.hex: %
	arm-none-eabi-objcopy -O ihex $< $@

%.bin: %
	arm-none-eabi-objcopy -O binary $< $@
