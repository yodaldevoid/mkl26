OBJCOPY=arm-none-eabi-objcopy

OUTDIR=target/thumbv7em-none-eabi/release

BINARIES=teensy_3_1 frdm_k64f
OUTPUT=$(addprefix $(OUTDIR)/,$(BINARIES))

TEENSY_3_1_HEX=$(OUTDIR)/teensy_3_1.hex

all:: $(BINARIES)

.PHONY: flash_teensy
flash_teensy: $(TEENSY_3_1_HEX)
	teensy_loader_cli -w --mcu=mk20dx256 $< -v

.PHONY: flash_frdm_k64f
flash_frdm_k64f: frdm_k64f
	udisksctl mount -b  /dev/disk/by-label/MBED
	cp $(OUTDIR)/$< /media/$(USER)/MBED/ && sync; udisksctl unmount -b  /dev/disk/by-label/MBED

.PHONY: $(BINARIES)
$(BINARIES): %: $(OUTDIR)/%

.PHONY: $(OUTPUT)
$(OUTPUT):
	xargo build --target=thumbv7em-none-eabi --release --bin $(notdir $@)

%.hex: %
	$(OBJCOPY) -O ihex $< $@

%.bin: %
	$(OBJCOPY) -O binary $< $@

%.srec: %
	$(OBJCOPY) -O srec $< $@
