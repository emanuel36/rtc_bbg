CROSS_COMPILE ?= arm-linux-gnueabihf-

all: BM_BBG.s
	$(CROSS_COMPILE)as BM_BBG.s -o BM_BBG.o
	$(CROSS_COMPILE)ld -o BM_BBG -T memmap BM_BBG.o
	$(CROSS_COMPILE)objcopy BM_BBG BM_BBG.bin -O binary
	mv BM_BBG.bin /servers/tftpboot/teste.bin
	rm *.o
	rm BM_BBG

clean:
	rm *.o *.bin *.lst
