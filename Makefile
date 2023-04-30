ARCH := riscv64imac
LOG  := info
RELEASE := release
KERNEL_ELF = target/$(ARCH)-unknown-none-elf/$(RELEASE)/kernel
# SBI	:= tools/rustsbi-qemu.bin
FS_IMG  := mount.img
SBI := tools/rustsbi-k210.bin
QEMU_EXEC := qemu-system-riscv64 \
				-machine virt \
				-kernel $(KERNEL_ELF) \
				-m 128M \
				-bios $(SBI) \
				-drive file=$(FS_IMG),if=none,format=raw,id=x0 \
        		-device virtio-blk-device,drive=x0,bus=virtio-mmio-bus.0 \
				-nographic \
				-smp 2
RUST_BUILD_OPTIONS := 
K210-SERIALPORT	= /dev/ttyUSB0
K210-BURNER	= tools/k210/kflash.py
BIN_FILE = byteos.bin

ifeq ($(RELEASE), release)
	RUST_BUILD_OPTIONS += --release
endif

all: 
	RUST_BACKTRACE=1 LOG=$(LOG) cargo build $(RUST_BUILD_OPTIONS) --offline
	cp $(SBI) sbi-qemu
	cp $(KERNEL_ELF) kernel-qemu

fs-img:
	rm -f $(FS_IMG)
	dd if=/dev/zero of=$(FS_IMG) bs=1M count=40
	mkfs.vfat -F 32 $(FS_IMG)
	sudo mount $(FS_IMG) mount/ -o uid=1000,gid=1000
	cp -r tools/testcase-step2/* mount/
	sudo umount $(FS_IMG)

build: fs-img
	cp .cargo/linker-qemu.ld .cargo/linker-riscv.ld
	RUST_BACKTRACE=1 LOG=$(LOG) cargo build $(RUST_BUILD_OPTIONS) $(OFFLINE)

k210-build: 
	cp .cargo/linker-k210.ld .cargo/linker-riscv.ld
	RUST_BACKTRACE=1 LOG=$(LOG) cargo build $(RUST_BUILD_OPTIONS) $(OFFLINE)
	rust-objcopy --binary-architecture=riscv64 $(KERNEL_ELF) --strip-all -O binary $(BIN_FILE)
	@cp $(SBI) $(SBI).copy
	@dd if=$(BIN_FILE) of=$(SBI).copy bs=131072 seek=1
	@mv $(SBI).copy $(BIN_FILE)

run: build
	$(QEMU_EXEC)

debug: build
	@tmux new-session -d \
	"$(QEMU_EXEC) -s -S && echo '按任意键继续' && read -n 1" && \
	tmux split-window -h "riscv64-elf-gdb -ex 'file $(KERNEL_ELF)' -ex 'set arch riscv:rv64' -ex 'target remote localhost:1234'" && \
	tmux -2 attach-session -d

clean:
	rm -rf target/

gdb:
	riscv64-elf-gdb \
        -ex 'file $(KERNEL_ELF)' \
        -ex 'set arch riscv:rv64' \
        -ex 'target remote localhost:1234'

flash: k210-build
	(which $(K210-BURNER)) || (cd tools && git clone https://github.com/sipeed/kflash.py.git k210)
	@sudo chmod 777 $(K210-SERIALPORT)
	python3 $(K210-BURNER) -p $(K210-SERIALPORT) -b 1500000 $(BIN_FILE)
	python3 -m serial.tools.miniterm --eol LF --dtr 0 --rts 0 --filter direct $(K210-SERIALPORT) 115200

addr2line:
	addr2line -sfipe $(KERNEL_ELF) | rustfilt

.PHONY: all run build clean gdb
