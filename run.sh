#./build/RISCV/gem5.opt --debug-flags=O3CPUAll --debug-file=o3_trace.out ./configs/example/se.py --cmd=/home/yangqinghong/coremark.riscv --cpu-type=DerivO3CPU --caches
#./build/RISCV/gem5.opt --debug-flags=O3CPUAll --debug-file=o3_trace.out ./configs/example/se.py --cmd=/home/yangqinghong/dhrystone-src/dhrystone --cpu-type=DerivO3CPU --caches
./build/RISCV/gem5.opt --debug-flags=VirtualReg,PhysReg --debug-file=virphys_trace.out ./configs/example/se.py --cmd=tests/test-progs/hello/bin/riscv/linux/hello --cpu-type=DerivO3CPU --caches
#./build/RISCV/gem5.opt --debug-flags=StaticCntRef --debug-file=intrc_trace.out ./configs/example/se.py --cmd=/home/yangqinghong/rocket-chip-finish/emulator/hello.riscv --cpu-type=DerivO3CPU --caches
