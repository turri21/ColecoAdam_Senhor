V = verilator
COSIM = n

TOP = --top-module emu
RTL = ../rtl
V_INC = +incdir+$(RTL)

V_DEFINE = +define+debug=1 +define+SIMULATION=1   -CFLAGS "-I../sim/imgui -I../sim/vinc -I../sim/ -g "  --timescale-override 1ps/1ps -Wno-TIMESCALEMOD \
	-I../$(RTL) \
	-I../$(RTL)/vdp18v \
	-I../$(RTL)/tv80 \
	-I../$(RTL)/sn76489 \
        --timing
#V_DEFINE += --converge-limit 2000 -Wno-WIDTH -Wno-IMPLICIT -Wno-MODDUP -Wno-UNSIGNED -Wno-CASEINCOMPLETE -Wno-CASEX -Wno-SYMRSVDWORD -Wno-COMBDLY -Wno-INITIALDLY -Wno-BLKANDNBLK -Wno-UNOPTFLAT -Wno-SELRANGE -Wno-CMPCONST -Wno-CASEOVERLAP -Wno-PINMISSING -Wno-MULTIDRIVEN
#V_DEFINE += --threads 8  # this slows it way down
V_DEFINE += --trace-fst
V_DEFINE +=

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S), Darwin) #APPLE
	ECHO_MESSAGE = "Mac OS X"
	LIBS += -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo `sdl2-config --libs`
	LIBS += -L/usr/local/lib -L/opt/local/lib

	CXXFLAGS += `sdl2-config --cflags` -Iimgui
	CXXFLAGS += `sdl2-config --cflags | sed 's|include/SDL2|include/|'`
	CXXFLAGS += -I/usr/local/include -I/opt/local/include
	CFLAGS = $(CXXFLAGS) -Iimgui
endif

ifeq ($(UNAME_S), Linux) #LINUX
	ECHO_MESSAGE = "Linux"
	LIBS += -lGL -ldl `sdl2-config --libs`

	CXXFLAGS += `sdl2-config --cflags` -Iimgui
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(findstring MINGW,$(UNAME_S)),MINGW)
	ECHO_MESSAGE = "MinGW"
	LIBS += -lgdi32 -lopengl32 -limm32 `pkg-config --static --libs sdl2`

	CXXFLAGS += `pkg-config --cflags sdl2`
	CFLAGS = $(CXXFLAGS)
endif

CFLAGS += $(CC_OPT) $(CC_DEFINE) -Iimgui
LDFLAGS = $(LIBS)
EXE = ./obj_dir/Vemu
#V_OPT = -O3 --x-assign fast --x-initial fast --noassert
V_OPT =  --x-assign fast --x-initial fast --noassert
#CC_OPT = -O
CC_OPT =

V_SRC = \
	sim.v \
	$(RTL)/spram.sv \
	rtl/YM2149.sv \
	../rtl/cv_clock.sv \
	../rtl/cv_ctrl.sv \
	../rtl/sdram.sv \
	../rtl/cv_addr_dec.sv \
	../rtl/cv_console.sv \
	../rtl/cv_adamnet.sv \
	../rtl/cv_por.sv	 \
	../rtl/vdp18v/vdp18_pkg.sv \
	../rtl/vdp18v/vdp18_clk_gen.sv \
	../rtl/vdp18v/vdp18_core.sv \
	../rtl/vdp18v/vdp18_addr_mux.sv\
	../rtl/vdp18v/vdp18_col_mux.sv\
	../rtl/vdp18v/vdp18_cpuio.sv\
	../rtl/vdp18v/vdp18_ctrl.sv\
	../rtl/vdp18v/vdp18_hor_vert.sv\
	../rtl/vdp18v/vdp18_pattern.sv\
	../rtl/vdp18v/vdp18_sprite.sv\
	../rtl/sn76489/sn76489_attenuator.sv \
	../rtl/sn76489/sn76489_clock_div.sv \
	../rtl/sn76489/sn76489_latch_ctrl.sv \
	../rtl/sn76489/sn76489_noise.sv \
	../rtl/sn76489/sn76489_tone.sv \
	../rtl/sn76489/sn76489_top.sv \
	$(RTL)/tv80/tv80_alu.v  $(RTL)/tv80/tv80_core.v  $(RTL)/tv80/tv80_mcode.v  $(RTL)/tv80/tv80_reg.v  $(RTL)/tv80/tv80e.v



#	AdamNet.c ColGlue.c  EMULib/FDIDisk.c sim_main.cpp
C_SRC = \
        ColGlue.c  EMULib/FDIDisk.c \
	sim_main.cpp  \
	sim/sim_bus.cpp sim/sim_blkdevice.cpp sim/sim_clock.cpp sim/sim_console.cpp sim/sim_video.cpp sim/sim_console.cpp sim/sim_input.cpp  sim/sim_audio.cpp \
	sim/imgui/imgui_impl_sdl.cpp sim/imgui/imgui_impl_opengl2.cpp sim/imgui/imgui_draw.cpp sim/imgui/imgui_widgets.cpp sim/imgui/imgui_tables.cpp sim/imgui/imgui.cpp sim/imgui/ImGuiFileDialog.cpp sim/imgui/implot.cpp sim/imgui/implot_items.cpp

VOUT = obj_dir/Vemu.cpp

all: $(EXE)

$(VOUT): $(V_SRC)  Makefile
	$V -cc $(V_OPT) -LDFLAGS "$(LDFLAGS) " -exe --trace --Mdir ./obj_dir $(V_DEFINE) $(V_INC) $(TOP) -CFLAGS $(CFLAGS) $(V_SRC) $(C_SRC)

$(EXE): $(VOUT) $(C_SRC)
#	(cd obj_dir; make OPT="-fauto-inc-dec -fdce -fdefer-pop -fdse -ftree-ccp -ftree-ch -ftree-fre -ftree-dce -ftree-dse" -f Vemu.mk)
	(cd obj_dir; make -f Vemu.mk)

fast:
	(cd obj_dir; rm -f *.o ; make OPT="-fcompare-elim -fcprop-registers -fguess-branch-probability -fauto-inc-dec -fif-conversion2 -fif-conversion -fipa-pure-const -fdce -fipa-profile -fipa-reference -fmerge-constants -fsplit-wide-types -fdefer-pop -fdse -ftree-ccp -ftree-ch -ftree-fre -ftree-dce -ftree-dse -ftree-builtin-call-dce -ftree-copyrename -ftree-dominator-opts -ftree-forwprop -ftree-phiprop -ftree-sra -ftree-pta -ftree-ter -funit-at-a-time -ftree-bit-ccp -falign-functions  -falign-jumps -falign-loops  -falign-labels -fcaller-saves -fcrossjumping -fcse-follow-jumps -fcse-skip-blocks -fdelete-null-pointer-checks -fdevirtualize -fexpensive-optimizations -fgcse  -fgcse-lm -finline-small-functions -findirect-inlining -fipa-sra -foptimize-sibling-calls -fpartial-inlining -fpeephole2 -fregmove -freorder-blocks  -freorder-functions -frerun-cse-after-loop -fsched-interblock  -fsched-spec -fschedule-insns -fschedule-insns2 -fstrict-aliasing -fstrict-overflow -ftree-switch-conversion -ftree-pre -ftree-vrp" -f Vemu.mk)

clean:
	rm -f obj_dir/*
