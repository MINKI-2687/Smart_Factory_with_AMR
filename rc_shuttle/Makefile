# RC 셔틀 - 분리 컴파일 빌드 (2026-08-11 리팩토링)
#
#   make            # main_shuttle 빌드 (기본)
#   make all        # main_shuttle + 브링업 도구 + 테스트 + 시뮬용 .so
#   make tools      # tools/ 의 하드웨어 브링업 실행파일
#   make tests      # tests/ 의 검증 실행파일
#   make libs       # sim/ 파이썬 ctypes 용 공유 라이브러리
#   make run        # live_view.py 로 실기 실행 (RUNARGS 로 옵션 조정)
#   make clean
#
# 예전 구조와 달라진 점:
#   - 헤더는 include/ 에 '선언만', 본문은 src/ 에 있다. 헤더를 고쳐도
#     그 헤더를 쓰는 파일만 다시 컴파일된다.
#   - src/ 전체는 build/libshuttle.a 로 묶인다. tests/ 와 sim/ 은 이걸 링크한다.
#   - _POSIX_C_SOURCE 는 예전엔 robot_runner.h 맨 위에서 정의했는데,
#     이제 각 .c 가 독립 컴파일되므로 CFLAGS 로 옮겼다. 순서 의존이 사라진다.

CC      = gcc
CSTD    = -std=c11
WARN    = -Wall -Wextra
OPT     = -O2
DEFS    = -D_POSIX_C_SOURCE=200809L
INC     = -Iinclude
CFLAGS  = $(CSTD) $(OPT) $(WARN) $(DEFS) $(INC)
LDLIBS  = -lm -lpthread

# 헤더 온리 시절엔 전부 static inline 이라 컴파일러가 마음껏 인라인했다.
# 분리 컴파일하면 그게 끊기므로 LTO 로 되살린다. 문제가 생기면 LTO= 로 끄면 된다.
LTO     = -flto
CFLAGS += $(LTO)
LDFLAGS = $(LTO) $(OPT)

BUILD   = build
LIB     = $(BUILD)/libshuttle.a

SRCS    = $(wildcard src/*.c)
OBJS    = $(patsubst src/%.c,$(BUILD)/%.o,$(SRCS))
DEPS    = $(OBJS:.o=.d)

TOOL_SRCS = $(wildcard tools/*.c)
TOOL_BINS = $(patsubst tools/%.c,$(BUILD)/%,$(TOOL_SRCS))

TEST_SRCS = $(wildcard tests/*.c)
TEST_BINS = $(patsubst tests/%.c,$(BUILD)/%,$(TEST_SRCS))

# 파일명 규칙이 아니라 파이썬 래퍼가 dlopen 하는 이름에 맞춘다.
#   slam_cwrap.py -> libslam.so, graph_slam_cwrap.py -> libgraphslam.so ...
SO_LIBS = sim/libslam.so sim/libnav.so sim/libfrontier.so \
          sim/libgraphslam.so sim/libsession.so

.PHONY: default all tools tests libs run clean help
.DEFAULT_GOAL := default

default: main_shuttle

all: main_shuttle tools tests libs

# ---- 본체 ----
main_shuttle: $(BUILD)/main_shuttle
	@cp $< $@

$(BUILD)/main_shuttle: $(BUILD)/main_shuttle.o $(LIB)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(LIB): $(OBJS)
	@mkdir -p $(BUILD)
	ar rcs $@ $^

$(BUILD)/%.o: src/%.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -MMD -MP -c -o $@ $<

$(BUILD)/main_shuttle.o: app/main_shuttle.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -MMD -MP -c -o $@ $<

# ---- 하드웨어 브링업 도구 ----
# 라이다/시리얼만 쓰는 것도 있고 프로젝트 헤더를 넓게 쓰는 것도 있어서
# 전부 libshuttle.a 를 링크한다. 안 쓰는 심볼은 링커가 버린다.
tools: $(TOOL_BINS)

$(BUILD)/%: tools/%.c $(LIB)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $< $(LIB) $(LDLIBS)

# ---- 검증 실행파일 ----
# 예전 Makefile 의 align_test 타겟은 test_slot_align_room.c 를 요구했는데
# 그 파일이 저장소에 없어서 make 가 그 자리에서 죽었다. wildcard 로 바꿔서
# tests/ 에 있는 것만 빌드하도록 했다. (test_slam 타겟도 규칙이 없어 깨져 있었다.)
tests: $(TEST_BINS)

$(BUILD)/%: tests/%.c $(LIB)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $< $(LIB) $(LDLIBS)

# ---- 파이썬 시뮬레이터용 ctypes 공유 라이브러리 ----
# 예전엔 헤더 온리라서 *_capi.c 하나만 컴파일하면 됐다. 이제 본문이 src/ 에
# 있으므로 -fPIC 로 다시 빌드한 오브젝트를 같이 넣어야 한다.
libs: $(SO_LIBS)

$(BUILD)/pic/%.o: src/%.c
	@mkdir -p $(BUILD)/pic
	$(CC) $(CFLAGS) -fPIC -c -o $@ $<

PIC_OBJS = $(patsubst src/%.c,$(BUILD)/pic/%.o,$(SRCS))

# 래퍼가 자기 옆(sim/)에서 .so 를 찾으므로 결과물도 sim/ 에 둔다.
sim/libslam.so:      sim/slam_capi.c            $(PIC_OBJS)
sim/libnav.so:       sim/nav_capi.c             $(PIC_OBJS)
sim/libfrontier.so:  sim/frontier_capi.c        $(PIC_OBJS)
sim/libgraphslam.so: sim/graph_slam_capi.c      $(PIC_OBJS)
sim/libsession.so:   sim/mapping_session_capi.c $(PIC_OBJS)

sim/lib%.so:
	$(CC) $(CFLAGS) -Isim -shared -fPIC -o $@ $< $(PIC_OBJS) $(LDLIBS)

# ---- 실행 ----
RUNARGS ?= --map-source load --load-map maps/set_map.txt \
           --grid-size 1.5 0.9 --lidar-mirror --slot-parking \
           --lidar-offset 0.06 --slot-align-tol 2.5 \
           --robot-radius 0.14 --slot-staging 0.60 --slot-fast 0.4275 2.5 \
           --trigger-gpio-lines 17,27,22,23 --trigger-gpio-chip /dev/gpiochip0 \
           --trigger-allow-enter \
           --fpga-device /dev/ttyUSB2 --lidar-device /dev/ttyUSB0 \
           --point-a 0.225 0.177 -90 --point-b 1.275 0.177 -90 --sg-settle 0.12

run: main_shuttle
	python3 viewer/live_view.py --run "./main_shuttle $(RUNARGS)"

clean:
	rm -rf $(BUILD) main_shuttle sim/*.so

help:
	@sed -n '1,20p' Makefile

-include $(DEPS)
