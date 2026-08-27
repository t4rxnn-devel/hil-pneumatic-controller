# =====================================================================
# @file Makefile
# @brief PlatformIO Build & Flash Orchestrator for HIL Pneumatic Controller
# =====================================================================

TARGET_ENV = teensy41

all: build

build:
	@echo "[MAKE] Compiling embedded firmware for target: $(TARGET_ENV)..."
	pio run -e $(TARGET_ENV)

flash:
	@echo "[MAKE] Uploading firmware binary to target MCU..."
	pio run -e $(TARGET_ENV) -t upload

clean:
	@echo "[MAKE] Cleaning build artifacts..."
	pio run -e $(TARGET_ENV) -t clean

monitor:
	@echo "[MAKE] Opening high-speed serial telemetry monitor (500000 baud)..."
	pio device monitor -b 500000
