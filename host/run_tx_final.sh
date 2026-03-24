#!/bin/bash

# Путь к скомпилированной программе (относительно папки сборки или текущей папки)
UHD_TX_PATH="./tx_samples_from_file"

# Если бинарника нет в текущей папке, пробуем найти его в папке build
if [ ! -f "$UHD_TX_PATH" ]; then
    UHD_TX_PATH="./build/examples/tx_samples_from_file"
fi

# Проверка наличия бинарника
if [ ! -f "$UHD_TX_PATH" ]; then
    echo "Ошибка: Файл tx_samples_from_file не найден. Сначала выполните компиляцию."
    exit 1
fi

# Даем права на запуск бинарнику
chmod +x "$UHD_TX_PATH"

echo "--- Настройка системы для ANTSDR (B210/AD9364) ---"

# 1. Лимит памяти USB (убирает Sequence Errors 'S')
echo 1000 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb

# 2. Режим процессора (Performance для стабильного потока)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# echo "--- Запуск передачи ---"

# Запуск. Путь к файлу signal.bin должен быть верным относительно места запуска скрипта.
#success sudo taskset -c 0 nice -n -20 "$UHD_TX_PATH" --args "priority=high,num_send_frames=512,send_frame_size=4096,master_clock_rate=50e6" --file "signal.bin" --type short --rate 2500000 --freq 1575420000 --gain 90 --repeat --ant 1 --ref internal --wirefmt sc16 --spb 8192 #--eavg 20 --econd 30000

sudo taskset -c 0 nice -n -20 "$UHD_TX_PATH" --args "master_clock_rate=50e6" --file "signal.bin" --type short --rate 2500000 --freq 1575420000 --gain 90 --repeat --ant 1 --ref internal --wirefmt sc16 --spb 8192 #--eavg 20 --econd 30000