#!/usr/bin/env python3
"""Build and inspect only. Never connects to or flashes hardware.

STM32 half of the standalone Mega <-> STM32 UART link test (see README.md).
Runs the shared-protocol host test first and stops if it fails, then
cross-compiles for a fresh WeAct STM32F401CCU6 at 0x08000000 (ROM DFU
bootloader, no WeAct HID bootloader assumed) and checks the image.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

root = Path(__file__).resolve().parent
repo = root.parent.parent
proto = repo / 'src' / 'Bench' / 'UartLinkTest'
out = root / 'dist'
out.mkdir(exist_ok=True)

host_cc = os.environ.get('HOST_CC') or shutil.which('gcc') or shutil.which('cc')
if not host_cc:
    raise SystemExit('Install a host C compiler (gcc/cc) to run the unit tests.')
with tempfile.TemporaryDirectory() as tmp:
    host_bin = Path(tmp) / 'host_test_ulink'
    subprocess.run([host_cc, '-Wall', '-Wextra', '-Werror', '-std=c11',
                    '-I', str(proto),
                    str(root / 'test/host_test_ulink.c'), str(proto / 'ulink.c'),
                    '-o', str(host_bin)], check=True)
    subprocess.run([str(host_bin)], check=True)

compiler = os.environ.get('ARM_GCC') or shutil.which('arm-none-eabi-gcc')
if not compiler:
    candidate = Path.home() / '.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc'
    if candidate.exists():
        compiler = str(candidate)
if not compiler:
    raise SystemExit('Install ARM GNU toolchain; set ARM_GCC to arm-none-eabi-gcc.')
prefix = str(Path(compiler).resolve()).removesuffix('gcc')

APP_BASE = 0x08000000
name = 'stm32f401cc_mega_uart_test'
elf = out / f'{name}.elf'
binary = out / f'{name}.bin'
flags = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-Os',
         '-ffreestanding', '-fno-builtin', '-fno-tree-loop-distribute-patterns',
         '-ffunction-sections', '-fdata-sections',
         '-Wall', '-Wextra', '-Werror', '-nostdlib', '-nostartfiles',
         '-I', str(root / 'src'), '-I', str(proto),
         '-Wl,--gc-sections', '-Wl,--build-id=none',
         '-Wl,-Map=' + str(out / f'{name}.map')]
subprocess.run([compiler, *flags, '-T', str(root / 'linker.ld'),
                str(root / 'src/startup.S'), str(root / 'src/main.c'),
                str(proto / 'ulink.c'), '-o', str(elf)], check=True)
subprocess.run([prefix + 'objcopy', '-O', 'binary', str(elf), str(binary)], check=True)
headers = subprocess.check_output([prefix + 'objdump', '-h', str(elf)], text=True)
(out / 'disassembly.txt').write_text(
    subprocess.check_output([prefix + 'objdump', '-d', str(elf)], text=True))

data = binary.read_bytes()
stack, reset = struct.unpack_from('<II', data)
assert stack == 0x20010000, hex(stack)
assert reset & 1, 'Reset entry must select Thumb mode'
assert APP_BASE <= (reset & ~1) < APP_BASE + len(data)
assert len(data) <= 256 * 1024

for line in headers.splitlines():
    columns = line.split()
    if len(columns) >= 6 and columns[1] == '.isr_vector':
        assert int(columns[3], 16) == int(columns[4], 16) == APP_BASE
        break
else:
    raise AssertionError('Missing vector section')

vectors = struct.unpack_from('<102I', data)
assert all(v & 1 and APP_BASE <= (v & ~1) < APP_BASE + len(data)
           for v in vectors[1:]), 'Invalid exception handler pointer'
assert vectors[15] != vectors[2], 'SysTick_Handler must not fall back to Default_Handler'
assert vectors[53] != vectors[2], 'USART1_IRQHandler must not fall back to Default_Handler'

undefined = subprocess.check_output([prefix + 'nm', '-u', str(elf)], text=True)
assert not undefined.strip(), 'Undefined symbols (libc call slipped in?):\n' + undefined

report = {
    'compiler': subprocess.check_output([compiler, '--version'], text=True).splitlines()[0],
    'image_base': hex(APP_BASE),
    'flash_method': 'ROM USB DFU (BOOT0) -- see README.md; no HID bootloader assumed',
    'uart': 'USART1 PA9 TX / PA10 RX, 115200 8N1 (BRR 0x8B)',
    'initial_stack': hex(stack), 'reset_vector': hex(reset),
    'usart1_irq_vector': hex(vectors[53]), 'systick_vector': hex(vectors[15]),
    'binary_bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
    'checks': ['host ulink protocol + link simulation test', 'vector VMA/LMA at 0x08000000',
               'Thumb handler pointers', 'SysTick/USART1 vectors wired',
               'stack bounds', 'image size', 'no undefined symbols'],
    'hardware_tested': False, 'flashed': False,
}
(out / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
(out / 'sections.txt').write_text(headers)
print(json.dumps(report, indent=2))
