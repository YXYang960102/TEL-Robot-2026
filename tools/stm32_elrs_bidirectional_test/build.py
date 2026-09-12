#!/usr/bin/env python3
"""Build and inspect only. Never connects to or flashes hardware.

Runs the host-side CRSF protocol unit test first (native gcc, no embedded
toolchain involved) and stops before cross-compiling if it fails, per the
project's staged verification discipline: prove the protocol layer on the
host before it ever runs on the target.
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
out = root / 'dist'
out.mkdir(exist_ok=True)

host_cc = os.environ.get('HOST_CC') or shutil.which('gcc') or shutil.which('cc')
if not host_cc:
    raise SystemExit('Install a host C compiler (gcc/cc) to run the CRSF unit test.')
with tempfile.TemporaryDirectory() as tmp:
    host_test_bin = Path(tmp) / 'host_test_crsf'
    subprocess.run([host_cc, '-Wall', '-Wextra', '-Werror', '-std=c11',
                     str(root / 'test/host_test_crsf.c'), str(root / 'src/crsf.c'),
                     '-o', str(host_test_bin)], check=True)
    subprocess.run([str(host_test_bin)], check=True)

compiler = os.environ.get('ARM_GCC') or shutil.which('arm-none-eabi-gcc')
if not compiler:
    candidate = Path.home() / '.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc'
    if candidate.exists():
        compiler = str(candidate)
if not compiler:
    raise SystemExit('Install ARM GNU toolchain; set ARM_GCC to arm-none-eabi-gcc.')
prefix = str(Path(compiler).resolve()).removesuffix('gcc')

name = 'stm32f401cc_elrs_bidir_test'
elf = out / f'{name}.elf'
binary = out / f'{name}.bin'
flags = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-Os',
         '-ffreestanding', '-fno-builtin', '-ffunction-sections', '-fdata-sections',
         '-Wall', '-Wextra', '-Werror', '-nostdlib', '-nostartfiles',
         '-Wl,--gc-sections', '-Wl,--build-id=none',
         '-Wl,-Map=' + str(out / f'{name}.map')]
subprocess.run([compiler, *flags, '-T', str(root / 'linker.ld'),
                str(root / 'src/startup.S'), str(root / 'src/main.c'),
                str(root / 'src/crsf.c'),
                '-o', str(elf)], check=True)
subprocess.run([prefix + 'objcopy', '-O', 'binary', str(elf), str(binary)], check=True)
headers = subprocess.check_output([prefix + 'objdump', '-h', str(elf)], text=True)
disassembly = subprocess.check_output([prefix + 'objdump', '-d', str(elf)], text=True)
(out / 'disassembly.txt').write_text(disassembly)

data = binary.read_bytes()
stack, reset = struct.unpack_from('<II', data)
assert stack == 0x20010000, hex(stack)
assert reset & 1, 'Reset entry must select Thumb mode'
assert 0x08004000 <= (reset & ~1) < 0x08004000 + len(data)
assert len(data) <= 240 * 1024

for line in headers.splitlines():
    columns = line.split()
    if len(columns) >= 6 and columns[1] == '.isr_vector':
        assert int(columns[3], 16) == int(columns[4], 16) == 0x08004000
        break
else:
    raise AssertionError('Missing vector section')

vectors = struct.unpack_from('<102I', data)
assert all(v & 1 and 0x08004000 <= (v & ~1) < 0x08004000 + len(data)
           for v in vectors[1:]), 'Invalid exception handler pointer'
# Confirm the two interrupt handlers we actually depend on are wired to the
# addresses this build expects at the exact word offsets computed from
# USART1_IRQn=37 / USART2_IRQn=38 (verified against the ST cmsis_device_f4
# stm32f401xc.h IRQn_Type enum, see startup.S).
assert vectors[15] != vectors[2], 'SysTick_Handler must not fall back to Default_Handler'
assert vectors[53] != vectors[2], 'USART1_IRQHandler must not fall back to Default_Handler'
assert vectors[54] != vectors[2], 'USART2_IRQHandler must not fall back to Default_Handler'
assert vectors[53] != vectors[54], 'USART1 and USART2 vectors must not collide'

undefined = subprocess.check_output([prefix + 'nm', '-u', str(elf)], text=True)
assert not undefined.strip(), undefined

report = {
    'compiler': subprocess.check_output([compiler, '--version'], text=True).splitlines()[0],
    'application_base': '0x08004000', 'reserved_bootloader_bytes': 16384,
    'application_limit_bytes': 245760, 'ram_limit_bytes': 65536,
    'initial_stack': hex(stack), 'reset_vector': hex(reset),
    'usart1_irq_vector': hex(vectors[53]), 'usart2_irq_vector': hex(vectors[54]),
    'systick_vector': hex(vectors[15]),
    'binary_bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
    'checks': ['host CRSF protocol unit test', 'vector VMA/LMA',
               'Thumb handler pointers', 'SysTick/USART1/USART2 vectors wired',
               'stack bounds', 'image size', 'no undefined symbols'],
    'hardware_tested': False, 'flashed': False,
}
(out / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
(out / 'sections.txt').write_text(headers)
print(json.dumps(report, indent=2))
