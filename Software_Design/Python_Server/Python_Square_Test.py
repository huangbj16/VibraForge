import asyncio
import time
import bleak
from bleak import BleakScanner, BleakClient

CHARACTERISTIC_UUID = 'f22535de-5375-44bd-8ca9-d0ea9ff9e410'
CONTROL_UNIT_NAME = 'QT Py ESP32-S3'  # name advertised by the main-branch controller (BLE_peripheral_QTPyS3)
SCAN_TIMEOUT = 10.0

# duty sweep settings
GROUP = 0       # serial group (subchain) 0-3
ADDR = 0        # board address on that subchain
FREQ = 1        # frequency index 0-7
SWEEP_TIME = 8.0  # seconds to ramp from duty 0 to 15, no stops in between
HOLD_TIME = 2.0   # seconds to hold duty 15 after the sweep

# square firmware (Vibration_Unit.c) tables, only used to label the log
DUTY_PCT = [0, 1, 2, 3, 5, 6, 9, 12, 16, 21, 27, 35, 46, 59, 77, 99]  # duty_cycle_array
PR_VAL = [85, 72, 60, 52, 44, 37, 32, 27]  # f = 32 MHz / (4 * 4 * (PR2 + 1) * 200 steps)

'''
command format (main-branch controller + square vibration firmware), same as Python_Test.py
First byte: 00XXXX0Y, X is serial group number, Y is mode (1 start, 0 stop),
Second byte: 01XXXXXX, X is address,
Third byte: 1XXXXYYY, X is duty (0-15), Y is frequency (0-7).
The controller splits the third byte into duty, freq (2 bits) and wave (1 bit), then sends the PIC
the same low 3 bits, which the square firmware reads as one frequency index.
'''

# main's scripts pad every write to 20 frames; the controller skips frames starting with 0xFF
PADDING = bytes([0xFF, 0xFF, 0xFF]) * 19

T0 = time.monotonic()

def log(msg):
    print(f'[{time.monotonic() - T0:6.2f} s] {msg}', flush=True)

def frame(group, addr, start, duty4, freq3):
    return bytes([(group & 0x0F) << 2 | (start & 1),
                  0x40 | (addr & 0x3F),
                  0x80 | (duty4 & 0x0F) << 3 | (freq3 & 0x07)])

def describe(f):
    target = f'group {(f[0] >> 2) & 0x0F}, addr {f[1] & 0x3F}'
    if not f[0] & 1:
        return f'{f.hex(" ")}  (stop, {target})'  # duty and freq are ignored on stop
    duty, freq = (f[2] >> 3) & 0x0F, f[2] & 0x07
    return (f'{f.hex(" ")}  (start, square, {target}, duty {duty} = {DUTY_PCT[duty]}%, '
            f'freq {freq}, nominal {10000 / (PR_VAL[freq] + 1):.1f} Hz)')

async def send(c, f):
    log(f'sending {describe(f)}')
    # response=True waits for the controller to acknowledge the write
    await c.write_gatt_char(CHARACTERISTIC_UUID, f + PADDING, response=True)
    log('acknowledged by controller')

async def main():
    log(f'bleak {getattr(bleak, "__version__", "?")}; scanning for "{CONTROL_UNIT_NAME}" (up to {SCAN_TIMEOUT:.0f} s)')
    dev = await BleakScanner.find_device_by_filter(lambda d, ad: d.name == CONTROL_UNIT_NAME, timeout=SCAN_TIMEOUT)
    if dev is None:
        log('controller not found; devices seen nearby:')
        for d in await BleakScanner.discover():
            log(f'  {d.address}  {d.name}')
        raise SystemExit(1)
    log(f'found "{dev.name}" at {dev.address}; connecting')

    async with BleakClient(dev.address) as c:  # bleak 0.14 on Windows fails if given the BLEDevice itself
        log(f'connected: {c.is_connected}')
        char = c.services.get_characteristic(CHARACTERISTIC_UUID)
        if char is None:
            log(f'characteristic {CHARACTERISTIC_UUID} not found on the controller')
            raise SystemExit(1)
        log(f'characteristic found, properties: {", ".join(char.properties)}')

        stop = frame(GROUP, ADDR, 0, 0, 0)
        step = SWEEP_TIME / 16
        try:
            t_start = time.monotonic()
            for duty in range(16):
                await send(c, frame(GROUP, ADDR, 1, duty, FREQ))
                # timed from the sweep start, so BLE write delays don't stretch the sweep
                await asyncio.sleep(max(0.0, t_start + (duty + 1) * step - time.monotonic()))
            log(f'sweep finished in {time.monotonic() - t_start:.2f} s; holding duty 15 for {HOLD_TIME:.0f} s')
            await asyncio.sleep(HOLD_TIME)
        finally:
            if c.is_connected:  # also stops the motor if the sweep is interrupted (Ctrl+C)
                await send(c, stop)
        log('disconnecting')
    log('disconnected, done')

asyncio.run(main())
