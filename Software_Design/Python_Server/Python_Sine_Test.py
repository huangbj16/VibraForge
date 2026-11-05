import asyncio
import time
import bleak
from bleak import BleakScanner, BleakClient

CHARACTERISTIC_UUID = 'f22535de-5375-44bd-8ca9-d0ea9ff9e410'
CONTROL_UNIT_NAME = 'VibraForge-BLE'  # name advertised by the 3-byte controller firmware
SCAN_TIMEOUT = 10.0

# duty sweep settings
GROUP = 0       # serial group (subchain) 0-3
ADDR = 0        # board address on that subchain
WAVE = 0        # 1 sine, 0 square
FREQ = 3        # frequency index 0-7
SWEEP_TIME = 8.0  # seconds to ramp from duty 0 to 31, no stops in between

'''
command format (3-byte controller + sine/square vibration firmware)
First byte: W0XXXXYY, W is wave (1 sine, 0 square), X is serial group number, Y is mode (1 start, 0 stop),
Second byte: 00XXXXXX, X is address,
Third byte: XXXXXYYY, X is duty (0-31), Y is frequency (0-7).
'''

T0 = time.monotonic()

def log(msg):
    print(f'[{time.monotonic() - T0:6.2f} s] {msg}', flush=True)

def frame(group, addr, start, duty5, freq3, wave):
    return bytes([(wave & 1) << 7 | (group & 0x0F) << 2 | (start & 1),
                  addr & 0x3F,
                  (duty5 & 0x1F) << 3 | (freq3 & 0x07)])

def describe(f):
    target = f'group {(f[0] >> 2) & 0x0F}, addr {f[1] & 0x3F}'
    if not f[0] & 1:
        return f'{f.hex(" ")}  (stop, {target})'  # wave, duty and freq are ignored on stop
    wave = 'sine' if f[0] >> 7 else 'square'
    return f'{f.hex(" ")}  (start, {wave}, {target}, duty {f[2] >> 3}, freq {f[2] & 0x07})'

async def send(c, f):
    log(f'sending {describe(f)}')
    # response=True waits for the controller to acknowledge the write
    await c.write_gatt_char(CHARACTERISTIC_UUID, f, response=True)
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

        stop = frame(GROUP, ADDR, 0, 0, 0, 0)
        step = SWEEP_TIME / 32
        try:
            t_start = time.monotonic()
            for duty in range(32):
                await send(c, frame(GROUP, ADDR, 1, duty, FREQ, 1))
                # timed from the sweep start, so BLE write delays don't stretch the sweep
                await asyncio.sleep(max(0.0, t_start + (duty + 1) * step - time.monotonic()))
            log(f'sweep finished in {time.monotonic() - t_start:.2f} s')
            await send(c, frame(GROUP, ADDR, 1, duty, FREQ, 1))
            await asyncio.sleep(2)
            await send(c, frame(GROUP, ADDR, 1, duty, FREQ, 0))
            await asyncio.sleep(2)
        finally:
            if c.is_connected:  # also stops the motor if the sweep is interrupted (Ctrl+C)
                await send(c, stop)
        log('disconnecting')
    log('disconnected, done')

asyncio.run(main())
