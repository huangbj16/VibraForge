import asyncio
from bleak import BleakScanner, BleakClient
import json
import time
import argparse

CHARACTERISTIC_UUID = 'f22535de-5375-44bd-8ca9-d0ea9ff9e410'
CONTROL_UNIT_NAME = 'QT Py ESP32-S3 BJ'
COMMAND_FILENAME = 'command.json'

def create_command(addr, mode, duty, freq):
    serial_group = addr // 30
    serial_addr = addr % 30
    byte1 = (serial_group << 2) | (mode & 0x01)
    byte2 = 0x40 | (serial_addr & 0x3F)  # 0x40 represents the leading '01'
    byte3 = 0x80 | ((duty & 0x0F) << 3) | (freq)  # 0x80 represents the leading '1'
    return bytearray([byte1, byte2, byte3])

async def sendCommands(client):
    with open(COMMAND_FILENAME) as f:
        commands = f.readlines()
        command_idx = 0
        time_offset = time.perf_counter() # record the starting time
        while command_idx < len(commands):
            # collect commands that are at the same time and form the output
            command_parsed = json.loads(commands[command_idx])
            command_output = bytearray([])
            print('command = ', command_parsed)
            command_output = command_output + create_command(command_parsed['addr'], command_parsed['mode'], command_parsed['duty'], command_parsed['freq'])
            ts = float(command_parsed['time'])
            command_idx += 1
            command_count = 1 # max allowed in one command is 10
            while True:
                if command_count == 10:
                    break
                if command_idx < len(commands):
                    command_parsed = json.loads(commands[command_idx])
                    if (ts+1e-6) > float(command_parsed['time']): # basically two commands are at the same time
                        command_output = command_output + create_command(command_parsed['addr'], command_parsed['mode'], command_parsed['duty'], command_parsed['freq'])
                        command_idx += 1
                        command_count += 1
                    else:
                        break
                else:
                    break
            command_output = command_output + bytearray([0xFF, 0xFF, 0xFF]) * (20-command_count)
            # wait for the send time
            print('command time = ', ts)
            start = time.perf_counter()
            # await asyncio.sleep(ts-current_time)
            while (time.perf_counter()-time_offset) < ts:
                pass
            # actual_sleep_duration = time.perf_counter() - start
            # print(f"{start}, Actual sleep duration: {actual_sleep_duration} seconds")
            # print('commands = \n', command_output)
            # print('command len = ', len(command_output))
            await client.write_gatt_char(CHARACTERISTIC_UUID,  command_output)
            

async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        print('device name = ', d.name)
        if d.name != None:
            if d.name == CONTROL_UNIT_NAME:
                print('central unit BLE found!!!')
                async with BleakClient(d.address) as client:
                    print(f'BLE connected to {d.address}')
                    print('mtu_size = ', client.mtu_size)
                    val = await client.read_gatt_char(CHARACTERISTIC_UUID)
                    print('Motor read = ', val)
                    await sendCommands(client)

if __name__ == "__main__":
    # Initialize the argument parser
    parser = argparse.ArgumentParser(description="Read CHARACTERISTIC_UUID, CONTROL_UNIT_NAME and COMMAND_FILENAME from the command line.")

    # Add arguments with flags
    parser.add_argument(
        "-uuid", "--characteristic_uuid", required=False, type=str,
        default="f22535de-5375-44bd-8ca9-d0ea9ff9e410",
        help="The UUID of the characteristic"
    )
    parser.add_argument(
        "-name", "--control_unit_name", required=False, type=str, 
        default="QT Py ESP32-S3",
        help="The Bluetooth name of the control unit"
    )

    parser.add_argument(
        "-file", "--command_filename", required=False, type=str, 
        default="command.json",
        help="The filename of the command file"
    )

    # Parse the arguments
    args = parser.parse_args()

    # Access and print the parameters
    print(f"CHARACTERISTIC_UUID: {args.characteristic_uuid}")
    print(f"CONTROL_UNIT_NAME: {args.control_unit_name}")
    print(f"COMMAND_FILENAME: {args.command_filename}")

    CHARACTERISTIC_UUID = args.characteristic_uuid
    CONTROL_UNIT_NAME = args.control_unit_name
    COMMAND_FILENAME = args.command_filename
    
    asyncio.run(main())