import asyncio
import argparse
from bleak import BleakScanner, BleakClient

CHARACTERISTIC_UUID = 'f22535de-5375-44bd-8ca9-d0ea9ff9e410'
CONTROL_UNIT_NAME = 'QT Py ESP32-S3'

'''
command format
First byte: 00XXXX0Y, X is serial group number, Y is mode,
Second byte: 01XXXXXX, X is address,
Third byte: 1XXXXYYZ, X is duty, Y is frequency, Z is wave.
'''

def create_command(addr, mode, duty, freq):
    serial_group = addr // 30
    serial_addr = addr % 30
    byte1 = (serial_group << 2) | (mode & 0x01)
    byte2 = 0x40 | (serial_addr & 0x3F)  # 0x40 represents the leading '01'
    byte3 = 0x80 | ((duty & 0x0F) << 3) | (freq)  # 0x80 represents the leading '1'
    return bytearray([byte1, byte2, byte3])

async def setMotor(client):
    while True:
        motor_addr = int(input('what is the unit you want to control (0-127)?'))
        duty = int(input('0-15 for duty?'))
        freq = int(input('0-7 for frequency?'))
        start_or_stop = int(input('1 for start and 0 for stop?'))
        user_input = {
            'addr':motor_addr,
            'mode':start_or_stop,
            'duty':duty,
            'freq':freq
        }
        command = bytearray([])
        command = command + create_command(user_input['addr'], user_input['mode'], user_input['duty'], user_input['freq'])
        command = command + bytearray([0xFF, 0xFF, 0xFF]) * 19 # Padding
        await client.write_gatt_char(CHARACTERISTIC_UUID, command)

async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        print('device name = ', d.name)
        if d.name != None:
            if d.name == CONTROL_UNIT_NAME:
                print('central unit BLE found!!!')
                async with BleakClient(d.address) as client:
                    print(f'BLE connected to {d.address}')
                    val = await client.read_gatt_char(CHARACTERISTIC_UUID)
                    print('Motor read = ', val)
                    await setMotor(client)

if __name__ == "__main__":
    # Initialize the argument parser
    parser = argparse.ArgumentParser(description="Read CHARACTERISTIC_UUID and CONTROL_UNIT_NAME from the command line.")

    # Add arguments with flags
    parser.add_argument(
        "-uuid", "--characteristic_uuid", required=False, type=str,
        default="f22535de-5375-44bd-8ca9-d0ea9ff9e410",
        help="The UUID of the characteristic"
    )
    parser.add_argument(
        "-name", "--control_unit_name", required=False, type=str, 
        default="QT Py ESP32-S3 Control Unit",
        help="The Bluetooth name of the control unit"
    )

    # Parse the arguments
    args = parser.parse_args()

    # Access and print the parameters
    print(f"CHARACTERISTIC_UUID: {args.characteristic_uuid}")
    print(f"CONTROL_UNIT_NAME: {args.control_unit_name}")

    CHARACTERISTIC_UUID = args.characteristic_uuid
    CONTROL_UNIT_NAME = args.control_unit_name
    
    asyncio.run(main())