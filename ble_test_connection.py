#!/usr/bin/env python3
"""
Simple BLE Connection Test
Tests connection to ESP32 BLE slave without logging
"""

import asyncio
from bleak import BleakClient, BleakScanner

# BLE Configuration
DEVICE_NAME = "LoadCell_BLE_Server"
BLE_SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
BLE_CHARACTERISTIC_UUID = "87654321-4321-4321-4321-cba987654321"

async def find_and_select_device():
    """Find and manually select BLE device"""
    print(f"🔍 Scanning for BLE devices...")
    
    # Scan for devices
    devices = await BleakScanner.discover(timeout=10.0)
    
    if not devices:
        print("❌ No BLE devices found")
        return None
    
    # Filter devices that have names (ignore unnamed devices)
    named_devices = [d for d in devices if d.name and d.name.strip()]
    
    if not named_devices:
        print("❌ No named BLE devices found")
        return None
    
    # Display available devices
    print(f"\n📱 Found {len(named_devices)} BLE devices:")
    print("=" * 60)
    for i, device in enumerate(named_devices):
        rssi_info = f" (RSSI: {device.rssi})" if hasattr(device, 'rssi') and device.rssi else ""
        print(f"  {i+1:2d}. {device.name:<25} | {device.address}{rssi_info}")
    print("=" * 60)
    
    # Manual selection
    while True:
        try:
            choice = input(f"\n🎯 Select device (1-{len(named_devices)}) or 'q' to quit: ").strip()
            
            if choice.lower() == 'q':
                print("❌ User cancelled device selection")
                return None
            
            device_index = int(choice) - 1
            
            if 0 <= device_index < len(named_devices):
                selected_device = named_devices[device_index]
                print(f"✅ Selected: {selected_device.name} ({selected_device.address})")
                return selected_device
            else:
                print(f"❌ Invalid choice. Please enter 1-{len(named_devices)}")
                
        except ValueError:
            print("❌ Invalid input. Please enter a number or 'q'")
        except KeyboardInterrupt:
            print("\n❌ User cancelled")
            return None

async def test_device_connection(target_device):
    """Test connection to selected device"""
    print(f"✅ Testing device: {target_device.name} ({target_device.address})")
    
    # Test connection
    try:
        print(f"🔗 Connecting to {target_device.address}...")
        async with BleakClient(target_device.address, timeout=15.0) as client:
            print(f"✅ Connected successfully!")
            
            # Check if connected
            is_connected = client.is_connected
            print(f"Connection status: {is_connected}")
            
            # List services
            services = client.services
            print(f"Available services: {len(services)}")
            
            for service in services:
                print(f"  Service: {service.uuid}")
                for char in service.characteristics:
                    print(f"    Characteristic: {char.uuid} (Properties: {char.properties})")
            
            # Test our specific service/characteristic
            try:
                char = client.services.get_characteristic(BLE_CHARACTERISTIC_UUID)
                if char:
                    print(f"✅ Found our characteristic: {char.uuid}")
                    print(f"   Properties: {char.properties}")
                else:
                    print(f"❌ Our characteristic not found")
            except Exception as e:
                print(f"❌ Error accessing characteristic: {e}")
            
            print("🔗 Connection test successful!")
            return True
            
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False

async def main():
    print("🧪 BLE Connection Test")
    print("=" * 40)
    
    # Find and select device
    target_device = await find_and_select_device()
    if not target_device:
        print("\n❌ No device selected")
        return
    
    # Test connection
    success = await test_device_connection(target_device)
    
    if success:
        print("\n✅ BLE connection test PASSED!")
        print("You can now run the full logger: python ble_redis_logger.py")
    else:
        print("\n❌ BLE connection test FAILED!")
        print("Check that:")
        print("1. ESP32 is powered on and running")
        print("2. BLE is advertising")
        print("3. Selected device is the correct one")

if __name__ == "__main__":
    asyncio.run(main())
