import serial.tools.list_ports

# Lấy danh sách các thiết bị serial có sẵn
ports = serial.tools.list_ports.comports()

# Duyệt qua từng thiết bị và in ra thông tin
for port in ports:
    print(f"Device: {port.device}")
    print(f"Name: {port.name}")
    print(f"Description: {port.description}")
    print(f"Manufacturer: {port.manufacturer}")
    print(f"Serial Number: {port.serial_number}")
    print(f"Location: {port.location}")
    print(f"Vendor ID: {port.vid if port.vid else 'Unknown'}")
    print(f"Product ID: {port.pid if port.pid else 'Unknown'}")
    print("-" * 40)

if not ports:
    print("Không tìm thấy thiết bị serial nào!")
