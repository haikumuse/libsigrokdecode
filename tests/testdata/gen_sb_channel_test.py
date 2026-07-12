import os
import json

test_dir = os.path.join(os.path.dirname(__file__), "sb_channel_c", "default")
os.makedirs(test_dir, exist_ok=True)

config = {
    "samplerate": 20000000, # 20 MHz
    "num_channels": 1,
    "channels": {"sb": 0},
    "options": {}
}

samplerate = config["samplerate"]
samples = []
pin_state = 1

def add_samples(state, duration_us):
    global pin_state
    pin_state = state
    num = int(duration_us * (samplerate / 1000000))
    for _ in range(num):
        samples.append(pin_state)

def toggle(duration_us):
    global pin_state
    pin_state ^= 1
    add_samples(pin_state, duration_us)

# Idle
add_samples(1, 10.0)

def send_byte(b):
    for i in range(8):
        bit = (b >> i) & 1
        toggle(0.5 if bit else 1.0)
        if bit:
            toggle(0.5)

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def send_packet(packet):
    send_byte(0xFE)
    for b in packet:
        send_byte(b)
    crc_val = crc16(packet)
    send_byte(crc_val & 0xFF)
    send_byte((crc_val >> 8) & 0xFF)
    send_byte(0xFE)
    toggle(1.0) # Finish last bit!
    add_samples(pin_state, 5.0)

def send_lt_packet(cmd):
    send_byte(0xFE)
    send_byte(cmd)
    send_byte((~cmd) & 0xFF)
    # Note: LT transactions do not have CRC or trailing 0xFE, they just end.
    toggle(1.0)
    add_samples(pin_state, 5.0)

# Packet 1: LT Transaction (Type 0x80, ~0x80 = 0x7F)
send_lt_packet(0x80)

# Packet 2: Link Configuration (AT Command)
# Opcode 12, Length 0, Byte0=0x01, Byte1=0x11, Byte2=0x01
send_packet([0x41, 12, 0, 0x01, 0x11, 0x01])

# Packet 3: Gen 2/3 TxFFE (AT Command)
# Opcode 13, Length 0, Byte0=0xA1, Byte1=0x51, Byte2=0x82, Byte3=0x42
send_packet([0x41, 13, 0, 0xA1, 0x51, 0x82, 0x42])

# Packet 4: SB Version (AT Command)
# Opcode 15, Length 0, Byte0=5, Byte1=1, Byte2=0, Byte3=1
send_packet([0x41, 15, 0, 0x05, 0x01, 0x00, 0x01])

# Packet 5: Scrambler Re-Sync (RT Command)
# Type 0x21 (RT), Opcode 5, Length 0, Byte0=0x2A
send_packet([0x21, 5, 0, 0x2A])

# Packet 6: Gen 4 TxFFE (RT Command)
# Type 0x21 (RT), Opcode 14, Length 0, Byte0=0x8A, Byte1=0x45, Byte2=0x01, Byte3=0x02
send_packet([0x21, 14, 0, 0x8A, 0x45, 0x01, 0x02])

add_samples(pin_state, 10.0)
toggle(1.0) # Dummy toggle to flush the last bit of the last 0xFE

config["sample_count"] = len(samples)

with open(f"{test_dir}/config.json", "w") as f:
    json.dump(config, f, indent=4)

with open(f"{test_dir}/input.bin", "wb") as f:
    byte_val = 0
    bit_idx = 0
    for s in samples:
        if s:
            byte_val |= (1 << bit_idx)
        bit_idx += 1
        if bit_idx == 8:
            f.write(bytes([byte_val]))
            byte_val = 0
            bit_idx = 0
    if bit_idx > 0:
        f.write(bytes([byte_val]))

print(f"Generated {len(samples)} samples to {test_dir}/input.bin")
