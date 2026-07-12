import os
import json
import struct

# Create test directory
test_dir = os.path.join(os.path.dirname(__file__), "dp_aux_c", "default")
os.makedirs(test_dir, exist_ok=True)

# Generate config.json
config = {
    "samplerate": 20000000, # 20 MHz
    "num_channels": 1,
    "channels": {"aux": 0},
    "options": {}
}

samplerate = config["samplerate"]
samples = []
pin_state = 1

def add_samples(state, duration_us):
    num = int(duration_us * (samplerate / 1000000))
    for _ in range(num):
        samples.append(state)

def encode_bits(val, num_bits):
    for i in range(num_bits - 1, -1, -1):
        bit = (val >> i) & 1
        if bit == 1:
            add_samples(0, 0.5)
            add_samples(1, 0.5)
        else:
            add_samples(1, 0.5)
            add_samples(0, 0.5)

# Idle
add_samples(1, 10.0)

def generate_sync():
    for _ in range(16):
        add_samples(0, 0.5)
        add_samples(1, 0.5)

def generate_req_sync_end():
    add_samples(0, 2.5)
    add_samples(1, 2.0)

def generate_reply_sync_end():
    add_samples(0, 1.5)
    add_samples(1, 1.0)

def generate_stop():
    add_samples(0, 1.5)
    add_samples(1, 1.5)

def req_read(addr, length):
    generate_sync()
    generate_req_sync_end()
    encode_bits(0x9, 4)
    encode_bits(addr, 20)
    encode_bits(length - 1, 8)
    generate_stop()

def req_write(addr, data_bytes):
    generate_sync()
    generate_req_sync_end()
    encode_bits(0x8, 4)
    encode_bits(addr, 20)
    encode_bits(len(data_bytes) - 1, 8)
    for b in data_bytes:
        encode_bits(b, 8)
    generate_stop()

def reply(status, data_bytes=None):
    generate_sync()
    generate_reply_sync_end()
    encode_bits(status, 8) # Reply command is 8 bits
    if data_bytes:
        for b in data_bytes:
            encode_bits(b, 8)
    generate_stop()

# 1. Native AUX Read Request (Len=1)
req_read(0x00700, 1)
add_samples(1, 5.0)

# 2. Native AUX Read Reply (ACK + Data)
reply(0x00, [0x01]) # ACK = 0x00, Data = 0x01
add_samples(1, 15.0)

# 3. Native AUX Write Request (Len=2)
req_write(0x00100, [0xAA, 0x55])
add_samples(1, 5.0)

# 4. Native AUX Write Reply (ACK)
reply(0x00)
add_samples(1, 15.0)

config["sample_count"] = len(samples)

with open(f"{test_dir}/config.json", "w") as f:
    json.dump(config, f, indent=4)

# Pack samples into input.bin (LSB first)
# For 1 channel, each byte holds 8 samples.
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
