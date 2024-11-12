import struct
import sys

if ("0x" in sys.argv[1] or "0X" in sys.argv[1]):
    size = int(sys.argv[1], 16)
else:
    size = int(sys.argv[1], 10)

with open("test_file","wb") as f:
    count = 0
    while (count < size):
        s = struct.pack('>Q', count)
        f.write(s)
        count = count + 1