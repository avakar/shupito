import struct
from uuid import UUID

class And:
    def __init__(self, *children):
        self.children = children

    def store(self):
        return chr(len(self.children)) + ''.join((child.store() for child in self.children))

class Or:
    def __init__(self, *children):
        self.children = children

    def store(self):
        return chr(0x80 | len(self.children)) + ''.join((child.store() for child in self.children))

class Config:
    def __init__(self, guid, first_pipe, pipe_count, flags = 0, data=''):
        self.flags = flags
        self.guid = guid
        self.first_pipe = first_pipe
        self.pipe_count = pipe_count
        self.data = data

    def store(self):
        return (chr(0) + chr(self.flags) + self.guid.bytes + chr(self.first_pipe) + chr(self.pipe_count)
            + chr(len(self.data)) + self.data)

def make_descriptor(device_guid, child):
    return (chr(1) # version
        + device_guid.bytes + child.store())

def to_c(s):
    res = []
    while s:
        chunk = s[:16]
        s = s[16:]
        res.append(' '.join(('0x%02x,' % ord(ch) for ch in chunk)))
        res.append('\n')
    return ''.join(res)

def adler16(s, mod=251):
    c0 = 1
    c1 = 0
    for ch in s:
        c0 = (c0 + ord(ch)) % mod
        c1 = (c1 + c0) % mod
    return s + chr(c0) + chr(c1)

open('desc.h', 'w').write(to_c(adler16(make_descriptor(UUID('093d7f32-cdc6-4928-955d-513d17a85358'),
    And(
        Or(
            Config(UUID('46dbc865-b4d0-466b-9b70-2f3f5b264e65'), 1, 8,  # ICSP
                data=struct.pack('<BIHH',
                    1, # version
                    16000000,
                    1,
                    (1<<12)
                    )),
            Config(UUID('71efb903-3030-4fd3-8896-1946aba37efc'), 1, 8,  # PDI
                data=struct.pack('<BIHH',
                    1, #version
                    16000000,
                    1,
                    (1<<12)
                    )),
            Config(UUID('ee047e35-dec8-48ab-b194-e3762c8f6b66'), 1, 4,  # JTAG
                data=struct.pack('<BI', 1, 32000000)),
            Config(UUID('76e37480-3f61-4e7a-9b1b-37af6bd418fa'), 1, 5,  # cc25xx
                data=struct.pack('<BIHH',
                    1, #version
                    16000000,
                    1,
                    (1<<12)
                    )),
            Config(UUID('633125ab-32e0-49ec-b240-7d845bb70b2d'), 1, 3,  # SPI
                data=struct.pack('<BIHH',
                    1, # version
                    16000000,
                    1,
                    (1<<12)
                    ))
        ),
        Config(UUID('356e9bf7-8718-4965-94a4-0be370c8797c'), 9, 1, flags=0x03,   # tunnel
            data=struct.pack('<BI', 1, 2000000)),
        Config(UUID('1d4738a0-fc34-4f71-aa73-57881b278cb1'), 10, 1, flags=0x00,  # measurement
            data=struct.pack('<BI',
                1, # version
                0x0002B401)) # 16.16 fixpoint millivolts per unit
    )
))))
