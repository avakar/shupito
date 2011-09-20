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
    def __init__(self, guid, first_pipe, pipe_count, flags = 0):
        self.flags = flags
        self.guid = guid
        self.first_pipe = first_pipe
        self.pipe_count = pipe_count

    def store(self):
        return chr(0) + chr(self.flags) + self.guid.bytes + chr(self.first_pipe) + chr(self.pipe_count)

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

print to_c(adler16(make_descriptor(UUID('093d7f32-cdc6-4928-955d-513d17a85358'),
    And(
        Or(
            Config(UUID('46dbc865-b4d0-466b-9b70-2f3f5b264e65'), 1, 8), # SPI
            Config(UUID('71efb903-3030-4fd3-8896-1946aba37efc'), 1, 8)  # PDI
        ),
        Config(UUID('356e9bf7-8718-4965-94a4-0be370c8797c'), 9, 1, flags=0x03),  # tunnel
        Config(UUID('1d4738a0-fc34-4f71-aa73-57881b278cb1'), 10, 1, flags=0x03), # measurement
        Config(UUID('0f75b62c-e9ad-4840-ac43-eb28b12cb080'), 11, 1, flags=0x03)  # measurement
    )
)))
