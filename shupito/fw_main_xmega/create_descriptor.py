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
    def __init__(self, guid, pipe_count):
        self.guid = guid
        self.pipe_count = pipe_count

    def store(self):
        return chr(0) + self.guid.bytes + chr(self.pipe_count)

def make_descriptor(device_guid, child):
    return device_guid.bytes + child.store()

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
            Config(UUID('46dbc865-b4d0-466b-9b70-2f3f5b264e65'), 8), # SPI
            Config(UUID('71efb903-3030-4fd3-8896-1946aba37efc'), 8)  # PDI
        ),
        Config(UUID('356e9bf7-8718-4965-94a4-0be370c8797c'), 1)  # tunnel
    )
)))