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

open('desc.h', 'w').write(to_c(adler16(make_descriptor(UUID('68a4e349-b109-41d5-99b0-fda1aeca126f'),
    Config(UUID('46dbc865-b4d0-466b-9b70-2f3f5b264e65'), 1, 8), # SPI
))))
