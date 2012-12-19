supported_baudrates = [
    50,
    75,
    110,
    134,
    150,
    300,
    600,
    1200,
    1800,
    2400,
    4800,
    7200,
    9600,
    14400,
    19200,
    38400,
    57600,
    115200,
    128000,
    230400,
    250000,
    256000,
    460800,
    500000,
    512000,
    921600,
    1000000,
    1024000,
    1250000,
    1500000,
    1750000,
    1843200,
    2000000,
    2048000,
    2250000,
    2500000,
    2750000,
    3000000,
    3250000,
    3500000,
    3686400,
    3750000,
    4000000,
    ]

def fbaud(fper, bscale, bsel, dbl):
    if dbl:
        if bscale < 0:
            return fper / float(8*((1.0/(2**-bscale))*bsel + 1))
        else:
            return fper / float(2**bscale * 8*(bsel+1))
    else:
        if bscale < 0:
            return fper / float(16*((1.0/(2**-bscale))*bsel + 1))
        else:
            return fper / float(2**bscale * 16*(bsel+1))

res = []
for dbl in (False, True):
    for bscale in xrange(-7, 8):
        for bsel in xrange(0, 0x1000):
            res.append((fbaud(32000000, bscale, bsel, dbl), -1 if dbl else 0, bscale, bsel))

baudrates = {}
res.sort()
for fbaud, dbl, bscale, bsel in res:
    baudrates[fbaud] = (bsel, bscale, dbl)

res = {}
for fbaud in supported_baudrates:
    closest = None
    error = None
    for key in baudrates:
        if closest is None or abs(key - fbaud) < error:
            error = abs(key - fbaud)
            closest = key

    bsel, bscale, dbl = baudrates[closest]
    res[fbaud] = bsel, bscale, dbl, closest

print '''#include <avr/pgmspace.h>

static uint8_t const usart_baudctrls[] PROGMEM = {'''

for key in sorted(res):
    bsel, bscale, dbl, closest = res[key]
    
    orig_bscale = bscale
    if bscale < 0:
        bscale += 16

    baudctrl = (bscale << 12) | bsel

    c = int(round(closest))
    data = [c & 0xff, (c >> 8) & 0xff, (c >> 16) & 0xff, baudctrl >> 8, baudctrl & 0xff]
    if dbl:
        data[2] |= 0x80

    error = 100 * abs(key - closest) / key
    print '    %s,    // %7d, %7.0f, %.2f%%, (%2d, %4d, %d) ' % (
        ', '.join(('0x%02x' % d for d in data)), key, closest, error, orig_bscale, bsel, 1 if dbl else 0)

print '''};

static uint8_t const usart_baudctrl_count = %d;''' % len(res)
