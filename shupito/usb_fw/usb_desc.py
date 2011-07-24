from struct import pack

class DescriptorType:
    DEVICE = 1
    CONFIGURATION = 2
    STRING = 3
    INTERFACE = 4
    ENDPOINT = 5
    DEVICE_QUALIFIER = 6
    OTHER_SPEED_CONFIGURATION = 7
    INTERFACE_POWER = 8
    OTG = 9
    DEBUG = 10
    INTERFACE_ASSOCIATION = 11

class ConfigurationAttributes:
    Sig = (1<<7)
    SelfPowered = (1<<6)
    RemoteWakeup = (1<<5)

class Endpoint:
    Out = (0<<7)
    In = (1<<7)

    Control = 0
    Isochronous = 1
    Bulk = 2
    Interrupt = 3

    NoSynchronization = (0<<2)
    Asynchronous = (1<<2)
    Adaptive = (2<<2)
    Synchronous = (3<<2)

    DataEndpoint = (0<<4)
    FeedbackEndpoint = (1<<4)
    ImplicitFeedbackDataEndpoint = (2<<4)

def DeviceDescriptor(bcdUSB=None, bDeviceClass=None, bDeviceSubClass=None, bDeviceProtocol=None,
        bMaxPacketSize0=None, idVendor=None, idProduct=None, bcdDevice=None,
        iManufacturer=0, iProduct=0, iSerialNumber=0, bNumConfigurations=None):
    return pack('<BBHBBBBHHHBBBB',
        18, DescriptorType.DEVICE,
        bcdUSB, bDeviceClass, bDeviceSubClass, bDeviceProtocol,
        bMaxPacketSize0,
        idVendor, idProduct, bcdDevice,
        iManufacturer, iProduct, iSerialNumber,
        bNumConfigurations)

def ConfigurationDescriptor(bConfigurationValue=None, iConfiguration=0,
        bmAttributes=None, bMaxPower=None, interfaces=None):
    res = pack('<BBHBBBBB',
        9, DescriptorType.CONFIGURATION,
        9 + sum((len(intf) for intf in interfaces)),
        len(interfaces),
        bConfigurationValue, iConfiguration, bmAttributes, bMaxPower)
    return res + ''.join(interfaces)

def InterfaceDescriptor(bInterfaceNumber=None, bAlternateSetting=0, bInterfaceClass=None,
        bInterfaceSubClass=None, bInterfaceProtocol=None, iInterface=0, endpoints=None):
    return pack('<BBBBBBBBB',
        9, DescriptorType.INTERFACE,
        bInterfaceNumber, bAlternateSetting, len(endpoints), bInterfaceClass, bInterfaceSubClass,
        bInterfaceProtocol, iInterface) + ''.join(endpoints)

def EndpointDescriptor(bEndpointAddress=None, bmAttributes=None, wMaxPacketSize=None, bInterval=None):
    return pack('<BBBBHB',
        7, DescriptorType.ENDPOINT,
        bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval)

def StringDescriptor(s):
    s = unicode(s).encode('utf-16-le')
    return pack('<BB', 2 + len(s), DescriptorType.STRING) + s

def LangidsDescriptor(langids):
    return (pack('<BB', 2 + 2*len(langids), DescriptorType.STRING)
        + ''.join((pack('<H', langid) for langid in langids)))

def print_descriptors(fout, descriptors):
    fout.write('struct { uint16_t index, first, last; } const usb_descriptor_map[] = {\n')
    data = ''
    for key, value in sorted(descriptors.iteritems()):
        oldlen = len(data)
        data += value
        fout.write('    { 0x%x, %d, %d },\n' % (key, oldlen, len(data)))
    fout.write('};\n\nstatic prog_uint8_t const usb_descriptors[] PROGMEM = {\n')

    while data:
        line = '    '
        for ch in data[:16]:
            line += '0x%0.2x, ' % ord(ch)
        fout.write(line.rstrip())
        fout.write('\n')
        data = data[16:]

    fout.write('};\n')
