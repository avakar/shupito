import sys
from struct import pack

def descriptors():
    return {
        0x100: DeviceDescriptor(
            bcdUSB=0x110,
            bDeviceClass=0xff,
            bDeviceSubClass=0xff,
            bDeviceProtocol=0xff,
            bMaxPacketSize0=32,
            idVendor=0x4a61,
            idProduct=0x679a,
            bcdDevice=0x0001,
            iManufacturer=0,
            iProduct=1,
            iSerialNumber=2,
            bNumConfigurations=1
            ),
        0x200: ConfigurationDescriptor(
            bConfigurationValue=1,
            bmAttributes=ConfigurationAttributes.Sig,
            bMaxPower=50,
            interfaces=[
                InterfaceDescriptor(
                    bInterfaceNumber=0x00,
                    bInterfaceClass=0x02,
                    bInterfaceSubClass=0x02,
                    bInterfaceProtocol=0x01,
                    functional=[
                        CdcHeaderDescriptor(bcdCDC=0x100),
                        CdcCallManagementDescriptor(bmCapabilities=0, bDataInterface=0x01),
                        CdcAcmDescriptor(bmCapabilities=0),
                        CdcUnionDescriptor(bControllingInterface=0, bSubordinateInterfaces=[1]),
                        ],
                    endpoints=[
                        EndpointDescriptor(
                            bEndpointAddress=1 | Endpoint.In,
                            bmAttributes=Endpoint.Interrupt,
                            wMaxPacketSize=8,
                            bInterval=16)
                        ],
                    ),
                InterfaceDescriptor(
                    bInterfaceNumber=0x01,
                    bInterfaceClass=0x0A,
                    bInterfaceSubClass=0,
                    bInterfaceProtocol=0,
                    endpoints=[
                        EndpointDescriptor(
                            bEndpointAddress=3 | Endpoint.Out,
                            bmAttributes=Endpoint.Bulk,
                            wMaxPacketSize=64,
                            bInterval=1),
                        EndpointDescriptor(
                            bEndpointAddress=4 | Endpoint.In,
                            bmAttributes=Endpoint.Bulk,
                            wMaxPacketSize=32,
                            bInterval=1),
                        ]
                    )
                ]
            ),
        0x300: LangidsDescriptor([0x409]),
        0x301: StringDescriptor('Shupito'),
        }

def _main():
    print_descriptors(open('usb_descriptors.h', 'w'), descriptors())

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

    CS_INTERFACE = 0x24
    CS_ENDPOINT = 0x25

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
        bInterfaceSubClass=None, bInterfaceProtocol=None, iInterface=0, endpoints=[], functional=[]):
    return pack('<BBBBBBBBB',
        9, DescriptorType.INTERFACE,
        bInterfaceNumber, bAlternateSetting, len(endpoints), bInterfaceClass, bInterfaceSubClass,
        bInterfaceProtocol, iInterface) + ''.join(functional) + ''.join(endpoints)

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

def CdcHeaderDescriptor(bcdCDC):
    return pack('<BBBH', 5, DescriptorType.CS_INTERFACE, 0x00, bcdCDC)

def CdcUnionDescriptor(bControllingInterface, bSubordinateInterfaces):
    return (pack('<BBBB', 4 + len(bSubordinateInterfaces), DescriptorType.CS_INTERFACE, 0x06, bControllingInterface)
        + ''.join((pack('<B', bSubordinateInterface) for bSubordinateInterface in bSubordinateInterfaces)))

def CdcCallManagementDescriptor(bmCapabilities, bDataInterface):
    return pack('<BBBBB', 5, DescriptorType.CS_INTERFACE, 0x01, bmCapabilities, bDataInterface)

def CdcAcmDescriptor(bmCapabilities):
    return pack('<BBBB', 4, DescriptorType.CS_INTERFACE, 0x02, bmCapabilities)

def print_descriptors(fout, descriptors):
    fout.write('struct { uint16_t index, first, last; } const usb_descriptor_map[] = {\n')
    data = ''
    for key, value in sorted(descriptors.iteritems()):
        oldlen = len(data)
        data += value
        fout.write('    { 0x%03x, 0x%03x, 0x%03x }, // length: 0x%03x (%d)\n' % (key, oldlen, len(data), len(data) - oldlen, len(data) - oldlen))
    fout.write('};\n\nstatic uint8_t const usb_descriptors[] PROGMEM = {\n')

    while data:
        line = '    '
        for ch in data[:16]:
            line += '0x%0.2x, ' % ord(ch)
        fout.write(line.rstrip())
        fout.write('\n')
        data = data[16:]

    fout.write('};\n')

if __name__ == '__main__':
    _main()
