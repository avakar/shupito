import sys, os.path
sys.path.insert(0, os.path.join(os.path.split(__file__)[0], '../../fw_common/avrlib'))
from usb_desc import *

def descriptors():
    return {
        0x100: DeviceDescriptor(
            bcdUSB=0x110,
            bDeviceClass=0xff,
            bDeviceSubClass=0xff,
            bDeviceProtocol=0xff,
            bMaxPacketSize0=64,
            idVendor=0x4a61,
            idProduct=0x679c,
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
                    bInterfaceNumber=0,
                    bInterfaceClass=0x0A,
                    bInterfaceSubClass=0,
                    bInterfaceProtocol=0,
                    iInterface=3,
                    endpoints=[
                        EndpointDescriptor(
                            bEndpointAddress=1 | Endpoint.In,
                            bmAttributes=Endpoint.Bulk,
                            wMaxPacketSize=64,
                            bInterval=16),
                        EndpointDescriptor(
                            bEndpointAddress=1,
                            bmAttributes=Endpoint.Bulk,
                            wMaxPacketSize=64,
                            bInterval=16)
                        ]
                    )
                ]
            ),
        0x300: LangidsDescriptor([0x409]),
        0x301: StringDescriptor('Shupito'),
        0x303: StringDescriptor('.debug'),
        }

if __name__ == '__main__':
    print_descriptors(open('usb_descriptors.h', 'w'), descriptors())
