from usb_desc import *
import sys

descriptors = {
    0x100: DeviceDescriptor(
        bcdUSB=0x200,
        bDeviceClass=0xFF,
        bDeviceSubClass=0xFF,
        bDeviceProtocol=0xFF,
        bMaxPacketSize0=32,
        idVendor=0x4a61,
        idProduct=0x679a,
        bcdDevice=0x0001,
        iManufacturer=1,
        iProduct=2,
        bNumConfigurations=1
        ),
    0x200: ConfigurationDescriptor(
        bConfigurationValue=1,
        bmAttributes=ConfigurationAttributes.Sig,
        bMaxPower=50,
        interfaces=[
            InterfaceDescriptor(
                bInterfaceNumber=1,
                bInterfaceClass=0,
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
    0x301: StringDescriptor('Manufacturer Name'),
    0x302: StringDescriptor('Product Name'),
}

print_descriptors(open('usb_descriptors.h', 'w'), descriptors)
