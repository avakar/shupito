import struct, sys, os, os.path
from uuid import UUID
sys.path.insert(0, os.path.join(os.path.split(__file__)[0], '../../fw_common/avrlib'))

from yb_desc import *
hi = get_hg_info()

yb_desc = make_yb_desc(UUID('093d7f33-cdc6-4928-955d-513d17a85358'),
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
        Config(UUID('1d4738a0-fc34-4f71-aa73-57881b278cb1'), 10, 1, flags=0x03,  # measurement
            data=struct.pack('<BI',
                1, # version
                0x0002B401)), # 16.16 fixpoint millivolts per unit
        Config(UUID('c49124d9-4629-4aef-ae35-ddc32c21b279'), 0, 0, flags=0x04,   # info
            data=(struct.pack('<BBBIh', 1,
                2, 3, # hw version
                hi.timestamp, -hi.zoffset/60) # fw timestamp
                + hi.rev_hash)) # fw version
        )
    )

from usb_desc import *
usb_desc = {
    'comment': hi.hg_log,
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
                bInterfaceClass=0xff,
                bInterfaceSubClass=0,
                bInterfaceProtocol=0,
                iInterface=5,
                functional=[
                    CustomDescriptor(75, yb_desc)
                    ],
                endpoints=[
                    EndpointDescriptor(
                        bEndpointAddress=2,
                        bmAttributes=Endpoint.Bulk,
                        wMaxPacketSize=64,
                        bInterval=16),
                    EndpointDescriptor(
                        bEndpointAddress=2 | Endpoint.In,
                        bmAttributes=Endpoint.Bulk,
                        wMaxPacketSize=64,
                        bInterval=16),
                    ]
                ),
            InterfaceDescriptor(
                bInterfaceNumber=1,
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
                ),
            InterfaceDescriptor(
                bInterfaceNumber=2,
                bAlternateSetting=0,
                bInterfaceClass=0x0A,
                bInterfaceSubClass=0,
                bInterfaceProtocol=0xff,
                iInterface=4,
                endpoints=[
                    EndpointDescriptor(
                        bEndpointAddress=3 | Endpoint.In,
                        bmAttributes=Endpoint.Bulk,
                        wMaxPacketSize=64,
                        bInterval=16),
                    EndpointDescriptor(
                        bEndpointAddress=3,
                        bmAttributes=Endpoint.Bulk,
                        wMaxPacketSize=64,
                        bInterval=16)
                    ]
                ),
            ]
        ),
    0x300: LangidsDescriptor([0x409]),
    0x301: StringDescriptor('Shupito'),
    0x303: StringDescriptor('.debug'),
    0x304: StringDescriptor('tunnel'),
    0x305: StringDescriptor('yb'),
    }

if __name__ == '__main__':
    if len(sys.argv) < 2:
        fout = sys.stdout
    else:
        fout = open(sys.argv[1], 'w')
    print_descriptors(fout, usb_desc)
    fout.close()
