import struct, sys, os, os.path, json
from uuid import UUID
sys.path.insert(0, os.path.join(os.path.split(__file__)[0], '../../fw_common/avrlib'))

from yb_desc import *
hi = get_hg_info()

device_guid = '093d7f33-cdc6-4928-955d-513d17a85358'

yb_desc = make_yb_desc(UUID(device_guid),
    And(
        Or(
            Config(UUID('46dbc865-b4d0-466b-9b70-2f3f5b264e65'), 1, 8,  # ICSP
                data=struct.pack('<BIHH',
                    1, # version
                    16000000,
                    1,
                    (1<<12)
                    )
                ),
            Config(UUID('71efb903-3030-4fd3-8896-1946aba37efc'), 1, 8,  # PDI
                data=struct.pack('<BIHH',
                    1, #version
                    16000000,
                    1,
                    (1<<12)
                    )
                ),
            Config(UUID('633125ab-32e0-49ec-b240-7d845bb70b2d'), 1, 3,  # SPI
                data=struct.pack('<BIHH',
                    1, # version
                    16000000,
                    1,
                    (1<<12)
                    )
                ),
            Config(UUID('fe047e35-dec8-48ab-b194-e3762c8f6b66'), 1, 4,  # JTAG
                data=struct.pack('<BII',
                    1,
                    32000000,
                    1000000)),
            Config(UUID('b1a28e62-6d13-44b5-8894-0b9f7a3061c9'), 1, 5,  # UART
                data=struct.pack('<BIHH',
                    1, # version
                    16000000,
                    1,
                    (1<<12)
                    )
                ),
            ),
        Config(UUID('1d4738a0-fc34-4f71-aa73-57881b278cb1'), 10, 1, flags=0x03,  # measurement
            data=struct.pack('<BI',
                1, # version
                0x0002B401) # 16.16 fixpoint millivolts per unit
            ),
        Config(UUID('c49124d9-4629-4aef-ae35-ddc32c21b279'), 11, 1, flags=0x03,  # fw info/update
            data=(struct.pack('<BBBIh', 1,
                2, 3, # hw version
                hi.timestamp, -hi.zoffset/60) # fw timestamp
                + hi.rev_hash), # fw version
            ),
        Config(UUID('e5e646a8-beb6-4a68-91f2-f005c72e9e57'), 12, 1, flags=0x03), # button
        Config(UUID('9034d141-c47e-406b-a6fd-3f5887729f8f'), 13, 1, flags=0x03), # led
        Config(UUID('64d5bf39-468a-4fbb-80bb-334d8ca3ad81'), 14, 1, flags=0x03,  # rename
            data=struct.pack('<BH', 1, 30)
            ),
        Config(UUID('0a77e245-db84-4871-8d0a-daefa901df21'), 16, 2, flags=0x03,  # PWM
            data=struct.pack('<BI',
                1, # version
                16000000 # base frequency
                )
            )
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
        bcdDevice=0x0203,
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
                functional=[
                    pack('<BB', 18, 75) + UUID('ea5c3c23-ea74-f841-bfa2-8e1983e796be').bytes
                    ],
                endpoints=[
                    EndpointDescriptor(
                        bEndpointAddress=3 | Endpoint.In,
                        bmAttributes=Endpoint.Bulk,
                        wMaxPacketSize=64,
                        bInterval=16),
                    EndpointDescriptor(
                        bEndpointAddress=5,
                        bmAttributes=Endpoint.Bulk,
                        wMaxPacketSize=64,
                        bInterval=16)
                    ]
                ),
            InterfaceDescriptor(
                bInterfaceNumber=3,
                bAlternateSetting=0,
                bInterfaceClass=0x0A,
                bInterfaceSubClass=0,
                bInterfaceProtocol=0xff,
                iInterface=5,
                functional=[
                    pack('<BB', 18, 75) + UUID('ea5c3c23-ea74-f841-bfa2-8e1983e796be').bytes
                    ],
                endpoints=[
                    EndpointDescriptor(
                        bEndpointAddress=4 | Endpoint.In,
                        bmAttributes=Endpoint.Bulk,
                        wMaxPacketSize=64,
                        bInterval=16),
                    EndpointDescriptor(
                        bEndpointAddress=4,
                        bmAttributes=Endpoint.Bulk,
                        wMaxPacketSize=64,
                        bInterval=16)
                    ]
                ),
            ]
        ),
    0x300: LangidsDescriptor([0x409]),
    0x303: StringDescriptor('.debug'),
    0x304: StringDescriptor('tunnel'),
    0x305: StringDescriptor('bluetooth'),
    }

if __name__ == '__main__':
    if len(sys.argv) < 2:
        fout = sys.stdout
    else:
        fout = open(sys.argv[1], 'w')
    print_descriptors(fout, usb_desc, rev_hash=hi.rev_hash)
    fout.close()

    if len(sys.argv) > 2:
        with open(sys.argv[2], 'w') as fout:
            json.dump({
                'device_guid': device_guid,
                'fw_timestamp': hi.timestamp,
                'dfu_padding': 64
                }, fout)
