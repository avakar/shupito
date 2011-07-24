struct { uint16_t index, first, last; } const usb_descriptor_map[] = {
    { 0x100, 0, 18 },
    { 0x200, 18, 50 },
    { 0x300, 50, 54 },
    { 0x301, 54, 90 },
    { 0x302, 90, 116 },
};

static prog_uint8_t const usb_descriptors[] PROGMEM = {
    0x12, 0x01, 0x00, 0x02, 0xff, 0xff, 0xff, 0x20, 0x61, 0x4a, 0x9a, 0x67, 0x01, 0x00, 0x01, 0x02,
    0x00, 0x01, 0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32, 0x09, 0x04, 0x01, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x01, 0x07, 0x05, 0x84, 0x02, 0x20,
    0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x24, 0x03, 0x4d, 0x00, 0x61, 0x00, 0x6e, 0x00, 0x75, 0x00,
    0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x74, 0x00, 0x75, 0x00, 0x72, 0x00, 0x65, 0x00, 0x72, 0x00,
    0x20, 0x00, 0x4e, 0x00, 0x61, 0x00, 0x6d, 0x00, 0x65, 0x00, 0x1a, 0x03, 0x50, 0x00, 0x72, 0x00,
    0x6f, 0x00, 0x64, 0x00, 0x75, 0x00, 0x63, 0x00, 0x74, 0x00, 0x20, 0x00, 0x4e, 0x00, 0x61, 0x00,
    0x6d, 0x00, 0x65, 0x00,
};
