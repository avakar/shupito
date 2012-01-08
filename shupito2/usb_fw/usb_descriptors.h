struct { uint16_t index, first, last; } const usb_descriptor_map[] = {
    { 0x100, 0x000, 0x012 }, // length: 0x012 (18)
    { 0x200, 0x012, 0x055 }, // length: 0x043 (67)
    { 0x300, 0x055, 0x059 }, // length: 0x004 (4)
    { 0x301, 0x059, 0x069 }, // length: 0x010 (16)
    { 0x302, 0x069, 0x079 }, // length: 0x010 (16)
};

static prog_uint8_t const usb_descriptors[] PROGMEM = {
    0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x20, 0x61, 0x4a, 0x9a, 0x67, 0x01, 0x00, 0x01, 0x02,
    0x00, 0x01, 0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32, 0x09, 0x04, 0x00, 0x00, 0x01,
    0x02, 0x02, 0x01, 0x00, 0x05, 0x24, 0x00, 0x00, 0x01, 0x05, 0x24, 0x01, 0x00, 0x01, 0x04, 0x24,
    0x02, 0x00, 0x05, 0x24, 0x06, 0x00, 0x01, 0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x10, 0x09, 0x04,
    0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, 0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x01, 0x07, 0x05,
    0x84, 0x02, 0x20, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x10, 0x03, 0x53, 0x00, 0x68, 0x00, 0x75,
    0x00, 0x70, 0x00, 0x69, 0x00, 0x74, 0x00, 0x6f, 0x00, 0x10, 0x03, 0x53, 0x00, 0x68, 0x00, 0x75,
    0x00, 0x70, 0x00, 0x69, 0x00, 0x74, 0x00, 0x6f, 0x00,
};
