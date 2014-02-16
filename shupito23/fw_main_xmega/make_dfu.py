import sys, os.path
sys.path.append(os.path.join(os.path.split(__file__)[0], '../../fw_common/avrlib'))

import dfusuffix, ihex, struct, StringIO, json, uuid

dev_id = '\xd3\xae\xad\xb0\x7c\x19\x01\x0f'

if __name__ == '__main__':
    ff = StringIO.StringIO()

    with open(sys.argv[3], 'r') as finfo:
        yb_file = json.load(finfo)

    yb_prefix = dfusuffix.YbPrefix()
    device_guid = uuid.UUID(yb_file['device_guid']).bytes
    yb_prefix.add(2, device_guid)
    if 'fw_timestamp' in yb_file:
        yb_prefix.add(4, struct.pack('<I', yb_file['fw_timestamp']))

    with open(sys.argv[1], 'r') as fin:
        hf = ihex.load(fin)

    for addr, block in hf.make_big_blocks(block_size=256, pad_value=0xff):
        yb_prefix.add(2, dev_id)
        yb_prefix.add(3, struct.pack('<I', addr))
        yb_prefix.add(1, struct.pack('<H', len(block)))
        ff.write(yb_prefix.store(64))
        yb_prefix.clear()
        ff.write(block)

    with open(sys.argv[2], 'wb') as fout:
        dfusuffix.add_suffix(StringIO.StringIO(ff.getvalue()), fout, 0x4a61, 0x679c, 0x300, yb_file)
