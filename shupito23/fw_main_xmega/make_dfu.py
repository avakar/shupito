import sys, os.path
sys.path.append(os.path.join(os.path.split(__file__)[0], '../../fw_common/avrlib'))

import dfusuffix, ihex, struct, StringIO, json

if __name__ == '__main__':
    ff = StringIO.StringIO()

    with open(sys.argv[1], 'r') as fin:
        hf = ihex.load(fin)
    for addr, block in hf.make_big_blocks(block_size=256, pad_value=0xff):
        header = struct.pack('<III', 1, addr, len(block)) + '\x00'*52
        ff.write(header)
        ff.write(block)

    with open(sys.argv[3], 'r') as finfo:
        yb_file = json.load(finfo)

    with open(sys.argv[2], 'wb') as fout:
        dfusuffix.add_suffix(StringIO.StringIO(ff.getvalue()), fout, 0x4a61, 0x679c, 0x300, yb_file)
