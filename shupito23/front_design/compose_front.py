with open('front_template.svg', 'r') as fin:
    templ = fin.read()

lines = templ.split('\n')
outlines = list(lines[:53])

rows = 7
cols = 6
xoff = 17
yoff = 30
height = 29
width = 45
reps = 4
i = 0
for x in xrange(cols):
    for y in xrange(rows):
        out = templ.replace('translate(0 24)', 'translate(%d %d)' % (x * width + xoff, y * height + yoff))
        out = out.replace('2014-01', '2014-%02d' % (1+(i // reps)))
        i += 1
        lines = out.split('\n')
        outlines.extend(lines[53:-2])

outlines.extend(lines[-2:])

with open('out.svg', 'w') as fout:
    fout.write('\n'.join(outlines))
