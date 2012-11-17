import sys
fin = list(open(sys.argv[1], 'r'))

components = {}
for line in fin[10:]:
    toks = line.split()
    if len(toks) != 6:
        continue
    name, value, type, package, lib, sheet = toks
    component = value, type, package
    if component not in components:
        components[component] = 1
    else:
        components[component] += 1

components = [(cnt, value, type, package) for (value, type, package), cnt in components.iteritems()]
components.sort(key=lambda x: (x[2], x[1]))

for x in components:
    print '%5d %25s %25s %25s' % x
