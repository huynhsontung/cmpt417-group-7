import os
from pathlib import Path


MAP_PATH = './benchmark/Berlin_1_256.map'
SCENE_PATH = './benchmark/scen-even/Berlin_1_256-even-1.scen'
OUT_PATH = './benchmark/instances'
AGENTS = 10

map_name = Path(MAP_PATH).name.split('.')[0]

with open(MAP_PATH) as map_file:
    map_file.readline() # discard
    height = int(map_file.readline().split()[1])
    width = int(map_file.readline().split()[1])
    map_file.readline() # discard
    map = map_file.readlines()

with open(SCENE_PATH) as scene_file:
    scene_file.readline()   # discard
    lines = scene_file.readlines()
    lines = [x.split() for x in lines]
    agents = [(x[4], x[5], x[6], x[7]) for x in lines]

for i in range(2, AGENTS + 1):
    with open(os.path.join(OUT_PATH, '{}_{}'.format(map_name, i)), 'w') as out_file:
        out_file.write('%d\n' % height)
        out_file.write('%d\n' % width)
        out_file.writelines(map)
        out_file.write('\n')
        agents_lines = [' '.join(a) + '\n' for a in agents[:i]]
        out_file.writelines(agents_lines)


