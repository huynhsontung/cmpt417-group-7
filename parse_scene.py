import os
import argparse
from pathlib import Path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run PointNet network')
    parser.add_argument('--map', type=str, required=True,
                        help='Path to map file')
    parser.add_argument('--scene', type=str, required=True,
                        help='Path to scene file')
    parser.add_argument('--out', type=str, required=True,
                        help='Path to output directory')
    parser.add_argument('--agents', type=int, default=10,
                        help='Number of agents to generate')

    args = parser.parse_args()

    MAP_PATH = args.map
    SCENE_PATH = args.scene
    OUT_PATH = args.out
    AGENTS = args.agents

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
        with open(os.path.join(OUT_PATH, '{}_{}'.format(map_name, str(i).zfill(3))), 'w') as out_file:
            out_file.write('{} {}\n'.format(height, width))
            out_file.writelines(map)
            if '\n' not in map[-1]:
                out_file.write('\n')
            out_file.write('{}\n'.format(i))
            agents_lines = [' '.join(a) + '\n' for a in agents[:i]]
            out_file.writelines(agents_lines)


