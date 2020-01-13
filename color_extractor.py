import sys
import glob

PCD_HEADER_SIZE = 11
VELODYNE_MAX_INTENSITY = 255
COLOR_PRECISION = 3


def convert_file(file_path, new_file_path):
    f = open(file_path, 'r')
    new_f = open(new_file_path, 'w')

    i = 0
    for l in f:
        if i < PCD_HEADER_SIZE:
            if l.startswith('FIELDS'):
                new_f.write('FIELDS x y z rgb\n')
            elif l.startswith('SIZE'):
                new_f.write('SIZE 4 4 4 4\n')
            elif l.startswith('TYPE'):
                new_f.write('TYPE F F F F\n')
            elif l.startswith('COUNT'):
                new_f.write('COUNT 1 1 1 1\n')
            else:
                new_f.write(l)
            i += 1
            continue

        parts = l.split(' ')[:-2]
        # The next line done to get lower precision (no need in high)
        # Maybe it can be done in beautiful way using python built-ins
        parts[-1] = str((float(parts[-1]) * 10 ** COLOR_PRECISION
                         // VELODYNE_MAX_INTENSITY) / 10 ** COLOR_PRECISION)
        new_l = ' '.join(parts) + '\n'
        new_f.write(new_l)

    f.close()
    new_f.close()


# To use: color_extractor.py <path to dir with raw pcd> <path to dir with new pcd with colors>
if __name__ == '__main__':
    ex_dir = sys.argv[1]
    new_dir = sys.argv[2]
    files = glob.glob(ex_dir + '/*.pcd')
    files.sort()
    i = 0
    for file in files:
        convert_file(file, new_dir + f'{i:03d}.pcd')
        i += 1
