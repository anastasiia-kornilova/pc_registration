import sys
import os
import glob
import logger

from logger import get_configured_logger_by_name
logger = get_configured_logger_by_name(__file__)

PCD_HEADER_SIZE = 11
VELODYNE_MAX_INTENSITY = 255
COLOR_PRECISION = 3
ONE_PCD_TIME = 0.1  # Warning: this is approximation for 10Hz rotation frequency


def convert_file(file_path, new_file_path, conv_type='timestamp'):
    f = open(file_path, 'r')
    new_f = open(new_file_path, 'w')
    first_timestamp = -1

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

        if conv_type == 'intensity':
            parts = l.split(' ')[:-2]
            # The next line done to get lower precision (no need in high)
            # Maybe it can be done in beautiful way using python built-ins
            parts[-1] = str((float(parts[-1]) * 10 ** COLOR_PRECISION
                             // VELODYNE_MAX_INTENSITY) / 10 ** COLOR_PRECISION)
            new_f.write(' '.join(parts) + '\n')
        elif conv_type == 'timestamp':
            if first_timestamp == -1:
                first_timestamp = float(l.split(' ')[-1])
            timestamp = (float(l.split(' ')[-1]) - first_timestamp) / ONE_PCD_TIME
            parts = l.split(' ')[:-2]
            parts[-1] = str(timestamp)
            new_f.write(' '.join(parts) + '\n')
        else:
            logger.error('Type {0} of conversion does not supported'.format(conv_type))
            break

    f.close()
    new_f.close()


# To use: color_extractor.py <path to dir with raw pcd> <path to dir with new pcd with colors>
if __name__ == '__main__':
    ex_dir = sys.argv[1]
    if not os.path.exists(ex_dir):
        logger.error('Folder {0} with PCDs does not exist.'.format(ex_dir))

    new_dir = sys.argv[2]
    if not os.path.exists(new_dir):
        logger.error('Folder {0} for result does not exist.'.format(new_dir))

    files = glob.glob(ex_dir + '/*.pcd')
    files.sort()
    logger.info('Read {0} PCDs from {1}'.format(len(files), ex_dir))

    i = 0
    for file in files:
        convert_file(file, new_dir + f'{i:03d}.pcd')
        i += 1
