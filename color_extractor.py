import sys

PCD_HEADER_SIZE = 11
VELODYNE_MAX_INTENSITY = 255
COLOR_PRECISION = 3

# To use: color_extractor.py <path to raw pcd> <path to new pcd with colors>
if __name__ == '__main__':
    ex_filename = sys.argv[1]
    new_filename = sys.argv[2]
    f = open(ex_filename, 'r')
    new_f = open(new_filename, 'w')

    # TODO: add convertion of header -- now it should be converted manually
    i = 0
    for l in f:
        if i < PCD_HEADER_SIZE:
            new_f.write(l)
            i += 1
            continue

        parts = l.split(' ')[:-2]
        # The next line done to get lower precision (no need in high)
        # Maybe it can be done in beautiful way using python built-ins
        parts[-1] = str((float(parts[-1]) * 10**COLOR_PRECISION
                         // VELODYNE_MAX_INTENSITY) / 10**COLOR_PRECISION)
        new_l = ' '.join(parts) + '\n'
        new_f.write(new_l)

    f.close()
    new_f.close()
