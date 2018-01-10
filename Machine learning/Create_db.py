import argparse

from scipy.io import savemat, loadmat


def main():

    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--input_file", help="Input Mat file.")

    args = parser.parse_args()

    input_file = args.input_file

    input_file = input_file.split(' ')

    img, label = [], []

    for files in input_file:
        data = loadmat(files)

        X, Y = data['X'], data['Y']

        Y = Y.flatten()

        print Y.shape
        i = 0
        for x, y in zip(X, Y):
            if i % 2 == 0:
                img.append(x)
                label.append(y)
            i += 1

    dic = {'X': img, 'Y': label}
    savemat('Data_img.mat', dic)


if __name__ == '__main__':
    main()
