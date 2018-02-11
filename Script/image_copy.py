import os
import argparse
import tqdm


from shutil import copyfile

""" 
This script will copy images from source and 
will skip a given range on copy to generate 
a better dataset with less similar images.
"""


def parse_args(self):
    """
    Parse script arguments.
    :return: parsed args
    """
    parser = argparse.ArgumentParser(description="An image copy utility to generate our dataset.")

    parser.add_argument('-s', '--source_dir', required=True,
                        help='Source directory containing images')
    parser.add_argument('-o', '--output_dir', required=True,
                        help='Output directory')
    parser.add_argument('-c', '--skip_count', required=True,
                        help='how many images will be skipped between copy')

    return parser.parse_args()


def walk_dir(folder_path):
    """Walk through each files in a directory"""
    for dir_path, dirs, files in os.walk(folder_path):
        for filename in files:
            yield os.path.abspath(os.path.join(dir_path, filename))


def process_copy_with_progress(source_path, dest_path, skip_count):

    """
    Copy file from source to destination directory depending on skip count.
    :param source_path: File source path
    :param dest_path:  Destination file path
    :param skip_count: Modulo count of to skip files
    """
    # Preprocess the total files count
    file_counter = 0
    for file_path in walk_dir(source_path):
        file_counter += 1

    current_file_index = 0
    for source_path in tqdm(walk_dir(source_path), total=file_counter, unit="files"):
        if current_file_index % skip_count == 0:
            file_name = os.path.basename(source_path)
            file_dest_path = os.path.join(dest_path, file_name)
            copyfile(source_path, file_dest_path)


def main():
    """
    Image copy script main method.
    """

    parsed_args = parse_args()

    process_copy_with_progress(parsed_args.source_dir,
                               parsed_args.output_dir,
                               parsed_args.skip_count)


if __name__ == '__main__':
    main()





