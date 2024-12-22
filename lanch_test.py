import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Test the lancher')
    parser.add_argument('--name', type=str, default='lancher', help='The name of the lancher')
    parser.add_argument('--age', type=int, default=18, help='The age of the lancher')
    return parser.parse_args()


if __name__ == '__main__':

    args = parse_args()
    print(args)

    pass