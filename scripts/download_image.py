import argparse, pickle, io

from urllib import request
from PIL import Image

parser = argparse.ArgumentParser(description="Process parameters.")
parser.add_argument("--img_list", nargs=1)
parser.add_argument("--image_nr", nargs=1)
parser.add_argument("--path", nargs=1)
a = parser.parse_args()

with open(a.img_list[0], 'rb') as fp:
    image_list = pickle.load(fp)

image_data = request.urlopen(image_list[int(a.image_nr[0])]).read()

tpl_image = Image.open(io.BytesIO(image_data))

tpl_image.save(a.path[0])
