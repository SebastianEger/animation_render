import argparse
import flickrapi


parser = argparse.ArgumentParser(description="Process image keywords.")
parser.add_argument("--path", nargs=1)
parser.add_argument("--size", nargs=2)
parser.add_argument('--length', nargs=1)
parser.add_argument('--keywords', nargs='+')
args = parser.parse_args()

api = flickrapi.FlickrAPI(u'7f37f84f7b8454767ff11c683ba2da67', u'6e78737b9b53e6cb', cache=True)

photos_gen = api.walk(text=args.keywords,
                      tag_mode='all',
                      tags=args.keywords,
                      extras='url_c',
                      media='photos',
                      per_page=500)

photos_list = list()

for photo in photos_gen:
    if photo.get('url_c') is None \
            or int(photo.get('height_c')) < int(args.size[0]) \
            or int(photo.get('width_c')) <  int(args.size[1]) \
            or int(photo.get('height_c')) != int(photo.get('width_c')):
        continue
    elif len(photos_list) < int(args.length[0]):
        photos_list.append(photo.get('url_c') + "\n")
    else:
        break

# write list to file
import pickle
with open(args.path[0], 'wb') as fp:
    pickle.dump(photos_list, fp)
