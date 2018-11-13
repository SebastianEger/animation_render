import argparse
import flickrapi

# Parse arugments
parser = argparse.ArgumentParser(description="Process image keywords.")
parser.add_argument("--path", nargs=1)
parser.add_argument("--size", nargs=2)
parser.add_argument('--length', nargs=1)
parser.add_argument('--keywords', nargs='+')
args = parser.parse_args()

# Init flickr api
api = flickrapi.FlickrAPI(u'7f37f84f7b8454767ff11c683ba2da67', u'6e78737b9b53e6cb', cache=True)

# Get photos
photos_gen = api.walk(text=args.keywords,
                      tag_mode='any',
                      tags=args.keywords,
                      extras='url_c',
                      media='photos',
                      safe_search='1',
                      per_page=50)
# Init photo list
photos_list = list()

iteration = 0
for photo in photos_gen:
    if(len(photos_list) < int(args.length[0])):
        # Check photo properties
        iteration = iteration + 1
        if photo.get('url_c') is None \
                or int(photo.get('height_c')) != int(photo.get('width_c')):
                # or int(photo.get('height_c')) < int(args.size[0]) \
                # or int(photo.get('width_c'))  < int(args.size[1]):
                # or int(photo.get('height_c')) != int(photo.get('width_c')):
            print(str(iteration) + " - Photo not usable.")
            continue
        else:
            new_url = photo.get('url_c') + "\n"

            if new_url not in photos_list:
                photos_list.append(new_url)
                print(str(iteration) + " - New photo added to list. Photos added to list: " + str(len(photos_list)) )
            else:
                print(str(iteration) + " - Photo already in list")
    else:
        print("List created.")
        break

# Write list to file
import pickle
with open(args.path[0], 'wb') as fp:
    pickle.dump(photos_list, fp)
