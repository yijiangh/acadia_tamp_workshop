import imageio.v2 as imageio
import os

# This script uses imageio to create a gif from a series of images.
#  To install imageio, use the following command:
#  >> pip install imageio

# Make a folder next to the location of this script.
# Put all the images in that folder. Do not put any other images in that folder.
# Images must not have any transparency layers.
# The filename of the images should be in numerical order, for example 000000.png, 000001.png, etc.

folder_name = 'animate'
extension = 'jpg'

# Set the duration of each frame in milliseconds.
frame_duration_ms = 50
# -------------------------------------------------------
# There should be no need to make changes below this line
# -------------------------------------------------------

HERE = os.path.dirname(__file__)
folder = os.path.join(HERE, folder_name)
filenames = sorted((fn for fn in os.listdir(
    folder) if fn.endswith('.' + extension)))

print("Total number of images: {}".format(len(filenames)))
print("Now creating the gif...")

output_file = os.path.join(HERE, folder_name + '.gif')
with imageio.get_writer(output_file, mode='I', duration=frame_duration_ms) as writer:
    for filename in filenames:
        image = imageio.imread(os.path.join(folder, filename))
        writer.append_data(image)
