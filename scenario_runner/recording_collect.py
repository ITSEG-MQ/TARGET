import argparse
import time

import cv2
import os
from scenario_parser import run

def img2mp4(image_folder, video_name):
    # image_folder = 'recordings/mmfn/rule1-3/'
    # video_name = 'test_video.mp4'

    images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
    lssorted = sorted(images, key=lambda x: int(os.path.splitext(x)[0]))
    print(lssorted)

    video_FourCC = cv2.VideoWriter_fourcc(*"mp4v")
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape
    # print(height, width, layers)
    video = cv2.VideoWriter(video_name, video_FourCC, 15, (width,height))

    for image in lssorted:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()


def run_scenario(rule, model, town, reload=1):
    run(rule, model, reload, 'single', town)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', default='generated_yaml/rule2.yaml')
    parser.add_argument('-m', '--model', default='mmfn')
    parser.add_argument('-r', '--reload', default=1)
    parser.add_argument('-t', '--town', default=5)

    args, unknown = parser.parse_known_args()
    # print(args)

    # run_scenario(args.file, args.model, args.town, args.reload)
    # dirs = os.listdir('recordings/auto')
    # for dir in dirs:
    #     try:
    #         img2mp4('recordings/auto/{}'.format(dir), 'recordings/auto_recording/{}.mp4'.format(dir))
    #     except:
    #         pass
    img2mp4('recordings/mmfn/rule2-10', 'recordings/auto_recording/mmfn_2_10.mp4')

    # for rule in range(3, 4):
    #     for town in [10]:
    #         run_scenario('generated_yaml/rule{}.yaml'.format(rule), 'auto', town)
    #         time.sleep(10)