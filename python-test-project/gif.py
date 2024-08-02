#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 25 15:12:27 2020

@author: 10969theodore
"""
import imageio

def create_gif(image_list, gif_name, duration=0.35):
    frames = []
    for image_name in image_list:
        frames.append(imageio.imread(image_name))
    imageio.mimsave(gif_name, frames, 'GIF', duration=duration)
    return

path =

def main():
    image_list = os.listdir(path)
    gif_name = 'CVT-Vary.gif'
    duration = 0.0625
    create_gif(image_list, gif_name, duration)


if __name__ == '__main__':
    main()
