#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 25 15:12:27 2020

@author: 10969theodore
"""
import imageio.v2 as imageio
import os
folder = 'sym1'
path = '/home/robolab/plot_result/cbf_cvt/'+ folder +'/'

def create_gif(image_list, gif_name, duration=0.03):
    frames = []
    for image_name in image_list:
        frames.append(imageio.imread(path+image_name))
    imageio.mimsave(gif_name, frames, 'GIF', duration=duration)
    return

def main():
    image_list = os.listdir(path)
    ims = sorted(image_list)
    new_im_list = []
    for i in range(len(ims)):
        if i%100 == 0:
            new_im_list.append(ims[i])
    gif_name = 'cvt_cbf_cells_'+folder +'.gif'
    duration = 0.03
    create_gif(ims, gif_name, duration)


if __name__ == '__main__':
    main()
