#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json  
import math  
import sys  
import signal  
from datetime import datetime  
import matplotlib.pyplot as plt  
from matplotlib.animation import FuncAnimation, FFMpegWriter  
from matplotlib.offsetbox import OffsetImage, AnnotationBbox  
from PIL import Image  
import numpy as np  
import os  
from matplotlib.widgets import Slider

class GeneralLogVisualizer:
    def __init__(self, log_file, map_file, resolution, x_offset=0, y_offset=0, yaw_offset=0, robot_img_file=None, enemy_img_file=None):
        self.log_file = log_file  
        self.map_file = map_file  
        self.resolution = resolution  
        self.x_offset = x_offset  
        self.y_offset = y_offset  
        self.yaw_offset = yaw_offset  
        self.robot_img_file = robot_img_file  
        self.enemy_img_file = enemy_img_file  
        self.fig, self.ax = plt.subplots(figsize=(19.2, 10.8))  
        self.colors = ['black', 'blue', 'green', 'red', 'purple', 'orange', 'brown', 'pink', 'gray', 'olive', 'cyan', 'magenta', 'yellow', 'navy', 'teal']
        self.text_boxes = []  
        with open(self.log_file, 'r') as f:  
            self.log_data = json.load(f)  
        for entry in self.log_data:
            entry['timestamp'] = datetime.strptime(entry['timestamp'], "%Y-%m-%dT%H:%M:%S.%fZ")
        self.log_data.sort(key=lambda x: x['timestamp'])
        self.fields = list(self.log_data[0].keys())
        for i in range(len(self.fields)):
            text_box = self.ax.text(-0.75, 0.95 - 0.045 * i, '', color=self.colors[i % len(self.colors)], fontsize=16, transform=self.ax.transAxes, verticalalignment='top', wrap=True)
            self.text_boxes.append(text_box)
        self.map_image, self.map_extent = self.load_pgm_map(self.map_file)
        self.ax.imshow(self.map_image, extent=self.map_extent, origin='upper', cmap='gray')  
        self.robot_img = plt.imread(self.robot_img_file)  
        self.enemy_img = plt.imread(self.enemy_img_file)  
        self.robot_icon = OffsetImage(self.robot_img, zoom=0.07)
        self.enemy_icon = OffsetImage(self.enemy_img, zoom=0.1)
        self.robot_icon.image.axes = self.ax
        self.enemy_icon.image.axes = self.ax
        self.robot_abox = AnnotationBbox(self.robot_icon, [0, 0], frameon=False)
        self.enemy_abox = AnnotationBbox(self.enemy_icon, [0, 0], frameon=False)
        self.ax.add_artist(self.robot_abox)
        self.ax.add_artist(self.enemy_abox)
        self.fig.subplots_adjust(left=0.4, right=0.95, top=0.95, bottom=0.05)
        self.ax.axis('off')  
        ax_slider = plt.axes([0.1, 0.01, 0.8, 0.03])
        self.slider = Slider(ax_slider, 'Time', 0, len(self.log_data) - 1, valinit=0, valstep=1)
        self.slider.on_changed(self.update_slider)

    def load_pgm_map(self, path):
        img = Image.open(path)
        width, height = img.size
        extent = (0, width * self.resolution, 0, height * self.resolution)
        return img, extent

    def update_plot(self, frame):
        if frame >= len(self.log_data):
            print("动画结束")
            plt.close('all')
            sys.exit(0)
        data = self.log_data[frame]
        for i, key in enumerate(self.fields):
            val = data[key]
            if isinstance(val, datetime):
                val = val.strftime('%H:%M:%S.%f')
            self.text_boxes[i].set_text(f"{key} : {val}")
        rx, ry = data['world_x'], data['world_y']
        ex, ey = data['enemy_x'], data['enemy_y']
        self.ax.clear()
        self.ax.imshow(self.map_image, extent=self.map_extent, origin='upper', cmap='gray')
        self.ax.axis('off')
        self.robot_abox = AnnotationBbox(self.robot_icon, [rx, ry], frameon=False)
        self.enemy_abox.xy = [ex, ey]
        self.ax.add_artist(self.robot_abox)
        self.ax.add_artist(self.enemy_abox)
        for text_box in self.text_boxes:
            self.ax.add_artist(text_box)
        return self.ax.artists

    def update_slider(self, val):
        frame = int(self.slider.val)
        self.update_plot(frame)

    def animate(self):
        signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))
        anim = FuncAnimation(self.fig, self.update_plot, frames=len(self.log_data), interval=100, repeat=False)
        Writer = FFMpegWriter(fps=10, metadata={'title': 'robot log'}, bitrate=1800)
        anim.save("robot_log.mp4", writer=Writer)
        plt.show()

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.realpath(__file__))
    log_file = os.path.join(script_dir, 'output.json')
    map_file = os.path.join(script_dir, '../scripts/map/map.pgm')
    robot_img_file = os.path.join(script_dir, '../scripts/sources/sentry.png')
    enemy_img_file = os.path.join(script_dir, '../scripts/sources/enemy.png')
    print(f"Log file path: {log_file}")
    print(f"Map file path: {map_file}")
    print(f"Robot image file path: {robot_img_file}")
    print(f"Enemy image file path: {enemy_img_file}")
    resolution = 0.05
    x_offset = 21.437990
    y_offset = 17.812218
    yaw_offset = math.radians(0)
    visualizer = GeneralLogVisualizer(log_file, map_file, resolution, x_offset, y_offset, yaw_offset, robot_img_file, enemy_img_file)
    visualizer.animate()
