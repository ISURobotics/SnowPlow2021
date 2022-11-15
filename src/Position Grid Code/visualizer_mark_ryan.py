"""
    File: visualizer_mark_ryan.py
    Author: ISU Robotics - Snow Plow
    Creation Date: 11/14/2022
"""

import recast
import time
import random
import keyboard
import PySimpleGUI as sg
from shapely.geometry import *


class Agent:
    def __init__(self):
        self.origin = (800, 150)
        self.size = 5

    def update(self, pose, field, graph):
        if self.withinBounds(pose, field):
            graph.DrawCircle((self.origin[0] + round(pose.getPosition()['X'] * 100),
                              self.origin[1] + round(pose.getPosition()['Y'] * 100)), self.size, fill_color="red")
        else:
            print("Pose Failure: Out of Bounds")

    def withinBounds(self, pose, field):
        po = Point((self.origin[0] + round(pose.getPosition()['X'] * 100),
                    self.origin[1] + round(pose.getPosition()['Y'] * 100)))
        return field.innerBounds.contains(po)


class Field:
    def __init__(self):
        self.absWidth = 1500
        self.absHeight = 700
        self.cordsOrigin = (0, 0)
        self.cordsExtent = (self.absWidth, self.absHeight)
        self.primaryConnectors = [(0, 300), (550, 300), (550, 0), (1050, 0), (1050, 300), (1500, 300),
                          (self.absWidth, self.absHeight), (0, self.absHeight)]
        self.innerBounds = Polygon(self.primaryConnectors)


def initializeGraphics(field):
    # screenWidth, screenHeight = sg.Window.get_screen_size()
    layout = [
        [sg.Graph(
            canvas_size=(field.absWidth, field.absHeight),
            graph_bottom_left=field.cordsOrigin,
            graph_top_right=field.cordsExtent,
            pad=(0, 0),
            background_color='#333333',
            key='graph'
        )]
    ]
    window = sg.Window('Window Title', layout, no_titlebar=True, resizable=False, location=(0, 0), margins=(0, 0),
                       size=(field.absWidth, field.absHeight), background_color='#333333', keep_on_top=True,
                       finalize=True)
    graph = window.Element("graph")

    graph.DrawPolygon(field.primaryConnectors, fill_color='#3C3C3C', line_color="white", line_width=4)

    gridScale = 25
    for x in range(gridScale, field.absWidth+1, gridScale):
        for y in range(gridScale, field.absHeight+1, gridScale):
            graph.DrawRectangle((x - gridScale, y - gridScale), (x, y), line_color="#434343", line_width=2)

    return window, graph


def run(agent, field):
    window, graph = initializeGraphics(field)

    while True:
        try:
            if keyboard.is_pressed('q'):
                break
            else:
                agent.update(poseData[random.randrange(0, len(poseData))], field, graph)
                window.refresh()
                time.sleep(0.01)
        except IndexError:
            continue


poseData = recast.importPoseData("./Inputs/pose.txt")
if __name__ == '__main__':
    newAgent = Agent()
    newField = Field()
    run(newAgent, newField)
