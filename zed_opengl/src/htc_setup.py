#!/usr/bin/env python

from gl_renderer import OpenVrGlRenderer
from openvr.color_cube_actor import ColorCubeActor
from sdl_app import SdlApp
import sys
import time
import openvr


"""
Minimal sdl programming example which colored OpenGL cube scene that can be closed by pressing ESCAPE.
"""

print ("OpenVR test program")

if openvr.isHmdPresent():
    print ("VR head set found")

if openvr.isRuntimeInstalled():
    print ("Runtime is installed")

vr_system = openvr.init(openvr.VRApplication_Scene)

print (openvr.runtimePath())

print (vr_system.getRecommendedRenderTargetSize())

print (vr_system.isDisplayOnDesktop())


if __name__ == "__main__":
	renderer = OpenVrGlRenderer(multisample=2)
	renderer.append(ColorCubeActor())
	with SdlApp(renderer, "sdl2 OpenVR color cube") as app:
		app.run_loop()
