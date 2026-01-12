#!/usr/bin/env python3
#-*- coding: utf-8 -*-
#
# This file is part of SimulationTeachingElan, a python code used for teaching at Elan Inria.
#
# Copyright 2020 Mickael Ly <mickael.ly@inria.fr> (Elan / Inria - Université Grenoble Alpes)
# SimulationTeachingElan is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# SimulationTeachingElan is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with SimulationTeachingElan.  If not, see <http://www.gnu.org/licenses/>.
#

import numpy as np
from src.rectangle import *
from src.floor import *
from graphics import *
from dynamics import *
from geom import *
from src.circleObject import *
from src.ramp import *
from src.simpleRamp import *
from src.rampA import *



def indexedTest(viewer):
    """
    @brief Demonstration for a basic static rendering
           Renders a simple square 
    """

    # Indexed square
    positions = np.array([0., 0.,   # x0, y0
                          1., 0.,   # x1, y1
                          0., 1.,   # x2, y2
                          1., 1.],  # x3, y3
                         np.float64)
    colours = np.array([1., 0., 0.,  # (r, g, b) for vertex 0
                        0., 0., 1.,  # (r, g, b) for vertex 1
                        0., 1., 0.,  # ...
                        1., 1., 1.]) # ...
    indices = np.array([0, 1, 2,   # First triangle composed by vertices 0, 1 and 2
                        1, 2, 3])  # Second triangle composed by vertices 1, 2 and 3

    # Create the object
    squareMesh = Mesh2D(positions, indices, colours)
    # Create the correspondung GPU object
    squareMeshRenderable = Mesh2DRenderable(squareMesh)
    # Add it to the list of objects to render
    viewer.addRenderable(squareMeshRenderable)

def dynamicTest(viewer):
    """
    @brief Demonstration for a basic dynamic rendering
           Renders a simple square, moved by a dummy dynamic system
    """

    # Indexed square
    positions = np.array([0., 0.,   # x0, y0
                          1., 0.,   # x1, y1
                          0., 1.,   # x2, y2
                          1., 1.],  # x3, y3
                         np.float64)
    colours = np.array([1., 0., 0.,  # (r, g, b) for vertex 0
                        0., 0., 1.,  # (r, g, b) for vertex 1
                        0., 1., 0.,  # ...
                        1., 1., 1.]) # ...
    indices = np.array([0, 1, 2,   # First triangle composed by vertices 0, 1 and 2
                        1, 2, 3])  # Second triangle composed by vertices 1, 2 and 3
    # Create the object
    squareMesh = Mesh2D(positions, indices, colours)
    # Create the correspondung GPU object
    squareMeshRenderable = Mesh2DRenderable(squareMesh)
    # Add it to the list of objects to render
    viewer.addRenderable(squareMeshRenderable)

    # Create a dynamic system
    dyn = DummyDynamicSystem(squareMesh)
    # And add it to the viewer
    # Each frame will perform a call to the 'step' method of the viewer
    viewer.addDynamicSystem(dyn)

def rampASquare(viewer):
    """
    @brief Demonstration for a basic dynamic rendering
           Renders a simple square, moved by a dummy dynamic system
    """

    # Indexed square
    calc = Calculator()
    square_inst = Rectangle(calc)
    print("square hit: ", square_inst.hitbox)
    floor_inst = Floor(calc)
    #square_two = Rectangle(calc, h = 0.1, w = 0.25, mass = 5000, center = np.array([-0.25,1]), attitude = np.array([[1,0], [0,1]]), adhesion = 0.5)
    ramp_inst = RampA(calc)
    print("ramp hit: ", ramp_inst.hitbox)

    print(type(calc.objects[0]))
    print(type(calc.objects[1]))
    print(type(calc.objects[2]))


    # Create the object
    squareMesh = Mesh2D(square_inst.getPositions(), square_inst.indices, square_inst.colours)
    # Create the correspondung GPU object
    squareMeshRenderable = Mesh2DRenderable(squareMesh)
    # Add it to the list of objects to render
    viewer.addRenderable(squareMeshRenderable)

    #square2Mesh = Mesh2D(square_two.getPositions(), square_two.indices, square_two.colours)
    # Create the correspondung GPU object
    #square2MeshRenderable = Mesh2DRenderable(square2Mesh)
    # Add it to the list of objects to render
    #viewer.addRenderable(square2MeshRenderable)


    floorMesh = Mesh2D(floor_inst.getPositions(), floor_inst.indices, floor_inst.colours)
    floorMeshRenderable = Mesh2DRenderable(floorMesh)
    viewer.addRenderable(floorMeshRenderable)

    rampMesh = Mesh2D(ramp_inst.getPositions(), ramp_inst.indices, ramp_inst.colours)
    rampMeshRenderable = Mesh2DRenderable(rampMesh)
    viewer.addRenderable(rampMeshRenderable)
    # Create a dynamic system
    dyn = DummyDynamicSystem(squareMesh, square_inst)
    #dyn2 = DummyDynamicSystem(square2Mesh, square_two)
    # And add it to the viewer
    # Each frame will perform a call to the 'step' method of the viewer
    viewer.addDynamicSystem(dyn)
    #viewer.addDynamicSystem(dyn2)


def floorOnly(viewer):

    # Indexed square
    calc = Calculator()
    square_inst = Rectangle(calc)
    print("square hit: ", square_inst.hitbox)
    floor_inst = Floor(calc)

    # Create the object
    squareMesh = Mesh2D(square_inst.getPositions(), square_inst.indices, square_inst.colours)
    # Create the correspondung GPU object
    squareMeshRenderable = Mesh2DRenderable(squareMesh)
    # Add it to the list of objects to render
    viewer.addRenderable(squareMeshRenderable)


    floorMesh = Mesh2D(floor_inst.getPositions(), floor_inst.indices, floor_inst.colours)
    floorMeshRenderable = Mesh2DRenderable(floorMesh)
    viewer.addRenderable(floorMeshRenderable)
    
    # Create a dynamic system
    dyn = DummyDynamicSystem(squareMesh, square_inst)
    #dyn2 = DummyDynamicSystem(square2Mesh, square_two)
    # And add it to the viewer
    # Each frame will perform a call to the 'step' method of the viewer
    viewer.addDynamicSystem(dyn)
    #viewer.addDynamicSystem(dyn2)


    
def rotatingSquare(viewer):
    """
    @brief Demonstration for a basic dynamic rendering
           Renders a simple square, moved by a dummy dynamic system
    """

    # Indexed square
    calc = Calculator()
    square_inst = Rectangle(calc)
    print("square hit: ", square_inst.hitbox)
    floor_inst = Floor(calc)
    #square_two = Rectangle(calc, h = 0.1, w = 0.25, mass = 5000, center = np.array([-0.25,1]), attitude = np.array([[1,0], [0,1]]), adhesion = 0.5)
    ramp_inst = SimpleRamp(calc)
    print("yaaaaaa: ", ramp_inst.hitbox)


    # Create the object
    squareMesh = Mesh2D(square_inst.getPositions(), square_inst.indices, square_inst.colours)
    # Create the correspondung GPU object
    squareMeshRenderable = Mesh2DRenderable(squareMesh)
    # Add it to the list of objects to render
    viewer.addRenderable(squareMeshRenderable)

    #square2Mesh = Mesh2D(square_two.getPositions(), square_two.indices, square_two.colours)
    # Create the correspondung GPU object
    #square2MeshRenderable = Mesh2DRenderable(square2Mesh)
    # Add it to the list of objects to render
    #viewer.addRenderable(square2MeshRenderable)


    floorMesh = Mesh2D(floor_inst.getPositions(), floor_inst.indices, floor_inst.colours)
    floorMeshRenderable = Mesh2DRenderable(floorMesh)
    viewer.addRenderable(floorMeshRenderable)

    rampMesh = Mesh2D(ramp_inst.getPositions(), ramp_inst.indices, ramp_inst.colours)
    rampMeshRenderable = Mesh2DRenderable(rampMesh)
    viewer.addRenderable(rampMeshRenderable)
    # Create a dynamic system
    dyn = DummyDynamicSystem(squareMesh, square_inst)
    #dyn2 = DummyDynamicSystem(square2Mesh, square_two)
    # And add it to the viewer
    # Each frame will perform a call to the 'step' method of the viewer
    viewer.addDynamicSystem(dyn)
    #viewer.addDynamicSystem(dyn2)

def slidingSquare(viewer):
    """
    @brief Demonstration for a basic dynamic rendering
           Renders a simple square, moved by a dummy dynamic system
    """

    # Indexed square
    calc = Calculator()
    square_inst = Rectangle(calc, center=np.array([-2.5, 0.5]), attitude= np.array([[np.sqrt(2)/2, np.sqrt(2)/2],[-np.sqrt(2)/2, np.sqrt(2)/2]]))
    floor_inst = Floor(calc)
    ramp_inst = Ramp(calc)
    # Create the object
    squareMesh = Mesh2D(square_inst.getPositions(), square_inst.indices, square_inst.colours)
    # Create the correspondung GPU object
    squareMeshRenderable = Mesh2DRenderable(squareMesh)
    # Add it to the list of objects to render
    viewer.addRenderable(squareMeshRenderable)


    floorMesh = Mesh2D(floor_inst.getPositions(), floor_inst.indices, floor_inst.colours)
    floorMeshRenderable = Mesh2DRenderable(floorMesh)
    viewer.addRenderable(floorMeshRenderable)

    rampMesh = Mesh2D(ramp_inst.getPositions(), ramp_inst.indices, ramp_inst.colours)
    rampMeshRenderable = Mesh2DRenderable(rampMesh)
    viewer.addRenderable(rampMeshRenderable)
    # Create a dynamic system
    dyn = DummyDynamicSystem(squareMesh, square_inst)
    # And add it to the viewer
    # Each frame will perform a call to the 'step' method of the viewer
    viewer.addDynamicSystem(dyn)

def rodTest(viewer):

    """
    @brief Demonstration for a rendering of a rod object
           Specific case, as a rod is essentialy a line, we
           need to generate a mesh over it to git it a thickness
           + demonstration of the scaling matrix for the rendering
    """
    positions = np.array([-1., 1.,
                          -1., 0.,
                          -0.5, -0.25],
                         np.float64)
    colours = np.array([1., 0., 0.,
                        0., 1., 0.,
                        0., 0., 1.])

    rod = Rod2D(positions, colours)

    rodRenderable = Rod2DRenderable(rod, thickness = 0.005)
    viewer.addRenderable(rodRenderable)
    
    positionsScaled = np.array([0., 1.,
                                0., 0.,
                                0.5, -0.25],
                               np.float64)
    rodScaled = Rod2D(positionsScaled, colours)

    rodRenderableScaled = Rod2DRenderable(rodScaled, thickness = 0.005)
    rodRenderableScaled.modelMatrix[0, 0] = 2.   # scale in X
    rodRenderableScaled.modelMatrix[1, 1] = 0.75 # scale in Y
    viewer.addRenderable(rodRenderableScaled)
