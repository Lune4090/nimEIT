#######################################

import std/[sequtils, math]
import arraymancer, results
import plotter, mesh, calc, fem

#######################################

const
  numElectrodes = 36
  diameter = 10.0
  numsInnerVertices = @[30, 24, 18, 12, 6]
  diameters = @[9.0, 7.5, 6.0, 4.0, 2.0]

#######################################

var mesh2d = generate_mesh_circle(numElectrodes, diameter).value

draw_vertices(mesh2d)
draw_mesh(mesh2d)

for i in 0..<(numsInnerVertices.foldl(a+b)):
  var newVertPos: (float, float)
  for (num_layer, num_vert) in numsInnerVertices.pairs():
    if num_layer == 0 and i < numsInnerVertices[0]:
      newVertPos = (diameters[0]*cos((i/num_vert)*2*PI), diameters[0]*sin((i/num_vert)*2*PI))
      break

    elif i < numsInnerVertices[0..num_layer].foldl(a+b):
      newVertPos = (diameters[num_layer]*cos(((i-numsInnerVertices[0..<num_layer].foldl(a+b))/num_vert)*2*PI), diameters[num_layer]*sin(((i-numsInnerVertices[0..<num_layer].foldl(a+b))/num_vert)*2*PI))
      break

  var newVert = Vertice2D(pos: newVertPos, V: 0.0)

  delauney_method_mesh_update(mesh2d, newVert)

echo len(mesh2d.vertices)
echo len(mesh2d.elements)

draw_vertices(mesh2d)
draw_mesh(mesh2d)

let
  stacked_local_stiffness_mat = stack_stiffness_mat_local_tri(mesh2d).value
  stiffness_mat = create_stiffness_mat(mesh2d, stacked_local_stiffness_mat).value

var
  I: seq[float]
for i in 0..<len(mesh2d.vertices):
  I.add(0.0)
for i in 1..17:
  I[i] = 1.0
for i in 19..36:
  I[i] = -1.0

let V = solve(stiffness_mat, I.toTensor) # FV = I(I1, ..., In, 0, ..., 0)
echo V

for (i, vert) in mesh2d.vertices.mpairs():
  vert.V = V[i]

draw_V(mesh2d)