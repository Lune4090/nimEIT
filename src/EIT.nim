#######################################

import std/[sequtils, math]
import arraymancer, results
import plotter, mesh, fem

#######################################

const
  numElectrodes = 36
  diameter = 10.0
  numsInnerVertices = @[30, 24, 18, 12, 6]
  diameters = @[9.0, 7.5, 6.0, 4.0, 2.0]

#######################################

var mesh2d = generate_mesh_circle(numElectrodes, diameter).value

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

echo "Number of vertices: " & $len(mesh2d.vertices)
echo "Number of elements: " & $len(mesh2d.elements)

draw_vertices(mesh2d)
draw_mesh(mesh2d)

let
  stackedLocalStiffnessMat = stack_stiffness_mat_local_tri(mesh2d).value
var  
  unitStackedLocalStiffnessMat = zeros_like(stackedLocalStiffnessMat)

# 無次元剛性行列に導電率を乗算
var
  σs: seq[float]
for vertice in mesh2d.vertices.items:
  σs.add(vertice.σ)
for (i, element) in mesh2d.elements.pairs:
  unitStackedLocalStiffnessMat[i, _] = stackedLocalStiffnessMat[i, _]*((σs[element.idxVertice1] + σs[element.idxVertice2] + σs[element.idxVertice3])/3)

# set current
for (i, vert) in mesh2d.vertices.mpairs:
  if i mod 12 == 0:
    vert.I = 1.0
  elif i mod 12 == 6:
    vert.I = -1.0
  else:
    vert.I = 0.0

var
  I: seq[float]
for (i, vert) in mesh2d.vertices.pairs:
  I.add(vert.I)

let stiffness_mat = create_stiffness_mat(mesh2d, unitStackedLocalStiffnessMat).value

let V = solve(stiffness_mat, I.toTensor) # FV = I(I1, ..., In, 0, ..., 0)
echo V

for (i, vert) in mesh2d.vertices.mpairs():
  vert.V = V[i]

draw_V(mesh2d)

let jac = mesh2d.compute_jac_2d_tri(stiffness_mat, unitStackedLocalStiffnessMat).value