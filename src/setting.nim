import std/[sequtils, math]
import arraymancer, results
import plotter, mesh, forward

type
  MeshParams* = object
    numElectrodes*: int
    diameter*: float
    numsInnerVertices*: seq[int]
    diameters*: seq[float]
  
proc generate_mesh*(system: MeshParams, drawVert = false, drawMesh = false): Mesh =
  ## input: Parameters
  ## output: Mesh
  
  # Initial-1. Generate circle mesh with outer and center vertices
  var mesh2d = generate_mesh_circle(system.numElectrodes, system.diameter).value

  # Initial-2. Fill the circle mesh with added inner vertices
  for i in 0..<(system.numsInnerVertices.foldl(a+b)):
    var newVertPos: (float, float)
    for (num_layer, num_vert) in system.numsInnerVertices.pairs():
      if num_layer == 0 and i < system.numsInnerVertices[0]:
        newVertPos = (system.diameters[0]*cos((i/num_vert)*2*PI), system.diameters[0]*sin((i/num_vert)*2*PI))
        break

      elif i < system.numsInnerVertices[0..num_layer].foldl(a+b):
        newVertPos = (system.diameters[num_layer]*cos(((i-system.numsInnerVertices[0..<num_layer].foldl(a+b))/num_vert)*2*PI), system.diameters[num_layer]*sin(((i-system.numsInnerVertices[0..<num_layer].foldl(a+b))/num_vert)*2*PI))
        break

    var newVert = Vertice2D(pos: newVertPos, V: 0.0)

    delauney_method_mesh_update(mesh2d, newVert)

  echo "Number of vertices: " & $len(mesh2d.vertices)
  echo "Number of elements: " & $len(mesh2d.elements)
  if drawVert:
    draw_vertices(mesh2d)
  if drawMesh:
    draw_mesh(mesh2d)

  return mesh2d

proc get_stiffness_matrices*(mesh2d: Mesh): (Tensor[float], Tensor[float], Tensor[float]) =
  ## input: Mesh
  ## output: (stackedLocalStiffnessMat, unitStackedLocalStiffnessMat, stiffness_mat)

  # Initial-3. Calc local stiffness matrix
  let
    stackedLocalStiffnessMat = stack_stiffness_mat_local_tri(mesh2d).value
  var  
    unitStackedLocalStiffnessMat = zeros_like(stackedLocalStiffnessMat)

  # Initial-4. Multiply σRef(conductivity) to non-unit local stiffness matrix for each element

  for (i, element) in mesh2d.elements.pairs:
    unitStackedLocalStiffnessMat[i, _] = stackedLocalStiffnessMat[i, _]*element.σRef # ここで伝導率の初期推定値への依存が発生

  # Initial-5. Map local stiffness matrix with conductivity to global large dense stiffness matrix

  let stiffness_mat = create_stiffness_mat(mesh2d, unitStackedLocalStiffnessMat).value

  return (stackedLocalStiffnessMat, unitStackedLocalStiffnessMat, stiffness_mat)