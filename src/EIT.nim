## Outerloop of FEM -> EIT calculation flow

import std/[rdstdin, stats]
import arraymancer except readCsv
import datamancer, results
import setting, plotter, backward

echo "Welcome to nimEIT! please choose the mode..."
echo "1: forward-mode"
echo "2: backward-mode"
echo "0: exit"

var mode: int
while true:
  let mode_num = readLineFromStdin("mode: ")
  if mode_num != "0" and mode_num != "1" and mode_num != "2":
    echo "input is invalid, please try again"
  if mode_num == "1":
    mode = 1
    break
  if mode_num == "2":
    mode = 2
    break
  if mode_num == "0":
    mode = 0
    echo "good bye!"
    break

if mode == 1:
  # Define simulation parameters

  let system = Parameters(
    numElectrodes: 10*12,
    diameter: 10.0,
    numsInnerVertices: @[9*12, 8*12, 7*12, 6*12, 5*12, 4*12, 3*12, 2*12, 1*12, 6],
    diameters: @[9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.5],
  )

  # Initial setting
  var
    mesh2d = initial_setting(system)

  # modify σRef
  for element in mesh2d.elements.mitems():
    var
      p1 = mesh2d.vertices[element.idxVertice1].pos
      p2 = mesh2d.vertices[element.idxVertice2].pos
      p3 = mesh2d.vertices[element.idxVertice3].pos
      p0 = ((p1[0]+p2[0]+p3[0])/3, (p1[1]+p2[1]+p3[1])/3)
    if (p0[0] - 0.0)^2 + (p0[1] - 5.0)^2 <= 2.0^2:
      element.σRef = 0.8
    if (p0[0] + 3.0)^2 + (p0[1] + 2.0)^2 <= 1.5^2:
      element.σRef = 1.5
  

  # modify I
  for (i, vert) in mesh2d.vertices.mpairs():
    if i == 0:
      vert.I = 1.0
    elif i == 60:
      vert.I = -1.0
    else:
      vert.I = 0.0

  # Get stiffness matrices
  var
    (stackedLocalStiffnessMat, unitStackedLocalStiffnessMat, stiffness_mat) = get_stiffness_matrices(mesh2d)

  discard stackedLocalStiffnessMat
  discard unitStackedLocalStiffnessMat

  # Forward. Solve KV=I based on Galerkin method and update V, then get the voltage mapping
  var σRef: seq[float]
  for (i, elem) in mesh2d.elements.pairs():
    σRef.add(elem.σRef)
  
  var I: seq[float]
  for (i, vert) in mesh2d.vertices.pairs():
    I.add(vert.I)
  
  let V = solve(stiffness_mat, I.toTensor)
  for (i, vert) in mesh2d.vertices.mpairs():
    vert.V = V[i]

  echo "I: " & $I
  echo "V: " & $V.toSeq1D
  echo "σRef: " & $σRef

  let csvName = readLineFromStdin("type data file name: ")
  
  let df = toDf({
    "I": I,
    "V": V,
    "σRef": σRef,
  })

  df.write_csv("data/created/" & csvName & ".csv", precision = 16)

  draw_V(mesh2d)


if mode == 2:
  # Define simulation parameters

  let system = Parameters(
    numElectrodes: 10*12,
    diameter: 10.0,
    numsInnerVertices: @[9*12, 8*12, 7*12, 6*12, 5*12, 4*12, 3*12, 2*12, 1*12, 6],
    diameters: @[9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.5],
  )

  # Initial setting
  var
    mesh2d = initial_setting(system)
  
  # modify parameters
  
  let
    csvName1 = readLineFromStdin("type 1st data file name: ")
    df1 = read_csv("data/created/" & csvName1 & ".csv")
    csVname2 = readLineFromStdin("type 2nd data file name: ")
    df2 = read_csv("data/created/" & csvName2 & ".csv")
  
  for (i, elem) in mesh2d.elements.mpairs():
    elem.σRef = df1["σRef", i, float]
    elem.Δσ = df2["σRef", i, float] - df1["σRef", i, float]
  for (i, vert) in mesh2d.vertices.mpairs():
    vert.I = df1["I", i, float]
    vert.V = df1["V", i, float]
    vert.ΔV = df2["V", i, float]-df1["V", i, float]

  # Get stiffness matrices
  var
    (stackedLocalStiffnessMat, unitStackedLocalStiffnessMat, stiffness_mat) = get_stiffness_matrices(mesh2d)

  discard stackedLocalStiffnessMat

  # Backward-1. Calculate jacobian from global / local stiffness matrix and outer node's voltages
  let jac = mesh2d.compute_jac_2d_tri(stiffness_mat, unitStackedLocalStiffnessMat).value

  # Backward-2. Converge RMS based on differential re-construction method with regularization term
  let
    coef = jac.δσ_over_δV().value
    δσ = mesh2d.reconstruct_δσ(coef).value
  
  
  
  var
    errors: RunningStat

  for (i, elem) in mesh2d.elements.mpairs():
    elem.δσ = δσ[i]
    errors.push(abs(elem.δσ - elem.Δσ))
  
  echo errors

  # Backward-3. Get the reconstructed image !
  
  draw_Δσ(mesh2d)
  draw_δσ(mesh2d)

