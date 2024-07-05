## Outerloop of FEM -> EIT calculation flow

import std/[rdstdin, strutils]
import arraymancer, db_connector/db_sqlite, results
import setting, plotter, backward

echo "Welcome to nimEIT! please choose the mode..."
echo "1: forward-mode"
echo "2: backward-mode"
echo "0: exit"

var mode: int
while true:
  let mode_num = readLineFromStdin("Mode: ")
  if mode_num != "0" and mode_num != "1" and mode_num != "2":
    echo "Input is invalid, please try again"
  if mode_num == "1":
    mode = 1
    break
  if mode_num == "2":
    mode = 2
    break
  if mode_num == "0":
    mode = 0
    echo "Good bye!"
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
      element.σRef = 2.0
  

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

  #echo "I: " & $I
  #echo "V: " & $V.toSeq1D
  #echo "σRef: " & $σRef

  #[
    データベースはメッシュ毎に分割する
    SQLは仕様上、第一正規形でもNULLが許されないので、
    テーブルはエレメントと頂点で分ける
    PascalCaseにしているが要検討

    TODO: 
      タイプミスや誤った入力で変なdbやtableが作られないように
      フォルダ内を調べて対象のファイルが無かったら警告を出すようにする
  ]#

  let
    dbName = readLineFromStdin("Database(mesh) name: ")
    experimentID = readLineFromStdin("Experiment id: ")
  
  let db = open("data/created/" & $dbName & ".db", "", "", "")

  echo "Writing database..."
    
  db.exec(sql"""CREATE TABLE IF NOT EXISTS ElementTable (
                ExperimentID INTEGER,
                ElementID INTEGER,
                σRef FLOAT
            )""")
  
  db.exec(sql"""CREATE TABLE IF NOT EXISTS VerticeTable (
                ExperimentID INTEGER,
                VerticeID INTEGER,
                I FLOAT,
                V FLOAT
            )""")

  for (i, elem) in mesh2d.elements.pairs():    
    db.exec(sql"INSERT INTO ElementTable (ExperimentID, ElementID, σRef) VALUES (?, ?, ?)",
      $experimentID.parseInt, $i, $elem.σRef)

  for (i, vert) in mesh2d.vertices.pairs():    
    db.exec(sql"INSERT INTO VerticeTable (ExperimentID, VerticeID, I, V) VALUES (?, ?, ?, ?)",
      $experimentID.parseInt, $i, $vert.I, $vert.V)

  echo "Database is updated"

  db.close()

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
  
  # read parameters
  
  let
    dbName = readLineFromStdin("Database(mesh) name: ")
    experimentID0 = readLineFromStdin("Experiment id 1st: ")
    experimentID1 = readLineFromStdin("Experiment id 2nd: ")
  
  let db = open("data/created/" & $dbName & ".db", "", "", "")

  echo "Reading database..."

  var
    I: seq[float]
    V0: seq[float]
    V1: seq[float]
    σ0: seq[float]
    σ1: seq[float]

  for row in db.fastRows(sql"SELECT ExperimentID, σRef FROM ElementTable"):
    if row[0].parseInt == experimentID0.parseInt:
      σ0.add(row[1].parseFloat)
    if row[0].parseInt == experimentID1.parseInt:
      σ1.add(row[1].parseFloat)

  for row in db.fastRows(sql"SELECT ExperimentID, I, V FROM VerticeTable"):
    if row[0].parseInt == experimentID0.parseInt:
      I.add(row[1].parseFloat)
      V0.add(row[2].parseFloat)
    if row[0].parseInt == experimentID1.parseInt:
      V1.add(row[2].parseFloat)

  db.close()
 
  for (i, elem) in mesh2d.elements.mpairs():
    elem.σRef = σ0[i]
    elem.Δσ = σ1[i] - σ0[i]
  for (i, vert) in mesh2d.vertices.mpairs():
    vert.I = I[i]
    vert.V = V0[i]
    vert.ΔV = V1[i] - V0[i]

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

  #var
  #  errors: RunningStat

  for (i, elem) in mesh2d.elements.mpairs():
    elem.δσ = δσ[i]
  #  errors.push(abs(elem.δσ - elem.Δσ))
  
  #echo errors

  # Backward-3. Get the reconstructed image !
  
  draw_Δσ(mesh2d)
  draw_δσ(mesh2d)

