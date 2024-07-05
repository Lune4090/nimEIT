## Outerloop of FEM -> EIT calculation flow

import std/[rdstdin, strutils, os]
import arraymancer, db_connector/db_sqlite, results
import setting, plotter, backward

echo "Welcome to nimEIT! please choose the mode..."
echo "11: forward-non-data-creation"
echo "12: forward-data-creation"
echo "21: backward-single"
echo "22: backward-multiple"
echo "0: exit"

var mode: int
while true:
  let mode_num = readLineFromStdin("Mode: ")
  if mode_num != "0" and mode_num != "11" and mode_num != "12" and mode_num != "21" and mode_num != "22":
    echo "Input is invalid, please try again"
  if mode_num == "11":
    mode = 11
    break
  if mode_num == "12":
    mode = 12
    break
  if mode_num == "21":
    mode = 21
    break
  if mode_num == "22":
    mode = 22
    break
  if mode_num == "0":
    mode = 0
    echo "Good bye!"
    break

if mode == 11:
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

  # TODO: 外部の何かしらのファイル(txt?)から自動的に一定領域内のメッシュの導電率を変えられるようにする
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
    if i == 15:
      vert.I = 1.0
    elif i == 75:
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
  
  draw_V(mesh2d)


if mode == 12:
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
      element.σRef = 1.0
    if (p0[0] + 3.0)^2 + (p0[1] + 2.0)^2 <= 1.5^2:
      element.σRef = 1.0
  

  # modify I
  for (i, vert) in mesh2d.vertices.mpairs():
    if i == 15:
      vert.I = 1.0
    elif i == 75:
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

  #[
    データベースはメッシュ毎に分割する
    SQLは仕様上、第一正規形でもNULLが許されないので、
    テーブルはエレメントと頂点で分ける
    PascalCaseにしているが要検討

    TODO: データベースに新しいページとして、実験番号についての条件をなんかいい感じに纏める
    TODO: データベース(= メッシュ)自体の情報もなんかいい感じに纏める
  ]#

  let
    dbName = readLineFromStdin("Database(mesh) name: ")
    experimentID = readLineFromStdin("Experiment id: ")
  if not fileExists("data/created/" & dbName & ".db"):
    echo dbName & ".db file is not found, new database will be generated"

  let db = open("data/created/" & dbName & ".db", "", "", "")

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


if mode == 21:
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
  
  var
    dbName = readLineFromStdin("Database(mesh) name: ")
  
  while not fileExists("data/created/" & dbName & ".db"):
    echo ".db file is not found, check whether the file is located under data/created"
    dbName = readLineFromStdin("Database(mesh) name: ")
  
  let
    experimentID0 = readLineFromStdin("1st experiment id: ")
    experimentID1 = readLineFromStdin("2nd experiment id: ")
  
  let db = open("data/created/" & dbName & ".db", "", "", "")

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

# TODO: 見通し悪すぎ、関数化進める
if mode == 22:
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
  
  var
    dbName = readLineFromStdin("Database(mesh) name: ")
  
  while not fileExists("data/created/" & dbName & ".db"):
    echo ".db file is not found, check whether the file is located under data/created"
    dbName = readLineFromStdin("Database(mesh) name: ")
  
  var
    input0 = readLineFromStdin("1st experiment ids (separation = ','): ")

  var
    experimentIDs0: seq[int]
    isValidInput0 = true
  
  var tmp = ""
  for i in input0.items():
    if i != ',' and i != '0' and i != '1' and i != '2' and i != '3' and i != '4' and i != '5' and i != '6' and i != '7' and i != '8' and i != '9':
      isValidInput0 = false
      echo "Invalid inputs are found, try to type again"
      break
    elif i == ',':
      experimentIDs0.add(tmp.parseInt)
      tmp = ""
    else:
      tmp = tmp & i
  
  while not isValidInput0:
    input0 = readLineFromStdin("1st experiment ids (separation = ','): ")
    var tmp = ""
    for i in input0.items():
      if i != ',' and i != '0' and i != '1' and i != '2' and i != '3' and i != '4' and i != '5' and i != '6' and i != '7' and i != '8' and i != '9':
        isValidInput0 = false
        echo "Invalid inputs are found, try to type again"
        break
      elif i == ',':
        experimentIDs0.add(tmp.parseInt)
        tmp = ""
      else:
        tmp = tmp & i
  
  var
    input1 = readLineFromStdin("2nd experiment ids (separation = ','): ")

  var
    experimentIDs1: seq[int]
    isValidInput1 = true
  
  tmp = ""
  for i in input1.items():
    if i != ',' and i != '0' and i != '1' and i != '2' and i != '3' and i != '4' and i != '5' and i != '6' and i != '7' and i != '8' and i != '9':
      isValidInput1 = false
      echo "Invalid inputs are found, try to type again"
      break
    elif i == ',':
      experimentIDs1.add(tmp.parseInt)
      tmp = ""
    else:
      tmp = tmp & i
  
  while not isValidInput1:
    input1 = readLineFromStdin("2nd experiment ids (separation = ','): ")
    var tmp = ""
    for i in input1.items():
      if i != ',' and i != '0' and i != '1' and i != '2' and i != '3' and i != '4' and i != '5' and i != '6' and i != '7' and i != '8' and i != '9':
        isValidInput1 = false
        echo "Invalid inputs are found, try to type again"
        break
      elif i == ',':
        experimentIDs1.add(tmp.parseInt)
        tmp = ""
      else:
        tmp = tmp & i

  if len(experimentIDs0) != len(experimentIDs1):
    echo "length of experimentID (1st/2nd) is not same, check it again"
  else:
    for i in 0..<len(experimentIDs0):
      let
        experimentID0 = experimentIDs0[i]
        experimentID1 = experimentIDs1[i]
      var
        I: seq[float]
        V0: seq[float]
        V1: seq[float]
        σ0: seq[float]
        σ1: seq[float]

      echo "Reading database..."
      let db = open("data/created/" & dbName & ".db", "", "", "")

      for row in db.fastRows(sql"SELECT ExperimentID, σRef FROM ElementTable"):
        if row[0].parseInt == experimentID0:
          σ0.add(row[1].parseFloat)
        if row[0].parseInt == experimentID1:
          σ1.add(row[1].parseFloat)

      for row in db.fastRows(sql"SELECT ExperimentID, I, V FROM VerticeTable"):
        if row[0].parseInt == experimentID0:
          I.add(row[1].parseFloat)
          V0.add(row[2].parseFloat)
        if row[0].parseInt == experimentID1:
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
