## 1. 前半は静電場FEM、後半は差分再構成法を使用したEIT
## 2. 領域外形は円形を仮定
## 3. センサ位置は領域外形を等分したものと仮定
## 4. メッシュ分割アルゴリズムはdelauney法(https://ryutai.ninja-web.net/mesh/mesh_4.html)を利用する
## 5. 計測中に領域は不変であり、メッシュもまた不変であると仮定

#######################################

import std/[math, sequtils, random, os]
import arraymancer, plotly, results, chroma

#######################################

type  
  InnerNode = object
    ## 2D-EITを対象とするため2次元を仮定
    ## 領域内部のノード(ポテンシャルエネルギー最小化の偏微分計算から立式)
    X: float
    Y: float
    V: float
  OuterNode = object
    ## 2D-EITを対象とするため2次元を仮定
    ## 領域外部ノード(ノイマン・ロビン境界条件から立式)
    ## 電極は外部ノードの一つとして表現
    X: float
    Y: float
    V: float
    isElectrode: bool
    inI: float
    outV: float

  Node = InnerNode | OuterNode

  Mesh = object
    idNode1: int
    idNode2: int
    idNode3: int
    isNode1Outer: bool
    isNode2Outer: bool
    isNode3Outer: bool

  System = object
    meshes: seq[Mesh]
    innerNodes: seq[InnerNode]
    outerNodes: seq[OuterNode]

#######################################

func generate_electrodes(num_sensors: int, R: float): Result[seq[Outernode], CatchableError] = 
  ## 円形状で一定間隔で電極を形成
  if num_sensors <= 1:
    return CatchableError(msg: "num_sensors must be over 1").err()

  var electrodes: seq[OuterNode]

  for i in 0..<num_sensors:
    let x = R*cos((i/num_sensors)*2*PI)
    let y = R*sin((i/num_sensors)*2*PI)
    let node = OuterNode(X: x, Y : y, V : 0.0, isElectrode : true, inI : 0.0, outV : 0.0)
    electrodes.add(node)

  return electrodes.ok()

proc draw_electrodes(electrodes: seq[OuterNode]) = 
  ## 電極を描画
  var
    d = Trace[float](mode: PlotMode.Markers, `type`: PlotType.Scatter)
    size = @[16.float]
  d.marker =Marker[float](size:size)
  
  var xs: seq[float]
  var ys: seq[float]

  for item in electrodes.items():
    xs.add(item.X)
    ys.add(item.Y)

  d.xs = xs
  d.ys = ys
  #d.text = @["hello", "data-point", "third", "highest", "<b>bold</b>"]

  var
    layout = Layout(title: "testing", width: 600, height: 600,
                      xaxis: Axis(title:"x"),
                      yaxis:Axis(title: "y"), autosize:false)
    p = Plot[float](layout:layout, traces: @[d])

  p.show()

func generate_system(electrodes: seq[OuterNode]): Result[System, CatchableError] =
  ## まず電極として設定した、円周上の各点から円の中心に向けてエッジを描く
  ## これで最初のメッシュを作り、ノードと合わせてシステムオブジェクトを生成する
  var
    meshes: seq[Mesh]
    outer_nodes: seq[OuterNode]
    inner_nodes: seq[InnerNode]
  for item in electrodes.items():
    outer_nodes.add(item)
  inner_nodes.add(InnerNode(X: 0.0, Y: 0.0, V: 0.0))

  for i in 0..<len(outer_nodes):
    meshes.add(Mesh(idNode1: 0, idNode2: i mod len(outer_nodes), idNode3: (i+1) mod len(outer_nodes), isNode1Outer: false, isNode2Outer: true, isNode3Outer: true))

  return System(meshes: meshes, outerNodes: outer_nodes, innerNodes: inner_nodes).ok()

func dist(pt1:(float, float), pt2: (float, float)): float =
  return sqrt((pt1[0] - pt2[0])^2 + (pt1[1] - pt2[1])^2)

func normalize_vec(pt: (float, float)): (float, float) =
  return (pt[0]/sqrt(pt[0]^2 + pt[1]^2), pt[1]/sqrt(pt[0]^2 + pt[1]^2))

func calc_rad_from_nvec(nvec: (float, float)): float =
  ## -π to π (rad = 0: Xaxis)
  var
    sign = nvec[1] >= 0
    theta = arccos(nvec[0])
  if not sign:
    theta *= -1
  return theta

func add_new_node_to_system(system: var System, newNode: InnerNode) =
  ## delauney法に基づく逐次メッシュ生成
  ## 取り扱うノードは内部ノードのみを対象とする
  ## このプログラムはiterateされ、新ノードは外側から与えられることを想定
  ## 新ノードは必ず末尾に加えられると仮定
  ## 1. メッシュ毎に外接円の中心を算出し、更新対象に入るか(= 新ノードを外接円内に含むか)を確認
  ## 2. 更新対象のメッシュをシステムから削除
  ## 3. 更新対象のメッシュについてそれらが保有するノード番号をリストに入れる
  ## 4. 対象のn個のノードに対して新しいノードからエッジを引く形でn個の新しいメッシュを生成
  
  var meshesUpdated: seq[(int, Mesh)]
  let numNewNode = len(system.innerNodes)

  # 1. メッシュ毎に外接円の中心を算出し、更新対象に入るか(= 新ノードを外接円内に含むか)を確認
  for i, mesh in system.meshes.pairs():
    var
      newNodeXY = (newNode.X, newNode.Y)
      node1XY: (float, float)
      node2XY: (float, float)
      node3XY: (float, float)
    if mesh.isNode1Outer:
      var node1 = system.outerNodes[mesh.idNode1]
      node1XY = (node1.X, node1.Y)
    else:
      var node1 = system.innerNodes[mesh.idNode1]
      node1XY = (node1.X, node1.Y)
    if mesh.isNode2Outer:
      var node2 = system.outerNodes[mesh.idNode2]
      node2XY = (node2.X, node2.Y)
    else:
      var node2 = system.innerNodes[mesh.idNode2]
      node2XY = (node2.X, node2.Y)
    if mesh.isNode3Outer:
      var node3 = system.outerNodes[mesh.idNode3]
      node3XY = (node3.X, node3.Y)
    else:
      var node3 = system.innerNodes[mesh.idNode3]
      node3XY = (node3.X, node3.Y)

    let
      # 外心の求め方(https://w3e.kanazawa-it.ac.jp/math/category/kika/heimenkika/henkan-tex.cgi?target=/math/category/kika/heimenkika/gaisinn_motomekata.html)
      x = (1/2)*(((node1XY[0]^2+node1XY[1]^2)*(node2XY[1]-node3XY[1])+(node2XY[0]^2+node2XY[1]^2)*(node3XY[1]-node1XY[1])+(node3XY[0]^2+node3XY[1]^2)*(node1XY[1]-node2XY[1]))/((node1XY[0]-node2XY[0])*(node2XY[1]-node3XY[1])-(node2XY[0]-node3XY[0])*(node1XY[1]-node2XY[1])))
      y = (1/2)*(((node1XY[0]^2+node1XY[1]^2)*(node2XY[0]-node3XY[0])+(node2XY[0]^2+node2XY[1]^2)*(node3XY[0]-node1XY[0])+(node3XY[0]^2+node3XY[1]^2)*(node1XY[0]-node2XY[0]))/((node2XY[0]-node3XY[0])*(node1XY[1]-node2XY[1])-(node1XY[0]-node2XY[0])*(node2XY[1]-node3XY[1])))
      centerXY = (x, y)

    if dist(centerXY, newNodeXY) < dist(centerXY, node1XY):
      meshesUpdated.add((i, mesh))
  
  # 2. 更新対象のメッシュをシステムから削除
  var deletedNum = 0
  for (i, mesh) in meshesUpdated.items():
    system.meshes.delete(i-deletedNum)
    deletedNum += 1
  
  # 3. 更新対象のメッシュについてそれらが保有するノード番号をリストに入れる
  var
    numOuterNodesUpdated: seq[int]
    numInnerNodesUpdated: seq[int]

  for (i, mesh) in meshesUpdated.items():
    if mesh.isNode1Outer and not numOuterNodesUpdated.anyIt(it == mesh.idNode1):
      numOuterNodesUpdated.add(mesh.idNode1)
    if not mesh.isNode1Outer and not numInnerNodesUpdated.anyIt(it == mesh.idNode1):
      numInnerNodesUpdated.add(mesh.idNode1)
    if mesh.isNode2Outer and not numOuterNodesUpdated.anyIt(it == mesh.idNode2):
      numOuterNodesUpdated.add(mesh.idNode2)
    if not mesh.isNode2Outer and not numInnerNodesUpdated.anyIt(it == mesh.idNode2):
      numInnerNodesUpdated.add(mesh.idNode2)
    if mesh.isNode3Outer and not numOuterNodesUpdated.anyIt(it == mesh.idNode3):
      numOuterNodesUpdated.add(mesh.idNode3)
    if not mesh.isNode3Outer and not numInnerNodesUpdated.anyIt(it == mesh.idNode3):
      numInnerNodesUpdated.add(mesh.idNode3)
 
  # 4. 対象のn個のノードに対して新しいノードからエッジを引く形でn個の新しいメッシュを生成

  # ここ面倒くさいな、新ノードと更新ノードを結ぶのは良いけど、
  # どのノードが三つ組を作ってメッシュになるのかの指定はできない
  # 単純に新ノードを中心とした時の角度を算出してソートしておくしか無いか
  var
    nodesUpdated: seq[(bool, int, float)] # true: OuterNode, false: InnerNode
  
  # 角度を算出して格納
  for i in numOuterNodesUpdated.items():
    var
      vec = (system.outerNodes[i].X - newNode.X, system.outerNodes[i].Y - newNode.Y)
      nvec = normalize_vec(vec)
      rad = calc_rad_from_nvec(nvec)
      tmp = (true, i, rad) # true: OuterNode, false: InnerNode
    nodesUpdated.add(tmp)

  for i in numInnerNodesUpdated.items():
    var
      vec = (system.innerNodes[i].X - newNode.X, system.innerNodes[i].Y - newNode.Y)
      nvec = normalize_vec(vec)
      rad = calc_rad_from_nvec(nvec)
      tmp = (false, i, rad) # true: OuterNode, false: InnerNode
    nodesUpdated.add(tmp)

  # 昇順ソート
  var
    probe = 0
    nodesUpdatedSorted: seq[(bool, int, float)] # true: OuterNode, false: InnerNode

  while probe < len(nodesUpdated):
    if probe == 0:
      nodesUpdatedSorted.add(nodesUpdated[probe])
      probe += 1
    else:
      for i in 0..<len(nodesUpdatedSorted):
        if nodesUpdated[probe][2] > nodesUpdatedSorted[i][2]:
          nodesUpdatedSorted.insert(nodesUpdated[probe], i)
          probe += 1
          break
        elif i == len(nodesUpdatedSorted)-1:
          nodesUpdatedSorted.add(nodesUpdated[probe])
          probe += 1
          break

  # ソート済みのノード番号を用いてaddしていく
  for i in 0..<len(nodesUpdatedSorted):
    var
      isNode2Outer = nodesUpdatedSorted[i mod len(nodesUpdatedSorted)][0]
      isNode3Outer = nodesUpdatedSorted[(i+1) mod len(nodesUpdatedSorted)][0]
    system.meshes.add(Mesh(
      idNode1: numNewNode, idNode2: nodesUpdatedSorted[i mod len(nodesUpdatedSorted)][1], idNode3: nodesUpdatedSorted[(i+1) mod len(nodesUpdatedSorted)][1], 
      isNode1Outer: false, isNode2Outer: isNode2Outer, isNode3Outer: isNode3Outer
      ))

  # ダブリが生じないように最後にnewNodeを加える
  system.innerNodes.add(newNode)

proc draw_system(system: System) = 
  ## メッシュのエッジを描画
  # const colors = @[Color(r: 0.0, g: 0.0, b:0.0, a: 0.4)]
  let
    layout = Layout(title: "mesh", width: 600, height: 600,
                      xaxis: Axis(title:"x"),
                      yaxis: Axis(title: "y"), 
                      autosize:false)
  var
    p = Plot[float](layout:layout)

  for mesh in system.meshes.items():
    let
      d = Trace[float](mode: PlotMode.Lines, `type`: PlotType.Scatter)
      size = @[16.float]
    d.marker = Marker[float](size: size)
    var
      xs: seq[float]
      ys: seq[float]
      x1: float
      y1: float
      x2: float
      y2: float
      x3: float
      y3: float
    echo "------"
    echo mesh.isNode1Outer
    echo mesh.idNode1
    echo mesh.isNode2Outer
    echo mesh.idNode2
    echo mesh.isNode3Outer
    echo mesh.idNode3
    if mesh.isNode1Outer:
      x1 = system.outerNodes[mesh.idNode1].X
      y1 = system.outerNodes[mesh.idNode1].Y
    else:
      x1 = system.innerNodes[mesh.idNode1].X
      y1 = system.innerNodes[mesh.idNode1].Y
    if mesh.isNode2Outer:
      x2 = system.outerNodes[mesh.idNode2].X
      y2 = system.outerNodes[mesh.idNode2].Y
    else:
      x2 = system.innerNodes[mesh.idNode2].X
      y2 = system.innerNodes[mesh.idNode2].Y
    if mesh.isNode3Outer:
      x3 = system.outerNodes[mesh.idNode3].X
      y3 = system.outerNodes[mesh.idNode3].Y
    else:
      x3 = system.innerNodes[mesh.idNode3].X
      y3 = system.innerNodes[mesh.idNode3].Y
    xs.add(x1)
    xs.add(x2)
    xs.add(x3)
    xs.add(x1)
    ys.add(y1)
    ys.add(y2)
    ys.add(y3)
    ys.add(y1)

    d.xs = xs
    d.ys = ys
    
    p = p.addTrace(d)

  p.show()


#######################################

const
  numElectrodes = 16
  diameter = 10.0
  gap = 1.0 # delauney法は、新しいノードを追加する際に必ず既存のメッシュで構築される領域内部に点が追加されなければならない
  num_inner_nodes = 101


#######################################

var electrodes = generate_electrodes(numElectrodes, diameter).value()
draw_electrodes(electrodes)
var system = generate_system(electrodes).value()

randomize()

for i in 0..<(num_inner_nodes-1):
  var newNodeXY: (float, float)
  while true:
    var
      x = rand(diameter*2) - diameter
      y = rand(diameter*2) - diameter
    if x^2 + y^2 < (diameter-gap)^2:
      newNodeXY = (x, y)
      break

  var newNode = InnerNode(X: newNodeXY[0], Y: newNodeXY[1], V: 0.0)
  echo newNode

  add_new_node_to_system(system, newNode)

draw_system(system)