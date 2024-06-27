## 1. 前半は静電場FEM、後半は差分再構成法を使用したEIT
## 2. 領域外形は円形を仮定
## 3. センサ位置は領域外形を等分したものと仮定
## 4.メッシュ分割アルゴリズムはdelauney法(https://ryutai.ninja-web.net/mesh/mesh_4.html)を利用する

#######################################

import std/math
import arraymancer, plotly, results

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

func generate_primitive_meshes(electrodes: seq[OuterNode]): Result[(seq[Mesh], seq[OuterNode], seq[Innernode]), CatchableError] =
  ## まず電極として設定した、円周上の各点から円の中心に向けてエッジを描く
  ## これで最初のメッシュを作る
  var
    meshes: seq[Mesh]
    outer_nodes: seq[OuterNode]
    inner_nodes: seq[InnerNode]
  for item in electrodes.items():
    outer_nodes.add(item)
  inner_nodes.add(InnerNode(X: 0.0, Y: 0.0, V: 0.0))

  for i in 0..<len(outer_nodes):
    meshes.add(
      Mesh(idNode1: 0, idNode2: i, idNode3: i+1, isNode1Outer: true, isNode2Outer: false, isNode3Outer: false))

  return (meshes, outer_nodes, inner_nodes).ok()

#######################################

const numElectrodes = 16
const diameter = 10.0

#######################################

var electrodes = generate_electrodes(numElectrodes, diameter).value()
echo electrodes
draw_electrodes(electrodes)

# 