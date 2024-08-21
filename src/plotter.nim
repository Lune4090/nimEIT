import sequtils, math
import plotly, chroma
import mesh, calc

proc draw_vertices*(mesh: Mesh) = 
  ## メッシュの頂点を描画
  # const colors = @[Color(r: 0.0, g: 0.0, b:0.0, a: 0.0)]
  let
    layout = Layout(title: "vertices", width: 600, height: 600,
                      xaxis: Axis(title: "x"),
                      yaxis: Axis(title: "y"), 
                      autosize:false)
    d = Trace[float](mode: PlotMode.Markers, `type`: PlotType.Scatter)
    size = @[16.float]
  d.marker = Marker[float](size: size)
  
  var
    p = Plot[float](layout:layout)
    xs: seq[float]
    ys: seq[float]

  for vert in mesh.vertices.items():
    xs.add(vert.pos[0])
    ys.add(vert.pos[1])

  d.xs = xs
  d.ys = ys
    
  p = p.addTrace(d)
  p.show()

proc draw_mesh*(mesh: Mesh) = 
  ## エレメントを描画
  const colors = @[Color(r: 0.0, g: 0.0, b:0.0, a: 0.0)]
  let
    layout = Layout(title: "mesh", width: 600, height: 600,
                      xaxis: Axis(title: "x"),
                      yaxis: Axis(title: "y"), 
                      autosize:false)
  var
    p = Plot[float](layout:layout)

  for element in mesh.elements.items():
    let
      d = Trace[float](mode: PlotMode.Lines, `type`: PlotType.Scatter)
      size = @[16.float]
    d.marker = Marker[float](size: size, color: colors)
    
    var
      xs: seq[float]
      ys: seq[float]
      x1 = mesh.vertices[element.idxVertice1].pos[0]
      y1 = mesh.vertices[element.idxVertice1].pos[1]
      x2 = mesh.vertices[element.idxVertice2].pos[0]
      y2 = mesh.vertices[element.idxVertice2].pos[1]
      x3 = mesh.vertices[element.idxVertice3].pos[0]
      y3 = mesh.vertices[element.idxVertice3].pos[1]

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

proc draw_V*(mesh: Mesh, resolution: (int, int), drawingArea: ((float, float), (float, float))) =
  ## 1. 3つの頂点から1組のベクトルを作って外積計算、法線ベクトルを算出し電位方程式を導出
  ## 2. 対象のエレメント中にheatmapの描画対象のある点(x, y)が含まれるかを判定
    ## AABBで捜索範囲を制限
    ## 制限された範囲の中の各ピクセル座標について、エレメントの頂点から任意の2組のベクトルを選択
    ## 選択されたベクトルとピクセル座標から内積を通じて角度を導出し、領域内にあるかどうかを確認
  ## 3. 描画対象に含まれる場合、座標を電位方程式に代入して電位を算出し格納
  
  # const colors = @[Color(r: 0.0, g: 0.0, b:0.0, a: 0.4)]
  let
    layout = Layout(title: "voltages", width: 600, height: 600,
                      xaxis: Axis(title: "x"),
                      yaxis: Axis(title: "y"), 
                      autosize: false)
    # 計算精度の都合上と思われる電位計算時の導出ヌケを防ぐために意図的に小さな値を加算
    epsilon = 1e-8
  
  var
    xs = newSeq[float](resolution[0])
    ys = newSeq[float](resolution[1])
    colors: seq[Color]
    d = Trace[float](mode: PlotMode.Lines, `type`: PlotType.HeatMap)
    size = @[16.float]
  
  d.zs = newSeqWith(resolution[0], newSeq[float](resolution[1]))
  
  # ヒートマップのマス目の座標の定義(左下を基準としている)
  for i in 0..<resolution[0]:
    xs[i] = drawingArea[0][0] + i.toFloat*(drawingArea[1][0] - drawingArea[0][0])/resolution[0].toFloat
  for i in 0..<resolution[1]:
    ys[i] = drawingArea[0][1] + i.toFloat*(drawingArea[1][1] - drawingArea[0][1])/resolution[1].toFloat
  
  # 電位方程式の計算
  for elem in mesh.elements.items():
    let
      vert1 = mesh.vertices[elem.idxVertice1]
      vert2 = mesh.vertices[elem.idxVertice2]
      vert3 = mesh.vertices[elem.idxVertice3]
    
    # 外積計算をするベクトルの決定
    var
      vecA: (float, float, float)
      vecB: (float, float, float)
      vecC: (float, float, float)
      vecA_start: Vertice2D
      vecA_end: Vertice2D

    # 最もx座標が大きいノードを外積計算の2ベクトルの始点に指定
    if vert1.pos[0] > vert2.pos[0]:
      if vert1.pos[0] > vert3.pos[0]:
        vecA = (vert2.pos[0] - vert1.pos[0], vert2.pos[1] - vert1.pos[1], vert2.V - vert1.V)
        vecB = (vert3.pos[0] - vert1.pos[0], vert3.pos[1] - vert1.pos[1], vert3.V - vert1.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert1
        vecA_end = vert2
      else:
        vecA = (vert1.pos[0] - vert3.pos[0], vert1.pos[1] - vert3.pos[1], vert1.V - vert3.V)
        vecB = (vert2.pos[0] - vert3.pos[0], vert2.pos[1] - vert3.pos[1], vert2.V - vert3.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert3
        vecA_end = vert1
    else:
      if vert2.pos[0] > vert3.pos[0]:
        vecA = (vert3.pos[0] - vert2.pos[0], vert3.pos[1] - vert2.pos[1], vert3.V - vert2.V)
        vecB = (vert1.pos[0] - vert2.pos[0], vert1.pos[1] - vert2.pos[1], vert1.V - vert2.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert2
        vecA_end = vert3
      else:
        vecA = (vert1.pos[0] - vert3.pos[0], vert1.pos[1] - vert3.pos[1], vert1.V - vert3.V)
        vecB = (vert2.pos[0] - vert3.pos[0], vert2.pos[1] - vert3.pos[1], vert2.V - vert3.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert3
        vecA_end = vert1
    
    # 法線ベクトル(一次近似の電位方程式の係数ベクトル)算出(向きは考慮しない)
    let normVec = cross(vecA, vecB)

    # 捜索範囲としてのAABB(Axis-Aligned Bounding Box)の定義
    let
      AABB0 = (min(min(vert1.pos[0], vert2.pos[0]), vert3.pos[0]), min(min(vert1.pos[1], vert2.pos[1]), vert3.pos[1]))
      AABB1 = (max(max(vert1.pos[0], vert2.pos[0]), vert3.pos[0]), max(max(vert1.pos[1], vert2.pos[1]), vert3.pos[1]))
    
    # AABBのピクセル値取得
    let
      AABB0pixelX = int(resolution[0].toFloat*(AABB0[0] - drawingArea[0][0])/(drawingArea[1][0] - drawingArea[0][0]))
      AABB0pixelY = int(resolution[1].toFloat*(AABB0[1] - drawingArea[0][1])/(drawingArea[1][1] - drawingArea[0][1]))
      AABB1pixelX = int(resolution[0].toFloat*(AABB1[0] - drawingArea[0][0])/(drawingArea[1][0] - drawingArea[0][0]))
      AABB1pixelY = int(resolution[1].toFloat*(AABB1[1] - drawingArea[0][1])/(drawingArea[1][1] - drawingArea[0][1]))
      
    # 電位描画
    for pixelX in AABB0pixelX..min(AABB1pixelX, resolution[0]-1):
      for pixelY in AABB0pixelY..min(AABB1pixelY, resolution[1]-1):
        # 内積の大小から角度の大小関係を確認、エレメント内に対象の座標が居るかを確認
        let pos = (xs[pixelX], ys[pixelY])
        var
          posA: (float, float)
          posB: (float, float)

        if vecA_start == vert1:
          posA = (xs[pixelX] - vert1.pos[0], ys[pixelY] - vert1.pos[1])
        if vecA_start == vert2:
          posA = (xs[pixelX] - vert2.pos[0], ys[pixelY] - vert2.pos[1])
        if vecA_start == vert3:
          posA = (xs[pixelX] - vert3.pos[0], ys[pixelY] - vert3.pos[1])
        
        if vecA_end == vert1:
          posB = (xs[pixelX] - vert1.pos[0], ys[pixelY] - vert1.pos[1])
        if vecA_end == vert2:
          posB = (xs[pixelX] - vert2.pos[0], ys[pixelY] - vert2.pos[1])
        if vecA_end == vert3:
          posB = (xs[pixelX] - vert3.pos[0], ys[pixelY] - vert3.pos[1])

        if epsilon + dot(posA, (vecA[0], vecA[1]))/(norm(posA)*norm((vecA[0], vecA[1]))) >= dot((vecB[0], vecB[1]), (vecA[0], vecA[1]))/(norm((vecB[0], vecB[1]))*norm((vecA[0], vecA[1]))) and epsilon + dot(posA, (vecB[0], vecB[1]))/(norm(posA)*norm((vecB[0], vecB[1]))) >= dot((vecB[0], vecB[1]), (vecA[0], vecA[1]))/(norm((vecB[0], vecB[1]))*norm((vecA[0], vecA[1]))) and epsilon + dot(posB, (vecC[0], vecC[1]))/(norm(posB)*norm((vecC[0], vecC[1]))) >= dot((vecC[0], vecC[1]), (vecA.reversed()[0], vecA.reversed()[1]))/(norm((vecC[0], vecC[1]))*norm((vecA.reversed()[0], vecA.reversed()[1]))) and epsilon + dot(posB, (vecA.reversed()[0], vecA.reversed()[1]))/(norm(posB)*norm((vecA.reversed()[0], vecA.reversed()[1]))) >= dot((vecC[0], vecC[1]), (vecA.reversed()[0], vecA.reversed()[1]))/(norm((vecC[0], vecC[1]))*norm((vecA.reversed()[0], vecA.reversed()[1]))):
          d.zs[pixelX][pixelY] = (-normVec[0]/normVec[2])*(pos[0]-vert1.pos[0]) + (-normVec[1]/normVec[2])*(pos[1]-vert1.pos[1]) + vert1.V

  # 描画領域外をマスク
  for pixelX in 0..<resolution[0]:
    for pixelY in 0..<resolution[1]-1:
      if xs[pixelX]^2 + ys[pixelY]^2 >= drawingArea[0][0]^2:
        d.zs[pixelX][pixelY] = Inf

  d.marker = Marker[float](size: size, color: colors)
  var p = Plot[float](layout: layout)
  p = p.addTrace(d)
  #p.show(filename = "helloworld.png")
  p.saveImage("DeltaV.svg")
  # NOTE: if we compile this without --threads:on support, we'll get
  # an error at compile time that thread support is needed.

proc draw_Δσ*(mesh: Mesh, resolution: (int, int), drawingArea: ((float, float), (float, float)), title = "Δσ(actual conductivities change)") = 
  ## 1. 3つの頂点から1組のベクトルを作って外積計算、法線ベクトルを算出し電位方程式を導出
  ## 2. 対象のエレメント中にheatmapの描画対象のある点(x, y)が含まれるかを判定
    ## AABBで捜索範囲を制限
    ## 制限された範囲の中の各ピクセル座標について、エレメントの頂点から任意の2組のベクトルを選択
    ## 選択されたベクトルとピクセル座標から内積を通じて角度を導出し、領域内にあるかどうかを確認
  ## 3. 描画対象に含まれる場合、座標を電位方程式に代入して電位を算出し格納
  
  let
    layout = Layout(title: title, width: 600, height: 600,
                      xaxis: Axis(title: "x"),
                      yaxis: Axis(title: "y"), 
                      autosize:false)
    # 計算精度の都合上と思われる電位計算時の導出ヌケを防ぐために意図的に小さな値を加算
    epsilon = 1e-8
  
  var
    xs = newSeq[float](resolution[0])
    ys = newSeq[float](resolution[1])
    colors: seq[Color]
    d = Trace[float](mode: PlotMode.Lines, `type`: PlotType.HeatMap)
    size = @[16.float]
  
  d.zs = newSeqWith(resolution[0], newSeq[float](resolution[1]))
  
  # ヒートマップのマス目の座標の定義(左下を基準としている)
  for i in 0..<resolution[0]:
    xs[i] = drawingArea[0][0] + i.toFloat*(drawingArea[1][0] - drawingArea[0][0])/resolution[0].toFloat
  for i in 0..<resolution[1]:
    ys[i] = drawingArea[0][1] + i.toFloat*(drawingArea[1][1] - drawingArea[0][1])/resolution[1].toFloat
  
  # 電位方程式の計算
  for elem in mesh.elements.items():
    let
      vert1 = mesh.vertices[elem.idxVertice1]
      vert2 = mesh.vertices[elem.idxVertice2]
      vert3 = mesh.vertices[elem.idxVertice3]
    
    # 外積計算をするベクトルの決定
    var
      vecA: (float, float, float)
      vecB: (float, float, float)
      vecC: (float, float, float)
      vecA_start: Vertice2D
      vecA_end: Vertice2D

    # 最もx座標が大きいノードを外積計算の2ベクトルの始点に指定
    if vert1.pos[0] > vert2.pos[0]:
      if vert1.pos[0] > vert3.pos[0]:
        vecA = (vert2.pos[0] - vert1.pos[0], vert2.pos[1] - vert1.pos[1], vert2.V - vert1.V)
        vecB = (vert3.pos[0] - vert1.pos[0], vert3.pos[1] - vert1.pos[1], vert3.V - vert1.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert1
        vecA_end = vert2
      else:
        vecA = (vert1.pos[0] - vert3.pos[0], vert1.pos[1] - vert3.pos[1], vert1.V - vert3.V)
        vecB = (vert2.pos[0] - vert3.pos[0], vert2.pos[1] - vert3.pos[1], vert2.V - vert3.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert3
        vecA_end = vert1
    else:
      if vert2.pos[0] > vert3.pos[0]:
        vecA = (vert3.pos[0] - vert2.pos[0], vert3.pos[1] - vert2.pos[1], vert3.V - vert2.V)
        vecB = (vert1.pos[0] - vert2.pos[0], vert1.pos[1] - vert2.pos[1], vert1.V - vert2.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert2
        vecA_end = vert3
      else:
        vecA = (vert1.pos[0] - vert3.pos[0], vert1.pos[1] - vert3.pos[1], vert1.V - vert3.V)
        vecB = (vert2.pos[0] - vert3.pos[0], vert2.pos[1] - vert3.pos[1], vert2.V - vert3.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert3
        vecA_end = vert1
    
    # 捜索範囲としてのAABB(Axis-Aligned Bounding Box)の定義
    let
      AABB0 = (min(min(vert1.pos[0], vert2.pos[0]), vert3.pos[0]), min(min(vert1.pos[1], vert2.pos[1]), vert3.pos[1]))
      AABB1 = (max(max(vert1.pos[0], vert2.pos[0]), vert3.pos[0]), max(max(vert1.pos[1], vert2.pos[1]), vert3.pos[1]))
    
    # AABBのピクセル値取得
    let
      AABB0pixelX = int(resolution[0].toFloat*(AABB0[0] - drawingArea[0][0])/(drawingArea[1][0] - drawingArea[0][0]))
      AABB0pixelY = int(resolution[1].toFloat*(AABB0[1] - drawingArea[0][1])/(drawingArea[1][1] - drawingArea[0][1]))
      AABB1pixelX = int(resolution[0].toFloat*(AABB1[0] - drawingArea[0][0])/(drawingArea[1][0] - drawingArea[0][0]))
      AABB1pixelY = int(resolution[1].toFloat*(AABB1[1] - drawingArea[0][1])/(drawingArea[1][1] - drawingArea[0][1]))
      
    # 電位描画

    for pixelX in AABB0pixelX..min(AABB1pixelX, resolution[0]-1):
      for pixelY in AABB0pixelY..min(AABB1pixelY, resolution[1]-1):
        # 内積の大小から角度の大小関係を確認、エレメント内に対象の座標が居るかを確認
        var
          posA: (float, float)
          posB: (float, float)

        if vecA_start == vert1:
          posA = (xs[pixelX] - vert1.pos[0], ys[pixelY] - vert1.pos[1])
        if vecA_start == vert2:
          posA = (xs[pixelX] - vert2.pos[0], ys[pixelY] - vert2.pos[1])
        if vecA_start == vert3:
          posA = (xs[pixelX] - vert3.pos[0], ys[pixelY] - vert3.pos[1])
        
        if vecA_end == vert1:
          posB = (xs[pixelX] - vert1.pos[0], ys[pixelY] - vert1.pos[1])
        if vecA_end == vert2:
          posB = (xs[pixelX] - vert2.pos[0], ys[pixelY] - vert2.pos[1])
        if vecA_end == vert3:
          posB = (xs[pixelX] - vert3.pos[0], ys[pixelY] - vert3.pos[1])

        if epsilon + dot(posA, (vecA[0], vecA[1]))/(norm(posA)*norm((vecA[0], vecA[1]))) >= dot((vecB[0], vecB[1]), (vecA[0], vecA[1]))/(norm((vecB[0], vecB[1]))*norm((vecA[0], vecA[1]))) and epsilon + dot(posA, (vecB[0], vecB[1]))/(norm(posA)*norm((vecB[0], vecB[1]))) >= dot((vecB[0], vecB[1]), (vecA[0], vecA[1]))/(norm((vecB[0], vecB[1]))*norm((vecA[0], vecA[1]))) and epsilon + dot(posB, (vecC[0], vecC[1]))/(norm(posB)*norm((vecC[0], vecC[1]))) >= dot((vecC[0], vecC[1]), (vecA.reversed()[0], vecA.reversed()[1]))/(norm((vecC[0], vecC[1]))*norm((vecA.reversed()[0], vecA.reversed()[1]))) and epsilon + dot(posB, (vecA.reversed()[0], vecA.reversed()[1]))/(norm(posB)*norm((vecA.reversed()[0], vecA.reversed()[1]))) >= dot((vecC[0], vecC[1]), (vecA.reversed()[0], vecA.reversed()[1]))/(norm((vecC[0], vecC[1]))*norm((vecA.reversed()[0], vecA.reversed()[1]))):
          d.zs[pixelX][pixelY] = elem.Δσ

  # 描画領域外をマスク
  for pixelX in 0..<resolution[0]:
    for pixelY in 0..<resolution[1]-1:
      if xs[pixelX]^2 + ys[pixelY]^2 >= drawingArea[0][0]^2:
        d.zs[pixelX][pixelY] = Inf

  d.marker = Marker[float](size: size, color: colors)
  var p = Plot[float](layout: layout)
  p = p.addTrace(d)
  #p.show(filename = "helloworld.png")
  p.saveImage("DeltaSigma.svg")
  # NOTE: if we compile this without --threads:on support, we'll get
  # an error at compile time that thread support is needed.
  
proc draw_δσ*(mesh: Mesh, resolution: (int, int), drawingArea: ((float, float), (float, float)), title = "δσ(estimated conductivities change)") = 
  ## 1. 3つの頂点から1組のベクトルを作って外積計算、法線ベクトルを算出し電位方程式を導出
  ## 2. 対象のエレメント中にheatmapの描画対象のある点(x, y)が含まれるかを判定
    ## AABBで捜索範囲を制限
    ## 制限された範囲の中の各ピクセル座標について、エレメントの頂点から任意の2組のベクトルを選択
    ## 選択されたベクトルとピクセル座標から内積を通じて角度を導出し、領域内にあるかどうかを確認
  ## 3. 描画対象に含まれる場合、座標を電位方程式に代入して電位を算出し格納
  
  let
    layout = Layout(title: title, width: 600, height: 600,
                      xaxis: Axis(title: "x"),
                      yaxis: Axis(title: "y"), 
                      autosize:false)
    # 計算精度の都合上と思われる電位計算時の導出ヌケを防ぐために意図的に小さな値を加算
    epsilon = 1e-8
  
  var
    xs = newSeq[float](resolution[0])
    ys = newSeq[float](resolution[1])
    colors: seq[Color]
    d = Trace[float](mode: PlotMode.Lines, `type`: PlotType.HeatMap)
    size = @[16.float]
  
  d.zs = newSeqWith(resolution[0], newSeq[float](resolution[1]))
  
  # ヒートマップのマス目の座標の定義(左下を基準としている)
  for i in 0..<resolution[0]:
    xs[i] = drawingArea[0][0] + i.toFloat*(drawingArea[1][0] - drawingArea[0][0])/resolution[0].toFloat
  for i in 0..<resolution[1]:
    ys[i] = drawingArea[0][1] + i.toFloat*(drawingArea[1][1] - drawingArea[0][1])/resolution[1].toFloat
  
  # 電位方程式の計算
  for elem in mesh.elements.items():
    let
      vert1 = mesh.vertices[elem.idxVertice1]
      vert2 = mesh.vertices[elem.idxVertice2]
      vert3 = mesh.vertices[elem.idxVertice3]
    
    # 外積計算をするベクトルの決定
    var
      vecA: (float, float, float)
      vecB: (float, float, float)
      vecC: (float, float, float)
      vecA_start: Vertice2D
      vecA_end: Vertice2D

    # 最もx座標が大きいノードを外積計算の2ベクトルの始点に指定
    if vert1.pos[0] > vert2.pos[0]:
      if vert1.pos[0] > vert3.pos[0]:
        vecA = (vert2.pos[0] - vert1.pos[0], vert2.pos[1] - vert1.pos[1], vert2.V - vert1.V)
        vecB = (vert3.pos[0] - vert1.pos[0], vert3.pos[1] - vert1.pos[1], vert3.V - vert1.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert1
        vecA_end = vert2
      else:
        vecA = (vert1.pos[0] - vert3.pos[0], vert1.pos[1] - vert3.pos[1], vert1.V - vert3.V)
        vecB = (vert2.pos[0] - vert3.pos[0], vert2.pos[1] - vert3.pos[1], vert2.V - vert3.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert3
        vecA_end = vert1
    else:
      if vert2.pos[0] > vert3.pos[0]:
        vecA = (vert3.pos[0] - vert2.pos[0], vert3.pos[1] - vert2.pos[1], vert3.V - vert2.V)
        vecB = (vert1.pos[0] - vert2.pos[0], vert1.pos[1] - vert2.pos[1], vert1.V - vert2.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert2
        vecA_end = vert3
      else:
        vecA = (vert1.pos[0] - vert3.pos[0], vert1.pos[1] - vert3.pos[1], vert1.V - vert3.V)
        vecB = (vert2.pos[0] - vert3.pos[0], vert2.pos[1] - vert3.pos[1], vert2.V - vert3.V)
        vecC = (vecB[0] - vecA[0], vecB[1] - vecA[1], vecB[2] - vecA[2])
        vecA_start = vert3
        vecA_end = vert1
    
    # 法線ベクトル(一次近似の電位方程式の係数ベクトル)算出(向きは考慮しない)

    # 捜索範囲としてのAABB(Axis-Aligned Bounding Box)の定義
    let
      AABB0 = (min(min(vert1.pos[0], vert2.pos[0]), vert3.pos[0]), min(min(vert1.pos[1], vert2.pos[1]), vert3.pos[1]))
      AABB1 = (max(max(vert1.pos[0], vert2.pos[0]), vert3.pos[0]), max(max(vert1.pos[1], vert2.pos[1]), vert3.pos[1]))
    
    # AABBのピクセル値取得
    let
      AABB0pixelX = int(resolution[0].toFloat*(AABB0[0] - drawingArea[0][0])/(drawingArea[1][0] - drawingArea[0][0]))
      AABB0pixelY = int(resolution[1].toFloat*(AABB0[1] - drawingArea[0][1])/(drawingArea[1][1] - drawingArea[0][1]))
      AABB1pixelX = int(resolution[0].toFloat*(AABB1[0] - drawingArea[0][0])/(drawingArea[1][0] - drawingArea[0][0]))
      AABB1pixelY = int(resolution[1].toFloat*(AABB1[1] - drawingArea[0][1])/(drawingArea[1][1] - drawingArea[0][1]))
      
    # 電位描画

    for pixelX in AABB0pixelX..min(AABB1pixelX, resolution[0]-1):
      for pixelY in AABB0pixelY..min(AABB1pixelY, resolution[1]-1):
        # 内積の大小から角度の大小関係を確認、エレメント内に対象の座標が居るかを確認
        var
          posA: (float, float)
          posB: (float, float)

        if vecA_start == vert1:
          posA = (xs[pixelX] - vert1.pos[0], ys[pixelY] - vert1.pos[1])
        if vecA_start == vert2:
          posA = (xs[pixelX] - vert2.pos[0], ys[pixelY] - vert2.pos[1])
        if vecA_start == vert3:
          posA = (xs[pixelX] - vert3.pos[0], ys[pixelY] - vert3.pos[1])
        
        if vecA_end == vert1:
          posB = (xs[pixelX] - vert1.pos[0], ys[pixelY] - vert1.pos[1])
        if vecA_end == vert2:
          posB = (xs[pixelX] - vert2.pos[0], ys[pixelY] - vert2.pos[1])
        if vecA_end == vert3:
          posB = (xs[pixelX] - vert3.pos[0], ys[pixelY] - vert3.pos[1])

        if epsilon + dot(posA, (vecA[0], vecA[1]))/(norm(posA)*norm((vecA[0], vecA[1]))) >= dot((vecB[0], vecB[1]), (vecA[0], vecA[1]))/(norm((vecB[0], vecB[1]))*norm((vecA[0], vecA[1]))) and epsilon + dot(posA, (vecB[0], vecB[1]))/(norm(posA)*norm((vecB[0], vecB[1]))) >= dot((vecB[0], vecB[1]), (vecA[0], vecA[1]))/(norm((vecB[0], vecB[1]))*norm((vecA[0], vecA[1]))) and epsilon + dot(posB, (vecC[0], vecC[1]))/(norm(posB)*norm((vecC[0], vecC[1]))) >= dot((vecC[0], vecC[1]), (vecA.reversed()[0], vecA.reversed()[1]))/(norm((vecC[0], vecC[1]))*norm((vecA.reversed()[0], vecA.reversed()[1]))) and epsilon + dot(posB, (vecA.reversed()[0], vecA.reversed()[1]))/(norm(posB)*norm((vecA.reversed()[0], vecA.reversed()[1]))) >= dot((vecC[0], vecC[1]), (vecA.reversed()[0], vecA.reversed()[1]))/(norm((vecC[0], vecC[1]))*norm((vecA.reversed()[0], vecA.reversed()[1]))):
          d.zs[pixelX][pixelY] = elem.δσ

  # 描画領域外をマスク
  for pixelX in 0..<resolution[0]:
    for pixelY in 0..<resolution[1]-1:
      if xs[pixelX]^2 + ys[pixelY]^2 >= drawingArea[0][0]^2:
        d.zs[pixelX][pixelY] = Inf

  d.marker = Marker[float](size: size, color: colors)
  var p = Plot[float](layout: layout)
  p = p.addTrace(d)
  #p.show(filename = "helloworld.png")
  p.saveImage("DelSigma.svg")
  # NOTE: if we compile this without --threads:on support, we'll get
  # an error at compile time that thread support is needed.