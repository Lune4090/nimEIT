import plotly, chroma
import mesh

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

proc draw_V*(mesh: Mesh) =
  # TODO: heatmapに描写を切り替え
    # エレメント毎に電位についての平面方程式(V = f(x, y))を導出
    # 対象の領域にheatmapの描画対象のある点(x, y)が含まれるかは、
    # その座標を任意の2頂点を繋ぐ直線の方程式に代入した際の符号が残りの頂点を代入した時の符号に等しい
    # という条件を満たすかどうかから判断
  ## エレメント毎の電位を描写
  # const colors = @[Color(r: 0.0, g: 0.0, b:0.0, a: 0.4)]
  let
    layout = Layout(title: "voltages", width: 600, height: 600,
                      xaxis: Axis(title: "x"),
                      yaxis: Axis(title: "y"), 
                      autosize:false)
  var
    xs: seq[float]
    ys: seq[float]
    Vs: seq[float]
    Vmax: float
    Vmin: float
    colors: seq[Color]
    d = Trace[float](mode: PlotMode.Markers, `type`: PlotType.Scatter)
    size = @[16.float]
  
  for vert in mesh.vertices.items():
    xs.add(vert.pos[0])
    ys.add(vert.pos[1])
    Vs.add(vert.V)

  d.xs = xs
  d.ys = ys
  
  Vmin = Vs[0]
  Vmax = Vs[0]
  for (i, v) in Vs.pairs():
    if v < Vmin:
      Vmin = v
    if v > Vmax:
      Vmax = v

  for (i, v) in Vs.pairs():
    colors.add(Color(r: 0.1 + 0.9*((v-Vmin)/(Vmax-Vmin)), g: 1.0 - 0.9*((v-Vmin)/(Vmax-Vmin)), b: 0.0, a: 0.4))
    
  d.marker = Marker[float](size: size, color: colors)
  var p = Plot[float](layout: layout)
  p = p.addTrace(d)
  p.show()

proc draw_Δσ*(mesh: Mesh, title = "Δσ(actual conductivities change)") = 
  ## エレメントの中心を代表点として導電率変分を描画
  # const colors = @[Color(r: 0.0, g: 0.0, b:0.0, a: 0.0)]
  let
    layout = Layout(title: "Δσ(actual conductivities change)", width: 600, height: 600,
                      xaxis: Axis(title: "x"),
                      yaxis: Axis(title: "y"), 
                      autosize:false)
    d = Trace[float](mode: PlotMode.Markers, `type`: PlotType.Scatter)
    size = @[16.float]
  var
    p = Plot[float](layout:layout)
    xs: seq[float]
    ys: seq[float]
    Δσs: seq[float]

  for elem in mesh.elements.items():
    var
      p1 = mesh.vertices[elem.idxVertice1].pos
      p2 = mesh.vertices[elem.idxVertice2].pos
      p3 = mesh.vertices[elem.idxVertice3].pos
      p0 = ((p1[0]+p2[0]+p3[0])/3, (p1[1]+p2[1]+p3[1])/3)
    
    xs.add(p0[0])
    ys.add(p0[1])
    Δσs.add(elem.Δσ)

  d.xs = xs
  d.ys = ys

  var
    Δσmin = Δσs[0]
    Δσmax = Δσs[0]
    colors: seq[Color]

  for (i, Δσ) in Δσs.pairs():
    if Δσ < Δσmin:
      Δσmin = Δσ
    if Δσ > Δσmax:
      Δσmax = Δσ
  
  for (i, Δσ) in Δσs.pairs():
    colors.add(Color(r: 0.1 + 0.9*((Δσ-Δσmin)/(Δσmax-Δσmin)), g: 1.0 - 0.9*((Δσ-Δσmin)/(Δσmax-Δσmin)), b: 0.0, a: 0.4))

  d.marker = Marker[float](size: size, color: colors)
    
  p = p.addTrace(d)
  p.show()

proc draw_δσ*(mesh: Mesh, title = "δσ(estimated conductivities change)") = 
  ## エレメントの中心を代表点として導電率変分を描画
  # const colors = @[Color(r: 0.0, g: 0.0, b:0.0, a: 0.0)]
  let
    layout = Layout(title: "δσ(estimated conductivities change)", width: 600, height: 600,
                      xaxis: Axis(title: "x"),
                      yaxis: Axis(title: "y"), 
                      autosize:false)
    d = Trace[float](mode: PlotMode.Markers, `type`: PlotType.Scatter)
    size = @[16.float]
  var
    p = Plot[float](layout:layout)
    xs: seq[float]
    ys: seq[float]
    δσs: seq[float]

  for elem in mesh.elements.items():
    var
      p1 = mesh.vertices[elem.idxVertice1].pos
      p2 = mesh.vertices[elem.idxVertice2].pos
      p3 = mesh.vertices[elem.idxVertice3].pos
      p0 = ((p1[0]+p2[0]+p3[0])/3, (p1[1]+p2[1]+p3[1])/3)
    
    xs.add(p0[0])
    ys.add(p0[1])
    δσs.add(elem.δσ)

  d.xs = xs
  d.ys = ys

  var
    δσmin = δσs[0]
    δσmax = δσs[0]
    colors: seq[Color]

  for (i, δσ) in δσs.pairs():
    if δσ < δσmin:
      δσmin = δσ
    if δσ > δσmax:
      δσmax = δσ
  
  for (i, δσ) in δσs.pairs():
    colors.add(Color(r: 0.1 + 0.9*((δσ-δσmin)/(δσmax-δσmin)), g: 1.0 - 0.9*((δσ-δσmin)/(δσmax-δσmin)), b: 0.0, a: 0.4))

  d.marker = Marker[float](size: size, color: colors)
    
  p = p.addTrace(d)
  p.show()

