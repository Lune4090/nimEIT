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
  var
    p = Plot[float](layout:layout)
    xs: seq[float]
    ys: seq[float]


  for element in mesh.vertices.items():
    
    d.marker = Marker[float](size: size)
    xs.add(element.pos[0])
    ys.add(element.pos[1])

  d.xs = xs
  d.ys = ys
    
  p = p.addTrace(d)
  p.show()

proc draw_mesh*(mesh: Mesh) = 
  ## メッシュのエッジを描画
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
  ## ノード毎の電位を描写
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
