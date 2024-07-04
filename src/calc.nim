import std/[math]
import arraymancer, results

func dist_2D*(pt1:(float, float), pt2: (float, float)): float =
  return sqrt((pt1[0] - pt2[0])^2 + (pt1[1] - pt2[1])^2)

func circumcenter_2D*(p1:(float, float), p2: (float, float), p3: (float, float)): Result[(float, float), CatchableError] =
  ## 外心の求め方(https://w3e.kanazawa-it.ac.jp/math/category/kika/heimenkika/henkan-tex.cgi?target=/math/category/kika/heimenkika/gaisinn_motomekata.html)
  let
    x = (1/2)*(((p1[0]^2+p1[1]^2)*(p2[1]-p3[1])+(p2[0]^2+p2[1]^2)*(p3[1]-p1[1])+(p3[0]^2+p3[1]^2)*(p1[1]-p2[1]))/((p1[0]-p2[0])*(p2[1]-p3[1])-(p2[0]-p3[0])*(p1[1]-p2[1])))
    y = (1/2)*(((p1[0]^2+p1[1]^2)*(p2[0]-p3[0])+(p2[0]^2+p2[1]^2)*(p3[0]-p1[0])+(p3[0]^2+p3[1]^2)*(p1[0]-p2[0]))/((p2[0]-p3[0])*(p1[1]-p2[1])-(p1[0]-p2[0])*(p2[1]-p3[1])))
  
  return (x, y).ok()

func normalize_vec_2D*(pt: (float, float)): (float, float) =
  return (pt[0]/sqrt(pt[0]^2 + pt[1]^2), pt[1]/sqrt(pt[0]^2 + pt[1]^2))

func rad_nvec_2D*(nvec: (float, float)): float =
  ## -π to π (rad = 0 at Xaxis)
  var
    sign = nvec[1] >= 0
    theta = arccos(nvec[0])
  if not sign:
    theta *= -1
  return theta

func det2x2(t: Tensor[float]): Result[float, CatchableError] =
  if t.shape != [2, 2]:
    return CatchableError(msg: "input's shape must be 2x2").err()
  
  return (t[0, 0]*t[1, 1] - t[0, 1]*t[1, 0]).ok()

func area_2d*(t: Tensor[float]): Result[float, CatchableError] = 
  return (0.5*abs(det2x2(t).value)).ok()

proc diagonalize*(t: Tensor[float]): Result[Tensor[float], CatchableError] = 
  if t.shape[0] != t.shape[1] and t.rank != 2:
    return CatchableError(msg: "input must be square matrix!!").err()
  var ans = zeros_like(t)
  for i in 0..<t.shape[0]:
    ans[i, i] = t[i, i]
  
  return ans.ok()