import math, macros, sugar
import results

########################################
# higher-order func (not proc)の短縮記法
# プラグマ見づらいので... 
########################################

template `>>`(p, b: untyped): untyped =
  (p {.noSideEffect.} -> b)

########################################

########################################
# Grid types and their utilities
# おせろ作った時に、こういうの欲しかったので自作
# method-like(第一引数名: self)はプロパティの延長とみなしcamelCase, それ以外のfuncはsnake_caseで実装
########################################

type
  Grid2D*[T] = object
    ## original non-linear idx grid type.
    ## index: 0-index
    ## order: (x: 0, y: 0) ->(x: xlim-1, y: 0) -> (x: 0, y: 1) -> ...
    ## direction: x-positive is right side, y-positive is up side.
    pts*: seq[T]
    xlen*: int
    ylen*: int

  Grid2DDirection* = enum
    ## PZ means dirction (x: positive, y: zero)
    ## I deceided not to use azimuth because direction should be free from other library's coordinate implementation (y-positive is up/down side, right-handed/left-handed, ...).
    ZP
    PP
    PZ
    PN
    ZN
    NN
    NZ
    NP


func spawnGrid2D*[T](val: seq[T], xlen: int, ylen: int): Result[Grid2D[T], CatchableError] =
  mixin err

  if xlen > 10^9 or ylen > 10^9:
    return CatchableError(msg: "Supported grid length is limited to 10^9").err()
  elif xlen < 0 or ylen < 0:
    return CatchableError(msg: "Grid length should be >= 1").err()
    
  else:
    return Grid2D[T](
      pts: val,
      xlen: xlen,
      ylen: ylen
    ).ok()

func init_grid2D*[T](xlen: int, ylen: int): Result[Grid2D[T], CatchableError] =

  if xlen > 10^9 or ylen > 10^9:
    return CatchableError(msg: "Supported grid length is limited to 10^9").err()
  elif xlen < 0 or ylen < 0:
    return CatchableError(msg: "Grid length should be >= 1").err()
    
  else:
    return Grid2D[T](
      pts: newSeq[T](xlen*ylen),
      xlen: xlen,
      ylen: ylen
    ).ok()

func getVal*[T](self: Grid2D[T], pos: (int, int)): Result[T, CatchableError] =
  let
    x = pos[0]
    y = pos[1]

  if x < 0 or x >= self.xlen: 
    return CatchableError(msg: "x index is invalid, be sure that x index is 0 <= x < " & $(self.xlen)).err()

  elif y < 0 or y >= self.ylen:
    return CatchableError(msg: "y index is invalid, be sure that y index is 0 <= y < " & $(self.ylen)).err()

  else:
    return (self.pts[x+y*self.xlen]).ok()

func getIdx*(self: Grid2D, pos: (int, int)): Result[int, CatchableError] =
  let
    x = pos[0]
    y = pos[1]

  if x < 0 or x >= self.xlen: 
    return CatchableError(msg: "x index is invalid, be sure that x index is 0 <= x < " & $(self.xlen)).err()

  elif y < 0 or y >= self.ylen:
    return CatchableError(msg: "y index is invalid, be sure that y index is 0 <= y < " & $(self.ylen)).err()

  else:
    return (x+y*self.xlen).ok()

func getPos*(self: Grid2D, idx: int): Result[(int, int), CatchableError] =

  if idx < 0 or idx >= self.xlen*self.ylen: 
    return CatchableError(msg: "idx is invalid, be sure that idx is 0 <= idx < " & $(self.xlen*self.ylen)).err()

  else:
    return (idx mod self.xlen, idx div self.xlen).ok()
  
func getNextPos*(self: Grid2D, pos: (int, int), dir: Grid2DDirection): Result[(int, int), CatchableError] =
  let
    x = pos[0]
    y = pos[1]

  case dir
  of ZP:
    if x < 0 or x >= self.xlen: 
      return CatchableError(msg: "x index is invalid, be sure that x index is 0 <= x < " & $(self.xlen)).err()

    elif y < 0 or y >= self.ylen - 1:
      return CatchableError(msg: "y index is invalid, be sure that y index is 0 <= y < " & $(self.ylen - 1)).err()

    else:
      return (x, y+1).ok()

  of PP:
    if x < 0 or x >= self.xlen - 1: 
      return CatchableError(msg: "x index is invalid, be sure that x index is 0 <= x < " & $(self.xlen - 1)).err()

    elif y < 0 or y >= self.ylen - 1:
      return CatchableError(msg: "y index is invalid, be sure that y index is 0 <= y < " & $(self.ylen - 1)).err()

    else:
      return (x+1, y+1).ok()
    
  of PZ:
    if x < 0 or x >= self.xlen - 1: 
      return CatchableError(msg: "x index is invalid, be sure that x index is 0 <= x < " & $(self.xlen - 1)).err()

    elif y < 0 or y >= self.ylen:
      return CatchableError(msg: "y index is invalid, be sure that y index is 0 <= y < " & $(self.ylen)).err()

    else:
      return (x+1, y).ok()
    
  of PN:
    if x < 0 or x >= self.xlen - 1: 
      return CatchableError(msg: "x index is invalid, be sure that x index is 0 <= x < " & $(self.xlen - 1)).err()

    elif y < 1 or y >= self.ylen:
      return CatchableError(msg: "y index is invalid, be sure that y index is 1 <= y < " & $(self.ylen)).err()

    else:
      return (x+1, y-1).ok()

  of ZN:
    if x < 0 or x >= self.xlen: 
      return CatchableError(msg: "x index is invalid, be sure that x index is 0 <= x < " & $(self.xlen)).err()

    elif y < 1 or y >= self.ylen:
      return CatchableError(msg: "y index is invalid, be sure that y index is 1 <= y < " & $(self.ylen)).err()

    else:
      return (x, y-1).ok()

  of NN:
    if x < 1 or x >= self.xlen: 
      return CatchableError(msg: "x index is invalid, be sure that x index is 1 <= x < " & $(self.xlen)).err()

    elif y < 1 or y >= self.ylen:
      return CatchableError(msg: "y index is invalid, be sure that y index is 1 <= y < " & $(self.ylen)).err()

    else:
      return (x-1, y-1).ok()
    
  of NZ:
    if x < 1 or x >= self.xlen: 
      return CatchableError(msg: "x index is invalid, be sure that x index is 1 <= x < " & $(self.xlen)).err()

    elif y < 0 or y >= self.ylen:
      return CatchableError(msg: "y index is invalid, be sure that y index is 0 <= y < " & $(self.ylen)).err()

    else:
      return (x-1, y).ok()
    
  of NP:
    if x < 1 or x >= self.xlen: 
      return CatchableError(msg: "x index is invalid, be sure that x index is 1 <= x < " & $(self.xlen)).err()

    elif y < 0 or y >= self.ylen - 1:
      return CatchableError(msg: "y index is invalid, be sure that y index is 0 <= y < " & $(self.ylen - 1)).err()

    else:
      return (x-1, y+1).ok()

########################################
