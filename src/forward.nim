## 1. 前半は静電場FEM、後半は差分再構成法を使用したEIT
## 2. 領域外形は円形を仮定
## 3. 電極位置は領域外形を等分したものと仮定(即ち、最外殻ノードは全て電極と仮定)
## 4. メッシュ分割アルゴリズムはdelauney法(https://ryutai.ninja-web.net/mesh/mesh_4.html)を利用する
## 5. 計測中に領域は不変であり、メッシュもまた不変であると仮定
## 6. FEMアルゴリズムはガラーキン法を採用
## 7. 再構成アルゴリズムはヤコビアンを用いた差分再構成法であるため、順方向計算時にヤコビアンを計算可能にする
## 8. ヤコビアンJはエレメント毎に、 J[e] = R_e*K_e*V_e で求まる、詳しくはChatGPT様との会話参照
## 9. 一旦密行列計算で実装

import std/[sequtils]
import arraymancer, results
import mesh, calc

proc stiffness_mat_local_tri*(xy: Tensor[float]): Result[Tensor[float], CatchableError] =
  ## 三角形エレメントにおける剛性行列K_ijの計算
  ## https://github.com/eitcom/pyEIT/blob/master/pyeit/eit/fem.py
  
  if xy.shape != [3, 2]:
    return CatchableError(msg: "input's shape must be [3, 2]").err()

  let
    edges = concat(xy[2, _], xy[0, _], xy[1, _], axis = 0) - concat(xy[1, _], xy[2, _], xy[0, _], axis = 0)
    area = area_2d(edges[0..1]).value

  return (edges*edges.transpose / (4.0*area)).ok()

proc stack_stiffness_mat_local_tri*(mesh: Mesh): Result[Tensor[float], CatchableError] =
  ## 各三角形エレメントに対する局所剛性行列をスタックしてエレメント数*3*3の行列を得る
  var localStiffnessMat = zeros[float]([len(mesh.elements), 3, 3])
  
  for (i, element) in mesh.elements.pairs():
    var xy = zeros[float]([3, 2])
    xy[0, _] = [mesh.vertices[element.idxVertice1].pos[0], mesh.vertices[element.idxVertice1].pos[1]].toTensor
    xy[1, _] = [mesh.vertices[element.idxVertice2].pos[0], mesh.vertices[element.idxVertice2].pos[1]].toTensor
    xy[2, _] = [mesh.vertices[element.idxVertice3].pos[0], mesh.vertices[element.idxVertice3].pos[1]].toTensor
  
    localStiffnessMat[i, _] = stiffness_mat_local_tri(xy).value().reshape(1, 3, 3)
  
  return localStiffnessMat.ok()

proc create_stiffness_mat*(mesh: Mesh, mat_local: Tensor[float]): Result[Tensor[float], CatchableError] =
  ## エレメント数*3*3の局所剛性行列をスタックさせた行列を、頂点数*頂点数の密行列にマッピング
  ## 電位基準点を外周上の θ = π/2 の位置に設定、その行と列に対応する要素の内対角成分以外を0に、対角成分を1に設定
  var stiffnessMatrix = zeros[float]([len(mesh.vertices), len(mesh.vertices)])
  for (i, element) in mesh.elements.pairs():
    var idxVerts: seq[int]
    idxVerts.add(element.idxVertice1)
    idxVerts.add(element.idxVertice2)
    idxVerts.add(element.idxVertice3)
    
    for r in 0..<3:
      for c in 0..<3:
        stiffnessMatrix[idxVerts[r], idxVerts[c]] += mat_local[i, r, c]

  let idxReferenceVertice = (mesh.numOuterVertices div 4)

  
  stiffnessMatrix[idxReferenceVertice, _] = zeros_like(stiffnessMatrix[idxReferenceVertice, _])
  stiffnessMatrix[_, idxReferenceVertice] = zeros_like(stiffnessMatrix[_, idxReferenceVertice])
  stiffnessMatrix[idxReferenceVertice, idxReferenceVertice] = 1.0
  
  return stiffnessMatrix.ok()