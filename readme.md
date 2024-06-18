# EITアルゴリズムの実装

## 背景

### 有限差分法(FDM)と有限要素法(FEM)

FDMとFEMは、どちらもある支配方程式で表現される物理量について、一定領域内での値の分布を得る手法であるが、仕組みは大きく異なる

比較(例: 静電場)

- FDM
  1. 対象領域中にノードを**一定間隔で**配置
  2. 各ノードについて、物理量の違いに着目し、支配方程式(例: ポアソン方程式)を**位置変数**についての差分方程式として離散化
  3. それぞれのノードについて得た方程式と、境界条件を連立して解く

- FEM
  1. 対象領域中にノードを配置(**ランダムでも良い**)
  2. 複数のノードから成るエレメント(n角形領域)について、エレメント内の物理量分布を表す方程式(**形状関数**)を、そのエレメントを構成するノードの物理量を補間(1次式が基本だが高次も可)することで得る
  3. 形状関数を代入した支配方程式を汎関数(例: ポテンシャルエネルギー)に代入し、**物理量の変数(例: 電位)**について偏微分方程式を立式(**ポテンシャルエネルギー最小化**)
  4. それぞれのノードについて得た方程式と、境界条件を連立して解く

FDMと違いFEMが離散化を行なっていないように見えるのは、形状関数とポテンシャルエネルギー最小化原理を導入することで、支配方程式を離散化して連立する問題から、支配方程式の関数(離散化されていない)を最適化するのに必要な条件を連立する問題(ここは離散化されている)に問題を変形させるから

FEMがFDMよりも優れるのがノードの配置の自由度の高さ

## 変分原理に基づくFEMの定式化

### 変分原理とは

[参考1](https://ja.wikipedia.org/wiki/%E6%9C%80%E5%B0%8F%E4%BD%9C%E7%94%A8%E3%81%AE%E5%8E%9F%E7%90%86)

変分原理(歴史的名称: 最小作用の原理)
- _「力学系の運動は、**作用**と呼ばれる汎関数を停留値とするような軌道に沿って実現される」_

ここで、作用汎関数$S[\phi]$は、系の状態を指定する力学変数$\phi(x)$を引数にとる汎関数として与えられ、

$$
\frac{\delta{S[\phi]}}{\delta{\phi(x)}} = 0
$$

と表される。**ラグランジュ形式**において、**作用汎関数はラグランジュ関数Lの積分**

$$
S[q] = \int_{t_0}^{t_1}L(q(t), q'(t), t)dt
\\  
\\ eq.1: 作用汎関数のラグランジュ形式
$$

として与えられる。ここで、ラグランジュ形式における力学変数は一般化座標$q(t)$である事に注意。

### 具体例: 最速降下曲線の導出

[参考2](https://eman-physics.net/analytic/chapter3.html)


ちょっとこれだけだと具体例がなくて分かりづらいので、地点$A$から$B$までの最速降下曲線を求めることを変分原理で考えてみる。

対応関係は以下の通り。

- 作用 $S[\phi]$ = 降下時間 $t$
- 状態 $\phi(x)$ = 降下軌道 $f$
- ラグランジュ関数

$$
L(q(t), q'(t), t)dt = T = \int_A^B\sqrt{\frac{1 + f'(x)^2}{2gf(x)}}
$$

ここで求めたいのは、降下時間$t$を最小にするような降下軌道$f$である。すなわち

$$
\frac{\delta{t[f]}}{\delta{f(x)}} = 0
$$

となるような$f$を代入した$t$が解となる。

まずは分子に当たる $\delta t[f]$ (質点の軌道が微小変化( $f(x) \to f(x) + \delta f(x)$ )したときの降下時間の変分)を求める。

これは$eq.1$よりラグランジュ関数 $T$ の経路積分

$$
\delta t = \int_A^B \delta T dx
$$

を求めれば良いことが分かっている。被積分関数

$$
\delta T = T(f + \delta f, f' + \delta f) - T(f, f')
$$

については、テイラー展開で1次近似してしまえば

$$
\delta T = \frac{\partial T(f, f')}{\partial f}\delta f + \frac{\partial T(f, f')}{\partial f'}\delta f'
$$

であることがわかるので、$eq.1$に従いこれを再び作用汎関数 $t[f]$ に代入すれば

$$
\delta t = \int_A^B (\frac{\partial T}{\partial f}\delta f + \frac{\partial T}{\partial f'}\delta f') \ dx
$$

として立式される。後は $\delta f' = \frac{d(\delta f)}{dx}$ に注意して部分積分を行うと

$$
\delta t = \left[ \frac{\partial T}{\partial f'}\delta f \right]_A^B + \int_A^B (\frac{\partial T}{\partial f}\delta f - \frac{d}{dx}\frac{\partial T}{\partial f'}\delta f) \ dx
$$

となり、始点と終点が変化しない($\delta f(A) = \delta f(B)$)という条件の下では第一項が消えるので

$$
\delta t = \int_A^B (\frac{\partial T}{\partial f} - \frac{d}{dx}\frac{\partial T}{\partial f'})\delta f \ dx
$$

となる。ここで、**降下時間 $t$ が最短になる場合には変分 $\delta t$ が微小変化 $\delta f$ によらず0になる**、即ち被積分関数が0になっていれば良いので、結局求めたい関数は

$$
\frac{d}{dx}\frac{\partial T}{\partial f'} - \frac{\partial T}{\partial f} = 0
\\  
\\ eq.2: オイラー・ラグランジュ方程式？
$$

である。これはオイラー・ラグランジュ方程式と同型な方程式であることがわかる!

### 変分原理の静電場への応用

では、これを静電場の支配方程式であるラプラス方程式に適用することを考える

[参考3](https://qiita.com/atily17/items/fa8abcc4d778c16fa11a)


$$
\frac{\partial}{\partial x} \frac{\partial h}{\partial \frac{\partial u}{\partial x}} + \frac{\partial}{\partial y} \frac{\partial h}{\partial \frac{\partial u}{\partial y}} - \frac{\partial}{\partial x} - \frac{\partial h}{\partial u} = 0
$$



### Galerkin法を用いたFEMの定式化

[参考](https://qiita.com/atily17/items/fa8abcc4d778c16fa11a)

### EITアルゴリズムにおけるFEM順問題

### EITアルゴリズムにおける