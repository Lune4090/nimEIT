import arraymancer

import results

# 再構成のベースにFEM(有限要素法)を使用 

# 皮膚外形はセンサ位置を繋いだ形とする

type
  Sensor = object
    X: float
    Y: float
    Z: float
    inI: float
    outV: float
    

var sensors: seq[Sensor]


# 面倒くさいので一旦楕円形状で一定間隔でセンサを形成
# 孤長積分の公式を利用

func shape_of_wrist(num_sensors: int, x: float, y: float): Result[seq[Sensor], CatchableError] = 
  if num_sensors <= 0:
    return CatchableError(msg: "num_sensors must be over 0").err()


var sensor0 = Sensor()

sensors = @[]
