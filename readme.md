# EITアルゴリズムの実装

## 使用方法

nim2.0.4とOpenBLASがインストールされている環境下で、nimbleを用いて必要なライブラリ(EIT.nimbleに記載)をインストール

nimble build -r でプログラムをビルド&実行

## 手法の説明

explanations フォルダ内にFEM、EITの基本的な原理、実装詳細を記載

## TODO

- [ ] 計測電圧誤差を設定可能にする
- [ ] 初期導電率分布の予測の誤差を設定可能にする
- [ ] 単位を導入する(Unchained)
- [ ] 楕円形やB-spline等の外形形状に対応する