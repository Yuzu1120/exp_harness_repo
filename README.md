# exp_harness_repo

![CI](https://github.com/Yuzu1120/exp_harness_repo/actions/workflows/ci.yml/badge.svg)

## 概要

**exp_harness_repo** は、ROS 2 上で  
**ノードのパラメータを変更した前後で、トピックの統計量を自動比較する実験用ハーネス**です。

サービス呼び出し１回で、以下を自動で実行します。

- パラメータ A を設定
- 一定時間メトリクスを計測
- パラメータ B を設定
- 再度メトリクスを計測
- 前後の平均・分散・差分を計算
- 結果をExperimentReport メッセージとしてpublish

## パッケージ構成

本リポジトリには以下の ROS 2 パッケージが含まれています。

| パッケージ名 | 内容 |
|---|---|
| exp_harness | 実験制御・統計計算・レポート生成 |
| exp_harness_interfaces | サービス・メッセージ定義 |

## ノード一覧

| ノード名 | 役割 |
|---|---|
| metric_pub | メトリクス（Float32）を `/metric` に publish |
| experiment_server | 実験制御・統計計算・レポート生成 |
| report_printer | `ExperimentReport` を購読して表示 |

## トピック一覧

| トピック名 | 型 | 内容 |
|---|---|---|
| `/metric` | std_msgs/msg/Float32 | 計測対象のメトリクス |
| `/exp/report` | exp_harness_interfaces/msg/ExperimentReport | 実験結果レポート |

## サービス一覧

| サービス名 | 型 | 内容 |
|---|---|---|
| `/experiment/run` | exp_harness_interfaces/srv/RunExperiment | 実験の実行要求 |

## 対応環境

### OS
- Ubuntu 22.04 LTS
- WSL2（Ubuntu 22.04）
- Linux 環境（ROS 2 Humble）

### 必要なソフトウェア
- ROS 2 Humble Hawksbill
- Python 3.10

### 依存パッケージ (Ros 2)
- rclpy  
- std_msgs  
- rcl_interfaces  

※ すべてROS 2 Humble標準パッケージのみを使用しています。

## ビルド方法（参考）

※ ROS 2 のパッケージはインストール方法が規格化されているため、  
以下は動作確認用の一例です。

```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/Yuzu1120/exp_harness_repo.git

$ cd ~/ros2_ws
$ source /opt/ros/humble/setup.bash
$ colcon build --symlink-install
$ source install/setup.bash
```

## 使い方

まずノードを起動します。

```bash
$ ros2 launch exp_harness demo.launch.py
```

ノード起動後、以下のサービスを呼び出すことで実験を実行できます。

```bash
$ ros2 service call /experiment/run exp_harness_interfaces/srv/RunExperiment "{
    experiment_id: 'ci_demo',
    metric_topic: '/metric',
    target_node: '/metric_pub',
    param_name: 'gain',
    a_value: 0.5,
    b_value: 1.5,
    pre_duration_sec: 0.7,
    post_duration_sec: 0.7
  }"
```

実験結果は`/exp/report`トピックにpublish されます。

## 出力されるレポート例

```text
stamp:
  sec: 1766671365
  nanosec: 433872370
experiment_id: ci_demo
metric_topic: /metric
target_node: /metric_pub
param_name: gain
a_value: 0.5
b_value: 1.5
pre_duration_sec: 0.7
post_duration_sec: 0.7
pre_n: 35
pre_mean: -0.18034233360418248
pre_var: 0.03936059062857802
post_n: 35
post_mean: 0.48904339415686476
post_var: 0.029425072485350992
delta_mean: 0.6693857277610472
score: 3.3740044032091876
success: true
note: ok
```

## 各項目の意味（要点）

- **pre_mean / post_mean**  
  パラメータ変更前後の平均値

- **pre_var / post_var**  
  パラメータ変更前後の分散

- **delta_mean**  
  変化量（post − pre）

- **score**  
  標準偏差で正規化した変化の大きさ

- **success**  
  サンプル数が十分かどうか
 

## テストについて

本リポジトリには自動テストが含まれています。  
詳細は`TEST.md`を参照してください。

## ライセンス・謝辞
- このソフトウェアパッケージは、3条項BSDライセンスの下、再頒布および使用が許可されます。
- 本パッケージは、千葉工業大学「ロボットシステム学」の授業資料を参考にして作成しました。
  - https://ryuichiueda.github.io/slides_marp/robosys2025/lesson8.html
  - https://ryuichiueda.github.io/slides_marp/robosys2025/lesson9.html
  - https://ryuichiueda.github.io/slides_marp/robosys2025/lesson10.html
- また、本パッケージの作成にあたっては、以下のウェブサイトの内容を参考にしています。
  - https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
  - https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
  - https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Parameters/Understanding-ROS2-Parameters.html
  - https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm

© 2025 Yuzuki Fujita
