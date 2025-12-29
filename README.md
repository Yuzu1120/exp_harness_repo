# exp_harness_repo

![CI](https://github.com/Yuzu1120/exp_harness_repo/actions/workflows/ci.yml/badge.svg)

## 概要（何をするソフトか）

**exp_harness_repo** は、ROS 2 上で  
**ノードのパラメータを変更した前後で、トピックの統計量を自動比較する実験用ハーネス**です。

サービス呼び出し１回で、以下を自動で実行します。

- パラメータ A を設定
- 一定時間メトリクスを計測
- パラメータ B を設定
- 再度メトリクスを計測
- 前後の平均・分散・差分を計算
- 結果を**ExperimentReport メッセージとしてpublish**

ロボットシステム学（ROS 2）における  
**「パラメータ変更が挙動に与える影響を定量的に評価する」** ためのツールです。

本ツールは、制御パラメータやゲイン調整の効果を  
**定量的に比較したいロボット開発者・学生向け**に設計されています。

※ 本リポジトリ名は `exp_harness_repo` ですが、    
  ROS 2 の **パッケージ名は `exp_harness`** です。  
  そのため、起動時は `ros2 launch exp_harness ...` を使用します。

## システム構成

本リポジトリでは以下のノードが動作します。

| ノード名 | 役割 |
|---|---|
| metric_pub | メトリクス（Float32）を `/metric` に publish |
| experiment_server | 実験制御・統計計算・レポート生成 |
| report_printer | `ExperimentReport` を購読して表示 |

## 対応環境

### OS
- Ubuntu 22.04 LTS
- WSL2（Ubuntu 22.04）
- Linux 環境（ROS 2 Humble）

### 必要ソフトウェア
- ROS 2 Humble Hawksbill
- Python 3.10

### 依存パッケージ (Ros 2)
- rclpy  
- std_msgs  
- rcl_interfaces  

※ すべてROS 2 Humble標準パッケージのみを使用しています。

## ビルド方法（参考）

※ ROS 2 のパッケージはインストール方法が規格化されているため、  
以下は **動作確認用の一例**です。

```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/Yuzu1120/exp_harness_repo.git

$ cd ~/ros2_ws
$ source /opt/ros/humble/setup.bash
$ colcon build --symlink-install
$ source install/setup.bash
```

## 使い方（重要：ターミナルの順番）

このパッケージは**３つのターミナル**を使います。  
**順番を間違えると表示されない**ので注意してください。

### ターミナル１：ノード起動（最後まで閉じない）

```bash
$ cd ~/ros2_ws
$ source /opt/ros/humble/setup.bash
$ source install/setup.bash
$ ros2 launch exp_harness demo.launch.py
```

起動後、以下のようなログが出れば正常です。
- `Publishing metric on /metric`
- `metric_topic=/metric, report_topic=/exp/report`

### ターミナル２：レポート待機

```bash
$ cd ~/ros2_ws
$ source /opt/ros/humble/setup.bash
$ source install/setup.bash
$ ros2 topic echo /exp/report
```

※ この時点では**何も表示されなくて正常**です。  
（レポートがpublish されるのを待っています）

### ターミナル３：実験を実行（サービス呼び出し）

```bash
$ cd ~/ros2_ws
$ source /opt/ros/humble/setup.bash
$ source install/setup.bash

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

response:
exp_harness_interfaces.srv.RunExperiment_Response(
  accepted=True,
  message='レポートを publish しました'
)
```

## 出力されるレポート例

**ターミナル２**に以下のような出力が表示されます。

```test
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

本リポジトリには`test/test.bash`による自動テストが含まれています。
テストでは以下を確認しています。

- launch が正常に起動する
- ノード・サービス・トピックが存在する
- RunExperiment サービスが accepted=True を返す
- ExperimentReport が実際に publish される
- レポート内容が正しい形式である

サービス呼び出し結果の確認に加え、  
**実際にレポートが publish されることまで検証**しています。

GitHub Actions 上で自動実行されています。

## ライセンス・謝辞
- このソフトウェアパッケージは、3条項BSDライセンスの下、再頒布および使用が許可されます。
- このパッケージには、千葉工業大学「ロボットシステム学」授業資料に含まれるコード例を参考・引用した部分があります。
  - 引用元（ライセンス：CC-BY-SA 4.0 by Ryuichi Ueda）：
    - [ryuichiueda/my_slides robosys_2025](https://github.com/ryuichiueda/my_slides/tree/master/robosys_2025)
- また、このパッケージには、以下のウェブサイトに掲載されている内容も参考にしています。
  - https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
  - https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
  - https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Parameters/Understanding-ROS2-Parameters.html
  - https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm

© 2025 Yuzuki Fujita
