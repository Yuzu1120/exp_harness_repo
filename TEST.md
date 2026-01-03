# テストについて

本リポジトリには`test/test.bash`による自動テストが含まれています。
テストでは以下を確認しています。

- launch が正常に起動する
- ノード・サービス・トピックが存在する
- RunExperiment サービスが accepted=True を返す
- ExperimentReport が実際に publish される
- レポート内容が正しい形式である

サービス呼び出し結果の確認に加え、  
実際にレポートが publish されることまで検証しています。

GitHub Actions 上で自動実行されています。
