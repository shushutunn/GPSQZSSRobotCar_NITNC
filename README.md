# GPSQZSSRobotCar_NITNC

GNSS・QZSS ロボットカーコンテスト 2022
エントリー No.12 　奈良工業高等専門学校　ロボット名：無限廻車
**祝・特別賞受賞！！**

1. ディレクトリについて
   |:--------------|:-----------------------------------|
   | compassdata | 地磁気センサ補正用のデータ |
   | ContestCode | **大会本番で用いたソースコード** |
   | for_ArduinoDue| ArduinoDue での検証のプログラム |
   | for_GCC | GCC での検証プログラム |

2.主な検証プログラム
+for_ArduinoDue 内
+getdata_func.ino: 地磁気センサ、GNSS データ取得の検証
+ApproachWP.ino: 1 つの地点に近づく検証
+FigureEight.ino: 8 の字周回検証
+BonusPoint.ino: ボーナスポイント検証
+reiwa.ino: REIWA ポイント検証バージョン１
+reiwa2.ino: REIWA ポイント検証バージョン２
+reiwa2_TimeMemory.ino: リスタート対応のため、スタート時刻を EEPROM に記録する機能を追加。大会本番のソースコード。

3.ハードウェア構成
