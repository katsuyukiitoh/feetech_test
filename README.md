# feetech_test

## サーボ事前設定
- ID(0x05)
- Velocity P Gain(0x25) ... 254(デフォルト:10、変化はよくわからず）
- Velocity I Gain(0x27) ... 254(デフォルト:10、100くらいから変化はよくわからず）

## 実行方法
一つのターミナルで。
```
colcon build --packages-select feetech_test
ros2 run feetech_test feetech_test
```
もう一つのターミナルで。
```
ros2 launch feetech_test teleop-launch.py
```
