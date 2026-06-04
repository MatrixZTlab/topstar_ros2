## 上半身电机的编码器归零

* 使用 SSH 远程登录机器人，然后使用终端上的命令行将上半身电机的编码器归零。
```bash
$ ssh -X test@192.168.1.10
# password: 123456

test@KiWibot:~$ ethercat slave
...

Master2
  0  0:0  OP  +  CD-ECxxxx(COE)
Master3
   0      1:0  OP     +  EC-6SW(IN,X2,X3)
   1      1:1  OP     +  CD-ECxxxx(COE)
   2      1:2  OP     +  CD-ECxxxx(COE)
   3      1:3  OP     +  CD-ECxxxx(COE)
   4      1:4  OP     +  CD-ECxxxx(COE)
   5      1:5  OP     +  CD-ECxxxx(COE)
   6      1:6  OP     +  CD-ECxxxx(COE)
   7      1:7  OP     +  CD-ECxxxx(COE)
   8      1:8  OP     +  CD-ECxxxx(COE)
   9      1:9  OP     +  CD-ECxxxx(COE)
  10  13330:0  OP     +  LSLQ_DH116
  11      4:0  PREOP  +  EC-6SW(X4,X5,X6)
  12      4:1  OP     +  CD-ECxxxx(COE)
  13      4:2  OP     +  CD-ECxxxx(COE)
  14      4:3  OP     +  CD-ECxxxx(COE)
  15      4:4  OP     +  CD-ECxxxx(COE)
  16      4:5  OP     +  CD-ECxxxx(COE)
  17      4:6  OP     +  CD-ECxxxx(COE)
  18      4:7  OP     +  CD-ECxxxx(COE)
  19  13330:0  OP     +  LSLQ_DH116
```
* 腰部电机, 主站 2, 从站位置 0
* 左臂电机, 主站 3, 从站位置 3~9
* 右臂电机, 主站 3, 从站位置 12~18
* 例如，将 right shoulder pitch (12)关节的编码器归零。
```
ethercat download -m 3 -p 12 -t uint16 0x2711 1 1
```
* 例如，将 left wrist yaw (7)关节的编码器归零。
```
ethercat download -m 3 -p 7 -t uint16 0x2711 1 1
```
* 请注意，在将上身关节电机的编码器归零后，需要将机器人**完全断电**并重新启动，才能使归零生效。

## 下半身电机编码器归零

按照以下步骤将腿部电机的编码器归零：

1. 在当前计算机远程运行机器人上的 `motor_gui.py`:
```bash
ssh -X test@192.168.1.10
# password: 123456
cd ~/topstar_h2/
python3 python/motor_gui.py -f ec_rt.conf
``` 
然后您应该会看到一个电机控制界面弹出。

2. 按下`Connect to Shared memory`按钮，然后按下 `Enter MANUAL Mode` 按钮。此时当前状态变为 `MANUAL(9)`，您可以开始将腿部电机的编码器归零。

3. 在左腿选项卡中，有 6 个关节电机。每个电机的控制面板上都有一个 `Zero Encoder` 按钮。请确保关节处于机械原点位置，然后按下 `Zero Encoder` 按钮，即可将关节电机的编码器重置为零。

4. 如果 ankle_mode=PR，则会在选项卡标题中 `Enable All`/`Disable All` 按钮旁边添加 `Zero Ankle Pair` 按钮。在 AB模式下，该按钮不会显示。此设计允许您同时将两个关节归零，因为它们在运动学上是耦合的。

5. 完成所需关节电机的归零后，关闭电机控制界面并重启机器人。

## 更新流程说明

1. 将压缩文件解压到当前文件夹。系统将创建一个名为`topstar_h2_v2_update_xxxxxxxx_xxxxxx`的新文件夹。

2. 进入新创建的文件夹。

3. 使用指定选项运行以下脚本。该脚本将执行多个步骤来安装更新并将软件作为系统服务运行。运行此脚本后，机器人即可使用。无需重启机器人。
```
./deploy/systemd/update_v2_services_remote.sh \
  --host 192.168.1.10 \
  --password 123456
```


