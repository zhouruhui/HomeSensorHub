# WiFi-Switch

## 项目简介

WiFi-Switch是一个基于ESP32平台的WiFi阵列开关控制系统，专为智能家居场景设计。该项目支持Home Assistant MQTT自发现协议，让您可以通过Home Assistant轻松控制各种家庭中的开关量设备。

## 主要特点

- 基于ESP32平台开发
- 支持MQTT协议通信
- 兼容Home Assistant自发现功能
- 可控制多路开关设备
- 简单易用的配置方式
- 适用于各种智能家居场景

## 应用场景

- 智能照明控制
- 电器远程开关
- 智能窗帘/百叶窗控制
- 风扇、空调等家电控制
- 其他需要开关量控制的智能家居设备

## 安装说明

1. 下载项目代码
2. 使用Arduino IDE打开WifiSwitch.ino
3. 配置ESP32开发环境
4. 修改代码中的WiFi和MQTT配置信息
5. 编译并上传至ESP32设备

## 配置指南

在代码中设置以下参数：
- WiFi名称和密码
- MQTT服务器地址和端口
- 设备名称和ID
- 控制引脚定义

## 连接Home Assistant

1. 确保Home Assistant已启用MQTT集成
2. 上传程序至ESP32设备并启动
3. 设备将自动在Home Assistant中注册并可见
4. 通过Home Assistant界面可直接控制设备

## 参与贡献

欢迎对项目提出改进建议或提交PR，一起让这个项目变得更好！

## 许可证

MIT 
