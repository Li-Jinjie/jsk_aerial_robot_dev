# Pinocchio Birotor 快速参考

## 一键测试
```bash
cd aerial_robot_control/scripts/nmpc
./test_pinocchio_integration.sh
```

## 常用命令

### 基础仿真
```bash
# 标准单刚体模型
python3 sim_nmpc.py 0 -a bi --sim_model 0

# Pinocchio 多刚体模型（推荐）
python3 sim_nmpc.py 0 -a bi --sim_model 2
```

### 对比模式
```bash
# 步骤1: 运行标准模型并保存
python3 sim_nmpc.py 0 -a bi --sim_model 0 --save_data

# 步骤2: 运行 Pinocchio 并对比
python3 sim_nmpc.py 0 -a bi --sim_model 2 --viz_comparison \
  --comparison_data_path ../../../../test/data/nmpc_NMPCTiltBiServo_sim_NMPCTiltBiServo.npz
```

### 无可视化运行
```bash
python3 sim_nmpc.py 0 -a bi --sim_model 2 --no_viz
```

## 参数说明

| 参数 | 说明 | 默认值 |
|-----|------|-------|
| `--sim_model 0` | 标准单刚体 | ✓ |
| `--sim_model 2` | Pinocchio 多刚体 | |
| `--no_viz` | 禁用可视化 | False |
| `--viz_pinocchio` | Pinocchio 专用图 | True (当 sim_model=2) |
| `--viz_comparison` | 对比可视化 | False |
| `--save_data` | 保存数据 | False |

## 可视化内容

### 标准图 (所有模型)
- 位置 (x, y, z)
- 速度 (vx, vy, vz)
- 姿态 (roll, pitch, yaw)
- 角速度 (wx, wy, wz)
- 控制输入 (推力, 舵机角度)
- 计算时间

### Pinocchio 专用图 (sim_model=2)
- ⚡ 关节力矩
- 🔄 关节速度
- 📊 系统能量 (动能+势能)
- ⚡ 能量变化率
- 📍 重心位置
- 🚀 推力大小
- ⬆️ 推力 Z 分量
- 🎯 舵机跟踪

### 对比图 (--viz_comparison)
- 📈 位置对比
- ❌ 位置误差
- 💨 速度对比
- 🔄 姿态对比
- 🎛️ 舵机对比
- 📏 误差范数

## 性能

| 模型 | 速度 | 精度 | 用途 |
|-----|------|------|------|
| 标准 | ⚡⚡⚡ | ⭐⭐ | 快速原型 |
| Pinocchio | ⚡ | ⭐⭐⭐ | 高精度验证 |

## 故障排除

### 找不到 pinocchio 模块
```bash
conda install -c conda-forge pinocchio
```

### 可视化不显示
```bash
# 检查是否有 --no_viz 参数
# 确保 DISPLAY 环境变量已设置
echo $DISPLAY
```

### 仿真发散
- 降低目标角度
- 检查控制器参数
- 验证 URDF 模型

## 文件位置

```
scripts/nmpc/
├── sim_nmpc.py                    # 主程序
├── test_pinocchio_integration.sh  # 测试脚本
├── README_PINOCCHIO.md            # 完整文档
├── SUMMARY_zh.md                  # 完成总结
└── nmpc_tilt_mt/
    ├── utils/pinocchio_viz.py     # 可视化模块
    └── tilt_bi/
        ├── tilt_bi_pinocchio_sim.py  # 模拟器
        └── test_pinocchio_sim.py      # 单元测试
```

## 更多信息

📖 完整文档: `README_PINOCCHIO.md`
📋 实施计划: `plan-pinocchioBirotorIntegration.prompt.md`
📝 完成总结: `SUMMARY_zh.md`
