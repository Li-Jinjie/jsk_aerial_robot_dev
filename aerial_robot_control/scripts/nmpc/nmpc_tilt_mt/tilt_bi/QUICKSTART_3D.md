# 🚀 Pinocchio 3D 可视化 - 快速参考

## 一键运行

```bash
cd /home/li-jinjie/ros1/jsk_aerial_robot_ws/src/jsk_aerial_robot_dev/aerial_robot_control/scripts/nmpc/nmpc_tilt_mt/tilt_bi
python3 visualize_pinocchio_3d.py
```

## 安装（如果需要）

```bash
pip3 install meshcat
```

## Demo 选项

| 输入 | Demo | 时长 | 说明 |
|------|------|------|------|
| 1 | Hover + Disturbances | 10s | 悬停 + 周期性扰动（推荐） |
| 2 | Gimbal Actuation | 12s | 舵机运动模式展示 |
| 3 | Circular Trajectory | 16s | 圆形轨迹跟踪 |
| 4 | All Demos | 38s | 运行全部 Demo |

## 浏览器操作

- **旋转**: 鼠标左键拖动
- **平移**: 鼠标右键拖动
- **缩放**: 滚轮
- **URL**: http://127.0.0.1:7000/static/

## 文件位置

```
tilt_bi/
├── visualize_pinocchio_3d.py       # 主脚本 ⭐
├── README_3D_VIZ.md                # 详细文档
├── IMPLEMENTATION_SUMMARY.md       # 实现总结
└── tilt_bi_pinocchio_sim.py        # 模拟器（已有）
```

## 快速故障排除

| 问题 | 解决方案 |
|------|----------|
| ImportError: meshcat | `pip3 install meshcat` |
| 浏览器未打开 | 手动打开 http://127.0.0.1:7000/static/ |
| 端口被占用 | `sudo lsof -ti:7000 \| xargs kill -9` |
| 看不到模型 | 刷新浏览器或检查 URDF 路径 |

## 学习要点

✅ **MeshcatVisualizer 用法**
```python
from pinocchio.visualize import MeshcatVisualizer
viz = MeshcatVisualizer(model, collision_model=None, visual_model=None)
viz.initViewer(open=True)
viz.loadViewerModel()
viz.display(q)
```

✅ **状态转换**
```python
# NMPC → Pinocchio
q, v = simulator._nmpc_state_to_pinocchio(x_state)
```

✅ **实时更新**
```python
for i in range(n_steps):
    sim.solve()
    x = sim.get("x")
    viz.update_from_state(x)
    time.sleep(0.005)
```

## 下一步

1. 🎯 运行 Demo 1 体验基础功能
2. 🔧 修改参数（推力、舵机角度）
3. 📝 添加自定义 Demo
4. 🔬 与 sim_nmpc.py 的 matplotlib 可视化对比

## 更多信息

- 📖 完整文档: `README_3D_VIZ.md`
- 📋 实现总结: `IMPLEMENTATION_SUMMARY.md`
- 💻 代码: `visualize_pinocchio_3d.py`
