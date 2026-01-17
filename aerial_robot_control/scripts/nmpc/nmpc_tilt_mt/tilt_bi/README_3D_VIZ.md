# Pinocchio 3D Visualization for Tiltable Birotor

这个脚本使用 Pinocchio 自带的 **MeshCat** 可视化工具，在浏览器中实时显示机器人的 3D 运动。

## 📦 安装

首先安装 MeshCat：

```bash
pip3 install meshcat
```

## 🚀 使用方法

### 快速开始

```bash
cd /home/li-jinjie/ros1/jsk_aerial_robot_ws/src/jsk_aerial_robot_dev/aerial_robot_control/scripts/nmpc/nmpc_tilt_mt/tilt_bi

# 运行可视化脚本
python3 visualize_pinocchio_3d.py
```

### 运行流程

1. **启动脚本**：运行 `python3 visualize_pinocchio_3d.py`
2. **选择 Demo**：根据提示选择 1-4
3. **打开浏览器**：脚本会自动打开浏览器窗口，显示 URL（通常是 `http://127.0.0.1:7000/static/`）
4. **观看仿真**：在浏览器中看到机器人的 3D 模型实时运动
5. **交互**：
   - 鼠标左键拖动：旋转视角
   - 鼠标右键拖动：平移
   - 滚轮：缩放

## 🎬 可用的 Demo

### Demo 1: Hover with Disturbances（推荐首次使用）
- 机器人在 1m 高度悬停
- 有周期性的推力扰动
- 舵机小幅摆动
- 演示时间：10 秒

### Demo 2: Gimbal Actuation Patterns
- 展示舵机关节的不同运动模式
- 三个阶段：
  - 0-4s：两个舵机同步运动
  - 4-8s：两个舵机反向运动
  - 8-12s：两个舵机以不同频率运动
- 演示时间：12 秒

### Demo 3: Circular Trajectory Following
- 机器人飞行圆形轨迹
- 使用简单的 P 控制器跟踪目标
- 半径 0.5m，周期 8s
- 演示时间：16 秒（2 圈）

### Demo 4: Run All Demos
- 依次运行所有 Demo

## 📊 输出说明

运行时终端会显示：

```
  ⏱  t= 0.00s  |  pos=[ 0.000,  0.000,  1.000]  |  gimbal=[ 0.000,  0.000] rad
  ⏱  t= 1.00s  |  pos=[-0.001,  0.002,  0.998]  |  gimbal=[ 0.095, -0.031] rad
  ...
```

- `t`: 仿真时间（秒）
- `pos`: 机器人位置 [x, y, z]（米）
- `gimbal`: 两个舵机角度 [α1, α2]（弧度）

## 🔧 技术细节

### 关键组件

1. **PinocchioBirotorVisualizer 类**
   - 封装了 Pinocchio 的 MeshcatVisualizer
   - 自动转换 NMPC 状态格式到 Pinocchio 格式
   - 提供简单的 `update_from_state()` 接口

2. **状态转换**
   ```python
   # NMPC state (15D): [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz, a1, a2]
   # Pinocchio q (9D): [px, py, pz, qx, qy, qz, qw, a1, a2]
   ```

3. **可视化更新频率**
   - 仿真频率：100 Hz (dt=0.01s)
   - 可视化更新：50 Hz（每 2 步更新一次）
   - 终端输出：10 Hz（每 100 步输出一次）

### Pinocchio API 使用

这个脚本展示了以下 Pinocchio 功能：

```python
# 1. 创建可视化器
from pinocchio.visualize import MeshcatVisualizer
viz = MeshcatVisualizer(model, collision_model=None, visual_model=None)

# 2. 初始化并启动服务器
viz.initViewer(open=True)  # open=True 自动打开浏览器
viz.loadViewerModel()       # 加载机器人模型

# 3. 更新显示
viz.display(q)  # q 是广义坐标
```

## 🐛 故障排除

### 问题 1: ImportError: No module named 'meshcat'

**解决方案**：
```bash
pip3 install meshcat
```

### 问题 2: 浏览器没有自动打开

**解决方案**：
手动打开终端输出的 URL，通常是：
```
http://127.0.0.1:7000/static/
```

### 问题 3: 端口 7000 已被占用

**解决方案**：
1. 关闭其他使用 7000 端口的程序
2. 或者修改 MeshCat 端口（需要修改代码）

### 问题 4: 可视化很卡顿

**解决方案**：
- 降低可视化更新频率（修改 `if i % 2 == 0:` 中的数字）
- 减少仿真时间
- 检查浏览器性能

### 问题 5: 找不到 URDF 文件

**解决方案**：
确保你在正确的工作空间中，URDF 文件应该在：
```
robots/gimbalrotor/robots/bi/gimbalrotor.urdf.xacro
```

## 📚 学习资源

### Pinocchio 文档
- [官方文档](https://stack-of-tasks.github.io/pinocchio/)
- [MeshCat 可视化教程](https://github.com/stack-of-tasks/pinocchio/blob/master/examples/meshcat-viewer.py)
- [示例集合](https://github.com/stack-of-tasks/pinocchio/tree/master/examples)

### 相关脚本
- `tilt_bi_pinocchio_sim.py` - Pinocchio 模拟器实现
- `test_pinocchio_sim.py` - 单元测试
- `../../sim_nmpc.py` - 完整 NMPC 仿真（带 matplotlib 可视化）

## 🎓 进阶使用

### 自定义仿真参数

编辑脚本中的参数：

```python
# 修改仿真时间
t_total = 10.0  # 秒

# 修改推力扰动幅度
thrust1 = hover_thrust + 0.5 * np.sin(...)  # 改变 0.5

# 修改舵机摆动幅度
alpha1 = 0.1 * np.sin(...)  # 改变 0.1
```

### 添加自定义 Demo

在脚本末尾添加新函数：

```python
def demo_my_custom():
    """自定义 Demo"""
    sim = create_pinocchio_sim_solver(dt=0.01)
    viz = PinocchioBirotorVisualizer(sim.simulator)

    # 你的仿真逻辑
    ...
```

## 💡 提示

1. **首次使用**：建议先运行 Demo 1 熟悉界面
2. **观察细节**：可以暂停终端（Ctrl+Z）仔细观察浏览器中的模型
3. **对比学习**：同时运行 matplotlib 可视化（`sim_nmpc.py`）和 3D 可视化，对比理解
4. **调试工具**：3D 可视化非常适合调试控制算法和检查 URDF 模型

## ✨ 下一步

学习完 3D 可视化后，你可以：

1. **修改控制器**：在 Demo 3 中改进轨迹跟踪算法
2. **添加新轨迹**：实现 8 字形、螺旋等轨迹
3. **测试极限**：尝试更大的舵机角度或更快的机动
4. **对比仿真**：与 Gazebo 仿真结果对比
5. **集成 NMPC**：将 3D 可视化集成到完整的 NMPC 系统中

祝学习愉快！🚀
