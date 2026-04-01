# w10_kinematics

W10机械臂逆运动学求解器，基于臂角降维方法实现。

## 功能概述

该功能包提供：
- 基于Pinocchio的动力学和运动学计算
- 正向运动学求解
- 基于臂角降维的逆运动学求解器
- C++库和ROS 2节点接口

## 依赖项

### 构建依赖
- ROS 2（Humble或更新版本）
- ament_cmake_auto
- rclcpp
- Eigen3
- pinocchio
- w10_sim（提供w10.urdf模型）

### 安装依赖（Ubuntu）
```bash
sudo apt install libeigen3-dev libpinocchio-dev
rosdep install --from-paths src --ignore-src -r -y
```

## URDF模型

该包使用来自`w10_sim`的`w10.urdf`模型，位置：
```
<w10_sim_share>/urdf/w10.urdf
```

运动学求解器会在运行时自动加载此模型。

## 构建

```bash
cd ~/ros2_ws/dynamic_ws
colcon build --packages-select w10_kinematics
```

## 使用

### 1. 测试可执行文件

运行基本的运动学测试：

```bash
source install/setup.bash
ros2 run w10_kinematics w10_ik_test
```

### 2. 运行IK节点

启动完整的逆运动学求解节点：

```bash
source install/setup.bash
ros2 launch w10_kinematics w10_ik.launch.py
```

### 3. 在您的代码中使用库

```cpp
#include "w10_kinematics/w10_kinematics_solver.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// 获取URDF路径
std::string urdf_path = ament_index_cpp::get_package_share_directory("w10_sim") 
                       + "/urdf/w10.urdf";

// 创建求解器
w10_kinematics::W10KinematicsSolver solver(urdf_path);

// 正向运动学
Eigen::VectorXd q(10);
q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
Eigen::Affine3d T;
solver.forwardKinematics(q, T);

// 逆运动学（使用臂角降维）
Eigen::Affine3d T_target = T;  // 目标位姿
Eigen::VectorXd q_init = q;    // 初始配置
Eigen::VectorXd q_solution;
solver.inverseKinematics(T_target, q_init, q_solution);
```

## Git Submodule配置

该包作为Git Submodule集成到`dynamic_ws`中：

```bash
# 查看submodule状态
git config -f .gitmodules --get-regexp .

# 更新submodule
git submodule update --init --recursive

# 从submodule提交
cd src/w10_kinematics
git add -A
git commit -m "commit message"
git push
```

## 项目结构

```
w10_kinematics/
├── include/w10_kinematics/
│   └── w10_kinematics_solver.hpp    # 主求解器头文件
├── src/
│   ├── w10_kinematics_solver.cpp    # 求解器实现
│   ├── w10_ik_node.cpp              # ROS 2 IK节点
│   └── w10_ik_test.cpp              # 测试可执行文件
├── launch/
│   └── w10_ik.launch.py             # ROS 2启动文件
├── config/
│   └── w10_kinematics.yaml          # 配置参数
├── CMakeLists.txt                   # CMake配置
└── package.xml                      # ROS 2包定义
```

## 臂角降维方法

臂角降维是一种减少冗余自由度的方法，通过约束某些关节角度的关系，将问题转化为低维优化问题。

实现细节：
- `reduceArmAngle()`: 将完整配置降维到主DOF
- `expandArmAngle()`: 从主DOF扩展到完整配置

## TODO

- [ ] 实现臂角降维算法的详细计算
- [ ] 添加逆运动学优化算法
- [ ] 创建ROS 2服务接口
- [ ] 添加单元测试
- [ ] 性能基准测试

## 许可证

Apache License 2.0

## 作者

Huang Yang (hyang@buaa.edu.cn)
