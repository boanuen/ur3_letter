# UR3e Letter Writer — ROS 2 MoveIt 2 


## Tổng quan

- Robot: **UR3e** 
- Chữ: **T** 
- Mỗi chữ được chia thành các nét; giữa các nét, end-effector nâng lên, di chuyển, rồi hạ xuống vẽ nét tiếp theo
- Quỹ đạo vẽ hiển thị trên RViz bằng Marker (đường xanh = toàn bộ, đường đỏ = nét vẽ thực)

- Ubuntu 22.04 (hoặc WSL2)
- ROS 2 Humble
- Gazebo Ignition (Fortress)
- Các repo upstream (clone vào cùng workspace, branch `humble`):
  - [Universal_Robots_ROS2_GZ_Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/humble)
  - [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
  - [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble)

## Cài đặt

```bash
# Tạo workspace
mkdir -p ~/ur3e_ws/src && cd ~/ur3e_ws/src

# Clone các repo upstream (branch humble)
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git

# Clone package này
git clone https://github.com/<USERNAME>/ur3e_letter_writer.git

# Cài dependencies
cd ~/ur3e_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

## Chạy

```bash
# Terminal 1: Launch simulation + MoveIt + letter writer node
ros2 launch ur3e_letter_writer letter_writer.launch.py

# Trên WSL2, nếu Gazebo GUI bị crash:
ros2 launch ur3e_letter_writer letter_writer.launch.py gazebo_gui:=false
```

Node sẽ tự động:
1. Chờ MoveIt services sẵn sàng
2. Di chuyển đến vị trí bắt đầu (approach move)
3. Vẽ chữ T bằng Cartesian path
4. Publish marker liên tục trên RViz

## Xem kết quả trên RViz

Sau khi launch, trong RViz:
1. Click **Add** → **By topic** → chọn `/letter_writer/eef_path` → **Marker**
2. Đường xanh: toàn bộ quỹ đạo (bao gồm pen-up)
3. Đường đỏ: chỉ các nét vẽ thực (pen-down)

## Đổi chữ cái

Sửa file `ur3e_letter_writer/letter_writer_node.py`, thay giá trị `self._letter`:

```python
self._letter = 'T'  # đổi sang chữ T (hoặc B, L, V, M, N, A)
```

Rebuild: `colcon build --packages-select ur3e_letter_writer`

## Cấu trúc package

```
ur3e_letter_writer/
├── launch/
│   └── letter_writer.launch.py      # Launch file chính
├── ur3e_letter_writer/
│   ├── letter_paths.py              # Định nghĩa hình dạng chữ cái 
│   └── letter_writer_node.py        # Node điều khiển, giao tiếp MoveIt 2
├── config/
│   └── ur_controllers.yaml          # Controller config (nới lỏng tolerance cho WSL2)
├── package.xml
├── setup.py
└── README.md
```

## Thông số kỹ thuật

| Thông số | Giá trị |
|----------|---------|
| Kích thước chữ | 12 cm × 16 cm |
| Mặt phẳng vẽ | Nằm ngang, z = 0.06 m |
| Độ cao nâng bút | z = 0.11 m |
| Hướng tool0 | Chỉ thẳng xuống, quaternion (1, 0, 0, 0) |
| Cartesian max_step | 0.005 m (nội suy mỗi 5 mm) |
| Velocity scaling | 0.05 |
| Acceleration scaling | 0.15 |
