# SW 요구 사항

Ubuntu 22.04 혹은 WSL 2(Ubuntu 22.04)

# ROS 2 Humble 설치

https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html

위 링크 참고해서 ROS 2 Humble 및 Workspace 구성

# 시뮬레이션 구성

https://github.com/f1tenth/f1tenth_gym_ros

위 GitHub 레포지토리 참고해서 시뮬레이션 구성(Docker, with an NVIDIA GPU 부분은 생략)

레포지토리에선 Ubuntu 20.04 및 ROS 2 Foxy를 기준으로 설명하고 있으나, Ubuntu 22.04 및 ROS 2 Humble로도 정상적으로 작동합니다. source 명령어는 foxy를 humble로 변경 후 사용해주세요.

예시입니다.

```bash
source /opt/ros/humble/setup.bash
```
