# SW 요구 사항

Ubuntu 22.04 혹은 WSL 2(Ubuntu 22.04)

# ROS 2 Humble 설치

https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html

위 링크 참고해서 ROS 2 Humble을 설치해주세요.

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

해당 링크를 참고하면 ROS 2 Humble workspace를 구성할 수 있습니다. Workspace까지 구성 후, TurtleSim 또는 F1Tenth 공식 문서의 Lab 1을 통해 환경 구성이 잘 되었는지 확인해주세요.

Lab 1은 아래 링크에서 찾을 수 있습니다.

https://f1tenth.org/learn.html

# 시뮬레이션 구성

https://github.com/f1tenth/f1tenth_gym_ros

위 GitHub 레포지토리 참고해서 시뮬레이션 구성(Docker, with an NVIDIA GPU 부분은 생략)

레포지토리에선 Ubuntu 20.04 및 ROS 2 Foxy를 기준으로 설명하고 있으나, Ubuntu 22.04 및 ROS 2 Humble로도 정상적으로 작동합니다. source 명령어는 foxy를 humble로 변경 후 사용해주세요.

예시입니다.

```bash
source /opt/ros/humble/setup.bash
```

# 이후 구성 사항

이 레포지토리의 wall_follow_pkg 폴더를 workspace/src 폴더로 옮겨주시고, wall_follow_pkg/wall_follow_pkg/wall_follow_node.py 파일을 수정해주시면 됩니다.
