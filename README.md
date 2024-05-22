# Docker images to use with ros2 humble + VSCodes devcontainers

Pull images:

    ghcr.io/robinheitz/ros:<tag>

Available tags:

- humble
- noetic

## Setup

In your project folder:

    mkdir -p ros_ws/src
    mkdir .devcontainer

Copy the content into .devcontainer/devcontainer.json:

```
{
    "name": "ROS 2 Development Container",
    "privileged": true,
    "remoteUser": "ros",
    "containerUser": "ros",
    "image":"ghcr.io/robinheitz/ros:humble",
    "workspaceFolder": "/home/ros/ros_ws",
    "workspaceMount": "source=${localWorkspaceFolder}/ros_ws/src,target=/home/ros/ros_ws/src,type=bind",
    "containerEnv": {
        "DISPLAY": "unix:0",
        "ROS_LOCALHOST_ONLY": "1",
        "ROS_DOMAIN_ID": "42"
    },
    "runArgs": [
        "--net=host",
        "-e", "DISPLAY=${env:DISPLAY}"
    ]
}
```

Make sure to get the correct tag at `image`.

## Start container

If vscode doesn't asks for it, CTRL+SHIFT+P, type `devcontainer` and select `Open Folder in Container`.
