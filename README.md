# Docker images to use with ros2 humble + VSCodes devcontainers

See other branches for different ros versions (like noetic)

## Setup

In your project's root directory, create a file `.devcontainer/devcontainer.json` and add the following content to it:

```
{
	"name": "ros2-humble",
	"privileged": true,
    "image": "ghcr.io/robinheitz/ros-humble:ur",
	"workspaceFolder": "/ros_stuff",
	"runArgs": [
		"--name",
		"ros2-humble-dev-container",
		"--net=host",
        "--ipc=host",
        "-e", "DISPLAY=${env:DISPLAY}"
	],
	"remoteUser": "ros",
    "customizations": {
        "vscode": {
            "extensions":[
                "ms-vscode.cpptools",
                "ms-vscode.cpptools-themes",
                "twxs.cmake",
                "donjayamanne.python-extension-pack",
                "eamodio.gitlens",
                "ms-iot.vscode-ros"
            ]
        }
    },
	"containerEnv": {
        "DISPLAY": "unix:0",
        "ROS_LOCALHOST_ONLY": "1",
        "ROS_DOMAIN_ID": "42"
    }
}
```

You can adjust according to your needs, e.g. in the vscode extensions list.

## Start container

If vscode doesn't asks for it, CTRL+SHIFT+P, type `devcontainer` and select `Open Folder in Container`.
