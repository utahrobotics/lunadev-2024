# lunadev-2024

This is the official workspace for the software team for Utah Student Robotics for the NASA Lunabotics 2024 competition.

## Where to start

You will need these two programs:

1. Visual Studio Code (We'll call it VSCode)
2. Docker Desktop (if you are on Linux, you only need to install Docker)

Then, git clone this repository and open it with VSCode. VSCode will then ask if you want to install the recommended extensions. In case that does not happen or you missed it, you only need 1 extension: DevContainers.
This [guide to installing DevContainers](https://code.visualstudio.com/docs/devcontainers/tutorial) will also help you with Docker. Remember, you install this through VSCode.

Assuming that the extension is installed, you will see a notification at the bottom right of the screen asking if you would like to open the repository in the container.
If you missed that, refer back to the guide for DevContainers to see how to do that. Now, you can start developing locally. To make the development process easier for you guys, I configured our code to run with privileged access.
This means that any code you run here has the possiblity to mess up your computer. As such, *please do not run code from untrusted sources*. As long as you don't copy code from sketchy websites, you should be good.

## Troubleshooting DevContainers

If you can't open this repository using DevContainers, I can really only offer two pieces advice without seeing the error logs:

1. Check that Docker Desktop is running
2. Check that you are connected to the internet

If you did both, send me a message and we'll work it through.

## Lunaserver

Another way to work which I think is better for you guys is to connect to Lunaserver. Lunaserver is the computer that will run on the robot on competition day, but until then is just a computer that is on 24/7.
It will be connected to all the sensors we will use, and maybe a microcontroller for you to test stuff on. You will use a technique called SSH to connect to Lunaserver. There is a dedicated extension in VScode for this that you should use. [Here is a guide](https://code.visualstudio.com/docs/remote/ssh#_connect-to-a-remote-host).

Before connecting for your first time, provide me with your preferred username and password/public-key for me to set up an account on Lunaserver for you.

If you use the VSCode SSH extension, you can then follow the first set of steps with DevContainers just like normal. Only this time, your Dev Container will have access to all the sensors we will use.

## Note about Lunaserver

Obviously, you will notice that there isn't an IP address for Lunaserver, and that is because we don't have it set up yet. The instructions are here because I am writing them in advance. However, you can still
provide me with your preferred username and credentials right now and I will set them up when it is ready.

## Wiki

There is a [wiki](https://github.com/utahrobotics/lunadev-2024/wiki) for this repository that contains lots of useful information. Please read through it if you want to contribute effectively.
