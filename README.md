# lunadev-2024

This is the official workspace for the software team for Utah Student Robotics for the NASA Lunabotics 2024 competition.

## Where to start

You will need Visual Studio Code (We'll call it VSCode).

Lunaserver is the computer that will run on the robot on competition day, but until then is just a computer that is on 24/7.
It will be connected to all the sensors we will use, and maybe a microcontroller for you to test stuff on. You will use a technique called SSH to connect to Lunaserver. There is a dedicated extension in VScode for this that you should use. [Here is a guide](https://code.visualstudio.com/docs/remote/ssh#_connect-to-a-remote-host).

The address is `5.tcp.ngrok.io` and the port is `22735`. The SSH Fingerprint is `AAAAE2VjZHNhLXNoYTItbmlzdHAyNTYAAAAIbmlzdHAyNTYAAABBBMDjVvCSpLtGiopbc79AwfkZNcOkTsvVq/TiEGJEAvw9MXH8pJOL4kIHGusbJ3a8KTNa8uR7Y9Nb5K6yheM5pI4=`. When you connect to Lunaserver for the first time, VSCode will show you the fingerprint it received from what it thinks is Lunaserver. You should verify that this fingerprint is the same as that one. If they are different, disconnect and double check that you have written the address and port correctly. If they are correct, then it is likely that someone is trying to [piggy back](https://en.wikipedia.org/wiki/Man-in-the-middle_attack) off of your SSH connection for whatever reason. The chances of that happening is very low, but still worth checking. After the first connection, you will not need to verify the SSH fingerprint anymore as VSCode will check it for you. Very rarely, you may face a warning that the SSH fingerprint check has failed, which usually means that someone is once again trying to use your SSH connection. Simply let me know, and don't worry as your computer is not compromised. 

Before connecting for your first time, provide me with your preferred username and password for me to set up an account on Lunaserver for you. There is a guest account, username is `usr`. This account does not have any access outside of its home directory so you shouldn't use it to do your work.

*By connecting to Lunaserver, you are agreeing to the terms and conditions. [Refer to the wiki for the terms and conditions](https://github.com/utahrobotics/lunadev-2024/wiki/Terms-and-Conditions).*

After connecting, VSCode may ask you to type in your password very frequently. Since Lunaserver is exposed to the internet, I do want to enforce some cybersecurity. As such, you will have strong passwords that should not be convenient to type frequently. Thus, you should use SSH keys. Here is a [guide for how you can set that up](https://www.digitalocean.com/community/tutorials/how-to-configure-ssh-key-based-authentication-on-a-linux-server). You must already have an account to do this. Refer back to the first guide on SSH in VSCode to see how to add this key to your SSH config file. Do note that using SSH keys does not eliminate the need for a password; Lunaserver may still ask you to provide a password occasionally, but less often.

If you use the VSCode SSH extension, you can then follow the first set of steps with DevContainers just like normal. Only this time, your Dev Container will have access to all the sensors we will use.

## VNC

VNC is a very useful feature that allows Lunadev to share its screen with you so that you can run regular applications, such as RViz2. [Refer to the guide here.](https://github.com/utahrobotics/lunadev-2024/tree/main/lunadev)

# DevContainers
`git clone` this repository and open it with VSCode. VSCode will then ask if you want to install the recommended extensions. In case that does not happen or you missed it, you only need 1 extension: DevContainers.
This [guide to installing DevContainers](https://code.visualstudio.com/docs/devcontainers/tutorial) will also help you with Docker. Remember, you install this through VSCode.

Assuming that the extension is installed, you will see a notification at the bottom right of the screen asking if you would like to open the repository in the container.
If you missed that, refer back to the guide for DevContainers to see how to do that. Now, you can start developing locally. To make the development process easier for you guys, I configured our code to run with privileged access.
This means that any code you run here has the possiblity to mess up the computer it is running on. As such, *please do not run code from untrusted sources*. As long as you don't copy code from sketchy websites, you should be good.

If you do not want to use Lunaserver for whatever reason, such as experimenting with your own hardware, you will most likely need to download Docker Desktop (if you are on Linux, you only need to install Docker). After running it, you can just follow the above instructions on your own computer. Fair warning, connecting to USB devices from inside a DevContainer running on a Windows computer can be quite a pain. [Here is a guide for that](https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/). Your best bet is to just use Lunaserver.

## Troubleshooting DevContainers

If you can't open this repository using DevContainers on your own computer, I can really only offer two pieces advice without seeing the error logs:

1. Check that Docker Desktop is running
2. Check that you are connected to the internet

If you did both, send me a message and we'll work it through.

## Wiki

There is a [wiki](https://github.com/utahrobotics/lunadev-2024/wiki) for this repository that contains lots of useful information. Please read through it if you want to contribute effectively.
