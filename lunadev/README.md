# Lunadev

This is where the code to generate the `luna` image resides. `luna` is automatically built and gets pulled by anyone starting up Lunadev. If you wanted to find the best way to mess with literally everyone's Lunadev instance, here it is. This is mainly maintained by Najman Husaini.

## VNC

VNC is a remote desktop protocol. Basically, it allows Lunadev to share its screen to you so that you can run applications like RViz2 which visualize a lot of data. This is very important for developing our autonomous actions. To start using VNC, you first need to [get VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/). Then, you need to download the `client_tunnel.py` script in this folder. You only need to download that file and nothing else. Now, check that you have added public key authentication for your SSH account (refer to the README at the root of this repository). Finallly, inside the DevContainer on Lunaserver, run `startvnc`. It will print out a one-time password that you should copy.

Now, run the following command on your own computer:

`python3 client_tunnel.py YOUR_USERNAME LUNASERVER_ADDR LUNASERVER_PORT YOUR_PRIVATE_KEY`

Replace `YOUR_USERNAME` with your username on Lunaserver, `LUNASERVER_ADDR` and `LUNASERVER_PORT` with Lunaserver's current address and port, and `YOUR_PRIVATE_KEY` with the path to your private key on your computer (On windows, your keys are usually in `C:\Users\YOUR_NAME\.ssh`). An example of a proper command is as follows:

`python3 client_tunnel.py naj 0.tcp.us-cal-1.ngrok.io 11800 C:\Users\Najman\.ssh\lunaserver_private_key`

Keep in mind that the address and port written here may not be up to date.

If the command runs, you should see the welcome message from Lunaserver. You don't need to touch that anymore now, but do not close that window until you are done.

Finally, open RealVNC Viewer and connect to `127.0.0.1:5901`. Ignore the warning about the connection not being encrypted (it is), and paste the one-time password you got from Lunaserver. Do not check the 'remember password' option, as the password will change everytime. You should now see the screen running inside of Lunadev. It will still only be a terminal, but if you run `rviz2` inside of this terminal, you will then see the RViz2 window start up. You can now use RViz2 like normal.
