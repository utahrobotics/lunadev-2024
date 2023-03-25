# source foxy installation
source /opt/ros/foxy/setup.bash
# source install
alias srcinstall='source install/setup.bash'
# resolve deps, build, then source overlay
alias colcbuild='rosdep install --from-paths src -y --ignore-src && colcon build --symlink-install && source install/setup.bash'
# test with output printed to console
alias colctest='colcon test --return-code-on-test-failure --event-handlers console_cohesion+'
# list dev paths for usb devices
# alias findusb='/root/findusbdev.sh'
# delete all generated files in the current directory
alias colcclean='
if [ "$PWD" = "/usr-ws-2023" ]; then
    rm -rf build install log
else
    echo "Not in usr-ws-2023"
fi'
# Run rviz
alias rviz='ros2 run rviz2 rviz2'
