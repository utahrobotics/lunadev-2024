# source humble installation
source /opt/ros/humble/setup.bash
# source install
alias srcinstall='source /root/lunadev-2024/install/setup.bash'
# resolve deps, build, then source overlay
alias colcbuild='rosdep install --from-paths src -y --ignore-src && colcon build --symlink-install && source install/setup.bash'
# test with output printed to console
alias colctest='colcon test --return-code-on-test-failure --event-handlers console_cohesion+'
# delete all generated files in the current directory
alias colcclean='
if [ "$PWD" = "/usr-ws-2023" ]; then
    rm -rf build install log
else
    echo "Not in usr-ws-2023"
fi'
# Run rviz
alias rviz='ros2 run rviz2 rviz2'
# Try srcinstall
if [ -f /root/lunadev-2024/install/setup.bash ]; then
    source /root/lunadev-2024/install/setup.bash
fi
