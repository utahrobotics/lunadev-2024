# source humble installation
source /opt/ros/humble/setup.bash
# source install
alias srcinstall='source /root/lunadev-2024/install/setup.bash'
# resolve deps, build, then source overlay
alias colcbuild='rosdep install --from-paths src -y --ignore-src && colcon build --symlink-install && source install/setup.bash'
# test with output printed to console
alias colctest='colcon test --return-code-on-test-failure --event-handlers console_cohesion+'
# delete all auto-generated files in lunadev
alias colcclean='rm -rf /root/lunadev-2024/build /root/lunadev-2024/install /root/lunadev-2024/log'
# Run rviz
alias rviz='ros2 run rviz2 rviz2'

# Start VNC
alias startvnc='python3 /root/lunadev-2024/lunadev/start_vnc.py'

# Start telemetry tunnel
alias starttele='python3 /root/lunadev-2024/lunadev/start_telemetry_tunnel.py'

# Try srcinstall
if [ -f /root/lunadev-2024/install/setup.bash ]; then
    source /root/lunadev-2024/install/setup.bash
fi
