# connect to husky
1. connect to husky wifi
2. ssh iaac@10.42.0.1
3. tmux new -s husky

# split the terminal 
crtl + b then %
terminal 1:
ros2 launch /etc/clearpath/platform/launch/platform-service.launch.py 
