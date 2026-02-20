T1
./dev_env.sh start daep
source .bashrc
./simulation.sh simple_forest dynamic "(0,0,0.5)" false true

T2
./dev_env.sh bash daep
source .bashrc
./exploration.sh simple_forest_exploration.yaml

T3
./dev_env.sh bash daep
source .bashrc
roslaunch tree_identifier tree_stack.launch target_frame:=world


roslaunch tree_identifier tree_stack.launch \
  target_frame:=world \
  fixed_axes:=true \
  x_min:=-25 x_max:=25 \
  y_min:=-28 y_max:=20 \
  tick_step:=5 \
  log_coords:=true \
  log_throttle_sec:=2.0