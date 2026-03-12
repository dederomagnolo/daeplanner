cd
source .bashrc
config=$1
seed=${2:-}

if [[ -n "$seed" && "$seed" =~ ^-?[0-9]+$ ]]; then
  roslaunch rpl_exploration exploration.launch config_file:=$config experiment_seed:=$seed
else
  roslaunch rpl_exploration exploration.launch config_file:=$config
fi
