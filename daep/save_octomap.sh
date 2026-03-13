cd
source .bashrc
octomap_name=$1
if [[ -z "${octomap_name}" ]]; then
  echo "Usage: save_octomap.sh <output_name(.bt|.ot)>"
  exit 1
fi
if [[ "${octomap_name}" != *.bt && "${octomap_name}" != *.ot ]]; then
  octomap_name="${octomap_name}.bt"
fi
mkdir -p octomaps
rosrun octomap_server octomap_saver -f /home/daep/octomaps/$octomap_name octomap_full:=/aeplanner/octomap_full
