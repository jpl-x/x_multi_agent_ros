#! /bin/bash
# fanciness
echo
echo
echo "        __  __  ______   _____      ";
echo "       /\ \/\ \/\__  _\ /\  __\`\    ";
echo -e "\e[1m\e[34m __  _ \e[39m\e[0m\ \ \ \ \/_/\ \/ \ \ \/\ \   ";
echo -e "\e[1m\e[34m/\ \/'\ \e[39m\e[0m\\ \ \ \ \ \ \ \  \ \ \ \ \  ";
echo -e "\e[1m\e[34m\/>  </ \e[39m\e[0m \ \ \_/ \ \_\ \__\ \ \_\ \ ";
echo -e "\e[1m\e[34m /\_/\_\ \e[39m\e[0m \ \`\___/ /\_____\\\\\ \_____\\";
echo -e "\e[1m\e[34m \//\/_/ \e[39m\e[0m  \`\/__/  \/_____/ \/_____/";
echo
echo
# This script loads and stops the xVIO GUI.
# Load GUI
rqt --clear-config --perspective-file $(rospack find x_vio_ros)/cfg/rqt_duo.perspective &
rviz -d $(rospack find x_vio_ros)/cfg/config_duo.rviz &
sleep 1 # Ensure user prompt below is the last thing to show up on screen

# Wait for user to press q to kill the script
while [ -z $key_pressed ] || [ $key_pressed != 'q' ]; do
read -p 'Press q to kill the GUI: ' key_pressed
done

# Kill GUI
killall -9 rqt
killall -9 rviz
