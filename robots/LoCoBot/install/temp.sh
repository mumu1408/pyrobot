script_abs=$(readlink -f "$0")
script_dir=$(dirname $script_abs)

PROJECT_FOLDER=$(dirname $(dirname $(dirname $(dirname $(dirname $script_dir)))))
echo $PROJECT_FOLDER

INSTALL_TYPE="full"

if [ $INSTALL_TYPE == "full" ]; then
	# STEP 7 - Dependencies and config for calibration
	cd $PROJECT_FOLDER
	chmod +x src/pyrobot/robots/LoCoBot/locobot_navigation/orb_slam2_ros/scripts/gen_cfg.py
	rosrun orb_slam2_ros gen_cfg.py
	HIDDEN_FOLDER=~/.robot
	if [ ! -d "$HIDDEN_FOLDER" ]; then
		mkdir ~/.robot
		cp $PROJECT_FOLDER/src/pyrobot/robots/LoCoBot/locobot_calibration/config/default.json ~/.robot/
	fi
	
	# STEP 8 - Setup udev rules
	cd $PROJECT_FOLDER/src/pyrobot/robots/LoCoBot
	sudo cp udev_rules/*.rules /etc/udev/rules.d
	sudo service udev reload
	sudo service udev restart
	sudo udevadm trigger
	sudo usermod -a -G dialout $USER
fi


echo "Installation complete, took $elapsed seconds in total"
echo "NOTE: Remember to logout and login back again before using the robot!"
