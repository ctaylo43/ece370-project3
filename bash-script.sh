#!/bin/bash

INSTALL_DIR=~/ece-370/mobileRobot
PROGRAM=pi_controller.py

DoInstall()
{
	sudo mkdir -p $INSTALL_DIR
	sudo cp $PROGRAM $INSTALL_DIR
	sudo pip install getkey
}

DoUninstall()
{
	sudo rm -rf $INSTALL_DIR
}

DoStart()
{
	python $INSTALL_DIR/$PROGRAM
}

DoStop()
{
	pkill -f $PROGRAM
}

ShowUsage()
{
	echo '-----------------------'
	echo '-----------------------'
	echo '------Caleb Taylor-----'
	echo '---------Robot---------'
	echo '-----------------------'
	echo
	echo
	echo 'Install : installs pi_controller program'
	echo 'Uninstall : uninstalls program'
	echo 'Start : start program'
	echo 'Stop : stop program'
}