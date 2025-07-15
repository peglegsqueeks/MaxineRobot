#!/bin/bash

DUB=`pwd`

echo '=== Installing prerequisites ==='
sudo apt-get update
sudo apt-get install -y tix

echo '=== Make scripts executable ==='
chmod a+x *.py
chmod a+x *.sh

echo '=== Create a desktop shortcut for the GUI example ==='
UB_SHORTCUT="${HOME}/Desktop/UltraBorg.desktop"
echo "[Desktop Entry]" > ${UB_SHORTCUT}
echo "Encoding=UTF-8" >> ${UB_SHORTCUT}
echo "Version=1.0" >> ${UB_SHORTCUT}
echo "Type=Application" >> ${UB_SHORTCUT}
echo "Exec=${DUB}/ubGui.py" >> ${UB_SHORTCUT}
echo "Icon=${DUB}/piborg.ico" >> ${UB_SHORTCUT}
echo "Terminal=false" >> ${UB_SHORTCUT}
echo "Name=UltraBorg Demo GUI" >> ${UB_SHORTCUT}
echo "Comment=UltraBorg demonstration GUI" >> ${UB_SHORTCUT}
echo "Categories=Application;Development;" >> ${UB_SHORTCUT}

echo '=== Create a desktop shortcut for the tuning GUI ==='
UB_SHORTCUT="${HOME}/Desktop/UltraBorgTuning.desktop"
echo "[Desktop Entry]" > ${UB_SHORTCUT}
echo "Encoding=UTF-8" >> ${UB_SHORTCUT}
echo "Version=1.0" >> ${UB_SHORTCUT}
echo "Type=Application" >> ${UB_SHORTCUT}
echo "Exec=${DUB}/ubTuningGui.py" >> ${UB_SHORTCUT}
echo "Icon=${DUB}/piborg.ico" >> ${UB_SHORTCUT}
echo "Terminal=false" >> ${UB_SHORTCUT}
echo "Name=UltraBorg Tuning GUI" >> ${UB_SHORTCUT}
echo "Comment=UltraBorg Tuning GUI" >> ${UB_SHORTCUT}
echo "Categories=Application;Development;" >> ${UB_SHORTCUT}

echo '=== Finished ==='
echo ''
echo 'Your Raspberry Pi should now be setup for running UltraBorg'
echo 'Please restart your Raspberry Pi and ensure the I2C interface is enabled'
