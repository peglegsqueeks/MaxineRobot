#!/bin/bash
# Adapted from 71529-ubucleaner.sh - https://web.archive.org/web/20151209182520/http://opendesktop.org/CONTENT/content-files/71529-ubucleaner.sh

OLDCONF=$(dpkg -l|grep "^rc"|awk '{print $2}')
CURKERNEL=$(uname -r|sed 's/-*[a-z]//g'|sed 's/-386//g')
LINUXPKG="linux-(image|headers|ubuntu-modules|restricted-modules)"
METALINUXPKG="linux-(image|headers|restricted-modules)-(generic|i386|server|common|rt|xen)"
OLDKERNELS=$(dpkg -l|awk '{print $2}'|grep -E $LINUXPKG |grep -vE $METALINUXPKG|grep -v $CURKERNEL)
YELLOW="\033[1;33m"; RED="\033[0;31m"; ENDCOLOR="\033[0m"

if [ $USER != root ]; then
  echo -e $RED"Error: must be root! Exiting..."$ENDCOLOR
  exit 0
fi

echo -e $YELLOW"Cleaning apt ..."$ENDCOLOR
aptitude clean
apt-get autoremove
apt-get autoclean

echo -e $YELLOW"Those packages were uninstalled without --purge:"$ENDCOLOR
echo $OLDCONF
#apt-get purge "$OLDCONF"  # fixes the error in the original script
for PKGNAME in $OLDCONF ; do  # a better way to handle errors
  echo -e $YELLOW"Purge package $PKGNAME"
  apt-cache show "$PKGNAME"|grep Description: -A3
  apt-get -y purge "$PKGNAME"
done

echo -e $YELLOW"Removing old kernels..."$ENDCOLOR
echo current kernel you are using:
uname -a
aptitude purge $OLDKERNELS