#!/bin/bash

# Stop on build error
#set -e

############################## set up the build processs ##############################
# do this so apt has a dns resolver
mkdir -p /run/systemd/resolve
echo 'nameserver 1.1.1.1' > /run/systemd/resolve/stub-resolv.conf
#######################################################################################

#################################### update the OS ####################################
apt-get update -y && apt-get upgrade -y
#######################################################################################

################################ update boot config ###################################

{
   echo ""
   echo '# Dennis Custom Image Build Test'
   echo ""
} >> /boot/config.txt