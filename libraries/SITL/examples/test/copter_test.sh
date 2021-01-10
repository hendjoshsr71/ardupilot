#!/bin/bash

# three copter swarm 

# assume we start the script from the root directory
ROOTDIR=$PWD
COPTER=$ROOTDIR/build/sitl/bin/arducopter

[ -x "$COPTER" ] || {
    ./waf configure --board sitl
    ./waf copter
}

# setup for either TCP or multicast
UARTA="mcast:"

COPTER_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/copter.parm"

mkdir -p swarm/copter1 swarm/copter2 swarm/copter3
(cd swarm/copter1 && $COPTER --model quad --home -35.3632627,149.1652382,584.090026855469,0 --uartA $UARTA --defaults $COPTER_DEFAULTS) &

# create default parameter file for the follower
cat <<EOF > swarm/copter2/follow.parm
SYSID_THISMAV 2
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID 1
FOLL_DIST_MAX 1000
EOF

(cd swarm/copter2 && $COPTER --model quad --home -35.3632642675716,149.165388867259,584.872916847261,0 --uartA $UARTA --instance 1 --defaults $COPTER_DEFAULTS,follow.parm) &

# create default parameter file for the follower 2
cat <<EOF > swarm/copter3/follow.parm
SYSID_THISMAV 3
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID 1
FOLL_DIST_MAX 1000
EOF

(cd swarm/copter3 && $COPTER --model quad --home -35.3632916093029,149.164973795414,584.926584510997,0 --uartA $UARTA --instance 2 --defaults $COPTER_DEFAULTS,follow.parm) &
wait
