#!/bin/bash

# three plane swarm (no set formaiton)

# assume we start the script from the root directory
ROOTDIR=$PWD
PLANE=$ROOTDIR/build/sitl/bin/arduplane

[ -x "$PLANE" ] || {
    ./waf configure --board sitl
    ./waf plane
}

# setup for either TCP or multicast
UARTA="mcast:"

PLANE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/plane.parm"

mkdir -p swarm/plane1 swarm/plane2 swarm/plane3
(cd swarm/plane1 && $PLANE --model plane --home -35.3632627,149.1652382,584.090026855469,0 --uartA $UARTA --defaults $PLANE_DEFAULTS) &

# create default parameter file for the follower
cat <<EOF > swarm/plane2/follow.parm
SYSID_THISMAV 2
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID 1
FOLL_DIST_MAX 1000
EOF

(cd swarm/plane2 && $PLANE --model plane --home -35.3632627,149.165388867259,584.872916847261,0 --uartA $UARTA --instance 1 --defaults $PLANE_DEFAULTS,follow.parm) &

# create default parameter file for the follower 2
cat <<EOF > swarm/plane3/follow.parm
SYSID_THISMAV 3
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID 1
FOLL_DIST_MAX 1000
EOF

(cd swarm/plane3 && $PLANE --model plane --home -35.3632627,149.164973795414,584.926584510997,0 --uartA $UARTA --instance 2 --defaults $PLANE_DEFAULTS,follow.parm) &
wait
