#!/bin/bash
echo "movj 30 0" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 30 45" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 30 90" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 30 135" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 30 180" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 30 225" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 30 270" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 30 315" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 30 0" |  yarp rpc /SoftNeckControl/rpc_transform:s
sleep 12
echo "movj 0 0" |  yarp rpc /SoftNeckControl/rpc_transform:s
