# ibeo scala lidar ros driver node

## Build
To build the package, you need to build [IbeoSDK](http://www.ibeo-as.com/) first.

Read HOWTO.txt in ibeosdk folder to find out how to build ibeosdk.

## Tips
Notice that the network protocol of Scala B2 and B3 are different:

for B2 use 'ibeosdk::IbeoTypeEthTcp()'.

for B3 use 'ibeosdk::IbeoTypeEthUdp()'.
