# ibeo scala lidar ros driver node

## About ibeo_scala
Build dependence:

[IbeoSDK](http://www.ibeo-as.com/)

You need a ibeo account to download the sdk.

Read HOWTO.txt in ibeosdk folder to find out how to build ibeosdk.

Notice that the network protocol of Scala B2 and B3 are different:

for B2 use 'ibeosdk::IbeoTypeEthTcp()'.

for B3 use 'ibeosdk::IbeoTypeEthUdp()'.
