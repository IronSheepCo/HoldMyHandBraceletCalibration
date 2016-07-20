sed s:\$NRF_SDK:$NRF_SDK: < load.jlink > loadtmp.jlink
JLinkExe -device nRF51822 -speed 4000 -commanderscript loadtmp.jlink
rm loadtmp.jlink
