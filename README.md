# BetaPilot 4.0 for Copter, BetaCopter

This is the BetaPilot fork of the ArduPilot project, for Copter.

I use it for developing new features, mainly related to the [STorM32 gimbal controller](http://www.olliw.eu/2013/storm32bgc/) project.

## Usage and Release Notes ##

Please see http://www.olliw.eu/storm32bgc-wiki/Using_STorM32_with_ArduPilot for information on the usage and features of BetaPilot/BetaCopter.


## Build, Compile Notes ##

In order to build BetaPilot 4.0, do this:

- Follow the instructions in the ArduPilot wiki for installing the build environment.
- Clone this fork and checkout the BetaCopter branch.
- Ensure that all git submodules are there. Run git submodule update --init -- recursive.
- Copy the file \mavlink_withmostbasicadditions\common.xml to the folder ardupilot\modules\mavlink\message_definitions\v1.0. This will overwrite the file existing there. (This step is crucuial since ArduPilot doesn't support the latest MAVLink standard).


## Acknowledgements and License ##

BetaPilot is based on the ArduPilot project, see: 

- ArduPilot home: http://ardupilot.com/ardupilot/index.html

- ArduPilot github repository: https://github.com/ArduPilot/ardupilot

BetaPilot inherits the ArduPilot licence(s). ArduPilot is licensed under GNU GPL version 3, see:

- ArduPilot license, overview: http://ardupilot.org/dev/docs/license-gplv3.html

- Full text of license: https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt
