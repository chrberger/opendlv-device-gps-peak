## OpenDLV Microservice to interface with PEAK GPS units

This repository provides source code to interface with a PEAK GPS unit
providing data over CAN for the OpenDLV software ecosystem. This
decoder extracts latitude, longitude, heading, and groundspeed.

[![Build Status](https://git.ita.chalmers.se/chalmers-revere/opendlv-device-gps-peak/badges/master/pipeline.svg)](https://git.ita.chalmers.se/chalmers-revere/opendlv-device-cangw-rhino/commits/master)
 [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg
)](https://www.gnu.org/licenses/gpl-3.0.txt)

* Provides: [Latitude/Longitude (OpenDLV Standard Message Set v0.9.7)](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L145-L148)
* Provides: [Altitude (OpenDLV Standard Message Set v0.9.7)](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L89-L91)
* Provides: [Heading (OpenDLV Standard Message Set v0.9.7)](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L141-L143)
* Provides: [GroundSpeed (OpenDLV Standard Message Set v0.9.7)](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L129-L131)
* Provides: [Temperature (OpenDLV Standard Message Set v0.9.7)](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L97-L99)
* Provides: [AccelerationReading (OpenDLV Standard Message Set v0.9.7)](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L71-L75)
* Provides: [AngularVelocityReading (OpenDLV Standard Message Set v0.9.7)](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L77-L81)
* Provides: [MagneticFieldReading (OpenDLV Standard Message Set v0.9.7)](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/bd5007e7723654563c388129a96a70b559f7fef6/opendlv.odvd#L83-L87)
* Provides: [device-specific messages](https://git.ita.chalmers.se/chalmers-revere/opendlv-device-gps-peak/raw/master/src/peak-can.odvd)


## Table of Contents
* [Dependencies](#dependencies)
* [Usage](#usage)
* [Build from sources on the example of Ubuntu 16.04 LTS](#build-from-sources-on-the-example-of-ubuntu-1604-lts)
* [License](#license)


## Dependencies
No dependencies! You just need a C++14-compliant compiler to compile this
project as it ships the following dependencies as part of the source distribution:

* [libcluon](https://github.com/chrberger/libcluon) - [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg
)](https://www.gnu.org/licenses/gpl-3.0.txt)
* [Unit Test Framework Catch2](https://github.com/catchorg/Catch2/releases/tag/v2.1.2) - [![License: Boost Software License v1.0](https://img.shields.io/badge/License-Boost%20v1-blue.svg)](http://www.boost.org/LICENSE_1_0.txt)


## Usage
* Command to run with Docker:
```
docker run --init --rm --privileged --net=host ghcr.io/chrberger/opendlv-device-gps-peak --can=can0 --cid=111 --verbose
```

* Section for `docker-compose.yml`:
```yml
version: '2' # Must be present exactly once at the beginning of the docker-compose.yml file
services:    # Must be present exactly once at the beginning of the docker-compose.yml file
    dev-gps-nmea:
        container_name: dev-gps-nmea
        image: ghcr.io/chrberger/opendlv-device-gps-peak:latest
        privileged: true
        restart: on-failure
        network_mode: "host"
        command: "--can=can0 --cid=111"
```


## Build from sources on the example of Ubuntu 16.04 LTS
To build this software, you need cmake, C++14 or newer, and make. Having these
preconditions, just run `cmake` and `make` as follows:

```
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make && make install
```


## License

* This project is released under the terms of the GNU GPLv3 License

