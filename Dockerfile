# Copyright (C) 2022  Christian Berger
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

## Part to generate /tmp/peak-can.hpp.
#FROM chalmersrevere/dbc2odvd-amd64:v0.0.6 as generator
#MAINTAINER Christian Berger "christian.berger@gu.se"
#
#ADD . /opt/sources
#WORKDIR /opt/sources
#RUN mkdir build && \
#    cd build && \
#    generateHeaderOnly.sh ../src/peak-can.dbc ../src/peak-can.odvd && \
#    cp peak_can.hpp /tmp


# Part to build opendlv-device-gps-peak.
FROM alpine:3.15 as builder
MAINTAINER Christian Berger "christian.berger@gu.se"

RUN apk update && \
    apk --no-cache add \
        cmake \
        g++ \
        linux-headers \
        make

ADD . /opt/sources
WORKDIR /opt/sources

RUN mkdir -p /opt/sources/build
#COPY --from=generator /tmp/peak_can.hpp /opt/sources/build

RUN cd /opt/sources/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/opendlv-device-gps-peak-dest .. && \
    make && make install


# Part to deploy opendlv-device-gps-peak.
FROM alpine:3.15
MAINTAINER Christian Berger "christian.berger@gu.se"

WORKDIR /usr/bin
COPY --from=builder /tmp/opendlv-device-gps-peak-dest/bin/opendlv-device-gps-peak .
ENTRYPOINT ["/usr/bin/opendlv-device-gps-peak"]

