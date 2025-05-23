#!/bin/bash -xe

git clone https://github.com/mguentner/cannelloni && cd cannelloni
cmake -DCMAKE_BUILD_TYPE=Release && make && sudo make install
cd .. && rm -rf ./cannelloni