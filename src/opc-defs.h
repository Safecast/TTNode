// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

//  Alphasense OPC-N2 Particle Counter SPI interface definitions    
//
//  Derived from library created by David H Hagan, March 2016,
//  published on github.com/dhhagan, and released under MIT license

#ifndef OPCDEFS_H
#define OPCDEFS_H

struct Status {
    int fanON;
    int laserON;
    int fanDAC;
    int laserDAC;
};

struct Firmware {
    int major;
    int minor;
};

struct HistogramData {
    int bin0;
    int bin1;
    int bin2;
    int bin3;
    int bin4;
    int bin5;
    int bin6;
    int bin7;
    int bin8;
    int bin9;
    int bin10;
    int bin11;
    int bin12;
    int bin13;
    int bin14;
    int bin15;

    // Mass Time-of-Flight
    float bin1MToF;
    float bin3MToF;
    float bin5MToF;
    float bin7MToF;

    // Sample Flow Rate
    float sfr;

    // Either the Temperature or Pressure
    unsigned long temp_pressure;

    // Sampling Period
    float period;

    // Checksum
    unsigned int checksum;

    float pm1;
    float pm25;
    float pm10;
};

struct PMData {
  float pm1;
  float pm25;
  float pm10;
};

struct ConfigVars {
    // Bin Boundaries
    int bb0;
    int bb1;
    int bb2;
    int bb3;
    int bb4;
    int bb5;
    int bb6;
    int bb7;
    int bb8;
    int bb9;
    int bb10;
    int bb11;
    int bb12;
    int bb13;
    int bb14;

    // Bin Particle Volume (floats)
    float bpv0;
    float bpv1;
    float bpv2;
    float bpv3;
    float bpv4;
    float bpv5;
    float bpv6;
    float bpv7;
    float bpv8;
    float bpv9;
    float bpv10;
    float bpv11;
    float bpv12;
    float bpv13;
    float bpv14;
    float bpv15;

    // Bin Particle Densities (floats)
    float bpd0;
    float bpd1;
    float bpd2;
    float bpd3;
    float bpd4;
    float bpd5;
    float bpd6;
    float bpd7;
    float bpd8;
    float bpd9;
    float bpd10;
    float bpd11;
    float bpd12;
    float bpd13;
    float bpd14;
    float bpd15;

    // Bin Sample Volume Weightings (floats)
    float bsvw0;
    float bsvw1;
    float bsvw2;
    float bsvw3;
    float bsvw4;
    float bsvw5;
    float bsvw6;
    float bsvw7;
    float bsvw8;
    float bsvw9;
    float bsvw10;
    float bsvw11;
    float bsvw12;
    float bsvw13;
    float bsvw14;
    float bsvw15;

    // Gain Scaling Coefficient
    float gsc;

    // Sample Flow Rate (ml/s)
    float sfr;

    // LaserDAC 8 bit int
    unsigned int laser_dac;
    unsigned int fan_dac;

    // Time of Flight to Sample Flow Rate Conversion Factor
    unsigned int tof_sfr;
};

struct ConfigVars2 {
  int AMSamplingInterval;
  int AMIntervalCount;
  int AMFanOnIdle;
  int AMLaserOnIdle;
  int AMMaxDataArraysInFile;
  int AMOnlySavePMData;
};

#endif // OPCDEFS_H
