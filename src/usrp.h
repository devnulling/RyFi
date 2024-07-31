#pragma once
#include <thread>
#include <uhd/usrp/multi_usrp.hpp>
#include "dsp/stream.h"
#include "dsp/types.h"
#include <stddef.h>

class USRP {
public:
    USRP(dsp::stream<dsp::complex_t>* in, double samplerate, double rxFrequency, double txFrequency);

    ~USRP();

    void start();

    void stop();

    dsp::stream<dsp::complex_t> out;

private:
    void rxWorker();
    void txWorker();

    // Configuration
    dsp::stream<dsp::complex_t>* in;
    double samplerate;
    double rxFrequency;
    double txFrequency;

    // Workers
    std::thread rxWorkerThread;
    std::thread txWorkerThread;

    // Device
    uhd::usrp::multi_usrp::sptr dev;

    // Status
    bool running = false;
};
