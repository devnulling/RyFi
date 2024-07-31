#include "usrp.h"
#include <stdexcept>
#include "flog/flog.h"

#define BUFFER_SIZE 8192*2

USRP::USRP(dsp::stream<dsp::complex_t>* in, double samplerate, double rxFrequency, double txFrequency) {
    // Save the configuration
    this->in = in;
    this->samplerate = samplerate;
    this->rxFrequency = rxFrequency;
    this->txFrequency = txFrequency;
}

USRP::~USRP() {
    // Stop the device
    stop();
}

void USRP::start() {
    // If already running, do nothing
    if (running) { return; }

    // Create and configure the USRP device
    uhd::device_addr_t dev_addr("type=b200");
    dev = uhd::usrp::multi_usrp::make(dev_addr);

    // Set RX parameters
    dev->set_rx_rate(samplerate);
    dev->set_rx_freq(rxFrequency);
    dev->set_rx_bandwidth(samplerate);
    dev->set_rx_gain(40);

    // Set TX parameters
    dev->set_tx_rate(samplerate);
    dev->set_tx_freq(txFrequency);
    dev->set_tx_bandwidth(samplerate);
    dev->set_tx_gain(40);

    // Start the workers
    rxWorkerThread = std::thread(&USRP::rxWorker, this);
    txWorkerThread = std::thread(&USRP::txWorker, this);

    // Mark as running
    running = true;
}

void USRP::stop() {
    // If not running, do nothing
    if (!running) { return; }

    // Stop the workers
    in->stopReader();
    out.stopWriter();
    if (rxWorkerThread.joinable()) { rxWorkerThread.join(); }
    if (txWorkerThread.joinable()) { txWorkerThread.join(); }
    in->clearReadStop();
    out.clearWriteStop();

    // Mark as not running
    running = false;
}

void USRP::rxWorker() {
    int sampleCount = samplerate / 200;

    // Allocate the sample buffers
    int16_t* samps = dsp::buffer::alloc<int16_t>(sampleCount*2);

    // Create a RX streamer
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::rx_streamer::sptr rx_stream = dev->get_rx_stream(stream_args);

    // Setup the streamer
    uhd::rx_metadata_t md;
    std::vector<void*> buffs;
    buffs.push_back(samps);
    rx_stream->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    while (running) {
        // Receive the samples from the device
        size_t num_rx_samps = rx_stream->recv(buffs, sampleCount, md, 3.5);

        // Convert the samples to complex float
        volk_16i_s32f_convert_32f((float*)out.writeBuf, samps, 2048.0f, num_rx_samps*2);

        // Send off the samples
        if (!out.swap(num_rx_samps)) { break; }
    }

    // Stop the RX streamer
    rx_stream->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);

    // Free the sample buffer
    delete[] samps;
}

void USRP::txWorker() {
    // Allocate the sample buffers with proper size check
    int bufferSize = STREAM_BUFFER_SIZE * 2;
    int16_t* samps = dsp::buffer::alloc<int16_t>(bufferSize);
    if (!samps) {
        throw std::runtime_error("Failed to allocate sample buffer");
    }

    // Create a TX streamer
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::tx_streamer::sptr tx_stream = dev->get_tx_stream(stream_args);

    // Initialize metadata
    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    while (running) {
        // Get the transmitter samples
        int count = in->read();
        if (count <= 0) { break; }

        // Ensure count does not exceed buffer size
        if (count * 2 > bufferSize) {
            throw std::runtime_error("Buffer overflow detected");
        }

        // Convert the samples to 16bit PCM
        volk_32f_s32f_convert_16i(samps, (float*)in->readBuf, 2048.0f, count * 2);

        // Flush the modulator stream
        in->flush();

        // Send the samples to the device
        std::vector<void*> buffs;
        buffs.push_back(samps);
        tx_stream->send(buffs, count, md);
    }

    // Mark end of burst
    md.end_of_burst = true;
    tx_stream->send("", 0, md);

    // Free the sample buffer
    delete[] samps;
}
