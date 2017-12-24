#ifndef bladeRF_DEVICE_H
#define bladeRF_DEVICE_H

#include <libbladeRF.h>
#include "RadioDevice.h"
#include "SampleBuffer.h"

#define TX_AMPL          0.3

class bladeRFDevice : public RadioDevice {
public:
	bladeRFDevice(double rate);
	~bladeRFDevice();

	bool open(const std::string &args, bool extref);
	bool start();
	bool stop();
	bool restart();
	void setPriority();
	enum TxWindowType getWindowType() { return tx_window; }

	int readSamples(short *buf, int len, bool *overrun,
			long long timestamp, bool *underrun, unsigned *RSSI);

	int writeSamples(short *buf, int len,
			 bool *underrun, long long timestamp);

        bool setVCTCXO(unsigned int) { return true; };

	bool setTxFreq(double wFreq);
	bool setRxFreq(double wFreq);

	inline long long initialWriteTimestamp() { return ts_initial; }
	inline long long initialReadTimestamp() { return ts_initial; }

	inline double fullScaleInputValue() { return 32000 * TX_AMPL; }
	inline double fullScaleOutputValue() { return 32000; }

	double setRxGain(double db);
	double getRxGain(void) { return rx_gain; }
	double maxRxGain(void) { return rx_gain_max; }
	double minRxGain(void) { return rx_gain_min; }

	double setTxGain(double db);
	double maxTxGain(void) { return tx_gain_max; }
	double minTxGain(void) { return tx_gain_min; }

	double getTxFreq() { return tx_freq; }
	double getRxFreq() { return rx_freq; }

	inline double getSampleRate() { return tx_rate; }
	inline double numberRead() { return rx_pkt_cnt; }
	inline double numberWritten() { return 0; }

	enum err_code {
		ERROR_TIMING = -1,
		ERROR_UNRECOVERABLE = -2,
		ERROR_UNHANDLED = -3,
	};

private:
    struct bladerf *dev;
	enum TxWindowType tx_window;

	int sps;
	int tx_spp, rx_spp;

	double tx_gain, tx_gain_min, tx_gain_max;
	double rx_gain, rx_gain_min, rx_gain_max;

	double tx_rate, rx_rate;
	double tx_freq, rx_freq;

	bool started;
	bool aligned;

	bool firstGoodTXDone;

	size_t rx_pkt_cnt;
	size_t drop_cnt;
	long long prev_ts;

	long long ts_initial, ts_offset;
	SampleBuffer *rx_buffer;

	void init_gains();
	void set_ref_clk(bool ext_clk);
	int set_master_clk(double rate);
	int set_rates(double tx_rate, double rx_rate);
	bool parse_dev_type();
	bool flush_recv(size_t num_pkts);

};

#endif /* bladeRF_DEVICE_H */
