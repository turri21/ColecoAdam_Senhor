#pragma once
#include <iostream>
#include <fstream>
#include "verilated.h"
#include "sim_console.h"


#ifndef _MSC_VER
#else
#define WIN32
#endif

#define kVDNUM 10
#define kBLKSZ 512

struct SimBlockDevice {
public:

	IData* sd_lba[kVDNUM];
	SData* sd_rd;
	SData* sd_wr;
	SData* sd_ack;
	SData* sd_buff_addr;
	CData* sd_buff_dout;
	CData* sd_buff_din[kVDNUM];
	CData* sd_buff_wr;
	SData* img_mounted;
	CData* img_readonly;
	QData* img_size;

	int bytecnt;
        long int disk_size[kVDNUM];
	bool reading;
	bool writing;
	int ack_delay;
	int current_disk;
	bool mountQueue[kVDNUM];
	std::fstream disk[kVDNUM];

	// Block transfer engine. Modelled on what hps_io.sv actually does, not on
	// what is convenient for the core -- see sim_blkdevice.cpp for why.
	enum { X_IDLE, X_WAIT, X_ACTIVE, X_DONE };
	int  xfer_state;
	int  xfer_wait;    // cycles left before we ack (HPS service latency)
	int  din_pipe;     // cycles the address is held before we sample sd_buff_din
	bool served_once;  // for the double-serve stress mode
	int  cfg_latency;      // SIM_SD_LATENCY
	int  cfg_din_latency;  // SIM_SD_DIN_LATENCY
	int  cfg_double_serve; // SIM_SD_DOUBLE_SERVE

	void BeforeEval(int cycles);
	void AfterEval(void);
	//void QueueDownload(std::string file, int index);
	//void QueueDownload(std::string file, int index, bool restart);
	//bool HasQueue();
	void MountDisk( std::string file, int index);

	SimBlockDevice(DebugConsole c);
	~SimBlockDevice();


private:
	//std::queue<SimBus_DownloadChunk> downloadQueue;
	//SimBus_DownloadChunk currentDownload;
	//void SetDownload(std::string file, int index);
};
