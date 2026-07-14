#include <iostream>
#include <queue>
#include <string>

#include "sim_blkdevice.h"
#include "sim_console.h"

#ifndef _MSC_VER
#else
#define WIN32
#endif


static DebugConsole console;

IData* sd_lba[kVDNUM]= {NULL,NULL,NULL,NULL,NULL,
                   NULL,NULL,NULL,NULL,NULL};
SData* sd_rd=NULL;
SData* sd_wr=NULL;
SData* sd_ack=NULL;
SData* sd_buff_addr=NULL;
CData* sd_buff_dout=NULL;
CData* sd_buff_din[kVDNUM]= {NULL,NULL,NULL,NULL,NULL,
                   NULL,NULL,NULL,NULL,NULL};
CData* sd_buff_wr=NULL;
SData* img_mounted=NULL;
CData* img_readonly=NULL;
QData* img_size=NULL;


#define bitset(byte,nbit)   ((byte) |=  (1<<(nbit)))
#define bitclear(byte,nbit) ((byte) &= ~(1<<(nbit)))
#define bitflip(byte,nbit)  ((byte) ^=  (1<<(nbit)))
#define bitcheck(byte,nbit) ((byte) &   (1<<(nbit)))


void SimBlockDevice::MountDisk( std::string file, int index) {
        disk[index].open(file.c_str(), std::ios::out | std::ios::in | std::ios::binary | std::ios::ate);
        if (disk[index]) {
                fprintf(stderr,"we are here\n");
           // we shouldn't do the actual mount here..
           disk_size[index]= disk[index].tellg();
        //fprintf(stderr,"mount size %ld\n",disk_size[index]);
           disk[index].seekg(0);
           mountQueue[index]=1;
           printf("disk %d inserted (%s)\n",index,file.c_str());
        }else {
                fprintf(stderr,"some kind of error: %s\n",file.c_str());
        }

}


// ---------------------------------------------------------------------------
// This shim used to be much friendlier to the core than the real HPS is, and it
// hid a real bug for years: floppy writes worked in simulation and corrupted the
// CP/M directory on hardware. Three divergences mattered:
//
//  1. It zeroed sd_buff_addr the instant it saw sd_rd/sd_wr. Real hps_io only
//     rezeroes sd_buff_addr when *it* starts a command, and leaves it stuck at
//     511 in between (hps_io.sv:292,348,414). A core that uses "&sd_buff_addr"
//     to mean "transfer done" therefore sees a stale 511 the moment it starts a
//     transfer, and declares the transfer complete before a single byte moved.
//     That could never happen here, so it never showed up in sim.
//     -> We now leave sd_buff_addr at 511 between transfers and only zero it
//        when the transfer actually begins.
//
//  2. It sampled sd_buff_din in the same eval in which it drove sd_buff_addr,
//     so it only worked against a zero-latency (combinational) buffer RAM. Real
//     hps_io holds the address for several clk_sys cycles before latching the
//     data, which is why registered BRAM is fine on hardware. Modelling this is
//     what stops the "data is off by N bytes" chases.
//     -> The address is now held SIM_SD_DIN_LATENCY cycles before we sample.
//
//  3. Every request was served exactly once, immediately. The real HPS takes
//     ~ms and has been observed serving a block TWICE (see the IIgs core).
//     -> SIM_SD_LATENCY models the delay; SIM_SD_DOUBLE_SERVE=1 replays each
//        block to prove the core's transfers are idempotent.
//
// Env knobs: SIM_SD_LATENCY (default 1200), SIM_SD_DIN_LATENCY (default 2),
//            SIM_SD_DOUBLE_SERVE (default 0).
// ---------------------------------------------------------------------------
void SimBlockDevice::BeforeEval(int cycles)
{
// wait until the computer boots to start mounting, etc
 if (cycles<2000) return;

 // ---- mounts (only while no transfer is in flight) ----
 if (xfer_state == X_IDLE) {
   for (int i=0; i<kVDNUM; i++) {
     if (mountQueue[i] && !*img_mounted) {
       fprintf(stderr,"mounting.. %d size %ld\n", i, disk_size[i]);
       mountQueue[i]  = 0;
       *img_size      = disk_size[i];
       *img_readonly  = 0;
       disk[i].clear();
       disk[i].seekg(0);
       bitset(*img_mounted, i);
       ack_delay = 1200;               // hold img_mounted for a while
     } else if (bitcheck(*img_mounted, i)) {
       if (ack_delay > 0) ack_delay--;
       if (ack_delay == 0) bitclear(*img_mounted, i);
     }
   }
 }

 // ---- block transfer engine ----
 switch (xfer_state) {

 case X_IDLE: {
   *sd_buff_wr = 0;
   // NOTE: sd_buff_addr is deliberately NOT touched here. Real hps_io leaves it
   // wherever the last transfer left it (511). The core must not depend on it.
   for (int i=0; i<kVDNUM; i++) {
     if (bitcheck(*sd_rd,i) || bitcheck(*sd_wr,i)) {
       current_disk = i;
       reading      = bitcheck(*sd_rd,i) != 0;
       writing      = bitcheck(*sd_wr,i) != 0;
       long lba     = (long)(*(sd_lba[i]));
       disk[i].clear();
       if (writing) disk[i].seekp(lba * kBLKSZ);
       else         disk[i].seekg(lba * kBLKSZ);
       served_once = false;
       xfer_wait   = cfg_latency;      // the HPS is not instant
       xfer_state  = X_WAIT;
       break;
     }
   }
   break;
 }

 case X_WAIT: {
   // The core must hold sd_rd/sd_wr until we ack; that is the contract.
   if (--xfer_wait <= 0) {
     bitset(*sd_ack, current_disk);
     bytecnt       = 0;
     *sd_buff_addr = 0;               // only now, exactly as hps_io does
     din_pipe      = cfg_din_latency;
     xfer_state    = X_ACTIVE;
   }
   break;
 }

 case X_ACTIVE: {
   int i = current_disk;
   if (reading) {
     if (bytecnt < kBLKSZ) {
       *sd_buff_dout = disk[i].get();
       *sd_buff_addr = bytecnt;
       *sd_buff_wr   = 1;
       bytecnt++;
     } else {
       *sd_buff_wr = 0;
       xfer_state  = X_DONE;
     }
   } else if (writing) {
     // hold the address a few cycles before latching, so a registered BRAM
     // (which is what the FPGA infers) reads out correctly.
     if (din_pipe > 0) {
       din_pipe--;
     } else {
       disk[i].put((char)(*(sd_buff_din[i])));
       bytecnt++;
       if (bytecnt < kBLKSZ) {
         *sd_buff_addr = bytecnt;
         din_pipe      = cfg_din_latency;
       } else {
         xfer_state = X_DONE;
       }
     }
   } else {
     xfer_state = X_DONE;
   }
   break;
 }

 case X_DONE: {
   int i = current_disk;
   *sd_buff_wr = 0;
   bitclear(*sd_ack, i);
   // hps_io saturates the address and leaves it here until its next command.
   *sd_buff_addr = kBLKSZ - 1;
   disk[i].flush();

   if (cfg_double_serve && !served_once) {
     // The real HPS has been seen serving the same block twice. Replay it: a
     // correct core is idempotent (it must not, say, advance its LBA per pass).
     served_once = true;
     long lba = (long)(*(sd_lba[i]));
     disk[i].clear();
     if (writing) disk[i].seekp(lba * kBLKSZ);
     else         disk[i].seekg(lba * kBLKSZ);
     xfer_wait  = cfg_latency;
     xfer_state = X_WAIT;
   } else {
     reading = writing = false;
     current_disk = -1;
     xfer_state   = X_IDLE;
   }
   break;
 }
 }
}

void SimBlockDevice::AfterEval()
{
}


static int env_int(const char *name, int dflt) {
        const char *e = getenv(name);
        return e ? atoi(e) : dflt;
}

SimBlockDevice::SimBlockDevice(DebugConsole c) {
        console = c;
        current_disk=-1;

        xfer_state  = X_IDLE;
        xfer_wait   = 0;
        din_pipe    = 0;
        served_once = false;
        bytecnt     = 0;
        reading     = false;
        writing     = false;
        ack_delay   = 0;

        cfg_latency      = env_int("SIM_SD_LATENCY", 1200);
        cfg_din_latency  = env_int("SIM_SD_DIN_LATENCY", 2);
        cfg_double_serve = env_int("SIM_SD_DOUBLE_SERVE", 0);
        fprintf(stderr, "BLKDEV: latency=%d din_latency=%d double_serve=%d\n",
                cfg_latency, cfg_din_latency, cfg_double_serve);

        sd_rd = NULL;
        sd_wr = NULL;
        sd_ack = NULL;
        sd_buff_addr = NULL;
        sd_buff_dout = NULL;
        for (int i=0;i<kVDNUM;i++) {
           sd_lba[i] = NULL;
           sd_buff_din[i] = NULL;
           mountQueue[i]=0;
        }
        sd_buff_wr=NULL;
        img_mounted=NULL;
        img_readonly=NULL;
        img_size=NULL;
}

SimBlockDevice::~SimBlockDevice() {

}
