/*
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Tushar Krishna
 */

#ifndef __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__
#define __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__

#include <jsoncpp/json/json.h>


#include <map>
#include <set>
#include <string>
#include <vector>

#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/GarnetSyntheticTraffic.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"

#define NHOPS 256
#define ROW 8
#define MaxRow 1000
#define MaxCol 500

extern int receiveMatrix[MaxRow];
extern int execMatrix[MaxRow];
extern bool isSDNoC;
extern std::string graph_file;
extern int syntheticPattern;
extern int isMulticast;
extern int hpcMax;

extern std::map<std::string,int> allLinks;
extern std::map<int,Json::Value> multiOutput;
extern double totalEnergy;
extern int transTime;


using namespace std;


enum TrafficType {BIT_COMPLEMENT_ = 0,
                  BIT_REVERSE_ = 1,
                  BIT_ROTATION_ = 2,
                  NEIGHBOR_ = 3,
                  SHUFFLE_ = 4,
                  TORNADO_ = 5,
                  TRANSPOSE_ = 6,
                  UNIFORM_RANDOM_ = 7,
                  TASKGRAPH_ = 8,
                  NUM_TRAFFIC_PATTERNS_};


class Packet;

struct routeInfo{
    int numberofPkt;
    Json::Value route;
    int state;
    int prior; 
    int turns;
    int src;
    int dst;
    string outputPort;
};

struct pathInfo
{
    string north;
    string south;
    string west;
    string east;
    string pport;

};

struct routerConf
{
    int pPort[4];
    int wPort;
    int ePort;
    int nPort;
    int sPort;
    int state;
};

struct mapTask
{
    int taskId;
    // state 0; Not start; 
    // State 1: start;
    // State 2: finish;
    int state;
    string linkname;
};

extern std::map<std::string,routeInfo> nowTransmit;
extern pathInfo networkPath[NHOPS]; 
extern std::vector<mapTask> dstMapTasks[MaxRow];

class GarnetSyntheticTraffic : public ClockedObject
{
  public:
    typedef GarnetSyntheticTrafficParams Params;
    GarnetSyntheticTraffic(const Params *p);

    void init() override;

    // main simulation loop (one cycle)
    void tick();

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);

  protected:
    EventFunctionWrapper tickEvent;

    class CpuPort : public MasterPort
    {
        GarnetSyntheticTraffic *tester;

      public:

        CpuPort(const std::string &_name, GarnetSyntheticTraffic *_tester)
            : MasterPort(_name, _tester), tester(_tester)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvReqRetry();
    };

    CpuPort cachePort;

    class GarnetSyntheticTrafficSenderState : public Packet::SenderState
    {
      public:
        /** Constructor. */
        GarnetSyntheticTrafficSenderState(uint8_t *_data)
            : data(_data)
        { }

        // Hold onto data pointer
        uint8_t *data;
    };

    PacketPtr retryPkt;
    unsigned size;
    int id;

    std::map<std::string, TrafficType> trafficStringToEnum;

    unsigned blockSizeBits;

    Tick noResponseCycles;

    int numDestinations;
    Tick simCycles;
    int numPacketsMax;
    int numPacketsSent;
    int numPacketsReceive;
    int singleSender;
    int singleDest;
    int sendTime;

    std::string trafficType; // string
    TrafficType traffic; // enum from string
    double injRate;
    int injVnet;
    int precision;

    //map<int,routeInfo> outLinks;

    const Cycles responseLimit;

    MasterID masterId;

    void completeRequest(PacketPtr pkt);

    void generatePkt(int src, unsigned dest, int realsrc, int realdst,string linkName);

    int generateSDNPkt(int src, unsigned dest, routeInfo onePath,string linkName);

    void sendPkt(PacketPtr pkt);
    void initTrafficType();

    void doRetry();
    int belongCluster(int routernumber);

    //SDN updated
    void PrintNetworkState();
    void PrintAllLinks();
    void printAllCandidate();

    bool CheckLink(int routerNum, string routerDir);
    void ReserveLink(int routerNum, string routerDir, string outPort);
    void ReleaseLink(int routerNum, string routerDir);
    string outPortCompute(string outPort);
    void PrintNetwork();
    void PrintdstMaptaks();


    friend class MemCompleteEvent;
};

#endif // __CPU_GARNET_SYNTHETIC_TRAFFIC_HH__