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
 * Multified By: Hui Chen 
 */

#include "cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTraffic.hh"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <unistd.h>



#include "base/logging.hh"
#include "base/random.hh"
#include "base/statistics.hh"
#include "debug/GarnetSyntheticTraffic.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"
#define EXBAR 0.108
#define EBF 0.078
#define EGLOB 0.031
#define ELoC 0.008
#define Lpk 128
#define ECU 0.917



int numTask=0;
int TESTER_NETWORK=0;
int numPseve = 0;
int numPreve = 0;
queue<string> graphQ;
Json::Reader reader;
Json::Value root;
int endSendTime = 0;
int receiveMatrix[MaxRow];
bool isSDNoC;
int execMatrix[MaxRow];
int needSend[MaxRow];
int flag = 0;
int blockTime = 0;
int transTime = 0;
int totalFlits = 0;
double totalEnergy = 0;
int checkTimes = 0;
int controlOverhead = 0;
pathInfo networkPath[NHOPS];
map<string,int> allLinks;
map<string,Json::Value> candidates;
std::map<std::string,routeInfo> nowTransmit;
std::map<int,Json::Value> multiOutput;
map<string,int> numHop;
map<string,int> numTurn;
map<string,routeInfo> outLinks[MaxRow];
int maptoMatrix[MaxRow];
set<string> totalStart;
set<string> totalEnd;
int totalDirect = 0;
int totalinDirect = 0;
std::vector<mapTask> mapTasks[MaxRow];
std::vector<mapTask> dstMapTasks[MaxRow];
double totalBufferEnergy = 0;
double totalArbEnergy = 0;
double totalTransEnergy = 0;
double totalSmartBufferEnergy = 0;
double totalSmartArbEnergy = 0;
double totalExe = 0;
int totalSize = 0;
int totalRouter = 0;


int syntheticPattern;
//int sendAllowMax[NHOPS];
int isMulticast=0;

void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  while(std::string::npos != pos2)
  {
    v.push_back(s.substr(pos1, pos2-pos1));
 
    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if(pos1 != s.length())
    v.push_back(s.substr(pos1));
}

bool
GarnetSyntheticTraffic::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tester->completeRequest(pkt);
    return true;
}

void
GarnetSyntheticTraffic::CpuPort::recvReqRetry()
{
    tester->doRetry();
}

void
GarnetSyntheticTraffic::sendPkt(PacketPtr pkt)
{
    if (!cachePort.sendTimingReq(pkt)) {
        retryPkt = pkt; // RubyPort will retry sending
    }

}

GarnetSyntheticTraffic::GarnetSyntheticTraffic(const Params *p)
    : ClockedObject(p),
      tickEvent([this]{ tick(); }, "GarnetSyntheticTraffic tick",
                false, Event::CPU_Tick_Pri),
      cachePort("GarnetSyntheticTraffic", this),
      retryPkt(NULL),
      size(p->memory_size),
      blockSizeBits(p->block_offset),
      numDestinations(p->num_dest),
      simCycles(p->sim_cycles),
      numPacketsMax(p->num_packets_max),
      numPacketsSent(0),
      numPacketsReceive(0),
      singleSender(p->single_sender),
      singleDest(p->single_dest),
      trafficType(p->traffic_type),
      injRate(p->inj_rate),
      injVnet(p->inj_vnet),
      precision(p->precision),
      responseLimit(p->response_limit),
      masterId(p->system->getMasterId(this))
{
    // set up counters
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    initTrafficType();
    if (trafficStringToEnum.count(trafficType) == 0) {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }
    traffic = trafficStringToEnum[trafficType];
    syntheticPattern = traffic;


    id = TESTER_NETWORK++;
    //id = 0;
    DPRINTF(GarnetSyntheticTraffic,"Config Created: Name = %s , and id = %d\n",
            name(), id);

}

Port &
GarnetSyntheticTraffic::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "test")
        return cachePort;
    else
        return ClockedObject::getPort(if_name, idx);
}

void
GarnetSyntheticTraffic::init()
{
    numPacketsSent = 0;
}


//Receive the packets
void
GarnetSyntheticTraffic::completeRequest(PacketPtr pkt)
{
    int dest = (pkt->req->getPaddr())>>6;
    DPRINTF(GarnetSyntheticTraffic,
            "Completed injection of %s packet for address %d\n",
            pkt->isWrite() ? "write" : "read",
            dest);

    assert(pkt->isResponse());
    noResponseCycles = 0;

    delete pkt;
}


void
GarnetSyntheticTraffic::tick()
{
    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }
    ///////////////////////////////////////////////////////
    //  Print Network State
    //  At the beginning of each cycle, print the infomation of network
    ///////////////////////////////////////////////////////
    if (id==0)
    {
        DPRINTF(GarnetSyntheticTraffic,"++++++++++++++++++++Cycle %d++++++++++++++++++++++\n",curTick());
        if(isSDNoC){
            DPRINTF(GarnetSyntheticTraffic,"nowTransmit\n");
            for (auto iter = nowTransmit.begin();
                        iter != nowTransmit.end();
                        iter++)
                {
                DPRINTF(GarnetSyntheticTraffic,"%s \n",iter->first);
                }
        }
        PrintNetwork();
        PrintNetworkState();
        PrintAllLinks();

    }

    if (traffic == TASKGRAPH_ && curTick()==0&&id==0){


        ifstream in(graph_file,ios::binary);
        char buff[1000];
        getcwd(buff, 1000);
        std::cout << "is: "<<buff<<"    "<<graph_file<<"\n";

        if (!in.is_open())
        {
            fatal("Fail to open file\n");
            return;
        }
        if(isSDNoC && (routing_algorithm==2))
        {
            //cout<<"Here is the multicast !!!!!!!!!!!!!!!!!!!!"<<endl;
            isMulticast=1;
        }

        if (reader.parse(in,root))
        {
            
            
            //Initialize receive send 
            for (int i = 0; i<root.size();i++)
            {
                receiveMatrix[i]=0;
                needSend[i]=0;
            }
            totalSize = root.size();
            if(totalSize>MaxRow)
            {
                exitSimLoop("Maximum number of tasks exceeded! ");
            }
            totalRouter = rowNum*rowNum;
            for (int i = 0; i < root.size() ; i++)
            {
                string t = to_string(i);
                maptoMatrix[i] = -1;
                //Directly read from the file
                //receiveMatrix[i] = root[t]["total_needReceive"].asInt();
                //needSend[i] = root[t]["total_needSend"].asInt();
                totalFlits+=root[t]["total_needReceive"].asInt();
                execMatrix[i] = root[t]["exe_time"].asInt();
                totalExe = totalExe +execMatrix[i];
                int coreId = root[t]["mapto"].asInt();
                maptoMatrix[i] = coreId;
                mapTask taskItem;
                taskItem.taskId = i;
                taskItem.state = 0;
                mapTasks[coreId].push_back(taskItem);


                int numberSender = root[t]["out_links"].size();
                for (int ii = 0; ii < numberSender; ii++){
                    routeInfo info;
                    info.state = 0;
                    info.numberofPkt = root[t]["out_links"][ii][0][1].asInt();
                    //Configuration time 
                    Json::Value aroute = root[t]["out_links"][i][0][3];
                    int shouldBlock = int(ceil(float(aroute.size())/float(hpcMax))*3)+info.numberofPkt+aroute.size();
                    info.numberofPkt = shouldBlock;
                    info.src = root[t]["out_links"][ii][0][5].asInt();
                    info.dst = root[t]["out_links"][ii][0][6].asInt();
                    


                    string outLink = root[t]["out_links"][ii][0][0].asString();
                    string outport1 = root[t]["out_links"][ii][0][3][0][1].asString();
                    info.outputPort = outport1;
 
                    int transId = root[t]["out_links"][ii][0][0].asInt();
                    
                    receiveMatrix[transId] += shouldBlock;
                    needSend[i] += shouldBlock;
                    mapTask taskItem;
                    taskItem.taskId = transId;
                    taskItem.state = 0;

                    
                    string linkName;
                    linkName = t+"+"+outLink+"+"+outport1;

                    outLinks[i].insert(make_pair(linkName,info));

                    
                   
                    
                    taskItem.linkname = linkName;
                    dstMapTasks[info.dst].push_back(taskItem);

                    allLinks.insert(make_pair(linkName,info.numberofPkt));
                    //numHop.insert(make_pair(linkName,hops));
                    //numTurn.insert(make_pair(linkName,turns));
                    Json::Value candi = root[t]["out_links"][ii];
                    candidates.insert(make_pair(linkName,candi));
                }


            }

            for (int i = 0;i<totalRouter; ++i)
            {
                networkPath[i].west = "Unknown";
                networkPath[i].east = "Unknown";
                networkPath[i].north = "Unknown";
                networkPath[i].south = "Unknown";
                networkPath[i].pport = "Unknown";
            }
            // PrintNetworkState();
            // PrintAllLinks();
            
        }
        else
        {
            fatal("Fail to load file\n");
        }

    }

    //At the begining of each cycle, check if it is the "included cycle"
    //If yes, send packets,
    if (traffic == TASKGRAPH_ && id == 0)
    {
        //for(int ii=0;ii<NHOPS;ii++)
        //{sendAllowMax[ii] = 0;}

        DPRINTF(GarnetSyntheticTraffic,
         "\ncurTick is %d: id %d send packets"
         "%d receive packets %d receiveMatrix %d\n"
         , curTick(),id,numPseve, numPreve,receiveMatrix[4]);

    }
    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    // for this core, check if this can send the packet
    // changed here
    // for(int zz=0;zz<MaxRow;zz++){
    //     if(maptoMatrix[zz]==-1||maptoMatrix[zz]!=id)
    //         continue;
     
    for(int m = 0; m<mapTasks[id].size();m++){

        int zz = mapTasks[id][m].taskId;
        //cout<<maptoMatrix[zz]<<" "<<zz<<" is maped to "<<id<<endl;
        bool sendAllowedThisCycle = false;
        bool execAllowedThisCycle = false;
        double injRange = pow((double) 10, (double) precision);
        unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
        if (traffic == TASKGRAPH_){
            if(receiveMatrix[zz]<= 0)
            {
                if(execMatrix[zz]>0){

                    sendAllowedThisCycle = false;
                    execAllowedThisCycle = true;
                }
                else{
                    if (needSend[zz] > 0)
                    {
                        //cout<<"send allowed this cycle "<<curTick()<<endl;
                        sendAllowedThisCycle = true;
                    }
                    else
                        sendAllowedThisCycle = false;
                }
            }
        }
        else{
            if (trySending < injRate*injRange)
                sendAllowedThisCycle = true;
            else
                sendAllowedThisCycle = false;
        }

        if (execAllowedThisCycle)
        {
            cout<<"execute this time "<<zz<<" "<<curTick()<<endl;
            execMatrix[zz]--;
        }
            

            // always generatePkt unless fixedPkts or singleSender is enabled
        if (sendAllowedThisCycle) {           
            bool senderEnable = true;

            if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
                senderEnable = false;

            if (singleSender >= 0 && zz != singleSender)
                senderEnable = false;

            if (senderEnable){
                //int nextCycle = 0;
                if (traffic == TASKGRAPH_){
                    string trackid = to_string(zz);
                    

                    int totalSend = 0;
                    for (auto iter = outLinks[zz].begin();iter != outLinks[zz].end();++iter)
                    {
                        string linkName = iter->first;
                        vector<string> t;
                        SplitString(linkName,t,"+");
                        int destNow = stoi(t[1],nullptr,0);

                        //sendAllowMax[destNow]++;
                        Json::Value route1;

                        // string linkName;
                        // linkName = trackid+"+"+to_string(destNow)+"+"+iter->second.outputPort;
                        // cout<<"send allowed+++++"<<linkName<<endl;
                        
                        //if((iter->seccoutond.numberofPkt==0 && iter->second.state ==1 ))
                        //Changed here
                        if(allLinks[linkName]<=1 && iter->second.state !=-1)
                        {
                            //  for(int tt = 0;tt<dstMapTasks[iter->second.dst].size();tt++)
                            //     {
                            //         if(dstMapTasks[iter->second.dst][tt].linkname == linkName)
                            //         {
                            //             dstMapTasks[iter->second.dst][tt].state = 2;
                            //         }
                            // }
                            //cout<<linkName<<endl;
                            if(isSDNoC){
                                //Release Resources
                                int arrayNum = iter->second.route.size();

                                for(int i=0;i<arrayNum;++i)
                                {
                                    int routerNum = iter->second.route[i][0].asInt();

                                    string routerDirIn = iter->second.route[i][1].asString();
                                    string routerDirOut = iter->second.route[i][2].asString();
                                    //cout<<"Release Link+++++++++"<<linkName<<endl;
                                    ReleaseLink(routerNum,routerDirIn);
                                    //ReleaseLink(routerNum,routerDirOut);

                                }
                            }
                           
                            cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!End Transmission!!"<<trackid<<" "<<linkName<<" "<<curTick()<<endl;
                            //totalEnd++;
                            totalEnd.insert(linkName);
                            iter->second.state = -1;
                            //PrintNetworkState();
                            //PrintAllLinks();
                            //nowTransmit.erase(linkName);


                        }
                        //changed here
                        //if ((iter->second.numberofPkt>0&&totalSend<4))
                        if(allLinks[linkName]>0)
                        {
                            // cout<<"This is outLinks"<<id<<" "
                            //  <<iter->first<<"this is reminder"
                            //  <<iter->second.numberofPkt<<" total sent "<<totalSend<<endl;

                            if(!isSDNoC)
                            {  
                                //totalSent numbers
                                if(totalSend<=1){
                                cout<<"Sending Now+++"<<id<<" dest: "<<iter->second.dst<<" linkname: "<<linkName<<endl;
                                generatePkt(zz,destNow,iter->second.src,iter->second.dst,linkName);

                                for(int tt = 0;tt<dstMapTasks[iter->second.dst].size();tt++)
                                {
                                    if(dstMapTasks[iter->second.dst][tt].linkname == linkName)
                                    {
                                        dstMapTasks[iter->second.dst][tt].state = 1;
                                    }
                                }
                                totalSend+= 1;
                                break;
                                
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {

                                //If the state is 0; check the state and begin to transfer
                                if(iter->second.state == 0 )
                                {
                                    route1 = iter->second.route;

                                    int checkSuccess = generateSDNPkt(zz,destNow,iter->second,linkName);
                                    if(checkSuccess==1)
                                    {
                                        cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Start Transmission!!"<<trackid<<" "<<destNow<<" "<<curTick()<<endl;
                                        //totalStart++;
                                        totalStart.insert(linkName);
                                        totalSend+= 1;
                                        
                                        receiveMatrix[destNow]--;
                                        needSend[zz]--;
                                        allLinks[linkName]--;
                                        
                                        int hops = iter->second.route.size();
                                        int turns = iter->second.turns;
                                        int srccluster = belongCluster(iter->second.src);
                                        int dstcluster = belongCluster(iter->second.dst);
                                        int clusterDiff = 0;
                                        if((srccluster==1&&dstcluster==2)||(srccluster==2&&dstcluster==1)) 
                                            clusterDiff =3;
                                        else if((srccluster==0&&dstcluster==2)||(srccluster==2&&dstcluster==0))
                                            clusterDiff =2;
                                        else
                                            clusterDiff = abs(srccluster - dstcluster)+1;
                                        
                                        controlOverhead += clusterDiff*2; 
                                        iter->second.state = clusterDiff*2;
                                        if(turns==0)
                                            totalDirect++;
                                        else if(turns==1)
                                            totalinDirect++;
                                        else
                                        {
                                            //cout<<"error"<<turns<<endl;
                                            turns = 0;
                                            totalDirect++;
                                        }
                                        transTime+=turns*1;
                                        transTime++;
                                        //cout<<"hops: "<<hops<<endl;
                                        //totalEnergy+= (EXBAR*(hops+1) + EBF*turns + EGLOB*hops + 2* ELoC) * Lpk + ECU*(hops+1);
                                        totalEnergy+= (EXBAR*(hops+1) + EBF*(hops-2) + EGLOB*hops + 2* ELoC) * Lpk + ECU*(hops+1);
                                        totalSmartBufferEnergy+= (EBF*(hops-2) + EGLOB*hops + 2* ELoC)* Lpk;
                                        totalSmartArbEnergy+=ECU*(hops+1);
                                        totalTransEnergy+=EXBAR*(hops+1)* Lpk;
                                        totalArbEnergy+=EGLOB*hops;
                                    }
                                    // else
                                    // {
                                    //     blockTime++;
                                    // }
                                     
                                }
                                if(iter->second.state>1)
                                {
                                    iter->second.state--;
                                }
                                if(iter->second.state==1 || iter->second.state==-1)
                                //else
                                {
                                    // if(iter->second.state!=1)
                                    //     cout<<"state=="<<iter->second.state<<endl;
                                    int hops = iter->second.route.size();
                                    int turns = iter->second.turns;
                                    
                                    if(turns==0)
                                        totalDirect++;
                                    else if(turns==1)
                                        totalinDirect++;
                                    else
                                    {
                                        //cout<<"error"<<turns<<endl;
                                        turns = 0;
                                        totalDirect++;
                                    }
                                    
                                    generatePkt(zz,destNow,iter->second.src,iter->second.dst,linkName);
                                    totalSend+= 1;
                                    receiveMatrix[destNow]--;   
                                    transTime++;
                                    transTime+=turns*1;
                                    needSend[zz]--;  
                                    allLinks[linkName]--;   
                                    //cout<<"Sending Now+++"<<id<<" dest: "<<iter->second.dst<<" linkname: "<<linkName<<endl;
                                    //totalEnergy+= (EXBAR*(hops+1) + EBF*turns + EGLOB*hops + 2* ELoC) * Lpk + ECU*(hops+1); 
                                    totalEnergy+= (EXBAR*(hops+1) + EBF*(hops-2) + EGLOB*hops + 2* ELoC) * Lpk + ECU*(hops+1);   
                                    totalSmartBufferEnergy+= (EBF*(hops-2) + EGLOB*hops + 2* ELoC)* Lpk;
                                    totalSmartArbEnergy+=ECU*(hops+1);
                                    totalTransEnergy+=EXBAR*(hops+1)* Lpk;
                                    
                                }

                            }
                            
                        }
                       
                    }
                }
                else
                {
                    generatePkt(0,0,0,0," ");
                }
            }

        }
    }
    
    if (traffic == TASKGRAPH_ && id == 0 && curTick()>1)
    {
        //cout<<"checking+++++++++"<<allLinks.size()<<endl;

        flag = 1;
        for (auto iter = allLinks.begin();iter != allLinks.end();iter++)
        {
            //cout<<iter->first<<iter->second<<endl;
            if(int(iter->second)>0)
            {
                flag = 0;
                break;
            }
        }
       
        
    }
    
    if (flag == 1 && curTick()>0 && id==0)
    {
        //PrintNetworkState();
        //PrintAllLinks();
        // cout<<"Total blocking time: "<<blockTime<<endl;
        cout<<"checking+++++++++"<<endl;
        vector<string> v;
        cout<<graph_file<<endl;
        SplitString(graph_file,v,"/");

        cout<<v[1]<<endl;
        vector<string> t;
        SplitString(v[1],t,"_");
        vector<string> z;
        SplitString(t[2],z,"R");
        //cout<<z[1]<<endl;
        int appSize = stoi(z[1]);

        int tranTotal = transTime+blockTime;
        double latency = tranTotal/appSize;
        double throughput = totalFlits/curTick();
        double checkEnergy = checkTimes*ECU;
        cout<<"Hpcmax: "<<hpcMax<<endl;
        cout<<"Total trasmission time: "<<tranTotal<<endl;
        cout<<"Total Flit: "<<totalFlits<<endl;
        cout<<"Total energy: "<<totalEnergy<<endl;
        cout<<"Communication latency: "<<latency<<endl;
        cout<<"Network throughput: "<< throughput<<endl;
        cout<<"Total direct: "<< totalDirect<<endl;
        cout<<"Total indirect: "<< totalinDirect<<endl;
        cout<<"Contention Time: "<<blockTime<<endl;
        cout<<"totalSmartArbEnergy: "<<totalSmartArbEnergy<<endl;
        cout<<"totalSmartBufferEnergy: "<<totalSmartBufferEnergy<<endl;
        cout<<"totalTransEnergy: "<<totalTransEnergy<<endl;
        cout<<"totalBufferEnergy: "<<totalBufferEnergy<<endl;
        cout<<"totalArbEnergy: "<<totalArbEnergy<<endl;
        cout<<"checkEnergy: "<<checkEnergy<<endl;
        cout<<"controlOverhead: "<<controlOverhead<<endl;
        cout<<"totalExe: "<<totalExe<<endl;
        

        //cout<<t[2]<<endl;
        ofstream out("results.txt",ios::app);
        if(!out)
        {
            cout<< "results.txt can't open"<< endl;
        }
        else
        {   
            cout<<"Hpcmax: "<<hpcMax<<endl;
            out<<"Filename: "<<graph_file<<endl;
            out<<"Schedule Length: "<<curTick()<<endl;
            out<<"Total trasmission time: "<<transTime+blockTime<<endl;
            out<<"Total Flit: "<<totalFlits<<endl;
            out<<"Total energy: "<<totalEnergy<<endl;
            out<<"Communication latency: "<< (transTime+blockTime)/appSize<<endl;
            out<<"Network throughput: "<< totalFlits/curTick()<<endl;
            out<<"Total direct: "<< totalDirect<<endl;
            out<<"Total indirect: "<< totalinDirect<<endl;
            out<<"Contention Time: "<<blockTime<<endl;
            out<<"totalSmartArbEnergy: "<<totalSmartArbEnergy<<endl;
            out<<"totalSmartBufferEnergy: "<<totalSmartBufferEnergy<<endl;
            out<<"totalTransEnergy: "<<totalTransEnergy<<endl;
            out<<"totalBufferEnergy: "<<totalBufferEnergy<<endl;
            out<<"totalArbEnergy: "<<totalArbEnergy<<endl;
            out<<"checkEnergy: "<<checkEnergy<<endl;
            out<<"controlOverhead: "<<controlOverhead<<endl;
            out<<"totalExe: "<<totalExe<<endl;
            out.close();
        }
        //PrintdstMaptaks();
        exitSimLoop("Network Tester completed. There is no task to execute.\n");


    }

    if (curTick() >= simCycles){
        if(id==0){
        
        set<string> result;
        
        set_difference(  totalEnd.begin(), totalEnd.end() ,totalStart.begin(), totalStart.end(), inserter(result,result.begin()));
        
        PrintNetworkState();
        PrintNetwork();
        PrintAllLinks();
        cout<<"Difference: "<<totalStart.size()<<" "<<totalEnd.size()<<" "<<result.size()<<endl;
        for (auto iter = result.begin();iter != result.end();iter++)
        {
            cout<<*iter<<endl;
        }
        cout<<"\n";
        PrintdstMaptaks();

    }
        exitSimLoop("Network Tester completed simCycles");
    }
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void GarnetSyntheticTraffic::PrintNetworkState()
{
    DPRINTF(GarnetSyntheticTraffic,
        "\n receiveMatrix: task remaining needs to receive: ");

    for(int i2=0;i2<totalSize;++i2)
    {
        DPRINTF(GarnetSyntheticTraffic," %d %d |",i2,receiveMatrix[i2]);
        if(i2%rowNum==0)
        {
            DPRINTF(GarnetSyntheticTraffic,"\n");
        }
    }
    DPRINTF(GarnetSyntheticTraffic,"\n");
    DPRINTF(GarnetSyntheticTraffic,
        "\n needSend: task needs to send: ");
    for(int i2=0;i2<totalSize;++i2)
    {
        DPRINTF(GarnetSyntheticTraffic," %d %d |",i2,needSend[i2]);
        if(i2%rowNum==0)
        {
            DPRINTF(GarnetSyntheticTraffic,"\n");
        }
    }
    DPRINTF(GarnetSyntheticTraffic,"\n");
    DPRINTF(GarnetSyntheticTraffic,
        "\n execMatrix: processor needs to exe: ");

    for(int i2=0;i2<totalSize;++i2)
    {
        DPRINTF(GarnetSyntheticTraffic," %d %d |",i2,execMatrix[i2]);
        if(i2%rowNum==0)
        {
            DPRINTF(GarnetSyntheticTraffic,"\n");
        }
    }
    DPRINTF(GarnetSyntheticTraffic,"\n");
    //cout<<totalStart<<" "<<totalEnd<<endl;
}
void GarnetSyntheticTraffic::PrintdstMaptaks()
{
    cout<<"\n dstMatrix: router remaining needs to receive";
    for(int i2=0;i2<50;++i2)
    {
        cout<<i2<<endl;
        for (int tt=0;tt<dstMapTasks[i2].size();tt++)
            cout<<i2<<" "<<dstMapTasks[i2][tt].linkname<<" "<<dstMapTasks[i2][tt].state<<" | ";
        cout<<endl;
        
    }
    for(int i2=0;i2<50;++i2)
    {
        cout<<i2<<endl;
        for (int tt=0;tt<mapTasks[i2].size();tt++)
            cout<<mapTasks[i2][tt].taskId<<" | ";
        cout<<endl;
        
    }
}

void GarnetSyntheticTraffic::PrintAllLinks()
{
    DPRINTF(GarnetSyntheticTraffic,"Print ALL Links!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    DPRINTF(GarnetSyntheticTraffic,"All Links: from+to+port remaining\n");
    for (auto iter = allLinks.begin();iter != allLinks.end();iter++)
    {
        
        DPRINTF(GarnetSyntheticTraffic," %d %d |",iter->first,iter->second);
    }
    DPRINTF(GarnetSyntheticTraffic,"\n");
}


void GarnetSyntheticTraffic::printAllCandidate()
{
    DPRINTF(GarnetSyntheticTraffic,"Print ALL Candidates!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

    for (auto iter = candidates.begin();iter != candidates.end();iter++)
    {
        DPRINTF(GarnetSyntheticTraffic,"%s \n",iter->first);
        int i = 0;
        for (i=0;i<iter->second.size();i++)
            DPRINTF(GarnetSyntheticTraffic," %s ",iter->second[i][0].asString());
    }
    DPRINTF(GarnetSyntheticTraffic,"\n");
}

void GarnetSyntheticTraffic::PrintNetwork()
{
    
    DPRINTF(GarnetSyntheticTraffic,"\n networkPath: networkPath info\n");
    for(int i2=0;i2<totalRouter;i2++)
    {
        if(i2%rowNum==0)
        {
            DPRINTF(GarnetSyntheticTraffic,"\n");
        }
        DPRINTF(GarnetSyntheticTraffic," %d %d |",i2,networkPath[i2].north[0]);

    }
    DPRINTF(GarnetSyntheticTraffic,"\n");
}




bool GarnetSyntheticTraffic::CheckLink(int routerNum, string routerDir)
{
    //cout<<"checklink"<<routerNum<<routerDir<<endl;
    int z=routerDir.size();
    for(int i=0;i<z;i++){
        //cout<<"network port: "<<routerDir[i]<<" W "<<networkPath[routerNum].west<<" E "<<networkPath[routerNum].east<<" S "<<networkPath[routerNum].south<<" N "<<networkPath[routerNum].north<<endl;
        if((networkPath[routerNum].west!="Unknown"&&routerDir[i]=='W')||
                (networkPath[routerNum].east!="Unknown"&&routerDir[i]=='E')||
                (networkPath[routerNum].north!="Unknown"&&routerDir[i]=='N')||
                (networkPath[routerNum].south!="Unknown"&&routerDir[i]=='S')||
                (networkPath[routerNum].pport!="Unknown"&&routerDir[i]=='P'))
            {
                return false;
            }
    }
    return true;
}
string GarnetSyntheticTraffic::outPortCompute(string outPort)
{
    if(outPort=="N")
        return "South";
    else if(outPort=="P")
        return "Local";
    else if(outPort=="W")
        return "West";
    else if(outPort=="E")    
        return "East";
    else if(outPort=="S")
        return "North";
    else
        return "Unknown";

}

void GarnetSyntheticTraffic::ReserveLink(int routerNum, string routerDir,string outPort)
{
    if(routerDir=="W")
    {
        //networkPath[routerNum].west=outPortCompute(outPort);
        networkPath[routerNum].west="W";
    }
    else if(routerDir=="E")
    {
        //networkPath[routerNum].east=outPortCompute(outPort);
        networkPath[routerNum].east="E";
    }
    else if(routerDir=="N")
    {
        //changed here
        //networkPath[routerNum].south=outPortCompute(outPort);
        networkPath[routerNum].north="N";
    }
    else if(routerDir=="S")
    {
        //changed here
        //networkPath[routerNum].north=outPortCompute(outPort);
        networkPath[routerNum].south="S";
    }
    else if(routerDir=="P")
    {
        networkPath[routerNum].pport="P";
    }
    else
    {
        //cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++Unknown port"<<endl;
    }

}


void GarnetSyntheticTraffic::ReleaseLink(int routerNum, string routerDir)
{
    if(routerDir=="W")
    {
        networkPath[routerNum].west="Unknown";
    }
    else if(routerDir=="E")
    {
        networkPath[routerNum].east="Unknown";
    }
    else if(routerDir=="N")
    {
        networkPath[routerNum].north="Unknown";
    }
    else if(routerDir=="S")
    {
        networkPath[routerNum].south="Unknown";
    }
    else if(routerDir=="P")
    {
        networkPath[routerNum].pport="Unknown";
    }
    else
    {
       // cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++Unknown port"<<endl;
    }
}

int
GarnetSyntheticTraffic::belongCluster(int routernumber)
{
    int t = 0;
    if (routernumber%16>8)
    {
        t++;
    }
    if(routernumber/16>8)
    {
        t++;
    }
    return t;
}
void
GarnetSyntheticTraffic::generatePkt(int src, unsigned dest, int realsrc, int realdst,string linkName)
{
    int num_destinations = numDestinations;
    int radix = (int) sqrt(num_destinations);
    unsigned destination = id;
    int dest_x = -1;
    int dest_y = -1;
    int source = id;
    int src_x = id%radix;
    int src_y = id/radix;
    unsigned realdst1 = realdst;

    if (singleDest >= 0)
    {
        destination = singleDest;
    } else if (traffic == UNIFORM_RANDOM_) {
        destination = random_mt.random<unsigned>(0, num_destinations - 1);
    } else if (traffic == BIT_COMPLEMENT_) {
        dest_x = radix - src_x - 1;
        dest_y = radix - src_y - 1;
        destination = dest_y*radix + dest_x;
    } else if (traffic == BIT_REVERSE_) {
        unsigned int straight = source;
        unsigned int reverse = source & 1; // LSB

        int num_bits = (int) log2(num_destinations);

        for (int i = 1; i < num_bits; i++)
        {
            reverse <<= 1;
            straight >>= 1;
            reverse |= (straight & 1); // LSB
        }
        destination = reverse;
    } else if (traffic == BIT_ROTATION_) {
        if (source%2 == 0)
            destination = source/2;
        else // (source%2 == 1)
            destination = ((source/2) + (num_destinations/2));
    } else if (traffic == NEIGHBOR_) {
            dest_x = (src_x + 1) % radix;
            dest_y = src_y;
            destination = dest_y*radix + dest_x;
    } else if (traffic == SHUFFLE_) {
        if (source < num_destinations/2)
            destination = source*2;
        else
            destination = (source*2 - num_destinations + 1);
    } else if (traffic == TRANSPOSE_) {
            dest_x = src_y;
            dest_y = src_x;
            destination = dest_y*radix + dest_x;
    } else if (traffic == TORNADO_) {
        dest_x = (src_x + (int) ceil(radix/2) - 1) % radix;
        dest_y = src_y;
        destination = dest_y*radix + dest_x;
    }
      else if (traffic == TASKGRAPH_){
        destination = realdst1;
        }

    else {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }

    // The source of the packets is a cache.
    // The destination of the packets is a directory.
    // The destination bits are embedded in the address after byte-offset.
    Addr paddr =  destination;
    paddr <<= blockSizeBits;
    unsigned access_size = 1; // Does not affect Ruby simulation

    // Modeling different coherence msg types over different msg classes.
    //
    // GarnetSyntheticTraffic assumes the Garnet_standalone coherence protocol
    // which models three message classes/virtual networks.
    // These are: request, forward, response.
    // requests and forwards are "control" packets (typically 8 bytes),
    // while responses are "data" packets (typically 72 bytes).
    //
    // Life of a packet from the tester into the network:
    // (1) This function generatePkt() generates packets of one of the
    //     following 3 types (randomly) : ReadReq, INST_FETCH, WriteReq
    // (2) mem/ruby/system/RubyPort.cc converts these to RubyRequestType_LD,
    //     RubyRequestType_IFETCH, RubyRequestType_ST respectively
    // (3) mem/ruby/system/Sequencer.cc sends these to the cache controllers
    //     in the coherence protocol.
    // (4) Network_test-cache.sm tags RubyRequestType:LD,
    //     RubyRequestType:IFETCH and RubyRequestType:ST as
    //     Request, Forward, and Response events respectively;
    //     and injects them into virtual networks 0, 1 and 2 respectively.
    //     It immediately calls back the sequencer.
    // (5) The packet traverses the network (simple/garnet) and reaches its
    //     destination (Directory), and network stats are updated.
    // (6) Network_test-dir.sm simply drops the packet.
    //
    MemCmd::Command requestType;

    RequestPtr req = nullptr;
    Request::Flags flags;

    // Inject in specific Vnet
    // Vnet 0 and 1 are for control packets (1-flit)
    // Vnet 2 is for data packets (5-flit)
    int injReqType = injVnet;

    if (injReqType < 0 || injReqType > 2)
    {
        // randomly inject in any vnet
        injReqType = random_mt.random(0, 2);
    }

    if (injReqType == 0) {
        // generate packet for virtual network 0
        requestType = MemCmd::ReadReq;
        req = std::make_shared<Request>(paddr, access_size, flags, masterId);
    } else if (injReqType == 1) {
        // generate packet for virtual network 1
        requestType = MemCmd::ReadReq;
        flags.set(Request::INST_FETCH);
        req = std::make_shared<Request>(
            1, 0x1, access_size, flags, masterId, 0x1, 1);
        req->setPaddr(paddr);
    } else {  // if (injReqType == 2)
        // generate packet for virtual network 2
        requestType = MemCmd::WriteReq;
        req = std::make_shared<Request>(paddr, access_size, flags, masterId);
    }

    req->setContext(source);

    //No need to do functional simulation
    //We just do timing simulation of the network

    DPRINTF(GarnetSyntheticTraffic,
            "Generated packet with source %d, destination %d"
            ", embedded in address %x, packet size is %d\n",
            source, destination, req->getPaddr(), req->getSize());

    PacketPtr pkt = new Packet(req, requestType);
    if (traffic == TASKGRAPH_)
    {  
        //pkt->dataDynamic(var);
        uint8_t *var = new uint8_t[req->getSize()];
        var[0] = 0x10;
        var[1] = 0x12;
        pkt->dataDynamic(var);
        //pkt->realSrc=src;
        //pkt->realDst=dest;

       
       //int t = pkt->getBE();
        //cout<<"Sending package"<<var[0]<<endl;


    }
    else
        pkt->dataDynamic(new uint8_t[req->getSize()]);
    pkt->senderState = NULL;
    numPseve++;
    numPacketsSent++;
    //cout<<"Sending package"<<pkt->getBE()<<endl;
    if (traffic == TASKGRAPH_)
    {
        numPreve++;
        numPacketsReceive++;
        outLinks[src][linkName].numberofPkt--;
        //needSend[id]--;
    }
    if(!isSDNoC)
        sendPkt(pkt);

}


//check path information
int checkPath(int src, int dst, Json::Value route)
{
    //int temp;
    //cin>>temp;
    int arrayNum = route.size();
    if(src==dst)
        return 0;
    if(arrayNum==0)
    {
        cout<<"route info missing! "<<endl;
        return 1;
    }
    for(int i=0;i<(arrayNum-1);++i)
    {
        int routerNum = route[i][0].asInt();
        string routerDirIn = route[i][1].asString();
        if(i==0 &&(src!=routerNum))
        {
            cout<<"source router error! "<<endl;
            return 2;
        }
        int nextNum=0;

        if(routerDirIn[0]=='W')
            nextNum = -1;
        else if (routerDirIn[0]=='E')
            nextNum = 1;
        else if (routerDirIn[0]=='N')
            nextNum = -rowNum;
        else if (routerDirIn[0]=='S')
            nextNum = rowNum;
        else{
            cout<<"Unknown Port "<<endl;
            return 3;
        }
        int nextRouter = route[i+1][0].asInt();
        if(nextRouter!=(routerNum+nextNum))
        {
            cout<<"router error! "<<routerNum <<" to "<<nextRouter<<endl;
            return 4;
        }
            
    }
    int routerNum = route[arrayNum-1][0].asInt();
    string routerDirIn = route[arrayNum-1][1].asString();
    int nextNum=0;

    if(routerDirIn[0]=='W')
        nextNum = -1;
    else if (routerDirIn[0]=='E')
        nextNum = 1;
    else if (routerDirIn[0]=='N')
        nextNum = -rowNum;
    else if (routerDirIn[0]=='S')
        nextNum = rowNum;
    else{
        cout<<"Unknown Port "<<endl;
        return 3;
    }
    if(dst!=(routerNum+nextNum))
    {
        
        cout<<"destination router error! "<<"From "<<src<<" through "<<routerNum<<nextNum<<"to"<<dst<<endl;
        return 5;
    }

    return 0;
}
// Changed here, adding candidates selection 
// check can send or not
// source, destination, path information, link name
int
GarnetSyntheticTraffic::generateSDNPkt(int src,unsigned dest,routeInfo onePath,string linkName)
{

    checkTimes++;
    //string linkName = to_string(src)+"+"+to_string(dest);
    Json::Value candi = candidates[linkName];
    int z = candi.size();
    Json::Value route;
    int checkSuccess = 1;
    unsigned realdst1 = onePath.dst;
    
    //Candidate selection
    for(int j = 0;j<z;j++){
        checkSuccess = 1;
        route = candi[j][3];
        
        int arrayNum = route.size();
        int pathCheckResult= checkPath(onePath.src, onePath.dst,route);
        if(pathCheckResult!=0)
        {
          
            exitSimLoop("Router error, exist.\n");
        }

        for(int i=0;i<arrayNum;++i)
        {
            int routerNum = route[i][0].asInt();
            string routerDirIn = route[i][1].asString();
            //real source onePath.src
            //real destination onePath.dst
            

            checkSuccess = CheckLink(routerNum,routerDirIn);
            //cout<<"check result "<<routerNum<<" "<<routerDirIn<<endl;
            if(!checkSuccess)
            {
                //cout<<"check result "<<routerNum<<" "<<routerDirIn<<endl;
                break;
            }

            // string routerDirOut = route[i][2].asString();
            // checkSuccess = CheckLink(routerNum,routerDirOut);
            // if(!checkSuccess)
            //     break;

        }
        if(checkSuccess)
        {

            outLinks[src][linkName].route = route;
            if(candi[j][4]<=2)
                outLinks[src][linkName].turns = 0;
            else
                outLinks[src][linkName].turns = 1;
            break;

        }
    }
    //checkSuccess=1;

    if(checkSuccess==1)
    {
        unsigned destination = realdst1;

        int source = id;


        Addr paddr =  destination;
        paddr <<= blockSizeBits;
        unsigned access_size = 1; 

        MemCmd::Command requestType;

        RequestPtr req = nullptr;
        Request::Flags flags;
        // Inject in specific Vnet
        // Vnet 0 and 1 are for control packets (1-flit)
        // Vnet 2 is for data packets (5-flit)
        int injReqType = injVnet;

        if (injReqType < 0 || injReqType > 2)
        {
            // randomly inject in any vnet
            injReqType = random_mt.random(0, 2);
        }

        if (injReqType == 0) {
            // generate packet for virtual network 0
            requestType = MemCmd::ReadReq;
            req = std::make_shared<Request>(paddr, access_size, flags, masterId);
        } else if (injReqType == 1) {
            // generate packet for virtual network 1
            requestType = MemCmd::ReadReq;
            flags.set(Request::INST_FETCH);
            req = std::make_shared<Request>(
                0, 0x0, access_size, flags, masterId, 0x0, 0);
            req->setPaddr(paddr);
        } else {  // if (injReqType == 2)
            // generate packet for virtual network 2
            requestType = MemCmd::WriteReq;
            req = std::make_shared<Request>(paddr, access_size, flags, masterId);
        }


        req->setContext(source);

        //Allocate the resource in the resource table

        int arrayNum = route.size();
        for(int i=0;i<arrayNum;++i)
        {
            int routerNum = route[i][0].asInt();
            string routerDirIn = route[i][1].asString();
            string routerDirOut = route[i][2].asString();
            DPRINTF(GarnetSyntheticTraffic," %d Reserve Links+++++++++++++++++++++++++++++++++ %s \n",routerNum,routerDirOut);
            ReserveLink(routerNum, routerDirIn,routerDirOut);
        }



        DPRINTF(GarnetSyntheticTraffic,
                "Generated packet with source %d, destination %d"
                ", embedded in address %x, packet size is %d\n",
                source, destination, req->getPaddr(), req->getSize());

        PacketPtr pkt = new Packet(req, requestType);
        if (traffic == TASKGRAPH_)
            pkt->dataDynamic(new uint8_t[1]);
        else
            pkt->dataDynamic(new uint8_t[req->getSize()]);
        pkt->senderState = NULL;
        numPseve++;
        numPacketsSent++;
        DPRINTF(GarnetSyntheticTraffic,"Sending package +++++++++++++++++++++++++++++++++  from %d to %d \n",src,dest);
        
        if (traffic == TASKGRAPH_)
        {
            numPreve++;
            numPacketsReceive++;
            outLinks[src][linkName].numberofPkt--;
            //receiveMatrix[dest]--;
            //needSend[id]--;
        }
        if(!isSDNoC)
            sendPkt(pkt);
       
        string linkName;

        linkName = to_string(src)+"+"+to_string(dest);
        //nowTransmit.insert(make_pair(linkName,onePath));

    }
    else
    {
        blockTime++;
        DPRINTF(GarnetSyntheticTraffic,"contention!!!!!!!!!!!!!!!!!!!!!  from %d to %d \n",src,dest);
      
    }
    
    return checkSuccess;
}

void
GarnetSyntheticTraffic::initTrafficType()
{
    trafficStringToEnum["bit_complement"] = BIT_COMPLEMENT_;
    trafficStringToEnum["bit_reverse"] = BIT_REVERSE_;
    trafficStringToEnum["bit_rotation"] = BIT_ROTATION_;
    trafficStringToEnum["neighbor"] = NEIGHBOR_;
    trafficStringToEnum["shuffle"] = SHUFFLE_;
    trafficStringToEnum["tornado"] = TORNADO_;
    trafficStringToEnum["transpose"] = TRANSPOSE_;
    trafficStringToEnum["uniform_random"] = UNIFORM_RANDOM_;
    trafficStringToEnum["taskgraph"] = TASKGRAPH_;

}

void
GarnetSyntheticTraffic::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
GarnetSyntheticTraffic::printAddr(Addr a)
{
    cachePort.printAddr(a);
}


GarnetSyntheticTraffic *
GarnetSyntheticTrafficParams::create()
{

   return new GarnetSyntheticTraffic(this);
}
