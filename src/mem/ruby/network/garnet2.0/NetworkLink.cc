/*
 * Copyright (c) 2008 Princeton University
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
 * Authors: Niket Agarwal
 *          Tushar Krishna
 */


#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"

NetworkLink::NetworkLink(const Params *p)
    : ClockedObject(p), Consumer(this), m_id(p->link_id),
      m_type(NUM_LINK_TYPES_),
      m_latency(p->link_latency),
      linkBuffer(new flitBuffer()), link_consumer(nullptr),
      link_srcQueue(nullptr), m_link_utilized(0),
      m_vc_load(p->vcs_per_vnet * p->virt_nets)
{
}

NetworkLink::~NetworkLink()
{
    delete linkBuffer;
}

void
NetworkLink::setLinkConsumer(Consumer *consumer)
{
    link_consumer = consumer;
}

void
NetworkLink::setLinkConsumerInport(InputUnit *inport)
{
    link_consumer_inport = inport;
}


void
NetworkLink::setSourceQueue(flitBuffer *srcQueue)
{
    link_srcQueue = srcQueue;
}

void
NetworkLink::wakeup()
{
    //std::cout<<"NetworkLink.cc wakeup++++++++++++++++++++++++++"<<curCycle()<<std::endl;

    if (link_srcQueue->isReady(curCycle())) {
        //change here
        // if(syntheticPattern==8 &&(isSDNoC)){
        //     int totalFlit = link_srcQueue->getSize();
        //     for(int i = 0;i< totalFlit;++i)
        //     {
        //         flit *t_flit = link_srcQueue->getTopFlit();
        //         if(t_flit->get_type() == TAIL_ || t_flit->get_type() == HEAD_TAIL_)
        //         {
        //             std::cout<<"Router "<<m_id<<" receives a packet!!!"<<t_flit->get_type()<<" "<<t_flit->get_route()->src_router<<" to "<<t_flit->get_route()->dest_router<<" "<<curCycle()<<std::endl;
        //             //if it is the 
        //             if(t_flit->get_route()->src_router==t_flit->get_route()->dest_router)
        //             {
        //                 Json::Value t = multiOutput[m_id];
        //                 int a =t.size();
        //                 for(int i=0;i<a;i++)
        //                 {
        //                     receiveMatrix[t[i].asInt()]--;
        //                 }

        //             }
        //             else
        //             {
        //                 receiveMatrix[int(t_flit->get_route()->dest_router)]--;
        //             }
                    
        //             std::string linkname;

        //             linkname = to_string(t_flit->get_route()->src_router)+"+"+to_string(t_flit->get_route()->dest_router);
        //             allLinks[linkname]--;
        //         }
        //     }

            
        // }
        // else{
            flit *t_flit = link_srcQueue->getTopFlit();
                
            t_flit->set_time(curCycle());
            m_link_utilized++;
            m_vc_load[t_flit->get_vc()]++;

            bool router_bypass = false;
            if (m_type == INT_) {

                // SMART NoC:
                if (link_consumer_inport->get_net_ptr()->isSMART()) {
                    // See if router is set to bypass mode for this flit
                    // If it is, then flit will bypass and go to next link
                    t_flit->set_time(curCycle());
                    router_bypass = link_consumer_inport->try_smart_bypass(t_flit);
                }
                //Central Bypass
                if (link_consumer_inport->get_net_ptr()->isCentralControlled()){

                    
                    t_flit->set_time(curCycle());
                    router_bypass = link_consumer_inport->try_central_bypass(t_flit);
                    router_bypass = 1;

                    //linkBuffer->insert(t_flit);
                    //link_consumer->scheduleEventAbsolute(clockEdge(Cycles(0)));
                }
            }
            //std::cout<<"router bypass this cycle "<<curCycle()<<" "<<router_bypass<<std::endl;
            if (!router_bypass) {
                // Baseline, or
                // SMART where flit has to be buffered at next router 
                // because it is turning/dest router, or SSR lost
                t_flit->set_time(curCycle() + m_latency);
                linkBuffer->insert(t_flit);
                link_consumer->scheduleEventAbsolute(clockEdge(m_latency));
            }
        }
//    }

    // Two flits cannot be ready for LT at the same time
    // Assertion added to make sure SMART not getting
    // higher bandwidth unfairly
    assert(!link_srcQueue->isReady(curCycle()));
}

void
NetworkLink::resetStats()
{
    for (int i = 0; i < m_vc_load.size(); i++) {
        m_vc_load[i] = 0;
    }

    m_link_utilized = 0;
}

NetworkLink *
NetworkLinkParams::create()
{
    return new NetworkLink(this);
}

CreditLink *
CreditLinkParams::create()
{
    return new CreditLink(this);
}

uint32_t
NetworkLink::functionalWrite(Packet *pkt)
{
    return linkBuffer->functionalWrite(pkt);
}
