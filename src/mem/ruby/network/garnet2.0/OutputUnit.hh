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


#ifndef __MEM_RUBY_NETWORK_GARNET2_0_OUTPUTUNIT_HH__
#define __MEM_RUBY_NETWORK_GARNET2_0_OUTPUTUNIT_HH__

#include <iostream>
#include <vector>
#include <queue>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/OutVcState.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"
#include "mem/ruby/network/garnet2.0/flitBuffer.hh"

class OutputUnit : public Consumer
{
  public:
    OutputUnit(int id, PortDirection direction, Router *router);
    ~OutputUnit();
    void set_out_link(NetworkLink *link);
    void set_credit_link(CreditLink *credit_link);
    void wakeup();
    flitBuffer* getOutQueue();
    void print(std::ostream& out) const {};
    void decrement_credit(int out_vc);
    void increment_credit(int out_vc);
    bool has_credit(int out_vc);
    bool has_free_vc(int vnet);
    int select_free_vc(int vnet,flit* t_flit);

    inline PortDirection get_direction() { return m_direction; }

    int
    get_credit_count(int vc)
    {
        return m_outvc_state[vc]->get_credit_count();
    }
    void
    set_pkt_route(int src,int dest,int invc)
    {
        m_pkt_route_src= src;
        m_pkt_route_dest= dest;
        m_pkt_invc = invc;
    }

    int
    get_curr_pkt_src()
    {
        return m_pkt_route_src;
    }
    int
    get_curr_pkt_dest()
    {
        return m_pkt_route_dest;
    }
    int
    get_curr_pkt_invc()
    {
        return m_pkt_invc;
    }
    inline int
    get_outlink_id()
    {
        return m_out_link->get_id();
    }

    inline void
    set_vc_state(VC_state_type state, int vc, Cycles curTime)
    {
      m_outvc_state[vc]->setState(state, curTime);
    }

    inline bool
    is_vc_idle(int vc, Cycles curTime)
    {
        return (m_outvc_state[vc]->isInState(IDLE_, curTime));
    }

    inline void
    insert_flit(flit *t_flit)
    {
        m_out_buffer->insert(t_flit);
        m_out_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
    }

    uint32_t functionalWrite(Packet *pkt);

    // SMART NoC
    void insertSSR(SSR *t_ssr);
    bool isReadySSR();
    SSR* getTopSSR();
    void clearSSRreqs();
    void smart_bypass(flit *t_flit);
    void central_bypass(flit *t_flit);
  private:
    int m_id;
    PortDirection m_direction;
    int m_pkt_route_src,m_pkt_route_dest,m_pkt_invc;
    int m_num_vcs;
    int m_vc_per_vnet;
    Router *m_router;
    NetworkLink *m_out_link;
    CreditLink *m_credit_link;

    flitBuffer *m_out_buffer; // This is for the network link to consume
    std::vector<OutVcState *> m_outvc_state; // vc state of downstream router

    // SMART
    struct SSR_prio_local
    {
        bool operator()(const SSR *lhs, const SSR *rhs) const
        {
            if (lhs->m_time == rhs->m_time)
                // reverse the following for prio_bypass
                return lhs->m_src_hops > rhs->m_src_hops;
            else
                return lhs->m_time > rhs->m_time;
        }
    };

    std::priority_queue<SSR*, std::vector<SSR*>, SSR_prio_local> ssr_reqs;
    //std::priority_queue<SSR*, std::vector<SSR*>, SSR_prio_local> ssr_grant;
};

#endif // __MEM_RUBY_NETWORK_GARNET2_0_OUTPUTUNIT_HH__
