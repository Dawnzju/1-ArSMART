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


#include "mem/ruby/network/garnet2.0/InputUnit.hh"

#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/Credit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

InputUnit::InputUnit(int id, PortDirection direction, Router *router)
            : Consumer(router)
{
    m_id = id;
    m_direction = direction;
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();
    m_inject_vc_table.resize(m_num_vcs);
    m_invc_smart_table.resize(m_num_vcs);
    m_num_buffer_reads.resize(m_num_vcs/m_vc_per_vnet);
    m_num_buffer_writes.resize(m_num_vcs/m_vc_per_vnet);
    for (int i = 0; i < m_num_buffer_reads.size(); i++) {
        m_num_buffer_reads[i] = 0;
        m_num_buffer_writes[i] = 0;
    }

    creditQueue = new flitBuffer();
    // Instantiating the virtual channels
    m_vcs.resize(m_num_vcs);
    for (int i=0; i < m_num_vcs; i++) {
        m_vcs[i] = new VirtualChannel(i);
        m_inject_vc_table[i] = -1;
        m_invc_smart_table[i] = false;
    }
}

InputUnit::~InputUnit()
{
    delete creditQueue;
    deletePointers(m_vcs);
}

/*
 * The InputUnit wakeup function reads the input flit from its input link.
 * Each flit arrives with an input VC.
 * For HEAD/HEAD_TAIL flits, performs route computation,
 * and updates route in the input VC.
 * The flit is buffered for (m_latency - 1) cycles in the input VC
 * and marked as valid for SwitchAllocation starting that cycle.
 *
 */

void
InputUnit::wakeup()
{
    flit *t_flit;
    if (m_in_link->isReady(m_router->curCycle())) {
        int vc = -1;
        t_flit = m_in_link->consumeLink();
        //int vc = t_flit->get_vc();
        //new vc selection in SMART situation

        if (m_router->get_net_ptr()->isSMART()){
            switch(t_flit->get_type()){
                case HEAD_:
                    vc = t_flit->get_vc();
                    set_inject_vc_table(vc,
                    t_flit->get_route()->src_router);
                break;
                case BODY_:
                case TAIL_:
                //only Local flits needs to be send with VC
                    if (m_direction!="Local"){
                        vc = find_inject_vc_table(
                        t_flit->get_vnet(),
                        t_flit->get_route()->src_router);}
                    else{
                        vc = t_flit->get_vc();
                    }
                break;
                //case TAIL_:
                    //vc = find_inject_vc_table(
                    //    t_flit->get_vnet(),
                    //   t_flit->get_route()->src_router);
                    //set_inject_vc_table(vc,-1);
                    //move to router and free vc
                break;
                default:
                    vc = t_flit->get_vc();
                break;
            }
        }else{
            vc = t_flit->get_vc();

        }
        // Moved to crossbar
        // Counting hops as network hops
        // Not counting hops from/to NI
        //t_flit->increment_hops(); // for stats

    DPRINTF(RubyNetwork, "Router %d Inport %s received flit %s\n",
        m_router->get_id(), m_direction, *t_flit);



        if ((t_flit->get_type() == HEAD_) ||
            (t_flit->get_type() == HEAD_TAIL_)||
            ((m_router->get_net_ptr()->isSMART()) &&
            (m_vcs[vc]->get_state() == IDLE_) &&
            (vc != -1))){

            assert(m_vcs[vc]->get_state() == IDLE_);
            set_vc_active(vc, m_router->curCycle());

            // Route computation for this vc
            int outport;
            if(t_flit->get_route()->dest_router==t_flit->get_route()->src_router)
                outport=1;
            else
                outport = m_router->route_compute(t_flit->get_route(),
                    m_id, m_direction);
            set_outputPort(outport);
            // Update output port in VC
            // All flits in this packet will use this output port
            // The output port field in the flit is updated after it wins SA
            grant_outport(vc, outport);

        } else {
            //assert(m_vcs[vc]->get_state() == ACTIVE_);
            int outport = m_router->route_compute(t_flit->get_route(),
                m_id, m_direction);
            outport = outport + 1;

        }


        //Buffer the flit
        m_vcs[vc]->insertFlit(t_flit);

        int vnet = vc/m_vc_per_vnet;
        //number of writes same as reads
        //any flit that is written will be read only once
        m_num_buffer_writes[vnet]++;
        m_num_buffer_reads[vnet]++;

        Cycles pipe_stages = m_router->get_pipe_stages();



        if (pipe_stages == 1 || m_router->get_net_ptr()->isCentralControlled()) {
            // 1-cycle router
            // Flit goes for SA directly
            //Cycles a;
            //a= m_router->curCycle() - m_router->get_pipe_stages();

            //std::cout<<"InputUnit.cc +++++++++++++++"<<m_router->get_id()<<" my direction "<<m_direction<<"; current cycle = "<<a<<std::endl;

            t_flit->advance_stage(SA_, m_router->curCycle());
        } 
        else {
            //assert(pipe_stages > 1);
            // Router delay is modeled by making flit wait in buffer for
            // (pipe_stages cycles - 1) cycles before going for SA

            Cycles wait_time = pipe_stages - Cycles(1);
            t_flit->advance_stage(SA_, m_router->curCycle() + wait_time);

            // Wakeup the router in that cycle to perform SA
            m_router->schedule_wakeup(Cycles(wait_time));
        }
    }
}

// Send a credit back to upstream router for this VC.
// Called by SwitchAllocator when the flit in this VC wins the Switch.
void
InputUnit::increment_credit(int in_vc, bool free_signal, Cycles curTime)
{
    Credit *t_credit = new Credit(in_vc, free_signal, curTime + Cycles(1));
    creditQueue->insert(t_credit);
    m_credit_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
}


uint32_t
InputUnit::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (int i=0; i < m_num_vcs; i++) {
        num_functional_writes += m_vcs[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

void
InputUnit::resetStats()
{
    for (int j = 0; j < m_num_buffer_reads.size(); j++) {
        m_num_buffer_reads[j] = 0;
        m_num_buffer_writes[j] = 0;
    }
}

// SMART NoC
bool
InputUnit::try_smart_bypass(flit *t_flit)
{
    // Check if router is setup for SMART bypass this cycle
    DPRINTF(RubyNetwork, "Router %d Inport %s trying to bypass flit %s\n",
                 m_router->get_id(), m_direction, *t_flit);

//    PortDirection outport_dirn = t_flit->get_route()->outport_dirn;
//    return m_router->try_smart_bypass(m_id, outport_dirn, t_flit);            

    // Check SSR Grant for this cycle

    while (!ssr_grant.empty()) {
        SSR *t_ssr = ssr_grant.top();
        if (t_ssr->get_time() < m_router->curCycle()) {
            ssr_grant.pop();
            delete t_ssr;
        } else if (t_ssr->get_time() == m_router->curCycle()) {
            if (t_ssr->get_ref_flit() != t_flit) {
                // (i) this flit lost arbitration to a local flit, or
                // (ii) wanted to stop, and hence its SSR was not sent to 
                //      this router, and some other SSR won.
                //(iii) this flit is in a multi-flit transmitting which
                //is interrupted
                //to avoid being interrupted
                //during multi-flit sending
                //router expect :
                //1.Body/Tail flit
                //2.route NOT equal with before
                /*if (t_ssr->get_ref_flit()->get_route()->src_router!=
                    m_inject_vc_table[t_ssr->get_ref_flit()->get_vc()]
                    &&
                    (t_ssr->get_ref_flit()->get_type()!=BODY_||
                    t_ssr->get_ref_flit()->get_type()!=TAIL_)
                ){
                    ssr_grant.pop();
                    delete t_ssr;
                    continue;
                }
                    return false;*/
                break;
            } else {

                DPRINTF(RubyNetwork, "Router %d Inport %s Trying SMART Bypass for Flit %s\n",
                        m_router->get_id(), m_direction, *t_flit);

                // SSR for this flit won arbitration last cycle
                // and wants to bypass this router
                assert(t_ssr->get_bypass_req());
                //record smart vcs
                int vc;
                switch(t_flit->get_type()){
                    case HEAD_:
                        vc = t_flit->get_vc();
                        if (m_inject_vc_table[vc]!=-1)
                            return false;
                        set_inject_vc_table(vc,
                        t_flit->get_route()->src_router);
                    break;
                    case TAIL_:
                        vc = find_inject_vc_table(
                            t_flit->get_vnet(),
                            t_flit->get_route()->src_router);

                    break;
                    default:
                        vc = t_flit->get_vc();
                    break;
                }
                /*smart bypass try for
                1- Head flits/Head_Tail
                2- Body/Tail flits which invc_smart is true
                */
               bool smart_bypass;
               if ((t_flit->get_type()==HEAD_)||
               (t_flit->get_type()==HEAD_TAIL_)||
               (m_invc_smart_table[vc])){
                smart_bypass =
                    m_router->try_smart_bypass(m_id,
                                               t_ssr->get_outport_dirn(),
                                               t_flit);

                }else{
                    smart_bypass = false;
                }
                ssr_grant.pop();
                delete t_ssr;
                m_invc_smart_table[vc] = smart_bypass;
                //reset the smart table
                if ((t_flit->get_type()==TAIL_)||
               (t_flit->get_type()==HEAD_TAIL_)){
                   m_invc_smart_table[vc] = false;
               }
                return smart_bypass;
            }
        }

         else {
            break;
        }
    }

    m_invc_smart_table[t_flit->get_vc()] = false;
    return false;
}


//Central Controlled bypass
bool
InputUnit::try_central_bypass(flit *t_flit)
{
    // bypass this cycle
//    PortDirection outport_dirn = t_flit->get_route()->outport_dirn;
//    return m_router->try_smart_bypass(m_id, outport_dirn, t_flit);            
    // Check SSR Grant for this cycle
    std::vector<int> outputlist;
    int output1=m_router->route_compute(t_flit->get_route(), m_id, m_direction);

    if(output1==0)
        //return false;
        output1=1;

   // std::cout<<"Router "<<m_router->get_id()<<" bypass flit "<<"from "<<t_flit->get_route()->src_router<<" to "<<t_flit->get_route()->dest_router<<" at cycle "<<m_router->curCycle()<<std::endl;

  

    DPRINTF(RubyNetwork, "Router %d Inport %s Trying SMART Bypass for Flit %s\n",
            m_router->get_id(), m_direction, *t_flit);

    // SSR for this flit won arbitration last cycle
    // and wants to bypass this router

    //record smart vcs
    int vc;
    switch(t_flit->get_type()){
        case HEAD_:
            vc = t_flit->get_vc();
            if (m_inject_vc_table[vc]!=-1)
                return false;
            set_inject_vc_table(vc,
            t_flit->get_route()->src_router);
        break;
        case TAIL_:
            vc = find_inject_vc_table(
                t_flit->get_vnet(),
                t_flit->get_route()->src_router);

        break;
        default:
            vc = t_flit->get_vc();
        break;
    }
    /*smart bypass try for
    1- Head flits/Head_Tail
    2- Body/Tail flits which invc_smart is true
    */
   //std::cout<<get_outputPort()<<"+++++++++++++++"<<std::endl;
   bool smart_bypass;
   if ((t_flit->get_type()==HEAD_)||
   (t_flit->get_type()==HEAD_TAIL_)){
    smart_bypass =
        m_router->try_central_bypass(m_id,
                                   output1,
                                   t_flit);

    }else{
        smart_bypass = false;
    }
    

    return smart_bypass;
}

void
InputUnit::grantSSR(SSR *t_ssr)
{           
    DPRINTF(RubyNetwork, "Router %d Inport %s granted SSR for flit %d from src_hops %d for bypass = %d for Outport %s\n",
            m_router->get_id(), m_direction, *(t_ssr->get_ref_flit()), t_ssr->get_src_hops(), t_ssr->get_bypass_req(), t_ssr->get_outport_dirn());

    // Update valid time to next cycle
    t_ssr->set_time(m_router->curCycle() + Cycles(1));        
    ssr_grant.push(t_ssr);
}

//find vc by injector
int
InputUnit::find_inject_vc_table(int vnet,int injector){
    int iter;
    int vc_base = vnet * m_vc_per_vnet;
    for (iter = vc_base; iter < vc_base + m_vc_per_vnet; iter++){
        if (m_inject_vc_table[iter]==injector){
                return iter;
            }

    }
    /*assert when did not find anything */
    //assert(iter < m_num_vcs);

    return -1;
}

int
InputUnit::select_free_vc(int vnet)
{
    int vc_base = vnet*m_vc_per_vnet;
    for (int vc = vc_base; vc < vc_base + m_vc_per_vnet; vc++) {
        if (m_inject_vc_table[vc]==-1) {
            return vc;
        }
    }

    return -1;
}
