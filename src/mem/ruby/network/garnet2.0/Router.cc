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


#include "mem/ruby/network/garnet2.0/Router.hh"

#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/CrossbarSwitch.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/OutputUnit.hh"
#include "mem/ruby/network/garnet2.0/RoutingUnit.hh"
#include "mem/ruby/network/garnet2.0/SwitchAllocator.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

/*Router::Router(const Params *p)
    : BasicRouter(p), Consumer(this)
{
    m_latency = p->latency;
    m_virtual_networks = p->virt_nets;
    m_vc_per_vnet = p->vcs_per_vnet;
    m_num_vcs = m_virtual_networks * m_vc_per_vnet;
    m_routing_unit = new RoutingUnit(this);
    m_sw_alloc = new SwitchAllocator(this);
    m_switch = new CrossbarSwitch(this);

    m_input_unit.clear();
    m_output_unit.clear();
}

Router::~Router()
{
    deletePointers(m_input_unit);
    deletePointers(m_output_unit);
    delete m_routing_unit;
    delete m_sw_alloc;
    delete m_switch;
}

void
Router::init()
{
    BasicRouter::init();

    m_sw_alloc->init();
    m_switch->init();
}

void
Router::wakeup()
{
    DPRINTF(RubyNetwork, "Router %d woke up\n", m_id);

    // check for incoming flits
    for (int inport = 0; inport < m_input_unit.size(); inport++) {
        m_input_unit[inport]->wakeup();
    }

    // check for incoming credits
    // Note: the credit update is happening before SA
    // buffer turnaround time =
    //     credit traversal (1-cycle) + SA (1-cycle) + Link Traversal (1-cycle)
    // if we want the credit update to take place after SA, this loop should
    // be moved after the SA request
    for (int outport = 0; outport < m_output_unit.size(); outport++) {
        m_output_unit[outport]->wakeup();
    }

    // Switch Allocation
    m_sw_alloc->wakeup();

    // Switch Traversal
    m_switch->wakeup();
}

void
Router::addInPort(PortDirection inport_dirn,
                  NetworkLink *in_link, CreditLink *credit_link)
{
    int port_num = m_input_unit.size();
    InputUnit *input_unit = new InputUnit(port_num, inport_dirn, this);

    input_unit->set_in_link(in_link);
    input_unit->set_credit_link(credit_link);
    in_link->setLinkConsumer(this);
    in_link->setLinkConsumerInport(input_unit);
    credit_link->setSourceQueue(input_unit->getCreditQueue());

    m_input_unit.push_back(input_unit);

    m_routing_unit->addInDirection(inport_dirn, port_num);
}

void
Router::addOutPort(PortDirection outport_dirn,
                   NetworkLink *out_link,
                   const NetDest& routing_table_entry, int link_weight,
                   CreditLink *credit_link)
{
    int port_num = m_output_unit.size();
    OutputUnit *output_unit = new OutputUnit(port_num, outport_dirn, this);

    output_unit->set_out_link(out_link);
    output_unit->set_credit_link(credit_link);
    credit_link->setLinkConsumer(this);
    out_link->setSourceQueue(output_unit->getOutQueue());

    m_output_unit.push_back(output_unit);

    m_routing_unit->addRoute(routing_table_entry);
    m_routing_unit->addWeight(link_weight);
    m_routing_unit->addOutDirection(outport_dirn, port_num);
}

PortDirection
Router::getOutportDirection(int outport)
{
    return m_output_unit[outport]->get_direction();
}

PortDirection
Router::getInportDirection(int inport)
{
    return m_input_unit[inport]->get_direction();
}

int
Router::route_compute(RouteInfo *route, int inport, PortDirection inport_dirn)
{
    return m_routing_unit->outportCompute(route, inport, inport_dirn);
}

void
Router::grant_switch(int inport, flit *t_flit)
{
    m_switch->update_sw_winner(inport, t_flit);
}

void
Router::schedule_wakeup(Cycles time)
{
    // wake up after time cycles
    scheduleEvent(time);
}

std::string
Router::getPortDirectionName(PortDirection direction)
{
    // PortDirection is actually a string
    // If not, then this function should add a switch
    // statement to convert direction to a string
    // that can be printed out
    return direction;
}

void
Router::insertSSR(PortDirection inport_dirn, SSR* t_ssr)
{
    DPRINTF(RubyNetwork, "Router %d Inport %s received SSR from src_hops %d for bypass = %d for Outport %s\n",
            get_id(), inport_dirn, t_ssr->get_src_hops(), t_ssr->get_bypass_req(), t_ssr->get_outport_dirn());

    int inport = m_routing_unit->getInportIdx(inport_dirn);
    int outport = m_routing_unit->getOutportIdx(t_ssr->get_outport_dirn());

    t_ssr->set_inport(inport);

    // Update SSR if dest bypass enabled
    if (t_ssr->get_src_hops() > 0 && !t_ssr->get_bypass_req()) {
        // dest or turning router
        RouteInfo* route = t_ssr->get_ref_flit()->get_route();
        bool is_dest = (route->dest_router == m_id);

        if (get_net_ptr()->isSMARTdestBypass() && is_dest) {
            outport = m_routing_unit->lookupRoutingTable(route->vnet, route->net_dest);
            PortDirection outport_dirn = m_output_unit[outport]->get_direction();

            t_ssr->set_outport_dirn(outport_dirn);
            t_ssr->set_bypass_req(true);
        } else {
            delete t_ssr;
            return;
        }
    }
    m_output_unit[outport]->insertSSR(t_ssr);
}

bool
Router::smart_vc_select(int inport, int outport, flit *t_flit)
{
    // VC Selection
    int vnet = t_flit->get_vnet();
    bool has_free_vc = m_output_unit[outport]->has_free_vc(vnet);
    if (!has_free_vc)
        return false;

    // Update VC in flit
    int invc = t_flit->get_vc();
    int outvc;
    switch(t_flit->get_type()){
        case HEAD_:case HEAD_TAIL_:
            outvc = m_output_unit[outport]->select_free_vc(vnet,t_flit);
            m_input_unit[inport]->set_inject_vc_table(invc,
            t_flit->get_route()->src_router);
            m_output_unit[outport]->set_inject_vc_table(outvc,
            t_flit->get_route()->src_router);
            m_input_unit[inport]->grant_outvc(invc, outvc);
        break;
        case BODY_:case TAIL_:
            outvc = m_input_unit[inport]->get_outvc(invc);
        break;
        default:

        break;
    }

    if (outvc == -1 || outvc == -2 )
        return false;
    else if (!m_output_unit[outport]->has_credit(outvc))
    // if don't have credit then return false
        return false;
    else
        t_flit->set_vc(outvc);
    m_output_unit[outport]->decrement_credit(outvc);

    // Send credit for VCid flit came with
    // Verify input VC is free and IDLE
    assert(!(m_input_unit[inport]->isReady(invc, curCycle())));
    //assert(m_input_unit[inport]->is_vc_idle(invc));

    // Send credit for VCid flit came with
    switch(t_flit->get_type()){
        case TAIL_:case HEAD_TAIL_:
            m_output_unit[outport]->set_inject_vc_table(outvc,-1);
            m_input_unit[inport]->grant_outvc(invc, -1);
            m_input_unit[inport]->set_vc_idle(invc,curCycle());
            m_input_unit[inport]->increment_credit(invc, true, curCycle());
        break;
        default:
            m_input_unit[inport]->increment_credit(invc, false, curCycle());
        break;
    }


    return true;
}

void
Router::smart_route_update(int inport, int outport, flit* t_flit)
{
    // Update route in flit
    // Call route_compute so that x_hops/y_hops decremented
    int check_outport = route_compute(t_flit->get_route(),
        inport, m_input_unit[inport]->get_direction());
    assert(check_outport == outport);
    t_flit->set_outport(outport);
}

bool
Router::try_smart_bypass(int inport, PortDirection outport_dirn, flit *t_flit)
{
    int outport = m_routing_unit->getOutportIdx(outport_dirn);
    //Check outport
    //if outport is in use then
    //refuse head or head tail flit
    if (m_sw_alloc->get_outport_use(outport)&&
    ((t_flit->get_type()==HEAD_) ||
    (t_flit->get_type()==HEAD_TAIL_)))
        return false;
    // Update VC
    bool has_vc = smart_vc_select(inport, outport, t_flit);
    if (!has_vc)
        return false;

    // Update Route
    smart_route_update(inport, outport, t_flit);

    DPRINTF(RubyNetwork, "Router %d Inport %s and Outport %d successful SMART Bypass for Flit %s\n",
            get_id(), getPortDirectionName(m_input_unit[inport]->get_direction()), outport_dirn, *t_flit);

    //set outport status
    // one outport is free when tail or head_tail_

    //set outport pkt route
    //1.when head_ set route src,dest
    //2.when tail_ head_tail_ set 0
    switch(t_flit->get_type()){
        case HEAD_:
            m_output_unit[outport]->set_pkt_route(
                t_flit->get_route()->src_router,
                t_flit->get_route()->dest_router
            );
        case BODY_:
            m_sw_alloc->set_outport_in_use(outport,true);
            break;
        case TAIL_:
        case HEAD_TAIL_:
            m_sw_alloc->set_outport_in_use(outport,false);
            m_output_unit[outport]->set_pkt_route(-1,-1);
            break;
        default:
            break;
    }
    // Add flit to output link
    m_output_unit[outport]->smart_bypass(t_flit);
    t_flit->increment_hops(); // for stats

    return true;
}
 
void
Router::regStats()
{
    BasicRouter::regStats();

    m_buffer_reads
        .name(name() + ".buffer_reads")
        .flags(Stats::nozero)
    ;

    m_buffer_writes
        .name(name() + ".buffer_writes")
        .flags(Stats::nozero)
    ;

    m_crossbar_activity
        .name(name() + ".crossbar_activity")
        .flags(Stats::nozero)
    ;

    m_sw_input_arbiter_activity
        .name(name() + ".sw_input_arbiter_activity")
        .flags(Stats::nozero)
    ;

    m_sw_output_arbiter_activity
        .name(name() + ".sw_output_arbiter_activity")
        .flags(Stats::nozero)
    ;
}

void
Router::collateStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_buffer_reads += m_input_unit[i]->get_buf_read_activity(j);
            m_buffer_writes += m_input_unit[i]->get_buf_write_activity(j);
        }
    }

    m_sw_input_arbiter_activity = m_sw_alloc->get_input_arbiter_activity();
    m_sw_output_arbiter_activity = m_sw_alloc->get_output_arbiter_activity();
    m_crossbar_activity = m_switch->get_crossbar_activity();
}

void
Router::resetStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_input_unit[i]->resetStats();
        }
    }

    m_switch->resetStats();
    m_sw_alloc->resetStats();
}

void
Router::printFaultVector(ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    int num_fault_types = m_network_ptr->fault_model->number_of_fault_types;
    float fault_vector[num_fault_types];
    get_fault_vector(temperature_celcius, fault_vector);
    out << "Router-" << m_id << " fault vector: " << endl;
    for (int fault_type_index = 0; fault_type_index < num_fault_types;
         fault_type_index++) {
        out << " - probability of (";
        out <<
        m_network_ptr->fault_model->fault_type_to_string(fault_type_index);
        out << ") = ";
        out << fault_vector[fault_type_index] << endl;
    }
}

void
Router::printAggregateFaultProbability(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    float aggregate_fault_prob;
    get_aggregate_fault_probability(temperature_celcius,
                                    &aggregate_fault_prob);
    out << "Router-" << m_id << " fault probability: ";
    out << aggregate_fault_prob << endl;
}

uint32_t
Router::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    num_functional_writes += m_switch->functionalWrite(pkt);

    for (uint32_t i = 0; i < m_input_unit.size(); i++) {
        num_functional_writes += m_input_unit[i]->functionalWrite(pkt);
    }

    for (uint32_t i = 0; i < m_output_unit.size(); i++) {
        num_functional_writes += m_output_unit[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

Router *
GarnetRouterParams::create()
{
    return new Router(this);
}*/

Router::Router(const Params *p)
    : BasicRouter(p), Consumer(this)
{
    m_latency = p->latency;
    m_virtual_networks = p->virt_nets;
    m_vc_per_vnet = p->vcs_per_vnet;
    m_num_vcs = m_virtual_networks * m_vc_per_vnet;
    m_routing_unit = new RoutingUnit((Router*)this);
    m_sw_alloc = new SwitchAllocator((Router*)this);
    m_switch = new CrossbarSwitch((Router*)this);

    m_input_unit.clear();
    m_output_unit.clear();
}

Router::~Router()
{
    deletePointers(m_input_unit);
    deletePointers(m_output_unit);
    delete m_routing_unit;
    delete m_sw_alloc;
    delete m_switch;
}

void
Router::init()
{
    BasicRouter::init();
    m_curTask= nullptr;
    m_sw_alloc->init();
    m_switch->init();
}

void
Router::wakeup()
{
    DPRINTF(RubyNetwork, "Router %d woke up\n", m_id);
    //update task
    // if (m_network_ptr->isCentralControlled())
    //     update_task();
    // check for incoming flits
    //std::cout<<"->here Router wake up at cycle "<<m_id<<curCycle()<<std::endl;
    for (int inport = 0; inport < m_input_unit.size(); inport++) {
        m_input_unit[inport]->wakeup();
    }

    // check for incoming credits
    // Note: the credit update is happening before SA
    // buffer turnaround time =
    //     credit traversal (1-cycle) + SA (1-cycle) + Link Traversal (1-cycle)
    // if we want the credit update to take place after SA, this loop should
    // be moved after the SA request
    for (int outport = 0; outport < m_output_unit.size(); outport++) {
        m_output_unit[outport]->wakeup();
    }

    // Switch Allocation
    m_sw_alloc->wakeup();

    // Switch Traversal
    m_switch->wakeup();
    
    // if (m_network_ptr->isCentralControlled()){
    //     schedule_wakeup(Cycles(1));
    //     if (m_id < m_network_ptr->getNumRouters()-1)
    //         m_network_ptr->wakeupRouter(m_id+1);
    // }
}

void
Router::addInPort(PortDirection inport_dirn,
                  NetworkLink *in_link, CreditLink *credit_link)
{
    int port_num = m_input_unit.size();
    InputUnit *input_unit = new InputUnit(port_num, inport_dirn,
    (Router*)this);

    input_unit->set_in_link(in_link);
    input_unit->set_credit_link(credit_link);
    in_link->setLinkConsumer(this);
    in_link->setLinkConsumerInport(input_unit);
    credit_link->setSourceQueue(input_unit->getCreditQueue());

    m_input_unit.push_back(input_unit);

    m_routing_unit->addInDirection(inport_dirn, port_num);
}

void
Router::addOutPort(PortDirection outport_dirn,
                   NetworkLink *out_link,
                   const NetDest& routing_table_entry, int link_weight,
                   CreditLink *credit_link)
{
    int port_num = m_output_unit.size();
    OutputUnit *output_unit = new OutputUnit(port_num, outport_dirn,
    (Router*)this);

    output_unit->set_out_link(out_link);
    output_unit->set_credit_link(credit_link);
    credit_link->setLinkConsumer(this);
    out_link->setSourceQueue(output_unit->getOutQueue());

    m_output_unit.push_back(output_unit);

    m_routing_unit->addRoute(routing_table_entry);
    m_routing_unit->addWeight(link_weight);
    m_routing_unit->addOutDirection(outport_dirn, port_num);
}

PortDirection
Router::getOutportDirection(int outport)
{
    return m_output_unit[outport]->get_direction();
}

PortDirection
Router::getInportDirection(int inport)
{
    return m_input_unit[inport]->get_direction();
}

int
Router::route_compute(RouteInfo *route, int inport,
PortDirection inport_dirn)
{
    return m_routing_unit->outportCompute(route, inport, inport_dirn);
}

void
Router::route_central_compute(RouteInfo *route, int inport,
PortDirection inport_dirn, std::vector<int> *outputlist)
{
    m_routing_unit->outportCompute(route, inport, inport_dirn);//outputCentralCompute(route, inport, inport_dirn, outputlist);
}



void
Router::grant_switch(int inport, flit *t_flit)
{
    m_switch->update_sw_winner(inport, t_flit);
}

void
Router::schedule_wakeup(Cycles time)
{
    // wake up after time cycles
    scheduleEvent(time);
}

std::string
Router::getPortDirectionName(PortDirection direction)
{
    // PortDirection is actually a string
    // If not, then this function should add a switch
    // statement to convert direction to a string
    // that can be printed out
    return direction;
}

void
Router::insertSSR(PortDirection inport_dirn, SSR* t_ssr)
{
    DPRINTF(RubyNetwork,
    "Router %d Inport %s received SSR from src_hops "
     "%d for bypass = %d for Outport %s\n",
            get_id(), inport_dirn, t_ssr->get_src_hops(),
            t_ssr->get_bypass_req(), t_ssr->get_outport_dirn());

    int inport = m_routing_unit->getInportIdx(inport_dirn);
    int outport = m_routing_unit->getOutportIdx(
        t_ssr->get_outport_dirn());

    t_ssr->set_inport(inport);

    // Update SSR if dest bypass enabled
    if (t_ssr->get_src_hops() > 0 && !t_ssr->get_bypass_req()) {
        // dest or turning Router
        RouteInfo* route = t_ssr->get_ref_flit()->get_route();
        bool is_dest = (route->dest_router == m_id);

        if (get_net_ptr()->isSMARTdestBypass() && is_dest) {
            outport = m_routing_unit->lookupRoutingTable(
                route->vnet, route->net_dest);
            PortDirection outport_dirn = m_output_unit[outport]
            ->get_direction();

            t_ssr->set_outport_dirn(outport_dirn);
            t_ssr->set_bypass_req(true);
        } else {
            delete t_ssr;
            return;
        }
    }
    m_output_unit[outport]->insertSSR(t_ssr);
}

bool
Router::smart_vc_select(int inport, int outport, flit *t_flit)
{
    // VC Selection
    int vnet = t_flit->get_vnet();
    bool has_free_vc = m_output_unit[outport]->has_free_vc(vnet);
    if (!has_free_vc)
        return false;

    // Update VC in flit
    int invc = t_flit->get_vc();
    int outvc;
    switch(t_flit->get_type()){
        case HEAD_:case HEAD_TAIL_:
            outvc = m_output_unit[outport]->select_free_vc(vnet,t_flit);
            m_input_unit[inport]->set_inject_vc_table(invc,
            t_flit->get_route()->src_router);
            //m_output_unit[outport]->set_inject_vc_table(outvc,
            //t_flit->get_route()->src_router);
            m_input_unit[inport]->grant_outvc(invc, outvc);
        break;
        case BODY_:case TAIL_:
            outvc = m_input_unit[inport]->get_outvc(invc);
        break;
        default:

        break;
    }

    if (outvc == -1 || outvc == -2 )
        return false;
    else if (!m_output_unit[outport]->has_credit(outvc))
    // if don't have credit then return false
        return false;
    else
        t_flit->set_vc(outvc);
    m_output_unit[outport]->decrement_credit(outvc);

    // Send credit for VCid flit came with
    // Verify input VC is free and IDLE
    assert(!(m_input_unit[inport]->isReady(invc, curCycle())));
    //assert(m_input_unit[inport]->is_vc_idle(invc));

    // Send credit for VCid flit came with
    switch(t_flit->get_type()){
        case TAIL_:case HEAD_TAIL_:
            //m_output_unit[outport]->set_inject_vc_table(outvc,-1);
            m_input_unit[inport]->grant_outvc(invc, -1);
            m_input_unit[inport]->set_vc_idle(invc,curCycle());
            m_input_unit[inport]->increment_credit(invc, true,
             curCycle());
        break;
        default:
            m_input_unit[inport]->increment_credit(invc, false,
             curCycle());
        break;
    }


    return true;
}

void
Router::smart_route_update(int inport, int outport,
flit* t_flit)
{
    // Update route in flit
    // Call route_compute so that x_hops/y_hops decremented
    int check_outport = route_compute(t_flit->get_route(),
        inport, m_input_unit[inport]->get_direction());
    assert(check_outport == outport);
    t_flit->set_outport(outport);
}

bool
Router::try_smart_bypass(int inport,
PortDirection outport_dirn, flit *t_flit)
{
    int outport = m_routing_unit->getOutportIdx(outport_dirn);
    //Check outport
    //if outport is in use then
    //refuse head or head tail flit
    if (m_sw_alloc->get_outport_use(outport)&&
    ((t_flit->get_type()==HEAD_) ||
    (t_flit->get_type()==HEAD_TAIL_)))
        return false;
    // Update VC
    bool has_vc = smart_vc_select(inport, outport, t_flit);
    if (!has_vc)
        return false;

    // Update Route
    smart_route_update(inport, outport, t_flit);

    DPRINTF(RubyNetwork, "Router %d Inport %s and \
    Outport %d successful SMART Bypass for Flit %s\n",
            get_id(), getPortDirectionName(
                m_input_unit[inport]->get_direction()), outport_dirn,
                 *t_flit);

    //set outport status
    // one outport is free when tail or head_tail_

    //set outport pkt route
    //1.when head_ set route src,dest
    //2.when tail_ head_tail_ set 0
    switch(t_flit->get_type()){
        case HEAD_:
            m_output_unit[outport]->set_pkt_route(
                t_flit->get_route()->src_router,
                t_flit->get_route()->dest_router,
                t_flit->get_vc()
            );
        case BODY_:
            m_sw_alloc->set_outport_in_use(outport,true);
            break;
        case TAIL_:
        case HEAD_TAIL_:
            m_sw_alloc->set_outport_in_use(outport,false);
            m_output_unit[outport]->set_pkt_route(-1,-1,-1);
            break;
        default:
            break;
    }
    // Add flit to output link
    m_output_unit[outport]->smart_bypass(t_flit);
    t_flit->increment_hops(); // for stats

    return true;
}
//Process dest node
bool
Router::try_central_bypass(int inport,
int outport_dirn, flit *t_flit)
{
    int outport = outport_dirn;
    if(outport==0)
        //return false;
        outport=1;


    DPRINTF(RubyNetwork, "Router %d Inport %s and \
    Outport %d successful SMART Bypass for Flit %s\n",
            get_id(), getPortDirectionName(
                m_input_unit[inport]->get_direction()), outport_dirn,
                 *t_flit);

    //set outport status
    // one outport is free when tail or head_tail_

    //set outport pkt route
    //1.when head_ set route src,dest
    //2.when tail_ head_tail_ set 0
    switch(t_flit->get_type()){
        case HEAD_:
            m_output_unit[outport]->set_pkt_route(
                t_flit->get_route()->src_router,
                t_flit->get_route()->dest_router,
                t_flit->get_vc()
            );
        case BODY_:
            m_sw_alloc->set_outport_in_use(outport,true);
            break;
        case TAIL_:
        case HEAD_TAIL_:
            m_sw_alloc->set_outport_in_use(outport,false);
            m_output_unit[outport]->set_pkt_route(-1,-1,-1);
            break;
        default:
            break;
    }
    // Add flit to output central_bypass
    m_output_unit[outport]->central_bypass(t_flit);
    t_flit->increment_hops(); // for stats
    //receive the packet

  
    return true;
}

void
Router::regStats()
{
    BasicRouter::regStats();

    m_buffer_reads
        .name(name() + ".buffer_reads")
        .flags(Stats::nozero)
    ;

    m_buffer_writes
        .name(name() + ".buffer_writes")
        .flags(Stats::nozero)
    ;

    m_crossbar_activity
        .name(name() + ".crossbar_activity")
        .flags(Stats::nozero)
    ;

    m_sw_input_arbiter_activity
        .name(name() + ".sw_input_arbiter_activity")
        .flags(Stats::nozero)
    ;

    m_sw_output_arbiter_activity
        .name(name() + ".sw_output_arbiter_activity")
        .flags(Stats::nozero)
    ;
}

void
Router::collateStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_buffer_reads += m_input_unit[i]->
            get_buf_read_activity(j);
            m_buffer_writes += m_input_unit[i]->
            get_buf_write_activity(j);
        }
    }

    m_sw_input_arbiter_activity = m_sw_alloc->
    get_input_arbiter_activity();
    m_sw_output_arbiter_activity = m_sw_alloc->
    get_output_arbiter_activity();
    m_crossbar_activity = m_switch->get_crossbar_activity();
}

void
Router::resetStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_input_unit[i]->resetStats();
        }
    }

    m_switch->resetStats();
    m_sw_alloc->resetStats();
}

void
Router::printFaultVector(ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    int num_fault_types = m_network_ptr->
    fault_model->number_of_fault_types;
    float fault_vector[num_fault_types];
    get_fault_vector(temperature_celcius, fault_vector);
    out << "Router-" << m_id << " fault vector: " << endl;
    for (int fault_type_index = 0; fault_type_index < num_fault_types;
         fault_type_index++) {
        out << " - probability of (";
        out <<
        m_network_ptr->fault_model->fault_type_to_string(fault_type_index);
        out << ") = ";
        out << fault_vector[fault_type_index] << endl;
    }
}

void
Router::printAggregateFaultProbability(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    float aggregate_fault_prob;
    get_aggregate_fault_probability(temperature_celcius,
                                    &aggregate_fault_prob);
    out << "Router-" << m_id << " fault probability: ";
    out << aggregate_fault_prob << endl;
}

uint32_t
Router::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    num_functional_writes += m_switch->functionalWrite(pkt);

    for (uint32_t i = 0; i < m_input_unit.size(); i++) {
        num_functional_writes += m_input_unit[i]->functionalWrite(pkt);
    }

    for (uint32_t i = 0; i < m_output_unit.size(); i++) {
        num_functional_writes += m_output_unit[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

Router *
GarnetRouterParams::create()
{
    return new Router(this);
}

//Update task
void Router::update_task(){
    static Cycles cur = curCycle();
    //return when the task effective
    //a.k.a current cycle<end time
    if (m_curTask!= nullptr && cur < m_curTask->m_end_time)
        return;
    //check whether is expired
    else if (m_curTask != nullptr)
        delete m_curTask;
    //check for the json task file

    //initialize
    std::ifstream ifs ("timepoints.json");
    Json::Reader reader;
    Json::Value obj;
    bool b = reader.parse(ifs,obj);
    assert(b);

    for (Json::Value::const_iterator it = obj.begin();
        it!= obj.end();++it){
        //skip the iterator to the current time
        while (cur>Cycles((*it)[0]["timePoint"].asInt())){
            it++;
        }

        int begin_time =  (*it)[0]["period"][0].asInt();
        int end_time =  (*it)[0]["period"][1].asInt();
        int inport,outport;

        //cursed iterator for finding the right settings of the router
        //be advised that only 1 task would be generated for each
        //timePorint for every router in the task list
        Json::Value::const_iterator it_r;
        for (it_r = (*it)[0]["route"].begin();
        it_r!= (*it)[0]["route"].end() && (*it_r)[0].asInt() != m_id;
        ++it_r);
        if (it_r == (*it)[0]["route"].end())
            break;
        else
            {
            if ((*it_r)[1].asString()=="N")
                inport = m_routing_unit->getInportIdx("North");
            else if ((*it_r)[1].asString()=="S")
                inport = m_routing_unit->getInportIdx("South");
            else if ((*it_r)[1].asString()=="W")
                inport = m_routing_unit->getInportIdx("West");
            else if ((*it_r)[1].asString()=="E")
                inport = m_routing_unit->getInportIdx("East");
            else if ((*it_r)[1].asString()=="P")
                inport = 0;
            //this is because inport local have 2 ports
            //inport[0]==>flit inport
            //while getinportIdx("Local") would return 1
            if ((*it_r)[2].asString()=="N")
                outport = m_routing_unit->getOutportIdx("North");
            else if ((*it_r)[2].asString()=="S")
                outport = m_routing_unit->getOutportIdx("South");
            else if ((*it_r)[2].asString()=="W")
                outport = m_routing_unit->getOutportIdx("West");
            else if ((*it_r)[2].asString()=="E")
                outport = m_routing_unit->getOutportIdx("East");
            else if ((*it_r)[2].asString()=="P")
                outport = m_routing_unit->getOutportIdx("Local");}

            m_curTask= new task{
                Cycles(begin_time),
                Cycles(end_time),
                inport,
                outport
            };
            DPRINTF(RubyNetwork,
    "Router %d Task Period %lld to %lld Inport %s Outport %s \n",
        get_id(), m_curTask->m_begin_time, m_curTask->m_end_time,
        getInportDirection(m_curTask->m_inport_id),
        getOutportDirection(m_curTask->m_outport_id));
            break;
        }



}

