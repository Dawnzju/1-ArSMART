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

#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/GarnetLink.hh"
#include "mem/ruby/network/garnet2.0/NetworkInterface.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"
#include "mem/ruby/network/garnet2.0/SSR.hh"
#include "mem/ruby/system/RubySystem.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

int routing_algorithm;

/*
 * GarnetNetwork sets up the routers and links and collects stats.
 * Default parameters (GarnetNetwork.py) can be overwritten from command line
 * (see configs/network/Network.py)
 */
std::string graph_file;
int hpcMax;
int rowNum;
GarnetNetwork::GarnetNetwork(const Params *p)
    : Network(p)
{
    m_num_rows = p->num_rows;
    rowNum = m_num_rows;

    m_ni_flit_size = p->ni_flit_size;
    m_vcs_per_vnet = p->vcs_per_vnet;
    m_buffers_per_data_vc = p->buffers_per_data_vc;
    m_buffers_per_ctrl_vc = p->buffers_per_ctrl_vc;
    m_routing_algorithm = p->routing_algorithm;

    m_enable_smart = p->enable_smart;
    m_enable_smart2D = p->enable_smart2D;
    m_enable_central = p->enable_central;

    isSDNoC = p->enable_central;
    
    m_single_flit = p->enable_single_flit;
    m_smart_hpcmax = p->smart_hpcmax;
    m_smart_dest_bypass = p->smart_dest_bypass;
    graph_file = p->filename;
    hpcMax = m_smart_hpcmax;
    //std::cout<<graph_file<<endl;

    m_enable_fault_model = p->enable_fault_model;
    if (m_enable_fault_model)
        fault_model = p->fault_model;
    routing_algorithm = m_routing_algorithm;
    m_vnet_type.resize(m_virtual_networks);

    for (int i = 0 ; i < m_virtual_networks ; i++) {
        if (m_vnet_type_names[i] == "response")
            m_vnet_type[i] = DATA_VNET_; // carries data (and ctrl) packets
        else
            m_vnet_type[i] = CTRL_VNET_; // carries only ctrl packets
    }

    // record the routers
    for (vector<BasicRouter*>::const_iterator i =  p->routers.begin();
         i != p->routers.end(); ++i) {
        Router* router = safe_cast<Router*>(*i);
        m_routers.push_back(router);

        // initialize the router's network pointers
        router->init_net_ptr(this);
    }

    // record the network interfaces
    for (vector<ClockedObject*>::const_iterator i = p->netifs.begin();
         i != p->netifs.end(); ++i) {
        NetworkInterface *ni = safe_cast<NetworkInterface *>(*i);
        m_nis.push_back(ni);
        ni->init_net_ptr(this);
    }
}

void
GarnetNetwork::init()
{
    Network::init();

    for (int i=0; i < m_nodes; i++) {
        m_nis[i]->addNode(m_toNetQueues[i], m_fromNetQueues[i]);
    }

    // The topology pointer should have already been initialized in the
    // parent network constructor
    assert(m_topology_ptr != NULL);
    m_topology_ptr->createLinks(this);

    // Initialize topology specific parameters
    if (getNumRows() > 0) {
        // Only for Mesh topology
        // m_num_rows and m_num_cols are only used for
        // implementing XY or custom routing in RoutingUnit.cc
        m_num_rows = getNumRows();
        m_num_cols = m_routers.size() / m_num_rows;
        assert(m_num_rows * m_num_cols == m_routers.size());
    } else {
        m_num_rows = -1;
        m_num_cols = -1;
    }

    // FaultModel: declare each router to the fault model
    if (isFaultModelEnabled()) {
        for (vector<Router*>::const_iterator i= m_routers.begin();
             i != m_routers.end(); ++i) {
            Router* router = safe_cast<Router*>(*i);
            int router_id M5_VAR_USED =
                fault_model->declare_router(router->get_num_inports(),
                                            router->get_num_outports(),
                                            router->get_vc_per_vnet(),
                                            getBuffersPerDataVC(),
                                            getBuffersPerCtrlVC());
            assert(router_id == router->get_id());
            router->printAggregateFaultProbability(cout);
            router->printFaultVector(cout);
        }
    }
    // schedule wakeup for the 0 router
    m_routers[0]->schedule_wakeup(Cycles(0));
}

GarnetNetwork::~GarnetNetwork()
{
    deletePointers(m_routers);
    deletePointers(m_nis);
    deletePointers(m_networklinks);
    deletePointers(m_creditlinks);
}

/*
 * This function creates a link from the Network Interface (NI)
 * into the Network.
 * It creates a Network Link from the NI to a Router and a Credit Link from
 * the Router to the NI
*/

void
GarnetNetwork::makeExtInLink(NodeID src, SwitchID dest, BasicLink* link,
                            const NetDest& routing_table_entry)
{
    assert(src < m_nodes);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);

    // GarnetExtLink is bi-directional
    NetworkLink* net_link = garnet_link->m_network_links[LinkDirection_In];
    net_link->setType(EXT_IN_);
    CreditLink* credit_link = garnet_link->m_credit_links[LinkDirection_In];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection dst_inport_dirn = "Local";
    m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    m_nis[src]->addOutPort(net_link, credit_link, dest);
}

/*
 * This function creates a link from the Network to a NI.
 * It creates a Network Link from a Router to the NI and
 * a Credit Link from NI to the Router
*/

void
GarnetNetwork::makeExtOutLink(SwitchID src, NodeID dest, BasicLink* link,
                             const NetDest& routing_table_entry)
{
    assert(dest < m_nodes);
    assert(src < m_routers.size());
    assert(m_routers[src] != NULL);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);

    // GarnetExtLink is bi-directional
    NetworkLink* net_link = garnet_link->m_network_links[LinkDirection_Out];
    net_link->setType(EXT_OUT_);
    CreditLink* credit_link = garnet_link->m_credit_links[LinkDirection_Out];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection src_outport_dirn = "Local";
    m_routers[src]->addOutPort(src_outport_dirn, net_link,
                               routing_table_entry,
                               link->m_weight, credit_link);
    m_nis[dest]->addInPort(net_link, credit_link);
}

/*
 * This function creates an internal network link between two routers.
 * It adds both the network link and an opposite credit link.
*/

void
GarnetNetwork::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                const NetDest& routing_table_entry,
                                PortDirection src_outport_dirn,
                                PortDirection dst_inport_dirn)
{
    GarnetIntLink* garnet_link = safe_cast<GarnetIntLink*>(link);

    // GarnetIntLink is unidirectional
    NetworkLink* net_link = garnet_link->m_network_link;
    net_link->setType(INT_);
    CreditLink* credit_link = garnet_link->m_credit_link;

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    m_routers[src]->addOutPort(src_outport_dirn, net_link,
                               routing_table_entry,
                               link->m_weight, credit_link);
    
}

// Total routers in the network
int
GarnetNetwork::getNumRouters()
{
    return m_routers.size();
}

// Get ID of router connected to a NI.
int
GarnetNetwork::get_router_id(int ni)
{
    return m_nis[ni]->get_router_id();
}
void
GarnetNetwork::wakeupRouter(int m_id)
{
        m_routers[m_id]->schedule_wakeup(Cycles(0));
    };
void
GarnetNetwork::regStats()
{
    Network::regStats();

    // Packets
    m_packets_received
        .init(m_virtual_networks)
        .name(name() + ".packets_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_packets_injected
        .init(m_virtual_networks)
        .name(name() + ".packets_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_packet_network_latency
        .init(m_virtual_networks)
        .name(name() + ".packet_network_latency")
        .flags(Stats::oneline)
        ;

    m_packet_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".packet_queueing_latency")
        .flags(Stats::oneline)
        ;

    for (int i = 0; i < m_virtual_networks; i++) {
        m_packets_received.subname(i, csprintf("vnet-%i", i));
        m_packets_injected.subname(i, csprintf("vnet-%i", i));
        m_packet_network_latency.subname(i, csprintf("vnet-%i", i));
        m_packet_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_packet_vnet_latency
        .name(name() + ".average_packet_vnet_latency")
        .flags(Stats::oneline);
    m_avg_packet_vnet_latency =
        m_packet_network_latency / m_packets_received;

    //DPRINTF(GarnetSyntheticTraffic,"+++++++++++++%d\n",m_packets_received);

    m_avg_packet_vqueue_latency
        .name(name() + ".average_packet_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_packet_vqueue_latency =
        m_packet_queueing_latency / m_packets_received;

    m_avg_packet_network_latency
        .name(name() + ".average_packet_network_latency");
    m_avg_packet_network_latency =
        sum(m_packet_network_latency) / sum(m_packets_received);

    m_avg_packet_queueing_latency
        .name(name() + ".average_packet_queueing_latency");
    m_avg_packet_queueing_latency
        = sum(m_packet_queueing_latency) / sum(m_packets_received);

    m_avg_packet_latency
        .name(name() + ".average_packet_latency");
    m_avg_packet_latency
        = m_avg_packet_network_latency + m_avg_packet_queueing_latency;

    // Flits
    m_flits_received
        .init(m_virtual_networks)
        .name(name() + ".flits_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flits_injected
        .init(m_virtual_networks)
        .name(name() + ".flits_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flit_network_latency
        .init(m_virtual_networks)
        .name(name() + ".flit_network_latency")
        .flags(Stats::oneline)
        ;

    m_flit_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".flit_queueing_latency")
        .flags(Stats::oneline)
        ;

    for (int i = 0; i < m_virtual_networks; i++) {
        m_flits_received.subname(i, csprintf("vnet-%i", i));
        m_flits_injected.subname(i, csprintf("vnet-%i", i));
        m_flit_network_latency.subname(i, csprintf("vnet-%i", i));
        m_flit_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_flit_vnet_latency
        .name(name() + ".average_flit_vnet_latency")
        .flags(Stats::oneline);
    m_avg_flit_vnet_latency = m_flit_network_latency / m_flits_received;

    m_avg_flit_vqueue_latency
        .name(name() + ".average_flit_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_flit_vqueue_latency =
        m_flit_queueing_latency / m_flits_received;

    m_avg_flit_network_latency
        .name(name() + ".average_flit_network_latency");
    m_avg_flit_network_latency =
        sum(m_flit_network_latency) / sum(m_flits_received);

    m_avg_flit_queueing_latency
        .name(name() + ".average_flit_queueing_latency");
    m_avg_flit_queueing_latency =
        sum(m_flit_queueing_latency) / sum(m_flits_received);

    m_avg_flit_latency
        .name(name() + ".average_flit_latency");
    m_avg_flit_latency =
        m_avg_flit_network_latency + m_avg_flit_queueing_latency;


    // Hops
    m_avg_hops.name(name() + ".average_hops");
    m_avg_hops = m_total_hops / sum(m_flits_received);

    m_avg_smart_hops.name(name() + ".average_smart_hops");
    m_avg_smart_hops = m_total_smart_hops / sum(m_flits_received);

    m_avg_hpc.name(name() + ".average_hpc");
    m_avg_hpc = m_total_hops / m_total_smart_hops;

    //std::cout<<sum(m_flits_received)<<endl;
    // Links
    m_total_ext_in_link_utilization
        .name(name() + ".ext_in_link_utilization");
    m_total_ext_out_link_utilization
        .name(name() + ".ext_out_link_utilization");
    m_total_int_link_utilization
        .name(name() + ".int_link_utilization");
    m_average_link_utilization
        .name(name() + ".avg_link_utilization");

    m_average_vc_load
        .init(m_virtual_networks * m_vcs_per_vnet)
        .name(name() + ".avg_vc_load")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
}

void
GarnetNetwork::collateStats()
{
    RubySystem *rs = params()->ruby_system;
    double time_delta = double(curCycle() - rs->getStartCycle());

    for (int i = 0; i < m_networklinks.size(); i++) {
        link_type type = m_networklinks[i]->getType();
        int activity = m_networklinks[i]->getLinkUtilization();

        if (type == EXT_IN_)
            m_total_ext_in_link_utilization += activity;
        else if (type == EXT_OUT_)
            m_total_ext_out_link_utilization += activity;
        else if (type == INT_)
            m_total_int_link_utilization += activity;
        if(isCentralControlled())
        {
            if(getRoutingAlgorithm()==2)
            m_average_link_utilization +=
            (double(activity) / time_delta)*1.52;
       
            if(getRoutingAlgorithm()==0)
            m_average_link_utilization +=
            (double(activity) / time_delta)*1.41;
        }
        else
        {
            m_average_link_utilization +=
            (double(activity) / time_delta);
        }
        vector<unsigned int> vc_load = m_networklinks[i]->getVcLoad();
        for (int j = 0; j < vc_load.size(); j++) {
            m_average_vc_load[j] += ((double)vc_load[j] / time_delta);
        }
    }

    // Ask the routers to collate their statistics
    for (int i = 0; i < m_routers.size(); i++) {
        m_routers[i]->collateStats();
    }
}

void
GarnetNetwork::print(ostream& out) const
{
    out << "[GarnetNetwork]";
}

GarnetNetwork *
GarnetNetworkParams::create()
{
    return new GarnetNetwork(this);
}

uint32_t
GarnetNetwork::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (unsigned int i = 0; i < m_routers.size(); i++) {
        num_functional_writes += m_routers[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_nis.size(); ++i) {
        num_functional_writes += m_nis[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_networklinks.size(); ++i) {
        num_functional_writes += m_networklinks[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}


// SMART NoC
void
GarnetNetwork::sendSSR(int src, PortDirection outport_dirn, int req_hops,
                       SSR* t_ssr)
{
    //int src_x = src % m_num_cols;
    int src_y = src / m_num_cols;

    // Send SSR up to req_hops - 1 with bypass_req = true;
    for (int hops = 1; hops <= req_hops; hops++) {

        bool bypass_req = true;
        if (hops == req_hops)
            bypass_req = false;

        if (outport_dirn == "East") {
            int dst = src + hops;
            if (dst / m_num_cols == src_y) {
                // valid dst on same row

                insertSSR(dst, "West", hops, bypass_req, t_ssr);
            }
        } else if (outport_dirn == "West") {
            int dst = src - hops;
            if (dst > 0 && dst / m_num_cols == src_y) {
                // valid dst on same row

                insertSSR(dst, "East", hops, bypass_req, t_ssr);
            }
        } else if (outport_dirn == "North") {
            int dst = src + m_num_cols * hops;
            if (dst / m_num_cols < m_num_rows) {
                // valid dst

                insertSSR(dst, "South", hops, bypass_req, t_ssr);
            }
        } else if (outport_dirn == "South") {
            int dst = src - m_num_cols * hops;
            if (dst / m_num_cols > 0) {
                // valid dst

                insertSSR(dst, "North", hops, bypass_req, t_ssr);
            }
        } else {
            assert(0);
        }   
    }

    // insertSSR makes a copy of the SSR for every dest
    delete t_ssr;
}

void        
GarnetNetwork::insertSSR(int dst, PortDirection inport_dirn, 
                         int src_hops, bool bypass_req, SSR* orig_ssr)
{
    SSR *t_ssr = new SSR(orig_ssr->get_vnet(),
                         src_hops,
                         src_hops,
                         bypass_req,
                         orig_ssr->get_outport_dirn(),
                         orig_ssr->get_outport_dirn_2(),
                         orig_ssr->get_ref_flit(),
                         orig_ssr->get_time());

    m_routers[dst]->insertSSR(inport_dirn, t_ssr);


    //m_routers[dst]->insertSSR(inport_dirn, t_ssr);
}

void
GarnetNetwork::sendSSR_2D(int src,int dest, PortDirection outport_dirn,
    int req_hops,int req_hops_2, SSR* t_ssr)
{
    //int src_x = src % m_num_cols;
    int src_y = src / m_num_cols;
    PortDirection outport_dirn_2;

    int my_x = src % m_num_cols;
    int my_y = src / m_num_cols;

    int dest_x = dest % m_num_cols;
    int dest_y = dest / m_num_cols;

    int turn;//turning router id
    //int turn_x;
    int turn_y;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);



    // Send SSR up to req_hops - 1 with bypass_req = true;
    for (int hops = 1; hops <= req_hops; hops++) {
        //this is the first dimension ssr-traverse
        bool bypass_req = true;
        if (hops == req_hops)
            bypass_req = false;

        if (outport_dirn == "East") {
            int dst = src + hops;
            if (dst / m_num_cols == src_y) {
                // valid dst on same row

                insertSSR(dst, "West", hops, bypass_req, t_ssr);
            }
        } else if (outport_dirn == "West") {
            int dst = src - hops;
            if (dst > 0 && dst / m_num_cols == src_y) {
                // valid dst on same row

                insertSSR(dst, "East", hops, bypass_req, t_ssr);
            }
        } else if (outport_dirn == "North") {
            int dst = src + m_num_cols * hops;
            if (dst / m_num_cols < m_num_rows) {
                // valid dst

                insertSSR(dst, "South", hops, bypass_req, t_ssr);
            }
        } else if (outport_dirn == "South") {
            int dst = src - m_num_cols * hops;
            if (dst / m_num_cols > 0) {
                // valid dst

                insertSSR(dst, "North", hops, bypass_req, t_ssr);
            }
        } else {
            assert(0);
        }
    }
    //if req_hop_2 > 0 then need a second traverse
    if (req_hops_2 > 0){
        //decide the outport2 direction
        if (outport_dirn == "East"||outport_dirn == "West"){
            //the second should along Y axis
            outport_dirn_2 = (y_dirn)?"North":"South";
            //calculate the turning router
            if (x_dirn){
                //going from east
                turn = src + x_hops;
            }else{
                //going from west
                turn = src - x_hops;
            }
        }else if (outport_dirn == "North"||outport_dirn == "South")
        {
            //the second should along X axis
            outport_dirn_2 = (x_dirn)?"East":"West";
            //calculate the turning router
            if (y_dirn){
                //going from north
                turn = src + y_hops * m_num_cols;
            }else{
                //going from south
                turn = src - y_hops * m_num_cols;
            }
        }
            //turn_x = turn % m_num_cols;
            turn_y = turn / m_num_cols;
            //this is the second dimension ssr-traverse
        for (int hops = 1; hops <= req_hops_2; hops++) {
            //this is the first demension ssr-traverse
            bool bypass_req = true;
            if (hops == req_hops)
                bypass_req = false;

            if (outport_dirn_2 == "East") {
                int dst = turn + hops;
                if (dst / m_num_cols == turn_y) {
                    // valid dst on same row

                    insertSSR(dst, "West", hops, bypass_req, t_ssr);
                }
            } else if (outport_dirn_2 == "West") {
                int dst = turn - hops;
                if (dst > 0 && dst / m_num_cols == turn_y) {
                    // valid dst on same row

                    insertSSR(dst, "East", hops, bypass_req, t_ssr);
                }
            } else if (outport_dirn_2 == "North") {
                int dst = turn + m_num_cols * hops;
                if (dst / m_num_cols < m_num_rows) {
                    // valid dst

                    insertSSR(dst, "South", hops, bypass_req, t_ssr);
                }
            } else if (outport_dirn_2 == "South") {
                int dst = turn - m_num_cols * hops;
                if (dst / m_num_cols > 0) {
                    // valid dst

                    insertSSR(dst, "North", hops, bypass_req, t_ssr);
                }
            } else {
                assert(0);
            }
        }
    }
    // insertSSR makes a copy of the SSR for every dest
    delete t_ssr;
}
