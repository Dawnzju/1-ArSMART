
#ifndef __MEM_RUBY_NETWORK_GARNET_TASK_HH__
#define __MEM_RUBY_NETWORK_GARNET_TASK_HH__

#include <cassert>
#include <iostream>

#include "base/types.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/flit.hh"

struct task{
      Cycles m_begin_time;
      Cycles m_end_time;
      int m_inport_id;
      int m_outport_id;
    };

#endif