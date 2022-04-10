# ArSMART
> This is a simulator which enables basic NoC, SMART NoC and ArSMART NoC. This simulator is developed based on gem5. The main website can be found at http://www.gem5.org.  
---------------------------------------------------------------------------------
## To install this simulator:  
* 1. Download gem5: https://github.com/Dawnzju/gem5 (This is the version I am working on. Recommanded since the patch can be easily applied)  
* 2. Check the python version: Python 2.7.17 :: Anaconda, Inc. (conda activate py27)  
* 3. Make sure the original gem5 can be compiled and executed.  
* 4. Download and install JSONcpp: https://github.com/open-source-parsers/jsoncpp#generating-amalgamated-source-and-header  
* 5. Paste configs to gem5 folder
* 5. Download NoC.patch file in this repo to gem5 folder.  
* 6. Apply this patch using command: git apply NoC1.patch  
* 7. Compile again: scons build/Garnet_standalone/gem5.debug -j25 NUMBER_BITS_PER_SET=256  
---------------------------------------------------------------------------------
## To use this simulator:  
- Original traffic pattern for NoC are supported:  
        https://www.gem5.org/documentation/general_docs/ruby/garnet_synthetic_traffic/  
- Prepare the task graph file with mapping and routing information  
    - The task graph is saved in a json file: one task with out links is saved as:   
        "9": {"total_needReceive": 93, "input_links": [], "start_time": 0, "out_links": [[[2, 93, [], [[9, "S"], [17, "S"], [25, "S"], [33, "E"], [34, "E"], [35, "E"]], 1, 9, 36], [2, 93, [], [[9, "E"], [10, "E"], [11, "E"], [12, "S"], [20, "S"], [28, "S"]], 2, 9, 36], [2, 93, [], [[9, "E"], [10, "E"], [11, "S"], [19, "S"], [27, "S"], [35, "E"]], 3, 9, 36], [2, 93, [], [[9, "E"], [10, "S"], [18, "S"], [26, "E"], [27, "E"], [28, "S"]], 6, 9, 36], [2, 93, [], [[9, "E"], [10, "E"], [11, "S"], [19, "S"], [27, "E"], [28, "S"]], 6, 9, 36]]], "end_time": 0, "visited": 0, "total_needSend": 93, "exe_time": 102, "mapto": 9}  
    - **taskid(str)**:{**"total_needReceive"**:int(total message need to be received), **"input_links"**: list(opt,reserved for future use), **"start_time"**: int(opt,reserved for future use), **"out put links"**: [[link1[candidata path1: destinationTaskId,messagesize,priority(opt,reserved for future use),path,candidateCount,sourceRouter,destinationRouter],[candidata path2],[candidata path3]],[link2],[link3]],**"endTime"**:int(opt,reserved for future use),**"visited"**:0 or 1(opt,reserved for future use),**"totalneedSend"**: int(total Message need to send),**"exe_time"**:int(total task should be executed),**"mapto"**:int(The PE has been mapped to)}  
    - File name format: appName_Meshxxx_applicationInjectRate_Method.json  
- 3. Options:  
    *  **--num-cpus**  Number of PEs
    *  **--num-dirs** 
    *  **--sim-cycles** Number of simulation cycles
    *  **--topology** NoC topology
    *  **--debug-flags=GarnetSyntheticTraffic** Show the debug information
    *  **--network=garnet2.0** Enable garnet network
    *  **--mesh-rows**  Mesh size
    *  **--synthetic** Synthetic traffic loads
    *  **--smart_hpcmax=8**  Number of hops can be traversed in one cycle 
    *  **Example**: ./build/Garnet_standalone/gem5.debug --debug-flags=GarnetSyntheticTraffic configs/example/garnet_synth_traffic.py --network=garnet2.0 --num-cpus=64 --num-dirs=64 --topology=Mesh_XY --mesh-rows=8 --sim-cycles=1000000 --single-flit --synthetic=taskgraph --filename=Example_Mesh8x8_AIR1_xy.json  
    -  For SMART NoC:  
        *  **--smart** (GarnetNetwork.py/Network.py)  Enable SMART  
        *  **--smart2D** (GarnetNetwork.py/Network.py)  Enable SMART-2D	      
        - **Example**: ./build/Garnet_standalone/gem5.debug --debug-flags=GarnetSyntheticTraffic configs/example/garnet_synth_traffic.py --network=garnet2.0 --num-cpus=64 --num-dirs=64 --topology=Mesh_XY --mesh-rows=8 --sim-cycles=1000000 --single-flit --synthetic=taskgraph --smart --smart_hpcmax=4 --filename=Example_Mesh8x8_AIR1_xy.json  
    - For ArSMART NoC:  
        - Please read this paper:  https://ieeexplore.ieee.org/abstract/document/9464312  
        - **--central**	(GarnetNetwork.py/Network.py)	Enable the bypass at routers for the ArSMART 
        - **--filename** (GarnetNetwork.py/Network.py) The routing and mapping configuration file      
        - **Example**: ./build/Garnet_standalone/gem5.debug --debug-flags=GarnetSyntheticTraffic configs/example/garnet_synth_traffic.py --network=garnet2.0 --num-cpus=64 --num-dirs=64 --smart_hpcmax=8 --topology=Mesh_XY --mesh-rows=8 --sim-cycles=1000000 --single-flit --synthetic=taskgraph --central --filename=Example_Mesh8x8_AIR1_xy.json

## Mapping and routing
* Minimize contention via direct and indirect route:  
    * Peng Chen, Hui Chen, Jun Zhou, Mengquan Li, Weichen Liu, Chunhua Xiao, Yaoyao Ye, Nan Guan, ``Contention Minimization in Emerging SMART NoC via Direct and Indirect Routes.'' in IEEE Transactions on Computers (TC). 2021

    * **Example**: Example_Mesh8x8_AIR1_basic.json | Example_Mesh8x8_AIR1_improved.json

* Mapping and routing co-optimization:  
    * Hui Chen, Zihao Zhang, Peng Chen, Xiangzhong Luo, Shiqing Li, Weichen Liu, "MARCO: A High-performance Task Mapping and Routing Co-optimization Framework for Point-to-Point NoC-based Heterogeneous Computing Systems", in ACM Transactions on Embedded Computing Systems, doi: 10.1145/3476985

    * **Example**: Example_Mesh8x8_AIR1_coop.json 
    * https://github.com/Dawnzju/2-MARCO


* Mapping and routing co-optimization:  
    * Hui Chen, Peng Chen, Xiangzhong Luo, Shuohuai, Weichen Liu, ``LAMP: Load-balanced Multipath Parallel Transmission in Point-to-point NoCs.'' IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems (TCAD). 2022.

    * **Example**: Example_Mesh8x8_AIR1_split.json 
    * https://github.com/Dawnzju/3-DataSplitting

