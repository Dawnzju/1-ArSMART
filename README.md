# 1-Gem5-Updated
This is a simulator which enables basic NoC, SMART NoC and ArSMART NoC. This simulator is developed based on gem5. The main website can be found at http://www.gem5.org.

To install this simulator:
    1. Download gem5: https://github.com/Dawnzju/gem5 (This is the version I am working on. Recommanded since the patch can be easily applied)
    2. Check the python version: Python 2.7.17 :: Anaconda, Inc. (conda activate py27)
    3. Make sure the original gem5 can be compiled and executed.
    4. Download and install JSONcpp: https://github.com/open-source-parsers/jsoncpp#generating-amalgamated-source-and-header
    5. Download NoC.patch file in this repo to gem5 folder.
    6. Apply this patch using command: git apply NoC.patch
    7. Compile again.

To 


