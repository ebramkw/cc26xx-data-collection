Author: ebramkw
In this project, nodes sense the environment and record the data to flash.
When the flash is full, the nodes will sync the recorded data to the sink.

*****************************************************

Compile data collection using the following command:
make TARGET=srf06-cc26xx BOARD=sensortag/cc2650 cc26xx-data-collection.bin CPU_FAMILY=cc26xx

In case of editing the project-conf.h, remove all precompiled sources before compiling using the following command:
make TARGET=srf06-cc26xx BOARD=sensortag/cc2650 CPU_FAMILY=cc26xx clean

under directory: contiki-data-collection/contiki/apps/cc26xx-data-collection

*****************************************************

For the sink to get the timestamp, use the python script located in: contiki-data-collection/send-time.py
Example to feed the timestamp to /dev/ttyACM0 every minute, python send-time.py 0 60

*****************************************************

Under typedef and defenetions header file: contiki-data-collection/contiki/apps/cc26xx-data-collection/defs_and_types.h

you may need to change the sink node id which is in:
#define SINK_ID 18051
and the mapping between the nodes and the objects which are in
#define NODE_ID_MAPPING
#define NODE_OBJECT_MAPPING
as well as the number of nodes in the networ, execulding the sink which is in
#define DATA_COLLECTION_NODES 9

*****************************************************

You'll need to formate the flash before starting to make sure that the nodes have no old data.
You can do so using the file: contiki-data-collection/contiki/apps/cc26xx-erase-flash.c