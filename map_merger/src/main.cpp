#include "mapmerger.h"

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
//#define PROFILE

#ifdef PROFILE
    #include "google/profiler.h"
    #include "google/heap-profiler.h"
#endif

using namespace std;

void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}


int main(int argc, char **argv)
{
    signal(SIGSEGV, handler);
#ifdef PROFILE
    const char  fname[3] = "TS";
    ProfilerStart(fname);
    HeapProfilerStart(fname);
#endif
    ros::init(argc,argv,"map_merger");
    MapMerger *merger = new MapMerger();
    if(merger->getHasLocalMap())
        merger->waitForLocalMetaData();
    else
        merger->waitForRobotInformation();
    merger->start();
#ifdef PROFILE
    HeapProfilerStop();
    ProfilerStop();
#endif
    return 0;
}
