#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <list>
#include "IAdapter_tests_a_addComp.h"

#include "tests_a_addComp.h"

using namespace std;
using namespace chrono;

static int exeMs = 100;

bool parseCommandLineParameters(int argc, char* argv[]){
  if(argc == 1){
    return true;
  }
  if(argc == 3 && string(argv[1]) == "-t"){
    try{
      int tmp = stoi(argv[2]);
      if(tmp >= 0){
        exeMs = tmp;
        return true;
      }
    }catch(...){
      //Ignore
    }
  }
  cout << "Usage: " << argv[0] << " [-h | -t sleepTimeMs]\n";
  return false;
}

void startMiddleware(IAdapter_tests_a_addComp& adapter,tests_a_addComp& comp,atomic<bool>& done){
  adapter.init(&comp);
  done = true;
}

int main(int argc, char* argv[])
{
  if(!parseCommandLineParameters(argc,argv)){
    return 1;
  }

  atomic<bool> done(false);
  tests_a_addComp comp;
  comp.init();

  list<IAdapter_tests_a_addComp*> adapters;
  

  list<thread*> threads;
  for(auto a : adapters){
    threads.push_back(new thread(startMiddleware,ref(*a),ref(comp),ref(done)));
  }

  cout << "waiting for all middleware to start\n";
  this_thread::sleep_for(seconds(3));
  cout << "started! Executing every " << exeMs << "ms\n";

  time_point<system_clock> start, end;
  while(!done){
    start = system_clock::now();

    comp.execute();
    for(auto a : adapters){
       (*a).tick();
    }

    end = system_clock::now();
    int elapsedMs = duration_cast<milliseconds>(end-start).count();
    int newSleep = exeMs - elapsedMs;
    if(newSleep <= 0){
      cout << "Cant keep up! "<< (-newSleep) <<"ms late!\n";
    }else{
       this_thread::sleep_for(milliseconds(newSleep));
    }
  }

  return 0;
}