#include <iostream>
#include <thread>

void set_main_thread_sched_priority(int core_id, int sched_priority, int policy) {
  struct sched_param param;
  param.sched_priority = sched_priority;

  // set priority
  int ret = sched_setscheduler(0, policy, &param);
  if(ret < 0) {
    std::cout << "sched_setscheduler of main thread failed ret = " << ret << std::endl;
  }
  // set affinity
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  int rc = sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
  if (rc != 0) {
    std::cerr << "sched_setaffinity of main thread failed rc = " << rc << "\n";
  }
}

void set_sub_thread_sched_priority(int core_id, int sched_priority, int policy, std::thread& th) {
  struct sched_param param;
  param.sched_priority = sched_priority;

  // set priority
  int ret = pthread_setschedparam(th.native_handle(), policy, &param);
  if(0 != ret) {
    std::cout << "cannot set task thread scheduler, retval = " << ret
              << " policy = " << policy
              << " priority = " << sched_priority
              << std::endl;
  }
  // set affinity
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  int rc = pthread_setaffinity_np(th.native_handle(),
                                  sizeof(cpu_set_t), &cpuset);
  if (rc != 0) {
    std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
  }
}

void set_only_priority(int sched_priority, int policy, std::thread& th) {
  struct sched_param param;
  param.sched_priority = sched_priority;

  // set priority
  int ret = pthread_setschedparam(th.native_handle(), policy, &param);
  if(0 != ret) {
    std::cout << "cannot set task thread PRIORITY scheduler, retval = " << ret
              << " policy = " << policy
              << " priority = " << sched_priority
              << std::endl;
  }
}
