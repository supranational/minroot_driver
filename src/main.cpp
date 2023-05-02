// Copyright Supranational LLC
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <cstddef>
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <cinttypes>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <queue>
#include <thread>
#include <condition_variable>
#include <mutex>
#include "driver.hpp"
#include "model.hpp"

extern "C" {
#include "ftd2xx.h"
#include "libft4222.h"
}

bool CheckResults(Driver& driver, Model& model, uint8_t* job) {
  bool success = false; 
  mpz_t x, y;
  mpz_inits(x, y, NULL);

  uint32_t job_id;
  uint64_t iteration_count;

  driver.DeserializeJob(job, job_id, iteration_count, x, y);

  success = model.ValidateIter(x, y, job_id, iteration_count);

  mpz_clears(x, y, NULL);
  return success;
}

// Note: caller must free the buffer
uint8_t* PrepareJob(Driver& driver, Model& model, size_t& size,
                    uint64_t iteration_count = 0) {
  mpz_t x, y;
  mpz_inits(x, y, NULL);

  uint64_t starting_iteration;
  uint32_t job_id;

  model.GenInputs(x, y, job_id, iteration_count, starting_iteration);

  size = driver.CmdSize();
  //printf("new job buffer size = %zu\n", size);
  uint8_t *buf = (uint8_t *)malloc(size);
  driver.SerializeJob(buf, job_id, iteration_count, starting_iteration, x, y);

  mpz_clears(x, y, NULL);

  return buf;
}

typedef struct {
  uint8_t d[VDF_STATUS_END_REG_OFFSET - VDF_STATUS_JOB_ID_REG_OFFSET];
} intermediate_t;
  
class Worker {
private:
  std::queue<intermediate_t> m_queue_intermediate;
  std::queue<int> m_queue_result;
  Driver *driver;
  Model *model;
  std::thread m_thread;
  std::mutex m_mutex_intermediate;
  std::condition_variable m_cond_intermediate;
  std::mutex m_mutex_result;
  
public:
  Worker(Driver& _driver, Model& _model) {
    driver = &_driver;
    model = &_model;
    m_thread = std::thread(&Worker::doWork, this);
  }

  ~Worker() {
    m_thread.join();
  }

  void QueueIntermediate(uint8_t *d) {
    std::unique_lock<std::mutex> lock(m_mutex_intermediate);
    m_queue_intermediate.push(*(intermediate_t *)d);
    m_cond_intermediate.notify_one();
  }

  int GetResults(bool *success, bool *done) {
    int result;
    std::unique_lock<std::mutex> lock(m_mutex_result);
    if (m_queue_result.empty())
      return 0;
    result = m_queue_result.front();
    m_queue_result.pop();
    *success = (result & 1) ? true : false; 
    *done = (result & 2) ? true : false;
    return 1;
  }
  
  void doWork() {
    bool success, done=false;
    intermediate_t d;
    while(!done) {
      {
        std::unique_lock<std::mutex> lock(m_mutex_intermediate);
        m_cond_intermediate.wait(lock,
          [this]() { return !m_queue_intermediate.empty(); });
        d = m_queue_intermediate.front();
        m_queue_intermediate.pop();
      }
      success = CheckResults(*driver,*model,(uint8_t *)&d);
      done = model->RunComplete();
      int result = (success ? 1 : 0) | (done ? 2 : 0);
      {
        std::unique_lock<std::mutex> lock(m_mutex_result);
        m_queue_result.push(result);
      }
    }
  }

};

int main(int argc, char* argv[]) {
  int  opt = 0;
  unsigned int rand_seed = 1;
  unsigned int num_jobs  = 1;
  unsigned int num_engines = 1;
  double freq = 200.0;
  double voltage = 0.80;
  uint64_t iters = 0; // 0 implies randomize

  while ((opt = getopt(argc, argv, "f:v:i:s:n:e:h")) != -1) {
    switch(opt) {
      case 'h':
        printf("Help\n");
        printf(" -s <seed>\n");
        printf(" -n <num jobs>\n");
        printf(" -e <num engines>\n");
        printf(" -f <frequency> (should be between 200 and 1100)\n");
        printf(" -v <voltage> (should be between 0.70 and 1.0)\n");
        printf(" -i <iterations>\n");
        return 0;
        break;
      case 'f':
        freq = (double)atof(optarg);
        printf("Command line freq %lf\n", freq);
        break;
      case 'v':
        voltage = (double)atof(optarg);
        printf("Command line voltage %lf\n", voltage);
        break;
      case 'i':
        iters = (uint64_t)atol(optarg);
        printf("Command line iters %" PRIu64 "\n", iters);
        break;
      case 's':
        rand_seed  = (unsigned int)atoi(optarg);
        printf("Command line Seed %d\n", rand_seed);
        break;
      case 'n':
        num_jobs = (unsigned int)atoi(optarg);
        printf("Command line Number of jobs %d\n", num_jobs);
        break;
      case 'e':
        num_engines = (unsigned int)atoi(optarg);
        printf("Command line Number of engines %d\n", num_engines);
        break;
      case ':':
        printf("Command line option needs a value\n");
        break;
      case '?':
        printf("Command line option unknown: %c\n", optopt);
        break;
    }
  }

  srand (rand_seed);

  Driver driver(freq, voltage);

  bool     success = true;

  Model** model = new Model*[num_engines];
  Worker* worker[num_engines];
  size_t   size[num_engines];
  uint8_t* job[num_engines];
  unsigned job_csr[num_engines];

  std::chrono::time_point<std::chrono::system_clock> starts[num_engines];
    
  for (unsigned i=0; i<num_engines; i++) {
    model[i] = 
      new Model
      ("40000000000000000000000000000000224698fc094cf91b992d30ed00000001",
       "333333333333333333333333333333334e9ee0c9a10a60e2e0f0f3f0cccccccd",
       rand());
    job_csr[i] = VDF_CMD_JOB_ID_REG_OFFSET + (i * VDF_ENGINE_STRIDE);
    job[i] = NULL;
    size[i] = 0;
  }

  for (unsigned i=0; i<num_engines; i++) {
    driver.DisableEngine(VDF_CONTROL_REG_OFFSET + i * VDF_ENGINE_STRIDE);
  }

  {
    double power=0.0;
    for(unsigned i=0;i<100;i++) {
      power += driver.GetPower();
    }
    printf("Idle power = %lf\n",power/100.0);
  }
  
  for (unsigned i=0; i<num_engines; i++) {
    driver.EnableEngine(VDF_CONTROL_REG_OFFSET + i * VDF_ENGINE_STRIDE);
  }

  unsigned started_jobs = 0;
  unsigned completed_jobs = 0;

  do {
    // start jobs on idle engines as long as jobs remain
    for(unsigned i=0; i<num_engines; i++) {
      if (!job[i] && started_jobs<num_jobs) {
        job[i] = PrepareJob(driver, *model[i], size[i], iters);
        printf("Starting job %d id=0x%x on engine %d\n",
               started_jobs,*(unsigned *)job[i],i);
        starts[i] = std::chrono::system_clock::now();
        driver.ftdi.Write(job_csr[i], job[i], size[i]);
        worker[i] = new Worker(driver, *model[i]);
        started_jobs++;
      }
    }
    
    // read from burstmap area, checking for intermediates
    // and job completion
    {
      unsigned pvt = (PVT_VOLTAGE_REG_OFFSET -
                      PVT_TEMPERATURE_REG_OFFSET + 4);
      size_t size = (VDF_STATUS_END_REG_OFFSET -
                     VDF_STATUS_JOB_ID_REG_OFFSET);
      uint8_t *buf = (uint8_t *)malloc(pvt+num_engines*size);
      memset(buf, 0, num_engines*size);
      driver.ftdi.Read(BURST_ADDR, buf, pvt+num_engines*size);

      // dump power
      //{
      //  double temperature, voltage, power;
      //  driver.InterpretPvtBurst(buf, &temperature, &voltage);
      //  power = driver.GetPower();
      //  printf("voltage=%lf, temperature=%lf, power=%lf\n",
      //         voltage, temperature, power);
      //}

      // send intermediates to worker threads
      for(unsigned i=0; i<num_engines; i++) {
        if (job[i]!=NULL) {
          worker[i]->QueueIntermediate(&buf[pvt+i*size]);
        }
      }

      // review results from intermediate checking
      for(unsigned i=0; i<num_engines && success; i++) {
        if (job[i]!=NULL) {
          bool done;
          while (success && worker[i]->GetResults(&success,&done)) {
            printf("Intermediate on engine %d is %s\n",i,success?"OK":"NG");
            if (done) {
              auto cur = std::chrono::system_clock::now();
              int64_t dur =
                std::chrono::duration_cast
                <std::chrono::milliseconds>(cur - starts[i]).count();
              uint64_t iters_sec =
                (uint64_t)((double)iters / ((double) dur / 1000.0));
              printf("Completed job on engine %d in %" PRId64
                     " ms, ~%" PRId64 " iters/sec\n", i, dur, iters_sec);
              free(job[i]);
              job[i]=NULL;
              delete worker[i];
              completed_jobs++;
              break;
            }
          }
        }
      }

      free(buf);
    }
    
  } while (success && completed_jobs<num_jobs);

  for (unsigned i=0; i<num_engines; i++) {
    driver.DisableEngine(VDF_CONTROL_REG_OFFSET + i * VDF_ENGINE_STRIDE);
  }

  {
    double power=0.0;
    for(unsigned i=0;i<100;i++) {
      power += driver.GetPower();
    }
    printf("Idle power = %lf\n",power/100.0);
  }
  
  if (success == false) {
    printf("\n\nERROR - test failed\n\n");
  } else {
    printf("\n\nTest PASSED!\n\n");
  }
}
