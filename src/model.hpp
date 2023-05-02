// Copyright Supranational LLC
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <string>
#include <cstdlib> 
#include <cstdint> 
#include <cinttypes>
#include <gmp.h> 

extern "C"
int minroot_partial_verify_pallas(const unsigned char xy_d[64],
                                  const unsigned char xy0[64],
                                  size_t D, size_t E);

class Model {
public:
  Model() {}
  Model(std::string m, std::string root_exp, unsigned int rand_seed) {
    mpz_inits(modulus_, r_, rinv_, fifth_root_exp_,
              prev_x_, prev_y_, prev_x_raw_, prev_y_raw_, NULL);

    gmp_randinit_default(rand_state_g_);
    gmp_randseed_ui(rand_state_g_, rand_seed);

    mpz_set_str(modulus_, m.c_str(), 16);
    mpz_set_str(fifth_root_exp_, root_exp.c_str(), 16);

    mpz_set_ui(r_, 1);
    mpz_mul_2exp(r_, r_, 8 * 16);
    mpz_invert(rinv_, r_, modulus_);
  }

  ~Model() {
    mpz_clears(modulus_, r_, rinv_, fifth_root_exp_,
               prev_x_, prev_y_, prev_x_raw_, prev_y_raw_, NULL);
  }

  void GenInputs(mpz_t x, mpz_t y, uint32_t& job_id,
                 uint64_t& iteration_count, uint64_t& starting_iteration) {
    mpz_urandomb(x, rand_state_g_, (17 * 16) - 1);
    mpz_urandomb(y, rand_state_g_, (17 * 16) - 1);

    job_id = (uint32_t) rand();

    // If no iteration count is provided then randomize
    if (iteration_count == 0) {
      // Iteration size is 48 bits
      unsigned int rand_lower = rand();
      unsigned int rand_upper = rand() % 0xFFFF;

      iteration_count = (((uint64_t) rand_upper) << 32) | rand_lower;
    }

    // Limit the time to something not too large for now
    uint64_t max_iters = 100000000;

    if (max_iters >= iteration_count) {
      starting_iteration = 1;
    } else {
      uint64_t rand_start = rand() % max_iters;

      if (rand_start == 0) {
        rand_start = max_iters;
      }

      starting_iteration = iteration_count - rand_start;
    }

    mpz_mul(prev_x_raw_, x, rinv_);
    mpz_mod(prev_x_raw_, prev_x_raw_, modulus_);
    mpz_mul(prev_y_raw_, y, rinv_);
    mpz_mod(prev_y_raw_, prev_y_raw_, modulus_);

    printf("\nStarting iteration %" PRIu64 "\n", starting_iteration);
    printf("Iterations to run %" PRIu64 "\n", iteration_count);
    gmp_printf("Mont input x       %#Zx\n", x);
    gmp_printf("Mont input y       %#Zx\n", y);
    gmp_printf("Raw input x        %#Zx\n", prev_x_raw_);
    gmp_printf("Raw input y        %#Zx\n", prev_y_raw_);

    mpz_set(prev_x_, x);
    mpz_set(prev_y_, y);
    cur_job_id_ = job_id;
    prev_iter_  = starting_iteration;
    final_iter_ = iteration_count + 1;
  }

  bool ValidateIter(mpz_t x, mpz_t y, uint32_t job_id, uint64_t iter) {
    bool success = true;

    mpz_t cur_x_raw, cur_y_raw, exp_x_in, exp_y_in, xi_raw, yi_raw, giter;
    mpz_inits(cur_x_raw, cur_y_raw, exp_x_in, exp_y_in, xi_raw, yi_raw,
              giter, NULL);

    // Take values out of Montgomery
    mpz_mul(cur_x_raw, x, rinv_);
    mpz_mod(cur_x_raw, cur_x_raw, modulus_);

    mpz_mul(cur_y_raw, y, rinv_);
    mpz_mod(cur_y_raw, cur_y_raw, modulus_);

    if (job_id != cur_job_id_) {
      printf("ERROR: received job_id %x, expected %x\n",
             job_id, cur_job_id_);
      return false;
    }

    // Cannot have an iteration that is beyond the programmed count
    if (iter > final_iter_) {
      printf("ERROR: received iter %" PRIu64 " beyond final %" PRIu64 "\n",
             iter, final_iter_);
      return false;
    }

    unsigned char xy_d[64] = { 0 };
    unsigned char xy0[64] = { 0 };
    size_t byte_count;

    byte_count = (mpz_sizeinbase(cur_x_raw, 2) + 7) / 8;
    assert(byte_count <= 32);
    mpz_export(xy_d+32-byte_count, nullptr, 1, 1, 0, 0, cur_x_raw);
    byte_count = (mpz_sizeinbase(cur_y_raw, 2) + 7) / 8;
    assert(byte_count <= 32);
    mpz_export(xy_d+64-byte_count, nullptr, 1, 1, 0, 0, cur_y_raw);

    byte_count = (mpz_sizeinbase(prev_x_raw_, 2) + 7) / 8;
    assert(byte_count <= 32);
    mpz_export(xy0+32-byte_count, nullptr, 1, 1, 0, 0, prev_x_raw_);
    byte_count = (mpz_sizeinbase(prev_y_raw_, 2) + 7) / 8;
    assert(byte_count <= 32);
    mpz_export(xy0+64-byte_count, nullptr, 1, 1, 0, 0, prev_y_raw_);

    success = minroot_partial_verify_pallas(xy_d, xy0, iter-1, prev_iter_);

    // Update internal state
    if (success) {
      mpz_set(prev_x_, x);
      mpz_set(prev_y_, y);
      mpz_set(prev_x_raw_, cur_x_raw);
      mpz_set(prev_y_raw_, cur_y_raw);
      prev_iter_  = iter;
    }

    mpz_clears(cur_x_raw, cur_y_raw, exp_x_in, exp_y_in, xi_raw, yi_raw,
               giter, NULL);

    return success;
  }

  bool RunComplete() {
    return (prev_iter_ == final_iter_);
  }

private:
  gmp_randstate_t rand_state_g_;
  mpz_t           modulus_;
  mpz_t           fifth_root_exp_;
  mpz_t           rinv_;
  mpz_t           r_;
  mpz_t           prev_x_;
  mpz_t           prev_y_;
  mpz_t           prev_x_raw_;
  mpz_t           prev_y_raw_;
  uint32_t        cur_job_id_;
  uint64_t        prev_iter_;
  uint64_t        final_iter_;
};
#endif
