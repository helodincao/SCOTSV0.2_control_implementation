#pragma once

#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include "RungeKutta4.hh"

const int state_dim = 3;
const int input_dim = 3;

using state_type = std::array<double, state_dim>;
using input_type = std::array<double, input_dim>;

struct EllipsoidTarget {
  state_type c;
  state_type L;
};

struct ArenaConfig {
  double tau = 0.3;
  int nint = 10;

  state_type s_lb = {{0.0, 0.0, -3.5}};
  state_type s_ub = {{10.0, 10.0, 3.5}};
  state_type s_eta = {{0.2, 0.2, 0.2}};

  input_type i_lb = {{-0.5, -0.2, -0.8}};
  input_type i_ub = {{0.76, 0.2, 0.8}};
  input_type i_eta = {{0.25, 0.2, 0.8}};

  std::vector<EllipsoidTarget> targets;
  std::vector<std::array<double,4>> obstacles;
};

bool readArenaConfig(const std::string &path, ArenaConfig &cfg) {
  std::ifstream file(path);

  if (!file.good()) {
    return false;
  }

  std::string line;

  while (std::getline(file, line)) {
    const size_t hash = line.find('#');

    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }

    if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;
    }

    std::istringstream ss(line);
    std::string key;
    ss >> key;

    if (key == "tau") {
      ss >> cfg.tau;
    }
    else if (key == "nint") {
      ss >> cfg.nint;
    }
    else if (key == "state_lb") {
      ss >> cfg.s_lb[0] >> cfg.s_lb[1] >> cfg.s_lb[2];
    }
    else if (key == "state_ub") {
      ss >> cfg.s_ub[0] >> cfg.s_ub[1] >> cfg.s_ub[2];
    }
    else if (key == "state_eta") {
      ss >> cfg.s_eta[0] >> cfg.s_eta[1] >> cfg.s_eta[2];
    }
    else if (key == "input_lb") {
      ss >> cfg.i_lb[0] >> cfg.i_lb[1] >> cfg.i_lb[2];
    }
    else if (key == "input_ub") {
      ss >> cfg.i_ub[0] >> cfg.i_ub[1] >> cfg.i_ub[2];
    }
    else if (key == "input_eta") {
      ss >> cfg.i_eta[0] >> cfg.i_eta[1] >> cfg.i_eta[2];
    }
    else if (key == "target") {
      EllipsoidTarget t;

      ss >> t.c[0] >> t.c[1] >> t.c[2]
         >> t.L[0] >> t.L[1] >> t.L[2];

      cfg.targets.push_back(t);
    }
    else if (key == "obstacle") {
      std::array<double,4> box;

      ss >> box[0] >> box[1] >> box[2] >> box[3];

      if (box[0] < box[1] && box[2] < box[3]) {
        cfg.obstacles.push_back(box);
      }
      else {
        std::cout << "Skipping invalid obstacle." << std::endl;
      }
    }
  }

  return true;
}

inline void go2_post(state_type &x,
                     const input_type &u,
                     double tau,
                     int nint) {

  auto rhs = [](state_type &xx,
                const state_type &x,
                const input_type &u) {

    xx[0] = u[0] * std::cos(x[2])
          - u[1] * std::sin(x[2]);

    xx[1] = u[0] * std::sin(x[2])
          + u[1] * std::cos(x[2]);

    xx[2] = u[2];
  };

  scots::runge_kutta_fixed4(
      rhs,
      x,
      u,
      state_dim,
      tau,
      nint
  );
}

inline bool in_target(const state_type &x, const ArenaConfig &cfg) {
  for (const auto &t : cfg.targets) {
    double value =
        std::pow(t.L[0] * (x[0] - t.c[0]), 2) +
        std::pow(t.L[1] * (x[1] - t.c[1]), 2) +
        std::pow(t.L[2] * (x[2] - t.c[2]), 2);

    if (value <= 1.0) {
      return true;
    }
  }

  return false;
}