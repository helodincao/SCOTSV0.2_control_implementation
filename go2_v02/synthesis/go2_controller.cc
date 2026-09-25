/*
 * vehicle.cc
 *
 *  created: Oct 2016
 *   author: Matthias Rungger
 */

/*
 * information about this example is given in
 * http://arxiv.org/abs/1503.03715
 * doi: 10.1109/TAC.2016.2593947
 */

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"
#include "go2_model.hh"

/* time profiling */
#include "TicToc.hh"
/* memory profiling */
#include <sys/time.h>
#include <sys/resource.h>
struct rusage usage;

/*
 * data types for the state space elements and input space
 * elements used in uniform grid and ode solvers
 */

/* abbrev of the type for abstract states and inputs */
using abs_type = scots::abs_type;


auto radius_post = [](state_type &r, const state_type &, const input_type &u) -> void {
    //x
    r[0] = r[0];
    //y
    r[1] = r[1];
    //yaw
    r[2] = r[2];
  
};


int main() {

  ArenaConfig cfg;

  if (!readArenaConfig("arena_config.txt", cfg)) {
    std::cout << "Could not read arena_config.txt" << std::endl;
    return 1;
  }
  else{
    std::cout << "tau = " << cfg.tau << std::endl;
    std::cout << "nint = " << cfg.nint << std::endl;

    std::cout << "state_lb = "
              << cfg.s_lb[0] << " "
              << cfg.s_lb[1] << " "
              << cfg.s_lb[2] << std::endl;

    std::cout << "state_ub = "
              << cfg.s_ub[0] << " "
              << cfg.s_ub[1] << " "
              << cfg.s_ub[2] << std::endl;

    std::cout << "state_eta = "
              << cfg.s_eta[0] << " "
              << cfg.s_eta[1] << " "
              << cfg.s_eta[2] << std::endl;

    std::cout << "input_lb = "
              << cfg.i_lb[0] << " "
              << cfg.i_lb[1] << " "
              << cfg.i_lb[2] << std::endl;

    std::cout << "input_ub = "
              << cfg.i_ub[0] << " "
              << cfg.i_ub[1] << " "
              << cfg.i_ub[2] << std::endl;

    std::cout << "input_eta = "
              << cfg.i_eta[0] << " "
              << cfg.i_eta[1] << " "
              << cfg.i_eta[2] << std::endl;

    std::cout << "targets = " << cfg.targets.size() << std::endl;
    std::cout << "obstacles = " << cfg.obstacles.size() << std::endl;
  }

  auto post = [&cfg](state_type &x, const input_type &u) {
    go2_post(x, u, cfg.tau, cfg.nint);
  };
  

  if (cfg.targets.empty()) { 
    printf("no target parsed");
    return 1; 
  }
  /* to measure time */
  TicToc tt;

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  
  scots::UniformGrid ss(state_dim, cfg.s_lb, cfg.s_ub, cfg.s_eta); 
  std::cout << "Uniform grid details:" << std::endl;
  ss.print_info();
  

  scots::UniformGrid is(input_dim, cfg.i_lb, cfg.i_ub, cfg.i_eta);
  is.print_info();

  /* set up constraint functions with obtacles */

  /* avoid function returns 1 if x is in avoid set  */
  auto avoid = [&cfg,&ss](const abs_type& idx) {
    state_type x;
    ss.itox(idx,x);
    double c1 = cfg.s_eta[0] / 2.0 + 1e-10;
    double c2 = cfg.s_eta[1] / 2.0 + 1e-10;
    for (const auto &box : cfg.obstacles) {
      if ((box[0] - c1) <= x[0] &&
        x[0] <= (box[1] + c1) &&
        (box[2] - c2) <= x[1] &&
        x[1] <= (box[3] + c2)) {
        return true;
        }
    }
    return false;
  };
  /* write obstacles to file */
  write_to_file(ss,avoid,"obstacles");

  std::cout << "Computing the transition function: " << std::endl;
  /* transition function of symbolic model */
  scots::TransitionFunction tf;
  scots::Abstraction<state_type,input_type> abs(ss,is);

  tt.tic();
  abs.compute_gb(tf,post, radius_post, avoid);
  //abs.compute_gb(tf,vehicle_post, radius_post);
  tt.toc();

  if(!getrusage(RUSAGE_SELF, &usage))
    std::cout << "Memory per transition: " << usage.ru_maxrss/(double)tf.get_no_transitions() << std::endl;
  std::cout << "Number of transitions: " << tf.get_no_transitions() << std::endl;

  /* define target set */
  auto target = [&cfg, &ss](const abs_type& idx) {
    state_type x;
    ss.itox(idx, x);
    return in_target(x, cfg);
  };
  
   /* write target to file */
  write_to_file(ss,target,"target");

 
  std::cout << "\nSynthesis: " << std::endl;
  tt.tic();
  scots::WinningDomain win=scots::solve_reachability_game(tf,target);
  tt.toc();
  std::cout << "Winning domain size: " << win.get_size() << std::endl;

  std::cout << "\nWrite controller to controller.scs \n";
  if(write_to_file(scots::StaticController(ss,is,std::move(win)),"controller"))
    std::cout << "Done. \n";

  return 1;
}
