/*
 * cartpole.cc
 *
 * created: Feb 2017
 *  author: rungger
 */


#include <iostream>
#include <array>
#include <cmath>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"


/* time profiling */
#include "TicToc.hh"
/* memory profiling */
#include <sys/time.h>
#include <sys/resource.h>
struct rusage usage;


/* state space dim */
const int state_dim=4;
/* input space dim */
const int input_dim=1;
/* sampling time */
const double tau = .25;

/*
 * data types for the elements of the state space 
 * and input space used by the ODE solver
 */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

/* abbrev of the type for abstract states and inputs */
using abs_type = scots::abs_type;

/* system parameters */
const double omega=1;
const double ga=0.0125;

/* we integrate the cart pole system  (the result is stored in x)  */
auto  sys = [](state_type &x, const input_type &u) noexcept {

  /* the ode describing the system */
  auto  rhs = [](state_type &dxdt, const state_type &x, const input_type &u) noexcept {
    dxdt[0] = x[1];
    dxdt[1] = -omega*omega*(std::sin(x[0])+u[0]*std::cos(x[0]))-2*ga*x[1];
    dxdt[2] = x[3];
    dxdt[3] = u[0]-x[3];
    //dxdt[0] = x[1];
    //dxdt[1] = u[0];

  };
  scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius = [](state_type &r, const state_type&, const input_type &u) noexcept {

  auto rhs = [](state_type &drdt,  const state_type &r, const input_type &u) noexcept {
	drdt[0] = r[1];
    drdt[1] = omega*omega*(1+std::abs(u[0]))*r[0]-2*ga*r[1];
    drdt[2] = r[3];
    drdt[3] = -r[3];

	/* 
drdt[0] = 0;
    drdt[1] = 0;
    drdt[2] = 0;
    drdt[3] = 0;
 */
    //drdt[0] = r[1];
    //drdt[1] = 0;
  };
  scots::runge_kutta_fixed4(rhs,r,u,state_dim,tau);
};



int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
//  Cudd manager;
  /* enable variable reordering */
//  manager.AutodynEnable();

  /* construct SymbolicSet for the state space */
  // working_1
  //state_type s_lb={{ 0.755*M_PI, -1, -2, -2}};
  //state_type s_ub={{ 1.25*M_PI,  1,  2,  2}}; 
  
  state_type s_lb={{ 0.75*M_PI, -1, -2, -2}};
  state_type s_ub={{ 1.25*M_PI,  1,  2,  2}};  
  
  /* grid node distance diameter */
  
  //working_1
  //state_type s_eta={{.05,.1,.1,.1}};   working_1
  
  state_type s_eta={{.05,.1,.3,.3}};
  
  /* construct SymbolicSet for the input space */
  input_type i_lb={{-5}};  
  input_type i_ub={{5}};  
  input_type i_eta={{.1}};   
  

//#if 0
  scots::UniformGrid ss(state_dim,s_lb,s_ub,s_eta);
  ss.print_info();

  scots::UniformGrid is(input_dim,i_lb,i_ub,i_eta);
  is.print_info();

  /* compute transition function of symbolic model */
  std::cout << "Computing the transition function:\n";

  /* transition function of symbolic model */
  scots::TransitionFunction tf;
  scots::Abstraction<state_type,input_type> abs(ss,is);

  tt.tic();
  abs.compute_gb(tf,sys, radius);
  tt.toc();

  if(!getrusage(RUSAGE_SELF, &usage))
    std::cout << "Memory per transition: " << usage.ru_maxrss/(double)tf.get_no_transitions() << std::endl;
  std::cout << "Number of transitions: " << tf.get_no_transitions() << std::endl;

  /* compute winning domain (contains also valid inputs) */
  /* continue with synthesis */
  auto safeset = [&ss,&s_eta,&s_ub,&s_lb](const scots::abs_type& idx) {
    state_type x;
    ss.itox(idx,x);
    double c[state_dim];
    c[0]= s_eta[0]/2.0+1e-10;
    c[1]= s_eta[1]/2.0+1e-10;
    c[2]= s_eta[2]/2.0+1e-10;
    c[3]= s_eta[3]/2.0+1e-10;

    if ((s_lb[0]+c[0]) <= x[0] && x[0] <= (s_ub[0]-c[0]) && 
        (s_lb[1]+c[1]) <= x[1] && x[1] <= (s_ub[1]-c[1]) && 
        (s_lb[2]+c[2]) <= x[2] && x[2] <= (s_ub[2]-c[2]) && 
        (s_lb[3]+c[3]) <= x[3] && x[3] <= (s_ub[3]-c[3]) ) {
      return true;
    }
    return false;
  };
  std::cout << "\nSynthesis: \n";
  tt.tic();
  scots::WinningDomain win = scots::solve_invariance_game(tf,safeset);
  tt.toc();
  std::cout << "Winning domain size: " << win.get_size() << "\n";
  std::cout << "Winning domain percent: " << (double)win.get_size()/ss.size() << "\n";

  std::cout << "\nWrite controller to file \n";
  if(write_to_file(scots::StaticController(ss,is,std::move(win)),"cartpole_sparse"))
    std::cout << "Done. \n";

//#else

 //  scots::SymbolicSet ss_pre(manager,state_dim,s_lb,s_ub,s_eta);
//   ss_pre.print_info();
//   scots::SymbolicSet ss_input(manager,input_dim,i_lb,i_ub,i_eta);
//   ss_input.print_info();
// 
//   scots::SymbolicSet ss_post(manager,state_dim,s_lb,s_ub,s_eta);
// 
//   /* compute transition function of symbolic model */
//   std::cout << "Computing the transition function:\n";
// 
//   /* SymbolicModel class to compute the BDD encoding the transition function */ 
//   scots::SymbolicModel<state_type,input_type> sym_model(ss_pre,ss_input,ss_post);
//   tt.tic();
//   size_t no_trans;
//   BDD TF = sym_model.compute_gb(manager,sys,radius,no_trans);
//   tt.toc();
// 
//   std::cout << "No of Transitions " << no_trans  << "\n";
//   if(!getrusage(RUSAGE_SELF, &usage)) {
//     std::cout << "Memory pro Transition: " << usage.ru_maxrss/(double)no_trans<< "\n";
//   }
// 
//   /* continue with synthesis */
//   auto safeset = [&ss_pre,&s_eta,&s_ub,&s_lb](const scots::abs_type& idx) {
//     state_type x;
//     ss_pre.itox(idx,x);
//     double c[state_dim];
//     c[0]= s_eta[0]/2.0+1e-10;
//     c[1]= s_eta[1]/2.0+1e-10;
//     c[2]= s_eta[2]/2.0+1e-10;
//     c[3]= s_eta[3]/2.0+1e-10;
// 
//     if ((s_lb[0]+c[0]) <= x[0] && x[0] <= (s_ub[0]-c[0]) && 
//         (s_lb[1]+c[1]) <= x[1] && x[1] <= (s_ub[1]-c[1]) && 
//         (s_lb[2]+c[2]) <= x[2] && x[2] <= (s_ub[2]-c[2]) && 
//         (s_lb[3]+c[3]) <= x[3] && x[3] <= (s_ub[3]-c[3]) ) {
//       return true;
//     }
//     return false;
//   };
// 
// 
// 
//   BDD S = ss_pre.ap_to_bdd(manager,safeset);
// 
//   /* we continue with the controller synthesis for G (S) 
//    *
//    * we implement the fixed point algorithm 
//    *
//    * nu X. ( pre(X) & S ) 
//    *
//    */
// 
//   /* setup enforcable predecessor */
//   scots::EnfPre enf_pre(manager,TF,sym_model);
//   tt.tic();
//   BDD X = manager.bddZero();
//   BDD XX =manager.bddOne();
//   /* as long as not converged */
//   size_t i;
//   for(i=1; XX != X; i++ ) {
//     X=XX;
//     XX=enf_pre(X) & S;
//     /* print progress */
//     scots::print_progress(i);
//   }
//   std::cout << "\nNumber of iterations: " << i << std::endl;
//   tt.toc();
// 
//   std::cout << "Winning domain size: " << ss_pre.get_size(manager,X) << std::endl;
//   
//   /* symbolic set for the controller */
//   scots::SymbolicSet controller(ss_pre,ss_input);
//   std::cout << "\nWrite controller to file \n";
//   if(write_to_file(manager,controller,X,"cartpole"))
//     std::cout << "Done. \n";
// #endif


  return 1;
}
