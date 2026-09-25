#include "scots.hh"
#include "../synthesis/go2_model.hh"

#include <iostream>
#include <vector>

int main() {

    ArenaConfig cfg;
    scots::StaticController con;
    state_type x = {{-1.3, 0.0, 0.0}};

    if (!readArenaConfig("arena_config.txt", cfg)) {
        std::cout << "Could not read arena_config.txt" << std::endl;
        return 1;
    }

    if (!scots::read_from_file(con, "controller")) {
        std::cout << "Could not read controller.scs" << std::endl;
        return 1;
    }

    for (int step = 0; step < 2000; ++step) {
        std::vector<input_type> u = con.get_control<state_type, input_type>(x);

        if (u.empty()) {
            std::cout << "State is outside the winning set." << std::endl;
            break;
        }

        go2_post(x, u[0], cfg.tau, cfg.nint);

        std::cout<< x[0] << " "<< x[1] << " "<< x[2] << std::endl;

        if (in_target(x, cfg)) {
            std::cout << "Reached target." << std::endl;
            break;
        }
    }   
}