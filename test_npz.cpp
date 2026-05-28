#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include "cnpy.h"
int main() {
    std::string base = "/home/jayson/kuka_ram_ws/data/output_npz/cylinder_test_100_50_first10layers/";
    for(auto& p: std::filesystem::recursive_directory_iterator(base)) {
        if(p.path().extension() == ".npz") {
            try {
                auto npz = cnpy::npz_load(p.path().string());
            } catch(std::exception& e) {
                std::cout << "Error loading " << p.path() << ": " << e.what() << std::endl;
            }
        }
    }
    std::cout << "Done" << std::endl;
}
