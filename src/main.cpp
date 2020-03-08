#include <unistd.h>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>

#include <Eigen/Eigen>
#include <fstream>
#include <experimental/filesystem>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster/viz.h"

namespace OS1 = ouster::OS1;
namespace viz = ouster::viz;

/**
 * Print usage
 */
void print_help() {
    std::cout
        << "Usage: viz [options] [hostname] [udp_destination]\n"
        << "Options:\n"
        << "  -m <512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10> : lidar mode, "
           "default 1024x10\n"
        << "  -f <path> : use provided metadata file; do not configure via TCP"
        << std::endl;
}

std::string read_metadata(const std::string& meta_file) {
    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(meta_file);
    buf << ifs.rdbuf();
    ifs.close();

    if (!ifs) {
        std::cerr << "Failed to read " << meta_file
                  << "; check that the path is valid" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return buf.str();
}

//////////////////////////////////////////////////////////////////////////////
/*
Save data
YUN
*/
void save_data(std::unique_ptr<ouster::LidarScan>& ls, int frame_num, std::string save_path) {

    std::cout << "size : " << ls->x().size() << std::endl;

    Eigen::MatrixXd points(ls->x().size(), 6);
    points.col(0) = ls->x();
    points.col(1) = ls->y();
    points.col(2) = ls->z();
    points.col(3) = ls->intensity();
    points.col(4) = ls->noise();
    points.col(5) = ls->range();

    std::string txt_path = save_path + "/" + std::to_string(frame_num) + ".txt";
    // std::string txt_path = std::to_string(frame_num) + ".txt";
    std::ofstream points_txt(txt_path.c_str());
    std::cout << txt_path << std::endl;

    if(points_txt.is_open()) {
        points_txt << points << '\n';
    }
    points_txt.close();

}
////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    int W = 1024;
    int H = OS1::pixels_per_column;
    OS1::lidar_mode mode = OS1::MODE_1024x10;
    bool do_config = true;  // send tcp commands to configure sensor
    std::string metadata{};

    bool do_save = false;
    int save_idx = 0;
    // save path
    std::string save_path_ = "logging";


    try {
        int c = 0;
        while ((c = getopt(argc, argv, "hm:f:s:")) != -1) {
            switch (c) {
                case 'h':
                    print_help();
                    return 1;
                    break;
                case 'm':
                    mode = OS1::lidar_mode_of_string(optarg);
                    if (mode) {
                        W = OS1::n_cols_of_lidar_mode(mode);
                    } else {
                        std::cout << "Lidar Mode must be 512x10, 512x20, "
                                     "1024x10, 1024x20, or 2048x10"
                                  << std::endl;
                        print_help();
                        std::exit(EXIT_FAILURE);
                    }
                    break;
                case 'f':
                    do_config = false;
                    metadata = read_metadata(optarg);
                    break;
                case 's':
                    do_save = optarg;
                    break;
                case '?':
                    std::cout << "Invalid Argument Format" << std::endl;
                    print_help();
                    std::exit(EXIT_FAILURE);
                    break;
            }
        }
    } catch (const std::exception& ex) {
        std::cout << "Invalid Argument Format: " << ex.what() << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

    if (do_config && argc != optind + 2) {
        std::cerr << "Expected 2 arguments after options" << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

    std::shared_ptr<OS1::client> cli;
    if (do_config) {
        std::cout << "Configuring sensor: " << argv[optind]
                  << " UDP Destination:" << argv[optind + 1] << std::endl;
        cli = OS1::init_client(argv[optind], argv[optind + 1], mode);
    } else {
        std::cout << "Listening for sensor data" << std::endl;
        cli = OS1::init_client();
    }

    if (!cli) {
        std::cerr << "Failed to initialize client" << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

    uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    uint8_t imu_buf[OS1::imu_packet_bytes + 1];

    auto ls = std::unique_ptr<ouster::LidarScan>(new ouster::LidarScan(W, H));

    auto vh = viz::init_viz(W, H);

    if (do_config) metadata = OS1::get_metadata(*cli);

    auto info = OS1::parse_metadata(metadata);

    auto xyz_lut = OS1::make_xyz_lut(W, H, info.beam_azimuth_angles,
                                     info.beam_altitude_angles);

    // Use to signal termination
    std::atomic_bool end_program{false};

    // auto it = std::back_inserter(*ls);
    auto it = ls->begin();

    
    // callback that calls update with filled lidar scan
    auto batch_and_update = OS1::batch_to_iter<ouster::LidarScan::iterator>(
        xyz_lut, W, H, ouster::LidarScan::Point::Zero(),
        &ouster::LidarScan::make_val, [&](uint64_t) {

            //////// save data - YUN
            if((save_idx%10)==0 && (do_save==true)) {
                save_data(ls, save_idx, save_path_);
            }
            save_idx++;
            /////////

            // swap lidar scan and point it to new buffer
            viz::update(*vh, ls);
            it = ls->begin();
        });

    // Start poll thread
    std::thread poll([&] {
        while (!end_program) {
            // Poll the client for data and add to our lidar scan
            OS1::client_state st = OS1::poll_client(*cli);
            if (st & OS1::client_state::ERROR) {
                std::cerr << "Client returned error state" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (st & OS1::client_state::LIDAR_DATA) {
                if (OS1::read_lidar_packet(*cli, lidar_buf))
                    batch_and_update(lidar_buf, it);
            }
            if (st & OS1::client_state::IMU_DATA) {
                OS1::read_imu_packet(*cli, imu_buf);
            }
        }
    });

    // Start render loop
    viz::run_viz(*vh);
    end_program = true;

    // clean up
    poll.join();
    return EXIT_SUCCESS;
}
