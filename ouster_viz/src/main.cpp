#include <unistd.h>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>

#include <Eigen/Eigen>
#include <fstream>
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

/*
    Handler of saving lidar information 
*/
Eigen::ArrayXXd data_;
bool init = false;


void save_data(std::string save_path, int frame_num){
    // // test
    // std::cout << points->row(65516) << std::endl;
    // std::cout << "----------------------------------" << std::endl;


    // setting save path
    std::string txt_path = save_path + "/" + std::to_string(frame_num) + ".txt";
    std::ofstream points_txt(txt_path.c_str());
    
    if(points_txt.is_open()) {
        points_txt << data_ << '\n';
    }
    points_txt.close();
}

void handle_lidar(uint8_t* packet_buf, int W, int H, const std::vector<double>& xyz_lut) {
    
    // // data array (ex. (1024*64, 10))
    int num_points = W*H;
    data_.resize(num_points,10);

    int next_m_id{W};
    int32_t cur_f_id{-1};
    int64_t scan_ts{-1L};

    for (int icol = 0; icol < OS1::columns_per_buffer; icol++) {
        const uint8_t* col_buf = OS1::nth_col(icol, packet_buf);
        const uint16_t m_id = OS1::col_measurement_id(col_buf);
        const uint16_t f_id = OS1::col_frame_id(col_buf);
        const uint64_t ts = OS1::col_timestamp(col_buf);
        const bool valid = OS1::col_valid(col_buf) == 0xffffffff;
        const uint32_t h_encoder_cnt = OS1::col_h_encoder_count(col_buf);   // horizontal encoder count (tick : 90112)

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (!valid || m_id >= W || f_id + 1 == cur_f_id) continue;

        if (f_id != cur_f_id) {
            // if not initializing with first packet
            if (scan_ts != -1) {
                // zero out remaining missing columns
                // std::fill(it + (H * next_m_id), it + (H * W), empty);

            }

            // start new frame
            scan_ts = ts;
            next_m_id = 0;
            cur_f_id = f_id;
        }

        // zero out missing columns if we jumped forward
        if (m_id >= next_m_id) {
            // std::fill(it + (H * next_m_id), it + (H * m_id), empty);
            next_m_id = m_id + 1;
        }

        // index of the first point in current packet
        const int idx = H * m_id;   // Height (ring: 63, 64channel) * column id (0~1023)

        for (uint8_t ipx = 0; ipx < H; ipx++) {
            const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
            uint32_t r = OS1::px_range(px_buf);
            int ind = 3 * (idx + ipx);

            // // x, y, z(m), i, ts, reflectivity, ring, noise, range (mm)
            // it[idx + ipx] = c(r * 0.001f * xyz_lut[ind + 0],
            //                   r * 0.001f * xyz_lut[ind + 1],
            //                   r * 0.001f * xyz_lut[ind + 2],
            //                   OS1::px_signal_photons(px_buf), ts - scan_ts,
            //                   OS1::px_reflectivity(px_buf), ipx,
            //                   OS1::px_noise_photons(px_buf), r);
            

            // ring, encoder, timestamp, x, y, z(m), intensity, reflectivity, noise, range(mm)
            uint8_t ring_ = ipx;
            uint32_t encoder_ = h_encoder_cnt;
            uint64_t timestamp_ = ts - scan_ts;
            double x_ = r * 0.001f * xyz_lut[ind + 0];
            double y_ = r * 0.001f * xyz_lut[ind + 1];
            double z_ = r * 0.001f * xyz_lut[ind + 2];
            uint16_t intensity_ = OS1::px_signal_photons(px_buf);
            uint16_t reflectivity_ = OS1::px_reflectivity(px_buf);
            uint16_t noise_ = OS1::px_noise_photons(px_buf);
            uint32_t range_ = r;

            int indxx = idx + ipx;
            data_(indxx, 0) = ring_;
            data_(indxx, 1) = encoder_;
            data_(indxx, 2) = timestamp_;
            data_(indxx, 3) = x_;
            data_(indxx, 4) = y_;
            data_(indxx, 5) = z_;
            data_(indxx, 6) = intensity_;
            data_(indxx, 7) = reflectivity_;
            data_(indxx, 8) = noise_;
            data_(indxx, 9) = range_;

            // // test
            // if(indxx == 65516 ){
            //     std::cout << data_.row(indxx) << std::endl;
            //     std::cout << "----------------------------------" << std::endl;
            // }
            // if(ring_ == 63){
            //     std::cout << data_.row(indxx) << std::endl;
            //     std::cout << "=================================" << std::endl;
            // }

        }
         
    }

    
    
}



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

int main(int argc, char** argv) {
    int W = 1024;
    int H = OS1::pixels_per_column;
    OS1::lidar_mode mode = OS1::MODE_1024x10;
    bool do_config = true;  // send tcp commands to configure sensor
    std::string metadata{};

    /*
        save data parameters
    */
    bool do_save = false;
    int save_idx = 0;
    std::string save_path_ = "logging";     // save path


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
                if (OS1::read_lidar_packet(*cli, lidar_buf)) {
                    batch_and_update(lidar_buf, it);

                    /* 
                    Save data
                    modified by YUN */
                    handle_lidar(lidar_buf, W, H, xyz_lut);

                    init = true;

                }

            }
            if (st & OS1::client_state::IMU_DATA) {
                OS1::read_imu_packet(*cli, imu_buf);
            }
        }
    });

    // save data
    std::thread save_th([&] { 
        while(!end_program){
            if(init == false) continue;

            if(do_save == true){
                std::cout << "frame num : " << save_indx << std::endl;
                save_data(save_path_, save_idx);
            } 

            save_idx++;
        }
           
    });

    // Start render loop
    viz::run_viz(*vh);
    end_program = true;

    
   
    
                        
                   

    // clean up
    poll.join();
    save_th.join();

    return EXIT_SUCCESS;
}
