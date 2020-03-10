#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"

namespace OS1 = ouster::OS1;

uint64_t n_lidar_packets = 0;
uint64_t n_imu_packets = 0;

uint64_t lidar_col_0_ts = 0;
uint64_t imu_ts = 0;

float lidar_col_0_h_angle = 0.0;
float imu_av_z = 0.0;
float imu_la_y = 0.0;

uint32_t H = OS1::pixels_per_column;
uint32_t W = OS1::n_cols_of_lidar_mode(OS1::lidar_mode_of_string(cfg.response.lidar_mode));

auto xyz_lut = OS1::make_xyz_lut(W, H, cfg.response.beam_azimuth_angles,
                                     cfg.response.beam_altitude_angles);

void handle_lidar(uint8_t* buf, const std::vector<double>& xyz_lut) {
    // n_lidar_packets++;
    // lidar_col_0_ts = OS1::col_timestamp(OS1::nth_col(0, buf));
    // lidar_col_0_h_angle = OS1::col_h_angle(OS1::nth_col(0, buf));

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
        const float h_angle = OS1::col_h_angle(col_buf);    // horizontal angle

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (!valid || m_id >= W || f_id + 1 == cur_f_id) continue;

        if (f_id != cur_f_id) {
            // if not initializing with first packet
            if (scan_ts != -1) {
                // zero out remaining missing columns
                std::fill(it + (H * next_m_id), it + (H * W), empty);
                f(scan_ts);
            }

            // start new frame
            scan_ts = ts;
            next_m_id = 0;
            cur_f_id = f_id;
        }

        // zero out missing columns if we jumped forward
        if (m_id >= next_m_id) {
            std::fill(it + (H * next_m_id), it + (H * m_id), empty);
            next_m_id = m_id + 1;
        }

        // index of the first point in current packet
        const int idx = H * m_id;

        for (uint8_t ipx = 0; ipx < H; ipx++) {
            const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
            uint32_t r = OS1::px_range(px_buf);
            int ind = 3 * (idx + ipx);

            // x, y, z(m), i, ts, reflectivity, ring, noise, range (mm)
            it[idx + ipx] = c(r * 0.001f * xyz_lut[ind + 0],
                              r * 0.001f * xyz_lut[ind + 1],
                              r * 0.001f * xyz_lut[ind + 2],
                              OS1::px_signal_photons(px_buf), ts - scan_ts,
                              OS1::px_reflectivity(px_buf), ipx,
                              OS1::px_noise_photons(px_buf), r);
        }



    }




}

void handle_imu(uint8_t* buf) {
    n_imu_packets++;
    imu_ts = OS1::imu_sys_ts(buf);
    imu_av_z = OS1::imu_av_z(buf);
    imu_la_y = OS1::imu_la_y(buf);
}

void print_headers() {
    std::cout << std::setw(15) << "n_lidar_packets" << std::setw(15)
              << "col_0_azimuth" << std::setw(15) << "col_0_ts" << std::setw(15)
              << "n_imu_packets" << std::setw(15) << "im_av_z" << std::setw(15)
              << "im_la_y" << std::setw(15) << "imu_ts" << std::endl;
}

void print_stats() {
    std::cout << "\r" << std::setw(15) << n_lidar_packets << std::setw(15)
              << lidar_col_0_h_angle << std::setw(15) << lidar_col_0_ts
              << std::setw(15) << n_imu_packets << std::setw(15) << imu_av_z
              << std::setw(15) << imu_la_y << std::setw(15) << imu_ts;
    std::flush(std::cout);
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: ouster_client_example <os1_hostname> "
                     "<data_destination_ip>"
                  << std::endl;
        return 1;
    }

    auto cli = OS1::init_client(argv[1], argv[2]);
    if (!cli) {
        std::cerr << "Failed to connect to client at: " << argv[1] << std::endl;
        return 1;
    }

    uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    uint8_t imu_buf[OS1::imu_packet_bytes + 1];

    print_headers();

    while (true) {
        OS1::client_state st = OS1::poll_client(*cli);
        if (st & OS1::ERROR) {
            return 1;
        } else if (st & OS1::LIDAR_DATA) {
            if (OS1::read_lidar_packet(*cli, lidar_buf))
                handle_lidar(lidar_buf);
        } else if (st & OS1::IMU_DATA) {
            if (OS1::read_imu_packet(*cli, imu_buf)) handle_imu(imu_buf);
        }

        if (n_imu_packets % 50 == 0) print_stats();
    }

    return 0;
}
