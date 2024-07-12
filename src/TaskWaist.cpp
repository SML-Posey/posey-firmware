#include "platform.hpp"
#include "platform/config.h"

#include "TaskWaist.hpp"

#include "MessageAck.hpp"

#if defined(CONFIG_LOG)
    #include <zephyr/logging/log.h>
    #include <zephyr/logging/log_ctrl.h>
    #include <zephyr/sys/reboot.h>

    #include "posey-platform/platform/io/NordicNUSDriver.h"
    #include "posey-platform/platform/io/NordicSAADACDriver.h"

    #define LOG_MODULE_NAME posey_task_hub
LOG_MODULE_REGISTER(LOG_MODULE_NAME);
#else
    #define LOG_INF(...) ;
    #define LOG_WRN(...) ;

    #ifdef __cplusplus
extern "C" {
    #endif

int init_nus() {
    return 0;
}

bool init_flash() {
    return true;
}
bool erase_flash(const uint32_t size) {
    return true;
}
bool erase_used_flash() {
    return true;
}
bool erase_all_flash() {
    return true;
}

bool flash_is_logging() {
    return true;
}
void start_flash_logging() {}
void stop_flash_logging() {}

uint32_t flash_log_size();

void process_data(
    void* conn, const uint8_t slot, const uint8_t* data, const uint16_t size);

    #ifdef __cplusplus
}
    #endif

#endif

bool TaskWaist::setup() {
    bool success = imu.setup();
    success &= ble.setup();
    success &= ml.add_listener(cmd);
    return success;
}

void TaskWaist::loop() {
#if defined(CONFIG_ROLE_HUB)
    static const uint32_t max_loop_time = 1e3 / 50;
    static uint32_t iter = 0;

    tm.message.t_start_ms = Clock::get_msec<uint32_t>();

    // // Update sensor data.
    imu.collect();
    ble.collect();  // Does nothing.

    // Serialize data and store to flash.
    imu.serialize();
    process_data(NULL, 0xFA, imu.buffer.get_buffer(), imu.buffer.used());

    BLEData elem;
    while (ble.ring_buffer.read_next(elem)) {
        elem.serialize(ble.buffer);
        process_data(NULL, 0xFA, ble.buffer.get_buffer(), ble.buffer.used());
    }

    // Check for messages. Process everything available, we'll
    // let this task overrun.
    ml.poll(reader);
    while (true) {
        auto mid = ml.process_next();
        if (mid <= -1)
            break;
        process_message(mid);
    }

    // Send task TM at 1Hz.
    static float Vbatt = 0;
    static int lowbat = 0;

    Vbatt += read_Vbatt();
    if (iter % 50 == 0) {
        Vbatt /= 50.0;

        if (Vbatt < 3.3) {
            LOG_WRN("LOW Average Vbat: %.2f V", Vbatt);
            if (lowbat++ >= 1000) {
                LOG_ERR(
                    "Low battery: %.2f V! Sleeping for 5 minutes...", Vbatt);

                // Flush the log buffer before rebooting.
                if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED))
                    while (log_process())
                        ;

                // Disable scanning and close connections.
                disable_scanning();
                close_connections();

                deep_sleep();

                // Reboot to start fresh scanning.
                LOG_INF("Rebooting...");
                sys_reboot(SYS_REBOOT_COLD);
                lowbat = 0;
            }
        } else {
            // With normal voltages only log every 10s so it's not annoying.
            if (iter % (50 * 10) == 0)
                LOG_INF("NORMAL Average Vbat: %.2f V", Vbatt);
            lowbat = 0;
        }

        if (Vbatt < 3.2)
            Vbatt = 3.2;
        if (Vbatt > 4.2)
            Vbatt = 4.2;
        tm.message.Vbatt = (read_Vbatt() - 3.2) / 4.2 * 255;
        tm.message.ble_throughput = num_connected_sensors();
        tm.serialize();
        writer.write(tm.buffer, writer.SendNow);
        process_data(NULL, 0xFA, tm.buffer.get_buffer(), tm.buffer.used());
    }
    ++iter;

    // Update missed deadlines if needed.
    tm.message.t_end_ms = Clock::get_msec<uint32_t>();
    if (tm.message.t_end_ms - tm.message.t_start_ms > max_loop_time) {
        tm.message.missed_deadline++;
    }
#endif
}

bool TaskWaist::send_log_data() {
#if defined(CONFIG_ROLE_HUB)
    static constexpr uint16_t nus_buffer_size = 244;
    static uint8_t buffer[nus_buffer_size];
    uint32_t log_size = flash_log_size();
    int32_t to_send = log_size;
    LOG_INF(
        "Starting download of %d bytes, %.2f MB", to_send,
        to_send / 1024.0 / 1024.0);
    uint32_t cursor = 0;
    int iter = 0;
    struct bt_conn* pc_conn = get_pc_connection();
    if (!pc_conn) {
        LOG_ERR("PC seems to have been disconnected.");
        return false;
    }
    uint32_t t0 = Clock::get_msec<uint32_t>();
    uint32_t b0 = 0;
    while (to_send > 0) {
        to_send -= nus_buffer_size;
        uint32_t send_size = nus_buffer_size;
        if (to_send < 0)
            send_size += to_send;
        uint32_t read_size = read_flash(cursor, buffer, send_size);
        cursor += read_size;
        if (read_size != send_size) {
            LOG_ERR(
                "Read only %d of %d bytes! Bailing download.", read_size,
                send_size);
            return false;
        }
        int rc = bt_nus_send(pc_conn, buffer, send_size);
        if (rc != 0) {
            LOG_ERR("Failed to send NUS buffer! %d", rc);
            return false;
        }
        // Clock::delay_msec(10);

        if ((iter++ % 1000) == 0) {
            uint32_t t1 = Clock::get_msec<uint32_t>();
            uint32_t b1 = cursor;
            LOG_INF(
                "Left to send: %.2f of %.2f MB (%.2f%%, %.2f KBps)",
                to_send / 1024.0 / 1024.0, log_size / 1024.0 / 1024.0,
                100.0 * to_send / log_size,
                (b1 - b0) / 1024.0 / (t1 - t0) * 1000.0);
            b0 = b1;
            t0 = t1;
        }
    }

    LOG_INF("Finished download!");
#endif
    return true;
}

void TaskWaist::process_message(const uint16_t mid) {
#if defined(CONFIG_ROLE_HUB)
    bool invalid_checksum = false;

    // Command message?
    if (mid == Command::message_id) {
        // Extract command.
        if (cmd.valid_checksum()) {
            auto& msg = cmd.deserialize();
            if ((msg.command == Command::StartCollecting) ||
                (msg.command == Command::DownloadData) ||
                (msg.command == Command::FullFlashErase)) {
                cmd.message.ack = MessageAck::Working;
            } else
                cmd.message.ack = MessageAck::OK;
            cmd.serialize();
            writer.write(cmd, writer.SendNow);
            Clock::delay_msec(500);

            LOG_INF("Received %s command", msg.command_str());
            LOG_INF(
                "  - Logging to flash? %s", flash_is_logging() ? "YES" : "NO");

            // Need to stop logging?
            bool need_to_stop_logging =
                (msg.command == Command::Reboot) ||
                (msg.command == Command::DownloadData) ||
                (msg.command == Command::StopCollecting) ||
                (msg.command == Command::FullFlashErase);
            if (need_to_stop_logging && flash_is_logging()) {
                stop_flash_logging();
                config_update_data_end_ms(Clock::get_msec<uint32_t>());
                Clock::delay_msec(1000);
                refresh_device_config();
            }

            // Handle command.
            switch (msg.command) {
                case Command::NoOp:
                    LOG_WRN("No operation!");
                    break;

                case Command::Reboot:
                    LOG_INF("Waiting 5s then rebooting.");
                    Clock::delay_msec(5000);
                    sys_reboot(SYS_REBOOT_COLD);
                    break;

                case Command::GetDataSummary:
                    // Send data summary.
                    refresh_device_config();
                    memcpy(
                        tm_data_summary.message.datetime, device_config.data_dt,
                        DataSummary::MaxDatetimeSize);
                    tm_data_summary.message.start_ms =
                        device_config.data_start_ms;
                    tm_data_summary.message.end_ms = device_config.data_end_ms;
                    tm_data_summary.message.bytes = device_config.data_end;

                    LOG_INF("Sending summary packet.");
                    tm_data_summary.serialize();
                    writer.write(tm_data_summary, writer.SendNow);

                    break;

                case Command::DownloadData:
                    // Disable sensors.
                    disable_sensors();

                    // Send data.
                    LOG_INF("Initiating data download.");
                    cmd.message.ack = MessageAck::OK;

                    // Send out all flash data.
                    if (!send_log_data())
                        cmd.message.ack = MessageAck::Error;

                    // Reenable sensors.
                    enable_sensors();

                    // Send message indicating we're finished.
                    cmd.serialize();
                    writer.write(cmd, writer.SendNow);

                    break;

                case Command::FullFlashErase:
                    // Reset all FLASH.
                    LOG_INF("Erasing ALL flash, might take a bit...");
                    erase_all_flash();

                    // Send ack.
                    cmd.message.ack = MessageAck::OK;
                    cmd.serialize();
                    writer.write(cmd, writer.SendNow);

                    break;

                case Command::StartCollecting:
                    // Initiate logging start.
                    cmd.message.payload[DataSummary::MaxDatetimeSize - 1] = 0;
                    config_update_data_dt(
                        reinterpret_cast<const char*>(cmd.message.payload));
                    config_update_data_end_ms(0);
                    config_update_data_start_ms(Clock::get_msec<uint32_t>());
                    config_update_data_end(0);
                    refresh_device_config();
                    memcpy(
                        tm_data_summary.message.datetime, device_config.data_dt,
                        DataSummary::MaxDatetimeSize);

                    // Reset FLASH.
                    LOG_INF("Erasing flash, might take a bit...");
                    erase_used_flash();

                    // Start collection.
                    LOG_INF("Starting collection at...");
                    start_flash_logging();

                    // Send ack.
                    cmd.message.ack = MessageAck::OK;
                    cmd.serialize();
                    writer.write(cmd, writer.SendNow);

                    break;

                case Command::StopCollecting:
                    // Don't need to do anything.
                    break;

                case Command::Configure:
                case Command::ConnectPeripheral:
                    LOG_WRN("Not implemented.");
                    break;
                default:
                    LOG_WRN("Unknown command");
                    break;
            }

            LOG_INF(
                "  - Still to flash? %s; Data end: %d",
                flash_is_logging() ? "YES" : "NO", device_config.data_end);
        } else {
            invalid_checksum = true;
            cmd.message.ack = MessageAck::Resend;
            cmd.serialize();
            writer.write(cmd, writer.SendNow);
        }
    }

    // Invalid checksum?
    if (invalid_checksum) {
        tm.message.invalid_checksum++;
    }
#endif
}
