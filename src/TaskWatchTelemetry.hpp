#pragma once

#include "MessageID.hpp"

#include "platform/io/BufferSerializer.hpp"

class TaskWatchTelemetry {
    public:
        static constexpr uint8_t message_id = MessageID::TaskWatch;
        static constexpr uint16_t MessageSize = 1        // Message ID
                                                + 4 * 2  // Times
                                                + 1 * 1  // Invalid checksum
                                                + 1 * 1  // Missed deadline
                                                + 1 * 1  // Battery voltage.
            ;
        typedef BufferSerializer<MessageSize> Buffer;

    public:
        void serialize(Buffer& buffer) const {
            static auto message_id = this->message_id;
            buffer.reset();
            buffer.write_syncword()
                .write(message_id)
                .write(t_start_ms)
                .write(t_end_ms)
                .write(invalid_checksum)
                .write(missed_deadline)
                .write(Vbatt)
                .write_checksum();
        }

        bool deserialize(Buffer& buffer) {
            buffer.rewind();
            buffer.read<uint16_t>();  // Syncword.
            buffer.read<uint8_t>();   // Message ID.
            buffer.read(t_start_ms)
                .read(t_end_ms)
                .read(invalid_checksum)
                .read(missed_deadline)
                .read(Vbatt);
            // Checksum.
            return true;
        }

    public:
        uint32_t t_start_ms = 0;
        uint32_t t_end_ms = 0;

        uint8_t invalid_checksum = 0;
        uint8_t missed_deadline = 0;

        uint8_t Vbatt = 0;
};
