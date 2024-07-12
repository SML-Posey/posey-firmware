#pragma once

#include "MessageID.hpp"

#include "platform/BaseTask.hpp"
#include "platform/sensors/BaseIMU.hpp"

#include "platform/io/BaseMessageWriter.hpp"
#include "platform/io/BufferMessagePair.hpp"

#include "TaskWatchTelemetry.hpp"

class TaskWatch : public BaseTask {
    public:
        TaskWatch(BaseIMU& imu, BaseMessageWriter& writer)
            : imu(imu), writer(writer) {}

        bool setup() override;
        void loop() override;

    protected:
        void process_message(const uint16_t mid);

    protected:
        BaseIMU& imu;

        BaseMessageWriter& writer;

        // Messages handled.
        BufferMessagePair<TaskWatchTelemetry> tm;
};
