#ifndef SLAM_CORE_SLAMINPUTQUEUE_H
#define SLAM_CORE_SLAMINPUTQUEUE_H

#include "Core/SlamTypes.h"

#include <QMutex>
#include <QQueue>

#include <cstdint>

struct SlamInputQueueStats {
    int capacity = 0;
    int size = 0;
    uint64_t pushedFrameCount = 0;
    uint64_t poppedFrameCount = 0;
    uint64_t droppedFrameCount = 0;
};

class SlamInputQueue {
public:
    explicit SlamInputQueue(int capacity = 8);

    void setCapacity(int capacity);
    int capacity() const;
    int size() const;
    bool isEmpty() const;
    void clear();
    void push(SlamInputFrame&& frame);
    bool tryPop(SlamInputFrame& frame);
    SlamInputQueueStats stats() const;

private:
    mutable QMutex mutex_;
    QQueue<SlamInputFrame> frames_;
    int capacity_ = 8;
    uint64_t pushedFrameCount_ = 0;
    uint64_t poppedFrameCount_ = 0;
    uint64_t droppedFrameCount_ = 0;
};

#endif // SLAM_CORE_SLAMINPUTQUEUE_H
