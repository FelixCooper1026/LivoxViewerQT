#include "Core/SlamInputQueue.h"

#include <QMutexLocker>

#include <utility>

SlamInputQueue::SlamInputQueue(int capacity)
    : capacity_(capacity)
{
}

void SlamInputQueue::setCapacity(int capacity)
{
    QMutexLocker locker(&mutex_);
    capacity_ = capacity;
    while (frames_.size() > capacity_) {
        frames_.dequeue();
        ++droppedFrameCount_;
    }
}

int SlamInputQueue::capacity() const
{
    QMutexLocker locker(&mutex_);
    return capacity_;
}

int SlamInputQueue::size() const
{
    QMutexLocker locker(&mutex_);
    return frames_.size();
}

bool SlamInputQueue::isEmpty() const
{
    QMutexLocker locker(&mutex_);
    return frames_.isEmpty();
}

void SlamInputQueue::clear()
{
    QMutexLocker locker(&mutex_);
    frames_.clear();
}

void SlamInputQueue::push(SlamInputFrame&& frame)
{
    QMutexLocker locker(&mutex_);
    if (frames_.size() >= capacity_) {
        frames_.dequeue();
        ++droppedFrameCount_;
    }
    frames_.enqueue(std::move(frame));
    ++pushedFrameCount_;
}

bool SlamInputQueue::tryPop(SlamInputFrame& frame)
{
    QMutexLocker locker(&mutex_);
    if (frames_.isEmpty()) {
        return false;
    }
    frame = std::move(frames_.dequeue());
    ++poppedFrameCount_;
    return true;
}

SlamInputQueueStats SlamInputQueue::stats() const
{
    QMutexLocker locker(&mutex_);
    SlamInputQueueStats result;
    result.capacity = capacity_;
    result.size = frames_.size();
    result.pushedFrameCount = pushedFrameCount_;
    result.poppedFrameCount = poppedFrameCount_;
    result.droppedFrameCount = droppedFrameCount_;
    return result;
}
