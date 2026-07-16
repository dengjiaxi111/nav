#pragma once

#include <string>
#include <thread>
#include <atomic>

#include "miniaudio.h"

class AsyncAudioPlayer {
public:
    explicit AsyncAudioPlayer(const std::string& filepath);
    ~AsyncAudioPlayer();

private:
    std::string filePath;
    ma_engine engine{};
    ma_sound sound{};
    std::thread workerThread;
    std::atomic<bool> stopFlag;
};
