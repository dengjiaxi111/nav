#include "play_music/async_audio_player.hpp"
#include <iostream>
#include <chrono>
#include <thread>

#define MINIAUDIO_IMPLEMENTATION
#include "play_music/miniaudio.h"

AsyncAudioPlayer::AsyncAudioPlayer(const std::string& filepath)
    : filePath(filepath), stopFlag(false)
{
    if (ma_engine_init(NULL, &engine) != MA_SUCCESS) {
        std::cerr << "Failed to initialize audio engine!" << std::endl;
        return;
    }

    if (ma_sound_init_from_file(&engine, filePath.c_str(), 0, NULL, NULL, &sound) != MA_SUCCESS) {
        std::cerr << "Failed to load sound file: " << filePath << std::endl;
        ma_engine_uninit(&engine);
        return;
    }

    ma_sound_start(&sound);

    workerThread = std::thread([this]() {
        while (!stopFlag && ma_sound_is_playing(&sound)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        ma_sound_uninit(&sound);
        ma_engine_uninit(&engine);
    });
}

AsyncAudioPlayer::~AsyncAudioPlayer() {
    stopFlag = true;
    if (workerThread.joinable()) {
        workerThread.join();
    }
}
