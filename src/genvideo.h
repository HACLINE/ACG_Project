#ifndef GENVIDEO_H
#define GENVIDEO_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

void generateVideo(YAML::Node config, const std::string& figuresPath, std::string outputVideoPath = "video.avi") {
    int fps = config["fps"].as<int>();
    float videoLength = config["length"].as<float>();
    int frameCount = int(videoLength * fps);

    std::vector<std::string> imageFiles;

    for (const auto& entry : fs::directory_iterator(figuresPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            imageFiles.push_back(entry.path().string());
        }
    }

    std::sort(imageFiles.begin(), imageFiles.end());

    if (imageFiles.empty()) {
        std::cerr << "No images found in the directory!" << std::endl;
        return ;
    }

    cv::Mat firstFrame = cv::imread(imageFiles[0]);
    if (firstFrame.empty()) {
        std::cerr << "Failed to read the first image!" << std::endl;
        return ;
    }

    cv::Size frameSize(firstFrame.cols, firstFrame.rows);

    cv::VideoWriter videoWriter(outputVideoPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frameSize);

    if (!videoWriter.isOpened()) {
        std::cerr << "[Error] Failed to open video path!" << std::endl;
        return ;
    }

    int count = 0;
    for (const auto& imageFile : imageFiles) {
        cv::Mat frame = cv::imread(imageFile);
        if (frame.empty()) {
            std::cerr << "[Error] Failed to read image: " << imageFile << ". Continue to generate without it..." << std::endl;
            continue;
        }
        count++;
        videoWriter.write(frame);
        if (count >= frameCount) {
            break;
        }
    }

    if (count < frameCount) {
        std::cerr << "[WARNING] Image not enough: expected " << frameCount << ", but only have " << count << std::endl;
    }

    videoWriter.release();

    std::cout << "[Output] Video generated at: " << outputVideoPath << std::endl;

    return ;
}

#endif