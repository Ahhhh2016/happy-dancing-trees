#include "animation.h"
#include <iostream>
#include <fstream>
#include <algorithm>

Animation::Animation()
    : m_recording(false)
    , m_playing(false)
    , m_looping(true)
    , m_playbackTime(0.0f)
    , m_duration(0.0f)
{
}

void Animation::startRecording() {
    m_keyframes.clear();
    m_recording = true;
    m_playing = false;
    m_duration = 0.0f;
    m_recordStartTime = std::chrono::steady_clock::now();
    std::cout << "Animation recording started" << std::endl;
}

void Animation::stopRecording() {
    m_recording = false;
    if (!m_keyframes.empty()) {
        m_duration = m_keyframes.back().timestamp;
    }
    std::cout << "Animation recording stopped. " << m_keyframes.size()
              << " frames, " << m_duration << " seconds" << std::endl;
}

bool Animation::isRecording() const {
    return m_recording;
}

void Animation::recordFrame(const std::vector<Eigen::Vector3f>& vertices, const std::vector<Eigen::Vector3f>& anchorPositions) {
    if (!m_recording) return;

    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - m_recordStartTime).count();

    // record frames at short intervals
    float minFrameInterval = 1.0f / 30.0f;
    if (!m_keyframes.empty() && elapsed - m_keyframes.back().timestamp < minFrameInterval) {
        return;
    }

    Keyframe frame;
    frame.timestamp = elapsed;
    frame.vertices = vertices;
    frame.anchorPositions = anchorPositions;
    m_keyframes.push_back(frame);
}

void Animation::startPlayback() {
    if (m_keyframes.empty()) {
        std::cerr << "No animation data to play" << std::endl;
        return;
    }
    m_playing = true;
    m_recording = false;
    m_playbackTime = 0.0f;
    m_playbackStartTime = std::chrono::steady_clock::now();
    std::cout << "Animation playback started" << std::endl;
}

void Animation::stopPlayback() {
    m_playing = false;
    std::cout << "Animation playback stopped" << std::endl;
}

bool Animation::isPlaying() const {
    return m_playing;
}

void Animation::setLooping(bool loop) {
    m_looping = loop;
}

bool Animation::isLooping() const {
    return m_looping;
}

bool Animation::getFrameAtTime(float time, std::vector<Eigen::Vector3f>& outVertices) const {
    if (m_keyframes.empty()) return false;

    // looping
    if (m_looping && m_duration > 0.0f) {
        time = std::fmod(time, m_duration);
        if (time < 0.0f) time += m_duration;
    }

    // get keyframes
    if (m_keyframes.size() == 1) {
        outVertices = m_keyframes[0].vertices;
        return true;
    }

    // find keyframes to interpolate between
    size_t idx = 0;
    for (size_t i = 0; i < m_keyframes.size() - 1; i++) {
        if (m_keyframes[i + 1].timestamp > time) {
            idx = i;
            break;
        }
        idx = m_keyframes.size() - 2;
    }

    const Keyframe& kf1 = m_keyframes[idx];
    const Keyframe& kf2 = m_keyframes[idx + 1];

    if (outVertices.size() != kf1.vertices.size()) {
        outVertices.resize(kf1.vertices.size());
    }

    // interpolate between keyframes
    float range = kf2.timestamp - kf1.timestamp;
    float t = (range > 0.0001f) ? (time - kf1.timestamp) / range : 0.0f;
    t = std::clamp(t, 0.0f, 1.0f);

    for (size_t i = 0; i < kf1.vertices.size(); i++) {
        outVertices[i] = lerp(kf1.vertices[i], kf2.vertices[i], t);
    }

    return true;
}

float Animation::getDuration() const {
    return m_duration;
}

int Animation::getFrameCount() const {
    return static_cast<int>(m_keyframes.size());
}

void Animation::clear() {
    m_keyframes.clear();
    m_recording = false;
    m_playing = false;
    m_duration = 0.0f;
    m_playbackTime = 0.0f;
}

Eigen::Vector3f Animation::lerp(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float t) const {
    return a + (b - a) * t;
}

bool Animation::saveToFile(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    // write header
    int frameCount = static_cast<int>(m_keyframes.size());
    file.write(reinterpret_cast<const char*>(&frameCount), sizeof(int));
    file.write(reinterpret_cast<const char*>(&m_duration), sizeof(float));

    // write each keyframe
    for (const auto& kf : m_keyframes) {
        file.write(reinterpret_cast<const char*>(&kf.timestamp), sizeof(float));

        int vertexCount = static_cast<int>(kf.vertices.size());
        file.write(reinterpret_cast<const char*>(&vertexCount), sizeof(int));

        // write vertices
        for (const auto& v : kf.vertices) {
            float x = v.x(), y = v.y(), z = v.z();
            file.write(reinterpret_cast<const char*>(&x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&z), sizeof(float));
        }
    }

    file.close();
    std::cout << "Animation saved: to " << filename << std::endl;
    return true;
}

bool Animation::loadFromFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return false;
    }

    clear();

    int frameCount;
    file.read(reinterpret_cast<char*>(&frameCount), sizeof(int));
    file.read(reinterpret_cast<char*>(&m_duration), sizeof(float));

    for (int f = 0; f < frameCount; f++) {
        Keyframe kf;
        file.read(reinterpret_cast<char*>(&kf.timestamp), sizeof(float));

        int vertexCount;
        file.read(reinterpret_cast<char*>(&vertexCount), sizeof(int));

        kf.vertices.resize(vertexCount);
        for (int v = 0; v < vertexCount; v++) {
            float x, y, z;
            file.read(reinterpret_cast<char*>(&x), sizeof(float));
            file.read(reinterpret_cast<char*>(&y), sizeof(float));
            file.read(reinterpret_cast<char*>(&z), sizeof(float));
            kf.vertices[v] = Eigen::Vector3f(x, y, z);
        }

        m_keyframes.push_back(kf);
    }

    file.close();
    std::cout << "Animation loaded from " << filename << std::endl;
    return true;
}
