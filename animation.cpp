#include "animation.h"
#include <iostream>
#include <fstream>
#include <algorithm>

Animation::Animation()
    : m_recording(false)
    , m_playing(false)
    , m_overdubbing(false)
    , m_looping(true)
    , m_playbackTime(0.0f)
    , m_totalDuration(0.0f)
{
}

void Animation::startRecording() {
    // if already playing, start overdub instead
    if (m_playing) {
        startOverdub();
        return;
    }

    // create a new layer and start recording
    // layered for different deformations so you can do multiple at once!
    AnimationLayer layer;
    m_layers.push_back(layer);
    m_currentRecordingLayer = static_cast<int>(m_layers.size()) - 1;

    m_recording = true;
    m_playing = false;
    m_overdubbing = false;
    m_recordStartTime = std::chrono::steady_clock::now();
    std::cout << "Animation recording started (Layer " << m_currentRecordingLayer << ")" << std::endl;
}

void Animation::stopRecording() {
    if (m_recording && m_currentRecordingLayer >= 0 &&
        m_currentRecordingLayer < static_cast<int>(m_layers.size())) {
        auto& layer = m_layers[m_currentRecordingLayer];
        if (!layer.keyframes.empty()) {
            layer.duration = layer.keyframes.back().timestamp;
        }
        std::cout << "Recording stopped. " << layer.keyframes.size()
                  << " frames, " << layer.duration << " seconds" << std::endl;
    }
    m_recording = false;
    m_overdubbing = false;
    m_currentRecordingLayer = -1;

    m_totalDuration = 0.0f;
    for (const auto& layer : m_layers) {
        m_totalDuration = std::max(m_totalDuration, layer.duration);
    }
}

void Animation::startOverdub() {
    if (!m_playing) return;

    // new layer for overdub
    AnimationLayer layer;
    layer.duration = m_totalDuration;
    m_layers.push_back(layer);
    m_currentRecordingLayer = static_cast<int>(m_layers.size()) - 1;

    m_overdubbing = true;
    m_recording = true;
    m_overdubStartTime = std::chrono::steady_clock::now();
    std::cout << "Overdub recording started (Layer " << m_currentRecordingLayer
              << ") - move anchors while playback continues!" << std::endl;
}

void Animation::stopOverdub() {
    if (m_overdubbing && m_currentRecordingLayer >= 0 &&
        m_currentRecordingLayer < static_cast<int>(m_layers.size())) {
        auto& layer = m_layers[m_currentRecordingLayer];
        if (!layer.keyframes.empty()) {
            layer.duration = layer.keyframes.back().timestamp;
        }
        std::cout << "Overdub stopped. " << layer.keyframes.size()
                  << " frames recorded" << std::endl;
    }
    m_overdubbing = false;
    m_recording = false;
    m_currentRecordingLayer = -1;

    // update total duration
    m_totalDuration = 0.0f;
    for (const auto& layer : m_layers) {
        m_totalDuration = std::max(m_totalDuration, layer.duration);
    }
}

bool Animation::isRecording() const {
    return m_recording;
}

bool Animation::isPlaying() const {
    return m_playing;
}

bool Animation::isOverdubbing() const {
    return m_overdubbing;
}

void Animation::recordFrame(const std::vector<Eigen::Vector3f>& vertices) {
    if (!m_recording || m_currentRecordingLayer < 0) return;
    if (m_currentRecordingLayer >= static_cast<int>(m_layers.size())) return;

    auto& layer = m_layers[m_currentRecordingLayer];

    auto now = std::chrono::steady_clock::now();
    float elapsed;

    if (m_overdubbing) {
        // get overdub time relative to current playback position
        elapsed = m_playbackTime +
                  std::chrono::duration<float>(now - m_overdubStartTime).count();
    } else {
        elapsed = std::chrono::duration<float>(now - m_recordStartTime).count();
    }

    float minFrameInterval = 1.0f / 30.0f;
    if (!layer.keyframes.empty() && elapsed - layer.keyframes.back().timestamp < minFrameInterval) {
        return;
    }

    Keyframe frame;
    frame.timestamp = elapsed;
    frame.vertices = vertices;
    layer.keyframes.push_back(frame);
}

void Animation::startPlayback() {
    if (m_layers.empty()) {
        std::cerr << "No animation data to play" << std::endl;
        return;
    }
    m_playing = true;
    m_playbackTime = 0.0f;
    m_playbackStartTime = std::chrono::steady_clock::now();
    std::cout << "Animation playback started (" << m_layers.size()
              << " layers, " << m_totalDuration << "s)" << std::endl;
}

void Animation::stopPlayback() {
    if (m_overdubbing) {
        stopOverdub();
    }
    if (m_recording && !m_overdubbing) {
        stopRecording();
    }
    m_playing = false;
    std::cout << "Animation playback stopped" << std::endl;
}

bool Animation::isLooping() const {
    return m_looping;
}

void Animation::setLooping(bool loop) {
    m_looping = loop;
}

bool Animation::getFrameAtTime(float time, const std::vector<Eigen::Vector3f>& restPose,
                               std::vector<Eigen::Vector3f>& outVertices) const {
    if (m_layers.empty() || restPose.empty()) return false;

    // looping
    float t = time;
    if (m_looping && m_totalDuration > 0.0f) {
        t = std::fmod(t, m_totalDuration);
        if (t < 0.0f) t += m_totalDuration;
    }

    outVertices = restPose;

    // apply each layer's deformation as offset from rest pose
    for (const auto& layer : m_layers) {
        if (layer.keyframes.empty()) continue;
        if (layer.duration <= 0.0f) continue;

        std::vector<Eigen::Vector3f> layerVerts;
        if (!getLayerFrameAtTime(layer, t, layerVerts)) continue;
        if (layerVerts.size() != restPose.size()) continue;

        float w = layer.blendWeight;
        for (size_t i = 0; i < outVertices.size(); i++) {
            Eigen::Vector3f offset = layerVerts[i] - restPose[i];
            outVertices[i] += offset * w;
        }
    }

    return true;
}

bool Animation::getLayerFrameAtTime(const AnimationLayer& layer, float time,
                                    std::vector<Eigen::Vector3f>& outVertices) const {
    if (layer.keyframes.empty()) return false;

    float t = std::min(time, layer.duration);

    if (layer.keyframes.size() == 1) {
        outVertices = layer.keyframes[0].vertices;
        return true;
    }

    // interpolate between keyframes
    size_t idx = 0;
    for (size_t i = 0; i < layer.keyframes.size() - 1; i++) {
        if (layer.keyframes[i + 1].timestamp > t) {
            idx = i;
            break;
        }
        idx = std::min(i, layer.keyframes.size() - 2);
    }

    const Keyframe& kf1 = layer.keyframes[idx];
    const Keyframe& kf2 = layer.keyframes[std::min(idx + 1, layer.keyframes.size() - 1)];

    outVertices.resize(kf1.vertices.size());

    float range = kf2.timestamp - kf1.timestamp;
    float alpha = (range > 0.0001f) ? (t - kf1.timestamp) / range : 0.0f;
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    for (size_t i = 0; i < kf1.vertices.size(); i++) {
        outVertices[i] = lerp(kf1.vertices[i], kf2.vertices[i], alpha);
    }

    return true;
}

float Animation::getDuration() const {
    return m_totalDuration;
}

int Animation::getFrameCount() const {
    int total = 0;
    for (const auto& layer : m_layers) {
        total += static_cast<int>(layer.keyframes.size());
    }
    return total;
}

void Animation::clear() {
    m_layers.clear();
    m_recording = false;
    m_playing = false;
    m_overdubbing = false;
    m_totalDuration = 0.0f;
    m_playbackTime = 0.0f;
    m_currentRecordingLayer = -1;
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

    int layerCount = static_cast<int>(m_layers.size());
    file.write(reinterpret_cast<const char*>(&layerCount), sizeof(int));

    for (const auto& layer : m_layers) {
        file.write(reinterpret_cast<const char*>(&layer.duration), sizeof(float));
        file.write(reinterpret_cast<const char*>(&layer.blendWeight), sizeof(float));

        int frameCount = static_cast<int>(layer.keyframes.size());
        file.write(reinterpret_cast<const char*>(&frameCount), sizeof(int));

        for (const auto& kf : layer.keyframes) {
            file.write(reinterpret_cast<const char*>(&kf.timestamp), sizeof(float));

            int vertexCount = static_cast<int>(kf.vertices.size());
            file.write(reinterpret_cast<const char*>(&vertexCount), sizeof(int));

            for (const auto& v : kf.vertices) {
                float x = v.x(), y = v.y(), z = v.z();
                file.write(reinterpret_cast<const char*>(&x), sizeof(float));
                file.write(reinterpret_cast<const char*>(&y), sizeof(float));
                file.write(reinterpret_cast<const char*>(&z), sizeof(float));
            }
        }
    }

    file.close();
    std::cout << "Animation saved: " << layerCount << " layers to " << filename << std::endl;
    return true;
}

bool Animation::loadFromFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return false;
    }

    clear();

    int layerCount;
    file.read(reinterpret_cast<char*>(&layerCount), sizeof(int));

    for (int l = 0; l < layerCount; l++) {
        AnimationLayer layer;
        file.read(reinterpret_cast<char*>(&layer.duration), sizeof(float));
        file.read(reinterpret_cast<char*>(&layer.blendWeight), sizeof(float));

        int frameCount;
        file.read(reinterpret_cast<char*>(&frameCount), sizeof(int));

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

            layer.keyframes.push_back(kf);
        }

        m_layers.push_back(layer);
    }

    m_totalDuration = 0.0f;
    for (const auto& layer : m_layers) {
        m_totalDuration = std::max(m_totalDuration, layer.duration);
    }

    file.close();
    std::cout << "Animation loaded: " << layerCount << " layers from " << filename << std::endl;
    return true;
}
