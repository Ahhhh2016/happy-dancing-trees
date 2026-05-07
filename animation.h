#ifndef ANIMATION_H
#define ANIMATION_H

#include <vector>
#include <Eigen/Core>
#include <chrono>
#include <string>

struct Keyframe {
    float timestamp;
    std::vector<Eigen::Vector3f> vertices;
};

// layer stores recorded or overdubbed animation
struct AnimationLayer {
    std::vector<Keyframe> keyframes;
    float duration = 0.0f;
    float blendWeight = 1.0f;
};

class Animation {
public:
    Animation();

    // Recording
    void startRecording();
    void stopRecording();
    bool isRecording() const;
    void recordFrame(const std::vector<Eigen::Vector3f>& vertices);

    // Playback
    void startPlayback();
    void stopPlayback();
    bool isPlaying() const;
    void setLooping(bool loop);
    bool isLooping() const;

    // Record on top of prior recording (overdub)
    void startOverdub();
    void stopOverdub();
    bool isOverdubbing() const;

    // Frame setup
    bool getFrameAtTime(float time, const std::vector<Eigen::Vector3f>& restPose,
                        std::vector<Eigen::Vector3f>& outVertices) const;
    float getDuration() const;
    int getFrameCount() const;
    int getLayerCount() const { return static_cast<int>(m_layers.size()); }

    void clear();
    bool saveToFile(const std::string& filename) const;
    bool loadFromFile(const std::string& filename); // iffy right now

private:
    bool m_recording;
    bool m_playing;
    bool m_overdubbing;
    bool m_looping;
    float m_playbackTime;
    float m_totalDuration;
    std::chrono::steady_clock::time_point m_recordStartTime;
    std::chrono::steady_clock::time_point m_playbackStartTime;
    std::chrono::steady_clock::time_point m_overdubStartTime;

    std::vector<AnimationLayer> m_layers;
    int m_currentRecordingLayer = -1;

    Eigen::Vector3f lerp(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float t) const;
    bool getLayerFrameAtTime(const AnimationLayer& layer, float time,
                             std::vector<Eigen::Vector3f>& outVertices) const;
};

#endif // ANIMATION_H
