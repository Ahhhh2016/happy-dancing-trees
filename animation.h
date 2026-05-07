#ifndef ANIMATION_H
#define ANIMATION_H

#include <vector>
#include <Eigen/Core>
#include <chrono>

// A single keyframe stores all vertex positions at a point in time
struct Keyframe {
    float timestamp;                           // Time in seconds from start
    std::vector<Eigen::Vector3f> vertices;     // All vertex positions
    std::vector<Eigen::Vector3f> anchorPositions;
};

class Animation {
public:
    Animation();

    // recording
    void startRecording();
    void stopRecording();
    bool isRecording() const;
    void recordFrame(const std::vector<Eigen::Vector3f>& vertices, const std::vector<Eigen::Vector3f>& anchorPositions);

    // playback
    void startPlayback();
    void stopPlayback();
    bool isPlaying() const;
    void setLooping(bool loop);
    bool isLooping() const;

    bool getFrameAtTime(float time, std::vector<Eigen::Vector3f>& outVertices) const;
    float getDuration() const;
    int getFrameCount() const;

    // clear data
    void clear();

    // save/load functions
    bool saveToFile(const std::string& filename) const;
    bool loadFromFile(const std::string& filename);

private:
    bool m_recording;
    bool m_playing;
    bool m_looping;
    float m_playbackTime;
    float m_duration;
    std::chrono::steady_clock::time_point m_recordStartTime;
    std::chrono::steady_clock::time_point m_playbackStartTime;
    std::vector<Keyframe> m_keyframes;

    // Interpolator
    Eigen::Vector3f lerp(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float t) const;
};

#endif // ANIMATION_H
