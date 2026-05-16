#include <control_input_msgs/msg/inputs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>


/*
Usage:
    ros2 run test traject_build --ros-args \
  -p motion_sequence:="w:3.0;a:1.5;space:0.5" \
  -p cmd2_to_cmd3_delay:=2.0 \
  -p cmd3_to_motion_delay:=2.0 \
  -p post_motion_delay:=1.0

*/


namespace
{
constexpr double kDefaultSensitivity = 0.05;
constexpr double kDefaultPublishRateHz = 50.0;
constexpr double kCommandPulseDurationSec = 0.05;

struct MotionSegment
{
    char key = ' ';
    double duration_sec = 0.0;
};

std::string trim(const std::string& text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string& text, const char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream stream(text);
    std::string token;
    while (std::getline(stream, token, delimiter))
    {
        if (token.find_first_not_of(" \t\r\n") != std::string::npos)
        {
            tokens.push_back(token);
        }
    }
    return tokens;
}

bool isStopKeyText(const std::string& key_text)
{
    return key_text.empty() || key_text == "space" || key_text == "stop" || key_text == "halt" ||
           key_text == "_";
}

bool parseMotionSegment(const std::string& token, MotionSegment& segment, std::string& error)
{
    const auto colon = token.find(':');
    if (colon == std::string::npos || colon + 1 >= token.size())
    {
        error = "segment must look like <key>:<seconds>, got \"" + token + "\"";
        return false;
    }

    const std::string key_text = trim(token.substr(0, colon));
    if (isStopKeyText(key_text))
    {
        segment.key = ' ';
    }
    else if (key_text.size() == 1)
    {
        segment.key = static_cast<char>(std::tolower(key_text.front()));
    }
    else
    {
        error = "motion key must be one of w/s/a/d/i/j/k/l, space/stop/halt/_, or a single space before ':', got \"" +
                key_text + "\"";
        return false;
    }
    try
    {
        segment.duration_sec = std::stod(trim(token.substr(colon + 1)));
    }
    catch (const std::exception&)
    {
        error = "invalid duration in \"" + token + "\"";
        return false;
    }

    if (segment.duration_sec <= 0.0)
    {
        error = "duration must be positive in \"" + token + "\"";
        return false;
    }

    return true;
}

std::vector<MotionSegment> parseMotionSequence(const std::string& raw_sequence, std::string& error)
{
    std::vector<MotionSegment> segments;
    for (const auto& token : split(raw_sequence, ';'))
    {
        MotionSegment segment;
        if (!parseMotionSegment(token, segment, error))
        {
            return {};
        }
        segments.push_back(segment);
    }
    return segments;
}

void applyMotionKey(char key, float sensitivity, control_input_msgs::msg::Inputs& inputs)
{
    switch (key)
    {
    case 'w':
        inputs.ly = std::min(inputs.ly + sensitivity, 1.0F);
        break;
    case 's':
        inputs.ly = std::max(inputs.ly - sensitivity, -1.0F);
        break;
    case 'd':
        inputs.lx = std::min(inputs.lx + sensitivity, 1.0F);
        break;
    case 'a':
        inputs.lx = std::max(inputs.lx - sensitivity, -1.0F);
        break;
    case 'i':
        inputs.ry = std::min(inputs.ry + sensitivity, 1.0F);
        break;
    case 'k':
        inputs.ry = std::max(inputs.ry - sensitivity, -1.0F);
        break;
    case 'l':
        inputs.rx = std::min(inputs.rx + sensitivity, 1.0F);
        break;
    case 'j':
        inputs.rx = std::max(inputs.rx - sensitivity, -1.0F);
        break;
    case ' ':
        inputs.lx = 0.0F;
        inputs.ly = 0.0F;
        inputs.rx = 0.0F;
        inputs.ry = 0.0F;
        break;
    default:
        break;
    }
}

bool isSupportedMotionKey(char key)
{
    switch (key)
    {
    case 'w':
    case 's':
    case 'a':
    case 'd':
    case 'i':
    case 'k':
    case 'j':
    case 'l':
    case ' ':
        return true;
    default:
        return false;
    }
}
}  // namespace

class TrajectBuildNode final : public rclcpp::Node
{
public:
    TrajectBuildNode()
        : Node("traject_build")
    {
        topic_ = declare_parameter<std::string>("control_input_topic", "/control_input");
        cmd2_to_cmd3_delay_sec_ = declare_parameter<double>("cmd2_to_cmd3_delay", 2.0);
        cmd3_to_motion_delay_sec_ = declare_parameter<double>("cmd3_to_motion_delay", 1.0);
        command_pulse_duration_sec_ = declare_parameter<double>("command_pulse_duration", kCommandPulseDurationSec);
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", kDefaultPublishRateHz);
        sensitivity_ = static_cast<float>(declare_parameter<double>("sensitivity", kDefaultSensitivity));
        startup_delay_sec_ = declare_parameter<double>("startup_delay", 1.0);
        post_motion_delay_sec_ = declare_parameter<double>("post_motion_delay", 1.0);

        const std::string motion_sequence_raw =
            declare_parameter<std::string>("motion_sequence", "w:2.0");

        std::string parse_error;
        motion_segments_ = parseMotionSequence(motion_sequence_raw, parse_error);
        if (motion_segments_.empty())
        {
            throw std::runtime_error("motion_sequence is empty or invalid: " + parse_error);
        }

        for (const auto& segment : motion_segments_)
        {
            if (!isSupportedMotionKey(segment.key))
            {
                throw std::runtime_error(
                    std::string("unsupported motion key '") + segment.key +
                    "'. Use w/s/a/d/i/j/k/l or space.");
            }
        }

        if (publish_rate_hz_ <= 0.0)
        {
            throw std::runtime_error("publish_rate_hz must be positive");
        }

        publisher_ = create_publisher<control_input_msgs::msg::Inputs>(topic_, 10);
        inputs_ = control_input_msgs::msg::Inputs();

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TrajectBuildNode::onTimer, this));
        phase_start_sec_ = nowSec();

        RCLCPP_INFO(get_logger(), "Publishing scripted control_input to %s", topic_.c_str());
        RCLCPP_INFO(get_logger(),
                    "Flow: cmd2 -> wait %.2fs -> cmd3 -> wait %.2fs -> motion -> wait %.2fs -> exit",
                    cmd2_to_cmd3_delay_sec_, cmd3_to_motion_delay_sec_, post_motion_delay_sec_);
        RCLCPP_INFO(get_logger(), "Motion sequence: %s", motion_sequence_raw.c_str());
    }

private:
    enum class Phase
    {
        StartupDelay,
        CommandPulse,
        WaitBeforeCommand3,
        WaitBeforeMotion,
        Motion,
        WaitBeforeExit,
    };

    void onTimer()
    {
        const auto now = nowSec();

        switch (phase_)
        {
        case Phase::StartupDelay:
            if (elapsedSince(phase_start_sec_, now) >= startup_delay_sec_)
            {
                beginCommandPulse(2, Phase::WaitBeforeCommand3);
            }
            break;

        case Phase::CommandPulse:
            publishCurrent();
            if (elapsedSince(phase_start_sec_, now) >= command_pulse_duration_sec_)
            {
                clearCommand();
                publishCurrent();
                phase_ = next_phase_after_pulse_;
                phase_start_sec_ = now;
            }
            break;

        case Phase::WaitBeforeCommand3:
            publishCurrent();
            if (elapsedSince(phase_start_sec_, now) >= cmd2_to_cmd3_delay_sec_)
            {
                beginCommandPulse(3, Phase::WaitBeforeMotion);
            }
            break;

        case Phase::WaitBeforeMotion:
            publishCurrent();
            if (elapsedSince(phase_start_sec_, now) >= cmd3_to_motion_delay_sec_)
            {
                phase_ = Phase::Motion;
                phase_start_sec_ = now;
                motion_index_ = 0;
                beginMotionSegment(now);
            }
            break;

        case Phase::Motion:
            applyMotionKey(motion_segments_[motion_index_].key, sensitivity_, inputs_);
            publishCurrent();

            if (elapsedSince(phase_start_sec_, now) >= motion_segments_[motion_index_].duration_sec)
            {
                if (motion_segments_[motion_index_].key != ' ')
                {
                    clearAxes();
                    publishCurrent();
                }

                ++motion_index_;
                if (motion_index_ >= motion_segments_.size())
                {
                    clearAxes();
                    clearCommand();
                    publishCurrent();
                    beginPostMotionDelay(now);
                    return;
                }

                phase_start_sec_ = now;
                beginMotionSegment(now);
            }
            break;

        case Phase::WaitBeforeExit:
            publishCurrent();
            if (elapsedSince(phase_start_sec_, now) >= post_motion_delay_sec_)
            {
                finishScript();
            }
            break;
        }
    }

    void beginCommandPulse(int command, Phase next_phase)
    {
        inputs_.command = command;
        clearAxes();
        phase_ = Phase::CommandPulse;
        next_phase_after_pulse_ = next_phase;
        phase_start_sec_ = nowSec();
        publishCurrent();
        RCLCPP_INFO(get_logger(), "Publishing command %d", command);
    }

    void beginPostMotionDelay(double now)
    {
        RCLCPP_INFO(get_logger(), "Motion finished, waiting %.2fs before exit",
                    post_motion_delay_sec_);
        phase_ = Phase::WaitBeforeExit;
        phase_start_sec_ = now;
    }

    void finishScript()
    {
        clearAxes();
        clearCommand();
        publishCurrent();
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "Motion script finished, exiting traject_build.");
        rclcpp::shutdown();
    }

    void beginMotionSegment(double now)
    {
        const auto& segment = motion_segments_[motion_index_];
        RCLCPP_INFO(get_logger(), "Motion segment %zu: '%c' for %.2fs",
                    motion_index_ + 1, segment.key, segment.duration_sec);
        if (segment.key == ' ')
        {
            clearAxes();
            clearCommand();
        }
        phase_start_sec_ = now;
    }

    void publishCurrent()
    {
        publisher_->publish(inputs_);
    }

    void clearAxes()
    {
        inputs_.lx = 0.0F;
        inputs_.ly = 0.0F;
        inputs_.rx = 0.0F;
        inputs_.ry = 0.0F;
    }

    void clearCommand()
    {
        inputs_.command = 0;
    }

    static double elapsedSince(double start_sec, double now_sec)
    {
        return std::max(0.0, now_sec - start_sec);
    }

    double nowSec() const
    {
        return this->now().seconds();
    }

    std::string topic_;
    double cmd2_to_cmd3_delay_sec_ = 2.0;
    double cmd3_to_motion_delay_sec_ = 1.0;
    double command_pulse_duration_sec_ = kCommandPulseDurationSec;
    double publish_rate_hz_ = kDefaultPublishRateHz;
    double startup_delay_sec_ = 1.0;
    double post_motion_delay_sec_ = 1.0;
    float sensitivity_ = static_cast<float>(kDefaultSensitivity);

    Phase phase_ = Phase::StartupDelay;
    Phase next_phase_after_pulse_ = Phase::WaitBeforeMotion;
    double phase_start_sec_ = 0.0;
    size_t motion_index_ = 0;

    std::vector<MotionSegment> motion_segments_;
    control_input_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<TrajectBuildNode>());
    }
    catch (const std::exception& exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("traject_build"), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
