/*
 * SemanticWorker.h
 *
 * Background worker that consumes (nodeId, rgb) frames at a low rate,
 * runs the single-call LLM pipeline, and posts per-box LabeledBox
 * results back through a callback. The submit() interface is latest-only:
 * prior pending frames are dropped, which keeps the SLAM thread non-
 * blocking and trades freshness for throughput.
 *
 * Pipeline (one network round-trip per keyframe):
 *   /detect  — Python sidecar calls a single LLM (Gemini 3 Flash by
 *              default; Claude Sonnet 4.6 alternative) which returns
 *              boxes + labels + confidences + visual_angle in one shot.
 *              SAM refines the boxes into a union mask before responding.
 *
 * Because each /detect call goes through the LLM (~1-3 s), we rely on
 * the latest-only submit() semantics for natural rate limiting: while a
 * call is in flight, intermediate keyframes simply overwrite slot_ and
 * are dropped. The most recent unprocessed frame is the next one taken.
 */

#ifndef SEMANTICWORKER_H_
#define SEMANTICWORKER_H_

#include "SemanticLabeledBox.h"
#include "rtabmap/utilite/ULogger.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// cpp-httplib pulls in winsock2 on Windows. GridPublisher.h already brings
// it in before this header, but guard WIN32_LEAN_AND_MEAN defensively in
// case SemanticWorker.h is ever included from a different translation unit.
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#endif
#include <httplib.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class SemanticWorker
{
public:
    enum class Mode { Mock, Http };

    using LabeledBox     = ::semantic::LabeledBox;
    using ResultCallback = std::function<void(int nodeId,
                                               const std::vector<LabeledBox> & boxes)>;

    struct Frame {
        int     nodeId = -1;
        cv::Mat rgb;        // CV_8UC3
    };

    SemanticWorker(Mode mode,
                   std::string httpUrl,
                   ResultCallback cb)
        : mode_(mode),
          url_(std::move(httpUrl)),
          callback_(std::move(cb)),
          running_(true),
          hasFrame_(false)
    {
        thread_ = std::thread(&SemanticWorker::loop, this);
        UINFO("SemanticWorker: started (mode=%s, url='%s')",
              mode_ == Mode::Mock ? "mock" : "http",
              url_.c_str());
    }

    ~SemanticWorker()
    {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            running_ = false;
        }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    // Latest-only enqueue. If a frame is already pending, it is overwritten.
    void submit(Frame frame)
    {
        if (frame.rgb.empty() || frame.nodeId < 0) return;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            slot_     = std::move(frame);
            hasFrame_ = true;
        }
        cv_.notify_one();
    }

private:
    void loop()
    {
        while (true)
        {
            Frame frame;
            bool  gotFrame   = false;
            bool  shouldExit = false;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait(lk, [this] { return hasFrame_ || !running_; });
                if (!running_ && !hasFrame_) shouldExit = true;
                if (hasFrame_)
                {
                    frame     = std::move(slot_);
                    slot_     = Frame{};
                    hasFrame_ = false;
                    gotFrame  = true;
                }
            }

            if (shouldExit) return;
            if (!gotFrame) continue;

            if (mode_ == Mode::Mock)
            {
                handleMock(frame);
                continue;
            }

            std::vector<LabeledBox> labeled;
            if (httpDetect(frame, labeled))
            {
                if (callback_) callback_(frame.nodeId, labeled);
            }
            else
            {
                // Dispatch empty so MaskStore entries don't sit pending forever.
                if (callback_) callback_(frame.nodeId, {});
            }
        }
    }

    // Mock: paints a single full-image box with no label so the downstream
    // mask path still has something to ray-cast for development.
    void handleMock(const Frame & frame)
    {
        if (!callback_) return;
        LabeledBox lb;
        lb.x1 = frame.rgb.cols * 0.25f;
        lb.y1 = frame.rgb.rows * 0.65f;
        lb.x2 = frame.rgb.cols * 0.75f;
        lb.y2 = frame.rgb.rows * 0.85f;
        lb.label      = "pillar";
        lb.confidence = 1.0f;
        callback_(frame.nodeId, {lb});
    }

    // ---- /detect ---------------------------------------------------------

    // Calls /detect once. The Python sidecar runs the LLM (and SAM mask
    // refinement) and returns boxes + labels + confidences + visual_angle
    // in a single response. Returns false on transport / parse error;
    // returns true with an empty `out` when the LLM saw nothing of interest.
    bool httpDetect(const Frame & frame, std::vector<LabeledBox> & out)
    {
        if (!ensureClient()) return false;

        std::vector<uchar> jpeg;
        const std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 85};
        if (!cv::imencode(".jpg", frame.rgb, jpeg, jpegParams))
        {
            UWARN("SemanticWorker: imencode failed");
            return false;
        }

        const std::string b64 = base64Encode(jpeg.data(), jpeg.size());
        nlohmann::json body = {
            {"image_jpeg_b64", b64},
            {"request_id",     frame.nodeId},
        };

        auto res = client_->Post(detectPath_.c_str(),
                                  body.dump(), "application/json");
        if (!res)
        {
            UWARN("SemanticWorker: /detect POST failed (host=%s:%d)",
                  host_.c_str(), port_);
            return false;
        }
        if (res->status != 200)
        {
            UWARN("SemanticWorker: /detect status=%d body=%.200s",
                  res->status, res->body.c_str());
            return false;
        }

        try
        {
            auto j = nlohmann::json::parse(res->body);
            if (j.contains("detections") && j["detections"].is_array())
            {
                for (const auto & d : j["detections"])
                {
                    if (!d.contains("box")) continue;
                    const auto & b = d["box"];
                    if (!b.is_array() || b.size() < 4) continue;
                    LabeledBox lb;
                    lb.x1 = b[0].get<float>();
                    lb.y1 = b[1].get<float>();
                    lb.x2 = b[2].get<float>();
                    lb.y2 = b[3].get<float>();
                    lb.label       = d.value("label", "");
                    lb.confidence  = d.value("confidence", 0.0f);
                    if (d.contains("visual_angle") && !d["visual_angle"].is_null())
                        lb.visualAngle = d["visual_angle"].get<float>();
                    out.push_back(std::move(lb));
                }
            }
            const int infMs = j.value("inference_ms", -1);
            UDEBUG("SemanticWorker: /detect node=%d boxes=%zu inference_ms=%d",
                   frame.nodeId, out.size(), infMs);
        }
        catch (const std::exception & e)
        {
            UWARN("SemanticWorker: /detect JSON parse failed: %s", e.what());
            return false;
        }
        return true;
    }

    // ---- HTTP client setup ----------------------------------------------

    bool ensureClient()
    {
        if (client_) return true;

        // url_ may include a path (legacy: ".../detect"). We strip it and
        // hard-wire the endpoint path instead.
        if (!parseUrl(url_, host_, port_))
        {
            UERROR("SemanticWorker: bad RTABMAP_SEMANTIC_URL '%s' "
                   "(expected http://host:port[/...])",
                   url_.c_str());
            return false;
        }
        client_ = std::make_unique<httplib::Client>(host_, port_);
        client_->set_keep_alive(true);
        client_->set_connection_timeout(5, 0);
        // /detect now goes through the LLM + SAM, so it can take several
        // seconds. Keep this generous.
        client_->set_read_timeout(30, 0);
        client_->set_write_timeout(5, 0);
        UINFO("SemanticWorker: HTTP client ready -> %s:%d (path %s)",
              host_.c_str(), port_, detectPath_.c_str());
        return true;
    }

    // Parses http://host[:port][/anything] and discards any path. Defaults
    // port to 80 if absent.
    static bool parseUrl(const std::string & url,
                         std::string & host, int & port)
    {
        const std::string scheme = "http://";
        if (url.compare(0, scheme.size(), scheme) != 0) return false;
        size_t i = scheme.size();
        size_t pathStart = url.find('/', i);
        std::string authority = (pathStart == std::string::npos)
                              ? url.substr(i) : url.substr(i, pathStart - i);

        size_t colon = authority.find(':');
        if (colon == std::string::npos)
        {
            host = authority;
            port = 80;
        }
        else
        {
            host = authority.substr(0, colon);
            try { port = std::stoi(authority.substr(colon + 1)); }
            catch (...) { return false; }
        }
        return !host.empty();
    }

    // ---- base64 ---------------------------------------------------------

    static std::string base64Encode(const uint8_t * data, size_t len)
    {
        static const char tbl[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out;
        out.reserve(((len + 2) / 3) * 4);
        size_t i = 0;
        while (i + 3 <= len)
        {
            const uint32_t v = (uint32_t(data[i]) << 16)
                             | (uint32_t(data[i + 1]) << 8)
                             |  uint32_t(data[i + 2]);
            out += tbl[(v >> 18) & 0x3F];
            out += tbl[(v >> 12) & 0x3F];
            out += tbl[(v >>  6) & 0x3F];
            out += tbl[ v        & 0x3F];
            i += 3;
        }
        if (i < len)
        {
            uint32_t v = uint32_t(data[i]) << 16;
            if (i + 1 < len) v |= uint32_t(data[i + 1]) << 8;
            out += tbl[(v >> 18) & 0x3F];
            out += tbl[(v >> 12) & 0x3F];
            out += (i + 1 < len) ? tbl[(v >> 6) & 0x3F] : '=';
            out += '=';
        }
        return out;
    }

    Mode             mode_;
    std::string      url_;
    ResultCallback   callback_;

    std::thread             thread_;
    std::mutex              mtx_;
    std::condition_variable cv_;
    bool                    running_;
    Frame                   slot_;
    bool                    hasFrame_;

    // HTTP-mode state. Owned and used exclusively by the worker thread.
    std::unique_ptr<httplib::Client> client_;
    std::string                      host_;
    int                              port_ = 0;
    std::string                      detectPath_ = "/detect";
};

#endif /* SEMANTICWORKER_H_ */
