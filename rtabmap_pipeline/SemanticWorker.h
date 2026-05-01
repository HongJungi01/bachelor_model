/*
 * SemanticWorker.h
 *
 * Background worker that consumes (nodeId, rgb) frames at a low rate (~1 Hz),
 * runs an inference (mock or HTTP), and posts the resulting mask back through
 * a callback. The submit() interface is latest-only: prior pending frames are
 * dropped, which keeps the SLAM thread non-blocking and trades freshness for
 * throughput.
 *
 * Phase-1: MOCK mode only. HTTP backend is wired in Phase 5 (cpp-httplib).
 */

#ifndef SEMANTICWORKER_H_
#define SEMANTICWORKER_H_

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

    using ResultCallback = std::function<void(int nodeId, const cv::Mat & mask)>;

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
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait(lk, [this] { return hasFrame_ || !running_; });
                if (!running_) return;
                frame     = std::move(slot_);
                slot_     = Frame{};
                hasFrame_ = false;
            }

            cv::Mat mask;
            try
            {
                switch (mode_)
                {
                case Mode::Mock: mask = generateMockMask(frame.rgb); break;
                case Mode::Http: mask = httpDetect(frame.rgb);       break;
                }
            }
            catch (const std::exception & e)
            {
                UWARN("SemanticWorker: inference threw: %s", e.what());
            }

            if (!mask.empty() && callback_)
                callback_(frame.nodeId, mask);
        }
    }

    // Mock: paints a horizontal band in the lower-center of the image, where the
    // floor naturally appears for a forward-tilted camera. This lets us validate
    // the back-projection + raster path without any model dependency.
    static cv::Mat generateMockMask(const cv::Mat & rgb)
    {
        cv::Mat mask = cv::Mat::zeros(rgb.size(), CV_8UC1);
        const int w = rgb.cols, h = rgb.rows;
        const int x1 = w * 1 / 4;
        const int x2 = w * 3 / 4;
        const int y1 = h * 65 / 100;
        const int y2 = h * 85 / 100;
        cv::rectangle(mask,
                      cv::Point(x1, y1), cv::Point(x2, y2),
                      cv::Scalar(255), cv::FILLED);
        return mask;
    }

    // HTTP path. Lazy-initialises the client on first use, then reuses it.
    // Returns an empty Mat on any failure; the worker thread treats that
    // as "no detection" and silently drops the frame.
    cv::Mat httpDetect(const cv::Mat & rgb)
    {
        if (!ensureClient()) return cv::Mat();

        // 1. JPEG-encode the original RGB. Python side resizes for inference
        //    and converts box coords back to this resolution.
        std::vector<uchar> jpeg;
        const std::vector<int> jpegParams = {cv::IMWRITE_JPEG_QUALITY, 85};
        if (!cv::imencode(".jpg", rgb, jpeg, jpegParams))
        {
            UWARN("SemanticWorker: imencode failed");
            return cv::Mat();
        }

        // 2. Base64.
        const std::string b64 = base64Encode(jpeg.data(), jpeg.size());

        // 3. Build JSON request.
        nlohmann::json body = {
            {"image_jpeg_b64",  b64},
            {"prompt",          "parking space line . lane marking"},
            {"box_threshold",   0.3},
            {"text_threshold",  0.25},
            {"request_id",      reqIdCounter_.fetch_add(1)},
        };
        const std::string payload = body.dump();

        // 4. POST.
        auto res = client_->Post(path_.c_str(), payload, "application/json");
        if (!res)
        {
            UWARN("SemanticWorker: POST failed (no response, host=%s:%d)",
                  host_.c_str(), port_);
            return cv::Mat();
        }
        if (res->status != 200)
        {
            UWARN("SemanticWorker: POST status=%d body=%.200s",
                  res->status, res->body.c_str());
            return cv::Mat();
        }

        // 5. Parse JSON, raster boxes into a binary mask at rgb.size().
        cv::Mat mask = cv::Mat::zeros(rgb.size(), CV_8UC1);
        try
        {
            auto j = nlohmann::json::parse(res->body);
            const auto & dets = j.at("detections");
            int nBoxes = 0;
            for (const auto & d : dets)
            {
                const auto & bx = d.at("box");
                if (bx.size() != 4) continue;
                const float x1 = bx[0].get<float>();
                const float y1 = bx[1].get<float>();
                const float x2 = bx[2].get<float>();
                const float y2 = bx[3].get<float>();
                if (x2 <= x1 || y2 <= y1) continue;
                cv::rectangle(mask,
                              cv::Point(static_cast<int>(x1), static_cast<int>(y1)),
                              cv::Point(static_cast<int>(x2), static_cast<int>(y2)),
                              cv::Scalar(255), cv::FILLED);
                ++nBoxes;
            }
            const int infMs = j.value("inference_ms", -1);
            UDEBUG("SemanticWorker: nodeId boxes=%d inference_ms=%d", nBoxes, infMs);
        }
        catch (const std::exception & e)
        {
            UWARN("SemanticWorker: JSON parse failed: %s", e.what());
            return cv::Mat();
        }

        return mask;
    }

    bool ensureClient()
    {
        if (client_) return true;

        if (!parseUrl(url_, host_, port_, path_))
        {
            UERROR("SemanticWorker: bad --semantic-url '%s' (expected http://host:port/path)",
                   url_.c_str());
            return false;
        }
        client_ = std::make_unique<httplib::Client>(host_, port_);
        client_->set_keep_alive(true);
        client_->set_connection_timeout(5, 0);
        client_->set_read_timeout(10, 0);
        client_->set_write_timeout(5, 0);
        UINFO("SemanticWorker: HTTP client ready -> %s:%d%s",
              host_.c_str(), port_, path_.c_str());
        return true;
    }

    // Minimal http://host[:port]/path parser. Defaults port to 80 if absent.
    static bool parseUrl(const std::string & url,
                         std::string & host, int & port, std::string & path)
    {
        const std::string scheme = "http://";
        if (url.compare(0, scheme.size(), scheme) != 0) return false;
        size_t i = scheme.size();
        size_t pathStart = url.find('/', i);
        std::string authority = (pathStart == std::string::npos)
                              ? url.substr(i) : url.substr(i, pathStart - i);
        path = (pathStart == std::string::npos) ? "/" : url.substr(pathStart);

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
    std::string                      path_;
    std::atomic<int>                 reqIdCounter_{0};
};

#endif /* SEMANTICWORKER_H_ */
