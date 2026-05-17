/*
 * SemanticWorker.h
 *
 * Background worker that consumes (nodeId, rgb) frames, runs the YOLO
 * pipeline, and posts per-box LabeledBox results back through a callback.
 * The submit() interface is latest-only: prior pending frames are dropped,
 * which keeps the SLAM thread non-blocking and trades freshness for
 * throughput.
 *
 * Pipeline (one network round-trip per keyframe):
 *   /detect  — Python sidecar runs YOLO11-seg in a single forward pass
 *              and returns boxes + labels + confidences + union mask
 *              (~10-50 ms on an RTX 4090 at 720p).
 *
 * Latest-only submit() still provides natural rate limiting: while a call
 * is in flight, intermediate keyframes overwrite slot_ and are dropped.
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
    // `mask` is CV_8UC1 (0 = background, >0 = wall pixel). May be empty —
    // callers fall back to box raster when it is.
    using ResultCallback = std::function<void(int nodeId,
                                               const std::vector<LabeledBox> & boxes,
                                               const cv::Mat & mask)>;

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
            cv::Mat mask;
            if (httpDetect(frame, labeled, mask))
            {
                if (callback_) callback_(frame.nodeId, labeled, mask);
            }
            else
            {
                // Dispatch empty so MaskStore entries don't sit pending forever.
                if (callback_) callback_(frame.nodeId, {}, cv::Mat());
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
        lb.label      = "centerLine";
        lb.confidence = 1.0f;
        callback_(frame.nodeId, {lb}, cv::Mat());
    }

    // ---- /detect ---------------------------------------------------------

    // Calls /detect once. The Python sidecar runs YOLO11-seg and returns
    // boxes + labels + confidences + union mask in a single response.
    // Returns false on transport / parse error; returns true with an empty
    // `out` when YOLO found nothing of interest. `outMask` is filled with the
    // decoded binary union mask (CV_8UC1, 0/255) when present, or left empty.
    bool httpDetect(const Frame & frame,
                    std::vector<LabeledBox> & out,
                    cv::Mat & outMask)
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
            if (j.contains("mask_png_b64") && j["mask_png_b64"].is_string())
            {
                const std::string & b64 = j["mask_png_b64"].get_ref<const std::string &>();
                if (!b64.empty())
                {
                    std::vector<uint8_t> png = base64Decode(b64);
                    if (!png.empty())
                    {
                        outMask = cv::imdecode(cv::Mat(png), cv::IMREAD_GRAYSCALE);
                        if (outMask.empty())
                            UWARN("SemanticWorker: mask_png_b64 imdecode failed");
                    }
                }
            }
            const int infMs = j.value("inference_ms", -1);
            std::string lbls;
            for (size_t i = 0; i < out.size(); ++i)
                lbls += (i ? "," : "") + out[i].label;
            UINFO("SemanticWorker: /detect node=%d det=%zu [%s] mask=%dx%d infer=%dms",
                  frame.nodeId, out.size(), lbls.c_str(),
                  outMask.cols, outMask.rows, infMs);
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
        // YOLO inference is ~10-50 ms; 5 s is a generous cushion.
        client_->set_read_timeout(5, 0);
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

    static std::vector<uint8_t> base64Decode(const std::string & in)
    {
        static int8_t T[256];
        static bool inited = false;
        if (!inited)
        {
            for (int i = 0; i < 256; i++) T[i] = -1;
            static const char tbl[] =
                "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            for (int i = 0; i < 64; i++) T[(uint8_t)tbl[i]] = static_cast<int8_t>(i);
            inited = true;
        }
        std::vector<uint8_t> out;
        out.reserve((in.size() * 3) / 4);
        uint32_t val = 0;
        int bits = 0;
        for (char c : in)
        {
            if (c == '=') break;
            const int8_t d = T[(uint8_t)c];
            if (d < 0) continue;   // skip whitespace / invalid
            val = (val << 6) | static_cast<uint32_t>(d);
            bits += 6;
            if (bits >= 8)
            {
                bits -= 8;
                out.push_back(static_cast<uint8_t>((val >> bits) & 0xFF));
            }
        }
        return out;
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
    std::string                      detectPath_ = "/detect";
};

#endif /* SEMANTICWORKER_H_ */
