/*
 * SemanticWorker.h
 *
 * Background worker that consumes (nodeId, rgb) frames at a low rate (~1 Hz),
 * runs the two-stage semantic pipeline, and posts per-box labels back through
 * a callback. The submit() interface is latest-only: prior pending frames are
 * dropped, which keeps the SLAM thread non-blocking and trades freshness for
 * throughput.
 *
 * Two-stage pipeline:
 *   1. /detect          — Florence-2 + SAM, returns bounding boxes only.
 *                         Labels from Florence-2 are intentionally ignored.
 *   2. /classify_batch  — Gemini classifier. Up to 5 frames are buffered
 *                         and submitted in a single call to amortise
 *                         API latency. Returns a label + confidence +
 *                         visual_angle per box id.
 *
 * The worker accumulates frames with non-empty box detections into a batch.
 * The batch is flushed when it hits BATCH_SIZE or after BATCH_TIMEOUT_MS of
 * idle. Frames with zero boxes short-circuit and dispatch an empty result
 * immediately.
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

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
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
    static constexpr size_t kBatchSize       = 5;
    static constexpr int    kBatchTimeoutMs  = 1500;

    // A frame that has cleared /detect and is waiting for /classify_batch.
    // We keep the JPEG bytes around to avoid re-encoding when classifying.
    struct PendingFrame {
        int                  nodeId = -1;
        std::vector<uint8_t> jpeg;        // raw JPEG (NOT base64)
        std::vector<std::array<float,4>> boxes;  // x1,y1,x2,y2 in image pixels
    };

    void loop()
    {
        std::vector<PendingFrame> batch;
        batch.reserve(kBatchSize);
        auto lastFrameTime = std::chrono::steady_clock::now();

        while (true)
        {
            Frame frame;
            bool  gotFrame  = false;
            bool  shouldExit = false;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                if (batch.empty())
                {
                    cv_.wait(lk, [this] { return hasFrame_ || !running_; });
                }
                else
                {
                    cv_.wait_for(lk,
                                 std::chrono::milliseconds(kBatchTimeoutMs),
                                 [this] { return hasFrame_ || !running_; });
                }
                if (!running_ && !hasFrame_) shouldExit = true;
                if (hasFrame_)
                {
                    frame     = std::move(slot_);
                    slot_     = Frame{};
                    hasFrame_ = false;
                    gotFrame  = true;
                }
            }

            if (shouldExit)
            {
                if (!batch.empty()) flushBatch(batch);
                return;
            }

            if (gotFrame)
            {
                if (mode_ == Mode::Mock)
                {
                    handleMock(frame);
                }
                else
                {
                    PendingFrame pf;
                    if (httpDetect(frame, pf))
                    {
                        if (pf.boxes.empty())
                        {
                            // No detections — dispatch empty immediately.
                            if (callback_) callback_(frame.nodeId, {});
                        }
                        else
                        {
                            batch.push_back(std::move(pf));
                            lastFrameTime = std::chrono::steady_clock::now();
                        }
                    }
                }
            }

            if (batch.size() >= kBatchSize)
            {
                flushBatch(batch);
            }
            else if (!batch.empty())
            {
                auto idleMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - lastFrameTime).count();
                if (idleMs >= kBatchTimeoutMs)
                {
                    flushBatch(batch);
                }
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

    // Calls /detect, fills out.boxes (image-pixel coords) and out.jpeg.
    // Florence-2 labels are intentionally NOT propagated — L3 (Gemini) is
    // the only source of truth for labels.
    bool httpDetect(const Frame & frame, PendingFrame & out)
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

        // Server uses its default prompt; we deliberately don't send one.
        nlohmann::json body = {
            {"image_jpeg_b64", b64},
            {"request_id",     reqIdCounter_.fetch_add(1)},
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
                    out.boxes.push_back({
                        b[0].get<float>(), b[1].get<float>(),
                        b[2].get<float>(), b[3].get<float>(),
                    });
                }
            }
            const int infMs = j.value("inference_ms", -1);
            UDEBUG("SemanticWorker: /detect node=%d boxes=%zu inference_ms=%d",
                   frame.nodeId, out.boxes.size(), infMs);
        }
        catch (const std::exception & e)
        {
            UWARN("SemanticWorker: /detect JSON parse failed: %s", e.what());
            return false;
        }

        out.nodeId = frame.nodeId;
        out.jpeg.assign(jpeg.begin(), jpeg.end());
        return true;
    }

    // ---- /classify_batch -------------------------------------------------

    void flushBatch(std::vector<PendingFrame> & batch)
    {
        if (batch.empty()) return;

        bool ok = httpClassifyBatch(batch);
        if (!ok)
        {
            // On error, dispatch empty results so MaskStore entries don't
            // sit in pending forever.
            for (const auto & pf : batch)
                if (callback_) callback_(pf.nodeId, {});
        }
        batch.clear();
    }

    // Builds the /classify_batch request, parses the response, and dispatches
    // one callback per frame. Returns false on error (caller drops the batch).
    bool httpClassifyBatch(const std::vector<PendingFrame> & batch)
    {
        if (!ensureClient()) return false;

        nlohmann::json frames = nlohmann::json::array();
        for (const auto & pf : batch)
        {
            nlohmann::json boxes = nlohmann::json::array();
            for (size_t i = 0; i < pf.boxes.size(); ++i)
            {
                const auto & b = pf.boxes[i];
                boxes.push_back({
                    {"id",  static_cast<int>(i)},
                    {"box", {b[0], b[1], b[2], b[3]}},
                });
            }
            frames.push_back({
                {"request_id",     pf.nodeId},
                {"image_jpeg_b64", base64Encode(pf.jpeg.data(), pf.jpeg.size())},
                {"boxes",          boxes},
            });
        }

        nlohmann::json body = {{"frames", frames}};
        auto res = client_->Post(classifyPath_.c_str(),
                                  body.dump(), "application/json");
        if (!res)
        {
            UWARN("SemanticWorker: /classify_batch POST failed");
            return false;
        }
        if (res->status != 200)
        {
            UWARN("SemanticWorker: /classify_batch status=%d body=%.200s",
                  res->status, res->body.c_str());
            return false;
        }

        // Response shape:
        //   { result: { frames: [{request_id, classifications: [{id,label,confidence,visual_angle}]}] },
        //     usage: {...}, inference_ms: int }
        try
        {
            auto j = nlohmann::json::parse(res->body);
            const int infMs = j.value("inference_ms", -1);

            const auto & framesOut = j.at("result").at("frames");

            // Index batch by request_id (== nodeId) for fast lookup.
            std::unordered_map<int, const PendingFrame*> byNode;
            for (const auto & pf : batch) byNode[pf.nodeId] = &pf;

            int dispatched = 0;
            for (const auto & f : framesOut)
            {
                const int nodeId = f.value("request_id", -1);
                auto it = byNode.find(nodeId);
                if (it == byNode.end()) continue;

                const PendingFrame & pf = *it->second;
                std::vector<LabeledBox> labeled;
                labeled.reserve(pf.boxes.size());

                if (f.contains("classifications") && f["classifications"].is_array())
                {
                    for (const auto & c : f["classifications"])
                    {
                        const int id = c.value("id", -1);
                        if (id < 0 || id >= static_cast<int>(pf.boxes.size())) continue;
                        const auto & b = pf.boxes[id];
                        LabeledBox lb;
                        lb.x1 = b[0]; lb.y1 = b[1]; lb.x2 = b[2]; lb.y2 = b[3];
                        lb.label       = c.value("label", "");
                        lb.confidence  = c.value("confidence", 0.0f);
                        if (c.contains("visual_angle") && !c["visual_angle"].is_null())
                            lb.visualAngle = c["visual_angle"].get<float>();
                        labeled.push_back(std::move(lb));
                    }
                }

                if (callback_) callback_(nodeId, labeled);
                byNode.erase(it);
                ++dispatched;
            }

            // Any frames not present in the response — dispatch empty so
            // mask store entries don't leak.
            for (const auto & kv : byNode)
                if (callback_) callback_(kv.first, {});

            UDEBUG("SemanticWorker: /classify_batch dispatched=%d/%zu inference_ms=%d",
                   dispatched, batch.size(), infMs);
        }
        catch (const std::exception & e)
        {
            UWARN("SemanticWorker: /classify_batch JSON parse failed: %s", e.what());
            return false;
        }
        return true;
    }

    // ---- HTTP client setup ----------------------------------------------

    bool ensureClient()
    {
        if (client_) return true;

        // url_ may include a path (legacy: ".../detect"). We strip it and
        // hard-wire endpoint paths instead so both /detect and
        // /classify_batch share one connection.
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
        client_->set_read_timeout(15, 0);   // /classify_batch is slow
        client_->set_write_timeout(5, 0);
        UINFO("SemanticWorker: HTTP client ready -> %s:%d (paths %s, %s)",
              host_.c_str(), port_,
              detectPath_.c_str(), classifyPath_.c_str());
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
    std::string                      detectPath_   = "/detect";
    std::string                      classifyPath_ = "/classify_batch";
    std::atomic<int>                 reqIdCounter_{0};
};

#endif /* SEMANTICWORKER_H_ */
