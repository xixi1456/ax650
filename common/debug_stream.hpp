/**
 * @file debug_stream.hpp
 * @brief 极简 HTTP MJPEG 调试服务器（单头文件，零外部依赖）
 *
 * 用法：
 *   // 1. main.cpp 里启动
 *   auto &s = debug::DebugStream::instance();
 *   s.start(8090);
 *
 *   // 2. 任意子模块里推送一帧（线程安全）
 *   debug::DebugStream::instance().update("sensor", im);
 *   debug::DebugStream::instance().update("detect", im2show);
 *
 *   // 3. 浏览器访问：
 *   //    http://host:8090/              — 六宫格首页
 *   //    http://host:8090/sensor       — 单独 MJPEG 流
 *   //    http://host:8090/detect
 *   //    http://host:8090/corner
 *   //    http://host:8090/planner
 *   //    http://host:8090/snapshot     — 最新六宫格截图 PNG
 *
 * 4. main.cpp 退出前 stop()：
 *   debug::DebugStream::instance().stop();
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <vector>
#include <deque>
#include <cstring>
#include <sstream>
#include <iostream>
#include <chrono>
#include <algorithm>

namespace debug {

// 角度历史点（用于折线图）
struct AnglePoint {
    double t;            // 相对时间（秒）
    float imu_yaw;
    float imu_pitch;
    float tgt_yaw;
    float tgt_pitch;
    bool  in_autoaim;
    bool  has_target;
};

// 坐标变换诊断数据
struct CoordDiag {
    bool  valid = false;
    // Camera 坐标系 (solvePnP 输出)
    float pc_x=0, pc_y=0, pc_z=0;
    // IMU 坐标系 (R_c2i * pc + T_c2i)
    float pi_x=0, pi_y=0, pi_z=0;
    // World 坐标系 (R_world2imu^T * pi)
    float pw_x=0, pw_y=0, pw_z=0;
    // 云台目标角 cal_gimbal_target 计算
    float gimbal_yaw=0, gimbal_pitch=0;
    // R_c2i (3x3 row-major)
    float R_c2i[9] = {1,0,0, 0,1,0, 0,0,1};
    // T_c2i
    float T_c2i[3] = {0,0,0};
    // R_world2imu (3x3 row-major)
    float R_world2imu[9] = {1,0,0, 0,1,0, 0,0,1};
    // IMU 当前姿态
    float imu_yaw_deg=0, imu_pitch_deg=0;
    // 云台指令偏移
    float cmd_yaw_offset=0, cmd_pitch_offset=0;
};

class DebugStream {
public:
    static DebugStream &instance() {
        static DebugStream s;
        return s;
    }

    // 启动 HTTP 服务器（后台线程）
    bool start(int port = 8090) {
        if (running_.load()) return true;
        port_ = port;
        running_.store(true);
        server_thread_ = std::thread([this]() { this->serve(); });
        return true;
    }

    // 停止
    void stop() {
        running_.store(false);
        if (server_thread_.joinable()) server_thread_.join();
    }

    // 推送一帧（线程安全，非阻塞）
    // name: "sensor" / "detect" / "corner" / "planner" / "preprocess"
    // 如果服务器没启动（start 没被调用），update 是零开销（一次 atomic load 后 return）
    void update(const std::string &name, const cv::Mat &frame) {
        if (!running_.load()) return;
        if (frame.empty()) return;
        std::lock_guard<std::mutex> lock(mtx_);
        std::vector<uchar> encoded;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", frame, encoded, params);
        buffers_[name] = std::move(encoded);
        last_tick_[name] = now_us();
    }

    // 推送角度数据（IMU当前 + Planner目标），用于 /chart 折线图
    void update_angles(float imu_yaw, float imu_pitch,
                       float tgt_yaw, float tgt_pitch,
                       bool in_autoaim, bool has_target) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lock(mtx_);
        auto now = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now().time_since_epoch()).count();
        if (angle_start_us_ == 0) angle_start_us_ = now;
        AnglePoint pt;
        pt.t = (now - angle_start_us_) / 1e6;
        pt.imu_yaw = imu_yaw;
        pt.imu_pitch = imu_pitch;
        pt.tgt_yaw = tgt_yaw;
        pt.tgt_pitch = tgt_pitch;
        pt.in_autoaim = in_autoaim;
        pt.has_target = has_target;
        angles_.push_back(pt);
        if (angles_.size() > MAX_ANGLE_POINTS) angles_.pop_front();
    }

    // 推送坐标变换诊断数据（一帧即可覆盖上一帧）
    void update_coord_diag(const CoordDiag &d) {
        std::lock_guard<std::mutex> lock(mtx_);
        diag_ = d;
    }

    // 读取当前诊断数据（用于增量更新）
    CoordDiag peek_coord_diag() {
        std::lock_guard<std::mutex> lock(mtx_);
        return diag_;
    }

    // 注册显示名称（可选，让首页标签更好看）
    void set_label(const std::string &name, const std::string &label) {
        labels_[name] = label;
    }

    // 是否激活（服务器已启动且有客户端在拉流）——用来让子模块判断要不要做 clone+drawing
    bool active() const { return running_.load(); }

private:
    DebugStream() = default;
    ~DebugStream() { stop(); }
    DebugStream(const DebugStream &) = delete;

    int port_ = 8090;
    std::atomic<bool> running_{false};
    std::thread server_thread_;

    mutable std::mutex mtx_;
    std::unordered_map<std::string, std::vector<uchar>> buffers_;
    std::unordered_map<std::string, int64_t> last_tick_;
    std::unordered_map<std::string, std::string> labels_;

    // 角度历史环形缓冲
    static constexpr size_t MAX_ANGLE_POINTS = 2000;  // 100Hz × 20s
    std::deque<AnglePoint> angles_;
    int64_t angle_start_us_ = 0;

    // 最新坐标变换诊断
    CoordDiag diag_;

    static int64_t now_us() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    // 所有已注册端点（按 update 的次数自动增长）
    std::vector<std::string> known_names() const {
        std::vector<std::string> out;
        for (auto &kv : buffers_) out.push_back(kv.first);
        // 固定顺序
        std::vector<std::string> order = {"sensor", "preprocess", "detect", "corner", "planner"};
        std::vector<std::string> result;
        for (auto &o : order)
            if (std::find(out.begin(), out.end(), o) != out.end()) result.push_back(o);
        for (auto &n : out)
            if (std::find(result.begin(), result.end(), n) == result.end()) result.push_back(n);
        return result;
    }

    void serve() {
        int lsock = socket(AF_INET, SOCK_STREAM, 0);
        if (lsock < 0) { std::cerr << "[DebugStream] socket fail\n"; return; }
        int opt = 1;
        setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);
        if (bind(lsock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            std::cerr << "[DebugStream] bind fail on port " << port_ << "\n";
            close(lsock);
            return;
        }
        listen(lsock, 16);
        std::cerr << "[DebugStream] MJPEG server on http://0.0.0.0:" << port_ << "/\n";

        while (running_.load()) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(lsock, &rfds);
            struct timeval tv{1, 0};
            int r = select(lsock + 1, &rfds, nullptr, nullptr, &tv);
            if (r <= 0) continue;

            struct sockaddr_in cli{};
            socklen_t clen = sizeof(cli);
            int c = accept(lsock, (struct sockaddr *)&cli, &clen);
            if (c < 0) continue;

            std::thread([this, c]() { this->handle(c); }).detach();
        }
        close(lsock);
    }

    void handle(int fd) {
        char buf[2048] = {};
        ssize_t n = recv(fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) { close(fd); return; }

        // 解析路径
        std::string req(buf, n);
        size_t sp = req.find(' ');
        if (sp == std::string::npos) { close(fd); return; }
        std::string path = req.substr(0, sp);
        // 只看方法后面的路径（跳过 "GET "）
        sp = req.find(' ', sp + 1);
        if (sp != std::string::npos) {
            size_t start = req.find('/', req.find(' ')) + 1;
            path = req.substr(start, sp - start);
            // 去掉 query string
            auto q = path.find('?');
            if (q != std::string::npos) path = path.substr(0, q);
        }

        if (path == "" || path == "/index.html") {
            send_index(fd);
        } else if (path == "snapshot") {
            send_snapshot(fd);
        } else if (path == "health") {
            send_health(fd);
        } else if (path == "chart") {
            send_chart(fd);
        } else if (path == "angles.json") {
            send_angles_json(fd);
        } else if (path == "diag") {
            send_diag(fd);
        } else if (buffers_.count(path)) {
            send_mjpeg(fd, path);
        } else {
            send_404(fd);
        }
        close(fd);
    }

    static void write_all(int fd, const void *data, size_t len) {
        const char *p = (const char *)data;
        size_t sent = 0;
        while (sent < len) {
            ssize_t r = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
            if (r <= 0) return;
            sent += r;
        }
    }

    static void send_str(int fd, const std::string &s) { write_all(fd, s.data(), s.size()); }

    // --- MJPEG 流 ---
    void send_mjpeg(int fd, const std::string &name) {
        std::string header =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n"
            "\r\n";
        send_str(fd, header);

        // 客户端断开或 name 不再注册时退出
        int64_t last_send = 0;
        while (running_.load()) {
            std::vector<uchar> jpg;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                auto it = buffers_.find(name);
                if (it == buffers_.end()) break;
                jpg = it->second;
            }
            if (jpg.empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(30)); continue; }

            // 节流：每秒 ~15 帧
            int64_t now = now_us();
            if (now - last_send < 66000) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
            last_send = now;

            char part_hdr[128];
            int hlen = snprintf(part_hdr, sizeof(part_hdr),
                                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                                jpg.size());
            write_all(fd, part_hdr, hlen);
            write_all(fd, jpg.data(), jpg.size());
            write_all(fd, "\r\n", 2);
        }
    }

    // --- 首页 ---
    void send_index(int fd) {
        auto names = known_names();
        std::string imgs;
        for (auto &n : names) {
            std::string label = labels_.count(n) ? labels_[n] : n;
            imgs += "<div class='cell'>";
            imgs += "<div class='title'>" + label + "</div>";
            imgs += "<img src='/" + n + "' onerror=\"this.src='data:image/svg+xml;utf8,<svg xmlns=%22http://www.w3.org/2000/svg%22 width=%22640%22 height=%22360%22><rect width=%22100%25%22 height=%22100%25%22 fill=%22%23222%22/><text x=%2250%25%22 y=%2250%25%22 fill=%22%23888%22 text-anchor=%22middle%22 font-size=%2220%22>waiting</text></svg>'\">";
            imgs += "</div>";
        }

        int active = (int)names.size();
        std::string grid_cols = active <= 2 ? "1fr" : active <= 4 ? "1fr 1fr" : "1fr 1fr 1fr";
        std::string badge_class = active ? "on" : "off";

        // 图表 Canvas + JS（内联，避免外链依赖）
        const char *chart_js = R"(
<script>
(function(){
const W=380,H=160,P=28;
function draw(cid,imuKey,tgtKey,imuColor,tgtColor,title){
  const c=document.getElementById(cid); if(!c)return;
  const g=c.getContext('2d');
  g.fillStyle='#0a0b0f';g.fillRect(0,0,W,H);
  g.strokeStyle='#2a2d36';g.lineWidth=1;
  g.beginPath();g.moveTo(P,H-P);g.lineTo(W-P,H-P);g.lineTo(P,P);g.stroke();
  g.font='9px monospace';g.fillStyle='#6b7280';g.textAlign='right';
  for(let i=0;i<=4;i++){let y=P+(H-2*P)*i/4;g.beginPath();g.moveTo(P,y);g.lineTo(W-P,y);g.stroke();
    let v=40-20*i;g.fillText(v+'°',P-3,y+3);}
  let pts=window.__pts||[];
  if(pts.length<2){g.fillStyle='#6b7280';g.textAlign='center';g.fillText('...',W/2,H/2);return;}
  let t0=pts[0].t,t1=pts[pts.length-1].t;
  function sx(t){return P+(W-2*P)*(t-t0)/Math.max(0.001,t1-t0);}
  function sy(v){return P+(H-2*P)*(40-v)/80;}
  g.strokeStyle=imuColor;g.lineWidth=1.5;g.beginPath();
  for(let i=0;i<pts.length;i++){let x=sx(pts[i].t),y=sy(pts[i][imuKey]);if(i==0)g.moveTo(x,y);else g.lineTo(x,y);}
  g.stroke();
  g.strokeStyle=tgtColor;g.setLineDash([3,2]);g.beginPath();
  for(let i=0;i<pts.length;i++){let x=sx(pts[i].t),y=sy(pts[i][tgtKey]);if(i==0)g.moveTo(x,y);else g.lineTo(x,y);}
  g.stroke();g.setLineDash([]);
  g.fillStyle='#9ca3af';g.textAlign='left';g.fillText(title,P+2,P+10);
}
async function tick(){
  try{const r=await fetch('/angles.json?_='+Date.now());if(!r.ok)return;window.__pts=(await r.json()).points;}catch(e){}
  draw('cy','imu_yaw','tgt_yaw','#60a5fa','#ef4444','Yaw');
  draw('cp','imu_pitch','tgt_pitch','#34d399','#f59e0b','Pitch');
  const st=document.getElementById('st');
  if(window.__pts&&window.__pts.length>0){
    const p=window.__pts[window.__pts.length-1];
    st.textContent='autoaim='+(p.in_autoaim?'ON':'off')+' target='+(p.has_target?'Y':'-');
  }
}
setInterval(tick,100);tick();
})();
</script>)";

        std::string page = std::string() +
            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"
            "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<title>Auto-Aim Debug Stream</title>"
            "<style>"
            "body{background:#0f1115;color:#e0e0e0;font-family:monospace;margin:0;padding:20px}"
            "h1{font-size:20px;margin:0 0 12px}"
            ".badge{display:inline-block;padding:2px 10px;background:#1a1d24;border-radius:12px;margin-left:8px;font-size:13px}"
            ".badge.on{color:#4ade80}"
            ".badge.off{color:#ef4444}"
            ".grid{display:grid;gap:10px;grid-template-columns:" + grid_cols + ";}"
            ".cell{background:#1a1d24;border-radius:8px;padding:8px}"
            ".title{font-size:12px;color:#9ca3af;margin-bottom:4px}"
            "img{width:100%;display:block;border-radius:4px;background:#111}"
            "a{color:#60a5fa;text-decoration:none}"
            ".chart-box{background:#1a1d24;border-radius:8px;padding:8px;margin-top:10px;display:grid;grid-template-columns:1fr 1fr;gap:8px}"
            ".chart-item canvas{display:block;width:100%;height:auto;background:#0a0b0f;border-radius:4px}"
            ".chart-label{font-size:10px;color:#9ca3af;margin-bottom:3px}"
            ".legend{font-size:11px;color:#9ca3af;margin-top:4px}"
            ".dot{display:inline-block;width:7px;height:7px;border-radius:50%;margin-right:3px;vertical-align:middle}"
            "</style></head><body>"
            "<h1>🎯 Auto-Aim Debug<span class='badge " + badge_class + "'>" + std::to_string(active) + " active</span> <span class='badge' id='st'>loading</span></h1>"
            "<div class='grid'>" + imgs + "</div>"
            "<div class='chart-box'>"
            "<div class='chart-item'><div class='chart-label'>📡 Yaw (实际 vs 目标)</div><canvas id='cy' width='380' height='160'></canvas>"
            "<div class='legend'><span class='dot' style='background:#60a5fa'></span>IMU &nbsp; <span class='dot' style='background:#ef4444'></span>Target</div></div>"
            "<div class='chart-item'><div class='chart-label'>📡 Pitch (实际 vs 目标)</div><canvas id='cp' width='380' height='160'></canvas>"
            "<div class='legend'><span class='dot' style='background:#34d399'></span>IMU &nbsp; <span class='dot' style='background:#f59e0b'></span>Target</div></div>"
            "</div>"
            + chart_js;

        // 底部端点链接
        page += "<p style='color:#6b7280;font-size:12px;margin-top:16px'>";
        page += "单独端点: ";
        for (auto &n : names) page += "<a href='/" + n + "'>/" + n + "</a> ";
        page += "| <a href='/snapshot'>/snapshot</a> | <a href='/health'>/health</a> | <a href='/chart'>📈 /chart</a>";
        page += "</p></body></html>";

        send_str(fd, page);
    }

    // --- 六宫格快照 PNG ---
    void send_snapshot(int fd) {
        auto names = known_names();
        std::vector<cv::Mat> mats;
        std::vector<std::string> lbls;
        for (auto &n : names) {
            std::vector<uchar> jpg;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                auto it = buffers_.find(n);
                if (it == buffers_.end()) continue;
                jpg = it->second;
            }
            if (jpg.empty()) continue;
            cv::Mat m = cv::imdecode(jpg, cv::IMREAD_COLOR);
            if (m.empty()) continue;
            mats.push_back(m);
            lbls.push_back(labels_.count(n) ? labels_[n] : n);
        }
        if (mats.empty()) { send_404(fd); return; }

        int cols = mats.size() <= 2 ? 1 : mats.size() <= 4 ? 2 : 3;
        int rows = (int)ceil(mats.size() / (double)cols);
        int tw = 480, th = 360;
        cv::Mat big(th * rows, tw * cols, CV_8UC3, cv::Scalar(20, 22, 28));
        for (size_t i = 0; i < mats.size(); i++) {
            cv::Mat resized;
            cv::resize(mats[i], resized, {tw, th});
            int r = i / cols, c = i % cols;
            resized.copyTo(big(cv::Rect(c * tw, r * th, tw, th)));
            cv::putText(big, lbls[i], {c * tw + 8, r * th + 20}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 1);
        }

        std::vector<uchar> png;
        cv::imencode(".png", big, png);
        std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: image/png\r\nContent-Length: " + std::to_string(png.size()) + "\r\nConnection: close\r\n\r\n";
        send_str(fd, hdr);
        write_all(fd, png.data(), png.size());
    }

    void send_health(int fd) {
        std::string body = "{";
        bool first = true;
        auto names = known_names();
        for (auto &n : names) {
            std::lock_guard<std::mutex> lock(mtx_);
            int64_t ts = last_tick_.count(n) ? last_tick_[n] : 0;
            double age_ms = ts ? (now_us() - ts) / 1000.0 : -1;
            if (!first) body += ",";
            first = false;
            body += "\"" + n + "\":{";
            body += "\"active\":" + std::string(age_ms > 0 && age_ms < 3000 ? "true" : "false") + ",";
            body += "\"age_ms\":" + std::to_string((int)age_ms);
            body += "}";
        }
        body += "}";
        std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n";
        send_str(fd, hdr + body);
    }

    void send_404(int fd) {
        std::string s = "HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\nConnection: close\r\n\r\nnot found";
        send_str(fd, s);
    }

    void send_diag(int fd) {
        CoordDiag d;
        { std::lock_guard<std::mutex> lock(mtx_); d = diag_; }

        auto m3 = [](const float *m) -> std::string {
            char buf[256];
            snprintf(buf, sizeof(buf), "{%g,%g,%g; %g,%g,%g; %g,%g,%g}",
                m[0],m[1],m[2], m[3],m[4],m[5], m[6],m[7],m[8]);
            return buf;
        };

        std::string body;
        body += "{\n";
        body += "  \"valid\": " + std::string(d.valid ? "true" : "false") + ",\n";
        body += "  \"camera_pc\": [" + std::to_string(d.pc_x) + "," + std::to_string(d.pc_y) + "," + std::to_string(d.pc_z) + "],\n";
        body += "  \"imu_pi\":    [" + std::to_string(d.pi_x) + "," + std::to_string(d.pi_y) + "," + std::to_string(d.pi_z) + "],\n";
        body += "  \"world_pw\":  [" + std::to_string(d.pw_x) + "," + std::to_string(d.pw_y) + "," + std::to_string(d.pw_z) + "],\n";
        body += "  \"gimbal\":    {\"yaw\":" + std::to_string(d.gimbal_yaw) + ",\"pitch\":" + std::to_string(d.gimbal_pitch) + "},\n";
        body += "  \"imu_att\":   {\"yaw_deg\":" + std::to_string(d.imu_yaw_deg) + ",\"pitch_deg\":" + std::to_string(d.imu_pitch_deg) + "},\n";
        body += "  \"cmd_offset\":{\"yaw\":" + std::to_string(d.cmd_yaw_offset) + ",\"pitch\":" + std::to_string(d.cmd_pitch_offset) + "},\n";
        body += "  \"R_c2i\": " + m3(d.R_c2i) + ",\n";
        body += "  \"T_c2i\": [" + std::to_string(d.T_c2i[0]) + "," + std::to_string(d.T_c2i[1]) + "," + std::to_string(d.T_c2i[2]) + "],\n";
        body += "  \"R_world2imu\": " + m3(d.R_world2imu) + "\n";
        body += "}";

        std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: " + std::to_string(body.size()) + "\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n";
        send_str(fd, hdr + body);
    }

    // --- 角度折线图 HTML ---
    void send_chart(int fd) {
        std::string page = std::string() +
            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n"
            "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<title>Yaw / Pitch Chart</title>"
            "<style>"
            "body{background:#0f1115;color:#e0e0e0;font-family:monospace;margin:0;padding:16px}"
            "h1{font-size:16px;margin:0 0 8px}"
            ".row{display:flex;gap:16px;margin-bottom:16px}"
            ".card{background:#1a1d24;border-radius:8px;padding:8px}"
            ".stat{font-size:12px;color:#9ca3af;margin-right:16px}"
            ".dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px;vertical-align:middle}"
            "canvas{display:block;background:#0a0b0f;border-radius:4px}"
            ".grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}"
            "</style></head><body>"
            "<h1>📡 Gimbal Angles <span class='stat' id='st'></span></h1>"
            "<div class='row'>"
            "<span><span class='dot' style='background:#60a5fa'></span>IMU Yaw</span>"
            "<span><span class='dot' style='background:#ef4444'></span>Target Yaw</span>"
            "<span><span class='dot' style='background:#34d399'></span>IMU Pitch</span>"
            "<span><span class='dot' style='background:#f59e0b'></span>Target Pitch</span>"
            "</div>"
            "<div class='grid'>"
            "<canvas id='cy' width='720' height='260'></canvas>"
            "<canvas id='cp' width='720' height='260'></canvas>"
            "</div>"
            "<script>"
            "const W=720,H=260,P=36;"
            "function drawCanvas(cid,imuKey,tgtKey,imuColor,tgtColor,title){"
            "const c=document.getElementById(cid),g=c.getContext('2d');"
            "g.fillStyle='#0a0b0f';g.fillRect(0,0,W,H);"
            "g.strokeStyle='#2a2d36';g.lineWidth=1;"
            "g.beginPath();g.moveTo(P,H-P);g.lineTo(W-P,H-P);g.lineTo(P,P);g.stroke();"
            "g.font='10px monospace';g.fillStyle='#6b7280';g.textAlign='right';"
            "for(let i=0;i<=4;i++){let y=P+(H-2*P)*i/4;g.beginPath();g.moveTo(P,y);g.lineTo(W-P,y);g.stroke();"
            "let v=60-30*i;g.fillText(v.toFixed(0)+'°',P-4,y+3);}"
            "g.textAlign='center';"
            "let n=window.__pts?window.__pts.length:0;"
            "if(n<2){g.fillStyle='#6b7280';g.fillText('waiting...',W/2,H/2);return;}"
            "let pts=window.__pts;"
            "let t0=pts[0].t,t1=pts[n-1].t;"
            "function sx(t){return P+(W-2*P)*(t-t0)/Math.max(0.001,t1-t0);}"
            "function sy(v){return P+(H-2*P)*(60-v)/120;}"
            "g.strokeStyle=imuColor;g.lineWidth=2;g.beginPath();"
            "for(let i=0;i<n;i++){let x=sx(pts[i].t),y=sy(pts[i][imuKey]);if(i==0)g.moveTo(x,y);else g.lineTo(x,y);}"
            "g.stroke();"
            "g.strokeStyle=tgtColor;g.lineWidth=2;g.setLineDash([4,3]);g.beginPath();"
            "for(let i=0;i<n;i++){let x=sx(pts[i].t),y=sy(pts[i][tgtKey]);if(i==0)g.moveTo(x,y);else g.lineTo(x,y);}"
            "g.stroke();g.setLineDash([]);"
            "g.fillStyle='#9ca3af';g.textAlign='left';g.fillText(title+'  ('+t1.toFixed(1)+'s)',P+4,P+12);"
            "}"
            "async function tick(){"
            "try{const r=await fetch('/angles.json?_='+Date.now());if(!r.ok)return;const j=await r.json();"
            "window.__pts=j.points;"
            "const st=document.getElementById('st');"
            "if(j.points.length>0){"
            "const p=j.points[j.points.length-1];"
            "st.textContent='autoaim='+(p.in_autoaim?'ON':'off')+' target='+(p.has_target?'Y':'-')+'"
            " imu=('+'+p.imu_pitch.toFixed(1)).slice(-5)+','+('+'+p.imu_yaw.toFixed(1)).slice(-6)+'°';"
            "}"
            "}catch(e){}"
            "drawCanvas('cy','imu_yaw','tgt_yaw','#60a5fa','#ef4444','Yaw');"
            "drawCanvas('cp','imu_pitch','tgt_pitch','#34d399','#f59e0b','Pitch');"
            "}"
            "setInterval(tick,80);tick();"
            "</script>"
            "</body></html>";
        std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: " + std::to_string(page.size()) + "\r\nConnection: close\r\n\r\n";
        send_str(fd, hdr + page);
    }

    // --- 角度数据 JSON（给 /chart 的 JS 轮询） ---
    void send_angles_json(int fd) {
        std::string body;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            body += "{\"points\":[";
            bool first = true;
            // 最多返回最近 20 秒
            double cutoff_t = angles_.empty() ? 0.0 : angles_.back().t - 20.0;
            for (auto &p : angles_) {
                if (p.t < cutoff_t) continue;
                if (!first) body += ",";
                first = false;
                char buf[128];
                snprintf(buf, sizeof(buf),
                    "{\"t\":%.2f,\"imu_yaw\":%.2f,\"imu_pitch\":%.2f,"
                    "\"tgt_yaw\":%.2f,\"tgt_pitch\":%.2f,"
                    "\"in_autoaim\":%s,\"has_target\":%s}",
                    p.t, p.imu_yaw, p.imu_pitch, p.tgt_yaw, p.tgt_pitch,
                    p.in_autoaim ? "true" : "false",
                    p.has_target ? "true" : "false");
                body += buf;
            }
            body += "]}";
        }
        std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: " + std::to_string(body.size()) + "\r\nCache-Control: no-store, no-cache, must-revalidate\r\nPragma: no-cache\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n";
        send_str(fd, hdr + body);
    }
};

} // namespace debug
