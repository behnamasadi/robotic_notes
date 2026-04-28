// Minimal V4L2 zero-copy capture (Linux only).
//
// Goal: capture N frames from /dev/videoX without ever memcpy-ing pixel data.
//
// Why this is "zero-copy":
//   The kernel driver writes pixels straight into a small pool of buffers
//   that we mmap() *once*, up front. Every frame the driver hands us back
//   the same pointer to one of those buffers via VIDIOC_DQBUF. We read
//   from that pointer, then return the buffer to the driver via VIDIOC_QBUF.
//   No copy from kernel to user; no per-frame allocation.
//
// Reference: V4L2 streaming I/O ("Streaming I/O (Memory Mapping)") — see
//   https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/mmap.html
//
// Build:  g++ -std=c++20 -O2 v4l2_zero_copy_capture.cpp -o v4l2_zero_copy_capture
// Run:    ./v4l2_zero_copy_capture            # /dev/video0, 30 frames
//         ./v4l2_zero_copy_capture /dev/video1 60
//
// Caveats: educational. No format negotiation fallback, no select()/poll(),
// no error recovery. If the device doesn't support YUYV 640x480 with MMAP
// streaming, switch the FOURCC / size below or pick a different camera.

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// Wrap ioctl so it retries on EINTR (signals can interrupt these calls).
static int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    return r;
}

#define DIE(msg) do { std::perror(msg); std::exit(1); } while (0)

struct MappedBuffer {
    void*       start;   // mmap'd userspace pointer — reused every frame
    std::size_t length;  // size of this buffer in bytes
};

int main(int argc, char** argv) {
    const char* dev      = (argc > 1) ? argv[1] : "/dev/video0";
    const int   n_frames = (argc > 2) ? std::atoi(argv[2]) : 30;

    // 1. Open the device. O_NONBLOCK lets DQBUF return EAGAIN instead of
    //    blocking forever — useful if you want to integrate with poll().
    //    For this minimal example we use blocking DQBUF, so plain O_RDWR is fine.
    int fd = open(dev, O_RDWR);
    if (fd < 0) DIE("open");

    // 2. Sanity-check: device must support video capture and streaming I/O.
    v4l2_capability cap{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) DIE("VIDIOC_QUERYCAP");
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        { std::fprintf(stderr, "%s: not a capture device\n", dev); return 1; }
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
        { std::fprintf(stderr, "%s: no streaming I/O — zero-copy unavailable\n", dev); return 1; }
    std::printf("device : %s (%s)\n", dev, cap.card);

    // 3. Set the pixel format. YUYV 640x480 is supported by ~every webcam.
    //    The driver may adjust width/height to what it actually supports —
    //    always read fmt back after the ioctl.
    v4l2_format fmt{};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = 640;
    fmt.fmt.pix.height      = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) DIE("VIDIOC_S_FMT");
    std::printf("format : %ux%u, %u bytes/frame\n",
                fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);

    // 4. Ask the driver for a small ring of buffers in MMAP memory.
    //    The driver may return fewer than requested.
    constexpr std::uint32_t kRequested = 4;
    v4l2_requestbuffers req{};
    req.count  = kRequested;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) DIE("VIDIOC_REQBUFS");
    if (req.count < 2) { std::fprintf(stderr, "too few buffers: %u\n", req.count); return 1; }

    // 5. For each buffer index, query its offset+length and mmap it into us.
    //    These pointers are the *only* allocation we'll do for pixel data.
    std::vector<MappedBuffer> bufs(req.count);
    for (std::uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer b{};
        b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index  = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) DIE("VIDIOC_QUERYBUF");

        bufs[i].length = b.length;
        bufs[i].start  = mmap(nullptr, b.length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, b.m.offset);
        if (bufs[i].start == MAP_FAILED) DIE("mmap");
        std::printf("buffer %u: %p  len=%zu\n", i, bufs[i].start, bufs[i].length);
    }

    // 6. Hand every buffer to the driver so it has somewhere to write into.
    for (std::uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(fd, VIDIOC_QBUF, &b) < 0) DIE("VIDIOC_QBUF");
    }

    // 7. Start streaming.
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) DIE("VIDIOC_STREAMON");

    // 8. The zero-copy loop.
    //    DQBUF blocks until the driver hands a filled buffer back. The buffer
    //    is identified by index; bufs[index].start is the SAME userspace
    //    pointer we mmap'd in step 5 — proof that no per-frame copy happened.
    //    We do a trivial "process" (sum of bytes) so the optimizer can't
    //    elide the read, then QBUF the buffer back so the driver can reuse it.
    std::printf("\ncapturing %d frames (zero-copy)...\n", n_frames);
    for (int i = 0; i < n_frames; ++i) {
        v4l2_buffer b{};
        b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) DIE("VIDIOC_DQBUF");

        const auto* px = static_cast<const std::uint8_t*>(bufs[b.index].start);
        std::uint64_t checksum = 0;
        for (std::uint32_t k = 0; k < b.bytesused; ++k) checksum += px[k];

        std::printf("frame %3d  buf=%u  ptr=%p  bytes=%u  ts=%ld.%06ld  sum=%lu\n",
                    i, b.index, bufs[b.index].start, b.bytesused,
                    (long)b.timestamp.tv_sec, (long)b.timestamp.tv_usec, checksum);

        if (xioctl(fd, VIDIOC_QBUF, &b) < 0) DIE("VIDIOC_QBUF");
    }

    // 9. Tear down.
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) DIE("VIDIOC_STREAMOFF");
    for (auto& m : bufs) munmap(m.start, m.length);
    close(fd);
    return 0;
}
