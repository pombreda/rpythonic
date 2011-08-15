# 18 "/usr/local/include/GLG3D/VideoInput.h" 2 3

// forward declarations for ffmpeg
struct AVFrame;
struct AVFormatContext;
struct AVCodecContext;
struct AVCodec;
struct AVPacket;

namespace G3D {

class GImage;

/** 
    Read video files from MPG, MP4, AVI, MOV, OGG, ASF, and WMV files.
    
    There are three ways to read: by frame index, by time position, and
    selectively reading a frame if it is time for it to display.

    Reading frames in non-sequential order can decrease performance due 
    to seek times.
 */
class VideoInput : public ReferenceCountedObject {
public:

    class Settings {
    public:
        Settings(): numBuffers(2) {}

        /** Number of asynchronous buffers to allocate. */
        int numBuffers;
    };

    typedef ReferenceCountedPointer<VideoInput> Ref;

    /** @return NULL if the file is not found or cannot be opened. */
    static Ref fromFile(__const__ std::string& filename, __const__ Settings& settings = Settings());

    ~VideoInput();

    /** Advances the current file position to pos() + timeStep. If
        that advance stepped over a frame boundary, sets frame to that
        frame and returns true. Otherwise, returns false. */
    bool readNext(RealTime timeStep, Texture::Ref& frame);
    /** Advances the current file position to pos() + timeStep. If
        that advance stepped over a frame boundary, sets frame to that
        frame and returns true. Otherwise, returns false. */
    bool readNext(RealTime timeStep, GImage& frame);
    /** Advances the current file position to pos() + timeStep. If
        that advance stepped over a frame boundary, sets frame to that
        frame and returns true. Otherwise, returns false. */
    bool readNext(RealTime timeStep, Image3uint8::Ref& frame);
    /** Advances the current file position to pos() + timeStep. If
        that advance stepped over a frame boundary, sets frame to that
        frame and returns true. Otherwise, returns false. */
    bool readNext(RealTime timeStep, Image3::Ref& frame);


    /** Gets the frame at playback position pos and returns false if
        pos is out of bounds. 

        @param pos Time in seconds from the beginning of playback.  */
    bool readFromPos(RealTime pos, Texture::Ref& frame);
    /** Gets the frame at playback position pos and returns false if
        pos is out of bounds. 

        @param pos Time in seconds from the beginning of playback.  */
    bool readFromPos(RealTime pos, GImage& frame);
    /** Gets the frame at playback position pos and returns false if
        pos is out of bounds. 

        @param pos Time in seconds from the beginning of playback.  */
    bool readFromPos(RealTime pos, Image3uint8::Ref& frame);
    /** Gets the frame at playback position pos and returns false if
        pos is out of bounds. 

        @param pos Time in seconds from the beginning of playback.  */
    bool readFromPos(RealTime pos, Image3::Ref& frame);


    /** Gets the frame at frame index and returns false if index is out of bounds. */
    bool readFromIndex(int index, Texture::Ref& frame);
    /** Gets the frame at frame index and returns false if index is out of bounds. */
    bool readFromIndex(int index, GImage& frame);
    /** Gets the frame at frame index and returns false if index is out of bounds. */
    bool readFromIndex(int index, Image3uint8::Ref& frame);
    /** Gets the frame at frame index and returns false if index is out of bounds. */
    bool readFromIndex(int index, Image3::Ref& frame);

    /** Seek to playback position
        @param pos Seek time, in seconds.*/
    void setTimePosition(RealTime pos);
    /** Seek to playback position
        @param index Seek frame index (zero based)*/
    void setIndex(int index);


    /** Seek ahead in playback position
        @param length seek time in seconds. */
    void skipTime(RealTime length);

    /** Seek ahead @a length frames. */
    void skipFrames(int length);

    /** Horizontal pixels in each frame */
    int width() __const__;

    /** Vertical pixels in each frame */
    int height() __const__;

    /** Preferred playback speed in frames per second */
    RealTime fps() __const__;

    /** Length of video in seconds */
    RealTime length() __const__;

    /** Current playback position in seconds */
    RealTime pos() __const__;

    /** Length of video in frames */
    int numFrames() __const__;

    /** Current playback frame index */
    int index() __const__;

    bool finished() __const__ { return m_finished; }

private:

    VideoInput();

    void initialize(__const__ std::string& filename, __const__ Settings& settings);

    static void decodingThreadProc(void* param);
    static void seekToTimestamp(VideoInput* vi, AVFrame* decodingFrame, AVPacket* packet, bool& validPacket);

    Settings m_settings;
    std::string m_filename;

    RealTime m_currentTime;
    int m_currentIndex;

    bool m_finished;

    struct Buffer {
        AVFrame* m_frame;
        RealTime m_pos;
        int64 m_timestamp;
    };

    Queue<Buffer*> m_decodedBuffers;
    Queue<Buffer*> m_emptyBuffers;
    GMutex m_bufferMutex;

    GThreadRef m_decodingThread;
    volatile bool m_quitThread;

    volatile bool m_clearBuffersAndSeek;
    int64 m_seekTimestamp;
    int64 m_lastTimestamp;

    // ffmpeg management
    AVFormatContext* m_avFormatContext;
    AVCodecContext* m_avCodecContext;
    AVCodec* m_avVideoCodec;
    int m_avVideoStreamIdx;
};

} // namespace G3D

