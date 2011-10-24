#include <QTimer>
#include <QPixmap>
#include "videoprocessor.h"
#include "video.h"

/*!
    Constructs the VideoProcessor.
    The \a parent parameter is sent to the QThread constructor.
*/
VideoProcessor::VideoProcessor(QObject *parent) : QThread(parent)
{
    qDebug() << "[VideoProcessor] Constructor";
    mAbort = false;
    initialize();
}

/*!
    Destructor.
    Waits for thread to abort before destroying the \c VideoProcessor object.
*/
VideoProcessor::~VideoProcessor()
{
    qDebug() << "[VideoProcessor] Destructor";
    mMutex.lock();
    mAbort = true;
    mCondition.wakeOne();
    mMutex.unlock();
    wait();
}

/*!
    This function sets up a UDP socket and sends a message to the drone that we are ready to
    receive the video feed.
*/
void VideoProcessor::initialize()
{
    qDebug() << "[VideoProcessor] Initialize";

    QMutexLocker locker(&mMutex);

    // UDP socket for receiving video data from the drone.
    mUdpSocketVideo = new QUdpSocket(this);
    bool res=mUdpSocketVideo->bind(VIDEO_PORT, QUdpSocket::ShareAddress);
    if (!res) {
        qDebug() << "[VideoProcessor] Error connecting to video. result: " << res;
    } else {
        connect(mUdpSocketVideo, SIGNAL(readyRead()), this, SLOT(newVideoDataReady()),
                Qt::QueuedConnection);
    }

    // Send message to drone to signal that we are ready to receive the video data.
    QHostAddress reciever;
    int res2;
    const char data[] = "Init";
    reciever=QHostAddress::QHostAddress(WIFI_MYKONOS_IP);
    res2=mUdpSocketVideo->writeDatagram(data,reciever, VIDEO_PORT);

#ifdef DEBUG_IMAGES_PER_SECOND
    mDipsCounter = 0;
    mIpsCounter = 0;
    mIpsTimer = new QTimer(this);
    connect(mIpsTimer, SIGNAL(timeout()), this, SLOT(handleIPSTimer()));
    mIpsTimer->start(1000);
#endif
}

/*!
    Thread main function.
    Reads video data received from the drone and convert it into a QImage.
*/
void VideoProcessor::run()
{
    qDebug() << "[VideoProcessor] run";

    video_controller_t controller;
    memset(&picture, 0, sizeof(picture));

    picture.format        = PIX_FMT_RGB565;
    picture.framerate     = 15;
    picture.width         = DRONE_VIDEO_MAX_WIDTH;
    picture.height        = DRONE_VIDEO_MAX_HEIGHT;
    picture.y_buf         = (uint8_t*)malloc(DRONE_VIDEO_MAX_WIDTH * DRONE_VIDEO_MAX_HEIGHT);
    picture.y_line_size   = DRONE_VIDEO_MAX_WIDTH;

    memset(&controller, 0, sizeof(controller));

    if (video_codec_open( &controller, UVLC_CODEC )) {
          qDebug() << "[VideoProcessor] video_codec_open() failed";
    }

    video_controller_set_motion_estimation(&controller, FALSE);
    video_controller_set_format(&controller, H_ACQ_WIDTH, H_ACQ_HEIGHT);

    forever {
        // Do the stuff but check if we need to abort first...
        if (mAbort) {
            return;
        }

        QByteArray datagram;

        // Copy the next datagram in the list to a variable that is local to the thread.
        mMutex.lock();
        if (!mDatagramList.empty()) {
            datagram = mDatagramList.takeFirst();
        }
        mMutex.unlock();

        int size=datagram.size();

        if (size > 0) {
            C_RESULT status=0;
            int decodeOK = FALSE;

            controller.in_stream.bytes = (uint32_t*) datagram.data();
            controller.in_stream.used	= size;
            controller.in_stream.size	= size;
            controller.in_stream.index	= 0;
            controller.in_stream.length	= 32;
            controller.in_stream.code	= 0;

            status = video_decode_blockline(&controller, &picture, &decodeOK);
            if (status) {
                    qDebug() << "[VideoProcessor] video_decode_blockline() failed";
            } else if (decodeOK) {
                // There are 2 cameras with different resolutions on the drone.
                if (controller.width == 320) {
                    mMutex.lock();
                    // Crop image by removing the top and bottom 30 lines.
                    mMyVideoImage=QImage(  &picture.y_buf[0], 320, 240, QImage::Format_RGB16 );
                    emit(newImageProcessed(this->mMyVideoImage));
                    mMutex.unlock();
                } else if (controller.width == 176) {
                    mMutex.lock();
                    // Decoder is dependent on a line length of 320.
                    mMyVideoImage=QImage(  &picture.y_buf[0], 320, 144, QImage::Format_RGB16 );
                    emit(newImageProcessed(this->mMyVideoImage));
                    mMutex.unlock();
                } else {
                    qDebug() << "[VideoProcessor] Image decoded but size doesn't correspond to any "
                             << "known camera. Width x Height: "
                             << controller.width << "x" << controller.height;
                }
            }
        }

#ifdef DEBUG_IMAGES_PER_SECOND
        mIpsCounter++;
#endif

        // If we have more video data in queue, then wait 1 millisecond before continuing.
        // Else wait until new data arrives.
        mMutex.lock();
        if (mDatagramList.count() > 0) {
            mCondition.wait(&mMutex, 1);
        } else {
            mCondition.wait(&mMutex);
        }
        mMutex.unlock();
    }
}

/*!
    Slot called whenever new video data is ready from the drone.
    Starts the thread with LowestPriority unless it is already running.
*/
void VideoProcessor::newVideoDataReady()
{
#ifdef DEBUG_IMAGES_PER_SECOND
    mDipsCounter++;
#endif

    int size = mUdpSocketVideo->pendingDatagramSize();
    QByteArray datagram;
    datagram.resize(size);
    mUdpSocketVideo->readDatagram(datagram.data(), size);

    mMutex.lock();
    if (mDatagramList.count() < IMAGE_BUFFER_SIZE) {
        mDatagramList.append(datagram);
        if (!isRunning()) {
            start(LowestPriority);
        } else {
            mCondition.wakeOne();
        }
    } else {
        qDebug() << "[VideoProcessor] Image buffer full. Discarding this image.";
    }
    mMutex.unlock();
}

#ifdef DEBUG_IMAGES_PER_SECOND
/*!
    Prints debug output at a timed interval.
*/
void VideoProcessor::handleIPSTimer()
{
    qDebug() << "[VideoProcessor] Images per second: From Drone: " << QString::number(mIpsCounter)
             << ". Parsed: " << QString::number(mDipsCounter);
    mIpsCounter = 0;
    mDipsCounter = 0;
}
#endif
