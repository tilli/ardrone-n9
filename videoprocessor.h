#ifndef VIDEOPROCESSOR_H
#define VIDEOPROCESSOR_H

#define DRONE_VIDEO_MAX_WIDTH 640
#define DRONE_VIDEO_MAX_HEIGHT 480
#define IMAGE_BUFFER_SIZE 15

// Debug
//#define DEBUG_IMAGES_PER_SECOND

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QUdpSocket>
#include <QImage>

#include "navdata.h"
#include "vlib.h"

class QTimer;

class VideoProcessor : public QThread
{
    Q_OBJECT
public:
    VideoProcessor(QObject *parent = 0);
    ~VideoProcessor();

protected:
    void run();

signals:
    void newImageProcessed(QImage image);

private slots:
    void newVideoDataReady();
#ifdef DEBUG_IMAGES_PER_SECOND
    void handleIPSTimer();
#endif

private:
    void initialize();

private:
    QMutex mMutex;
    QWaitCondition mCondition;
    bool mAbort;
    QUdpSocket *mUdpSocketVideo;
    QImage mMyVideoImage;
    QList<QByteArray> mDatagramList;

#ifdef DEBUG_IMAGES_PER_SECOND
    QTimer *mIpsTimer;
    int mIpsCounter;
    int mDipsCounter;
#endif
};

#endif // VIDEOPROCESSOR_H
