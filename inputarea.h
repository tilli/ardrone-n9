#ifndef INPUTAREA_H
#define INPUTAREA_H

// Debug
//#define PAINT_TOUCH_AREAS
//#define SHOW_FPS

#include <QWidget>
#include <QTextBrowser>

struct HUD_item
{
    HUD_item(){}
    HUD_item(QPixmap i, QPointF l, QRect r, qreal o)
    {
        image = i;
        location = l;
        touchArea = r;
        opacity = o;
    }

    QPixmap image;
    QPointF location;
    QRectF touchArea;
    qreal opacity;
};

class InputArea : public QWidget
{
    Q_OBJECT
public:
    explicit InputArea(QWidget *parent = 0);
    void setLoadText(QString s);
    void setFirmwareVersion(QString version);

signals:
    void emergencyPressed();
    void resetPressed();
    void startPressed();
    void stopPressed();
    void cameraPressed();
    void accelPressed();
    void accelReleased();
    void dataChange(qreal, qreal);

public slots:
    void recieveVideoImage(QImage image);
    void updateBatteryLevel(uint level);
    void updateEmergencyState(bool on);

private slots:
    void handleDataTimer();
    void handleFPSTimer();
    void handleNavdataTimer();

protected:
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    bool event(QEvent *event);

private:
    void generateHUD();
    void handleKeyPress(int key);

    enum HUD_ITEMS
    {
        HUD_EMERGENCY,
        HUD_START,
        HUD_SETTINGS,
        HUD_CAMERA,
        HUD_EXIT,
        HUD_BATTERY,
        HUD_ACCEL_BG,
        HUD_LAST_ITEM
    };

    bool mUIReady;
    bool mVideoImageReady;
    bool mUIVisible;
    QPixmap mVideoPixmap;
    QPixmap mSplashPixmap;
    int mAccelId;
    QPointF mAccelOffset;
    QPointF mAccelCenter;
    qreal mMaxAccelDistance;
    QPointF mOriginalAccelPos;
    QPointF mOriginalAccelBGPos;
    QPointF mOriginalAccelCenter;
    qreal mOriginalAccelOpacity;
    HUD_item mAccelItem;
    HUD_item mHudItems[HUD_LAST_ITEM];
    qreal mDefaultOpacities[HUD_LAST_ITEM];
    bool mEmergencyActivated;
    bool mIsStarted;
    bool mIsLoading;
    int mLoadDots;
    QString mLoadDotString;
    bool mNavdataTimedOut;
    bool mShowSettings;
    QTextBrowser *mTextAbout;
    QString mFirmwareVersion;
    QTimer *mDataTimer;
    QTimer *mFpsTimer;
#ifdef SHOW_FPS
    int mFpsCounter;
    QString mFpsString;
#endif
};

#endif // INPUTAREA_H
