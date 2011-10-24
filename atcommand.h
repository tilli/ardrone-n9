#ifndef ATCOMMAND_H
#define ATCOMMAND_H
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QTimer>
#include <QAccelerometer>

#ifndef NUM_STR
    #define NUM_STR(x) QByteArray::number(x)
#endif

#define ARDRONE_NO_TRIM			\
    ((1 << ARDRONE_UI_BIT_TRIM_THETA) |	\
     (1 << ARDRONE_UI_BIT_TRIM_PHI) |	\
     (1 << ARDRONE_UI_BIT_TRIM_YAW) |	\
     (1 << ARDRONE_UI_BIT_X) |		\
     (1 << ARDRONE_UI_BIT_Y))

// Debug
//#define PRINT_INPUT_PER_SECOND

QTM_USE_NAMESPACE

class AtCommand : public QThread
{
    Q_OBJECT

public:
    AtCommand(QObject *parent = 0);
    ~AtCommand();

    void startThread();

protected:
    void run();

public slots:
    void setData(qreal gaz, qreal yaw);
    void changeCamera();
    void updateARDroneState(uint state);
    void emergencyPressed();
    void resetPressed();
    void startPressed();
    void stopPressed();
    void accelPressed();
    void accelReleased();

private slots:
    void accelFilter();
#ifdef PRINT_INPUT_PER_SECOND
    void handleIPSTimer();
#endif

private:
    void initialize();

private:
    QMutex mMutex;
    QWaitCondition mCondition;
    bool mAbort;
    uint mARDroneState;
    int mPhi;
    int mTheta;
    int mGaz;
    int mYaw;
    bool mCameraChange;
    uint mUserInput;
    QAccelerometer *mAccelSensor;
    bool mAccel;

#ifdef PRINT_INPUT_PER_SECOND
    QTimer *mIpsTimer;
    int mIpsMovCounter;
    int mIpsDataCounter;
    int mIpsSendCounter;
#endif

};

#endif // ATCOMMAND_H
