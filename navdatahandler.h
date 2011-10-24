#ifndef NAVDATAHANDLER_H
#define NAVDATAHANDLER_H
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QUdpSocket>
#include "navdata.h"

class NavDataHandler : public QThread
{
    Q_OBJECT
public:
    NavDataHandler(QObject *parent = 0);
    ~NavDataHandler();

protected:
    void run();

signals:
    void updateARDroneState(uint state);
    void updateBatteryLevel(uint level);
    void updateEmergencyState(bool on);

private slots:
    void newNavDataReady();

private:
    void initialize();

private:
    QMutex mMutex;
    QWaitCondition mCondition;
    bool mAbort;
    QUdpSocket *mUdpSocketNavData;
    QList<QByteArray> mDatagramList;
};

#endif // NAVDATAHANDLER_H
