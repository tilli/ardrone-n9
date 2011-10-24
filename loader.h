#ifndef LOADER_H
#define LOADER_H

#include <QWidget>
#include <QTimer>

class QHostInfo;
class QFtp;
class QFile;

namespace Ui {
    class Loader;
}

class Loader : public QWidget
{
    Q_OBJECT
public:
    explicit Loader(QWidget *parent = 0);
    ~Loader();

private:
    void setStatusLabelText(QString text);
    void setErrorLabelText(QString text, QString size = "10");
    void uploadFile();

signals:
    void noDroneConnection();
    void DroneConnected(QString version);

private slots:
    void handleLoaderTimer();
    void lookUpHost();
    void lookedUp(const QHostInfo &host);
    void readDroneFtp();
    void droneConnectionTimeout();
    void on_pushButtonExit_clicked();
    void on_pushButtonCurrent_clicked();

private:
    Ui::Loader *ui;
    QTimer mLoaderTimer;
    QTimer connectTimer;
    int mLoadDots;
    QFtp *mFtp;
    QString mVersion;
    bool mConnectedToDrone;
    uint mConnectState;
};

#endif // LOADER_H
