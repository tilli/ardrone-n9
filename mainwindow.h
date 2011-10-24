#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Debug
//#define NO_DRONE_CONNECTION

// Features
#define USE_VIDEO_PROCESSOR


#include <QtGui/QMainWindow>

class Loader;
class InputArea;
class NavDataHandler;
class AtCommand;
class VideoProcessor;
class QFtp;
class QHostInfo;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    enum ScreenOrientation {
        ScreenOrientationLockPortrait,
        ScreenOrientationLockLandscape,
        ScreenOrientationAuto
    };

    explicit MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();

    void setOrientation(ScreenOrientation orientation);
    void showExpanded();

private slots:
    void loadInputArea(QString version);
    void noDroneConnection();

private:
    void initialize();

private:
    Loader *mLoader;
    InputArea *mInputArea;
    NavDataHandler *mNavDataHandler;
    AtCommand *mAtCommand;
#ifdef USE_VIDEO_PROCESSOR
    VideoProcessor *mVideoProcessor;
#endif
    bool mNoConnectionExit;
};

#endif // MAINWINDOW_H
