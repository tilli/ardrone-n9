#include <QtCore/QCoreApplication>
#include <QDebug>

#if defined(Q_OS_SYMBIAN) && defined(ORIENTATIONLOCK)
#include <eikenv.h>
#include <eikappui.h>
#include <aknenv.h>
#include <aknappui.h>
#endif // Q_OS_SYMBIAN && ORIENTATIONLOCK

#include "mainwindow.h"
#include "loader.h"
#include "inputarea.h"
#include "navdatahandler.h"
#include "atcommand.h"

#ifdef USE_VIDEO_PROCESSOR
#include "videoprocessor.h"
#endif

/*!
    Constructs the \c MainWindow.
    The \a parent parameter is sent to the QMainWindow constructor.
*/
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    qDebug() << "[ARDrone] Constructor";
    mNoConnectionExit = true;

#ifndef NO_DRONE_CONNECTION
    mLoader = new Loader(this);
    setCentralWidget(mLoader);
    connect(mLoader, SIGNAL(noDroneConnection()), this, SLOT(noDroneConnection()));
    connect(mLoader, SIGNAL(DroneConnected(QString)), this, SLOT(loadInputArea(QString)));
#else
    loadInputArea("0.0.0");
#endif
}

/*!
    Destructor.
*/
MainWindow::~MainWindow()
{
    // If we are exiting before we got a connection to the drone then the following components will
    // not have been loaded yet.
    if (!mNoConnectionExit) {
        delete mInputArea;
        delete mNavDataHandler;
        delete mAtCommand;
#ifdef USE_VIDEO_PROCESSOR
        delete mVideoProcessor;
#endif
    }
}

/*!
    This function is used to lock the app orientation to landscape.
*/
void MainWindow::setOrientation(ScreenOrientation orientation)
{
#ifdef Q_OS_SYMBIAN
    if (orientation != ScreenOrientationAuto) {
#if defined(ORIENTATIONLOCK)
        const CAknAppUiBase::TAppUiOrientation uiOrientation =
                (orientation == ScreenOrientationLockPortrait) ? CAknAppUi::EAppUiOrientationPortrait
                    : CAknAppUi::EAppUiOrientationLandscape;
        CAknAppUi* appUi = dynamic_cast<CAknAppUi*> (CEikonEnv::Static()->AppUi());
        TRAPD(error,
            if (appUi)
                appUi->SetOrientationL(uiOrientation);
        );
        Q_UNUSED(error)
#else // ORIENTATIONLOCK
        qWarning("'ORIENTATIONLOCK' needs to be defined on Symbian when locking the orientation.");
#endif // ORIENTATIONLOCK
    }
#elif defined(Q_WS_MAEMO_5)
    Qt::WidgetAttribute attribute;
    switch (orientation) {
    case ScreenOrientationLockPortrait:
        attribute = Qt::WA_Maemo5PortraitOrientation;
        break;
    case ScreenOrientationLockLandscape:
        attribute = Qt::WA_Maemo5LandscapeOrientation;
        break;
    case ScreenOrientationAuto:
    default:
        attribute = Qt::WA_Maemo5AutoOrientation;
        break;
    }
    setAttribute(attribute, true);
#else // Q_OS_SYMBIAN
    Q_UNUSED(orientation);
#endif // Q_OS_SYMBIAN
}

/*!
    This function will show the app in fullscreen.
*/
void MainWindow::showExpanded()
{
#ifdef Q_OS_SYMBIAN
    showFullScreen();
#elif defined(Q_WS_MAEMO_5) || defined(Q_WS_MAEMO_6)
    showMaximized();
#else
    show();
#endif
}

/*!
    Slot that will load all the different components once the Loader signals that connection to the
    drone has been confirmed.
*/
void MainWindow::loadInputArea(QString version)
{
    mInputArea = new InputArea(this);
    setCentralWidget(mInputArea);
    mInputArea->setFirmwareVersion(version);
    initialize();
}

/*!
    Slot called from the Loader in case it could not connect to the drone.
*/
void MainWindow::noDroneConnection()
{
    this->close();
}

/*!
    This function loads all the communication components and sets up signals between them.
*/
void MainWindow::initialize()
{
    qDebug() << "[ARDrone] initializing";
    mNavDataHandler = new NavDataHandler();
    mAtCommand = new AtCommand();

#ifdef USE_VIDEO_PROCESSOR
    mVideoProcessor = new VideoProcessor();
    qRegisterMetaType<QImage>("QImage");
    connect(mVideoProcessor, SIGNAL(newImageProcessed(QImage)),
            mInputArea, SLOT(recieveVideoImage(QImage)));
#endif

    connect(mNavDataHandler, SIGNAL(updateARDroneState(uint)),
            mAtCommand, SLOT(updateARDroneState(uint)));
    connect(mNavDataHandler, SIGNAL(updateBatteryLevel(uint)),
            mInputArea, SLOT(updateBatteryLevel(uint)));
    connect(mNavDataHandler, SIGNAL(updateEmergencyState(bool)),
            mInputArea, SLOT(updateEmergencyState(bool)));

    connect(mInputArea, SIGNAL(emergencyPressed()), mAtCommand, SLOT(emergencyPressed()));
    connect(mInputArea, SIGNAL(resetPressed()), mAtCommand, SLOT(resetPressed()));
    connect(mInputArea, SIGNAL(startPressed()), mAtCommand, SLOT(startPressed()));
    connect(mInputArea, SIGNAL(stopPressed()), mAtCommand, SLOT(stopPressed()));
    connect(mInputArea, SIGNAL(cameraPressed()), mAtCommand, SLOT(changeCamera()));
    connect(mInputArea, SIGNAL(dataChange(qreal,qreal)), mAtCommand, SLOT(setData(qreal,qreal)));
    connect(mInputArea, SIGNAL(accelPressed()), mAtCommand, SLOT(accelPressed()));
    connect(mInputArea, SIGNAL(accelReleased()), mAtCommand, SLOT(accelReleased()));

    mAtCommand->startThread();

    mNoConnectionExit = false;

#ifdef NO_DRONE_CONNECTION
    // Updating the battery level will make the UI visible.
    mInputArea->updateBatteryLevel(0);
#endif
}
