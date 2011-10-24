#include <QTimer>
#include <QHostInfo>
#include <QFtp>
#include <QFile>
#include <QFileDialog>
#include <QDebug>

#include "loader.h"
#include "ui_loader.h"
#include "navdata.h"

/*!
    Constructs the \c Loader.
    The \a parent parameter is sent to the QWidget constructor.
*/
Loader::Loader(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Loader)
{
    qDebug() << "[Loader] Constructor";
    // UI created with QtCreator's WYSIWYG editor.
    ui->setupUi(this);

    // Hide the buttons until they are needed.
    ui->pushButtonExit->setVisible(false);
    ui->labelError->setVisible(false);
    ui->pushButtonCurrent->setVisible(false);

    // Create a scene to place the background image on and add it to the
    // graphicsView object in the UI.
    QGraphicsScene *scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    scene->addPixmap( QPixmap(":/images/drone_bg.png"));

    mConnectState = 0;
    setStatusLabelText("Searching for AR.Drone<br />.");

    mLoaderTimer = new QTimer(this);
    connect(mLoaderTimer, SIGNAL(timeout()), this, SLOT(handleLoaderTimer()));
    mLoaderTimer->start(750);

    QTimer::singleShot(1, this, SLOT(lookUpHost()));
}

/*!
    Destructor.
*/
Loader::~Loader()
{
    delete ui;
}

/*!
    Changes the text of the status label in the UI.
    HTML formatting is added around the parameter \a text, before it is displayed.
*/
void Loader::setStatusLabelText(QString text)
{
    ui->labelStatus->setText(
            QString::fromUtf8("<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\""
                              " \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                              "<html><head><meta name=\"qrichtext\" content=\"1\" /><style "
                              "type=\"text/css\">\np, li { white-space: pre-wrap; }\n</style>"
                              "</head><body style=\" font-family:'MS Shell Dlg 2'; "
                              "font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; "
                              "margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span "
                              "style=\" font-size:6pt; color:#00007f;\">")
            + text
            + QString::fromUtf8("</span></p></body></html>"));
}

/*!
    Changes the text of the error label in the UI.
    HTML formatting is added around the parameter \a text, before it is displayed. The parameter
    \a size is the font size used. If \a size is not set a default value of 10 will be used.
*/
void Loader::setErrorLabelText(QString text, QString size)
{
    ui->labelError->setText(
            QString::fromUtf8("<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\""
                              " \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                              "<html><head><meta name=\"qrichtext\" content=\"1\" /><style "
                              "type=\"text/css\">\np, li { white-space: pre-wrap; }\n"
                              "</style></head><body style=\" font-family:'MS Shell Dlg 2'; "
                              "font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; "
                              "margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span "
                              "style=\" font-size:")
            + size
            + QString::fromUtf8("pt; color:#999999;\">")
            + text
            + QString::fromUtf8("</span></p></body></html>"));
}

/*!
    The loader timer updates the status message and indicates loading activity to the user.
*/
void Loader::handleLoaderTimer()
{
    mLoadDots++;
    mLoadDots = (mLoadDots%30) + 1;
    QString loadDotString = "";

    if (mConnectState == 0) {
        loadDotString  = "Searching for AR.Drone<br />";
    } else if (mConnectState == 1) {
        loadDotString = "WLAN connection found. Connecting to AR.Drone<br />";
    }

    for (int i = 0; i < mLoadDots; i++) {
        loadDotString += ".";
    }
    setStatusLabelText(loadDotString);
    update();
}

/*!
    Looks up the AR.Drone host to check if we are connected to the drone.
*/
void Loader::lookUpHost()
{
    QHostInfo::lookupHost(WIFI_MYKONOS_IP, this, SLOT(lookedUp(QHostInfo)));
}

/*!
    This function receives the host information from the host look up.
    The parameter \a host contains the information on the host to which we are connected.
*/
void Loader::lookedUp(const QHostInfo &host)
{
    if (host.localHostName() == "") {
        qDebug() << "[Loader] Host lookup failed:" << host.errorString();
        mLoaderTimer->stop();
        ui->labelStatus->setVisible(false);
        setErrorLabelText("Unable to connect to the AR.Drone.<br />Make sure the AR.Drone "
                          "is powered and that the phone is connected to the Drone through "
                          "WLAN. Then please restart the application.<br /><br />"
                          "Note that you need an AR.Drone quadrocopter for this app to be of any "
                          "use. See http://ardrone.parrot.com"
                          , "8");
        ui->labelError->setAutoFillBackground(true);
        ui->labelError->setVisible(true);
        ui->pushButtonExit->setVisible(true);
    } else {
        mConnectState = 1;
        mFtp = new QFtp(this);
        mFtp->connectToHost(WIFI_MYKONOS_IP, FTP_PORT);
        connect(mFtp, SIGNAL(readyRead()),this, SLOT(readDroneFtp()));

        mFtp->login("anonymous");
        mFtp->get("version.txt");
        mFtp->close();
        // Set a timeout timer in case FTP get fails.
        QTimer::singleShot(5000, this, SLOT(droneConnectionTimeout()));
    }
}

/*!
    This function is called when the FTP get returns with version.txt.
*/
void Loader::readDroneFtp()
{
    // FTP answer from the drone. This means we are connected!
    mConnectedToDrone = true;
    mVersion = mFtp->readAll();

    // This version of the app is made for firmware 1.4.7.
    if (mVersion == tr("1.4.7\n")) {
        emit DroneConnected(mVersion);
    } else {
        // The drone has a different firmware. Show arning text and give the user the choice between
        // exiting the app or continuing.
        mLoaderTimer->stop();
        ui->labelStatus->setVisible(false);
        setErrorLabelText("This program was made for AR.Drone firmware version 1.4.7.<br />The "
                          "current firmware version in the Drone is "
                          + mVersion
                          + ".<br /><br />We recommend that you update your Drone firmware to "
                          "version 1.4.7 but you can also continue with the current version.<br />"
                          "If you choose to continue with the current version we cannot guarantie "
                          "that the app and/or Drone will behave as expected.", "6");
        ui->labelError->setAutoFillBackground(true);
        ui->labelError->setVisible(true);
        ui->pushButtonExit->setVisible(true);
        ui->pushButtonCurrent->setVisible(true);
    }
}

/*!
    This function is called if the FTP get takes too long answering.
*/
void Loader::droneConnectionTimeout()
{
    // Check if we have a connection. Maybe the user is just waiting a long time before pressing
    // a button.
    if (!mConnectedToDrone) {
        qDebug() << "[ParrotDrone] FTP to drone timed out.";
        mLoaderTimer->stop();
        ui->labelStatus->setVisible(false);
        setErrorLabelText("Unable to retrieve version information from the AR.Drone.<br />Make sure"
                          " the AR.Drone is powered and that the phone is connected to the Drone "
                          "through WLAN. Then please restart the application.<br /><br />"
                          "Note that you need an AR.Drone quadrocopter for this app to be of any "
                          "use. See http://ardrone.parrot.com"
                          , "8");
        ui->labelError->setAutoFillBackground(true);
        ui->labelError->setVisible(true);
        ui->pushButtonExit->setVisible(true);
    }
}

/*!
    Called if user presses the Exit button. The Exit button is shown in case the drone has
    a firmware version that is different from version 1.4.7.
*/
void Loader::on_pushButtonExit_clicked()
{
    emit noDroneConnection();
}

/*!
    Called if user presses the continue button. The Continue button is shown in case the drone has
    a firmware version that is different from version 1.4.7.
*/
void Loader::on_pushButtonCurrent_clicked()
{
    emit DroneConnected(mVersion);
}
