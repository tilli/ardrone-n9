#include <QtGui>
#include <QImage>
#include <QDebug>

#include "inputarea.h"
#include "about.h"

/*!
    Constructs the \c InputArea object.
    The \a parent parameter is is sent to the QWidget constructor.
*/
InputArea::InputArea(QWidget *parent) :
    QWidget(parent)
{
    qDebug() << "[InputArea] Constructor";
    mUIReady = false;
    mUIVisible = false;
    mVideoImageReady = false;
    mSplashPixmap = QPixmap(":/images/drone_bg.png");
    mIsLoading = true;
    mLoadDots = 1;
    mLoadDotString = ".";
    mAccelId = -1;
    mAccelOffset = QPointF(0,0);
    mMaxAccelDistance = 0;
    mAccelCenter = QPointF(0,0);
    mOriginalAccelPos = QPointF(0,0);
    mOriginalAccelBGPos = QPointF(0,0);
    mOriginalAccelCenter = QPointF(0,0);
    mOriginalAccelOpacity = 0.15;
    mEmergencyActivated = false;
    mIsStarted = false;
    mDataTimer = new QTimer(this);
    connect(mDataTimer, SIGNAL(timeout()), this, SLOT(handleDataTimer()));
    mShowSettings = false;
    mFirmwareVersion = "N/A";
    mTextAbout = new QTextBrowser(this);
    mTextAbout->setObjectName(QString::fromUtf8("textAbout"));
    mTextAbout->setGeometry(QRect(75, 35, 490, 220));
    mTextAbout->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mTextAbout->setTextInteractionFlags(Qt::LinksAccessibleByKeyboard|Qt::LinksAccessibleByMouse);
    mTextAbout->setVisible(false);
    setAttribute(Qt::WA_AcceptTouchEvents);
    setAttribute(Qt::WA_StaticContents);
#ifdef SHOW_FPS
    mFpsCounter = 0;
#endif
    mFpsTimer = new QTimer(this);
    connect(mFpsTimer, SIGNAL(timeout()), this, SLOT(handleFPSTimer()));
    mFpsTimer->start(1000);
    mNavdataTimedOut = false;
    QTimer::singleShot(8000, this, SLOT(handleNavdataTimer()));
}

/*!
    Function for setting the firmware version variable.
    The \a version parameter contains the firmware version.
*/
void InputArea::setFirmwareVersion(QString version)
{
    mFirmwareVersion = version;
}

/*!
    This function paints the UI.
    The \a event parameter is unused.
*/
void InputArea::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setOpacity(1.0);
    if (mVideoImageReady && mUIReady && mUIVisible) {
        // The drone has 2 cameras, with different resolutions.
        if (mVideoPixmap.height() == 240) {
            painter.drawPixmap(0, 0, 854, 480, mVideoPixmap);
        } else if (mVideoPixmap.height() == 144) {
            painter.drawPixmap(0, 0, 854, 480, mVideoPixmap, 0, 0, 176, 144);
        }
    } else {
        // We are either still loading or in the settings screen.
        painter.drawPixmap(0, 0, width(), height(), mSplashPixmap);
        if (mNavdataTimedOut && !mShowSettings) {
            // Timeout. Drone has taken too long in sending the first status data.
            painter.setPen(Qt::black);
            painter.drawText(0, 0, width(), 35, Qt::AlignBottom | Qt::AlignHCenter,
                             "Error: No response from Drone");
        } else if (mIsLoading) {
            // Still waiting to receive the first status data from the drone.
            painter.setPen(Qt::black);
            painter.drawText(0, 0, width(), 45, Qt::AlignBottom | Qt::AlignHCenter, mLoadDotString);
            painter.drawText(0, 0, width(), 35, Qt::AlignBottom | Qt::AlignHCenter,
                             "Connected to Drone. Setting up communication");
        }
    }

    if (mUIReady && mUIVisible) {
#ifdef PAINT_TOUCH_AREAS
        // Paint touch areas.
        for (int i = 0; i < HUD_BATTERY; i++) {
            painter.setOpacity(0.2);
            painter.fillRect(mHudItems[i].touchArea, Qt::black);
        }
#endif
        // Paint UI components except accel background. It is the last item in the list.
        for (int i = 0; i < HUD_LAST_ITEM-1; i++) {
            painter.setOpacity(mHudItems[i].opacity);
            painter.drawPixmap(mHudItems[i].location, mHudItems[i].image);
        }

        // Paint the accel/joystick
        painter.setOpacity(mAccelItem.opacity);
        QPointF l2 = mAccelItem.location;
        QVector2D v2 = QVector2D(mAccelCenter-l2);
        if (v2.length() > mMaxAccelDistance) {
            l2 = mAccelCenter - ((v2.normalized()).toPointF()*mMaxAccelDistance);
        }
        painter.drawPixmap(l2+mAccelOffset, mAccelItem.image);
        painter.drawPixmap(mHudItems[HUD_ACCEL_BG].location, mHudItems[HUD_ACCEL_BG].image);

#ifdef SHOW_FPS
        painter.setPen(Qt::black);
        painter.setOpacity(1.0);
        painter.drawText(width()-50, height()-10, mFpsString);
#endif
    } else if (mShowSettings) {
#ifdef PAINT_TOUCH_AREAS
        // Paint touch area.
        painter.setOpacity(0.2);
        painter.fillRect(mHudItems[HUD_SETTINGS].touchArea, Qt::black);
#endif
        painter.setOpacity(mHudItems[HUD_SETTINGS].opacity);
        painter.drawPixmap(mHudItems[HUD_SETTINGS].location, mHudItems[HUD_SETTINGS].image);
    }

#ifdef SHOW_FPS
    mFpsCounter++;
#endif
}

/*!
    Called whenever the application is resized.
    The parameter \a event is passed on to QWidget.
*/
void InputArea::resizeEvent(QResizeEvent *event)
{
    generateHUD();
    QWidget::resizeEvent(event);
}

/*!
    This function takes care of the user input logic.
    If the parameter \a event is a touch event it is consumed. If not it is passed on to QWidget.
*/
bool InputArea::event(QEvent *event)
{
    if ((mUIReady && mUIVisible) || mShowSettings) {
        // Only update the UI if something has changed.
        bool updateUI = false;
        switch (event->type()) {
        case QEvent::TouchBegin:
        case QEvent::TouchUpdate:
        case QEvent::TouchEnd:
        {
            QList<QTouchEvent::TouchPoint> touchPoints =
                    static_cast<QTouchEvent *>(event)->touchPoints();
            foreach (const QTouchEvent::TouchPoint &touchPoint, touchPoints) {
                switch (touchPoint.state()) {
                case Qt::TouchPointStationary:
                {
                    // Don't do anything if this touch point hasn't moved.
                    break;
                }
                case Qt::TouchPointPressed:
                {
                    bool buttonFound = false;
                    if (!mShowSettings) {
                        // We are in the UI screen.
                        // Check if user has pressed an area containing a button. HUD_BATTERY is the
                        // first object in the list that is not a button.
                        for (int i = 0; i < HUD_BATTERY; i++) {
                            if (mHudItems[i].touchArea.contains(touchPoint.startPos())) {
                                mHudItems[i].opacity = 1.0;
                                buttonFound = true;
                            }
                        }
                        if (!buttonFound) {
                            // Only activate joystick if it is not already active.
                            if (mAccelId == -1) {
                                mAccelId = touchPoint.id();
                                mHudItems[HUD_ACCEL_BG].location =
                                        QPoint(touchPoint.startPos().x()
                                               -(mHudItems[HUD_ACCEL_BG].image.width()/2),
                                               touchPoint.startPos().y()
                                               -(mHudItems[HUD_ACCEL_BG].image.height()/2));
                                mHudItems[HUD_ACCEL_BG].opacity = 1.0;
                                mAccelCenter = touchPoint.startPos();
                                mAccelItem.location = touchPoint.startPos();
                                mAccelItem.opacity = 1.0;
                                // Start sending data to AtCommand every 66 milliseconds.
                                mDataTimer->start(66);
                                emit accelPressed();
                            }
                        }
                    } else {
                        // We are in the settings screen. Here we only have the settings button to
                        // check for.
                        if (mHudItems[HUD_SETTINGS].touchArea.contains(touchPoint.startPos())) {
                            mHudItems[HUD_SETTINGS].opacity = 1.0;
                        }
                    }
                    updateUI = true;
                    break;
                }
                case Qt::TouchPointMoved:
                {
                    if (!mShowSettings) {
                        // We are in the UI screen.
                        if (touchPoint.id() == mAccelId) {
                            mAccelItem.location = touchPoint.pos();
                        } else {
                            // HUD_BATTERY is the first object in the list that is not a button.
                            for (int i = 0; i < HUD_BATTERY; i++) {
                                if (mHudItems[i].touchArea.contains(touchPoint.startPos())) {
                                    if (mHudItems[i].touchArea.contains(touchPoint.pos())) {
                                        mHudItems[i].opacity = 1.0;
                                    } else {
                                        mHudItems[i].opacity = mDefaultOpacities[i];
                                    }
                                }
                            }
                        }
                    } else {
                        // We are in the settings screen.
                        if (mHudItems[HUD_SETTINGS].touchArea.contains(touchPoint.startPos())) {
                            if (mHudItems[HUD_SETTINGS].touchArea.contains(touchPoint.pos())) {
                                mHudItems[HUD_SETTINGS].opacity = 1.0;
                            } else {
                                mHudItems[HUD_SETTINGS].opacity = mDefaultOpacities[HUD_SETTINGS];
                            }
                        }
                    }
                    updateUI = true;
                    break;
                }
                case Qt::TouchPointReleased:
                {
                    if (!mShowSettings) {
                        // We are in the UI screen.
                        if (touchPoint.id() == mAccelId) {
                            emit accelReleased();
                            mDataTimer->stop();
                            mAccelId = -1;
                            mHudItems[HUD_ACCEL_BG].location = mOriginalAccelBGPos;
                            mHudItems[HUD_ACCEL_BG].opacity = mDefaultOpacities[HUD_ACCEL_BG];
                            mAccelItem.location = mOriginalAccelPos;
                            mAccelItem.opacity = mOriginalAccelOpacity;
                            mAccelCenter = mOriginalAccelCenter;
                        } else {
                            // HUD_BATTERY is the first non-touchable object in the list
                            for (int i = 0; i < HUD_BATTERY; i++) {
                                if (mHudItems[i].touchArea.contains(touchPoint.startPos())
                                    && mHudItems[i].touchArea.contains(touchPoint.lastPos())) {
                                    handleKeyPress(i);
                                }
                            }
                        }
                    } else {
                        // We are in the settings screen.
                        if (mHudItems[HUD_SETTINGS].touchArea.contains(touchPoint.startPos())
                            && mHudItems[HUD_SETTINGS].touchArea.contains(touchPoint.lastPos())) {
                            handleKeyPress(HUD_SETTINGS);
                        }
                    }
                    updateUI = true;
                    break;
                }
                default:
                    break;
                }
            }
            break;
        }
        default:
            return QWidget::event(event);
        }

        if (updateUI) {
            update();
        }
    } else {
        return QWidget::event(event);
    }
    return true;
}

/*!
    This function calculates the positions of the different UI elements.
*/
void InputArea::generateHUD()
{
    mUIReady = false;

    QPixmap pm;
    QPointF l;
    QRect r;

    pm = QPixmap(":/images/emergency.png");
    l = QPointF((width()/2) - (pm.width()/2), 0);
    r = QRect(l.x()-10, 0, pm.width()+20, pm.height()+10);
    mHudItems[HUD_EMERGENCY] = HUD_item(pm, l, r, 0.75);
    mDefaultOpacities[HUD_EMERGENCY] = 0.75;

    pm = QPixmap(":/images/take_off.png");
    l = QPointF((width()/2) - (pm.width()/2), height()-pm.height());
    r = QRect(l.x()-10, l.y()-10, pm.width()+20, pm.height()+10);
    mHudItems[HUD_START] = HUD_item(pm, l, r, 0.75);
    mDefaultOpacities[HUD_START] = 0.75;

    pm = QPixmap(":/images/settings.png");
    l = QPointF(0, 45);
    r = QRect(0, l.y()-5, pm.width()+5, pm.height()+10);
    mHudItems[HUD_SETTINGS] = HUD_item(pm, l, r, 0.5);
    mDefaultOpacities[HUD_SETTINGS] = 0.5;

    pm = QPixmap(":/images/switch.png");
    l = QPointF(0, mHudItems[HUD_SETTINGS].location.y()+mHudItems[HUD_SETTINGS].image.height()+15);
    r = QRect(0, l.y()-5, pm.width()+5, pm.height()+10);
    mHudItems[HUD_CAMERA] = HUD_item(pm, l, r, 0.5);
    mDefaultOpacities[HUD_CAMERA] = 0.5;

    pm = QPixmap(":/images/button-close.png");
    l = QPointF(width() - pm.width() - 10, 10);
    r = QRect(l.x() - 10, l.y() - 10, pm.width()+20, pm.height()+20);
    mHudItems[HUD_EXIT] = HUD_item(pm, l, r, 0.5);
    mDefaultOpacities[HUD_EXIT] = 0.5;

    pm = QPixmap(":/images/battery0.png");
    l = QPointF(5, 0);
    r = QRect(0, 0, 1, 1);
    mHudItems[HUD_BATTERY] = HUD_item(pm, l, r, 0.25);
    mDefaultOpacities[HUD_BATTERY] = 1.0;

    pm = QPixmap(":/images/joystick_background.png");
    pm = pm.scaled(pm.width()*2, pm.height()*2);
    l = QPointF((width()/4)*3 - pm.width()/2, (height()/8)*5 - pm.height()/2);
    r = QRect(0, 0, 1, 1);
    mHudItems[HUD_ACCEL_BG] = HUD_item(pm, l, r, 0.25);
    mOriginalAccelBGPos = l;
    mDefaultOpacities[HUD_ACCEL_BG] = 0.25;

    pm = QPixmap(":/images/joystick_thumb.png");
    l = QPointF((width()/4)*3, (height()/8)*5);
    r = QRect((width()/4)*3-pm.width()/2, (height()/8)*5-pm.height()/2, pm.width(), pm.height());
    mAccelItem = HUD_item(pm, l, r, 0.25);
    mAccelOffset = QPointF(-(pm.width()/2), -(pm.height()/2));
    mAccelCenter = l;
    mMaxAccelDistance = (mHudItems[HUD_ACCEL_BG].image.width()/2) - (pm.width()/2) + 12;
    mOriginalAccelPos = l;
    mOriginalAccelCenter = mAccelCenter;
    mOriginalAccelOpacity = 0.25;

    mUIReady = true;
}

/*!
    Slot for receiving a new image from the video feed.
    The parameter \a image contains the new image.
*/
void InputArea::recieveVideoImage(QImage image)
{
    // Convert and save as QPixmap, since this is more efficient when drawing with QPainter.
    mVideoPixmap = QPixmap::fromImage(image);
    mVideoImageReady = true;
    update();
}

/*!
    Handles when user has pressed an area that contains a button.
    The parameter \a key is the button that has been pressed.
*/
void InputArea::handleKeyPress(int key)
{
    switch(key){
    case HUD_EMERGENCY:
    {
        // The emergency button is used for both Emergency and Reset, so we need to check if
        // Emergency is already activated.
        if (mEmergencyActivated) {
            emit resetPressed();
            mHudItems[HUD_EMERGENCY].image = QPixmap(":/images/emergency.png");
        } else {
            emit emergencyPressed();
            mHudItems[HUD_EMERGENCY].image = QPixmap(":/images/reset.png");
            mHudItems[HUD_START].image = QPixmap(":/images/take_off.png");
            mIsStarted = false;
        }
        mHudItems[HUD_EMERGENCY].opacity = mDefaultOpacities[HUD_EMERGENCY];
        mEmergencyActivated = !mEmergencyActivated;
        break;
    }
    case HUD_START:
    {
        // The start button is used for both Take-off and Land, so we need to check if we are
        // already flying.
        if (mIsStarted) {
            emit stopPressed();
            mHudItems[HUD_START].image = QPixmap(":/images/take_off.png");
        } else {
            emit startPressed();
            mHudItems[HUD_START].image = QPixmap(":/images/take_land.png");
        }
        mHudItems[HUD_START].opacity = mDefaultOpacities[HUD_START];
        mIsStarted = !mIsStarted;
        break;
    }
    case HUD_SETTINGS:
    {
        // The settings button is used in both the UI and settings screen, so we toggle which should
        // be visible.
        mUIVisible = !mUIVisible;
        mShowSettings = !mShowSettings;

        mTextAbout->setHtml(QString::fromUtf8("<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\""
                                              " \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                              "<html><head><meta name=\"qrichtext\" content=\"1\""
                                              " /><style type=\"text/css\">\n"
                                              "p, li { white-space: pre-wrap; }\n"
                                              "</style></head><body style=\" font-family:'MS Shell "
                                              "Dlg 2'; font-size:8.25pt; font-weight:400; "
                                              "font-style:normal;\">\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; "
                                              "margin-left:0px; margin-right:0px; -qt-block-indent:"
                                              "0; text-indent:0px;\"><span style=\" font-size:6pt;"
                                              "\">")
                            + mClientVersionString
                            + QString::fromUtf8("<br />A.R. Drone firmware version: ")
                            + mFirmwareVersion + QString::fromUtf8("<br /><br />")
                            + mAboutString + QString::fromUtf8("<br /><br />")
                            + mNokiaString + QString::fromUtf8("<br />")
                            + mParrotString + QString::fromUtf8("</span></p></body></html>"));

        mTextAbout->setVisible(mShowSettings);
        mHudItems[HUD_SETTINGS].opacity = mDefaultOpacities[HUD_SETTINGS];
        update();
        break;
    }
    case HUD_CAMERA:
    {
        emit cameraPressed();
        mHudItems[HUD_CAMERA].opacity = mDefaultOpacities[HUD_CAMERA];
        break;
    }
    case HUD_EXIT:
    {
        exit(0);
        break;
    }
    default:
        break;
    }
}

/*!
    Handles the sending of new rotation and altitude change commands.
    Triggered by a timer, to limit the amount of signals passed to \c AtCommand.
*/
void InputArea::handleDataTimer()
{
    qreal h = (mAccelCenter.y() - mAccelItem.location.y())/mMaxAccelDistance;
    if (h < 0.3 && h > -0.3) {
        h = 0.0;
    } else if (h > 1.0) {
        h = 1.0;
    } else if (h < -1.0) {
        h = -1.0;
    }

    qreal d = (mAccelItem.location.x() - mAccelCenter.x())/mMaxAccelDistance;
    if (d < 0.3 && d > -0.3) {
        d = 0.0;
    } else if (d > 1.0) {
        d = 1.0;
    } else if (d < -1.0) {
        d = -1.0;
    }

    emit dataChange(h, d);
}

/*!
    Handles the Frames Per Second timer.
    Is also used to print loading activity and to keep backlight on.
    This is a 1 second timer.
*/
void InputArea::handleFPSTimer()
{
#ifdef SHOW_FPS
    mFpsString = QString::number(mFpsCounter) + "FPS";
    mFpsCounter = 0;
#endif

    // Change the number of dots that show loading activity.
    if (mIsLoading) {
        mLoadDots++;
        mLoadDots = (mLoadDots%20) + 1;
        mLoadDotString = "";
        for (int i = 0; i < mLoadDots; i++) {
            mLoadDotString += ".";
        }
    }

#ifdef Q_OS_SYMBIAN
    User::ResetInactivityTime(); // Make sure that the backlight is kept on
#endif
    update();
}

/*!
    Called when the status data timer times out.
    If this timer runs out before we have received the first status data from the drone we assume
    that there is something wrong with the connection.
*/
void InputArea::handleNavdataTimer()
{
    mNavdataTimedOut = true;
}

/*!
    Slot for receiving battery level status from \c NavDataHandler.
    The parameter \a level contains the new battery status.
    The first time battery status is received from the drone is also taken as an indication that
    communication has been successfully established.
*/
void InputArea::updateBatteryLevel(uint level)
{
    if (level == 3) {
        mHudItems[HUD_BATTERY].image = QPixmap(":/images/battery3.png");
    } else if (level == 2) {
        mHudItems[HUD_BATTERY].image = QPixmap(":/images/battery2.png");
    } else if (level == 1) {
        mHudItems[HUD_BATTERY].image = QPixmap(":/images/battery1.png");
    } else if (level == 0) {
        mHudItems[HUD_BATTERY].image = QPixmap(":/images/battery0.png");
    }
    mHudItems[HUD_BATTERY].opacity = mDefaultOpacities[HUD_BATTERY];
    mIsLoading = false;
    mUIVisible = true;
    update();
}

/*!
    Slot to receive updated emergency status from \c NavDataHandler.
    The parameter \a on tells wether emergency state is active or not.
*/
void InputArea::updateEmergencyState(bool on)
{
    // Change button graphics and functionality if drone status is different from the current UI.
    if (on && !mEmergencyActivated) {
        mHudItems[HUD_EMERGENCY].image = QPixmap(":/images/reset.png");
        mEmergencyActivated = true;
        mHudItems[HUD_START].image = QPixmap(":/images/take_off.png");
        mIsStarted = false;
    } else if (!on && mEmergencyActivated) {
        mHudItems[HUD_EMERGENCY].image = QPixmap(":/images/emergency.png");
        mEmergencyActivated = false;
    }
    update();
}

