#-------------------------------------------------
# AR.Drone
# This app enables you to fly your drone while 
# watching the video feed. The app was made for 
# fun and to give an example on how to create a 
# control app using Qt. Feel free to experiment
# with the code yourself. 
#-------------------------------------------------

QT += core gui network

TARGET = ARDrone
TEMPLATE = app
VERSION = 1.0.1

# Avoid auto screen rotation
DEFINES += ORIENTATIONLOCK

SOURCES += \
    main.cpp\
    mainwindow.cpp \
    atcommand.cpp \
    videoprocessor.cpp \
    navdatahandler.cpp \
    inputarea.cpp \
    loader.cpp

HEADERS += \
    mainwindow.h \
    navdata.h \
    vlib.h \
    video.h \
    atcommand.h \
    videoprocessor.h \
    navdatahandler.h \
    inputarea.h \
    about.h \
    loader.h

FORMS += \
    loader.ui

CONFIG += mobility
MOBILITY = sensors

symbian {
    TARGET.UID3 = 0xE7777777
    TARGET.CAPABILITY += NetworkServices
    LIBS += -lcone -leikcore -lavkon
    ICON = drone.svg
}

RESOURCES += \
    resources.qrc

OTHER_FILES += \
    ParrotCopyrightAndDisclaimer.txt \
    ParrotLicense.txt \
    qtc_packaging/debian_harmattan/rules \
    qtc_packaging/debian_harmattan/README \
    qtc_packaging/debian_harmattan/copyright \
    qtc_packaging/debian_harmattan/control \
    qtc_packaging/debian_harmattan/compat \
    qtc_packaging/debian_harmattan/changelog

contains(MEEGO_EDITION,harmattan) {
    target.path = /opt/ARDrone/bin
    desktop.files = ARDrone.desktop
    desktop.path = /usr/share/applications
    icon.files = ARDrone80.png
    icon.path = /usr/share/icons/hicolor/80x80/apps
    INSTALLS += target desktop icon
}
