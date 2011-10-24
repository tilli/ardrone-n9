#ifndef ABOUT_H
#define ABOUT_H

#include <QString>

QString mClientVersion = "1.0.1";

#ifdef Q_OS_SYMBIAN
QString mClientVersionString = "Symbian client version: " + mClientVersion;
#elif defined(Q_WS_MAEMO_5) || defined(Q_WS_MAEMO_6)
QString mClientVersionString = "Maemo client version: " + mClientVersion;
#else
QString mClientVersionString = "Client version: " + mClientVersion;
#endif

QString mAboutString = "Qt application for doing basic control of Parrot A.R. Drone "
                       "(http://ardrone.parrot.com). Developed for fun and joy.<br />"
                       "Feel free to improve or change by installing Qt development tools from "
                       "http://qt.nokia.com and find the source code for this application on "
                       "http://forum.nokia.com";

QString mNokiaString = "Copyright (C) 2011, Nokia Danmark A/S, all rights reserved.";

QString mParrotString = "Copyright (C) 2007-2011, PARROT SA, all rights reserved.<br /><br />"
                        "DISCLAIMER<br />The APIs is provided by PARROT and contributors \"AS IS\""
                        " and any express or implied warranties, including, but not limited to, "
                        "the implied warranties of merchantability and fitness for a particular "
                        "purpose are disclaimed. In no event shall PARROT and contributors be "
                        "liable for any direct, indirect, incidental, special, exemplary, or "
                        "consequential damages (including, but not limited to, procurement of "
                        "substitute goods or services; loss of use, data, or profits; or business "
                        "interruption) however caused and on any theory of liability, whether in "
                        "contract, strict liability, or tort (including negligence or otherwise) "
                        "arising in any way out of the use of this software, even if advised of "
                        "the possibility of such damage.";

#endif // ABOUT_H
