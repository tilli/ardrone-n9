#include "navdatahandler.h"

/*!
    Constructs the NavDataHandler.
    The \a parent parameter is sent to the QThread constructor.
*/
NavDataHandler::NavDataHandler(QObject *parent) : QThread(parent)
{
    qDebug() << "[NavDataHandler] Constructor";
    mAbort = false;
    initialize();
}

/*!
    Destructor.
    Waits for thread to abort before destroying the \c NavDataHandler object.
*/
NavDataHandler::~NavDataHandler()
{
    qDebug() << "[NavDataHandler] Destructor";
    mMutex.lock();
    mAbort = true;
    mCondition.wakeOne();
    mMutex.unlock();
    wait();
}

/*!
    This function sets up a UDP socket and sends a message to the drone that we are ready to
    receive data.
*/
void NavDataHandler::initialize()
{
    qDebug() << "[NavDataHandler] Initialize";

    QMutexLocker locker(&mMutex);

    // UDP socket for receiving data from the drone.
    mUdpSocketNavData = new QUdpSocket(this);
    bool res=mUdpSocketNavData->bind(NAVDATA_PORT, QUdpSocket::ShareAddress);
    if (!res) {
        qDebug()<<"[NavDataHandler] Error connecting to Navdata "<<" result:"<<res;
    } else {
        connect(mUdpSocketNavData, SIGNAL(readyRead()),this, SLOT(newNavDataReady()));
    }

    // Send message to drone to signal that we are ready to receive data.
    QHostAddress reciever;
    int res2;
    const char data[] = "Init";
    reciever=QHostAddress::QHostAddress(WIFI_MYKONOS_IP);
    res2=mUdpSocketNavData->writeDatagram(data,reciever,NAVDATA_PORT);
}

/*!
    Thread main function.
    Reads status data received from the drone.
*/
void NavDataHandler::run()
{
    qDebug() << "[NavDataHandler] run";

    uint sequence = NAVDATA_SEQUENCE_DEFAULT-1;
    uint aRDroneState = 0;
    bool emergencyState = false;
    uint batteryLevel;

    forever {
        // Do the stuff but check if we need to abort first...
        if (mAbort) {
            return;
        }

        QByteArray datagram;

        // Copy the next datagram in the list to a variable that is local to the thread.
        mMutex.lock();
        if (!mDatagramList.empty()) {
            datagram = mDatagramList.takeFirst();
        }
        mMutex.unlock();

        int size = datagram.size();

        if (size == 0) {
            sequence = NAVDATA_SEQUENCE_DEFAULT-1;
        } else {
            navdata_t* navdata = (navdata_t*)datagram.data();
            if (navdata->header == NAVDATA_HEADER) {
                // If drone status has changed, signal it to the other components.
                if (aRDroneState != navdata->ardrone_state) {
                    aRDroneState = navdata->ardrone_state;
                    emit updateARDroneState(aRDroneState);
                }

                // If emergency state has changed, signal it to the other components.
                if (emergencyState != get_mask_from_state(aRDroneState, ARDRONE_EMERGENCY_MASK)) {
                    emergencyState = get_mask_from_state(aRDroneState, ARDRONE_EMERGENCY_MASK);
                    emit updateEmergencyState(emergencyState);
                }

                // If the drone communication watchdog has been active then the sequence numbering
                // has been restarted.
                if (get_mask_from_state(aRDroneState, ARDRONE_COM_WATCHDOG_MASK)) {
                    sequence = NAVDATA_SEQUENCE_DEFAULT-1;
                }

                // Check if this is not old data we have received.
                if ( navdata->sequence > sequence ) {
                    uint navdata_cks = 0;
                    uint cks;
                    navdata_unpacked_t navdata_unpacked;
                    int res = C_OK;
                    navdata_cks_t tmp_cks = { 0 };
                    navdata_option_t* navdata_option_ptr;
                    uint tmpBat = 0;

                    navdata_option_ptr = (navdata_option_t*) &navdata->options[0];

                    vp_os_memset( &navdata_unpacked, 0, sizeof(navdata_unpacked) );

                    navdata_unpacked.ardrone_state = navdata->ardrone_state;
                    navdata_unpacked.vision_defined  = navdata->vision_defined;

                    while (navdata_option_ptr != NULL) {
                        // Check if we have a valid option
                        if (navdata_option_ptr->size == 0) {
                            qDebug() << "[NavDataHandler] Error: Option size is zero";
                            navdata_option_ptr = NULL;
                            res = C_FAIL;
                        } else {
                            switch( navdata_option_ptr->tag ) {
                            case NAVDATA_DEMO_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_demo);
                                tmpBat = navdata_unpacked.navdata_demo.vbat_flying_percentage;
                                break;
                            case NAVDATA_TIME_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_time);
                                break;
                            case NAVDATA_RAW_MEASURES_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_raw_measures);
                                break;
                            case NAVDATA_PHYS_MEASURES_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_phys_measures);
                                break;
                            case NAVDATA_GYROS_OFFSETS_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_gyros_offsets);
                                break;
                            case NAVDATA_EULER_ANGLES_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_euler_angles);
                                break;
                            case NAVDATA_REFERENCES_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_references);
                                break;
                            case NAVDATA_TRIMS_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_trims);
                                break;
                            case NAVDATA_RC_REFERENCES_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_rc_references);
                                break;
                            case NAVDATA_PWM_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_pwm);
                                break;
                            case NAVDATA_ALTITUDE_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_altitude);
                                break;
                            case NAVDATA_VISION_RAW_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_vision_raw);
                                break;
                            case NAVDATA_VISION_OF_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_vision_of);
                                break;
                            case NAVDATA_VISION_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_vision);
                                break;
                            case NAVDATA_VISION_PERF_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_vision_perf);
                                break;
                            case NAVDATA_TRACKERS_SEND_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_trackers_send);
                                break;
                            case NAVDATA_VISION_DETECT_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_vision_detect);
                                break;
                            case NAVDATA_WATCHDOG_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr, navdata_unpacked.navdata_watchdog);
                                break;
                            case NAVDATA_ADC_DATA_FRAME_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(
                                        navdata_option_ptr,
                                        navdata_unpacked.navdata_adc_data_frame);
                                break;
                            case NAVDATA_CKS_TAG:
                                navdata_option_ptr = ardrone_navdata_unpack(navdata_option_ptr,
                                                                            tmp_cks);
                                navdata_cks = tmp_cks.cks;
                                navdata_option_ptr = NULL; // End of structure
                                break;
                            default:
                                qDebug() << "[NavDataHandler] Tag "
                                         << QString::number((int) navdata_option_ptr->tag)
                                         << " is not a valid navdata option tag";
                                navdata_option_ptr = NULL;
                                res = C_FAIL;
                                break;
                            }
                        }
                    }

                    // Compute checksum.
                    cks = navdata_compute_cks((quint8*)datagram.data(), size-sizeof(navdata_cks_t));

                    // Compare checksums to see if data is valid.
                    if (cks == navdata_cks) {
                        // Data is valid.
                        // Battery level is the only information we are using from this structure.
                        uint tmpBatLevel = 0;
                        if (tmpBat > 75) {
                            tmpBatLevel = 3;
                        } else if (tmpBat > 50) {
                            tmpBatLevel = 2;
                        } else if (tmpBat > 25) {
                            tmpBatLevel = 1;
                        } else {
                            tmpBatLevel = 0;
                        }
                        
                        if (tmpBatLevel != batteryLevel) {
                            batteryLevel = tmpBatLevel;
                            emit updateBatteryLevel(batteryLevel);
                        }
                    } else {
                        qDebug() << "[NavDataHandler] Checksum failed!";
                    }
                }
                sequence = navdata->sequence;
            }
        }

        // If we have more data in queue, then wait 10 milliseconds before continuing.
        // Else wait until new data arrives.
        mMutex.lock();
        if (mDatagramList.count() > 0) {
            mCondition.wait(&mMutex, 10);
        } else {
            mCondition.wait(&mMutex);
        }
        mMutex.unlock();
    }
}

/*!
    Slot called whenever new data is ready from the drone.
    Starts the thread with NormalPriority unless it is already running.
*/
void NavDataHandler::newNavDataReady()
{
    int size = mUdpSocketNavData->pendingDatagramSize();
    QByteArray datagram;
    datagram.resize(size);
    mUdpSocketNavData->readDatagram(datagram.data(), size);
    mMutex.lock();
    mDatagramList.append(datagram);
    mMutex.unlock();
    if (!isRunning()) {
        start(NormalPriority);
    } else {
        mCondition.wakeOne();
    }
}
