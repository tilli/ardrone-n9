#ifndef NAVDATA_H
#define NAVDATA_H


/* Navdata constant */
#define NAVDATA_SEQUENCE_DEFAULT  1
#define NAVDATA_HEADER            0x55667788
#define NAVDATA_BUFFER_SIZE       2048
#define NAVDATA_PORT              5554
#define VIDEO_PORT                5555
#define FTP_PORT                  5551
#define AT_PORT                   5556
#define RAW_CAPTURE_PORT          5557

typedef float float32_t;
typedef qint8 int8_t;
typedef qint32 int32_t;
typedef qint16 int16_t;
typedef quint8 uint8_t;
typedef quint16 uint16_t;
typedef quint32 uint32_t;
#define bool_t  int32_t

#define MYKONOS_REFRESH_MS        28
#define WIFI_MYKONOS_IP           "192.168.1.1"

/* Timeout for mayday maneuvers*/
static const int32_t MAYDAY_TIMEOUT[9] = {1,1,1,1,1,1,5,5,1};

#define NAVDATA_SEQUENCE_DEFAULT  1

#define NAVDATA_HEADER  0x55667788

#define NAVDATA_MAX_SIZE 2048
#define NAVDATA_MAX_CUSTOM_TIME_SAVE 20

#define C_OK            0
#define C_FAIL          -1

// NUMBER OF TRACKERS FOR EACH TRACKING
#define NB_CORNER_TRACKERS_WIDTH    5      /* number of trackers in width of current picture */
#define NB_CORNER_TRACKERS_HEIGHT   4      /* number of trackers in height of current picture */

#define DEFAULT_NB_TRACKERS_WIDTH    (NB_CORNER_TRACKERS_WIDTH+1)// + NB_BLOCK_TRACKERS_WIDTH)
#define DEFAULT_NB_TRACKERS_HEIGHT   (NB_CORNER_TRACKERS_HEIGHT+1)// + NB_BLOCK_TRACKERS_HEIGHT)

#define YBUF_OFFSET                 0

// Bitfield definition for user input

typedef enum {
  ARDRONE_UI_BIT_AG             = 0,
  ARDRONE_UI_BIT_AB             = 1,
  ARDRONE_UI_BIT_AD             = 2,
  ARDRONE_UI_BIT_AH             = 3,
  ARDRONE_UI_BIT_L1             = 4,
  ARDRONE_UI_BIT_R1             = 5,
  ARDRONE_UI_BIT_L2             = 6,
  ARDRONE_UI_BIT_R2             = 7,
  ARDRONE_UI_BIT_SELECT         = 8,
  ARDRONE_UI_BIT_START          = 9,
  ARDRONE_UI_BIT_TRIM_THETA     = 18,
  ARDRONE_UI_BIT_TRIM_PHI       = 20,
  ARDRONE_UI_BIT_TRIM_YAW       = 22,
  ARDRONE_UI_BIT_X              = 24,
  ARDRONE_UI_BIT_Y              = 28,
} ardrone_ui_bitfield_t;

// Define constants for gyrometers handling
typedef enum {
  GYRO_X    = 0,
  GYRO_Y    = 1,
  GYRO_Z    = 2,
  NB_GYROS  = 3
} def_gyro_t;


// Define constants for accelerometers handling
typedef enum {
  ACC_X   = 0,
  ACC_Y   = 1,
  ACC_Z   = 2,
  NB_ACCS = 3
} def_acc_t;

typedef enum _navdata_tag_t {
  NAVDATA_DEMO_TAG = 0,
  NAVDATA_TIME_TAG,
  NAVDATA_RAW_MEASURES_TAG,
  NAVDATA_PHYS_MEASURES_TAG,
  NAVDATA_GYROS_OFFSETS_TAG,
  NAVDATA_EULER_ANGLES_TAG,
  NAVDATA_REFERENCES_TAG,
  NAVDATA_TRIMS_TAG,
  NAVDATA_RC_REFERENCES_TAG,
  NAVDATA_PWM_TAG,
  NAVDATA_ALTITUDE_TAG,
  NAVDATA_VISION_RAW_TAG,
  NAVDATA_VISION_OF_TAG,
  NAVDATA_VISION_TAG,
  NAVDATA_VISION_PERF_TAG,
  NAVDATA_TRACKERS_SEND_TAG,
  NAVDATA_VISION_DETECT_TAG,
  NAVDATA_WATCHDOG_TAG,
  NAVDATA_ADC_DATA_FRAME_TAG,
  NAVDATA_CKS_TAG = 0xFFFF
} navdata_tag_t;


typedef struct _matrix33_t
{
    float32_t m11;
    float32_t m12;
    float32_t m13;
    float32_t m21;
    float32_t m22;
    float32_t m23;
    float32_t m31;
    float32_t m32;
    float32_t m33;
} matrix33_t;

typedef union _float_or_int_t {
  float32_t f;
  int32_t   i;
} float_or_int_t;


typedef struct _vector31_t {
    union {
        float32_t v[3];
        struct
        {
            float32_t x;
            float32_t y;
            float32_t z;
        };
    };
} vector31_t;

typedef struct _navdata_option_t {
    uint16_t  tag;
    uint16_t  size;

    uint8_t   data[];
} navdata_option_t;


#define _ATTRIBUTE_PACKED_ __attribute__ ((packed))
typedef struct _navdata_t {
  uint32_t    header;
  uint32_t    ardrone_state;
  uint32_t    sequence;
  bool_t      vision_defined;

  navdata_option_t  options[1];
}_ATTRIBUTE_PACKED_ navdata_t;


typedef struct _navdata_demo_t {
  uint16_t    tag;
  uint16_t    size;

  uint32_t    ctrl_state;             /*!< instance of #def_ardrone_state_mask_t */
  uint32_t    vbat_flying_percentage; /*!< battery voltage filtered (mV) */

  float32_t   theta;                  /*!< UAV's attitude */
  float32_t   phi;                    /*!< UAV's attitude */
  float32_t   psi;                    /*!< UAV's attitude */

  int32_t     altitude;               /*!< UAV's altitude */

  float32_t   vx;                     /*!< UAV's estimated linear velocity */
  float32_t   vy;                     /*!< UAV's estimated linear velocity */
  float32_t   vz;                     /*!< UAV's estimated linear velocity */

  uint32_t    num_frames;			  /*!< streamed frame index */ // Not used -> To integrate in video stage.

  // Camera parameters compute by detection
  matrix33_t  detection_camera_rot; /*!<  Deprecated ! Don't use ! */
  vector31_t  detection_camera_trans; /*!<  Deprecated ! Don't use ! */
  uint32_t	  detection_tag_index; /*!<  Deprecated ! Don't use ! */
  uint32_t	  detection_camera_type; /*!<  Deprecated ! Don't use ! */

  // Camera parameters compute by drone
  matrix33_t  drone_camera_rot;
  vector31_t  drone_camera_trans;
}_ATTRIBUTE_PACKED_ navdata_demo_t;

/// Last navdata option that *must* be included at the end of all navdata packet
/// + 6 bytes
typedef struct _navdata_cks_t {
  uint16_t  tag;
  uint16_t  size;

  // Checksum for all navdatas (including options)
  uint32_t  cks;
}_ATTRIBUTE_PACKED_ navdata_cks_t;


/// + 6 bytes
typedef struct _navdata_time_t {
  uint16_t  tag;
  uint16_t  size;

  uint32_t  time;
}_ATTRIBUTE_PACKED_ navdata_time_t;

/**
 * \struct _velocities_t
 * \brief Velocities in float32_t format
 */
typedef struct _velocities_t {
  float32_t x;
  float32_t y;
  float32_t z;
} velocities_t;

typedef struct _navdata_raw_measures_t {
  uint16_t  tag;
  uint16_t  size;

  // +12 bytes
  uint16_t  raw_accs[NB_ACCS];    // filtered accelerometers
  uint16_t  raw_gyros[NB_GYROS];  // filtered gyrometers
  uint16_t  raw_gyros_110[2];     // gyrometers  x/y 110°/s
  uint32_t  vbat_raw;             // battery voltage raw (mV)
  uint16_t  us_debut_echo;
  uint16_t  us_fin_echo;
  uint16_t  us_association_echo;
  uint16_t  us_distance_echo;
  uint16_t  us_courbe_temps;
  uint16_t  us_courbe_valeur;
  uint16_t  us_courbe_ref;
}_ATTRIBUTE_PACKED_ navdata_raw_measures_t;


typedef struct _navdata_phys_measures_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   accs_temp;
  uint16_t    gyro_temp;
  float32_t   phys_accs[NB_ACCS];
  float32_t   phys_gyros[NB_GYROS];
  uint32_t    alim3V3;              // 3.3volt alim [LSB]
  uint32_t    vrefEpson;            // ref volt Epson gyro [LSB]
  uint32_t    vrefIDG;              // ref volt IDG gyro [LSB]
}_ATTRIBUTE_PACKED_ navdata_phys_measures_t;


typedef struct _navdata_gyros_offsets_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t offset_g[NB_GYROS];
}_ATTRIBUTE_PACKED_ navdata_gyros_offsets_t;


typedef struct _navdata_euler_angles_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   theta_a;
  float32_t   phi_a;
}_ATTRIBUTE_PACKED_ navdata_euler_angles_t;


typedef struct _navdata_references_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t   ref_theta;
  int32_t   ref_phi;
  int32_t   ref_theta_I;
  int32_t   ref_phi_I;
  int32_t   ref_pitch;
  int32_t   ref_roll;
  int32_t   ref_yaw;
  int32_t   ref_psi;
}_ATTRIBUTE_PACKED_ navdata_references_t;


typedef struct _navdata_trims_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t angular_rates_trim_r;
  float32_t euler_angles_trim_theta;
  float32_t euler_angles_trim_phi;
}_ATTRIBUTE_PACKED_ navdata_trims_t;

typedef struct _navdata_rc_references_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t    rc_ref_pitch;
  int32_t    rc_ref_roll;
  int32_t    rc_ref_yaw;
  int32_t    rc_ref_gaz;
  int32_t    rc_ref_ag;
}_ATTRIBUTE_PACKED_ navdata_rc_references_t;


typedef struct _navdata_pwm_t {
  uint16_t   tag;
  uint16_t   size;

  uint8_t     motor1;
  uint8_t     motor2;
  uint8_t     motor3;
  uint8_t     motor4;
  uint8_t	  sat_motor1;
  uint8_t	  sat_motor2;
  uint8_t	  sat_motor3;
  uint8_t	  sat_motor4;
  int32_t     gaz_feed_forward;
  int32_t     gaz_altitude;
  float32_t   altitude_integral;
  float32_t   vz_ref;
  int32_t     u_pitch;
  int32_t     u_roll;
  int32_t     u_yaw;
  float32_t   yaw_u_I;
  int32_t     u_pitch_planif;
  int32_t     u_roll_planif;
  int32_t     u_yaw_planif;
  float32_t   u_gaz_planif;
  uint16_t    current_motor1;
  uint16_t    current_motor2;
  uint16_t    current_motor3;
  uint16_t    current_motor4;
}_ATTRIBUTE_PACKED_ navdata_pwm_t;


typedef struct _navdata_altitude_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t   altitude_vision;
  float32_t altitude_vz;
  int32_t   altitude_ref;
  int32_t   altitude_raw;
}_ATTRIBUTE_PACKED_ navdata_altitude_t;


typedef struct _navdata_vision_raw_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t vision_tx_raw;
  float32_t vision_ty_raw;
  float32_t vision_tz_raw;
}_ATTRIBUTE_PACKED_ navdata_vision_raw_t;


typedef struct _navdata_vision_t {
  uint16_t   tag;
  uint16_t   size;

  uint32_t   vision_state;
  int32_t    vision_misc;
  float32_t  vision_phi_trim;
  float32_t  vision_phi_ref_prop;
  float32_t  vision_theta_trim;
  float32_t  vision_theta_ref_prop;

  int32_t    new_raw_picture;
  float32_t  theta_capture;
  float32_t  phi_capture;
  float32_t  psi_capture;
  int32_t    altitude_capture;
  uint32_t   time_capture;     // time in TSECDEC format (see config.h)
  velocities_t body_v;

  float32_t  delta_phi;
  float32_t  delta_theta;
  float32_t  delta_psi;

        uint32_t	gold_defined;
        uint32_t	gold_reset;
        float32_t gold_x;
        float32_t gold_y;
}_ATTRIBUTE_PACKED_ navdata_vision_t;


typedef struct _navdata_vision_perf_t {
  uint16_t   tag;
  uint16_t   size;

  // +44 bytes
  float32_t  time_szo;
  float32_t  time_corners;
  float32_t  time_compute;
  float32_t  time_tracking;
  float32_t  time_trans;
  float32_t  time_update;
  float32_t  time_custom[NAVDATA_MAX_CUSTOM_TIME_SAVE];
}_ATTRIBUTE_PACKED_ navdata_vision_perf_t;

typedef struct _screen_point_t {
  int32_t x;
  int32_t y;
} screen_point_t;

typedef struct _navdata_trackers_send_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t locked[DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT];
  screen_point_t point[DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT];
}_ATTRIBUTE_PACKED_ navdata_trackers_send_t;


typedef struct _navdata_vision_detect_t {
  uint16_t   tag;
  uint16_t   size;

  uint32_t   nb_detected;
  uint32_t   type[4];
  uint32_t   xc[4];
  uint32_t   yc[4];
  uint32_t   width[4];
  uint32_t   height[4];
  uint32_t   dist[4];
}_ATTRIBUTE_PACKED_ navdata_vision_detect_t;

typedef struct _navdata_vision_of_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   of_dx[5];
  float32_t   of_dy[5];
}_ATTRIBUTE_PACKED_ navdata_vision_of_t;


typedef struct _navdata_watchdog_t {
  uint16_t   tag;
  uint16_t   size;

  // +4 bytes
  int32_t    watchdog;
}_ATTRIBUTE_PACKED_ navdata_watchdog_t;

typedef struct _navdata_adc_data_frame_t {
  uint16_t  tag;
  uint16_t  size;

  uint32_t  version;
  uint8_t   data_frame[32];
}_ATTRIBUTE_PACKED_ navdata_adc_data_frame_t;


typedef struct _navdata_unpacked_t {
  uint32_t  ardrone_state;
  bool_t    vision_defined;

  navdata_demo_t           navdata_demo;
  navdata_time_t           navdata_time;
  navdata_raw_measures_t   navdata_raw_measures;
  navdata_phys_measures_t  navdata_phys_measures;
  navdata_gyros_offsets_t  navdata_gyros_offsets;
  navdata_euler_angles_t   navdata_euler_angles;
  navdata_references_t     navdata_references;
  navdata_trims_t          navdata_trims;
  navdata_rc_references_t  navdata_rc_references;
  navdata_pwm_t            navdata_pwm;
  navdata_altitude_t       navdata_altitude;
  navdata_vision_raw_t     navdata_vision_raw;
  navdata_vision_of_t      navdata_vision_of;
  navdata_vision_t         navdata_vision;
  navdata_vision_perf_t    navdata_vision_perf;
  navdata_trackers_send_t  navdata_trackers_send;
  navdata_vision_detect_t  navdata_vision_detect;
  navdata_watchdog_t       navdata_watchdog;
  navdata_adc_data_frame_t navdata_adc_data_frame;
} navdata_unpacked_t;




static inline int get_mask_from_state( uint32_t state, uint32_t mask )
{
    return state & mask ? TRUE : FALSE;
}

static inline uint8_t* navdata_unpack_option( uint8_t* navdata_ptr, uint8_t* data, uint32_t size )
{
    memcpy(data, navdata_ptr, size);

    return (navdata_ptr + size);
}


static inline navdata_option_t* navdata_next_option( navdata_option_t* navdata_options_ptr )
{
    uint8_t* ptr;

    ptr  = (uint8_t*) navdata_options_ptr;
    ptr += navdata_options_ptr->size;

    return (navdata_option_t*) ptr;
}

navdata_option_t* navdata_search_option( navdata_option_t* navdata_options_ptr, uint32_t tag );


static inline uint32_t navdata_compute_cks( uint8_t* nv, int32_t size )
{
    int32_t i;
    uint32_t cks;
    uint32_t temp;

    cks = 0;

    for( i = 0; i < size; i++ )
        {
            temp = nv[i];
            cks += temp;
        }

    return cks;
}


//C_RESULT ardrone_navdata_control_init( void* data )
//{
//  return C_OK;
//}

//C_RESULT ardrone_navdata_control_process( const navdata_unpacked_t* const navdata )
//{
//  return ardrone_control_resume_on_navdata_received(navdata->ardrone_state);
//}

//C_RESULT ardrone_navdata_control_release( void )
//{
//  return C_OK;
//}

//static void mykonos_navdata_unpack_all(navdata_unpacked_t* navdata_unpacked, navdata_t* navdata, uint32_t* cks);

/*New proto*/
// Facility to declare a set of navdata handler
// Handler to resume control thread is mandatory
/*#define BEGIN_NAVDATA_HANDLER_TABLE \
  ardrone_navdata_handler_t ardrone_navdata_handler_table[] = {

#define END_NAVDATA_HANDLER_TABLE																				\
    { ardrone_navdata_control_init, ardrone_navdata_control_process, ardrone_navdata_control_release, NULL },	\
    { NULL, NULL, NULL, NULL }																					\
};

#define NAVDATA_HANDLER_TABLE_ENTRY( init, process, release, init_data_ptr ) \
  { (ardrone_navdata_handler_init_t)init, process, release, init_data_ptr },*/


#define C_RESULT        int
typedef C_RESULT (*ardrone_navdata_handler_init_t)( void* data );
typedef C_RESULT (*ardrone_navdata_handler_process_t)( const navdata_unpacked_t* const navdata );
typedef C_RESULT (*ardrone_navdata_handler_release_t)( void );

typedef struct _ardrone_navdata_handler_t {
  ardrone_navdata_handler_init_t    init;
  ardrone_navdata_handler_process_t process;
  ardrone_navdata_handler_release_t release;

  void*                             data; // Data used during initialization
} ardrone_navdata_handler_t;

extern ardrone_navdata_handler_t ardrone_navdata_handler_table[];

static inline void *
vp_os_memset(void *s, int c, size_t n)
{
#ifdef DEBUG_MODE
  void *res;
  assert(s);
  res = memset(s, c, n);
  assert(res == s);
  return (res);
#else // ! DEBUG_MODE
  return memset(s, c, n);
#endif // <- DEBUG_MODE
}
/*************************From app.h**************************/
enum {
  NO_CONTROL_MODE = 0,          // Doing nothing
  MYKONOS_UPDATE_CONTROL_MODE,  // Mykonos software update reception (update is done next run)
                                // After event completion, card should power off
  PIC_UPDATE_CONTROL_MODE,      // Mykonos pic software update reception (update is done next run)
                                // After event completion, card should power off
  LOGS_GET_CONTROL_MODE,        // Send previous run's logs
  CFG_GET_CONTROL_MODE,         // Send activ configuration
  ACK_CONTROL_MODE              // Reset command mask in navdata
};


/// \enum def_ardrone_state_mask_t is a bit field representing ARDrone' state


// Define masks for ARDrone state
// 31                                                             0
//  x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x -> state
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | FLY MASK : (0) ardrone is landed, (1) ardrone is flying
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VIDEO MASK : (0) video disable, (1) video enable
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VISION MASK : (0) vision disable, (1) vision enable
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | CONTROL ALGO : (0) euler angles control, (1) angular speed control
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active
//  | | | | | | | | | | | | | | | | | | | | | | | | | | USER feedback : Start button state
//  | | | | | | | | | | | | | | | | | | | | | | | | | Control command ACK : (0) None, (1) one received
//  | | | | | | | | | | | | | | | | | | | | | | | | Trim command ACK : (0) None, (1) one received
//  | | | | | | | | | | | | | | | | | | | | | | | Trim running : (0) none, (1) running
//  | | | | | | | | | | | | | | | | | | | | | | Trim result : (0) failed, (1) succeeded
//  | | | | | | | | | | | | | | | | | | | | | Navdata demo : (0) All navdata, (1) only navdata demo
//  | | | | | | | | | | | | | | | | | | | | Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent
//  | | | | | | | | | | | | | | | | | | |
//  | | | | | | | | | | | | | | | | | |
//  | | | | | | | | | | | | | | | | | Bit means that there's an hardware problem with gyrometers
//  | | | | | | | | | | | | | | | | VBat low : (1) too low, (0) Ok
//  | | | | | | | | | | | | | | | VBat high (US mad) : (1) too high, (0) Ok
//  | | | | | | | | | | | | | | Timer elapsed : (1) elapsed, (0) not elapsed
//  | | | | | | | | | | | | | Power : (0) Ok, (1) not enough to fly
//  | | | | | | | | | | | | Angles : (0) Ok, (1) out of range
//  | | | | | | | | | | | Wind : (0) Ok, (1) too much to fly
//  | | | | | | | | | | Ultrasonic sensor : (0) Ok, (1) deaf
//  | | | | | | | | | Cutout system detection : (0) Not detected, (1) detected
//  | | | | | | | | PIC Version number OK : (0) a bad version number, (1) version number is OK
//  | | | | | | | ATCodec thread ON : (0) thread OFF (1) thread ON
//  | | | | | | Navdata thread ON : (0) thread OFF (1) thread ON
//  | | | | | Video thread ON : (0) thread OFF (1) thread ON
//  | | | | Acquisition thread ON : (0) thread OFF (1) thread ON
//  | | | CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled // Check frequency of control loop
//  | | ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good // Check frequency of uart2 dsr (com with adc)
//  | Communication Watchdog : (1) com problem, (0) Com is ok // Check if we have an active connection with a client
//  Emergency landing : (0) no emergency, (1) emergency

typedef enum {
  ARDRONE_FLY_MASK            = 1 << 0,  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
  ARDRONE_VIDEO_MASK          = 1 << 1,  /*!< VIDEO MASK : (0) video disable, (1) video enable */
  ARDRONE_VISION_MASK         = 1 << 2,  /*!< VISION MASK : (0) vision disable, (1) vision enable */
  ARDRONE_CONTROL_MASK        = 1 << 3,  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
  ARDRONE_ALTITUDE_MASK       = 1 << 4,  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
  ARDRONE_USER_FEEDBACK_START = 1 << 5,  /*!< USER feedback : Start button state */
  ARDRONE_COMMAND_MASK        = 1 << 6,  /*!< Control command ACK : (0) None, (1) one received */
  ARDRONE_FW_FILE_MASK        = 1 << 7,  /* Firmware file is good (1) */
  ARDRONE_FW_VER_MASK         = 1 << 8,  /* Firmware update is newer (1) */
//  ARDRONE_FW_UPD_MASK         = 1 << 9,  /* Firmware update is ongoing (1) */
  ARDRONE_NAVDATA_DEMO_MASK   = 1 << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
  ARDRONE_NAVDATA_BOOTSTRAP   = 1 << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
  ARDRONE_MOTORS_MASK  	      = 1 << 12, /*!< Motors status : (0) Ok, (1) Motors problem */
  ARDRONE_COM_LOST_MASK       = 1 << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
  ARDRONE_VBAT_LOW            = 1 << 15, /*!< VBat low : (1) too low, (0) Ok */
  ARDRONE_USER_EL             = 1 << 16, /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
  ARDRONE_TIMER_ELAPSED       = 1 << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
  ARDRONE_ANGLES_OUT_OF_RANGE = 1 << 19, /*!< Angles : (0) Ok, (1) out of range */
  ARDRONE_ULTRASOUND_MASK     = 1 << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
  ARDRONE_CUTOUT_MASK         = 1 << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
  ARDRONE_PIC_VERSION_MASK    = 1 << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
  ARDRONE_ATCODEC_THREAD_ON   = 1 << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_NAVDATA_THREAD_ON   = 1 << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_VIDEO_THREAD_ON     = 1 << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_ACQ_THREAD_ON       = 1 << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_CTRL_WATCHDOG_MASK  = 1 << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
  ARDRONE_ADC_WATCHDOG_MASK   = 1 << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
  ARDRONE_COM_WATCHDOG_MASK   = 1 << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
  ARDRONE_EMERGENCY_MASK      = 1 << 31  /*!< Emergency landing : (0) no emergency, (1) emergency */
} def_ardrone_state_mask_t;

// define video channel
typedef enum
{
        ZAP_CHANNEL_HORI=0,
        ZAP_CHANNEL_VERT,
        ZAP_CHANNEL_LARGE_HORI_SMALL_VERT,
        ZAP_CHANNEL_LARGE_VERT_SMALL_HORI,
        ZAP_CHANNEL_NEXT,
} ZAP_VIDEO_CHANNEL;

#define ardrone_navdata_unpack( navdata_ptr, option ) (navdata_option_t*) navdata_unpack_option( \
                                                                         (uint8_t*) navdata_ptr, \
                                                                         (uint8_t*) &option,     \
                                                                          navdata_ptr->size )

#endif // NAVDATA_H
