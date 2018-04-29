/**
 * Serial GPS module interface driver class (Version 0.0.1)
 * This interface driver supports NMEA-0183 serial based modules.
 *
 * Copyright (C) 2010 Shinichiro Nakamura (CuBeatSystems)
 * http://shinta.main.jp/
 */
#include "mbed.h"
#include "SerialBuffered.h"

/**
 * Serial GPS module interface driver class (Version 0.0.1)
 * This interface driver supports NMEA-0183 serial based modules.
 *
 * = A list of NMEA-0183 Based GPS modules =
 *   GT-720F : http://akizukidenshi.com/catalog/g/gM-02711/
 *
 * = References =
 *   NMEA Reference Manual (January 2005) - SiRF Technology, Inc.
 */
class SerialGPS {
public:

    /**
     * Create.
     *
     * @param tx A pin of transmit.
     * @param rx A pin of receive.
     * @param baud Baud rate. (Default = 9600)
     */
    SerialGPS(PinName tx, PinName rx, int baud = 9600);

    /**
     * Destroy.
     */
    ~SerialGPS();

    /**
     * GGA - Global Positioning System Fixed Data.
     *
     * $GPGGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M, , , ,0000*18
     */
    typedef struct {
        int hour;
        int min;
        int sec;
        double latitude;
        char ns;
        double longitude;
        char ew;
        int position_fix;
        int satellites_used;
        double hdop;
        int altitude;
        char altitude_unit;
    } gps_gga_t;

    /**
     * GSA&#226;&#8364;&#129;hGNSS DOP and Active Satellites.
     *
     * $GPGSA,A,3,07,02,26,27,09,04,15, , , , , ,1.8,1.0,1.5*33
     */
    typedef struct {
        char selmode;
        int fix;
    } gps_gsa_t;

    /**
     * for RMC:
     *  Time, date, position, course and speed data.
     */
    typedef struct {
        int hour;
        int min;
        int sec;
        char status;
        double nl;
        double el;
    } gps_rmc_t;

    /**
     * for GSV:
     *  The number of GPS satellites in view satellite ID numbers,
     *  elevation, azimuth, and SNR values.
     */
    typedef struct {
        int num;
        int elevation;
        int azimuth;
        int snr;
    } gps_gsv_satellite_t;

    /**
     * for GSV:
     *  The number of GPS satellites in view satellite ID numbers,
     *  elevation, azimuth, and SNR values.
     */
    typedef struct {
        int msgcnt;
        int msgnum;
        int satcnt;
        gps_gsv_satellite_t satellite[4];
    } gps_gsv_t;

    /**
     * Callback function structure.
     */
    typedef struct {
        /**
         * A callback function for logging data.
         */
        void (*cbfunc_log)(char *str);

        /**
         * A callback function for GGA.
         *
         * GGA - Global Positioning System Fixed Data.
         */
        void (*cbfunc_gga)(gps_gga_t *p);

        /**
         * A callback function for GLL.
         *
         * GLL - Geographic Position - Latitude/Longitude.
         */
        // TODO

        /**
         * A callback function for GSA.
         *
         * GSA - GNSS DOP and Active Satellites.
         */
        void (*cbfunc_gsa)(gps_gsa_t *p);

        /**
         * A callback function for GSV.
         *
         * GSV - GNSS Satellites in View.
         */
        void (*cbfunc_gsv)(gps_gsv_t *p);

        /**
         * A callback function for MSS.
         *
         * MSS - MSK Receiver Signal.
         */
        // TODO

        /**
         * A callback function for RMC.
         *
         * RMC - Recommended Minimum Specific GNSS Data.
         */
        void (*cbfunc_rmc)(gps_rmc_t *p);

        /**
         * A callback function for VTG.
         *
         * VTG - Course Over Ground and Ground Speed.
         */
        // TODO

        /**
         * A callback function for ZDA.
         *
         * ZDA - SiRF Timing Message.
         */
        // TODO

    } gps_callback_t;

    /**
     * Processing.
     */
    bool processing();

    /**
     * Attach a callback function.
     *
     * @param cbfuncs A pointer to a call back function structure.
     */
    void attach(gps_callback_t *cbfuncs);

    /**
     * Detach a callback function.
     */
    void detach(void);

private:
    SerialBuffered ser;
    gps_callback_t *cbfuncs;
    static const int PARAM_TXTMAXLEN = 64;
    static const int PARAM_ARRAYSIZE = 64;
    static bool exists(char c, char *buf) {
        const size_t n = strlen(buf);
        for (int i = 0; i < n; i++) {
            if (c == buf[i]) {
                return true;
            }
        }
        return false;
    }
    static char *parse(char *src, char *des, size_t siz, char *delim);
    static int parseAndCallbackGGA(char *src, gps_callback_t *cbfuncs);
    static int parseAndCallbackGSA(char *src, gps_callback_t *cbfuncs);
    static int parseAndCallbackRMC(char *src, gps_callback_t *cbfuncs);
    static int parseAndCallbackGSV(char *src, gps_callback_t *cbfuncs);
    static int parseAndCallbackUnknown(char *src, gps_callback_t *cbfuncs);
    static uint8_t calcCheckSum(char *buf, size_t siz);
};
