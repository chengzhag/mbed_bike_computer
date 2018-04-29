/**
 * Serial GPS module interface driver class (Version 0.0.1)
 * This interface driver supports NMEA-0183 serial based modules.
 *
 * Copyright (C) 2010 Shinichiro Nakamura (CuBeatSystems)
 * http://shinta.main.jp/
 */
#include "SerialGPS.h"

/**
 * Create.
 *
 * @param tx A pin of transmit.
 * @param rx A pin of receive.
 * @param baud Baud rate. (Default = 9600)
 */
SerialGPS::SerialGPS(PinName tx, PinName rx, int baud) : ser(tx, rx), cbfuncs(NULL) {
    ser.baud(baud);
    ser.setTimeout(50);
}

/**
 * Destroy.
 */
SerialGPS::~SerialGPS() {
}

/**
 * Processing.
 */
bool SerialGPS::processing() {
    /*
     * Read from a serial buffer.
     */
    static const int DATABUFSIZ = 128;
    char src[DATABUFSIZ];
    int cnt = 0;
    do {
        cnt = 0;
        bool done = false;
        do {
            int c = ser.getc();
            if (c < 0) {
                return false;
            }
            if ((c == '\r') || (c == '\n')) {
                done = true;
                c = '\0';
            }
            src[cnt++] = c & 0xff;
        } while (!done);
    } while (cnt <= 1);
    
    /*
     * Return if the callback function does not exists.
     */
    if (cbfuncs == NULL) {
        return true;
    }

    /*
     * Call a function for logging data.
     */
    if (cbfuncs->cbfunc_log != NULL) {
        cbfuncs->cbfunc_log(src);
    }

    /*
     * Check a check sum for the data. The data format is '$[DATA]*CS'.
     */
    const size_t srclen = strlen(src);
    if ((src[0] == '$') && (src[srclen - 3] == '*')) {
        int cs_src;
        if (sscanf(src + srclen - 2, "%X", &cs_src) != 1) {
            printf("Invalid check sum data found.(%s)\n", src);
            return false;
        }
        uint8_t cs_cal = calcCheckSum(src + 1, srclen - 4);
        if ((uint8_t)cs_src != cs_cal) {
            printf("Illegal data found.(%s)\n", src);
            return false;
        }
    } else {
        printf("Invalid data format found.(%s)\n", src);
        return false;
    }

    /*
     * Parse a data.
     */
    int res = 0;
    char *p = src;
    char des[DATABUFSIZ];
    if ((p = parse(p, des, sizeof(des), ",*")) != NULL) {
        if (strcmp(des, "$GPGGA") == 0) {
            res = parseAndCallbackGGA(src, cbfuncs);
        } else if (strcmp(des, "$GPGSA") == 0) {
            res = parseAndCallbackGSA(src, cbfuncs);
        } else if (strcmp(des, "$GPRMC") == 0) {
            res = parseAndCallbackRMC(src, cbfuncs);
        } else if (strcmp(des, "$GPGSV") == 0) {
            res = parseAndCallbackGSV(src, cbfuncs);
        } else {
            res = parseAndCallbackUnknown(src, cbfuncs);
        }
    }
    return (res == 0) ? true : false;
}

/**
 * Attach a callback function.
 *
 * @param cbfuncs A pointer to a callback function structure.
 */
void SerialGPS::attach(gps_callback_t *cbfuncs) {
    SerialGPS::cbfuncs = cbfuncs;
}

/**
 * Detach a callback function.
 */
void SerialGPS::detach(void) {
    SerialGPS::cbfuncs = NULL;
}

/**
 * Parse a text string.
 */
char * SerialGPS::parse(char *src, char *des, size_t dessiz, char *delim) {
    for (int i = 0; i < dessiz; i++, src++, des++) {
        if ((*src == '\0') || (*src == '\r') || (*src == '\n')) {
            *des = '\0';
            return NULL;
        }
        if (exists(*src, delim)) {
            *des = '\0';
            return ++src;
        }
        *des = *src;
    }
    des = '\0';
    return NULL;
}

int SerialGPS::parseAndCallbackGGA(char *src, gps_callback_t *cbfuncs) {

    if (cbfuncs->cbfunc_gga == NULL) {
        return -1;
    }

    char plist[PARAM_ARRAYSIZE][PARAM_TXTMAXLEN];
    char *p = src;
    int cnt = 0;
    while ((p = parse(p, plist[cnt], PARAM_TXTMAXLEN, ",*")) != NULL) {
        cnt++;
    }

    if (cnt == 15) {
        gps_gga_t data;
        data.hour = (plist[1][0] - '0') * 10 + (plist[1][1] - '0') * 1;
        data.min = (plist[1][2] - '0') * 10 + (plist[1][3] - '0') * 1;
        data.sec = (plist[1][4] - '0') * 10 + (plist[1][5] - '0') * 1;

        if (sscanf(plist[2], "%lf", &data.latitude) != 1) {
            return -2;
        }
        if (sscanf(plist[3], "%c", &data.ns) != 1) {
            return -3;
        }
        const char ns = data.ns;
        if ((ns != 'N') && (ns != 'S')) {
            return -4;
        }
        if (sscanf(plist[4], "%lf", &data.longitude) != 1) {
            return -5;
        }
        if (sscanf(plist[5], "%c", &data.ew) != 1) {
            return -6;
        }
        const char ew = data.ew;
        if ((ew != 'E') && (ew != 'W')) {
            return -7;
        }

        data.position_fix = atoi(plist[6]);
        data.satellites_used = atoi(plist[7]);

        if (sscanf(plist[8], "%lf", &data.hdop) != 1) {
            return -8;
        }

        data.altitude = atoi(plist[9]);
        if (strcmp(plist[10], "M") != 0) {
            return -9;
        }

        data.altitude = atoi(plist[11]);
        if (sscanf(plist[12], "%c", &data.altitude_unit) != 1) {
            return -10;
        }

        cbfuncs->cbfunc_gga(&data);
        return 0;
    }
    return -11;
}

int SerialGPS::parseAndCallbackGSA(char *src, gps_callback_t *cbfuncs) {

    if (cbfuncs->cbfunc_gsa == NULL) {
        return -1;
    }

    char plist[PARAM_ARRAYSIZE][PARAM_TXTMAXLEN];
    char *p = src;
    int cnt = 0;
    while ((p = parse(p, plist[cnt], PARAM_TXTMAXLEN, ",*")) != NULL) {
        cnt++;
    }

    if (cnt == 18) {
        gps_gsa_t data;
        data.selmode = plist[1][0];
        if ((data.selmode != 'A') && (data.selmode != 'M')) {
            return -2;
        }

        data.fix = atoi(plist[2]);
        if ((data.fix != 1) && (data.fix != 2) && (data.fix != 3)) {
            return -3;
        }

        cbfuncs->cbfunc_gsa(&data);
        return 0;
    }
    return -4;
}

int SerialGPS::parseAndCallbackRMC(char *src, gps_callback_t *cbfuncs) {

    if (cbfuncs->cbfunc_rmc == NULL) {
        return -1;
    }

    char plist[PARAM_ARRAYSIZE][PARAM_TXTMAXLEN];
    char *p = src;
    int cnt = 0;
    while ((p = parse(p, plist[cnt], PARAM_TXTMAXLEN, ",*")) != NULL) {
        cnt++;
    }

    if (cnt == 13) {
        gps_rmc_t data;
        data.hour = (plist[1][0] - '0') * 10 + (plist[1][1] - '0') * 1;
        data.min = (plist[1][2] - '0') * 10 + (plist[1][3] - '0') * 1;
        data.sec = (plist[1][4] - '0') * 10 + (plist[1][5] - '0') * 1;

        data.status = plist[2][0];

        if (sscanf(plist[3], "%lf", &data.nl) != 1) {
            return -2;
        }
        if (strcmp(plist[4], "N") != 0) {
            return -3;
        }

        if (sscanf(plist[5], "%lf", &data.el) != 1) {
            return -4;
        }
        if (strcmp(plist[6], "E") != 0) {
            return -5;
        }

        cbfuncs->cbfunc_rmc(&data);
        return 0;
    }
    return -4;
}

int SerialGPS::parseAndCallbackGSV(char *src, gps_callback_t *cbfuncs) {

    if (cbfuncs->cbfunc_gsv == NULL) {
        return -1;
    }

    char plist[PARAM_ARRAYSIZE][PARAM_TXTMAXLEN];
    char *p = src;
    int cnt = 0;
    while ((p = parse(p, plist[cnt], PARAM_TXTMAXLEN, ",*")) != NULL) {
        cnt++;
    }

    if (cnt == 20) {
        gps_gsv_t data;
        data.msgcnt = atoi(plist[1]);

        data.msgnum = atoi(plist[2]);

        data.satcnt = atoi(plist[3]);

        static const int SATINFOFS = 4;
        for (int i = 0; i < 4; i++) {
            data.satellite[i].num = atoi(plist[SATINFOFS + 0 * (i * 4)]);
            data.satellite[i].elevation = atoi(plist[SATINFOFS + 1 * (i * 4)]);
            data.satellite[i].azimuth = atoi(plist[SATINFOFS + 2 * (i * 4)]);
            data.satellite[i].snr = atoi(plist[SATINFOFS + 3 *(i * 4)]);
        }

        int chksum;
        if (sscanf(plist[20], "%x", &chksum) != 1) {
            return -2;
        }

        cbfuncs->cbfunc_gsv(&data);
        return 0;
    }
    return -3;
}

int SerialGPS::parseAndCallbackUnknown(char *src, gps_callback_t *cbfuncs) {
    return 0;
}

uint8_t SerialGPS::calcCheckSum(char *buf, size_t siz) {
    uint8_t cs = 0;
    for (int i = 0; i < siz; i++) {
        cs ^= buf[i];
    }
    return cs;
}
