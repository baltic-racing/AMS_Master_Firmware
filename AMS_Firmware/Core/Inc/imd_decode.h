/*
 * imd_decode.h
 *
 *  Created on: Sep 27, 2026
 *      Author: finja
 */

#ifndef IMD_DECODE_H
#define IMD_DECODE_H

#include <stdint.h>

/*
 * Auswertung fuer Bender IR155-3204, M_HS.
 * Die nachfolgenden Fehlernummern sind unsere Softwarecodes.
 */

#define IMD_PWM_INVERTED  0U
#define IMD_TIMEOUT_MS   350U
#define IMD_CAN_ID       0x203U

#define IMD_WAITING        0U
#define IMD_NORMAL         1U
#define IMD_LOW_ISOLATION  2U
#define IMD_UNDERVOLTAGE   3U
#define IMD_SST_GOOD       4U
#define IMD_SST_BAD        5U
#define IMD_DEVICE_ERROR   6U
#define IMD_EARTH_ERROR    7U
#define IMD_NO_PWM_LOW     8U
#define IMD_NO_PWM_HIGH    9U
#define IMD_INVALID       10U

#define IMD_FLAG_PWM_VALID  1U
#define IMD_FLAG_RES_VALID  2U
#define IMD_FLAG_LOW_RES    4U

#define IMD_RES_INVALID    65535U
#define IMD_RES_SATURATED  65534U

typedef struct
{
    uint16_t frequency_dHz;
    uint16_t duty_permille;
    uint16_t resistance_kohm;
    uint8_t code;
    uint8_t flags;
} IMD_Diagnostic;


static inline IMD_Diagnostic IMD_Decode(
    uint32_t period,
    uint32_t low,
    uint32_t timer_hz,
    uint16_t min_kohm)
{
    IMD_Diagnostic d = {
        0U, 0U, IMD_RES_INVALID, IMD_INVALID, 0U
    };

    uint32_t frequency;
    uint32_t duty;
    uint32_t resistance;
    uint8_t band;

    if (period == 0U ||
        low == 0U ||
        low >= period ||
        timer_hz == 0U)
    {
        return d;
    }

    frequency = (uint32_t)(
        ((uint64_t)timer_hz * 10U + period / 2U) / period
    );

    duty = ((period - low) * 1000U + period / 2U) / period;

    if (IMD_PWM_INVERTED)
    {
        duty = 1000U - duty;
    }

    d.frequency_dHz = (uint16_t)(
        frequency > 65535U ? 65535U : frequency
    );

    d.duty_permille = (uint16_t)duty;

    /*
     * Frequenzfenster +/-6 %.
     * Software-Toleranz fuer Frequenzabweichung und Quantisierung.
     */
    if (frequency >= 94U && frequency <= 106U)
    {
        band = 1U;
    }
    else if (frequency >= 188U && frequency <= 212U)
    {
        band = 2U;
    }
    else if (frequency >= 282U && frequency <= 318U)
    {
        band = 3U;
    }
    else if (frequency >= 376U && frequency <= 424U)
    {
        band = 4U;
    }
    else if (frequency >= 470U && frequency <= 530U)
    {
        band = 5U;
    }
    else
    {
        return d;
    }

    if (duty < 50U || duty > 950U)
    {
        return d;
    }

    /* 10 Hz oder 20 Hz: Widerstand auswerten */
    if (band <= 2U)
    {
        d.flags = IMD_FLAG_PWM_VALID | IMD_FLAG_RES_VALID;

        if (duty == 50U)
        {
            resistance = IMD_RES_SATURATED;
        }
        else
        {
            resistance = 1080000U / (duty - 50U) - 1200U;
        }

        d.resistance_kohm = (uint16_t)(
            resistance > IMD_RES_SATURATED
            ? IMD_RES_SATURATED
            : resistance
        );

        if (d.resistance_kohm <= min_kohm)
        {
            d.flags |= IMD_FLAG_LOW_RES;
        }

        if (band == 2U)
        {
            d.code = IMD_UNDERVOLTAGE;
        }
        else
        {
            d.code = (d.flags & IMD_FLAG_LOW_RES)
                     ? IMD_LOW_ISOLATION
                     : IMD_NORMAL;
        }
    }

    /* 30 Hz: Schnellstartmessung */
    else if (band == 3U)
    {
        if (duty <= 100U)
        {
            d.code = IMD_SST_GOOD;
        }
        else if (duty >= 900U)
        {
            d.code = IMD_SST_BAD;
        }
        else
        {
            return d;
        }

        d.flags = IMD_FLAG_PWM_VALID;
    }

    /* 40 Hz oder 50 Hz: Geraete-/Erdanschlussfehler */
    else
    {
        if (duty < 475U || duty > 525U)
        {
            return d;
        }

        d.code = (band == 4U)
                 ? IMD_DEVICE_ERROR
                 : IMD_EARTH_ERROR;

        d.flags = IMD_FLAG_PWM_VALID;
    }

    return d;
}


static inline void IMD_Pack(
    const IMD_Diagnostic *d,
    uint8_t flags,
    uint8_t data[8])
{
    data[0] = d->code;
    data[1] = d->flags | flags;

    data[2] = (uint8_t)d->frequency_dHz;
    data[3] = (uint8_t)(d->frequency_dHz >> 8);

    data[4] = (uint8_t)d->duty_permille;
    data[5] = (uint8_t)(d->duty_permille >> 8);

    data[6] = (uint8_t)d->resistance_kohm;
    data[7] = (uint8_t)(d->resistance_kohm >> 8);
}

#endif /* INC_IMD_DECODE_H_ */
