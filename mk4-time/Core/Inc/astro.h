/*
 * astro.h — minimal solar/lunar/grid astronomy for the Precision Clock Mk IV.
 *
 * Self-contained C99 + <math.h>; NO firmware dependencies, so it compiles and
 * unit-tests natively (see test_astro.c). All angles in degrees at the API
 * boundary; UTC instants are passed as a double of Unix seconds.
 *
 * Algorithms are low-precision (NOAA/Meeus-class) approximations — accurate to a
 * fraction of a degree / a minute or two, which is all a 7-segment readout shows,
 * and cheap enough to evaluate once per mode entry on the STM32's soft-double.
 */
#ifndef ASTRO_H
#define ASTRO_H

/* (a) Sun apparent alt/az for an observer at (lat, lon) decimal degrees, N+/E+,
 *     at the given UTC instant. Writes azimuth in [0,360) measured from North
 *     clockwise, and elevation in [-90,90] (negative = below the horizon).
 *     No refraction or parallax correction. */
void   sun_az_el(double lat, double lon, double unix_s, double *az, double *el);

/* (b) Sun event times for the UTC calendar day containing unix_s.
 *     Every output is a decimal UTC hour and MAY be < 0 or > 24 (the event falls
 *     on the previous/next day) — callers add the local offset and wrap, they do
 *     NOT clamp. solar_noon is always written. Returns 0 normally; returns
 *     nonzero on polar day/night (sun never crosses -0.833 deg), in which case
 *     sunrise/sunset are left untouched and only solar_noon is meaningful.
 *     Any of the optional twilight pointers may be NULL. civil/nautical/golden are
 *     evening (dusk) times that fall back to the sunset hour angle if that altitude
 *     is never reached; astro_dusk (sun at -18 deg) is set to NAN instead when the
 *     sun never dips that low (summer white night), so darkness can be reported absent. */
int    sun_times(double lat, double lon, double unix_s,
                 double *sunrise, double *sunset, double *solar_noon,
                 double *civil_dusk, double *nautical_dusk, double *golden_dusk,
                 double *astro_dusk);

/* (c) Moon. phase is the synodic fraction [0,1): 0=new, .25=first quarter,
 *     .5=full, .75=last quarter. */
double moon_phase(double unix_s);
double moon_illuminated_fraction(double phase);   /* (1 - cos(2*pi*phase)) / 2  */
int    moon_phase_index(double phase);            /* 0..7, see ASTRO_MOON_NAMES  */

/* (d) Equation of time in minutes (+ = apparent sun ahead of mean/clock sun). */
double equation_of_time(double unix_s);

double local_sidereal_time(double unix_s, double lon); /* LMST, hours [0,24), lon E+ */
double local_solar_time(double unix_s, double lon);    /* apparent solar, hours [0,24) */

/* (d2) Subsolar point at the given UTC instant: latitude = solar declination,
 *      longitude in [-180,180] (E+). Time-only — needs no observer position. */
void   sun_subsolar(double unix_s, double *lat, double *lon);

/* (e) 6-character Maidenhead locator for (lat, lon). out must hold >= 7 bytes.
 *     Writes "----\0" if either coordinate is non-finite. */
void   maidenhead(double lat, double lon, char out[7]);

/* Phase-index -> short name. Index from moon_phase_index(). */
extern const char *const ASTRO_MOON_NAMES[8];

#endif /* ASTRO_H */
