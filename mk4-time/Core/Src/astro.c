/*
 * astro.c — see astro.h.
 * Pure C99 + <math.h>; every intermediate is double on purpose (single-precision
 * loses the sub-arc-minute accuracy these approximations are otherwise good for).
 */
#include "astro.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define J2000_UNIX 946728000.0      /* 2000-01-01 12:00:00 UTC = JD 2451545.0 */
#define DEG (M_PI / 180.0)
#define RAD (180.0 / M_PI)

const char *const ASTRO_MOON_NAMES[8] = {
    "New", "Waxing Crescent", "First Quarter", "Waxing Gibbous",
    "Full", "Waning Gibbous", "Last Quarter", "Waning Crescent",
};

static double days_since_j2000(double unix_s) { return (unix_s - J2000_UNIX) / 86400.0; }

/* The shared low-precision solar block: from days-since-J2000 produce mean
 * longitude L, anomaly g, ecliptic longitude lambda, obliquity eps, and the
 * apparent right ascension alpha (deg) and declination delta (rad). */
static void sun_ecliptic(double n, double *L, double *g, double *lambda,
                         double *eps, double *alpha, double *delta) {
    double Lv = fmod(280.460 + 0.9856474 * n, 360.0);
    double gv = fmod(357.528 + 0.9856003 * n, 360.0);
    double lam = Lv + 1.915 * sin(gv * DEG) + 0.020 * sin(2.0 * gv * DEG);
    double ep = (23.439 - 0.0000004 * n) * DEG;
    double al = atan2(cos(ep) * sin(lam * DEG), cos(lam * DEG)) * RAD;
    double de = asin(sin(ep) * sin(lam * DEG));
    if (L) *L = Lv;
    if (g) *g = gv;
    if (lambda) *lambda = lam;
    if (eps) *eps = ep;
    if (alpha) *alpha = al;
    if (delta) *delta = de;
}

void sun_az_el(double lat, double lon, double unix_s, double *az, double *el) {
    double n = days_since_j2000(unix_s);
    double alpha, delta;
    sun_ecliptic(n, NULL, NULL, NULL, NULL, &alpha, &delta);

    double gmst = fmod(18.697374558 + 24.06570982441908 * n, 24.0); /* GMST in hours, used as-is */
    double lst = gmst * 15.0 + lon;                                 /* degrees */
    double ha = (lst - alpha) * DEG;                                /* radians */

    double e = asin(sin(lat * DEG) * sin(delta) + cos(lat * DEG) * cos(delta) * cos(ha)) * RAD;
    double a = atan2(-sin(ha), tan(delta) * cos(lat * DEG) - sin(lat * DEG) * cos(ha)) * RAD;
    if (a < 0.0) a += 360.0;
    if (el) *el = e;
    if (az) *az = a;
}

double equation_of_time(double unix_s) {
    double n = days_since_j2000(unix_s);
    double L, alpha;
    sun_ecliptic(n, &L, NULL, NULL, NULL, &alpha, NULL);
    double diff = fmod(L - alpha, 360.0);
    if (diff > 180.0) diff -= 360.0;
    else if (diff <= -180.0) diff += 360.0;
    return 4.0 * diff; /* minutes */
}

int sun_times(double lat, double lon, double unix_s,
              double *sunrise, double *sunset, double *solar_noon,
              double *civil_dusk, double *nautical_dusk, double *golden_dusk,
              double *astro_dusk) {
    /* Reference instant = 12:00:00 UTC of the calendar day of unix_s. */
    double noon_unix = trunc(unix_s / 86400.0) * 86400.0 + 43200.0;
    double n = days_since_j2000(noon_unix);
    double delta;
    sun_ecliptic(n, NULL, NULL, NULL, NULL, NULL, &delta);
    double eot = equation_of_time(noon_unix); /* minutes */

    double noon = 12.0 - eot / 60.0 - lon / 15.0;
    if (solar_noon) *solar_noon = noon;

    /* Hour angle (deg) at which the sun centre sits at altitude h (deg). */
    double cosO = (sin(-0.833 * DEG) - sin(lat * DEG) * sin(delta)) / (cos(lat * DEG) * cos(delta));
    if (cosO < -1.0 || cosO > 1.0) return 1; /* polar day / night: no rise or set */
    double omega = acos(cosO) * RAD;
    if (sunrise) *sunrise = noon - omega / 15.0;
    if (sunset) *sunset = noon + omega / 15.0;

    if (civil_dusk) {
        double c = (sin(-6.0 * DEG) - sin(lat * DEG) * sin(delta)) / (cos(lat * DEG) * cos(delta));
        double o = (c < -1.0 || c > 1.0) ? omega : acos(c) * RAD;
        *civil_dusk = noon + o / 15.0;
    }
    if (nautical_dusk) {
        double c = (sin(-12.0 * DEG) - sin(lat * DEG) * sin(delta)) / (cos(lat * DEG) * cos(delta));
        double o = (c < -1.0 || c > 1.0) ? omega : acos(c) * RAD;
        *nautical_dusk = noon + o / 15.0;
    }
    if (golden_dusk) {
        double c = (sin(6.0 * DEG) - sin(lat * DEG) * sin(delta)) / (cos(lat * DEG) * cos(delta));
        double o = (c < -1.0 || c > 1.0) ? omega : acos(c) * RAD;
        *golden_dusk = noon + o / 15.0;
    }
    if (astro_dusk) {
        /* Astronomical dusk (sun centre at -18 deg): true observing darkness. Unlike the tiers above
         * there is NO omega fallback — if the sun never dips to -18 (a summer "white night") we report
         * NAN so the caller can say "no astronomical dark tonight" rather than a fabricated time. */
        double c = (sin(-18.0 * DEG) - sin(lat * DEG) * sin(delta)) / (cos(lat * DEG) * cos(delta));
        *astro_dusk = (c < -1.0 || c > 1.0) ? NAN : noon + acos(c) * RAD / 15.0;
    }
    return 0;
}

double moon_phase(double unix_s) {
    double n = days_since_j2000(unix_s);
    double phase = fmod((n - 5.26) / 29.53059, 1.0);
    if (phase < 0.0) phase += 1.0;
    return phase;
}

double moon_illuminated_fraction(double phase) { return (1.0 - cos(2.0 * M_PI * phase)) / 2.0; }

int moon_phase_index(double phase) { return ((int)(phase * 8.0 + 0.5)) & 7; }

void maidenhead(double lat, double lon, char out[7]) {
    if (!isfinite(lat) || !isfinite(lon)) { strcpy(out, "----"); return; }
    lat = fmin(nextafter(90.0, -INFINITY), fmax(-90.0, lat));
    lon = fmin(nextafter(180.0, -INFINITY), fmax(-180.0, lon));
    double LON = lon + 180.0; /* [0,360) */
    double LAT = lat + 90.0;  /* [0,180) */

    int f0 = (int)(LON / 20.0);            if (f0 < 0) f0 = 0; if (f0 > 17) f0 = 17;
    int f1 = (int)(LAT / 10.0);            if (f1 < 0) f1 = 0; if (f1 > 17) f1 = 17;
    int s2 = (int)(fmod(LON, 20.0) / 2.0); if (s2 < 0) s2 = 0; if (s2 > 9) s2 = 9;
    int s3 = (int)(fmod(LAT, 10.0));       if (s3 < 0) s3 = 0; if (s3 > 9) s3 = 9;
    int u4 = (int)(fmod(LON, 2.0) * 12.0); if (u4 < 0) u4 = 0; if (u4 > 23) u4 = 23;
    int u5 = (int)(fmod(LAT, 1.0) * 24.0); if (u5 < 0) u5 = 0; if (u5 > 23) u5 = 23;

    out[0] = (char)('A' + f0);
    out[1] = (char)('A' + f1);
    out[2] = (char)('0' + s2);
    out[3] = (char)('0' + s3);
    out[4] = (char)('a' + u4);
    out[5] = (char)('a' + u5);
    out[6] = '\0';
}
