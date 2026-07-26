/* Native unit test for ../Core/Src/astro.c against the reference vectors used to
 * develop the astro pack. Not part of the firmware build (test/ is not a project
 * source path); build & run on a host:
 *
 *   cc -std=c99 -O2 -I ../Core/Inc ../Core/Src/astro.c test_astro.c -lm -o test_astro && ./test_astro
 */
#include "astro.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int fails = 0, total = 0;

static void chk(const char *name, double got, double exp, double tol) {
    total++;
    double d = fabs(got - exp);
    if (d > tol || !isfinite(got)) {
        printf("  FAIL %-28s got %.6f  exp %.6f  (|d|=%.6f > %.6f)\n", name, got, exp, d, tol);
        fails++;
    } else {
        printf("  ok   %-28s %.6f  (|d|=%.2e)\n", name, got, d);
    }
}

/* astro_dusk is deliberately NAN when the sun never reaches -18 deg (white night),
 * which is what lets MODE_DARK report darkness as absent rather than inventing one. */
static void chknan(const char *name, double got) {
    total++;
    if (isnan(got)) printf("  ok   %-28s nan (no astronomical darkness)\n", name);
    else            { printf("  FAIL %-28s got %.6f  exp nan\n", name, got); fails++; }
}

static void chks(const char *name, const char *got, const char *exp) {
    total++;
    if (strcmp(got, exp) != 0) { printf("  FAIL %-28s got \"%s\"  exp \"%s\"\n", name, got, exp); fails++; }
    else                        printf("  ok   %-28s \"%s\"\n", name, got);
}

struct loc { const char *tag; double lat, lon, t; };

int main(void) {
    struct loc V1 = {"Greenwich", 51.4779, -0.0015, 1718971200}; /* 2024-06-21 12:00 */
    struct loc V2 = {"Sydney", -33.8568, 151.2153, 1704067200};  /* 2024-01-01 00:00 */
    struct loc V3 = {"Quito", -0.1807, -78.4678, 1411819200};    /* 2014-09-27 12:00 */
    struct loc V4 = {"Fairbanks", 64.8378, -147.7164, 1671460200}; /* 2022-12-19 14:30 */
    struct loc *V[4] = {&V1, &V2, &V3, &V4};

    double az, el;
    printf("sun_az_el:\n");
    double exp_el[4] = {61.9537, 61.9872, 13.7817, -29.3334};
    double exp_az[4] = {179.0572, 75.1095, 91.7169, 82.8687};
    for (int i = 0; i < 4; i++) {
        char nm[40];
        sun_az_el(V[i]->lat, V[i]->lon, V[i]->t, &az, &el);
        sprintf(nm, "el %s", V[i]->tag); chk(nm, el, exp_el[i], 0.001);
        sprintf(nm, "az %s", V[i]->tag); chk(nm, az, exp_az[i], 0.001);
    }

    printf("equation_of_time (min):\n");
    double exp_eot[4] = {-1.92784, -3.09298, 8.99946, 2.88245};
    for (int i = 0; i < 4; i++) {
        char nm[40]; sprintf(nm, "eot %s", V[i]->tag);
        chk(nm, equation_of_time(V[i]->t), exp_eot[i], 0.0005);
    }

    printf("moon_phase / illuminated:\n");
    double exp_ph[4] = {0.49108, 0.64968, 0.10744, 0.86985};
    double exp_il[4] = {0.99921, 0.79471, 0.10966, 0.15807};
    int exp_idx[4] = {4, 5, 1, 7};
    for (int i = 0; i < 4; i++) {
        char nm[40]; double p = moon_phase(V[i]->t);
        sprintf(nm, "phase %s", V[i]->tag); chk(nm, p, exp_ph[i], 0.0002);
        sprintf(nm, "illum %s", V[i]->tag); chk(nm, moon_illuminated_fraction(p), exp_il[i], 0.0005);
        sprintf(nm, "idx %s", V[i]->tag); chk(nm, moon_phase_index(p), exp_idx[i], 0.0);
    }

    printf("sun_times (UTC h):\n");
    double rise, set, noon, civ, nau, gold, ast;
    /* V1 Greenwich — 21 June at 51.5N, the sun never reaches -18 deg */
    sun_times(V1.lat, V1.lon, V1.t, &rise, &set, &noon, &civ, &nau, &gold, &ast);
    chk("noon V1", noon, 12.032231, 0.0003);
    chk("rise V1", rise, 3.715903, 0.0003);
    chk("set  V1", set, 20.348558, 0.0003);
    chk("civil V1", civ, 21.143501, 0.0003);
    chk("naut V1", nau, 22.383822, 0.0003);
    chknan("astro V1", ast);
    /* V3 Quito */
    sun_times(V3.lat, V3.lon, V3.t, &rise, &set, &noon, NULL, NULL, NULL, NULL);
    chk("noon V3", noon, 17.081196, 0.0003);
    chk("rise V3", rise, 11.025277, 0.0003);
    chk("set  V3", set, 23.137114, 0.0003);
    /* V4 Fairbanks (events spill past midnight) */
    sun_times(V4.lat, V4.lon, V4.t, &rise, &set, &noon, &civ, &nau, &gold, &ast);
    chk("noon V4", noon, 21.798861, 0.0005);
    chk("rise V4", rise, 19.944940, 0.0005);
    chk("set  V4", set, 23.652783, 0.0005);
    chk("civil V4", civ, 25.076604, 0.0005);
    chk("naut V4", nau, 26.273118, 0.0005);
    chk("astro V4", ast, 27.301764, 0.0005);

    printf("maidenhead:\n");
    char g[7];
    maidenhead(V1.lat, V1.lon, g); chks("grid V1", g, "IO91xl");
    maidenhead(V2.lat, V2.lon, g); chks("grid V2", g, "QF56od");
    maidenhead(V3.lat, V3.lon, g); chks("grid V3", g, "FI09st");
    maidenhead(V4.lat, V4.lon, g); chks("grid V4", g, "BP64du");
    maidenhead(51.508, -0.128, g); chks("Trafalgar Sq", g, "IO91wm");
    maidenhead(41.714, -72.728, g); chks("ARRL HQ", g, "FN31pr");
    maidenhead(-41.283, 174.745, g); chks("Wellington", g, "RE78ir");
    maidenhead(1.0 / 0.0, 0.0, g); chks("non-finite", g, "----");

    printf("sun_subsolar:\n");
    /* Self-consistency (non-circular vs the sun_az_el vectors above): the sun must be
     * at the zenith of its own subsolar point — elevation 90 deg at every test instant. */
    for (int i = 0; i < 4; i++) {
        double slat, slon, e;
        sun_subsolar(V[i]->t, &slat, &slon);
        sun_az_el(slat, slon, V[i]->t, NULL, &e);
        char nm[40]; sprintf(nm, "zenith %s", V[i]->tag);
        chk(nm, e, 90.0, 0.01);
    }
    /* Declination anchors: June solstice ~ +23.44, deep northern winter ~ -23. */
    {
        double slat;
        sun_subsolar(V1.t, &slat, NULL); chk("decl solstice", slat, 23.436, 0.05);
        sun_subsolar(V2.t, &slat, NULL); chk("decl jan 1",   slat, -23.06, 0.10);
    }

    printf("local_sidereal_time (h):\n");
    /* GMST at J2000.0 = 18.697374558 h (IAU); Meeus ex. 12.b, 1987-04-10 19:21:00 UT
     * -> mean GMST 8h34m57.1s = 8.582525 h. LST = GMST + lon/15 checks shift + wrap. */
    chk("LST J2000 lon0",        local_sidereal_time(946728000.0,   0.0), 18.697375, 0.0001);
    chk("LST Meeus lon0",        local_sidereal_time(545080860.0,   0.0),  8.582525, 0.0001);
    chk("LST J2000 lon -75",     local_sidereal_time(946728000.0, -75.0), 13.697375, 0.0001);
    chk("LST J2000 lon +90wrap", local_sidereal_time(946728000.0,  90.0),  0.697375, 0.0001);

    printf("local_solar_time (h):\n");
    /* At meridian transit apparent solar time is 12:00 exactly. Build that instant from
     * sun_times()'s solar_noon and confirm -- catches any longitude/EoT sign or wrap error. */
    for (int i = 0; i < 4; i++) {
        double nn;
        sun_times(V[i]->lat, V[i]->lon, V[i]->t, NULL, NULL, &nn, NULL, NULL, NULL, NULL);
        double noon_unix = trunc(V[i]->t / 86400.0) * 86400.0 + nn * 3600.0;
        char nm[40]; sprintf(nm, "solar@noon %s", V[i]->tag);
        chk(nm, local_solar_time(noon_unix, V[i]->lon), 12.0, 0.02);
    }

    printf("\n%d/%d passed, %d failed\n", total - fails, total, fails);
    return fails ? 1 : 0;
}
