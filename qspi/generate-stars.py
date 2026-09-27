#!/usr/bin/env python3
"""
generate-stars.py  ->  output/stars.bin

Build the bright-star transit catalogue the clock reads from the QSPI flash (the CLOCK drive), mirroring
generate-tzrules.py. Source is the HYG database v4.1 (CC0), decimal J2000 RA (hours) / Dec (degrees) —
the same units the firmware wants. We keep the naked-eye "stars people actually recognise" (default
mag <= 2.5 -> 93 stars incl. the Megrez allow-list), magnitude-sorted so the firmware can early-stop at a `star_max_mag` config
knob. Names are 4-char, uppercase (the mk4-date 7-seg font is uppercase-only and has a glyph for every
letter; I/O/S/Z read as 1/0/5/2 and K/M/Q/V/W/X are rough approximations — flagged, not excluded).

FILE FORMAT (little-endian):
  header 16 B:  magic "MST1" (4) | count u16 | recordLength u16 (=14) | mag_scale u16 (=100) | 6 B zero
  record 14 B:  ra u16  (= round(ra_hours/24 * 65536))     -> firmware: ra_h = ra/65536*24
                dec i16  (= round(dec_deg * 100))            -> firmware: dec  = dec/100
                mag i16  (= round(mag * 100), load-filter)   -> dropped from RAM after the cut
                nm  char[4] (uppercase, space-padded, unique)
                pmra  i16 (mas/yr, mu_alpha* incl. cos-dec)  -> proper motion (alpha Cen drifts ~14 s
                pmdec i16 (mas/yr)                              of transit time by 2028 without it)
  (The firmware also accepts legacy 10-byte records — proper motion treated as zero.)
"""
import csv, os, struct, sys, urllib.request

HYG_URL = "https://raw.githubusercontent.com/astronexus/HYG-Database/main/hyg/CURRENT/hygdata_v41.csv"
HERE    = os.path.dirname(os.path.abspath(__file__))
CACHE   = os.path.join(HERE, "hyg_v41.csv")          # gitignored build cache
OUT     = os.path.join(HERE, "output", "stars.bin")
MAG_CUT = float(os.environ.get("STAR_MAG_CUT", "2.5"))
MAGIC   = b"MST1"
REC_LEN = 14
MAG_SCALE = 100

# --- 7-seg legibility (mk4-date lut_7seg is uppercase-only; every letter renders) ---
AMBIG = set("IOSZ")          # render but look like 1 0 5 2
ROUGH = set("KMQVWX")        # present but rough 7-seg approximations

def fetch_hyg():
    if not os.path.exists(CACHE):
        sys.stderr.write(f"downloading HYG v4.1 -> {CACHE} ...\n")
        urllib.request.urlretrieve(HYG_URL, CACHE)
    return CACHE

def clean_name(s):
    """Uppercase, keep A-Z0-9 only."""
    return "".join(c for c in s.upper() if c.isalnum())

# Curated overrides (review-driven, famous-stars scope):
#  - Mirach's natural stem "MIRA" IMPERSONATES Mira (omicron Ceti, itself famous and not on the card).
#  - Markab (alpha Peg, Great Square corner) deserves "MARK"; obscure Markeb (kappa Vel) sorted one
#    place earlier by magnitude and used to steal it, leaving Markab an unrecognizable "MARB".
NAME_OVERRIDE = { 'Mirach': 'MRCH', 'Markab': 'MARK', 'Markeb': 'MARB' }
# Asterism-completing showpieces admitted ABOVE the magnitude cut (famous-only scope intact):
# Megrez (mag 3.3) is the 7th star of the Big Dipper — without it the most-pointed-at northern
# asterism is forever one star short.
ALWAYS_INCLUDE = { 'Megrez' }

def abbrev(proper, bayer, con, used):
    """A unique 4-char uppercase name. Curated override first; else prefer the proper name; fall back to Bayer(greek)+con."""
    if proper in NAME_OVERRIDE:
        nm = NAME_OVERRIDE[proper]
        if nm in used:
            raise RuntimeError(f"override collision: {proper} -> {nm}")
        return nm
    GREEK = {  # HYG 3-letter Bayer prefix -> single display letter
        'Alp':'A','Bet':'B','Gam':'G','Del':'D','Eps':'E','Zet':'Z','Eta':'H','The':'T','Iot':'I',
        'Kap':'K','Lam':'L','Mu':'M','Nu':'N','Xi':'X','Omi':'O','Pi':'P','Rho':'R','Sig':'S',
        'Tau':'U','Ups':'U','Phi':'F','Chi':'C','Psi':'Y','Ome':'O'}
    cands = []
    if proper:
        base = clean_name(proper.split()[0])          # first word, e.g. "Rigil Kentaurus" -> RIGIL
        if len(base) >= 2:
            cands.append(base[:4])
            cands.append((base[:3] + base[-1]) if len(base) > 4 else base[:4])
            cands.append(base[:2] + base[-2:])
    if bayer and con:
        g = GREEK.get(bayer.split('-')[0])
        if g:
            cands.append((g + clean_name(con))[:4])
    for c in cands:
        c = (c + "    ")[:4].strip()
        if c and c not in used:
            return c
    # last resort: proper/bayer stem + a disambiguating digit
    stem = (cands[0] if cands else "STR")[:3]
    for d in "23456789":
        c = (stem + d)[:4]
        if c not in used:
            return c
    raise RuntimeError(f"cannot uniquely name {proper or bayer!r}")

def build():
    rows = [r for r in csv.DictReader(open(fetch_hyg())) if r['id'] != '0' and r['mag']]
    # drop secondary components of multiple-star systems (comp != 1): they duplicate the primary's
    # position, so they'd transit at the same instant (e.g. Capella's mag-0.96 component, or Toliman
    # = alpha Cen B sitting on Rigil Kentaurus). Keep only the primary / single stars.
    rows = [r for r in rows if (not r['comp']) or r['comp'] == '1']
    stars = [r for r in rows if float(r['mag']) <= MAG_CUT or r['proper'] in ALWAYS_INCLUDE]
    stars.sort(key=lambda r: float(r['mag']))          # brightest first -> firmware early-stop by mag

    used, out, report = set(), [], []
    for r in stars:
        nm = abbrev(r['proper'], r['bayer'], r['con'], used)
        used.add(nm)
        ra_h = float(r['ra']) % 24.0
        ra_u = round(ra_h / 24.0 * 65536.0) & 0xFFFF
        dec_i = max(-9000, min(9000, round(float(r['dec']) * 100)))
        mag_i = round(float(r['mag']) * MAG_SCALE)
        pmra  = max(-32768, min(32767, round(float(r['pmra']  or 0))))   # mas/yr (HYG: mu_alpha*)
        pmdec = max(-32768, min(32767, round(float(r['pmdec'] or 0))))
        out.append((ra_u, dec_i, mag_i, nm, pmra, pmdec))
        flags = ''.join(sorted(set(nm) & (AMBIG | ROUGH)))
        report.append((r['proper'] or ('*' + (r['bayer'] or '')), nm, float(r['mag']), flags))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "wb") as f:
        f.write(MAGIC + struct.pack("<HHH6x", len(out), REC_LEN, MAG_SCALE))
        for ra_u, dec_i, mag_i, nm, pmra, pmdec in out:
            f.write(struct.pack("<HhH", ra_u, dec_i, mag_i & 0xFFFF) + nm.encode('ascii').ljust(4, b' ')
                    + struct.pack("<hh", pmra, pmdec))

    # --- self-verify: sizes, uniqueness, round-trip a couple of records ---
    size = os.path.getsize(OUT)
    assert size == 16 + len(out) * REC_LEN, f"size {size} != header+records"
    assert len({rec[3] for rec in out}) == len(out), "duplicate names!"
    for nm in (rec[3] for rec in out):
        assert 1 <= len(nm) <= 4, f"bad name {nm!r}"
    sys.stderr.write(f"OK  stars.bin: {len(out)} stars, {size} B (mag<={MAG_CUT})\n")

    # legibility report to stderr (nothing excluded — just eyeball the rough ones)
    rough = [x for x in report if x[3]]
    sys.stderr.write(f"    names using ambiguous(IOSZ)/rough(KMQVWX) glyphs: {len(rough)}/{len(out)}\n")
    for proper, nm, mag, fl in report:
        print(f"  {mag:5.2f}  {nm:4}  {proper}{('   <'+fl+'>') if fl else ''}")

if __name__ == "__main__":
    build()
