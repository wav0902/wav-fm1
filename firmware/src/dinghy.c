/* SPDX-License-Identifier: GPL-3.0-only */
/* ACID: a two-voice minimal synth for the FM-1, built on Dinghy. No presets, no saves: switch on, a raw saw, turn the knobs.
 *
 *   KEYS (all 27)       they play the voice PRESETS picked.
 *   LEAD                hold a chord and let go: the arpeggiator keeps going (latch). A fresh press after letting go replaces it.
 *   BASS                a drone: it rings until you press another key (HOME stops it). Moog-like, saturated, deep.
 *   PRESETS             left: the LEAD, right: the BASS (keys, knobs and volume act on that voice)
 *   KNOB 1..4           page A: CUTOFF, RESONANCE, ENV MOD, DECAY (303 style)
 *                       page B: DRIVE, GLIDE, DETUNE, DELAY       (FX switches the page). Turn fast: bigger steps.
 *   ALGORITHM           the volume of the picked voice          SELECT   tempo          MASTER   the original master volume
 *   ARP                 the next arpeggio pattern: UP, DOWN, UP-DN, RANDOM, OCT, CONV, DOUBLE, ZIGZAG
 *   PLAY                lead start / stop       HOME   stop the lead and the bass       OCT- OCT+   octave
 *
 * Integers only (no FPU, no libgcc). The tables are in acid_tables.h. */
#define APP_NAME "ACID"
static void app_idle(void) {}
#define LCD_IDLE app_idle
#include "platform.c"
#include "acid_tables.h"
#define KEY0 53u                                /* key 0 = F3 */

/* ---- shared with the audio ISR: one writer each (docs/RULES.md) */
static volatile uint32_t a_mask;                /* latched lead keys: bit k = key k */
static volatile uint32_t seq_cur = 255u;        /* the key the arpeggiator plays now (for the LEDs) */
static volatile uint32_t a_run = 1, a_mode;
static volatile uint32_t step_len = 6014u, gate_len = 3608u;   /* frames per 16th, frames the gate stays open */
static volatile int32_t oct_sh;                 /* -1..1 */
/* LEAD P[0..7], BASS P[8..15]: cutoff, reso, env mod, decay | drive, glide, detune, delay send */
static volatile uint32_t P[16] = {36, 90, 70, 40, 0, 0, 0, 0, 52, 55, 90, 35, 0, 0, 8, 0};
static volatile uint32_t VOL[2] = {127, 127};   /* lead, bass */
static volatile uint32_t b_inc, b_trig;
static volatile int32_t b_gate;

static const uint32_t INC_C9[12] = {            /* MIDI 120..131 (C9..B9); lower octaves shift right */
    815363807u, 863847862u, 915214929u, 969636441u, 1027294024u, 1088380105u,
    1153098554u, 1221665363u, 1294309365u, 1371273005u, 1452813141u, 1539201906u};
static uint32_t note_inc(uint32_t note) { return INC_C9[note % 12u] >> (10u - note / 12u); }

/* ---- the sound */
typedef struct { int32_t low, band; } svf_t;

static int32_t svf(svf_t *s, int32_t in, int32_t f, int32_t q)   /* Chamberlin state-variable low-pass */
{
    int32_t high;
    s->low += (int32_t)(((int64_t)f * s->band) >> 15);
    high = in - s->low - (int32_t)(((int64_t)q * s->band) >> 15);
    s->band += (int32_t)(((int64_t)f * high) >> 15);
    if (s->band > 262143) s->band = 262143; else if (s->band < -262143) s->band = -262143;
    if (s->low > 262143) s->low = 262143; else if (s->low < -262143) s->low = -262143;
    return s->low;
}

static int32_t soft(int32_t x)                  /* cubic soft clip: x - x^3 / 3 */
{
    int32_t t;
    if (x > 32767) x = 32767; else if (x < -32767) x = -32767;
    t = (int32_t)(((int64_t)x * x) >> 15);
    t = (int32_t)(((int64_t)t * x) >> 15);
    return x - ((t * 10923) >> 15);
}

static int32_t resq(uint32_t r)                 /* damping: nearly flat up to the middle, then it squeals */
{
    int32_t r2 = (int32_t)((r * r) >> 7);       /* 0..126 */
    return 52000 - r2 * 388;
}
static int32_t env2(int32_t e)                  /* a sharper pluck: the envelope squared, 0..126 */
{
    int32_t x = e >> 8;
    return (x * x) >> 7;
}

static uint32_t glide(uint32_t cur, uint32_t tgt, uint32_t g)   /* the phase step slides to its target (per block) */
{
    int32_t d;
    if (!g || !cur) return tgt;
    d = (int32_t)cur - (int32_t)tgt;
    return (uint32_t)((int32_t)tgt + (int32_t)(((int64_t)d * DEC_K[g >> 1]) >> 15));
}

#define DMASK 65535u                            /* one delay line of 1.5 s, shared by the lead and the bass (the pool) */
static int16_t dl[65536] __attribute__((section(".pool")));
static uint32_t dpos, dlen = 18042u;
static int32_t dlp;

static uint32_t a_seed = 2463534242u;
static uint32_t rnd(void) { a_seed ^= a_seed << 13; a_seed ^= a_seed >> 17; a_seed ^= a_seed << 5; return a_seed; }

static struct {                                 /* the line (touched only by the ISR) */
    uint32_t ph, ph2, inc, inc_t, inc2, tick, pos, trig;
    int32_t gate, amp, tgt, env_f, env_a, f, q;
    svf_t sv, sv2;
} A;
static struct {                                 /* the bass */
    uint32_t ph1, ph2, ph3, seen, inc, inc_t, inc2;
    int32_t amp, env, f, q;
    svf_t sv, sv2;
} B;

static void fire(uint32_t key, uint32_t up)     /* the lead plays a key (+ up semitones): pitch, gate, envelopes */
{
    A.inc_t = note_inc((uint32_t)((int32_t)(KEY0 + key) - 12 + 12 * oct_sh) + up);
    if (!P[5]) A.inc = A.inc_t;                 /* no glide: the new pitch at once */
    A.gate = 1;
    A.env_f = 32767;
    A.env_a = 32767;
}

static void arp_step(void)                      /* pick the next note of the latched chord, 2 octaves */
{
    uint32_t m = a_mask, n = 0, t = m, total, p, j, k;
    if (!m) return;
    while (t) { n += t & 1u; t >>= 1; }
    total = n * 2u;
    switch (a_mode) {
    case 1:  p = total - 1u - A.pos % total; break;
    case 2: {
        uint32_t per = total > 1u ? 2u * total - 2u : 1u, q = A.pos % per;
        p = q < total ? q : per - q;
        break; }
    case 3:  p = rnd() % total; break;
    case 4: { uint32_t i = A.pos % (2u * n); p = i / 2u + (i & 1u) * n; break; }                 /* each note, then its octave */
    case 5: { uint32_t i = A.pos % total; p = (i & 1u) ? total - 1u - i / 2u : i / 2u; break; }    /* converge: low, high, 2nd low ... */
    case 6:  p = (A.pos / 2u) % total; break;                                                      /* each note twice */
    case 7:  p = ((A.pos >> 1) + (A.pos & 1u) * 2u) % total; break;                                /* zigzag: 0 2 1 3 2 4 ... */
    default: p = A.pos % total; break;
    }
    A.pos++;
    j = p % n;
    for (k = 0; k < 27u; k++)                   /* the j-th key of the chord, from the lowest */
        if (m >> k & 1u) { if (j == 0) break; j--; }
    seq_cur = k;
    fire(k, 12u * (p / n));
}

static void app_audio(int32_t *out, uint32_t n)
{
    static uint32_t blk;
    uint32_t i;
    for (i = 0; i < n; i++) {
        int32_t s, la, ya, yb, m;
        if (blk == 0) {                         /* every 32 frames: envelopes, filter coefficients */
            int32_t idx, k, dec = (int32_t)P[3];
            A.env_f = (int32_t)(((int64_t)A.env_f * DEC_K[dec]) >> 15);
            k = dec + 25; if (k > 127) k = 127;
            A.env_a = (int32_t)(((int64_t)A.env_a * DEC_K[k]) >> 15);
            if (!A.gate) A.env_a = (A.env_a * 24000) >> 15;                /* the gate closed: quick release */
            idx = (int32_t)P[0] + ((env2(A.env_f) * (int32_t)P[2]) >> 7);
            if (idx > 127) idx = 127;
            A.f = CUT_F[idx];
            A.q = resq(P[1]);
            A.tgt = A.env_a;
            if (B.seen != b_trig) { B.seen = b_trig; B.env = 32767; }
            B.env = (int32_t)(((int64_t)B.env * DEC_K[P[11]]) >> 15);
            idx = (int32_t)P[8] + ((env2(B.env) * (int32_t)P[10]) >> 7);
            if (idx > 127) idx = 127;
            B.f = CUT_F[idx];
            B.q = resq(P[9]);
            A.inc = glide(A.inc, A.inc_t, P[5]);                           /* glide and detune, per block */
            A.inc2 = A.inc + (uint32_t)(((uint64_t)A.inc * P[6]) >> 11);   /* detune: 0..127 -> up to about a semitone */
            B.inc_t = b_inc;
            B.inc = glide(B.inc, B.inc_t, P[13]);
            B.inc2 = B.inc + (uint32_t)(((uint64_t)B.inc * P[14]) >> 11);
            dlen = 3u * step_len;                                          /* the delay: a dotted eighth */
            blk = 32;
        }
        blk--;

        /* the line's clock: a 16th, sample-accurate */
        if (a_run && a_mask) {
            if (A.tick == 0) arp_step();
            else if (A.tick == gate_len) A.gate = 0;
            if (++A.tick >= step_len) A.tick = 0;
        } else {
            A.tick = 0; A.gate = 0; A.pos = 0; seq_cur = 255u;
        }
        A.ph += A.inc; A.ph2 += A.inc2;
        if (!P[6]) A.ph2 = A.ph;                                          /* no detune: one saw */
        s = (((int32_t)A.ph >> 16) + ((int32_t)A.ph2 >> 16)) >> 1;       /* two saws */
        la = svf(&A.sv2, svf(&A.sv, s, A.f, 36000), A.f, A.q);   /* two stages: 24 dB/oct, the resonance on the second */
        if (la > 32767) la = 32767; else if (la < -32767) la = -32767;
        A.amp += (A.tgt - A.amp) >> 4;
        ya = (int32_t)(((int64_t)la * A.amp) >> 15);
        ya = soft((ya * (int32_t)(8u + P[1] / 16u + P[4] / 4u)) >> 3);  /* resonance and DRIVE: up to a hard, dirty clip */
        ya = (ya * (int32_t)(VOL[0] + 1u)) >> 7;

        /* the bass: saw + detuned saw + sub square, low-pass, then saturation */
        {
            int32_t g = b_gate, in, lb;
            if (B.amp < g) { B.amp += 600; if (B.amp > g) B.amp = g; }
            else if (B.amp > g) { B.amp -= 10; if (B.amp < g) B.amp = g; }
            if (B.amp) {
                B.ph1 += B.inc; B.ph2 += B.inc2; B.ph3 += B.inc;
                if (!P[14]) B.ph2 = B.ph1;
                in = ((((int32_t)B.ph1 >> 16) + ((int32_t)B.ph2 >> 16)) * 3) >> 3;
                in += (int32_t)B.ph3 < 0 ? 12000 : -12000;
                lb = svf(&B.sv2, svf(&B.sv, in, B.f, 36000), B.f, B.q);
                yb = soft((lb * (int32_t)(32u + P[12])) >> 3);            /* DRIVE */
                yb = (int32_t)(((int64_t)yb * B.amp * 7) >> 18);
                yb = (yb * (int32_t)(VOL[1] + 1u)) >> 7;
            } else
                yb = 0;
        }
        {
            int32_t wet = dl[(dpos - dlen) & DMASK], v;                    /* the delay: sends of both, a darker echo fed back */
            dlp += (wet - dlp) >> 2;
            v = ((ya * (int32_t)P[7] + yb * (int32_t)P[15]) >> 7) + ((dlp * 14746) >> 15);
            if (v > 32767) v = 32767; else if (v < -32767) v = -32767;
            dl[dpos & DMASK] = (int16_t)v;
            dpos++;
            m = ((ya + yb + wet) * 19) >> 5;
            if (m > 32767) m = 32767; else if (m < -32767) m = -32767;
        }
        out[2u * i] += m;
        out[2u * i + 1u] += m;
    }
}

/* ---- the screen: night blue, soft pink / violet / blue type, nothing else */
#define C_BG RGB(9, 11, 32)
#define C_LEAD RGB(255, 125, 205)
#define C_BASS RGB(112, 170, 255)
#define C_LAB RGB(176, 152, 255)
#define C_TRK RGB(30, 29, 68)
static const char *const KNOB_NAME[2][4] = {{"CUTOFF", "RESO", "ENV MOD", "DECAY"}, {"DRIVE", "GLIDE", "DETUNE", "DELAY"}};
static const char *const TITLE[4] = {"LEAD A", "LEAD B", "BASS A", "BASS B"};
static const char *const MODE_NAME[8] = {"UP", "DOWN", "UP-DN", "RANDOM", "OCT", "CONV", "DOUBLE", "ZIGZAG"};

static void box(uint32_t y, const felucca_font_t *f, const char *s, uint16_t c)    /* centred text on the night blue */
{
    cv_begin(240, f->h, C_BG);
    cv_text(((int32_t)240 - text_w(f, s)) / 2, 0, f, s, c);
    cv_blit(0, y);
    lcd_sync();
}

static void draw_knob(uint32_t row, uint32_t v, uint16_t col, const char *name)
{
    cv_begin(240, 24, C_BG);
    cv_text(12, 4, &FONT_S, name, C_LAB);
    cv_rect(104, 10, 124, 4, C_TRK);
    cv_rect(104, 10, (int32_t)(v * 124u / 127u), 4, col);
    cv_blit(0, (uint32_t)(60 + 30 * (int32_t)row));
}

static char *put(char *p, const char *s) { while (*s) *p++ = *s++; return p; }
static char *put_num(char *p, uint32_t v)
{
    if (v >= 100u) *p++ = (char)('0' + v / 100u);
    if (v >= 10u) *p++ = (char)('0' + v / 10u % 10u);
    *p++ = (char)('0' + v % 10u);
    return p;
}

static void draw_status(uint32_t bpm, uint32_t mode, int32_t oct, uint32_t run)
{
    char s[44], *p = s;
    p = put_num(p, bpm); p = put(p, "  ");
    p = put(p, MODE_NAME[mode]);
    p = put(p, run ? "  " : "  STOP  "); *p++ = oct < 0 ? '-' : '+'; *p++ = (char)('0' + (oct < 0 ? -oct : oct));
    *p = 0;
    box(212, &FONT_S, s, C_LAB);
}

/* ---- the main loop: no settings are kept; every start is the same raw saw */
static struct { uint8_t v[16], vol[2], bpm, mode, tgt, page; int8_t oct; } set;
static const uint8_t DEF[16] = {36, 90, 70, 40, 0, 0, 0, 0, 52, 55, 90, 35, 0, 0, 8, 0};

static void set_defaults(void)
{
    uint32_t i;
    for (i = 0; i < 16u; i++) set.v[i] = DEF[i];
    set.vol[0] = set.vol[1] = 127;
    set.bpm = 112; set.mode = 0; set.tgt = 0; set.page = 0; set.oct = 0;
}

static void apply_tempo(uint32_t bpm)
{
    uint32_t len = 661500u / bpm;               /* 44100 * 60 / (4 * bpm) */
    gate_len = len * 3u / 5u;
    step_len = len;
}

static uint32_t lowest(uint32_t m, uint32_t *k)
{
    uint32_t i;
    for (i = 0; i < 27u; i++) if (m >> i & 1u) { *k = i; return 1; }
    return 0;
}

static int32_t turn(uint32_t *t, int32_t e)     /* detents -> a step; turn fast and it gets bigger */
{
    int32_t step = (fm1_ms - *t < 90u || e > 1 || e < -1) ? 10 : 4;
    *t = fm1_ms;
    return step * e;
}

static void app_main(void)
{
    uint32_t prev = 0, drawn[5] = {~0u, ~0u, ~0u, ~0u, ~0u}, st_drawn = ~0u, view_drawn = ~0u;
    uint32_t kt[5] = {0, 0, 0, 0, 0}, k, b, rel, b_key = 99;
    int32_t e;

    set_defaults();
    for (k = 0; k < 16u; k++) P[k] = set.v[k];
    VOL[0] = set.vol[0]; VOL[1] = set.vol[1];
    a_mode = set.mode; oct_sh = set.oct;
    apply_tempo(set.bpm);

    lcd_fill(0, 0, 240, 240, C_BG);

    for (;;) {
        uint32_t keys, down, old, st;
        platform_poll();

        keys = fm1_in.notes;
        down = keys & ~prev;
        old = prev;
        prev = keys;

        if (down) {
            if (set.tgt == 0u) {                /* the lead */
                uint32_t m = old ? a_mask : 0u;     /* a chord latches; a fresh press after letting go replaces it */
                a_mask = m | down;
            } else {                            /* the bass is a drone: it rings until another key (the lowest of the new ones) */
                lowest(down, &k);
                b_key = k;
                b_inc = note_inc((uint32_t)((int32_t)KEY0 + (int32_t)k - 24 + 12 * set.oct));
                b_trig++;
                b_gate = 32767;
            }
        }

        b = fm1_input_edges(&rel);
        if (b & (1u << BTN_PLAY)) a_run ^= 1u;
        if (b & (1u << BTN_HOME)) { a_mask = 0; b_gate = 0; }
        if (b & (1u << BTN_FX)) set.page ^= 1u;
        if (b & (1u << BTN_ARP)) { set.mode = (uint8_t)((set.mode + 1u) & 7u); a_mode = set.mode; }
        if (b & (1u << BTN_OCT_DOWN) && set.oct > -1) { set.oct--; oct_sh = set.oct; }
        if (b & (1u << BTN_OCT_UP) && set.oct < 1) { set.oct++; oct_sh = set.oct; }

        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) set.tgt = e > 0 ? 1u : 0u;   /* right: the bass, left: the lead */
        for (k = 0; k < 4u; k++) {              /* the four knobs act on the page of the lead or the bass */
            uint8_t *v = &set.v[set.tgt * 8u + set.page * 4u + k];
            if ((e = fm1_enc_take(ENC_KNOB1 + k)) != 0) {
                int32_t x = (int32_t)*v + turn(&kt[k], e);
                *v = (uint8_t)(x < 0 ? 0 : x > 127 ? 127 : x);
            }
        }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {   /* the volume of the lead or the bass */
            int32_t x = (int32_t)set.vol[set.tgt] + turn(&kt[4], e);
            set.vol[set.tgt] = (uint8_t)(x < 0 ? 0 : x > 127 ? 127 : x);
        }
        for (k = 0; k < 16u; k++) P[k] = set.v[k];
        VOL[0] = set.vol[0]; VOL[1] = set.vol[1];
        if ((e = fm1_enc_take(ENC_SELECT)) != 0) {
            int32_t x = (int32_t)set.bpm + 2 * e;
            set.bpm = (uint8_t)(x < 60 ? 60 : x > 200 ? 200 : x);
            apply_tempo(set.bpm);
        }

        for (k = 0; k < 27u; k++) {             /* the held keys, the latched chord, the step that plays, the bass key glow */
            uint32_t on = (keys >> k & 1u) || (b_gate && k == b_key) || (a_mask >> k & 1u);
            fm1_led_key(KEY_LED(k), on);
        }
        fm1_led_key(BTN_PLAY, a_run);
        fm1_led_key(BTN_FX, set.page);

        if ((st = set.tgt * 2u + set.page) != view_drawn) {
            view_drawn = st;
            box(14, &FONT_L, TITLE[st], set.tgt ? C_BASS : C_LEAD);
            for (k = 0; k < 5u; k++) drawn[k] = ~0u;
        }
        for (k = 0; k < 5u; k++) {
            uint32_t v = k < 4u ? set.v[set.tgt * 8u + set.page * 4u + k] : set.vol[set.tgt];
            if (v != drawn[k]) { drawn[k] = v; draw_knob(k, v, set.tgt ? C_BASS : C_LEAD, k < 4u ? KNOB_NAME[set.page][k] : "VOLUME"); }
        }
        if ((st = set.bpm | (uint32_t)set.mode << 8 | (uint32_t)(set.oct + 1) << 12 | a_run << 16) != st_drawn) {
            st_drawn = st;
            draw_status(set.bpm, set.mode, set.oct, a_run);
        }
    }
}
