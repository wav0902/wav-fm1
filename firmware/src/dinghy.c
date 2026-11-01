/* SPDX-License-Identifier: GPL-3.0-only */
/* ACID: a two-voice minimal synth for the FM-1, built on Dinghy. No presets, no saves: switch on, a raw saw, turn the knobs.
 *
 *   KEYS (all 27)       they play the voice PRESETS picked.
 *   LEAD                hold a chord and let go: the arpeggiator keeps going (latch). A fresh press after letting go replaces it.
 *   BASS                a drone: it rings until you press another key (HOME stops it). Moog-like, saturated, deep.
 *   PRESETS             left: the LEAD, right: the BASS (keys, knobs and volume act on that voice)
 *   KNOB 1..4           page A: CUTOFF, RESONANCE, ENV MOD, DECAY (303 style)
 *                       page B: GLIDE, FM, RING, VOWEL      page C: DRIVE, DELAY, WARP, CRUSH
 *   SCREEN              ARP WAV 09 neon logo at start (neon_img.h), neon titles, battery top right
 *                       (FX switches the page A, B, C). Turn fast: bigger steps.
 *   ALGORITHM           the volume of the picked voice          SELECT   tempo          MASTER   the original master volume
 *   ARP                 the next arpeggio pattern: UP, UP-DN, BOUNCE, OFFBEAT (on DRUM: the next drum pattern)
 *   DRUM                PRESETS all the way right: a kit with no sequencer. Page A kick, clap, hat, open; page B tune, decay, delay, crush
 *   PLAY                lead start / stop       HOME   stop the lead and the bass       OCT- OCT+   octave
 *
 * Integers only (no FPU, no libgcc). The tables are in acid_tables.h. */
#define APP_NAME "ARP WAV 09"
static void app_idle(void) {}
#define LCD_IDLE app_idle
#include "platform.c"
#include "acid_tables.h"
#include "neon_img.h"                           /* the neon logo and the six titles, drawn at build time */
#define KEY0 53u                                /* key 0 = F3 */

/* ---- shared with the audio ISR: one writer each (docs/RULES.md) */
static volatile uint32_t a_mask;                /* latched lead keys: bit k = key k */
static volatile uint32_t seq_cur = 255u;        /* the key the arpeggiator plays now (for the LEDs) */
static volatile uint32_t a_run = 1, a_mode;
static volatile uint32_t step_len = 6014u, gate_len = 3608u;   /* frames per 16th, frames the gate stays open */
static volatile int32_t oct_sh;                 /* -1..1 */
/* LEAD P[0..11], BASS P[12..23]; per voice: page A cutoff, reso, env mod, decay | page B glide, fm, ring, vowel | page C drive, delay send, warp, crush */
/* DRUMS P[24..35]: page A kick, clap, hat, open hat (levels) | page B tune, decay, delay, crush. 16-step patterns, no sequencer: you mix the kit */
static volatile uint32_t P[36] = {30, 25, 50, 45, 0, 0, 0, 0, 0, 0, 0, 0, 70, 30, 70, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 0, 0, 0, 0, 0, 0, 0};
static volatile uint32_t VOL[3] = {127, 127, 127};   /* lead, bass, drums */
static volatile uint32_t d_pat, d_off, a_restart;   /* the drum pattern, drums stopped by HOME, a chord began from nothing: restart the bar */
static volatile uint32_t b_inc, b_trig;
static volatile int32_t b_gate;
static volatile uint32_t d_len = 6014u;         /* frames per drum 16th: the drums have a tempo of their own */

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

#define DMASK 32767u                            /* a ping-pong delay: a left and a right line of 0.74 s, shared by the lead and the bass (the pool) */
static int16_t dl[32768] __attribute__((section(".pool")));
static int16_t dr[32768] __attribute__((section(".pool")));
static uint32_t dpos, dlen = 18042u, dq = 18042u << 8;   /* the wanted delay time; the time now, in 1/256 sample (it glides, like a tape) */
static int32_t dlpl, dlpr, wph;

static struct {                                 /* the line (touched only by the ISR) */
    uint32_t ph, mp, rp, inc, incb, inc_t, tick, pos, trig, len, gl;
    int32_t gate, amp, tgt, env_f, env_a, f, q, f1, f2, bm;
    svf_t sv, sv2, fa, fb;
} A;
static struct {                                 /* the bass */
    uint32_t ph1, ph2, ph3, mp, rp, seen, inc, incb, inc_t, inc2;
    int32_t amp, env, f, q, f1, f2, bm;
    svf_t sv, sv2, fa, fb;
} B;

/* ---- effects per voice, integers only. CRUSH: from zero up, fewer bits and a lower sample rate together. */
typedef struct { int32_t hold; uint32_t cnt; } fx_t;
static fx_t FXL, FXB;
static volatile uint32_t load_peak;             /* the longest app_audio call, in timer ticks */
static volatile uint32_t sp_hum, sp_pop;        /* the neon sign at start: the hum of the tube, and a crackle (set by the splash, played by the ISR) */
static uint32_t sp_ph, sp_seed = 1u, sp_env;

static int32_t fx_run(fx_t *f, int32_t x, int32_t crush)
{
    if (crush) {
        uint32_t sh = 6u + (((uint32_t)crush * 6u) >> 7), n = 1u + (((uint32_t)crush * (uint32_t)crush) >> 8);
        x = (x >> sh) * (int32_t)(1u << sh);    /* fewer bits (8 bits at once, 3 at the end) */
        if (++f->cnt >= n) { f->cnt = 0; f->hold = x; }          /* and a lower sample rate: hold every n-th sample */
        x = f->hold;
    }
    return x;
}

/* VOWEL: two resonant band-passes that glide through a e i o u (as filter indices, see CUT_F): the voice starts to talk */
static const uint8_t VOW1[5] = {71, 54, 47, 59, 50}, VOW2[5] = {79, 93, 99, 71, 68};
static void vowel_f(uint32_t v, int32_t *f1, int32_t *f2)
{
    uint32_t seg = v >> 5, fr = v & 31u;
    if (seg > 3u) { seg = 3u; fr = 32u; }
    *f1 = CUT_F[VOW1[seg] + (((int32_t)VOW1[seg + 1] - (int32_t)VOW1[seg]) * (int32_t)fr >> 5)];
    *f2 = CUT_F[VOW2[seg] + (((int32_t)VOW2[seg + 1] - (int32_t)VOW2[seg]) * (int32_t)fr >> 5)];
}
static int32_t vowel(svf_t *a, svf_t *b, int32_t x, int32_t f1, int32_t f2, int32_t v)
{
    int32_t y, m = v * 4 > 127 ? 127 : v * 4;
    svf(a, x, f1, 5000); svf(b, x, f2, 5000);
    y = (a->band + b->band) >> 1;
    return (x * (128 - m) + y * m) >> 7;
}
/* RING: the voice times a triangle that runs 2.76 times its pitch (inharmonic: metal, robots) */
static int32_t rwv;                             /* the carrier's warble: a 5.5 Hz triangle, +-16384 */
static uint32_t ring_inc(uint32_t incb, int32_t m)     /* the carrier's step: 2.76 x the pitch, bent up and down by the warble (more with RING) */
{
    (void)m; (void)rwv;
    return incb * 2u + (uint32_t)(((uint64_t)incb * 195u) >> 8);
}
static int32_t ringmod(int32_t x, uint32_t rp, int32_t m)
{
    int32_t t = (int32_t)rp >> 16, car;
    t = (t < 0 ? -t : t) - 16384;
    car = t * 4;
    return (x * (128 - m) + (int32_t)(((int64_t)x * car) >> 15) * m) >> 7;
}

static void fire(uint32_t key, uint32_t up)     /* the lead plays a key (+ up semitones): pitch, gate, envelopes */
{
    A.inc_t = note_inc((uint32_t)((int32_t)(KEY0 + key) - 12 + 12 * oct_sh) + up);
    if (!P[4]) A.inc = A.inc_t;                 /* no glide: the new pitch at once */
    A.gate = 1;
    A.env_f = 32767;
    A.env_a = 32767;
}

static void arp_step(void)                      /* the next step of the latched chord (two octaves), 4 patterns */
{
    uint32_t m = a_mask, n = 0, t = m, up, j, k, s;
    if (!m) return;
    while (t) { n += t & 1u; t >>= 1; }
    s = A.pos++;
    {                                           /* UP-DN: up through the chord and its octave, and back down; DN-UP starts from the top */
        uint32_t per = 4u * n - 2u, q = s % per, i = q < 2u * n ? q : per - q;
        if (a_mode) i = 2u * n - 1u - i;
        j = i % n; up = 12u * (i / n);
    }
    for (k = 0; k < 27u; k++)                   /* the j-th key of the chord, from the lowest */
        if (m >> k & 1u) { if (j == 0) break; j--; }
    seq_cur = k;
    fire(k, up);
}

/* ---- the drums, an 80s pair: kick (a sine that falls) and snare (a body and bright noise) */
static const uint16_t DK[4] = {0x1111, 0x1151, 0x0911, 0x1111};      /* kick, per pattern: FLOOR CHASE PULSE */
static const uint16_t DS[4] = {0x1010, 0x1010, 0x1010, 0x1010};      /* snare */
static struct { uint32_t tick, step, kph, sph, seed, cl; int32_t kenv, kamp, senv, stn, hlp; fx_t fx; } D;

static int32_t drums(void)
{
    int32_t y = 0, nz, k, hp = 0;
    uint32_t st;
    if (a_restart) { a_restart = 0; D.tick = 0; D.step = 0; }
    if (D.tick == 0 && a_run && !d_off) {                              /* a 16th: which drums hit */
        st = D.step; D.step = (D.step + 1u) & 15u;
        if (DK[d_pat & 3u] >> st & 1u) { D.kenv = 32767; D.kamp = 32767; D.kph = 0; D.cl = 64u; }
        if (DS[d_pat & 3u] >> st & 1u) { D.senv = 32767; D.stn = 32767; D.sph = 0; }
    }
    if (++D.tick >= d_len) D.tick = 0;
    D.seed = D.seed * 1664525u + 1013904223u;
    nz = (int32_t)D.seed >> 17;                                        /* white noise +-16384 */
    if (D.senv) { D.hlp += (nz - D.hlp) >> 1; hp = nz - D.hlp; }       /* high-passed noise */
    if (D.kamp) {
        uint32_t f = 97392u * 56u + (uint32_t)(((uint64_t)97392u * 120u * (uint32_t)D.kenv) >> 15);   /* 56 Hz, with a fast fall from +120 Hz, and a click: a punchy drum-machine kick */
        int32_t t;
        D.kph += f;
        t = (int32_t)D.kph >> 16; t = (t < 0 ? -t : t) * 2 - 32768;   /* a triangle, rounded by the soft clip into a sine */
        k = soft(t);
        k = (int32_t)(((int64_t)k * D.kamp) >> 15);
        y += (k * (int32_t)P[24] * 2) >> 8;
        if (D.cl) { y += (int32_t)(((int64_t)nz * (int32_t)D.cl) >> 6) * (int32_t)P[24] >> 10; D.cl--; }
        D.kenv -= (D.kenv >> 8) + (D.kenv ? 1 : 0);
        D.kamp -= (D.kamp >> 12) + 1;
        if (D.kamp < 0) D.kamp = 0;
    }
    if (D.senv || D.stn) {
        int32_t t, sn;
        D.sph += 97392u * 205u;
        t = (int32_t)D.sph >> 16; t = (t < 0 ? -t : t) * 2 - 32768;
        sn = (int32_t)(((int64_t)soft(t) * D.stn) >> 16) + (int32_t)(((int64_t)hp * D.senv) >> 15);   /* the body and the noise */
        y += (sn * (int32_t)P[25]) >> 6;
        D.stn -= (D.stn >> 9) + (D.stn ? 1 : 0); if (D.stn < 0) D.stn = 0;
        D.senv -= (D.senv >> 12) + (D.senv ? 1 : 0); if (D.senv < 0) D.senv = 0;
    }
    y = fx_run(&D.fx, y, (int32_t)P[27]);                              /* CRUSH */
    return (y * (int32_t)(VOL[2] + 1u)) >> 7;
}

static void app_audio(int32_t *out, uint32_t n)
{
    static uint32_t blk, dfb = 14746u;
    uint32_t i, t0 = fm1_ticks();
    for (i = 0; i < n; i++) {
        int32_t s, la, ya, yb, yd;
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
            B.env = (int32_t)(((int64_t)B.env * DEC_K[P[15]]) >> 15);
            idx = (int32_t)P[12] + ((env2(B.env) * (int32_t)P[14]) >> 7);
            if (idx > 127) idx = 127;
            B.f = CUT_F[idx];
            B.q = resq(P[13]);
            {
                uint32_t bar = 16u * step_len;                             /* WARP: at the top of each bar the pitch dives, then climbs back */
                int32_t fr = 32767 - (int32_t)((wph >> 8) * 32767u / (bar >> 8)), fa, fb;
                if (fr < 0) fr = 0;
                fa = 32768 - (((fr * (int32_t)P[7]) >> 7) >> 1);
                fb = 32768 - (((fr * (int32_t)P[19]) >> 7) >> 1);
                A.inc = glide(A.inc, A.inc_t, P[4]);
                A.incb = (uint32_t)(((uint64_t)A.inc * (uint32_t)fa) >> 15);
                B.inc_t = b_inc;
                B.inc = glide(B.inc, B.inc_t, P[16]);
                B.incb = (uint32_t)(((uint64_t)B.inc * (uint32_t)fb) >> 15);
                B.inc2 = B.incb + (uint32_t)(((uint64_t)B.incb * 8u) >> 11);   /* the bass keeps a slight fixed detune */
                vowel_f(P[10], &A.f1, &A.f2);
                vowel_f(P[22], &B.f1, &B.f2);
            }
            dlen = 3u * step_len;                                          /* the delay: a dotted eighth (halved until it fits, at very slow tempi) */
            while (dlen > 32000u) dlen >>= 1;
            dfb = 11000u + 190u * (P[6] > P[18] ? (P[6] > P[26] ? P[6] : P[26]) : (P[18] > P[26] ? P[18] : P[26]));           /* the more you send, the longer it rings: 33% .. 107% (it runs away at the top) */
            { static uint32_t rl; int32_t q; rl += 32u * 535656u; q = (int32_t)rl >> 16; rwv = (q < 0 ? -q : q) - 16384; }
            blk = 32;
        }
        blk--;
        if (++wph >= (int32_t)(16u * step_len)) wph = 0;

        /* the line's clock: a 16th, sample-accurate */
        if (a_run && a_mask) {
            if (A.tick == 0) arp_step();
            else if (A.tick == gate_len) A.gate = 0;
            if (++A.tick >= step_len) A.tick = 0;
        } else {
            A.tick = 0; A.gate = 0; A.pos = 0; seq_cur = 255u;
        }
        yd = drums();
        {
            uint32_t c;
            A.ph += A.incb; c = A.ph;
            if (P[8]) {                                                   /* FM: an inharmonic triangle (x1.41) bends the phase */
                int32_t x;
                A.mp += A.incb + (uint32_t)(((uint64_t)A.incb * 106u) >> 8);
                x = (int32_t)A.mp >> 16; x = (x < 0 ? -x : x) - 16384;
                c += (uint32_t)(x * (int32_t)P[8]) << 11;
            }
            s = (int32_t)c >> 16;
        }
        la = svf(&A.sv2, svf(&A.sv, s, A.f, 36000), A.f, A.q);   /* two stages: 24 dB/oct, the resonance on the second */
        if (la > 32767) la = 32767; else if (la < -32767) la = -32767;
        if (P[10]) la = vowel(&A.fa, &A.fb, la, A.f1, A.f2, (int32_t)P[10]);
        if (P[9]) { A.rp += ring_inc(A.incb, (int32_t)P[9]); la = ringmod(la, A.rp, (int32_t)P[9]); }
        A.amp += (A.tgt - A.amp) >> 4;
        ya = (int32_t)(((int64_t)la * A.amp) >> 15);
        ya = soft((ya * (int32_t)(7u + P[1] / 16u + P[5] / 3u)) >> 3);  /* resonance and DRIVE: up to a hard, dirty clip */
        if (P[5]) ya = soft(ya + ((ya * (int32_t)P[5]) >> 6));            /* a second stage: squarer and dirtier the more you turn */
        ya = fx_run(&FXL, ya, (int32_t)P[11]);
        ya = (ya * (int32_t)(VOL[0] + 1u)) >> 7;

        /* the bass: saw + detuned saw + sub square, low-pass, then saturation */
        {
            int32_t g = b_gate, in, lb;
            if (B.amp < g) { B.amp += 600; if (B.amp > g) B.amp = g; }
            else if (B.amp > g) { B.amp -= 10; if (B.amp < g) B.amp = g; }
            if (B.amp) {
                uint32_t c1, c2;
                B.ph1 += B.incb; B.ph2 += B.inc2; B.ph3 += B.incb;
                c1 = B.ph1; c2 = B.ph2;
                if (P[20]) {                                              /* FM */
                    int32_t x;
                    uint32_t off;
                    B.mp += B.incb + (uint32_t)(((uint64_t)B.incb * 106u) >> 8);
                    x = (int32_t)B.mp >> 16; x = (x < 0 ? -x : x) - 16384;
                    off = (uint32_t)(x * (int32_t)P[20]) << 11;
                    c1 += off; c2 += off;
                }
                in = ((((int32_t)c1 >> 16) + ((int32_t)c2 >> 16)) * 3) >> 3;
                in += (int32_t)B.ph3 < 0 ? 12000 : -12000;
                lb = svf(&B.sv2, svf(&B.sv, in, B.f, 36000), B.f, B.q);
                if (P[22]) lb = vowel(&B.fa, &B.fb, lb, B.f1, B.f2, (int32_t)P[22]);
                if (P[21]) { B.rp += ring_inc(B.incb, (int32_t)P[21]); lb = ringmod(lb, B.rp, (int32_t)P[21]); }
                yb = soft((lb * (int32_t)(8u + ((P[17] * 150u) >> 7))) >> 3);   /* DRIVE: clean at zero, the same fury at the top */
                if (P[17]) yb = soft(yb + ((yb * (int32_t)P[17]) >> 6));
                yb = fx_run(&FXB, yb, (int32_t)P[23]);
                yb = (int32_t)(((int64_t)yb * B.amp * 7) >> 18);
                yb = (yb * (int32_t)(VOL[1] + 1u)) >> 7;
            } else
                yb = 0;
        }
        {
            int32_t wl, wr, vl, vr, a0, a1, ml, mr, dry = ya + yb + yd;        /* the delay: ping-pong, the echo bounces left, right, left... */
            uint32_t dt = dlen << 8, di, df;
            if (dq < dt) { dq += 64u; if (dq > dt) dq = dt; }              /* a new tempo: the echo slides to its new time (up to 25% pitch bend) */
            else if (dq > dt) { dq -= 64u; if (dq < dt) dq = dt; }
            di = dq >> 8; df = dq & 255u;
            a0 = dl[(dpos - di) & DMASK]; a1 = dl[(dpos - di - 1u) & DMASK];
            wl = a0 + (((a1 - a0) * (int32_t)df) >> 8);
            a0 = dr[(dpos - di) & DMASK]; a1 = dr[(dpos - di - 1u) & DMASK];
            wr = a0 + (((a1 - a0) * (int32_t)df) >> 8);
            dlpl += ((wl - dlpl) * 3) >> 2;                               /* a light low-pass in the loop: the echoes stay bright */
            dlpr += ((wr - dlpr) * 3) >> 2;
            vl = ((ya * (int32_t)P[6] + yb * (int32_t)P[18] + yd * (int32_t)P[26]) >> 7) + ((dlpr * (int32_t)dfb) >> 15);
            vr = (dlpl * (int32_t)dfb) >> 15;
            dl[dpos & DMASK] = (int16_t)soft(vl);                          /* soft: past 100% it rings on and on without exploding */
            dr[dpos & DMASK] = (int16_t)soft(vr);
            dpos++;
            if (sp_pop) { sp_env = sp_pop; sp_pop = 0; }
            if (sp_env || sp_hum) {                                        /* the sign: a 100 Hz buzz, and a crackle that dies away */
                int32_t nz, sg;
                sp_seed = sp_seed * 1664525u + 1013904223u;
                sp_ph += 9739000u;
                nz = (((int32_t)sp_seed >> 17) * (int32_t)(sp_env >> 4)) >> 12;
                sg = (((int32_t)sp_ph >> 18) * (int32_t)sp_hum) >> 8;
                if (sp_env) sp_env -= (sp_env >> 10) + 1u;
                dry += nz + sg;
            }
            ml = ((dry + wl + (wl >> 1)) * 19) >> 5;
            mr = ((dry + wr + (wr >> 1)) * 19) >> 5;
            if (ml > 24000) ml = 24000 + ((ml - 24000) >> 1); else if (ml < -24000) ml = -24000 - ((-24000 - ml) >> 1);   /* a soft knee above 24000 */
            if (mr > 24000) mr = 24000 + ((mr - 24000) >> 1); else if (mr < -24000) mr = -24000 - ((-24000 - mr) >> 1);
            if (ml > 32767) ml = 32767; else if (ml < -32767) ml = -32767;
            if (mr > 32767) mr = 32767; else if (mr < -32767) mr = -32767;
            out[2u * i] += ml;
            out[2u * i + 1u] += mr;
        }
    }
    t0 = fm1_ticks() - t0;
    if (t0 > load_peak) load_peak = t0;
}

/* ---- the screen: night blue, soft pink / violet / blue type, nothing else */
#define C_BG RGB(9, 11, 32)
#define C_LEAD RGB(255, 125, 205)
#define C_BASS RGB(112, 170, 255)
#define C_LAB RGB(176, 152, 255)
#define C_TRK RGB(30, 29, 68)
#define C_CPU RGB(96, 90, 160)
static const char *const KNOB_NAME[3][4] = {{"CUTOFF", "RESO", "ENV MOD", "DECAY"}, {"GLIDE", "DRIVE", "DELAY", "WARP"}, {"FM", "RING", "VOWEL", "CRUSH"}};
static const char *const MODE_NAME[12] = {"UP-DN", "DN-UP", "", "", "", "", "", "", "FLOOR", "CHASE", "PULSE", ""};   /* the arps, then the drum patterns */
static const char *const DKNOB_NAME[2][4] = {{"KICK", "SNARE", "DELAY", "CRUSH"}, {"", "", "", ""}};

static void box(uint32_t y, const felucca_font_t *f, const char *s, uint16_t c)    /* centred text on the night blue */
{
    cv_begin(240, f->h, C_BG);
    cv_text(((int32_t)240 - text_w(f, s)) / 2, 0, f, s, c);
    cv_blit(0, y);
    lcd_sync();
}

/* an image on the night blue, k = 0..256 how much of it shows (the logo fades in) */
static void draw_img(const img_t *m, uint32_t x, uint32_t y, uint32_t k)
{
    uint32_t n = (uint32_t)m->w * m->h, i, bg = C_BG, br = bg >> 11, bgn = (bg >> 5) & 63u, bb = bg & 31u;
    cv_begin(m->w, m->h, C_BG);
    if (n > cv_w * cv_h) n = cv_w * cv_h;
    for (i = 0; i < n; i++) {
        uint32_t c = m->px[i];
        if (k < 256u) {
            uint32_t r = c >> 11, g = (c >> 5) & 63u, b = c & 31u;
            r = (uint32_t)((int32_t)br + (((int32_t)r - (int32_t)br) * (int32_t)k >> 8));
            g = (uint32_t)((int32_t)bgn + (((int32_t)g - (int32_t)bgn) * (int32_t)k >> 8));
            b = (uint32_t)((int32_t)bb + (((int32_t)b - (int32_t)bb) * (int32_t)k >> 8));
            c = r << 11 | g << 5 | b;
        }
        cv_px[i] = swap16(c);
    }
    cv_blit(x, y);
    lcd_sync();
}

/* the sign lights up like old neon: it stutters, goes out, catches, then holds. {time ms, brightness 0..256, crackle} */
static const uint16_t NEON[][3] = {
    {400, 70, 7000}, {450, 0, 0}, {700, 130, 9000}, {790, 12, 0}, {1000, 175, 9000}, {1070, 30, 0},
    {1230, 215, 11000}, {1280, 45, 0}, {1330, 190, 8000}, {1400, 60, 0}, {1560, 256, 20000}, {2150, 228, 6000}, {2200, 256, 0}};
static void splash(void)                        /* ARP WAV 09: it flickers on, hums, and stays lit a moment */
{
    uint32_t t0 = fm1_ms, t, k = 0, kd = ~0u, i = 0;
    lcd_fill(0, 0, 240, 240, C_BG);
    do {
        t = fm1_ms - t0;
        platform_poll();
        while (i < sizeof NEON / sizeof NEON[0] && t >= NEON[i][0]) {
            k = NEON[i][1];
            if (NEON[i][2]) sp_pop = NEON[i][2];
            i++;
        }
        sp_hum = i == 0u ? 0u : t < 3000u ? 22u : 0u;                      /* the hum comes with the first spark, and stops before the end */
        if (k != kd) { kd = k; draw_img(&IMG_LOGO, (240u - IMG_LOGO.w) / 2u, (240u - IMG_LOGO.h) / 2u, k); }
    } while (t < 3400u);
    sp_hum = 0;
    lcd_fill(0, 0, 240, 240, C_BG);
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

#define BAT_LO 512u                             /* ADC reading of an empty / a full battery: a first guess (see the BAT number on screen) */
#define BAT_HI 651u
static void draw_battery(uint32_t raw)          /* top right: a battery with four bars */
{
    uint32_t lv = raw <= BAT_LO ? 0u : raw >= BAT_HI ? 4u : (raw - BAT_LO) * 4u / (BAT_HI - BAT_LO), i;
    uint16_t c = lv == 0u ? RGB(255, 80, 90) : C_LAB;
    cv_begin(40, 14, C_BG);
    cv_rect(0, 2, 30, 10, c);
    cv_rect(1, 3, 28, 8, C_BG);
    cv_rect(30, 5, 3, 4, c);
    for (i = 0; i < lv; i++) cv_rect((int32_t)(3u + 6u * i), 5, 5, 4, c);
    cv_blit(200, 0);
    lcd_sync();
}

static void draw_status(uint32_t bpm, uint32_t mode, int32_t oct, uint32_t run, uint32_t cpu, uint32_t bat)
{
    char s[44], *p = s;
    p = put_num(p, bpm); p = put(p, "  "); p = put(p, MODE_NAME[mode]);
    p = put(p, run ? "  " : "  STOP  "); *p++ = oct < 0 ? '-' : '+'; *p++ = (char)('0' + (oct < 0 ? -oct : oct));
    *p = 0;
    box(206, &FONT_S, s, C_LAB);
    p = put(s, "CPU "); p = put_num(p, cpu); p = put(p, "%   BAT "); p = put_num(p, bat); *p = 0;      /* the audio load: the worst call of the last second (a test readout) */
    box(222, &FONT_S, s, C_CPU);
}

/* ---- the main loop: no settings are kept; every start is the same raw saw */
static struct { uint8_t v[36], vol[3], bpm, dbpm, mode, tgt, page; int8_t oct; } set;
static const uint8_t DEF[36] = {30, 25, 50, 45, 0, 0, 0, 0, 0, 0, 0, 0, 70, 30, 70, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 0, 0, 0, 0, 0, 0, 0};

static void set_defaults(void)
{
    uint32_t i;
    for (i = 0; i < 36u; i++) set.v[i] = DEF[i];
    set.vol[0] = set.vol[1] = set.vol[2] = 127;
    set.bpm = 112; set.dbpm = 112; set.mode = 0; set.tgt = 0; set.page = 0; set.oct = 0;
}

static void apply_tempo(uint32_t bpm)
{
    uint32_t len = 661500u / bpm;               /* 44100 * 60 / (4 * bpm) */
    gate_len = len * 3u / 5u;
    step_len = len;
}

static void apply_dtempo(uint32_t bpm) { d_len = 661500u / bpm; }

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
    uint32_t kt[6] = {0, 0, 0, 0, 0, 0}, k, b, rel, b_key = 99, cpu = 0, cpu_t = 0, bat = 0, bat_t = 0, bat_drawn = ~0u, bat_shown = ~0u;
    int32_t e;

    set_defaults();
    for (k = 0; k < 36u; k++) P[k] = set.v[k];
    VOL[0] = set.vol[0]; VOL[1] = set.vol[1]; VOL[2] = set.vol[2];
    a_mode = set.mode; oct_sh = set.oct;
    apply_tempo(set.bpm); apply_dtempo(set.dbpm);

    splash();

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
                if (!a_mask) a_restart = 1u;        /* a chord from nothing: the bar (and the drums) start together */
                a_mask = m | down;
            } else if (set.tgt == 2u) {         /* the drums: the keys do nothing, you mix the kit with the knobs */
                ;
            } else {                            /* the bass is a drone: it rings until another key (the lowest of the new ones) */
                lowest(down, &k);
                b_key = k;
                b_inc = note_inc((uint32_t)((int32_t)KEY0 + (int32_t)k - 24 + 12 * set.oct));
                b_trig++;
                b_gate = 32767;
            }
        }

        b = fm1_input_edges(&rel);
        if (b & (1u << BTN_PLAY)) { a_run ^= 1u; d_off = 0; }
        if (b & (1u << BTN_HOME)) { a_mask = 0; b_gate = 0; d_off = 1; }
        if (b & (1u << BTN_FX)) set.page = (uint8_t)((set.page + 1u) % (set.tgt == 2u ? 1u : 3u));
        if (b & (1u << BTN_ARP)) {
            if (set.tgt == 2u) { d_pat = (d_pat + 1u) % 3u; d_off = 0; }
            else { set.mode = (uint8_t)((set.mode + 1u) & 1u); a_mode = set.mode; }
        }
        if (b & (1u << BTN_OCT_DOWN) && set.oct > -1) { set.oct--; oct_sh = set.oct; }
        if (b & (1u << BTN_OCT_UP) && set.oct < 1) { set.oct++; oct_sh = set.oct; }

        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) {   /* left: the lead, middle: the bass, right: the drums */
            int32_t t = (int32_t)set.tgt + (e > 0 ? 1 : -1);
            if ((uint8_t)(t < 0 ? 0 : t > 2 ? 2 : t) != set.tgt) set.page = 0;      /* every voice starts on page A */
            set.tgt = (uint8_t)(t < 0 ? 0 : t > 2 ? 2 : t);
        }
        for (k = 0; k < 4u; k++) {              /* the four knobs act on the page of the lead or the bass */
            uint8_t *v = &set.v[set.tgt * 12u + set.page * 4u + k];
            if ((e = fm1_enc_take(ENC_KNOB1 + k)) != 0) {
                if (set.tgt == 2u) d_off = 0;
                int32_t x = (int32_t)*v + turn(&kt[k], e);
                *v = (uint8_t)(x < 0 ? 0 : x > 127 ? 127 : x);
            }
        }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {   /* the volume of the lead or the bass */
            int32_t x = (int32_t)set.vol[set.tgt] + turn(&kt[4], e);
            set.vol[set.tgt] = (uint8_t)(x < 0 ? 0 : x > 127 ? 127 : x);
        }
        for (k = 0; k < 36u; k++) P[k] = set.v[k];
        VOL[0] = set.vol[0]; VOL[1] = set.vol[1]; VOL[2] = set.vol[2];
        if ((e = fm1_enc_take(ENC_SELECT)) != 0) {
            uint8_t *bp = set.tgt == 2u ? &set.dbpm : &set.bpm;   /* the lead and the bass share a tempo; the drums have their own */
            int32_t x = (int32_t)*bp + ((fm1_ms - kt[5] < 90u || e > 1 || e < -1) ? 5 : 1) * e;   /* 1 BPM a click, 5 when you turn fast */
            kt[5] = fm1_ms;
            *bp = (uint8_t)(x < 20 ? 20 : x > 200 ? 200 : x);
            if (set.tgt == 2u) apply_dtempo(*bp); else apply_tempo(*bp);
        }

        for (k = 0; k < 27u; k++) {             /* the held keys, the latched chord, the step that plays, the bass key glow */
            uint32_t on = (keys >> k & 1u) || (b_gate && k == b_key) || (a_mask >> k & 1u);
            fm1_led_key(KEY_LED(k), on);
        }
        fm1_led_key(BTN_PLAY, a_run);
        fm1_led_key(BTN_FX, set.page != 0u);

        if ((st = set.tgt == 2u ? 6u + set.page : set.tgt * 3u + set.page) != view_drawn) {
            view_drawn = st;
            draw_img(IMG_TITLE[st], (240u - IMG_TITLE[st]->w) / 2u, 8u, 256u);
            for (k = 0; k < 5u; k++) drawn[k] = ~0u;
        }
        for (k = 0; k < 5u; k++) {
            uint32_t v = k < 4u ? set.v[set.tgt * 12u + set.page * 4u + k] : set.vol[set.tgt];
            if (v != drawn[k]) { drawn[k] = v; draw_knob(k, v, set.tgt == 2u ? C_LAB : set.tgt ? C_BASS : C_LEAD, k < 4u ? (set.tgt == 2u ? DKNOB_NAME[set.page][k] : KNOB_NAME[set.page][k]) : "VOLUME"); }
        }
        if (fm1_ms - cpu_t > 1000u) { cpu_t = fm1_ms; cpu = load_peak / 1393u; if (cpu > 99u) cpu = 99u; load_peak = 0; }   /* budget 5805 us = 139320 ticks */
        if (fm1_ms - bat_t > 2000u || !bat) {            /* the battery, every 2 s, smoothed */
            int32_t r = fm1_adc_read(FM1_ADC_BATT);
            bat_t = fm1_ms;
            if (r >= 0) bat = bat ? (bat * 3u + (uint32_t)r + 2u) / 4u : (uint32_t)r + 1u;
        }
        if (bat != bat_drawn) { bat_drawn = bat; draw_battery(bat); }
        if ((st = (set.tgt == 2u ? set.dbpm : set.bpm) | (uint32_t)(set.tgt == 2u ? 8u + d_pat : set.mode) << 8 | (uint32_t)(set.oct + 1) << 12 | a_run << 16 | cpu << 20) != st_drawn || bat != bat_shown) {
            bat_shown = bat;
            st_drawn = st;
            draw_status(set.tgt == 2u ? set.dbpm : set.bpm, set.tgt == 2u ? 8u + d_pat : set.mode, set.oct, a_run, cpu, bat);
        }
    }
}
