/* SPDX-License-Identifier: GPL-3.0-only */
/* ARP WAV 09: a minimal synth for the FM-1, built on Dinghy. No presets, no saves: switch on, turn the knobs.
 *
 *   PRESETS             six voices, left to right: ARP, WAV, SEQ, KIT, SFX, FM1 (keys, knobs and volume act on that voice)
 *   KEYS (all 27)       ARP: hold a chord and let go, it keeps arpeggiating (latch). WAV: a drone, until the next key.
 *                       SEQ: stopped = EDIT, a key is heard, REC writes it in the next of 16 boxes.
 *   KNOB 1..4           ARP  A cutoff, env mod, decay, range | B delay, stretch, fm, crush
 *                       WAV  A cutoff, drive, delay, move   | B detune, warp, fm, crush
 *                       SEQ  A cutoff, env mod, decay, delay | B stretch, warp, fm, crush
 *                       KIT  kick, snare, hat, crush        SFX ghost, alien, delay, crush        FM1 tune, fine, ghost, crush
 *   FX                  page A / B               ALGORITHM  volume of the voice     SELECT  tempo     MASTER  the original master volume
 *   PLAY                ARP / WAV / KIT start-stop;  SEQ: play the loop / back to edit
 *   SEL                 KIT: the kit gets a tempo of its own / follows again;  SEQ: delete the last step    REC  SEQ: write the note
 *   HOME                stop everything (and clear the SEQ)        OCT- OCT+  octave of the voice
 *
 * Integers only (no FPU, no libgcc). The tables are in acid_tables.h. */
#define APP_NAME "ARP WAV 09"
static void app_idle(void) {}
#define LCD_IDLE app_idle
#include "platform.c"
#include "acid_tables.h"
#include "neon_img.h"                           /* the neon logo and the nine titles, drawn at build time */
#define KEY0 53u                                /* key 0 = F3 */
#define NP 64u                                  /* parameters: ARP 0..11, WAV 12..23, KIT 24..35, VOX 36..39, FM1 40..43, SEQ 48..59 */

/* ---- shared with the audio ISR: one writer each (docs/RULES.md) */
static volatile uint32_t a_mask;                /* latched ARP keys: bit k = key k */
static volatile uint32_t seq_cur = 255u;        /* the key the arpeggiator plays now (for the LEDs) */
static volatile uint32_t a_run = 1, a_mode;
static volatile uint32_t step_len = 6014u, gate_len = 3608u;   /* frames per 16th, frames the ARP gate stays open */
static volatile int32_t octv[7];                /* octave of each voice, -1..1 */
static volatile uint32_t P[NP] = {50, 0, 30, 45, 0, 0, 0, 0, 0, 0, 0, 0, 80, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 50, 0, 30, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static volatile uint32_t VOL[7] = {127, 127, 127, 127, 127, 127, 127};   /* ARP, WAV, KIT, VOX, FM1, SEQ, SFX */
static volatile uint32_t d_pat, d_off, a_restart;   /* the drum pattern, drums stopped by HOME, a chord began from nothing: restart the bar */
static volatile uint32_t b_inc, b_trig;
static volatile int32_t b_gate;
static volatile uint32_t d_len = 6014u;         /* frames per drum 16th */
static volatile uint32_t f_off = 1u;            /* SFX: silent until its knobs are touched */
static volatile uint32_t s_off = 1u;            /* FM1: silent until its knobs are touched */
/* SEQ: the notes are written by the main loop, played by the ISR */
static volatile uint8_t sq[16], sqk[16];        /* note (with octave), key (for the LED) */
static volatile uint32_t sqn, sqc, sqrun;       /* steps written, the box under the cursor, playing */
static volatile uint32_t q_evt, q_inc, q_hold, q_restart;   /* a key was heard in EDIT, its pitch, still held, PLAY pressed */
static volatile uint32_t q_cur = 255u, q_idx = 255u;   /* (and q_pend: the key heard last, main loop only) */        /* the key / the box playing now */
static volatile uint32_t q_gate = 3608u;        /* frames the SEQ gate stays open (STRETCH) */

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
static int32_t vlim(int32_t x)                  /* clean up to 12000; only the rare peaks above are rounded off (a tanh, no distortion below) */
{
    int32_t a = x < 0 ? -x : x, uq, u2, th;
    if (a < 12000) return x;
    uq = (a - 12000) * 4096 / 14000; if (uq > 12288) uq = 12288;
    u2 = (uq * uq) >> 12;
    th = uq * ((27 * 4096 + u2) >> 3) / ((27 * 4096 + 9 * u2) >> 3);
    if (th > 4096) th = 4096;
    a = 12000 + ((14000 * th) >> 12);
    return x < 0 ? -a : a;
}
static int32_t ez(uint32_t v) { return EZ[v > 127u ? 127u : v]; }   /* every effect knob starts gently and takes over little by little */

#define DMASK 32767u                            /* a ping-pong delay: a left and a right line of 0.74 s, shared by every voice (the pool) */
static int16_t dl[32768] __attribute__((section(".pool")));
static int16_t dr[32768] __attribute__((section(".pool")));
static uint32_t dpos, dlen = 18042u, dq = 18042u << 8;   /* the wanted delay time; the time now, in 1/256 sample (it glides, like a tape) */
static int32_t dlpl, dlpr, wph;

/* ---- effects per voice, integers only. CRUSH: from zero up, fewer bits and a lower sample rate together. */
typedef struct { int32_t hold; uint32_t cnt; } fx_t;
static int32_t fx_run(fx_t *f, int32_t x, int32_t crush)
{
    crush = ez((uint32_t)crush);
    if (crush) {
        uint32_t sh = 6u + (((uint32_t)crush * 6u) >> 7), n = 1u + (((uint32_t)crush * (uint32_t)crush) >> 8);
        x = (x >> sh) * (int32_t)(1u << sh);    /* fewer bits (8 bits at once, 3 at the end) */
        if (++f->cnt >= n) { f->cnt = 0; f->hold = x; }          /* and a lower sample rate: hold every n-th sample */
        x = f->hold;
    }
    return x;
}

static volatile uint32_t load_peak;             /* the longest app_audio call, in timer ticks */
/* the start-up sound: a radio hunting for a signal (set by the splash, played by the ISR): hiss, a band-passed sweep, whistles, bleeps */
static volatile uint32_t sp_hs, sp_na, sp_cf, sp_wi, sp_wa, sp_da, sp_bi, sp_ba;
static uint32_t sp_seed = 1u, sp_wp, sp_bp, sp_d1, sp_d2, sp_d3;
static svf_t SPF;

/* ---- the lead voice, shared by ARP and SEQ: o = where its knobs live in P (0 for ARP, 48 for SEQ) */
typedef struct {
    uint32_t ph, mp, inc, incb, inc_t, tick, pos;
    int32_t gate, amp, tgt, env_f, env_a, f, q;
    svf_t sv, sv2;
    fx_t fx;
} lead_t;
static lead_t LA, LQ;

static void lead_block(lead_t *l, uint32_t o)   /* every 32 frames: envelopes and filter */
{
    int32_t dec = (int32_t)P[o + 3] + (int32_t)(P[o + 1] >> 1), k, idx;
    if (dec > 127) dec = 127;
    l->env_f = (int32_t)(((int64_t)l->env_f * DEC_K[dec]) >> 15);
    k = dec + 25; if (k > 127) k = 127;
    l->env_a = (int32_t)(((int64_t)l->env_a * DEC_K[k]) >> 15);
    if (!l->gate) l->env_a = (l->env_a * 24000) >> 15;                /* the gate closed: quick release */
    idx = (int32_t)P[o] + ((env2(l->env_f) * (int32_t)P[o + 2]) >> 7);
    if (idx > 127) idx = 127;
    l->f = CUT_F[idx];
    l->q = resq(25u + ((uint32_t)ez(P[o + 4]) * 70u >> 7));           /* RESO: gentle at first, then it squeals */
    l->tgt = l->env_a;
}

static int32_t lead_sample(lead_t *l, uint32_t o, uint32_t vol)
{
    uint32_t c;
    int32_t la, ya;
    if (l->amp < 16 && l->tgt < 16) return 0;                       /* silent: nothing to compute */
    l->ph += l->incb; c = l->ph;
    if (P[o + 8]) {                                                  /* FM: an inharmonic triangle (x1.41) bends the phase */
        int32_t x;
        l->mp += l->incb + (uint32_t)(((uint64_t)l->incb * 106u) >> 8);
        x = (int32_t)l->mp >> 16; x = (x < 0 ? -x : x) - 16384;
        c += (uint32_t)(x * ez(P[o + 8])) << 11;
    }
    la = svf(&l->sv2, svf(&l->sv, (int32_t)c >> 16, l->f, 36000), l->f, l->q);   /* two stages: 24 dB/oct, the resonance on the second */
    if (la > 32767) la = 32767; else if (la < -32767) la = -32767;
    l->amp += (l->tgt - l->amp) >> 4;
    ya = soft((int32_t)(((int64_t)la * l->amp) >> 15));
    ya = fx_run(&l->fx, ya, (int32_t)P[o + 11]);
    return (ya * (int32_t)(vol + 1u) * (o ? 13 : 8)) >> 10;               /* the SEQ is 1.6x louder: it has to stand against the bass */
}

static void arp_fire(uint32_t key, uint32_t up)
{
    LA.inc_t = note_inc((uint32_t)((int32_t)(KEY0 + key) - 12 + 12 * octv[0]) + up);
    LA.inc = LA.inc_t; LA.incb = LA.inc_t;
    LA.gate = 1;
    LA.env_f = 32767;
    LA.env_a = 32767;
}

static void arp_step(void)                      /* the next step of the latched chord: up through it and back down; RANGE sets how many octaves */
{
    uint32_t m = a_mask, n = 0, t = m, up, j, k, s, R, M, per, q, i;
    if (!m) return;
    while (t) { n += t & 1u; t >>= 1; }
    s = LA.pos++;
    R = 2u + ((P[5] * 3u) >> 7); M = R * n; per = 2u * M - 2u; q = s % per; i = q < M ? q : per - q;
    if (a_mode) i = M - 1u - i;
    j = i % n; up = 12u * (i / n);
    for (k = 0; k < 27u; k++)                   /* the j-th key of the chord, from the lowest */
        if (m >> k & 1u) { if (j == 0) break; j--; }
    seq_cur = k;
    arp_fire(k, up);
}

static void seq_step(void)                      /* the next box of the loop */
{
    uint32_t i = LQ.pos % sqn;
    LQ.pos++; q_cur = sqk[i]; q_idx = i;
    LQ.inc_t = note_inc((uint32_t)((int32_t)(KEY0 + (uint32_t)sq[i]) - 12 + 12 * octv[5])); LQ.inc = LQ.inc_t; LQ.incb = LQ.inc_t;
    LQ.gate = 1; LQ.env_f = 32767; LQ.env_a = 32767;
}

/* ---- the kit: KICK and SNARE set the level AND move along a row of different drums, so each position is another sound */
static const uint16_t DK[4] = {0x1111, 0x1151, 0x0911, 0x1111};      /* kick, per pattern */
static const uint16_t DS[4] = {0x1010, 0x1010, 0x1010, 0x1010};      /* snare */
static struct {
    int32_t kl, sl, cm, kg, kgain, bg, ng, ha;
    uint32_t fe, fs, kr, ar, bf, bs, br, nr;
} K;
static struct { uint32_t tick, step, kph, sph, seed, cl; int32_t kenv, kamp, senv, stn, hlp, nlp; fx_t fx; } D;
static int32_t d_duck;                          /* the kit presses the others down on every hit (a sidechain) */

static uint32_t rate24(int32_t sh_q8) { uint32_t s16 = (uint32_t)sh_q8 >> 4; return FR24[s16 & 15u] >> (s16 >> 4); }   /* 2^-shift in Q24 */
static void kmorph(const int32_t (*tab)[9], int32_t x, int32_t *o)    /* each position is another kit; they blend only in a short zone between two */
{
    int32_t p = (x > 4 ? x - 4 : 0) * 4 * 256 / 123, i = p >> 8, fr, u, j;
    if (i > 3) i = 3;
    fr = p - (i << 8);
    u = (fr - 96) * 4; u = u < 0 ? 0 : u > 256 ? 256 : u;
    fr = (u * u * (768 - 2 * u)) >> 16;                                   /* smoothstep */
    for (j = 0; j < 9; j++) o[j] = tab[i][j] + (((tab[i + 1][j] - tab[i][j]) * fr) >> 8);
}
/* a small echo on the kit: two sixteenths long (at most 0.37 s), it feeds back a little */
static int16_t kdl[16384] __attribute__((section(".pool")));
static uint32_t kdp;
static void kit_block(void)
{
    int32_t x = (int32_t)P[24], a[9];
    K.kl = x >= 4 ? 115 : 115 * x / 4;
    if (x) { kmorph(KT, x, a); K.fe = (uint32_t)a[0]; K.fs = (uint32_t)a[1]; K.kr = rate24(a[2]); K.ar = rate24(a[3]); K.cm = a[4]; K.kg = a[5]; K.kgain = a[6]; }
    x = (int32_t)P[25]; K.sl = x >= 4 ? 115 : 115 * x / 4;
    if (x) { kmorph(ST, x, a); K.bf = (uint32_t)a[0]; K.bs = (uint32_t)a[1]; K.br = rate24(a[2]); K.nr = rate24(a[3]); K.bg = a[4]; K.ng = a[5]; K.ha = a[6]; }
}

static int32_t drums(void)
{
    int32_t y = 0, nz, hp = 0, yk = 0, ysn = 0;
    uint32_t st;
    if (a_restart) { a_restart = 0; D.tick = 0; D.step = 0; }
    if (D.tick == 0 && a_run && !d_off) {                              /* a 16th: which drums hit */
        st = D.step; D.step = (D.step + 1u) & 15u;
        if (DK[d_pat & 3u] >> st & 1u) { D.kenv = 32767; D.kamp = 32767; D.kph = 0; D.cl = 64u; d_duck = K.kl * 190; }
        if (DS[d_pat & 3u] >> st & 1u) { D.senv = 32767; D.stn = 32767; D.sph = 0; if (K.sl * 130 > d_duck) d_duck = K.sl * 130; }
    }
    if (++D.tick >= d_len) D.tick = 0;
    D.seed = D.seed * 1664525u + 1013904223u;
    nz = (int32_t)D.seed >> 17;                                        /* white noise +-16384 */
    if (D.senv) { D.hlp += ((nz - D.hlp) * K.ha) >> 8; hp = nz - D.hlp; D.nlp += (hp - D.nlp) >> 1; hp = D.nlp; }   /* high-passed noise */
    if (D.kamp) {
        uint32_t f = (uint32_t)((97392ull * K.fe) >> 8) + (uint32_t)((97392ull * K.fs * (uint32_t)D.kenv) >> 23);   /* a fast fall: a boom with a punch */
        int32_t t, k;
        D.kph += f;
        t = (int32_t)D.kph >> 16; t = (t < 0 ? -t : t) * 2 - 32768;   /* a triangle, rounded by the soft clip into a sine */
        k = soft((int32_t)(((int64_t)soft(t) * K.kg) >> 8));
        k = (int32_t)(((int64_t)k * D.kamp) >> 15);
        yk += (int32_t)(((int64_t)k * K.kl * 17 * K.kgain) >> 17);
        if (D.cl) { yk += (int32_t)(((int64_t)(((int64_t)nz * (int32_t)D.cl) >> 6) * K.kl * 3 * K.cm) >> 19); D.cl--; }
        D.kenv -= (int32_t)(((int64_t)D.kenv * K.kr) >> 24) + (D.kenv ? 1 : 0);
        D.kamp -= (int32_t)(((int64_t)D.kamp * K.ar) >> 24) + 1;
        if (D.kamp < 0) D.kamp = 0;
    }
    if (D.senv || D.stn) {
        int32_t t, sn, na;
        D.sph += (uint32_t)((97392ull * K.bf) >> 8) + (uint32_t)((97392ull * K.bs * (uint32_t)D.stn) >> 23);
        t = (int32_t)D.sph >> 16; t = (t < 0 ? -t : t) * 2 - 32768;
        na = (D.senv >> 1) + (int32_t)(((int64_t)D.senv * D.senv) >> 16);    /* the noise starts hard and falls fast: a crack, then a tail */
        sn = (int32_t)(((int64_t)((soft(t) * D.stn) / 98304) * K.bg) >> 8) + (int32_t)(((((int64_t)hp * na) >> 15) * K.ng) >> 8);
        ysn += (int32_t)(((int64_t)sn * K.sl * 22) >> 8);
        D.stn -= (int32_t)(((int64_t)D.stn * K.br) >> 24) + (D.stn ? 1 : 0); if (D.stn < 0) D.stn = 0;
        D.senv -= (int32_t)(((int64_t)D.senv * K.nr) >> 24) + (D.senv ? 1 : 0); if (D.senv < 0) D.senv = 0;
    }
    y = yk + ysn;
    {                                                                  /* DELAY: a small echo of the kit */
        int32_t dx = (int32_t)ez(P[26]), kt = (int32_t)(2u * d_len), ke, w;
        kt = kt > 16383 ? 16383 : kt;
        ke = kdl[(kdp - (uint32_t)kt) & 16383u];
        w = y + ((ke * (32 + 26 * dx / 127)) >> 7);
        kdl[kdp & 16383u] = (int16_t)(w > 30000 ? 30000 : w < -30000 ? -30000 : w); kdp++;
        y += (ke * 64 * dx / 127) >> 7;
    }
    y = fx_run(&D.fx, y, (int32_t)P[27]);                                /* CRUSH */
    y = soft(y);                                                       /* the kit is glued by a soft clip */
    if (d_duck > 0) d_duck -= (d_duck >> 11) + 1;
    return (y * (int32_t)(VOL[2] + 1u) * 85) >> 12;
}

/* ---- WAV: saw + detuned saw + sub square, low-pass, then saturation */
static struct {
    uint32_t ph1, ph2, ph3, mp, seen, incb, inc2, lfo;
    int32_t amp, f, q;
    svf_t sv, sv2;
    fx_t fx;
} B;

static int32_t fcoef(uint32_t hz)               /* 2 sin(pi f / 44100) in Q15 (a short series: the formants are under 2.4 kHz) */
{
    int64_t x = (int64_t)hz * 1195, x3 = (((x * x) >> 24) * x) >> 24;
    int32_t c = (int32_t)((65536 * (x - ((x3 * 43691) >> 18))) >> 24);
    return c > 30000 ? 30000 : c;
}

/* ---- FM1: a radio. 20 stations hide on a long dial; at every power-on four are drawn, put in new places, and each one sounds a little different.
 * Everything is integers: phases are 32-bit turns, levels Q15, decays are "e -= e * rate >> 24" (RT(ms) gives the rate for a time constant). */
#define NST 24u
#define RT(ms) (380449u / (ms))                 /* 2^24 / (44.1 * ms): the per-sample decay rate for a time constant in ms */
#define MS(ms) ((uint32_t)(ms) * 441u / 10u)    /* ms -> samples */
#define HZP(h) ((uint32_t)(h) * 97392u)         /* an integer Hz -> a 32-bit phase step */
static uint32_t hz8p(uint32_t q8) { return (uint32_t)(((uint64_t)q8 * 97392u) >> 8); }   /* Q8 Hz -> phase step */
static const uint16_t SGQ[NST] = {192, 256, 410, 256, 256, 256, 230, 179, 256, 108, 243, 120, 538, 512, 384, 256, 248, 187, 947, 282, 410, 1408, 61, 282};   /* so that every station is as loud as the others (x256) */

static uint32_t r_seed = 1u;
static uint32_t rnd32(void) { r_seed = r_seed * 1664525u + 1013904223u; return r_seed; }
static uint32_t rnd16(void) { return rnd32() >> 16; }
static uint32_t rr(uint32_t a, uint32_t b) { return a + (uint32_t)(((uint64_t)rnd16() * (b - a)) >> 16); }   /* a number in [a, b) */
static int32_t sn(uint32_t ph)                  /* a sine of a 32-bit phase, Q15 */
{
    uint32_t i = ph >> 24, f = (ph >> 16) & 255u;
    int32_t a = SIN256[i], b = SIN256[(i + 1u) & 255u];
    return a + (((b - a) * (int32_t)f) >> 8);
}
static int32_t ex(uint32_t d, uint32_t tau)     /* exp(-d / tau) in Q15 for d and tau in samples (a table, linear) */
{
    uint32_t x = (d << 5) / tau;
    return x > 255u ? 0 : EXP256[x];
}
static uint32_t pw2(int32_t x100)               /* 2^(x/1200), Q16, x in cents of a semitone x100 = 100 per semitone */
{
    int32_t o = 0, s, f;
    while (x100 < 0) { x100 += 1200; o--; }
    while (x100 >= 1200) { x100 -= 1200; o++; }
    s = x100 / 100; f = x100 % 100;
    {
        uint32_t v = PW2[s] + (uint32_t)(((int32_t)(PW2[s + 1] - PW2[s]) * f) / 100);
        return o >= 0 ? v << o : v >> -o;
    }
}
static int32_t lp(int32_t *s, int32_t x, int32_t k16) { *s += (int32_t)(((int64_t)(x - *s) * k16) >> 16); return *s; }   /* a one-pole low-pass */
static void edec(int32_t *e, uint32_t rate) { *e -= (int32_t)(((int64_t)*e * rate) >> 24); }                             /* e *= exp(-1/tau), e in Q30 */
#define E30 1073741824

typedef struct { uint32_t f; uint32_t ph[4]; int32_t e[4]; int32_t att; uint32_t age; int32_t amp; } note_t;

static struct {
    uint32_t n, cont[4]; int32_t pos[4];
    /* the dial */
    int32_t pn, pw, bk, dist, near, aw, ad;
    uint32_t wp, wf;
    int32_t fcs;
    svf_t sv;
    /* ghost */
    int32_t gw, gi1, gi2, cn;
    uint32_t rp, cph, cf, cprog, cstep;
    int32_t cl, cs;
    /* 0 phone */
    uint8_t dig[8]; uint32_t dp1, dp2; int32_t dlp;
    /* 1 alien */
    uint32_t ant, ap, ap2, lp65, lp17, lp3; int32_t af, aft, alow;
    /* 2 storm */
    int32_t rl, t1, t2, te; uint32_t ln, tt, tper, p11;
    /* 3 sonar */
    uint32_t sper, sdel, spp[3], sf0; int32_t eb[3], ea[3], at[3], sdec;
    /* 4 numbers */
    uint8_t nd[5]; int32_t ncyc; uint32_t nf0; uint32_t nph; svf_t sa, sb, sc; int32_t sc1, sc2;
    /* 5 music box */
    uint32_t mnext, mroot, mtempo, mi, p07; uint8_t mel[16]; int8_t mdet[16];
    struct { uint32_t f, ph[3]; int32_t e[3]; uint32_t age; } mb[3];
    /* 6 jazz */
    uint32_t jbeat, jlast, jsw, jbf, jbp, jkey, jprog; int32_t jhit, jbl, jhp, jbe, jbd, jh1, jh2; svf_t jl, jh;
    note_t jp[12];
    /* 7 old man */
    uint32_t onext, ot0, odur, of0, oleft, ogap, opr, ovw, onas; uint32_t o23;
    /* 8 whale */
    uint32_t wnext, wt0, wdur, wacc, wstep, wfa, wfb, wph[3], w9;
    /* 9 bell */
    uint32_t bper, bph[7], bf; int32_t be[7], bat, bl;
    /* 10 siren */
    uint32_t stype, sph, s45, s32, s17, s14, s17f;
    /* 11 spaceship */
    uint32_t ssa, ssb, sbn, sbp, sbf, s07, s13, s97; int32_t sbe, sfa, sfb; svf_t ssva, ssvb;
    /* 12 car */
    uint32_t kf0, kacc, kph, kstep, k08; int32_t kl1, kf, kt1;
    /* 13 helicopter */
    uint32_t hrf, hbp, hacc, hstep, h38; int32_t hl1, hl2;
    /* 14 train */
    uint32_t trp, tacc, tstep, t80, h349, h440, h19, h19acc; int32_t cl1, cl2, tt1; int32_t tflag; svf_t tcl;
    uint32_t jbv[4], jsv[4];
    /* 18 truck horn */
    uint32_t thf, thk, thp[3]; int32_t the, thl;
    /* 20 typewriter, 21 clock, 22 wind, 23 neon, and the old speaker */
    uint32_t twn, twt, twl, twd, twr, twp, twdp, twm, wn1, wn2, wn3, nn1, nn2, nfn, nft; int32_t twe, twf, twg, xt1, wnfc, cke; svf_t wnd, vf, vg;
    /* 19 morse */
    uint32_t mof, mou, mot, moi, mon, moq_n; int32_t moe; uint32_t mop; uint16_t moq[200];
} R;
static int16_t gd[2048] __attribute__((section(".pool")));
static int16_t gg1[6615] __attribute__((section(".pool")));
static int16_t gg2[10143] __attribute__((section(".pool")));

static const uint16_t DTR[4] = {697, 770, 852, 941}, DTC[3] = {1209, 1336, 1477};
static const uint8_t DTK[10] = {0x31, 0x00, 0x01, 0x02, 0x10, 0x11, 0x12, 0x20, 0x21, 0x22};   /* digit -> row << 4 | column */
static const uint8_t MSC[10] = {0, 2, 3, 5, 7, 8, 10, 12, 14, 15};            /* a minor scale, for the music box */
static const int8_t WALK[6] = {-2, -1, -1, 1, 1, 2};
static const uint16_t OVW[6][2] = {{730, 1090}, {530, 1840}, {270, 2290}, {570, 840}, {300, 870}, {400, 1100}};
/* the digits, said by a small machine: kind (0 vowel, 1 hiss, 2 burst, 3 nasal, 5 soft hiss, 6 buzz), F1, F2, ms */
static const uint16_t DGS[][4] = {
    {6, 300, 4500, 70}, {0, 400, 1900, 70}, {3, 400, 1300, 60}, {0, 570, 840, 150}, {0, 300, 870, 60},        /* zero 0..4 */
    {0, 300, 700, 50}, {0, 640, 1190, 110}, {3, 250, 1500, 130},                                              /* one 5..7 */
    {2, 3500, 3500, 25}, {0, 300, 870, 240},                                                                    /* two 8..9 */
    {5, 2500, 6000, 80}, {3, 400, 1300, 50}, {0, 270, 2290, 240},                                              /* three 10..12 */
    {5, 1500, 6000, 70}, {0, 570, 840, 200}, {3, 400, 1300, 110},                                              /* four 13..15 */
    {5, 1500, 6000, 70}, {0, 730, 1090, 130}, {0, 400, 1900, 110}, {6, 250, 1200, 90},                          /* five 16..19 */
    {1, 5500, 7000, 110}, {0, 400, 1900, 110}, {2, 3000, 3000, 25}, {1, 5500, 7000, 120},                      /* six 20..23 */
    {1, 5500, 7000, 100}, {0, 530, 1840, 110}, {6, 250, 1200, 60}, {0, 500, 1500, 70}, {3, 250, 1500, 120},    /* seven 24..28 */
    {0, 530, 1840, 130}, {0, 400, 2000, 120}, {2, 3500, 3500, 30},                                            /* eight 29..31 */
    {3, 250, 1500, 80}, {0, 730, 1090, 120}, {0, 400, 1900, 110}, {3, 250, 1500, 110}};                        /* nine 32..35 */
static const uint8_t DGI[11] = {0, 5, 8, 10, 13, 16, 20, 24, 29, 32, 36};   /* where each digit starts in DGS */
static const uint16_t GKQ[7] = {563, 676, 394, 619, 507, 338, 563};        /* a gain per kind of sound: [1, 1.2, 0.7, 1.1, 0.9, 0.6, 1] x 2.2 x 256 */
static const uint16_t BELLR[7] = {128, 256, 305, 384, 512, 640, 768}, BELLA[7] = {16384, 32768, 22938, 13107, 18022, 8192, 6554}, BELLT[7] = {5500, 4200, 3000, 2600, 2000, 1300, 1000};
static const uint8_t MCODE[26] = {0x41,0x88,0x8a,0x64,0x20,0x82,0x66,0x80,0x40,0x87,0x65,0x84,0x43,0x42,0x67,0x86,0x8d,0x62,0x60,0x21,0x61,0x81,0x63,0x89,0x8b,0x8c};   /* morse: bit 7..5 = length, low bits = dots (0) and dashes (1), first element in the top bit */

/* a small voice: a buzz (or a hiss) through three resonances (kind: 0 vowel, 1 hiss, 2 burst, 3 nasal, 5 soft hiss, 6 buzz) */
static int32_t speak(uint32_t f0q8, uint32_t kind, uint32_t t1, uint32_t t2, int32_t ampq8, int32_t breath16)
{
    int32_t nz = (int32_t)rnd32() >> 17, exc = 0, o, c1, c2;
    R.nph += hz8p(f0q8);
    if (kind == 0u || kind == 3u || kind == 6u) exc = (((int32_t)R.nph >> 16) * (kind == 3u ? 700 : 1400)) >> 15;
    if (kind == 1u) exc += (nz * 31) >> 8; else if (kind == 5u) exc += (nz * 13) >> 8; else if (kind == 2u) exc += (nz * 51) >> 8; else if (kind == 6u) exc += (nz * 8) >> 8;
    exc += (nz * breath16) >> 16;
    R.sc1 += (((int32_t)(t1 << 8) - R.sc1) * 262) >> 16; R.sc2 += (((int32_t)(t2 << 8) - R.sc2) * 262) >> 16;
    c1 = fcoef((uint32_t)R.sc1 >> 8); c2 = fcoef((uint32_t)R.sc2 >> 8);
    svf(&R.sa, exc, c1, 2800); svf(&R.sb, exc, c2, 2800); svf(&R.sc, exc, 12500, 2800);
    o = R.sa.band + ((R.sb.band * 205) >> 8) + (R.sc.band >> 2);
    return (int32_t)(((int64_t)o * ampq8) >> 8);
}

static int32_t station(uint32_t ty)
{
    int32_t y = 0, nz = (int32_t)rnd32() >> 17;
    uint32_t n = R.n, d;
    switch (ty) {
    case 0: {                                   /* a telephone: someone is dialling, tone by tone */
        uint32_t m0 = n % 194040u, ix = m0 / 12348u;
        d = m0 - ix * 12348u;
        if (ix < 8u && d < 6615u) {
            int32_t e = 32767, a = (int32_t)d * 93, b = (int32_t)(6615u - d) * 93;     /* 8 ms in, 8 ms out */
            uint32_t kk = DTK[R.dig[ix]];
            if (a < e) e = a;
            if (b < e) e = b;
            R.dp1 += HZP(DTR[kk >> 4]); R.dp2 += HZP(DTC[kk & 15u]);
            y = ((((sn(R.dp1) + sn(R.dp2)) * 8400) >> 15) * e) >> 15;
        }
        y = lp(&R.dlp, y, 19661);
        break; }
    case 1: {                                   /* an alien: a wet, warbling voice that slides around */
        int32_t amp, md, inc;
        if (n >= R.ant) { R.aft = (int32_t)((220u + (rnd16() * (680u * (uint32_t)R.alow >> 8) >> 16)) << 8); R.ant = n + MS(250) + (rnd16() * MS(600) >> 16); }
        R.af += (int32_t)(((int64_t)(R.aft - R.af) * 26) >> 16);
        R.lp65 += hz8p(6u * 256u + 128u); R.lp17 += HZP(17); R.lp3 += HZP(3);
        md = 65536 + ((3932 * sn(R.lp65)) >> 15);
        inc = (int32_t)(((int64_t)hz8p((uint32_t)R.af) * md) >> 16);
        R.ap += (uint32_t)inc; R.ap2 += (uint32_t)inc >> 1;
        amp = 18022 + ((14746 * sn(R.lp17)) >> 15);
        y = (sn(R.ap + (uint32_t)(sn(R.ap2 + R.lp3) * 50066)) * amp) >> 15;
        y = (y * 8200) >> 15;
        break; }
    case 2: {                                   /* a storm: rain behind, and thunder rolling every few seconds */
        int32_t a, att;
        lp(&R.rl, nz, 22938);
        R.p11 += hz8p(28u);                      /* 0.11 Hz */
        y = ((((nz - R.rl) * 14418) >> 16) * (24576 + ((8192 * sn(R.p11)) >> 15))) >> 15;
        if (n >= R.ln) { R.ln = n + (uint32_t)(((uint64_t)R.tper * (16384u + (rnd16() * 22938u >> 16))) >> 15); R.tt = n + MS(350) + (rnd16() * MS(900) >> 16); R.te = 0; }
        if (n == R.tt) R.te = E30;
        lp(&R.t1, nz, 786); lp(&R.t2, R.t1, 786);
        if (R.te) {
            att = (int32_t)((n - R.tt) * 3u); if (att > 32767) att = 32767;      /* 0.25 s attack */
            edec(&R.te, RT(1800));
            a = R.t2 * 22;
            y += (int32_t)(((int64_t)a * att >> 15) * (R.te >> 15) >> 15);
        }
        break; }
    case 3: {                                   /* a sonar: a round ping, and its echoes coming back from far away */
        uint32_t m = n % R.sper, q;
        static const uint16_t SGN[3] = {32768, 11141, 4588};
        if (m == 0u) { R.ea[1] = R.ea[2] = 0; }
        for (q = 0; q < 3u; q++) {
            if (m == q * R.sdel) { R.eb[q] = R.ea[q] = R.at[q] = E30; }
            if (m >= q * R.sdel && R.ea[q]) {
                uint32_t fq = R.sf0 * (1000u - 25u * q) / 1000u;         /* Q8 Hz */
                int32_t bend = 65536 + (int32_t)(((int64_t)2294 * (R.eb[q] >> 15)) >> 15);
                R.spp[q] += (uint32_t)(((int64_t)hz8p(fq) * bend) >> 16);
                y += (int32_t)((((int64_t)sn(R.spp[q]) * SGN[q] >> 15) * (R.ea[q] >> 15) >> 15) * ((E30 - R.at[q]) >> 15) >> 15);
                edec(&R.eb[q], RT(60)); edec(&R.ea[q], 380449u / ((uint32_t)R.sdec * (10u + 7u * q) / 10u)); edec(&R.at[q], RT(4));
            }
        }
        lp(&R.t1, nz, 1311);
        y = (y * 20000 >> 15) + ((R.t1 * 205) >> 8);
        break; }
    case 4: {                                   /* a numbers station: a flat little voice reads out groups of five digits, in English */
        uint32_t cy = 229320u, cyc = n / cy, mc = n - cyc * cy, di = mc / 29106u, kd = 4u, f1 = 500u, f2 = 1000u, md = 0u;
        int32_t amp = 0;
        if ((int32_t)cyc != R.ncyc) { R.ncyc = (int32_t)cyc; for (d = 0; d < 5u; d++) R.nd[d] = (uint8_t)(rnd16() * 10u >> 16); }
        if (di < 5u) {
            uint32_t acc = 0, q, g0 = DGI[R.nd[di]], g1 = DGI[R.nd[di] + 1u];
            md = (mc - di * 29106u) * 10u / 441u;                       /* ms into the digit */
            for (q = g0; q < g1; q++) {
                if (md < acc + DGS[q][3]) {
                    uint32_t m = md - acc;
                    int32_t a = (int32_t)m * 5461, b = (int32_t)(DGS[q][3] - m) * 4096;
                    amp = a < b ? a : b; if (amp > 32767) amp = 32767;
                    kd = DGS[q][0]; f1 = DGS[q][1]; f2 = DGS[q][2];
                    break;
                }
                acc += DGS[q][3];
            }
        }
        y = speak(R.nf0 - (uint32_t)(((uint64_t)R.nf0 * md * 26u) >> 16), kd, f1, f2, (amp * GKQ[kd]) >> 15, 315);
        break; }
    case 5: {                                   /* a music box, a little out of tune, a lullaby whose last note is wrong */
        uint32_t q, h;
        static const uint16_t MR[3] = {256, 706, 1382}, MA[3] = {32767, 11469, 3932};
        R.p07 += hz8p(18u);
        if (n >= R.mnext) {
            uint32_t ni = R.mi % 16u, sl = R.mi % 3u;
            uint32_t tp = (uint32_t)(((uint64_t)R.mtempo * (65536 + ((16384 * sn(R.p07)) >> 15))) >> 16);
            R.mi++;
            R.mb[sl].f = (uint32_t)(((uint64_t)R.mroot * pw2((int32_t)R.mel[ni] * 100 + R.mdet[ni])) >> 16);
            R.mb[sl].age = 0;
            for (h = 0; h < 3u; h++) R.mb[sl].e[h] = E30;
            R.mnext = n + (ni == 15u ? tp * 32u / 10u : tp);
        }
        for (q = 0; q < 3u; q++) {
            if (R.mb[q].e[0] && R.mb[q].age < 176400u) {
                R.mb[q].age++;
                for (h = 0; h < 3u; h++) {
                    R.mb[q].ph[h] += (uint32_t)(((uint64_t)hz8p(R.mb[q].f) * MR[h]) >> 8);
                    y += (int32_t)(((int64_t)sn(R.mb[q].ph[h]) * MA[h] >> 15) * (R.mb[q].e[h] >> 15) >> 15);
                    edec(&R.mb[q].e[h], h == 0u ? RT(1400) : h == 1u ? RT(500) : RT(200));
                }
            }
        }
        y = (y * 5200) >> 15;
        break; }
    }
    return y;
}

static const uint8_t JPG[3][4][2] = {{{2, 1}, {7, 2}, {0, 3}, {9, 2}}, {{0, 3}, {9, 1}, {2, 1}, {7, 2}}, {{2, 1}, {7, 2}, {0, 1}, {5, 1}}};
static const uint8_t JQ[4][4] = {{0, 0, 0, 0}, {0, 3, 7, 10}, {0, 4, 7, 10}, {0, 4, 7, 11}};
static const uint8_t JHI[5] = {0, 3, 5, 7, 10};
static void npush(note_t *a, uint32_t n_slots, uint32_t fq8, int32_t amp)       /* a piano note: one slot of the list */
{
    uint32_t i, k = 0, oldest = 0;
    for (i = 0; i < n_slots; i++) { if (!a[i].e[0]) { k = i; goto got; } if (a[i].age > oldest) { oldest = a[i].age; k = i; } }
got:
    a[k].f = fq8; a[k].age = 0; a[k].amp = amp; a[k].e[0] = E30; a[k].e[1] = E30; a[k].e[2] = E30; a[k].e[3] = E30; a[k].att = E30;
    a[k].ph[0] = a[k].ph[1] = a[k].ph[2] = a[k].ph[3] = 0;
}
static void jpush(note_t *a, uint32_t n_slots, uint32_t midi, int32_t amp) { npush(a, n_slots, HZ8[midi], amp); }

static const uint8_t BLG[12][2] = {{0, 2}, {0, 2}, {0, 2}, {0, 2}, {5, 2}, {5, 2}, {0, 2}, {0, 2}, {7, 2}, {5, 2}, {0, 2}, {7, 2}};   /* the twelve bars of a blues */
static const uint8_t DXG[4][2] = {{0, 3}, {0, 3}, {7, 2}, {0, 3}};
/* old music on a radio: 6 swing jazz, 15 slow blues, 16 ballad, 17 ragtime (stride) */
static int32_t jazz(uint32_t ty)
{
    int32_t y = 0, nz = (int32_t)rnd32() >> 17;
    uint32_t n = R.n, q, vi = ty == 6u ? 0u : ty - 14u, jb = R.jbv[vi], js = R.jsv[vi], st = ty == 17u;
    uint32_t bi = n / jb, bar = bi >> 2, bib = bi & 3u, frs = n - bi * jb;
    const uint8_t *ch = ty == 15u ? BLG[bar % 12u] : ty == 17u ? DXG[bar & 3u] : JPG[R.jprog][bar & 3u], *nx = ty == 15u ? BLG[(bar + 1u) % 12u] : ty == 17u ? DXG[(bar + 1u) & 3u] : JPG[R.jprog][(bar + 1u) & 3u], *iv = JQ[ch[1]];
    if (bi != R.jlast) {
        int32_t off, k;
        R.jlast = bi; R.jhit = 0; R.jh1 = E30; R.jh2 = 0;
        off = st ? (bib == 2u ? (int32_t)iv[2] : 0) : bib == 0u ? 0 : bib == 1u ? iv[1] : bib == 2u ? iv[2] : (int32_t)((nx[0] + 12u - ch[0]) % 12u) + (rnd16() < 32768u ? -1 : 1);
        k = (((int32_t)(R.jkey + ch[0]) + off) % 12 + 12) % 12;
        if (!st || !(bib & 1u)) { R.jbf = HZ8[36 + k]; R.jbd = 0; R.jbe = E30; }
        if (st ? (bib & 1u) : bib == 0u) for (q = 0; q < 4u; q++) jpush(R.jp, 12, 60u + (R.jkey + ch[0] + iv[q]) % 12u, 24576);
        if (bib == (st ? 3u : 2u) && rnd16() < 39322u) jpush(R.jp, 12, 72u + (R.jkey + JHI[rnd16() * 5u >> 16]) % 12u, 32767);
    }
    if (!st && bib == 1u && !R.jhit && frs >= js) { R.jhit = 1; R.jh2 = E30; for (q = 0; q < 4u; q++) jpush(R.jp, 12, 60u + (R.jkey + ch[0] + iv[q]) % 12u, 18022); }
    for (q = 0; q < 12u; q++) {
        note_t *p = &R.jp[q];
        if (!p->e[0]) continue;
        if (++p->age > 70560u) { p->e[0] = 0; continue; }
        p->ph[0] += hz8p(p->f);
        { int32_t s3 = (sn(p->ph[0]) * 32768 + sn(p->ph[0] * 2u) * 13107 + sn(p->ph[0] * 3u) * 4915) >> 15;
          y += (int32_t)((((int64_t)s3 * p->amp >> 15) * (p->e[0] >> 15) >> 15) * 1600 >> 15); }
        edec(&p->e[0], ty == 16u ? RT(900) : RT(500));
    }
    R.jbp += hz8p(R.jbf);
    lp(&R.jbl, ((((int32_t)R.jbp >> 16) * 26214 + sn(R.jbp) * 19661) >> 15) * 9000 >> 15, 7864);
    { int32_t at = (int32_t)(R.jbd * 123u); if (at > 32767) at = 32767; if (R.jbd < 1000u) R.jbd++;
      y += (int32_t)(((int64_t)R.jbl * (R.jbe >> 15) >> 15) * at >> 15); edec(&R.jbe, RT(450)); }
    R.jhp += ((nz - R.jhp) * 19661) >> 16; R.jhp = nz - R.jhp;
    { int32_t hb = (R.jh1 >> 15) + (int32_t)(((int64_t)(R.jh2 >> 15) * 19661) >> 15);
      y += (((R.jhp * 6554) >> 15) * hb) >> 15; edec(&R.jh1, RT(60)); edec(&R.jh2, RT(60)); }
    if (rnd16() < 9u) y += ((int32_t)rnd16() - 32768) * 9000 >> 16;       /* a pop on the record */
    { int32_t yl = svf(&R.jl, y, 11000, 5000); y = yl - svf(&R.jh, yl, 1700, 5000); }   /* nothing under 200 Hz, nothing over 3 kHz */
    y = (y * 1075) >> 10;
    y = (y * 1075) >> 10;
    return y;
}

static int32_t station2(uint32_t ty)
{
    int32_t y = 0, nz = (int32_t)rnd32() >> 17;
    uint32_t n = R.n, d, q;
    switch (ty) {
    case 6: y = jazz(6u); break;
    case 7: {                                   /* an old man, speaking far away, in no language: a low murmur that rises and falls */
        if (n >= R.onext) {
            uint32_t gap;
            if ((int32_t)R.oleft <= 0) { R.oleft = 7u + (rnd16() * 6u >> 16); gap = MS(1200) + (rnd16() * MS(2000) >> 16); R.opr = 0; }
            else gap = (R.oleft % (2u + (rnd16() * 3u >> 16))) == 0u ? MS(140) + (rnd16() * MS(120) >> 16) : MS(20);
            R.ot0 = R.onext + gap; R.odur = MS(110) + (rnd16() * MS(150) >> 16); R.onext = R.ot0 + R.odur; R.oleft--;
            R.ovw = rnd16() * 6u >> 16; R.onas = rnd16() < 16384u; R.opr += 7864u;
            R.o23 = 0x80000000u / R.odur;
        }
        d = n - R.ot0;
        if (n >= R.ot0 && d < R.odur) {
            uint32_t x = (uint32_t)sn(d * R.o23) , i = x >> 11, fr = x & 2047u;
            int32_t e = POW07[i] + (int32_t)(((int32_t)(POW07[i + 1] - POW07[i]) * (int32_t)fr) >> 11);
            uint32_t f0 = (uint32_t)(((uint64_t)R.of0 * (65536u - (R.opr * 6554u >> 16))) >> 16);
            R.w9 += hz8p(589u);                                                 /* 2.3 Hz */
            f0 = (uint32_t)(((uint64_t)f0 * (uint32_t)(65536 + ((1966 * sn(R.w9)) >> 15))) >> 16);
            y = speak(f0, (R.onas && d < MS(40)) ? 3u : 0u, (uint32_t)OVW[R.ovw][0] * 941u >> 10, (uint32_t)OVW[R.ovw][1] * 941u >> 10, (e * 6) >> 7, 1416);
        }
        break; }
    case 8: {                                   /* a whale, far under the water: long slow moans that bend */
        if (n >= R.wnext) {
            R.wt0 = n; R.wdur = MS(2500) + (rnd16() * MS(2500) >> 16); R.wfa = (90u + (rnd16() * 160u >> 16)) << 8;
            R.wfb = (uint32_t)(((uint64_t)R.wfa * (39322u + (rnd16() * 59000u >> 16))) >> 16);
            R.wnext = n + R.wdur + MS(800) + (rnd16() * MS(3000) >> 16); R.wstep = 0x80000000u / R.wdur;
        }
        d = n - R.wt0;
        if (n >= R.wt0 && d < R.wdur) {
            uint32_t ph = d * R.wstep, x = (uint32_t)sn(ph), i = x >> 11, fr = x & 2047u;
            int32_t we = POW15[i] + (int32_t)(((int32_t)(POW15[i + 1] - POW15[i]) * (int32_t)fr) >> 11), wf;
            uint32_t df = ph >> 16;                                          /* position 0..1 (Q15) */
            R.w9 += hz8p(230u);                                                 /* 0.9 Hz */
            wf = (int32_t)R.wfa + (int32_t)(((int64_t)((int32_t)R.wfb - (int32_t)R.wfa) * (int32_t)df) >> 15) + ((2560 * sn(R.w9)) >> 15);
            for (q = 0; q < 3u; q++) R.wph[q] += (uint32_t)(((uint64_t)hz8p((uint32_t)wf) * (q + 1u)));
            y = (sn(R.wph[0]) + ((sn(R.wph[1] + (uint32_t)(sn(R.wph[0]) * 12515)) * 18022) >> 15) + ((sn(R.wph[2]) * 9830) >> 15));
            y = (int32_t)(((int64_t)y * we >> 15) * 9800 >> 15);
        }
        lp(&R.t1, nz, 1311); y += (R.t1 * 230) >> 8;
        break; }
    case 9: {                                   /* a bell tolling in the fog */
        d = n % R.bper;
        if (d == 0u) { for (q = 0; q < 7u; q++) R.be[q] = E30; R.bat = E30; }
        for (q = 0; q < 7u; q++) {
            R.bph[q] += (uint32_t)(((uint64_t)hz8p(R.bf) * BELLR[q]) >> 8);
            y += (int32_t)(((int64_t)sn(R.bph[q]) * BELLA[q] >> 15) * (R.be[q] >> 15) >> 15);
            edec(&R.be[q], 380449u / BELLT[q]);
        }
        if (d < 529u) y += (((nz * 2) * 29491 >> 15) * (int32_t)(529u - d) * 62) >> 15;
        y = (int32_t)(((int64_t)y * 5400 >> 15) * ((E30 - R.bat) >> 15) >> 15); edec(&R.bat, RT(2));
        lp(&R.bl, y, 16384); y = R.bl;
        break; }
    case 10: {                                  /* a police siren, passing somewhere in the night */
        uint32_t fq;
        int32_t ev;
        R.s45 += hz8p(57u); R.s14 += hz8p(18u);                                 /* 1/4.5 Hz, 1/14 Hz */
        if (R.stype == 0u) fq = 650u + (uint32_t)(550 * (16384 - (sn(R.s45 + 0x40000000u) >> 1)) >> 15);           /* the wail */
        else if (R.stype == 1u) { R.s32 += hz8p(819u); fq = 650u + (700u * (R.s32 >> 16) >> 16); }        /* the yelp */
        else { uint32_t o = R.s17; R.s17 += hz8p(435u); if (R.s17 < o) R.s17f ^= 1u; fq = R.s17f ? 960u : 770u; }  /* the two tones */
        R.sph += HZP(fq);
        ev = 8192 + (int32_t)(((int64_t)24576 * (16384 - (sn(R.s14 + 0x40000000u) >> 1))) >> 15);
        y = (sn(R.sph) + ((sn(R.sph * 2u) * 11469) >> 15) + ((sn(R.sph * 3u) * 6554) >> 15));
        y = (int32_t)(((int64_t)(y * 3600 >> 15) * ev) >> 15);
        break; }
    case 11: {                                  /* a spaceship: a deep hum that breathes, a sweeping whoosh, and little pings from the instruments */
        int32_t hum;
        R.ssa += hz8p(48u * 256u); R.ssb += hz8p(48u * 256u + 230u);
        R.s07 += hz8p(18u); R.s13 += hz8p(33u); R.s97 += HZP(97);
        if ((n & 31u) == 0u) {
            R.sfa = fcoef(180u + (uint32_t)(520 * (16384 + (sn(R.s07) >> 1)) >> 15));
            R.sfb = fcoef(300u + (uint32_t)(2500 * (16384 + (sn(R.s13) >> 1)) >> 15));
        }
        hum = ((((int32_t)R.ssa >> 16) + ((int32_t)R.ssb >> 16)) * 5200 >> 15) + (nz >> 2);
        y = svf(&R.ssva, hum, R.sfa, 5000) + ((sn(R.s97) * 500) >> 15);
        svf(&R.ssvb, nz, R.sfb, 3000); y += R.ssvb.band >> 1;
        if (n >= R.sbn) { R.sbn = n + MS(1300) + (rnd16() * MS(2000) >> 16); R.sbf = 1200u + (rnd16() * 1600u >> 16); R.sbe = E30; }
        R.sbp += HZP(R.sbf);
        y += (int32_t)(((int64_t)sn(R.sbp) * (R.sbe >> 15) >> 15) * 2200 >> 15); edec(&R.sbe, RT(120));
        break; }
    case 12: {                                  /* a car goes by: it grows, passes, and its engine drops */
        int32_t m, ca, tn;
        R.kacc += R.kstep; R.k08 += hz8p(205u);
        m = (int32_t)(R.kacc >> 17) * 2 - 32768;                                 /* -1 .. 1 */
        { int32_t x = (int32_t)(((int64_t)m * m >> 15) * 320 >> 15); ca = x > 255 ? 0 : EXP256[x]; }
        if ((n & 31u) == 0u) {                                                   /* kf = kf0 (1 - 0.18 tanh(5m)) */
            int32_t x = m * 5 >> 3, x2 = (x * x) >> 12, th, a2;                  /* Q12: tanh(5m) by a rational approximation */
            a2 = ((x >> 2) * ((110592 + x2) >> 2)) >> 8;
            th = (a2 * 64) / ((110592 + 9 * x2) >> 6);
            if (th > 4096) th = 4096; else if (th < -4096) th = -4096;
            R.kf = (int32_t)(((int64_t)R.kf0 * (4096 - ((737 * th) >> 12))) >> 12);
        }
        { int32_t kk = (int32_t)(((int64_t)R.kf * (65536 + ((1311 * sn(R.k08)) >> 15))) >> 16);
          R.kph += hz8p((uint32_t)kk); }
        lp(&R.kt1, nz, 1966);
        lp(&R.kl1, ((((int32_t)R.kph >> 16) * 7000 >> 15) + ((sn(R.kph * 2u) * 3500) >> 15)), 7864);
        tn = R.kl1 + ((R.kt1 * 29) >> 6);
        y = (int32_t)(((int64_t)tn * ca) >> 15);
        break; }
    case 13: {                                  /* a helicopter: the blades chop, far, then near, then far */
        int32_t th, fly, x;
        R.hbp += hz8p(R.hrf); R.hacc += R.hstep; R.h38 += HZP(38);
        x = (int32_t)(((R.hbp >> 17) * 320) >> 15); th = x > 255 ? 0 : EXP256[x];
        fly = 11469 + (int32_t)(((int64_t)21299 * (16384 - (sn(R.hacc + 0x40000000u) >> 1))) >> 15);
        lp(&R.hl1, nz, 3277); lp(&R.hl2, R.hl1, 9830);
        y = ((int32_t)(((int64_t)R.hl1 * 4 * th) >> 15) + ((((nz - R.hl1) * 41) >> 10) * (19661 + 13107 * th / 32768) >> 15) + (R.hl2 * 5 >> 2) + (int32_t)(((int64_t)sn(R.h38) * 900 >> 15) * th >> 15));
        y = (int32_t)(((int64_t)y * fly) >> 15);
        break; }
    }
    return y;
}

static void morse_build(void)                   /* three groups of five random letters */
{
    uint32_t g, c, j, k = 0, u = R.mou;
    for (g = 0; g < 3u; g++) {
        for (c = 0; c < 5u; c++) {
            uint32_t code = MCODE[rnd16() * 26u >> 16], len = code >> 5;
            for (j = 0; j < len; j++) {
                R.moq[k++] = (uint16_t)(((code >> (len - 1u - j)) & 1u) ? 3u * u : u);
                if (j < len - 1u) R.moq[k++] = (uint16_t)u;
            }
            if (c < 4u) R.moq[k++] = (uint16_t)(3u * u);
        }
        R.moq[k++] = (uint16_t)(g < 2u ? 7u * u : 14u * u);
    }
    R.moq_n = k;
}

static int32_t station3(uint32_t ty)
{
    int32_t y = 0, nz = (int32_t)rnd32() >> 17;
    uint32_t n = R.n, q;
    switch (ty) {
    case 14: {                                  /* a train: clickety-clack on the joints, a rumble, a horn far away */
        uint32_t m19 = n % 837900u;
        int32_t cl;
        R.tacc += R.tstep; R.t80 += HZP(80);
        if (R.tacc < R.tstep) { R.cl1 = E30; R.tflag = 0; }
        if (!R.tflag && R.tacc >= 773094113u) { R.tflag = 1; R.cl2 = E30; }
        cl = (R.cl1 >> 15) + (int32_t)(((int64_t)(R.cl2 >> 15) * 22938) >> 15);
        edec(&R.cl1, RT(30)); edec(&R.cl2, RT(30));
        svf(&R.tcl, nz, fcoef(1100), 2500);
        lp(&R.tt1, nz, 1966);
        y = (int32_t)((((int64_t)R.tcl.band * cl) >> 15) * 29491 >> 15) + ((sn(R.t80) * cl >> 15) * 1800 >> 15) + ((R.tt1 * 23) >> 6);
        if (m19 < 70560u) {
            int32_t e = (32768 - sn(m19 * 60871u + 0x40000000u)) >> 1;
            R.h349 += HZP(349); R.h440 += HZP(440);
            y += (int32_t)(((int64_t)(((sn(R.h349) + ((sn(R.h440) * 26214) >> 15)) * 1500) >> 15) * e) >> 15);
        }
        break; }
    case 15: case 16: case 17: y = jazz(ty); break;                /* old blues, a ballad, ragtime */
    case 18: {                                  /* a truck horn: a chord of air, two blasts and a silence */
        uint32_t hc = n % 441000u, on, hf;
        int32_t hs = 0;
        on = R.thk ? (hc < 44100u || (hc > 57330u && hc < 149940u)) : (hc < 15435u || (hc > 26460u && hc < 41895u) || (hc > 52920u && hc < 141120u));
        R.the += (int32_t)((((on ? 32767 : 0) - R.the) * 98) >> 16);
        hf = R.thf + (uint32_t)(((uint64_t)R.thf * 1311u * (uint32_t)(32768 - R.the)) >> 31);
        R.thp[0] += hz8p(hf); R.thp[1] += (uint32_t)(((uint64_t)hz8p(hf) * 323u) >> 8); R.thp[2] += (uint32_t)(((uint64_t)hz8p(hf) * 384u) >> 8);
        for (q = 0; q < 3u; q++) hs += (int32_t)R.thp[q] >> 16;
        lp(&R.thl, (hs * 1833) >> 15, 22938);
        y = (int32_t)(((int64_t)R.thl * R.the) >> 15);
        break; }
    case 20: {                                  /* a typewriter: keys, a bell at the end of the line, the carriage going back */
        int32_t hp, e;
        uint32_t d;
        if (n >= R.twn) {
            if (R.twl == 0u) { R.twl = 25u + (rnd16() * 20u >> 16); R.twn = n + MS(50); }
            else {
                R.twl--; R.twt = n; R.twe = E30; R.twf = E30; R.twp = 0;
                R.twn = n + MS(60) + (uint32_t)(((uint64_t)rnd16() * MS(140)) >> 16);
                if (R.twl == 0u) { R.twd = n + MS(300); R.twr = n + MS(350); R.twn = n + MS(3500) + (uint32_t)(((uint64_t)rnd16() * MS(2000)) >> 16); }
            }
        }
        R.twp += HZP(160); R.twdp += HZP(2300); R.twm += HZP(40);
        hp = nz - lp(&R.xt1, nz, 19661);
        y = (int32_t)(((int64_t)hp * (R.twe >> 15) >> 15) >> 1) + (int32_t)(((int64_t)sn(R.twp) * (R.twf >> 15) >> 15) * 5000 >> 15);
        edec(&R.twe, RT(3)); edec(&R.twf, RT(12));
        if (n >= R.twd) { R.twd = 0xFFFFFFFFu; R.twg = E30; }
        y += (int32_t)(((int64_t)sn(R.twdp) * (R.twg >> 15) >> 15) * 4500 >> 15);
        edec(&R.twg, RT(600));
        if (n >= R.twr) { R.twr = 0xFFFFFFFFu; R.twl = 0u; R.cke = (int32_t)MS(400); }
        if (R.cke > 0) {
            d = MS(400) - (uint32_t)R.cke; R.cke--;
            e = (int32_t)(((int64_t)sn(d * 121745u) * (32768 + sn(R.twm)) >> 16));
            y += (int32_t)(((int64_t)(nz >> 2) * e) >> 15);
        }
        break; }
    case 21: {                                  /* a clock: tick, tock */
        uint32_t m = n % 44100u, h = m % 22050u;
        if (h == 0u) R.twe = E30;
        y = (((sn(h * HZP(m < 22050u ? 1900u : 1450u)) * 6500) >> 15) + (nz >> 2));
        y = (int32_t)(((int64_t)y * (R.twe >> 15)) >> 15);
        edec(&R.twe, RT(12));
        break; }
    case 22: {                                  /* wind in an empty street */
        R.wn1 += hz8p(23); R.wn2 += hz8p(79); R.wn3 += hz8p(33);
        if ((n & 31u) == 0u) {
            int32_t a = (32768 + sn(R.wn1)) >> 1, b = 19661 + ((sn(R.wn2) * 13107) >> 15);
            R.wnfc = fcoef((uint32_t)(300 + ((((700 * a) >> 15) * b) >> 15)));
        }
        svf(&R.wnd, nz, R.wnfc, 2500);
        { int32_t s3 = sn(R.wn3 + 683565275u), g = 14746 + ((18022 * ((s3 * s3) >> 15)) >> 15);
          y = (int32_t)(((int64_t)R.wnd.band * g >> 15) * 11469 >> 13); }
        break; }
    case 23: {                                  /* a neon sign: a hum, and a flicker now and then */
        int32_t e;
        uint32_t d;
        R.nn1 += HZP(120); R.nn2 += HZP(240);
        if (n >= R.nfn) { R.nfn = n + MS(1500) + (uint32_t)(((uint64_t)rnd16() * MS(4000)) >> 16); R.nft = n; }
        d = n - R.nft; e = (d < MS(400) && sn(d * HZP(23)) < 0) ? 1 : 0;
        y = (((int32_t)R.nn1 >> 16) * 2600 >> 15) + (sn(R.nn2) * 1200 >> 15) + ((nz * 5) >> 7);
        if (e) y = ((y * 5) >> 5) + (nz >> 3);
        break; }
    case 19: {                                  /* morse: someone, far away, sending groups of five letters */
        if (n >= R.mot) {
            if (R.moi >= R.moq_n) { morse_build(); R.moi = 0; }
            R.mon = !(R.moi & 1u); R.mot = n + R.moq[R.moi]; R.moi++;
        }
        R.moe += (int32_t)((((R.mon ? 32767 : 0) - R.moe) * 786) >> 16);
        R.mop += HZP(R.mof);
        y = (int32_t)(((int64_t)sn(R.mop) * R.moe >> 15) * 6500 >> 15);
        break; }
    }
    return y;
}

static void sig_roll(void)                      /* "power on": four stations out of twenty, at four new places, each with a new face */
{
    uint32_t i, j, tries = 0, c[NST], tmp;
    int32_t p[4], a;
    R = (typeof(R)){0};
    R.n = 0; R.ncyc = -1;
    do {
        for (i = 0; i < 4u; i++) p[i] = 30 + (int32_t)(rnd16() * 450u >> 16);
        for (i = 1; i < 4u; i++) for (j = i; j > 0 && p[j - 1] > p[j]; j--) { a = p[j]; p[j] = p[j - 1]; p[j - 1] = a; }
    } while ((p[1] - p[0] < 100 || p[2] - p[1] < 100 || p[3] - p[2] < 100) && ++tries < 2000u);
    { static const uint8_t SC[12] = {0, 2, 10, 9, 14, 12, 18, 7, 20, 21, 22, 23}; for (i = 0; i < 12u; i++) c[i] = SC[i]; }   /* 1930-50, a scene from a film: phone, storm, police car, bell, train, traffic, horn, crowd, typewriter, clock, wind, neon */
    for (i = 11u; i > 0; i--) { j = rnd16() * (i + 1u) >> 16; tmp = c[i]; c[i] = c[j]; c[j] = tmp; }
    for (i = 0; i < 4u; i++) { R.pos[i] = p[i]; R.cont[i] = c[i]; }
    for (i = 0; i < 8u; i++) R.dig[i] = (uint8_t)(rnd16() * 10u >> 16);                             /* a new number to dial */
    R.alow = (int32_t)rr(205, 333);
    R.tper = MS(1000) * rr(4, 7);
    R.sf0 = rr(620, 1350) << 8; R.sper = MS(rr(3600, 6000)); R.sdec = (int32_t)rr(250, 500); R.sdel = MS(rr(700, 1300));
    {                                                                                                  /* the music box: the last note is wrong */
        uint32_t k, st = 4u + rr(0, 3);
        uint8_t A[4], B[4];
        for (k = 0; k < 4u; k++) { st = (uint32_t)((int32_t)st + WALK[rnd16() * 6u >> 16]); st = (int32_t)st < 0 ? 0u : st > 9u ? 9u : st; A[k] = MSC[st]; }
        st = 4u;
        for (k = 0; k < 4u; k++) { st = (uint32_t)((int32_t)st + WALK[rnd16() * 6u >> 16]); st = (int32_t)st < 0 ? 0u : st > 9u ? 9u : st; B[k] = MSC[st]; }
        for (k = 0; k < 4u; k++) { R.mel[k] = A[k]; R.mel[4 + k] = A[k]; R.mel[8 + k] = B[k]; R.mel[12 + k] = k < 3u ? A[k] : (uint8_t)(rnd16() < 32768u ? A[3] + 6 : A[3] - 1); }
        for (k = 0; k < 16u; k++) R.mdet[k] = (int8_t)((int32_t)rr(0, 60) - 30);
    }
    R.mroot = rr(880, 1250) << 8; R.mtempo = MS(rr(340, 550));
    R.jkey = rr(0, 12); R.jbeat = 2646000u / rr(96, 142); R.jprog = rr(0, 3); R.jsw = R.jbeat * rr(58, 68) / 100u; R.jlast = 0xFFFFFFFFu;
    R.jbv[0] = R.jbeat; R.jsv[0] = R.jsw; R.jbv[1] = R.jbeat * 3u >> 1; R.jsv[1] = R.jbv[1] * rr(62, 70) / 100u; R.jbv[2] = R.jbeat << 1; R.jsv[2] = R.jbv[2] * 58u / 100u; R.jbv[3] = R.jbeat * 4u / 5u; R.jsv[3] = R.jbv[3] >> 1;
    R.nf0 = rr(150, 230) << 8;
    R.of0 = rr(88, 118) << 8;
    R.bf = rr(170, 300) << 8; R.bper = MS(rr(2800, 4200));
    R.stype = rr(0, 3);
    R.kf0 = rr(52, 90) << 8; R.kstep = (uint32_t)(0xFFFFFFFFu / (uint32_t)MS(rr(9000, 13000)));
    R.hrf = rr(10, 14) << 8; R.hstep = (uint32_t)(0xFFFFFFFFu / (uint32_t)MS(rr(16000, 24000)));
    R.trp = MS(rr(500, 700)); R.tstep = (uint32_t)(0xFFFFFFFFu / (uint32_t)R.trp);
    R.thf = rr(190, 250) << 8; R.thk = rr(0, 2);
    R.mof = rr(560, 740); R.mou = 52920u / rr(12, 20); R.moq_n = 0; R.moi = 0;
    R.sc1 = 500 << 8; R.sc2 = 1500 << 8;
    R.twn = MS(1000); R.twt = 0xF0000000u; R.twd = R.twr = 0xFFFFFFFFu; R.nfn = MS(3000); R.nft = 0xF0000000u;
    R.wstep = 1; R.o23 = 1;
    for (i = 0; i < 2048u; i++) gd[i] = 0;
    for (i = 0; i < 6615u; i++) gg1[i] = 0;
    for (i = 0; i < 10143u; i++) gg2[i] = 0;
}

static int32_t sig_voice(void)
{
        int32_t nz = (int32_t)rnd32() >> 17, x, sig = 0, gh = (int32_t)P[42];
    uint32_t k, ty;
    R.n++;
    if ((R.n & 31u) == 0u || R.n == 1u) {                              /* every 32 frames: where is the dial, which station is near */
        int32_t fn = (int32_t)P[41], cl = 9830 + (22938 * fn) / 127, dial, best = -1, a1m = 0, a2m = 0, dist = 99999, near, ad, aw;
        uint32_t wp;
        dial = (int32_t)P[40] * 4 * 256 + ((384 * sn(R.n * 4870u)) >> 15);                       /* a little drift: 1.5 units at 0.05 Hz */
        for (k = 0; k < 4u; k++) {
            int32_t D = dial - R.pos[k] * 256, aD = D < 0 ? -D : D, a1 = 0, a2 = 0;
            if (aD < 25600) {
                int32_t d2 = (int32_t)(((int64_t)D * D) >> 12);                                   /* D squared x 16 */
                int32_t x1 = (d2 * 254) / (1270 + 90 * fn), x2 = d2 / 625;
                a1 = x1 > 255 ? 0 : EXP256[x1]; a2 = x2 > 255 ? 0 : EXP256[x2];
            }
            if (a1 > a1m) { a1m = a1; best = (int32_t)k; }
            if (a2 > a2m) a2m = a2;
            if (aD < (dist < 0 ? -dist : dist)) dist = D;
        }
        if (best < 0) best = 0;
        R.pn = a1m; R.pw = a2m; R.bk = best;
        near = (int32_t)(((int64_t)a1m * cl) >> 15); if (a2m / 2 > near) near = a2m / 2;
        R.near = near; ad = (dist < 0 ? -dist : dist) >> 8; R.ad = ad;
        wp = 30u + 45u * (uint32_t)ad; if (wp > 2800u) wp = 2800u; R.wf = wp;
        if (ad < 60) {
            int32_t u = 32768 - (ad * 32768) / 60, i = u >> 11, f = u & 2047, pw = POW15[i] + (((POW15[i + 1] - POW15[i]) * f) >> 11);
            int32_t pc = (int32_t)(((int64_t)a1m * cl) >> 15), pc2 = (int32_t)(((int64_t)pc * pc) >> 15);
            aw = (int32_t)(((int64_t)pw * (32768 - pc2)) >> 15);
        } else aw = 0;
        R.aw = aw;
        R.fcs = fcoef(300u + 6u * (uint32_t)(dial >> 8));
    }
    R.wp += HZP(R.wf);
    x = (int32_t)(((int64_t)sn(R.wp) * R.aw >> 15) * 3600 >> 15);                                    /* the squeal falls as you get close, and stops on the station */
    svf(&R.sv, nz, R.fcs, 7000);
    x += (int32_t)(((int64_t)((R.sv.band * 5) / 256 + (nz * 13) / 256) * (32768 - ((R.near * 26214) >> 15))) >> 15);      /* static, which clears near a station */
    if (R.pn > 328 || R.pw > 656) {
        int32_t cl = 9830 + (22938 * (int32_t)P[41]) / 127;
        int32_t g = ((R.pw * 3932) >> 15) + (int32_t)(((int64_t)((R.pn * 28836) >> 15) * cl) >> 15);
        ty = R.cont[R.bk];
        sig = ty < 6u ? station(ty) : ty < 14u ? station2(ty) : station3(ty);
        if (!(ty == 6u || (ty >= 15u && ty <= 17u))) {                  /* the same old speaker for every scene: narrow, a little crackle */
            int32_t vl;
            if (rnd16() < 7u) sig += (((int32_t)rnd16() - 32768) * 7000) >> 16;
            vl = svf(&R.vf, sig, 11000, 5000); sig = vl - svf(&R.vg, vl, 1700, 5000); sig = (sig * 21) >> 4;
        }
        sig = (int32_t)(((int64_t)sig * SGQ[ty] >> 8) * g >> 15);
        x += sig;
    }
    if (gh) {                                                           /* GHOST: the further you push it, the less the radio is a radio */
        int32_t g = GH125[gh], dl, i0, f2, yy, e1, e2, rg, cs;
        static uint32_t p37, p51, p11;
        p37 += hz8p(95u); p51 += hz8p(1306u); p11 += hz8p(28u);
        { int64_t w = (int64_t)sn(p37) * (280 * 256) + (int64_t)sn(p51) * (8 * 256);
          dl = 400 * 256 + (int32_t)(((w >> 15) * g) >> 15); }                                           /* the delay in 1/256 samples: the tape wobbles */
        i0 = dl >> 8; f2 = dl & 255;
        gd[R.gw & 2047] = (int16_t)(x > 32767 ? 32767 : x < -32767 ? -32767 : x);
        yy = gd[(R.gw - i0) & 2047] + (((gd[(R.gw - i0 - 1) & 2047] - gd[(R.gw - i0) & 2047]) * f2) >> 8); R.gw++;
        x = (int32_t)(((int64_t)x * (32768 - g) + (int64_t)yy * g) >> 15);
        R.rp += hz8p((60u + (uint32_t)((340 * (16384 + (sn(p11) >> 1))) >> 15)) * 256u);
        rg = (int32_t)(((int64_t)g * g >> 15) * 24576 >> 15);                                        /* 0.75 g^2 */
        x = (int32_t)(((int64_t)x * (32768 - rg)) >> 15) + (int32_t)(((((int64_t)x * sn(R.rp) >> 15) * rg >> 15) * 45875) >> 15);   /* a metal voice that wanders */
        e1 = gg1[R.gi1]; e2 = gg2[R.gi2];                                                            /* two echoes in an empty room */
        { int32_t w1 = x + (int32_t)(((int64_t)e1 * 18022 >> 15) * g >> 15), w2 = x + (int32_t)(((int64_t)e2 * 16384 >> 15) * g >> 15);
          gg1[R.gi1] = (int16_t)(w1 > 32767 ? 32767 : w1 < -32767 ? -32767 : w1); gg2[R.gi2] = (int16_t)(w2 > 32767 ? 32767 : w2 < -32767 ? -32767 : w2); }
        if (++R.gi1 >= 6615) R.gi1 = 0;
        if (++R.gi2 >= 10143) R.gi2 = 0;
        x += (int32_t)(((int64_t)((e1 * 19661 >> 15) + (e2 * 16384 >> 15)) * g) >> 15);
        if (R.cn <= 0 && rnd32() < (uint32_t)g * 1783u / 1000u) {                                            /* little signals from somewhere else */
            R.cn = (int32_t)MS(50) + (int32_t)(rnd16() * MS(200) >> 16); R.cl = R.cn; R.cf = rr(1500, 5000) << 8;
            cs = 16384 + (int32_t)(rnd16() * 32768u >> 16); R.cs = rnd16() < 32768u ? -cs : cs; R.cstep = 0x80000000u / (uint32_t)R.cl; R.cprog = 0;
        }
        if (R.cn > 0) {
            int32_t pr = (int32_t)(R.cprog >> 16);                                                  /* 0 .. 1 over the chirp (Q15) */
            uint32_t finc = (uint32_t)(((uint64_t)hz8p(R.cf) * (uint32_t)(65536 + ((R.cs * pr) >> 14))) >> 16);
            R.cph += finc; R.cprog += R.cstep; if (pr > 32767) pr = 32767;
            x += (int32_t)((((int64_t)sn(R.cph) * sn((uint32_t)pr << 16) >> 15) * g >> 15) * 2200 >> 15);
            R.cn--;
        }
    }
    return soft(x);
}


static fx_t FXS;


/* ---- SFX: a noise machine, like the radio. GHOST and ALIEN: a knob picks the sound (nine each), it keeps coming by itself, on any voice.
   One table row per sound: ms, Hz start / mid / end, vibrato Hz and per-mille, tremolo Hz and %, wave (0 sine 1 saw 2 square 3 noise), FM ratio x10 and index x10,
   noise %, two formants (start, end) x2, attack ms, release ms, repeats, gap ms, step (semitone x10) and chance (semitone x10) per repeat, gain %  */
static const int16_t SFXT[2][9][23] = {
 {
  {2200,230,190,130,5,20,0,0,0,10,25,8,420,300,800,700,500,700,1,0,0,0,18},
  {1600,200,200,200,0,0,0,0,3,0,0,100,1800,2800,3200,3600,400,600,1,0,0,0,67},
  {2600,320,900,280,5,40,0,0,0,10,15,5,500,800,1500,2200,300,700,1,0,0,0,54},
  {110,320,270,230,0,0,0,0,1,0,0,18,700,800,1200,1100,6,20,6,95,-10,18,54},
  {1300,55,90,70,0,0,30,60,1,0,0,25,600,900,1400,1800,50,200,1,0,0,0,160},
  {1000,170,140,110,0,0,0,0,0,10,20,6,300,320,700,650,80,300,1,0,0,0,19},
  {45,1900,1700,1500,0,0,0,0,0,35,60,25,2500,2500,4000,4000,1,40,7,70,0,60,101},
  {2800,600,880,540,6,35,0,0,0,0,0,0,0,0,0,0,200,500,1,0,0,0,214},
  {3200,100,100,100,0,0,0,0,3,0,0,100,300,900,1200,300,900,1200,1,0,0,0,144},
 },
 {
  {260,3000,900,150,0,0,0,0,1,0,0,0,0,0,0,0,2,40,1,0,0,0,220},
  {1600,420,780,430,12,120,0,0,0,20,35,0,0,0,0,0,200,400,1,0,0,0,230},
  {70,900,1300,1700,0,0,0,0,0,0,0,0,0,0,0,0,2,15,6,55,30,15,217},
  {95,400,560,330,9,50,0,0,1,0,0,4,800,1500,1800,2400,6,25,7,45,0,50,48},
  {1500,90,1800,2200,0,0,18,40,1,20,50,0,0,0,0,0,20,200,1,0,0,0,320},
  {310,1300,800,550,0,0,0,0,2,0,0,25,1400,900,2200,1700,4,50,2,60,0,0,32},
  {120,260,750,500,0,0,0,0,0,0,0,0,0,0,0,0,3,25,4,60,-25,20,213},
  {14,3000,3000,3000,0,0,0,0,3,0,0,100,2500,2600,4200,4200,1,5,8,50,0,0,53},
  {3000,92,97,90,0,0,7,45,0,15,35,0,0,0,0,0,300,500,1,0,0,0,279},
 },
};

static int16_t fdl[16384] __attribute__((section(".pool")));
static uint32_t fdp;
static fx_t FXF;
static struct {
    int32_t act, last, oct; uint32_t wait, seed, ph, vp, tp, tn, rep, per, durS, att, rel, inv, inva, invr, mrep, sqq, idx, cnt;
    svf_t a, b; int32_t f1, f2;
} FX[2];

static uint32_t pow2q(int32_t oq8)              /* 2^(oq8 / 256) in Q16 */
{
    int32_t i = oq8 >> 8;
    uint32_t f = (uint32_t)(oq8 & 255) << 4, p, r;
    p = 4096u + ((f * (2839u + ((f * (984u + ((f * 227u) >> 12))) >> 12))) >> 12);   /* Q12: 1 + f (0.6931 + f (0.2402 + f 0.0555)) */
    r = p << 4;
    return i >= 0 ? r << i : r >> -i;
}
static uint32_t fx_rand(uint32_t c) { FX[c].seed = FX[c].seed * 1664525u + 1013904223u; return FX[c].seed >> 8; }
static void sfx_rep(uint32_t c)                 /* each repeat has its own pitch */
{
    const int16_t *Q = SFXT[c][FX[c].idx];
    int32_t oq = FX[c].oct + (Q[20] * (int32_t)FX[c].rep * 256) / 120 + (((int32_t)(fx_rand(c) % 1001u) - 500) * Q[21] * 256) / 60000;
    FX[c].mrep = pow2q(oq);
}
static void sfx_trig(uint32_t c, uint32_t i)    /* a new sound: its pitch, its length are drawn each time */
{
    const int16_t *Q = SFXT[c][i];
    uint32_t ds = 230u + (((fx_rand(c) & 255u) * 64u) >> 8);
    FX[c].oct = ((int32_t)(fx_rand(c) % 20u) - 7) * 256 / 12 + octv[6] * 256;
    FX[c].idx = i;
    FX[c].durS = ((uint32_t)Q[0] * ds * 441u) / 2560u;
    FX[c].per = FX[c].durS + ((uint32_t)Q[19] * 441u) / 10u;
    FX[c].att = ((uint32_t)Q[16] * 441u) / 10u; if (FX[c].att < 1u) FX[c].att = 1u; if (FX[c].att > FX[c].durS / 2u) FX[c].att = FX[c].durS / 2u + 1u;
    FX[c].rel = ((uint32_t)Q[17] * 441u) / 10u; if (FX[c].rel > FX[c].durS - FX[c].att) FX[c].rel = FX[c].durS - FX[c].att; if (FX[c].rel < 1u) FX[c].rel = 1u;
    FX[c].inv = (1u << 30) / FX[c].durS; FX[c].inva = (1u << 30) / FX[c].att; FX[c].invr = (1u << 30) / FX[c].rel;
    FX[c].sqq = pow2q(FX[c].oct / 2);
    FX[c].tn = 0; FX[c].rep = 0; FX[c].ph = 0; FX[c].vp = 0; FX[c].tp = 0; FX[c].cnt = 0; FX[c].act = 1;
    sfx_rep(c);
}
static int32_t sfx_sample(void)
{
    int32_t y = 0, dq, t1, t2, w;
    uint32_t c;
    for (c = 0; c < 2u; c++) {
        const int16_t *Q;
        int32_t sel = f_off || !P[60u + c] ? -1 : (int32_t)(((uint32_t)P[60u + c] - 1u) * 9u / 127u), o, e, nz, f, u, vs, th;
        if (sel != FX[c].last) { FX[c].last = sel; if (sel >= 0) { sfx_trig(c, (uint32_t)sel); FX[c].wait = 0; } else FX[c].act = 0; }
        if (sel < 0) continue;
        if (!FX[c].act) { if (FX[c].wait) { FX[c].wait--; continue; } sfx_trig(c, (uint32_t)sel); }
        Q = SFXT[c][FX[c].idx];
        if (FX[c].tn >= FX[c].durS) {                                     /* the end of a repeat: the gap, or the end of the sound */
            if (FX[c].rep + 1u >= (uint32_t)Q[18]) { FX[c].act = 0; FX[c].wait = 22050u + fx_rand(c) % 110250u; continue; }
            if (++FX[c].tn >= FX[c].per) { FX[c].tn = 0; FX[c].rep++; FX[c].ph = 0; sfx_rep(c); }
            continue;
        }
        u = (int32_t)(((uint64_t)FX[c].tn * FX[c].inv) >> 15);             /* 0..32768 along the repeat */
        if (u < 16384) f = Q[1] + ((Q[2] - Q[1]) * u >> 14); else f = Q[2] + ((Q[3] - Q[2]) * (u - 16384) >> 14);
        f = (int32_t)(((uint32_t)f * FX[c].mrep) >> 16);
        FX[c].vp += (uint32_t)Q[4] * 97392u; FX[c].tp += (uint32_t)Q[6] * 97392u;
        vs = SIN256[FX[c].vp >> 24];
        f += (f * (Q[5] * vs / 1000)) >> 15;
        FX[c].ph += (uint32_t)f * 97392u;
        nz = (int32_t)(fx_rand(c) << 8) >> 16;
        if (Q[8] == 0) { uint32_t ph2 = (uint32_t)(((uint64_t)FX[c].ph * (uint32_t)Q[9] * 6554u) >> 16); o = sn(FX[c].ph + (uint32_t)Q[10] * (uint32_t)sn(ph2) * 2086u); }
        else if (Q[8] == 1) o = (int32_t)(FX[c].ph >> 16) - 32768;
        else if (Q[8] == 2) o = FX[c].ph < 0x80000000u ? 32767 : -32767;
        else o = 0;
        th = Q[8] == 3 ? 32768 : Q[11] * 327;
        o = (o * (32768 - th) + nz * th) >> 15;
        e = 32768;
        if (FX[c].tn < FX[c].att) e = (int32_t)(((uint64_t)FX[c].tn * FX[c].inva) >> 15);
        if (FX[c].durS - FX[c].tn < FX[c].rel) { int32_t er = (int32_t)(((uint64_t)(FX[c].durS - FX[c].tn) * FX[c].invr) >> 15); if (er < e) e = er; }
        if (e > 32768) e = 32768;
        if (Q[7]) e = (e * (32768 - ((Q[7] * 327 * (32768 + SIN256[FX[c].tp >> 24])) >> 16))) >> 15;
        if (Q[12]) {
            if ((FX[c].tn & 15u) == 0u) {                                  /* the two formants follow the sound (updated every 16 samples) */
                int32_t g1 = (int32_t)(((uint32_t)(Q[12] + ((Q[13] - Q[12]) * u >> 15)) * FX[c].sqq) >> 16), g2 = (int32_t)(((uint32_t)(Q[14] + ((Q[15] - Q[14]) * u >> 15)) * FX[c].sqq) >> 16);
                FX[c].f1 = 2 * sn((uint32_t)g1 * 48696u); FX[c].f2 = 2 * sn((uint32_t)g2 * 48696u);
                if (FX[c].f1 > 30000) { FX[c].f1 = 30000; }
                if (FX[c].f2 > 30000) { FX[c].f2 = 30000; }
            }
            svf(&FX[c].a, (o * 12000) >> 15, FX[c].f1, 1800); svf(&FX[c].b, (o * 12000) >> 15, FX[c].f2, 1800);
            o = (FX[c].a.band * 48 + FX[c].b.band * 29) >> 3;
            if (o > 40000) o = 40000; else if (o < -40000) o = -40000;
        }
        y += (((o * e) >> 15) * (Q[22] * 52)) >> 15;
        FX[c].tn++;
    }
    dq = ez(P[62]);                                                      /* DELAY: two echoes that feed back, the sounds pile up into a cloud */
    t1 = fdl[(fdp - 9000u) & 16383u]; t2 = fdl[(fdp - 13300u) & 16383u];
    w = y + ((((t1 * 77 + t2 * 58) >> 7) * (13 + 102 * dq / 127)) >> 7);
    fdl[fdp & 16383u] = (int16_t)(w > 30000 ? 30000 : w < -30000 ? -30000 : w); fdp++;
    y += (((t1 * 128 + t2 * 102) >> 7) * (115 * dq / 127)) >> 7;
    y = fx_run(&FXF, y, (int32_t)P[63]);
    y = soft(y);
    return (y * (int32_t)(VOL[6] + 1u) * 3) >> 8;
}

static void app_audio(int32_t *out, uint32_t n)
{
    static uint32_t blk, dfb = 14746u, q_seen, qr_seen, rl;
    static int32_t ds0, ds1, ds3;
    uint32_t i, t0 = fm1_ticks();
    for (i = 0; i < n; i++) {
        int32_t ya, yb, yq, ys, yd, yf;
        if (blk == 0) {                         /* every 32 frames: envelopes, filter coefficients */
            int32_t idx, bar_fr, fb, mx;
            uint32_t bar;
            lead_block(&LA, 0); lead_block(&LQ, 48); kit_block();
            q_gate = ((uint32_t)step_len * (762u + 14u * P[49])) / 1270u;           /* SEQ STRETCH */
            LA.incb = LA.inc_t;
            {                                                              /* SEQ WARP: at the top of every loop the pitch dives, then climbs back */
                uint32_t qb = 32768u;
                if (sqrun && sqn) {
                    uint32_t ll = sqn * step_len, lpos = ((LQ.pos + sqn - 1u) % sqn) * step_len + LQ.tick;
                    int32_t qf = 32767 - (int32_t)((lpos >> 8) * 32767u / (ll >> 8));
                    if (qf < 0) qf = 0;
                    qb = 32768u - (uint32_t)(((qf * ez(P[55])) >> 7) >> 1);
                }
                LQ.incb = (uint32_t)(((uint64_t)LQ.inc_t * qb) >> 15);
            }
            if (B.seen != b_trig) B.seen = b_trig;
            B.lfo = (B.lfo + 1u) % 1030u;                                  /* MOVE: the bass filter breathes (a slow triangle) */
            {
                int32_t tri2 = (int32_t)B.lfo - 515; tri2 = (tri2 < 0 ? -tri2 : tri2) * 2 - 515;    /* -515 .. +515 */
                int32_t num = tri2 * ez(P[15]) * 2 / 5;
                idx = (int32_t)P[12] + (num + (num < 0 ? -257 : 257)) / 515;
                if (idx > 127) idx = 127; else if (idx < 0) idx = 0;
            }
            B.f = CUT_F[idx]; B.q = resq(P[13]);
            bar = 16u * step_len;                                          /* WARP: at the top of each bar the pitch dives, then climbs back */
            bar_fr = 32767 - (int32_t)((uint32_t)(wph >> 8) * 32767u / (bar >> 8));
            if (bar_fr < 0) bar_fr = 0;
            fb = 32768 - (((bar_fr * ez(P[19])) >> 7) >> 1);
            B.incb = (uint32_t)(((uint64_t)b_inc * (uint32_t)fb) >> 15);
            B.inc2 = B.incb + (uint32_t)(((uint64_t)B.incb * (uint32_t)(8 + (ez(P[14]) >> 1))) >> 11);   /* DETUNE */
            dlen = 3u * step_len;                                          /* the delay: a dotted eighth (halved until it fits, at very slow tempi) */
            while (dlen > 32000u) dlen >>= 1;
            ds0 = ez(P[6]); ds1 = ez(P[18]); ds3 = ez(P[54]);
            mx = ds0 > ds1 ? ds0 : ds1; if (ds3 > mx) mx = ds3;
            dfb = 11000u + 190u * (uint32_t)mx;                            /* the more you send, the longer it rings: 33% .. 107% */
            rl += 32u * 535656u;
            blk = 32;
        }
        blk--;
        if (++wph >= (int32_t)(16u * step_len)) wph = 0;

        /* ARP: the line's clock, a 16th, sample-accurate */
        if (a_run && a_mask) {
            if (LA.tick == 0) arp_step();
            else if (LA.tick == gate_len) LA.gate = 0;
            if (++LA.tick >= step_len) LA.tick = 0;
        } else {
            LA.tick = 0; LA.gate = 0; LA.pos = 0; seq_cur = 255u;
        }
        /* SEQ: heard in EDIT, a loop in PLAY */
        if (q_seen != q_evt) { q_seen = q_evt; LQ.inc_t = q_inc; LQ.inc = q_inc; LQ.incb = q_inc; LQ.gate = 1; LQ.env_f = 32767; LQ.env_a = 32767; }
        if (qr_seen != q_restart) { qr_seen = q_restart; LQ.tick = 0; LQ.pos = 0; }
        if (sqrun && sqn) {
            if (LQ.tick == 0) seq_step();
            else if (LQ.tick == q_gate) LQ.gate = 0;
            if (++LQ.tick >= step_len) LQ.tick = 0;
        } else {
            LQ.tick = 0; if (!q_hold) LQ.gate = 0; LQ.pos = 0; q_cur = 255u; q_idx = 255u;
        }

        ya = (lead_sample(&LA, 0, VOL[0]) * 109) >> 7;
        yq = lead_sample(&LQ, 48, VOL[5]);

        /* WAV */
        yb = 0;
        {
            int32_t g = b_gate, in, lb, dv;
            if (B.amp < g) { B.amp += 600; if (B.amp > g) B.amp = g; }
            else if (B.amp > g) { B.amp -= 10; if (B.amp < g) B.amp = g; }
            if (B.amp) {
                uint32_t c1, c2;
                int32_t fmb = ez(P[20]);
                B.ph1 += B.incb; B.ph2 += B.inc2; B.ph3 += B.incb;
                c1 = B.ph1; c2 = B.ph2;
                if (fmb) {                                                /* FM */
                    int32_t x;
                    uint32_t off;
                    B.mp += B.incb + (uint32_t)(((uint64_t)B.incb * 106u) >> 8);
                    x = (int32_t)B.mp >> 16; x = (x < 0 ? -x : x) - 16384;
                    off = (uint32_t)(x * fmb) << 11;
                    c1 += off; c2 += off;
                }
                in = ((((int32_t)c1 >> 16) + ((int32_t)c2 >> 16)) * 3) >> 3;
                in += (int32_t)B.ph3 < 0 ? 12000 : -12000;
                lb = svf(&B.sv2, svf(&B.sv, in, B.f, 36000), B.f, B.q);
                dv = (int32_t)((P[17] * P[17] + 63u) / 127u);               /* DRIVE: progressive, clean at zero */
                yb = soft((lb * (8 + ((dv * 150) >> 7))) >> 3);
                if (dv) yb = soft(yb + ((yb * dv) >> 6));
                yb = fx_run(&B.fx, yb, (int32_t)P[23]);
                yb = (int32_t)(((int64_t)yb * B.amp * 30) >> 19);
                yb = (yb * (int32_t)(VOL[1] + 1u) * 3) >> 9;                 /* WAV sits a little lower so the others stay heard */
            }
        }

        /* FM1 */
        ys = 0;
        if (!s_off) { ys = sig_voice(); ys = fx_run(&FXS, ys, (int32_t)P[43]); ys = (ys * (int32_t)(VOL[4] + 1u) * 20) >> 9; }

        yd = drums();
        yf = sfx_sample();
        {
            int32_t wl, wr, vl, vr, a0, a1, ml, mr, dry = (int32_t)(((int64_t)(ya + yb + ys + yq) * (32768 - d_duck)) >> 15) + yd + yf;
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
            vl = ((ya * ds0 + yb * ds1 + yq * ds3) >> 7) + ((dlpr * (int32_t)dfb) >> 15);
            vr = (dlpl * (int32_t)dfb) >> 15;
            dl[dpos & DMASK] = (int16_t)soft(vl);                          /* soft: past 100% it rings on and on without exploding */
            dr[dpos & DMASK] = (int16_t)soft(vr);
            dpos++;
            if (sp_hs || sp_na || sp_wa || sp_da || sp_ba) {
                int32_t nz, b, w, q, d;
                sp_seed = sp_seed * 1664525u + 1013904223u;
                nz = (int32_t)sp_seed >> 17;
                svf(&SPF, nz, (int32_t)sp_cf, 7000);
                b = (SPF.band * (int32_t)sp_na) >> 6;                        /* a band of static */
                sp_wp += sp_wi; q = (int32_t)sp_wp >> 16; q = (q < 0 ? -q : q) * 2 - 32768;
                w = (soft(q) * (int32_t)sp_wa) >> 15;                        /* a soft ping, far away */
                sp_d1 += 10713000u; sp_d2 += 10824000u; sp_d3 += 16070000u;   /* the drone: 110 Hz twice (a slow beat) and its fifth */
                q = (int32_t)sp_d1 >> 16; d = soft((q < 0 ? -q : q) * 2 - 32768);
                q = (int32_t)sp_d2 >> 16; d += soft((q < 0 ? -q : q) * 2 - 32768);
                q = (int32_t)sp_d3 >> 16; d += soft((q < 0 ? -q : q) * 2 - 32768) >> 1;
                sp_bp += sp_bi; q = (int32_t)sp_bp >> 16; q = soft((q < 0 ? -q : q) * 2 - 32768);
                dry += ((nz * (int32_t)sp_hs) >> 8) + b + w + ((d * (int32_t)sp_da) >> 15) + ((q * (int32_t)sp_ba) >> 15);   /* and a few bleeps */
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

/* slow, like a title sequence: the logo rises out of the static in waves, LOST SIGNAL follows, and it stays lit. {time ms, brightness 0..256}, linear in between */
static const uint16_t LG[][2] = {{0, 0}, {700, 0}, {1300, 25}, {1600, 10}, {2100, 55}, {2400, 30}, {3000, 100}, {3300, 75}, {3800, 150},
                                 {4100, 120}, {4600, 200}, {4900, 170}, {5300, 256}, {9700, 256}};
static const uint16_t TG[][2] = {{0, 0}, {4200, 0}, {4700, 90}, {5000, 30}, {5400, 170}, {5700, 70}, {6100, 256}, {9700, 256}};
static const uint16_t PINGS[5] = {2600, 3300, 4000, 4800, 5600};   /* soft pings, a slow arpeggio that comes with LOST SIGNAL */
static const uint16_t PING_HZ[5] = {330, 494, 659, 587, 440};
static const uint16_t BIPS[5][4] = {{6300, 50, 1320, 2000}, {6750, 50, 1320, 2000}, {7500, 40, 990, 1700}, {8200, 50, 1320, 2000}, {8550, 140, 1760, 1700}};   /* time, length ms, Hz, level */
#define SPLASH_MS 9700u                         /* the sound fades from 8.4 s to 9.2 s, then half a second of silence, then the screen */

static uint32_t keyf(const uint16_t (*tb)[2], uint32_t n, uint32_t t)    /* brightness at time t */
{
    uint32_t i;
    for (i = 1; i < n; i++)
        if (t < tb[i][0]) {
            int32_t a = tb[i - 1][1], b = tb[i][1], t0 = tb[i - 1][0], t1 = tb[i][0];
            return (uint32_t)(a + (b - a) * ((int32_t)t - t0) / (t1 - t0));
        }
    return tb[n - 1][1];
}

static void splash_snd(uint32_t t)              /* the sound: a deep drone that swells, faint static, soft pings */
{
    static uint32_t cell = ~0u, rs = 7u;
    static uint32_t hsb;
    uint32_t i, w = 0, hz = 0, da, fa = t < 8400u ? 256u : t < 9200u ? 256u * (9200u - t) / 800u : 0u;   /* the fade-out of the static and the drone */
    if (t / 120u != cell) {                     /* the static changes level every 120 ms, with dropouts */
        cell = t / 120u; rs = rs * 1664525u + 1013904223u;
        hsb = t < 300u ? 0u : (rs >> 16 & 7u) < 2u ? 0u : 6u + (rs >> 20 & 15u) * (t < 3000u ? 1u : 2u) / 2u;
    }
    sp_hs = hsb * fa >> 8;
    sp_cf = (uint32_t)CUT_F[70u + (t >> 7 & 7u)];
    sp_na = t < 300u ? 0u : 8u * fa >> 8;
    da = t < 300u ? 0u : t < 5600u ? (t - 300u) * 3800u / 5300u : 3800u;
    sp_da = da * fa >> 8;
    for (i = 0; i < 5u; i++)
        if (t >= PINGS[i] && t < PINGS[i] + 800u) {
            uint32_t d = t - PINGS[i];
            w = d < 40u ? d * 40u : 1600u * (800u - d) / 760u;
            hz = PING_HZ[i];
        }
    sp_wa = w; sp_wi = 97392u * hz;
    hz = 0; w = 0;                              /* the bleeps: short, irregular, mostly towards the end */
    for (i = 0; i < 5u; i++)
        if (t >= BIPS[i][0] && t < BIPS[i][0] + BIPS[i][1] + 15u) {
            uint32_t d = t - BIPS[i][0];
            hz = BIPS[i][2];
            w = d < 5u ? BIPS[i][3] * d / 5u : d < BIPS[i][1] ? BIPS[i][3] : BIPS[i][3] * (BIPS[i][1] + 15u - d) / 15u;
        }
    sp_bi = 97392u * hz; sp_ba = w;
}

static void splash(void)                        /* ARP WAV 09: a signal found slowly in the static, then it holds a moment */
{
    uint32_t t0 = fm1_ms, t, dl_ = ~0u, dt_ = ~0u, kl, kt, tl = 0;
    lcd_fill(0, 0, 240, 240, C_BG);
    do {
        t = fm1_ms - t0;
        platform_poll();
        splash_snd(t);
        if (t - tl >= 40u) {                    /* the picture follows every 40 ms (no faster: the glow rises, it does not flicker) */
            tl = t;
            kl = keyf(LG, sizeof LG / sizeof LG[0], t) & ~3u; kt = keyf(TG, sizeof TG / sizeof TG[0], t) & ~3u;
            if (kl != dl_) { dl_ = kl; draw_img(&IMG_LOGO, (240u - IMG_LOGO.w) / 2u, LOGO_Y, kl); }
            if (kt != dt_) { dt_ = kt; draw_img(&IMG_TAG, (240u - IMG_TAG.w) / 2u, TAG_Y, kt); }
        }
    } while (t < SPLASH_MS);
    sp_hs = sp_na = sp_wa = sp_da = sp_ba = 0;
    lcd_fill(0, 0, 240, 240, C_BG);
}

static char *put(char *p, const char *s) { while (*s) *p++ = *s++; return p; }
static char *put_num(char *p, uint32_t v)
{
    if (v >= 100u) *p++ = (char)('0' + v / 100u);
    if (v >= 10u) *p++ = (char)('0' + v / 10u % 10u);
    *p++ = (char)('0' + v % 10u);
    return p;
}

#define BAT_LO 505u                             /* ADC reading of an empty / a full battery: an estimate (to be set from two real readings) */
#define BAT_HI 630u
static uint32_t bat_pct(uint32_t raw) { return raw <= BAT_LO ? 0u : raw >= BAT_HI ? 100u : (raw - BAT_LO) * 100u / (BAT_HI - BAT_LO); }   /* like a phone: 0..100 % */
static void draw_battery(uint32_t raw)          /* top right: a battery with four bars */
{
    uint32_t lv = (bat_pct(raw) + 12u) / 25u, i;
    uint16_t c = lv == 0u ? RGB(255, 80, 90) : C_LAB;
    cv_begin(40, 14, C_BG);
    cv_rect(0, 2, 30, 10, c);
    cv_rect(1, 3, 28, 8, C_BG);
    cv_rect(30, 5, 3, 4, c);
    for (i = 0; i < lv; i++) cv_rect((int32_t)(3u + 6u * i), 5, 5, 4, c);
    cv_blit(200, 0);
    lcd_sync();
}

#define C_ARP RGB(255, 125, 205)
#define C_WAV RGB(112, 170, 255)
#define C_KIT RGB(176, 152, 255)
#define C_VOX RGB(210, 120, 255)
#define C_FM1 RGB(205, 255, 185)
#define C_SEQ RGB(255, 194, 236)
#define C_SFX RGB(154, 72, 232)
static const uint16_t VCOL[7] = {C_ARP, C_WAV, C_KIT, C_VOX, C_FM1, C_SEQ, C_SFX};
static const uint8_t VORD[6] = {0, 1, 5, 2, 6, 4};              /* the PRESETS knob, left to right: ARP WAV SEQ KIT SFX FM1 */
static const uint8_t PGN[7] = {2, 2, 1, 1, 1, 2, 1};               /* pages per voice */
static const uint8_t UIMAP[7][2][4] = {{{0, 2, 3, 5}, {6, 1, 8, 11}}, {{12, 17, 18, 15}, {14, 19, 20, 23}}, {{24, 25, 26, 27}}, {{36, 37, 38, 39}}, {{40, 41, 42, 43}}, {{48, 50, 51, 54}, {49, 55, 56, 59}}, {{60, 61, 62, 63}}};
static const char *const UINAME[7][2][4] = {
    {{"CUTOFF", "ENV MOD", "DECAY", "RANGE"}, {"DELAY", "STRETCH", "FM", "CRUSH"}},
    {{"CUTOFF", "DRIVE", "DELAY", "MOVE"}, {"DETUNE", "WARP", "FM", "CRUSH"}},
    {{"KICK", "SNARE", "DELAY", "CRUSH"}, {"", "", "", ""}},
    {{"VOWEL", "ROBOT", "DELAY", "CRUSH"}, {"", "", "", ""}},
    {{"TUNE", "FINE", "GHOST", "CRUSH"}, {"", "", "", ""}},
    {{"CUTOFF", "ENV MOD", "DECAY", "DELAY"}, {"STRETCH", "WARP", "FM", "CRUSH"}},
    {{"GHOST", "ALIEN", "DELAY", "CRUSH"}, {"", "", "", ""}}};

static void draw_knob(uint32_t y, uint32_t v, uint16_t col, const char *name)
{
    cv_begin(240, 22, C_BG);
    cv_text(12, 3, &FONT_S, name, C_LAB);
    cv_rect(104, 9, 124, 4, C_TRK);
    cv_rect(104, 9, (int32_t)(v * 124u / 127u), 4, col);
    cv_blit(0, y);
}
static uint32_t row_y(uint32_t tgt, uint32_t row) { return tgt == 5u ? 46u + 22u * row : 60u + 30u * row; }   /* SEQ makes room for its 16 boxes */

static const char *const NOTE_NAME[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
static void draw_grid(uint32_t n, uint32_t cur, uint32_t run, uint32_t play, uint32_t pend)     /* the 16 boxes: notes written, the cursor (EDIT) or the box playing */
{
    uint32_t q, r;
    for (r = 0; r < 2u; r++) {
        cv_begin(240, 20, C_BG);
        for (q = 8u * r; q < 8u * r + 8u; q++) {
            uint32_t on = q < n, here = run ? (play == q) : (cur == q), pd = !run && cur == q && pend != 255u;
            int32_t x = (int32_t)(8u + 28u * (q - 8u * r));
            uint16_t bd = here && !run ? RGB(255, 255, 255) : on ? RGB(120, 78, 106) : RGB(64, 40, 58);
            uint16_t bg = here && run ? C_SEQ : on ? RGB(58, 32, 52) : C_BG;
            cv_rect(x, 0, 26, 20, bd);
            cv_rect(x + 1, 1, 24, 18, bg);
            if (on || pd) {
                char s[5], *p = s;
                uint32_t nt = (uint32_t)((int32_t)(KEY0 + (pd ? pend : (uint32_t)sq[q])) - 12 + 12 * octv[5]);
                p = put(p, NOTE_NAME[nt % 12u]); *p++ = (char)('0' + (nt / 12u - 1u)); *p = 0;
                cv_text(x + (26 - text_w(&FONT_S, s)) / 2, 3, &FONT_S, s, here && run ? RGB(42, 15, 34) : pd ? RGB(190, 190, 190) : C_SEQ);
            }
        }
        cv_blit(0, 160u + 22u * r);
    }
    lcd_sync();
}

static void draw_status(uint32_t bpm, uint32_t tgt, int32_t oct, uint32_t run, uint32_t play, uint32_t cpu, uint32_t bat, uint32_t fr)
{
    char s[44], *p = s;
    p = put_num(p, bpm); p = put(p, "  "); p = put(p, tgt == 0u ? "UP-DN" : tgt == 2u ? "FLOOR" : "");
    if (tgt == 5u) p = put(p, play ? "  PLAY  " : "  EDIT  "); else p = put(p, run ? "  " : "  STOP  ");
    *p++ = oct < 0 ? '-' : '+'; *p++ = (char)('0' + (oct < 0 ? -oct : oct));
    if (fr) p = put(p, "  FREE");                    /* the kit plays at a tempo of its own */
    *p = 0;
    box(206, &FONT_S, s, C_LAB);
    p = put(s, "CPU "); p = put_num(p, cpu); p = put(p, "%   BAT "); p = put_num(p, bat_pct(bat)); p = put(p, "%"); *p = 0;      /* the audio load: the worst call of the last second (a test readout) */
    box(222, &FONT_S, s, C_CPU);
}

/* ---- the main loop: no settings are kept; every start is the same */
static struct { uint8_t v[NP], vol[7], bpm, dbpm, pos, tgt, page; } set;
static uint32_t r_rolled;                       /* the stations of this power-on are drawn at the first touch of FM1 */
static uint32_t q_rec;                          /* SEQ: REC is armed, every key is heard and written */
static uint8_t d_free;                          /* 0: the kit follows the tempo (the default); 1: the kit has a tempo of its own (SEL) */
static const uint8_t DEF[NP] = {50, 0, 30, 45, 0, 0, 0, 0, 0, 0, 0, 0, 80, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 50, 0, 30, 45, 0, 0, 0, 0, 0, 0, 0, 0};

static void set_defaults(void)
{
    uint32_t i;
    for (i = 0; i < NP; i++) set.v[i] = DEF[i];
    for (i = 0; i < 7u; i++) set.vol[i] = 127;
    set.bpm = 80; set.dbpm = 80; d_free = 0; set.pos = 0; set.tgt = 0; set.page = 0;
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

static void seq_key(uint32_t k)                 /* a key is heard; when REC is armed it is also written in the next box */
{
    q_inc = note_inc(KEY0 + k - 12u + (uint32_t)(12 * octv[5])); q_hold = 1; q_evt++;
    if (q_rec) {
        sq[sqc] = (uint8_t)k; sqk[sqc] = (uint8_t)k;
        if (sqc >= sqn) sqn = sqc + 1u;
        sqc = (sqc + 1u) % 16u;
    }
}

static void app_main(void)
{
    uint32_t prev = 0, drawn[5] = {~0u, ~0u, ~0u, ~0u, ~0u}, st_drawn = ~0u, view_drawn = ~0u, grid_drawn = ~0u;
    uint32_t kt[6] = {0, 0, 0, 0, 0, 0}, k, b, rel, b_key = 99, cpu = 0, cpu_t = 0, bat = 0, bat_t = 0, bat_drawn = ~0u, bat_shown = ~0u;
    int32_t e;

    set_defaults();
    for (k = 0; k < NP; k++) P[k] = set.v[k];
    for (k = 0; k < 7u; k++) VOL[k] = set.vol[k];
    apply_tempo(set.bpm); apply_dtempo(set.dbpm);

    splash();

    for (;;) {
        uint32_t keys, down, old, st, tg;
        platform_poll();
        tg = set.tgt;

        keys = fm1_in.notes;
        down = keys & ~prev;
        old = prev;
        prev = keys;

        if (tg == 5u && !keys) q_hold = 0;
        if (down) {
            if (tg == 0u) {                     /* ARP: a chord latches; a fresh press after letting go replaces it */
                uint32_t m = old ? a_mask : 0u;
                if (!a_mask) a_restart = 1u;        /* a chord from nothing: the bar (and the drums) start together */
                a_mask = m | down;
            } else if (tg == 5u) {              /* SEQ: stopped, a key is heard and written in the next box */
                if (!sqrun) { lowest(down, &k); seq_key(k); }
            } else if (tg == 2u || tg == 4u || tg == 6u) {  /* the kit, the radio and the noise machine: the keys do nothing */
                ;
            } else {                            /* WAV is a drone: it rings until another key (the lowest of the new ones) */
                lowest(down, &k);
                b_key = k;
                b_inc = note_inc(KEY0 + k - 24u + (uint32_t)(12 * octv[1]));
                b_trig++;
                b_gate = 32767;
            }
        }

        b = fm1_input_edges(&rel);
        if (b & (1u << BTN_PLAY)) {
            q_rec = 0;
            if (tg == 5u) {
                if (sqrun) { sqrun = 0; sqc = sqn % 16u; }
                else if (sqn) { sqrun = 1; q_restart++; a_restart = 1u; }
            }
        }

        if (b & (1u << BTN_REC) && tg == 5u && !sqrun) q_rec ^= 1u;
        if (b & (1u << BTN_HOME)) { a_mask = 0; b_gate = 0; d_off = 1; s_off = 1; f_off = 1; sqrun = 0; sqn = 0; sqc = 0; q_rec = 0; }
        if (b & (1u << BTN_SEL)) {
            if (tg == 5u) { if (sqn) sqn--; sqc = sqn % 16u; if (!sqn) sqrun = 0; }   /* SEQ: delete the last step */
            else {                              /* the kit lets go of the tempo, or follows it again */
                d_free ^= 1u;
                if (d_free) set.dbpm = set.bpm;
                apply_dtempo(d_free ? set.dbpm : set.bpm);
            }
        }
        if (b & (1u << BTN_FX)) set.page = (uint8_t)((set.page + 1u) % PGN[tg]);
        if (b & (1u << BTN_OCT_DOWN) && octv[tg] > -1) octv[tg]--;
        if (b & (1u << BTN_OCT_UP) && octv[tg] < 1) octv[tg]++;

        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) {   /* left to right: ARP WAV SEQ KIT SFX FM1 */
            int32_t t = (int32_t)set.pos + (e > 0 ? 1 : -1);
            t = t < 0 ? 0 : t > 5 ? 5 : t;
            if ((uint8_t)t != set.pos) { set.page = 0; q_rec = 0; }      /* every voice starts on page A */
            set.pos = (uint8_t)t; set.tgt = VORD[t]; tg = set.tgt;
        }
        for (k = 0; k < 4u; k++) {              /* the four knobs act on the page of the voice */
            uint8_t *v = &set.v[UIMAP[tg][set.page][k]];
            if ((e = fm1_enc_take(ENC_KNOB1 + k)) != 0) {
                int32_t x;
                if (tg == 2u) d_off = 0;
                if (tg == 6u) f_off = 0;
                if (tg == 4u) {
                    if (!r_rolled) { r_seed ^= fm1_ticks() * 2654435761u ^ (fm1_ms << 11) ^ (uint32_t)fm1_adc_read(FM1_ADC_BATT); rnd32(); sig_roll(); r_rolled = 1; }   /* power-on: four stations, new places */
                    s_off = 0;
                }
                x = (int32_t)*v + turn(&kt[k], e);
                *v = (uint8_t)(x < 0 ? 0 : x > 127 ? 127 : x);
            }
        }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {   /* the volume of the voice */
            int32_t x = (int32_t)set.vol[tg] + turn(&kt[4], e);
            set.vol[tg] = (uint8_t)(x < 0 ? 0 : x > 127 ? 127 : x);
        }
        for (k = 0; k < NP; k++) P[k] = set.v[k];
        for (k = 0; k < 7u; k++) VOL[k] = set.vol[k];
        if ((e = fm1_enc_take(ENC_SELECT)) != 0) {
            uint8_t *bp = tg == 2u && d_free ? &set.dbpm : &set.bpm;   /* one tempo for all; the kit follows it, unless SEL has let it go */
            int32_t x = (int32_t)*bp + ((fm1_ms - kt[5] < 90u || e > 1 || e < -1) ? 5 : 1) * e;   /* 1 BPM a click, 5 when you turn fast */
            kt[5] = fm1_ms;
            *bp = (uint8_t)(x < 20 ? 20 : x > 200 ? 200 : x);
            if (tg == 2u && d_free) apply_dtempo(*bp); else { apply_tempo(*bp); if (!d_free) apply_dtempo(*bp); }
        }

        for (k = 0; k < 27u; k++) {             /* held keys, the latched chord, the bass key, the SEQ step that plays */
            uint32_t on = (keys >> k & 1u) || (b_gate && k == b_key) || (a_mask >> k & 1u) || (tg == 5u && sqrun && k == q_cur) || 0;
            fm1_led_key(KEY_LED(k), on);
        }
        fm1_led_key(BTN_PLAY, tg == 5u ? sqrun : 0u);
        fm1_led_key(BTN_REC, tg == 5u && q_rec);
        fm1_led_key(BTN_FX, set.page != 0u);
        fm1_led_key(BTN_SEL, tg == 2u ? d_free : 0u);

        st = tg == 6u ? 9u : tg == 5u ? 7u + set.page : tg == 4u ? 6u : tg == 3u ? 5u : tg == 2u ? 4u : tg * 2u + set.page;   /* the title: ARP A B, WAV A B, KIT, VOX, FM1, SEQ A B */
        if (st != view_drawn) {
            if ((view_drawn == ~0u) || ((view_drawn >= 7u && view_drawn <= 8u) != (st >= 7u && st <= 8u))) lcd_fill(0, 44, 240, 160, C_BG);    /* SEQ has its own layout */
            view_drawn = st;
            cv_begin(198, 36, C_BG); cv_blit(0, 8); lcd_sync();               /* wipe the old title (they differ in width); the battery stays */
            draw_img(IMG_TITLE[st], (240u - IMG_TITLE[st]->w) / 2u, 8u, 256u);
            for (k = 0; k < 5u; k++) drawn[k] = ~0u;
            grid_drawn = ~0u;
        }
        for (k = 0; k < 5u; k++) {
            uint32_t v = k < 4u ? set.v[UIMAP[tg][set.page][k]] : set.vol[tg];
            if (v != drawn[k]) { drawn[k] = v; draw_knob(row_y(tg, k), v, VCOL[tg], k < 4u ? UINAME[tg][set.page][k] : "VOLUME"); }
        }
        if (tg == 5u) {                         /* the 16 boxes */
            uint32_t g = sqn | sqc << 5 | sqrun << 10 | (sqrun ? q_idx : 0u) << 11 | (uint32_t)q_rec << 19 | (uint32_t)(octv[5] + 1) << 27;
            if (g != grid_drawn) { grid_drawn = g; draw_grid(sqn, sqc, sqrun, q_idx, 255u); }
        }
        if (fm1_ms - cpu_t > 1000u) { cpu_t = fm1_ms; cpu = load_peak / 1393u; if (cpu > 99u) cpu = 99u; load_peak = 0; }   /* budget 5805 us = 139320 ticks */
        if (fm1_ms - bat_t > 2000u || !bat) {            /* the battery, every 2 s, smoothed */
            int32_t r = fm1_adc_read(FM1_ADC_BATT);
            bat_t = fm1_ms;
            if (r >= 0) bat = bat ? (bat * 3u + (uint32_t)r + 2u) / 4u : (uint32_t)r + 1u;
        }
        if (bat != bat_drawn) { bat_drawn = bat; draw_battery(bat); }
        if ((st = (tg == 2u && d_free ? set.dbpm : set.bpm) | (uint32_t)d_free << 28 | tg << 8 | (uint32_t)(octv[tg] + 1) << 12 | (tg == 5u ? sqrun : a_run) << 16 | cpu << 20) != st_drawn || bat != bat_shown) {
            bat_shown = bat;
            st_drawn = st;
            draw_status(tg == 2u && d_free ? set.dbpm : set.bpm, tg, octv[tg], a_run, sqrun, cpu, bat, tg == 2u ? d_free : 0u);
        }
    }
}
