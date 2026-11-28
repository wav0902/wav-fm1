/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 zvenson (from OberVave; the Firmware Hub's look)
 * Copyright (C) 2026 illuminus5 (Skiff: the idea, a loader that draws, its 5 x 7 letters and the order of things) */
/* The update loader's screen (loader.c; tests/ldr_screen_test.c). A unit whose install was cut off comes back to the
 * loader at every power-on and showed a black screen; now it says what it does, in the FirmWare family's look:
 *   an oscilloscope screen (night blue, a faint grid) with the firmware's name in dot-matrix letters, one word per line
 *   what happens (UPDATE MODE, WRITING, UPDATE FAILED) and what to do, white and grey
 *   20 segments: a light running to and fro while it waits (ldr_anim), the share written while it writes, the percent
 *   (nothing at the foot)
 * Drawn with the loader's own letters (the app area may be half written: never read), before USB starts (USB is
 * polled: the panel's ~0.3 s init would leave the host's enumeration unanswered): so the name is lit at once.
 * FM-1 only (the SMK-37's stays dark).
 * LDR_NAME: the name, one or two words (a space between), at most 3 letters per word at the biggest size.
 * LDR_COLOUR / LDR_COLOUR2: the first / second word's colour (RGB565). */
#ifndef LDR_NAME
#define LDR_NAME "ARP WAV"
#endif
#ifndef LDR_COLOUR
#define LDR_COLOUR 0xFBF9u               /* the first word, pink #FF7DCD */
#endif
#ifndef LDR_COLOUR2
#define LDR_COLOUR2 0x755Fu              /* the second word, blue #70AAFF */
#endif
#define LDR_BG 0x0844u                   /* the night blue ground, #090B20 */
#define LDR_INK 0xEF5Eu                  /* the text, #E8E8F5 */
#define LDR_SUB 0x8C97u                  /* the grey, #8E92B8 */
#define LDR_DIM 0x190Au                  /* a segment off, #1C2150 */
#define LDR_GRID 0x10A6u                 /* the scope's grid lines */
#define LDR_AXIS 0x212Au                 /* its centre axes and the edge */
#define LDR_GHOST 0x10C7u                /* a dot off in the name */
#define LDR_PANEL 104u                   /* the scope screen's height */
#define LDR_MIX(a, b) ((((a) >> 11) + ((b) >> 11)) / 2u << 11 | ((((a) >> 5) & 63u) + (((b) >> 5) & 63u)) / 2u << 5 | \
                       (((a) & 31u) + ((b) & 31u)) / 2u)
#define LDR_TAIL LDR_MIX(LDR_COLOUR, LDR_DIM)   /* the waiting light's afterglow: halfway to a segment off */

/* the letters the screens use (others: a space): capitals, digits, % and the hub's address in small letters */
static const char LDR_CH[] = "ABCDEFGHIKLMNOPRSTUVWXY0123456789%abcefhimortuw.";
static const uint8_t LDR_FONT[sizeof LDR_CH - 1u][7] = {         /* rows, bit 4 the left dot */
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},   /* A B */
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},   /* C D */
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},   /* E F */
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}, {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},   /* G H */
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}, {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},   /* I K */
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}, {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11},   /* L M */
    {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11}, {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},   /* N O */
    {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}, {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},   /* P R */
    {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}, {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},   /* S T */
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},   /* U V */
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}, {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},   /* W X */
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04},                                               /* Y */
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},   /* 0 1 */
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E},   /* 2 3 */
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},   /* 4 5 */
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},   /* 6 7 */
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},   /* 8 9 */
    {0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03},                                               /* % */
    {0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F}, {0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x1E},   /* a b */
    {0x00, 0x00, 0x0E, 0x10, 0x10, 0x11, 0x0E}, {0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0E},   /* c e */
    {0x06, 0x09, 0x08, 0x1C, 0x08, 0x08, 0x08}, {0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x11},   /* f h */
    {0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E}, {0x00, 0x00, 0x1A, 0x15, 0x15, 0x11, 0x11},   /* i m */
    {0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E}, {0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10},   /* o r */
    {0x08, 0x08, 0x1C, 0x08, 0x08, 0x09, 0x06}, {0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0D},   /* t u */
    {0x00, 0x00, 0x11, 0x11, 0x15, 0x15, 0x0A}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},   /* w . */
};
#define LDR_SEGS 20u                     /* the segments: 9 px and a 2 px gap, 218 px, centred */
#define LDR_SEG_W 9u
#define LDR_SEG_X 11u
#define LDR_SEG_Y 160u
#define LDR_SEG_H 10u

static uint32_t ldr_missing;                        /* letters drawn as gaps: 0 (ldr_screen_test) */
static uint32_t ldr_len(const char *t)
{
    uint32_t n = 0;
    while (t[n])
        n++;
    return n;
}
static void ldr_text(uint32_t y, uint32_t s, uint16_t c, const char *t)   /* centred, a dot s x s px */
{
    uint32_t n = ldr_len(t), x, r, k, i, b;
    if (!n)
        return;
    for (x = (240u - (n * 6u - 1u) * s) / 2u; *t; t++, x += 6u * s) {
        for (k = 0; LDR_CH[k] && LDR_CH[k] != *t; k++)
            ;
        if (!LDR_CH[k]) {
            ldr_missing += *t != ' ';                   /* (a letter the font lacks: a gap) */
            continue;
        }
        for (r = 0; r < 7u; r++)
            for (i = 0; i < 5u; i = b) {                /* each run of dots in the row: one fill */
                for (; i < 5u && !(LDR_FONT[k][r] >> (4u - i) & 1u); i++)
                    ;
                for (b = i; b < 5u && (LDR_FONT[k][r] >> (4u - b) & 1u); b++)
                    ;
                if (b > i)
                    lcd_fill(x + i * s, y + r * s, (b - i) * s, s, c);
            }
    }
}
static void ldr_dots(uint32_t y, uint32_t s, uint16_t c, const char *t, uint32_t n)   /* a word in dots, centred: s px a cell */
{
    uint32_t l, r, q, k, x = (240u - (n * 6u - 1u) * s) / 2u;
    for (l = 0; l < n; l++, x += 6u * s) {
        for (k = 0; LDR_CH[k] && LDR_CH[k] != t[l]; k++)
            ;
        if (!LDR_CH[k]) {
            ldr_missing += t[l] != ' ';
            continue;
        }
        for (r = 0; r < 7u; r++)
            for (q = 0; q < 5u; q++)
                lcd_fill(x + q * s, y + r * s, s - 1u, s - 1u, LDR_FONT[k][r] >> (4u - q) & 1u ? c : LDR_GHOST);
    }
}
static void ldr_scope(void)                         /* the scope screen: faint grid, centre axes, an edge under it */
{
    uint32_t g;
    for (g = 1; g < 8u; g++)
        lcd_fill(g * 30u, 0, 1, LDR_PANEL, g == 4u ? LDR_AXIS : LDR_GRID);
    for (g = 1; g < 4u; g++)
        lcd_fill(0, g * 26u, 240, 1, g == 2u ? LDR_AXIS : LDR_GRID);
    lcd_fill(0, LDR_PANEL, 240, 1, LDR_AXIS);
}
static void ldr_seg(uint32_t i, uint16_t c)
{
    lcd_fill(LDR_SEG_X + i * (LDR_SEG_W + 2u), LDR_SEG_Y, LDR_SEG_W, LDR_SEG_H, c);
}
static void ldr_say(const char *what, const char *sub)   /* the status and what to do */
{
    lcd_fill(0, 110, 240, 44, LDR_BG);
    ldr_text(114, 2, LDR_INK, what);
    ldr_text(138, 1, LDR_SUB, sub);
}
static void ldr_percent(uint32_t p)                  /* "57%" under the segments; 0xFFFF: none */
{
    char t[5];
    uint32_t n = 0;
    lcd_fill(0, 176, 240, 26, LDR_BG);
    if (p > 100u)
        return;
    if (p >= 100u)
        t[n++] = '1';
    if (p >= 10u)
        t[n++] = (char)('0' + p / 10u % 10u);
    t[n++] = (char)('0' + p % 10u);
    t[n++] = '%';
    t[n] = 0;
    ldr_text(178, 3, LDR_INK, t);
}

static uint32_t ldr_lit;                            /* segments lit for the write */
static uint32_t ldr_pct = 0xFFFFu;                  /* the percent on the screen */
static uint8_t ldr_writing;                         /* WRITING is on the screen */
static uint8_t ldr_idle = 1;                        /* the waiting light runs (not after a write, nor a failure) */
static uint32_t ldr_pos, ldr_next;                  /* the waiting light: its segment (0..37, to and fro), its next move */
static void ldr_screen(void)
{
    uint32_t i, n1 = 0, n2 = 0, m, s, y;
    const char *w2;
    lcd_init();                                     /* (the panel cleared, on) */
    lcd_fill(0, 0, 240, 240, LDR_BG);
    ldr_scope();
    while (LDR_NAME[n1] && LDR_NAME[n1] != ' ')
        n1++;
    w2 = LDR_NAME[n1] ? LDR_NAME + n1 + 1u : LDR_NAME + n1;
    n2 = ldr_len(w2);
    m = n1 > n2 ? n1 : n2;
    s = m <= 3u ? 6u : m <= 4u ? 5u : m <= 5u ? 4u : 3u;             /* (the dot: the biggest that fits) */
    y = n2 ? (LDR_PANEL - 15u * s) / 2u : (LDR_PANEL - 7u * s) / 2u;
    ldr_dots(y, s, LDR_COLOUR, LDR_NAME, n1);
    if (n2)
        ldr_dots(y + 8u * s, s, LDR_COLOUR2, w2, n2);
    ldr_say("UPDATE MODE", "WAITING FOR THE INSTALL");
    for (i = 0; i < LDR_SEGS; i++)
        ldr_seg(i, LDR_DIM);
    /* (no address at the foot) */
    lcd_sync();
}
static uint32_t ldr_at(uint32_t pos) { return pos < LDR_SEGS ? pos : 2u * LDR_SEGS - 2u - pos; }   /* to and fro */
static void ldr_anim(uint32_t ms)                   /* while it waits: a light runs to and fro, its afterglow behind */
{
    uint32_t n = 2u * LDR_SEGS - 2u;
    if (!ldr_idle || (int32_t)(ms - ldr_next) < 0)
        return;
    ldr_next = ms + 60u;                            /* (a step every 60 ms: across in 1.2 s) */
    ldr_seg(ldr_at((ldr_pos + n - 1u) % n), LDR_DIM);    /* the old afterglow goes out */
    ldr_seg(ldr_at(ldr_pos), LDR_TAIL);                  /* the light becomes the afterglow */
    ldr_pos = (ldr_pos + 1u) % n;
    ldr_seg(ldr_at(ldr_pos), LDR_COLOUR);
    lcd_sync();
}
static void ldr_bar(uint32_t done, uint32_t total)
{
    uint32_t n = done * LDR_SEGS / total, i;         /* (the app area: below 2^32 / 20) */
    uint32_t p = done * 100u / total;               /* (the app area * 100: below 2^32) */
    if (!ldr_writing || n < ldr_lit) {              /* a write starts (again) */
        ldr_idle = 0;
        ldr_say("WRITING", "DO NOT SWITCH OFF");
        for (i = 0; i < LDR_SEGS; i++)
            ldr_seg(i, LDR_DIM);
        ldr_lit = 0;
        ldr_writing = 1;
        ldr_percent(ldr_pct = 0);
    }
    for (; ldr_lit < n && ldr_lit < LDR_SEGS; ldr_lit++)
        ldr_seg(ldr_lit, LDR_COLOUR);
    if (p > 100u)
        p = 100u;
    if (p != ldr_pct)                               /* (the number only when it changes: few fills) */
        ldr_percent(ldr_pct = p);
    lcd_sync();
}
static void ldr_failed(void)                        /* a session that wrote nothing or stopped: the way out */
{
    uint32_t i;
    ldr_say("UPDATE FAILED", "INSTALL AGAIN");
    for (i = 0; i < LDR_SEGS; i++)
        ldr_seg(i, LDR_DIM);
    ldr_percent(ldr_pct = 0xFFFFu);
    lcd_sync();
    ldr_writing = 0;
    ldr_idle = 0;
}
