/**
 * mechdng: Mechanical Shutter DNG Interceptor
 * v2 — JPEG+DNG workflow with EXIF metadata
 *
 * Saves uncompressed 14-bit DNG with full EXIF after mechanical shutter.
 * Uses its own minimal DNG writer with chunked byte-swap (256KB).
 * Canon handles JPEG natively — set quality to RAW+JPEG for both.
 *
 * New in v2:
 *   - EXIF sub-IFD: shutter, aperture, ISO, focal length, lens, datetime
 *   - camera.wait: poll lens_info.job_state before writing
 *   - Optimized byte-swap (32-bit word operations)
 *   - Make/Model/Software tags in IFD0
 */

#include <module.h>
#include <dryos.h>
#include <property.h>
#include <bmp.h>
#include <menu.h>
#include <config.h>
#include <raw.h>
#include <shoot.h>
#include <lens.h>

static CONFIG_INT("mechdng.enabled", mechdng_enabled, 0);
static int is_saving = 0;
static int last_file_number = -1;

/* =====================================================================
 * EXIF metadata capture
 * ===================================================================*/

struct mechdng_exif {
    int shutter_num;        /* ExposureTime numerator */
    int shutter_den;        /* ExposureTime denominator */
    int aperture_num;       /* FNumber numerator (f-number × 10) */
    int aperture_den;       /* FNumber denominator (always 10) */
    int iso;                /* linear ISO (100, 200, 400...) */
    int focal_len;          /* focal length in mm */
    char lens_name[32];     /* lens model string */
    char datetime[20];      /* "YYYY:MM:DD HH:MM:SS\0" */
    int wb_r_num;           /* AsShotNeutral R: num/den */
    int wb_r_den;
    int wb_g_num;           /* AsShotNeutral G: always 1/1 */
    int wb_g_den;
    int wb_b_num;           /* AsShotNeutral B: num/den */
    int wb_b_den;
};

/* Convert raw APEX ISO (1/8 EV) to linear ISO value */
static int raw_iso_to_linear(int raw_iso)
{
    if (raw_iso <= 0) return 100;
    /* APEX: raw 72 = ISO 100, each +8 = 1 stop */
    int stops8 = raw_iso - 72;
    if (stops8 < 0) stops8 = 0;
    int full = stops8 / 8;
    int frac = stops8 % 8;
    int base = 100 << full;
    switch (frac) {
        case 3: return base * 125 / 100;   /* +1/3 EV */
        case 4: return base * 141 / 100;   /* +1/2 EV */
        case 5: return base * 160 / 100;   /* +2/3 EV */
        default: return base;
    }
}

/* Convert raw APEX shutter to EXIF ExposureTime rational.
 * Uses "marketing" denominators: 1/30, 1/60, 1/125, 1/250... */
static void raw_shutter_to_rational(int raw, int* num, int* den)
{
    /* APEX: raw 56 = 1s, each +8 = 1 stop faster */
    static const int speed[] = {
        1, 2, 4, 8, 15, 30, 60, 125, 250, 500, 1000, 2000, 4000, 8000
    };

    if (raw <= 0) { *num = 1; *den = 1; return; }

    if (raw <= 56) {
        /* >= 1 second */
        int stops = (56 - raw + 4) / 8;
        *num = 1 << stops;
        *den = 1;
    } else {
        /* < 1 second: 1/N */
        int stops = (raw - 56 + 4) / 8;
        *num = 1;
        *den = (stops < 14) ? speed[stops] : 8000;
    }
}

/* Convert raw APEX aperture to f-number × 10 */
static int raw_aperture_to_x10(int raw)
{
    /* APEX: raw 8 = f/1.0, each +8 = 1 stop */
    static const int fnum_x10[] = {
        10, 14, 20, 28, 40, 56, 80, 110, 160, 220, 320
    };

    if (raw <= 0) return 28;
    int stops = (raw - 8 + 4) / 8;
    if (stops < 0) stops = 0;
    if (stops > 10) stops = 10;
    return fnum_x10[stops];
}

/**
 * Compute AsShotNeutral from WB preset mode.
 * Values are approximate 1/multiplier for 5D3 sensor, G normalized to 1.0.
 * Measured from 5D3 CR2 WB tags at each preset.
 */
static void wb_preset_to_neutral(int wb_mode, int* r_num, int* r_den,
                                               int* b_num, int* b_den)
{
    *r_den = 1000000;
    *b_den = 1000000;

    switch (wb_mode) {
        case 1:  /* Sunny ~5200K */
            *r_num = 459000; *b_num = 640000; break;
        case 2:  /* Cloudy ~6000K */
            *r_num = 420000; *b_num = 710000; break;
        case 3:  /* Tungsten ~3200K */
            *r_num = 620000; *b_num = 380000; break;
        case 4:  /* Fluorescent ~4000K */
            *r_num = 530000; *b_num = 480000; break;
        case 5:  /* Flash ~6000K */
            *r_num = 425000; *b_num = 700000; break;
        case 8:  /* Shade ~7000K */
            *r_num = 390000; *b_num = 760000; break;
        default: /* Auto / Daylight ~5500K fallback */
            *r_num = 473635; *b_num = 624000; break;
    }
}

/* Capture current exposure metadata from lens_info + RTC */
static void mechdng_capture_exif(struct mechdng_exif* exif)
{
    exif->iso = raw_iso_to_linear(lens_info.raw_iso);
    raw_shutter_to_rational(lens_info.raw_shutter,
                            &exif->shutter_num, &exif->shutter_den);
    exif->aperture_num = raw_aperture_to_x10(lens_info.raw_aperture);
    exif->aperture_den = 10;
    exif->focal_len = lens_info.focal_len;
    snprintf(exif->lens_name, sizeof(exif->lens_name), "%s", lens_info.name);

    struct tm now;
    LoadCalendarFromRTC(&now);
    snprintf(exif->datetime, sizeof(exif->datetime),
             "%04d:%02d:%02d %02d:%02d:%02d",
             now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
             now.tm_hour, now.tm_min, now.tm_sec);

    /* White balance → AsShotNeutral {R, G, B} as rationals.
     * G is always 1/1 (reference channel). */
    exif->wb_g_num = 1000000;
    exif->wb_g_den = 1000000;

    if (lens_info.wb_mode == WB_CUSTOM
        && lens_info.WBGain_R > 0
        && lens_info.WBGain_G > 0
        && lens_info.WBGain_B > 0)
    {
        /* Variant C: exact multipliers from PROP_CUSTOM_WB.
         * Canon gains: 1024 = 1.0x.  R gain > G means red is amplified,
         * so AsShotNeutral_R = G/R (inverse of amplification). */
        exif->wb_r_num = lens_info.WBGain_G;
        exif->wb_r_den = lens_info.WBGain_R;
        exif->wb_b_num = lens_info.WBGain_G;
        exif->wb_b_den = lens_info.WBGain_B;
    }
    else
    {
        /* Variant B: approximate from WB preset mode */
        wb_preset_to_neutral(lens_info.wb_mode,
                             &exif->wb_r_num, &exif->wb_r_den,
                             &exif->wb_b_num, &exif->wb_b_den);
    }
}

/* =====================================================================
 * Minimal DNG writer — self-contained, no dependency on chdk-dng.c
 * v2: includes EXIF sub-IFD with exposure metadata
 * ===================================================================*/

/* TIFF types */
#define T_BYTE      1
#define T_ASCII     2
#define T_SHORT     3
#define T_LONG      4
#define T_RATIONAL  5
#define T_SRATIONAL 10

/* 5D3 color matrix from dcraw (sRGB D65) — 9 signed rationals */
static int32_t color_matrix1[] = {
     6722, 10000,  -635, 10000,  -963, 10000,
    -4287, 10000, 12460, 10000,  2028, 10000,
     -908, 10000,  2162, 10000,  5668, 10000
};

/* Helper: write 16-bit LE */
static void put16(uint8_t* p, uint16_t v) { p[0]=v; p[1]=v>>8; }
/* Helper: write 32-bit LE */
static void put32(uint8_t* p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }

/* Write one IFD entry (12 bytes) */
static void ifd_entry(uint8_t* p, uint16_t tag, uint16_t type,
                      uint32_t count, uint32_t value)
{
    put16(p+0, tag);
    put16(p+2, type);
    put32(p+4, count);
    put32(p+8, value);
}

/* String constants */
static const char str_make[]     = "Canon";
static const char str_model[]    = "Canon EOS 5D Mark III";
static const char str_software[] = "Magic Lantern / mechdng";

/**
 * Build a DNG header with EXIF sub-IFD.
 *
 * Layout:
 *   [TIFF header 8 bytes]
 *   [IFD0: 25 entries]
 *   [EXIF IFD: 6 entries]
 *   [Extra data: strings, rationals, arrays]
 *   [Pad to raw_offset]
 *
 * Returns total header size (excluding padding).
 */
static int build_dng_header(uint8_t* buf, struct raw_info* ri,
                            uint32_t raw_offset, struct mechdng_exif* exif)
{
    int active_y1 = ri->active_area.y1;
    int active_x1 = ri->active_area.x1;
    int active_y2 = ri->active_area.y2;
    int active_x2 = ri->active_area.x2;

    /* --- TIFF header (8 bytes) --- */
    put16(buf+0, 0x4949);      /* little endian */
    put16(buf+2, 42);          /* TIFF magic */
    put32(buf+4, 8);           /* offset to IFD0 */

    /* --- IFD0 at offset 8 --- */
    int n_ifd0 = 25;
    uint8_t* ifd = buf + 8;
    put16(ifd, n_ifd0);
    uint8_t* e = ifd + 2;

    /* EXIF IFD follows immediately after IFD0 */
    int n_exif = 6;
    uint32_t exif_ifd_off = 8 + 2 + n_ifd0 * 12 + 4;

    /* Extra data area starts after EXIF IFD */
    uint32_t extra = exif_ifd_off + 2 + n_exif * 12 + 4;

    /* --- IFD0 entries (MUST be sorted by tag!) --- */

    /* 0x00FE: NewSubFileType = 0 */
    ifd_entry(e, 0x00FE, T_LONG, 1, 0); e += 12;

    /* 0x0100: ImageWidth */
    ifd_entry(e, 0x0100, T_LONG, 1, ri->width); e += 12;

    /* 0x0101: ImageLength */
    ifd_entry(e, 0x0101, T_LONG, 1, ri->height); e += 12;

    /* 0x0102: BitsPerSample */
    ifd_entry(e, 0x0102, T_SHORT, 1, ri->bits_per_pixel); e += 12;

    /* 0x0103: Compression = 1 (uncompressed) */
    ifd_entry(e, 0x0103, T_SHORT, 1, 1); e += 12;

    /* 0x0106: PhotometricInterpretation = CFA */
    ifd_entry(e, 0x0106, T_SHORT, 1, 32803); e += 12;

    /* 0x010F: Make → "Canon" */
    {
        int len = sizeof(str_make);  /* includes null */
        ifd_entry(e, 0x010F, T_ASCII, len, extra);
        memcpy(buf + extra, str_make, len);
        extra += (len + 1) & ~1;
        e += 12;
    }

    /* 0x0110: Model → "Canon EOS 5D Mark III" */
    uint32_t model_off = extra;
    {
        int len = sizeof(str_model);
        ifd_entry(e, 0x0110, T_ASCII, len, extra);
        memcpy(buf + extra, str_model, len);
        extra += (len + 1) & ~1;
        e += 12;
    }

    /* 0x0111: StripOffsets */
    ifd_entry(e, 0x0111, T_LONG, 1, raw_offset); e += 12;

    /* 0x0115: SamplesPerPixel = 1 */
    ifd_entry(e, 0x0115, T_SHORT, 1, 1); e += 12;

    /* 0x0116: RowsPerStrip = height */
    ifd_entry(e, 0x0116, T_LONG, 1, ri->height); e += 12;

    /* 0x0117: StripByteCounts = frame_size */
    ifd_entry(e, 0x0117, T_LONG, 1, ri->frame_size); e += 12;

    /* 0x0131: Software */
    {
        int len = sizeof(str_software);
        ifd_entry(e, 0x0131, T_ASCII, len, extra);
        memcpy(buf + extra, str_software, len);
        extra += (len + 1) & ~1;
        e += 12;
    }

    /* 0x828D: CFARepeatPatternDim = 2,2 */
    ifd_entry(e, 0x828D, T_SHORT, 2, 0x00020002); e += 12;

    /* 0x828E: CFAPattern = RGGB */
    ifd_entry(e, 0x828E, T_BYTE, 4, ri->cfa_pattern); e += 12;

    /* 0x8769: ExifIFD pointer */
    ifd_entry(e, 0x8769, T_LONG, 1, exif_ifd_off); e += 12;

    /* 0xC612: DNGVersion = 1.3.0.0 */
    ifd_entry(e, 0xC612, T_BYTE, 4, 0x00000301); e += 12;

    /* 0xC613: DNGBackwardVersion = 1.1.0.0 */
    ifd_entry(e, 0xC613, T_BYTE, 4, 0x00000101); e += 12;

    /* 0xC614: UniqueCameraModel → reuse model string */
    ifd_entry(e, 0xC614, T_BYTE, sizeof(str_model), model_off); e += 12;

    /* 0xC61A: BlackLevel */
    ifd_entry(e, 0xC61A, T_LONG, 1, ri->black_level); e += 12;

    /* 0xC61D: WhiteLevel */
    ifd_entry(e, 0xC61D, T_LONG, 1, ri->white_level); e += 12;

    /* 0xC621: ColorMatrix1 — 9 SRationals */
    ifd_entry(e, 0xC621, T_SRATIONAL, 9, extra);
    memcpy(buf + extra, color_matrix1, 9 * 8);
    extra += 9 * 8;
    e += 12;

    /* 0xC628: AsShotNeutral — 3 RATIONALs {R, G, B} */
    ifd_entry(e, 0xC628, T_RATIONAL, 3, extra);
    put32(buf + extra + 0,  exif->wb_r_num);
    put32(buf + extra + 4,  exif->wb_r_den);
    put32(buf + extra + 8,  exif->wb_g_num);
    put32(buf + extra + 12, exif->wb_g_den);
    put32(buf + extra + 16, exif->wb_b_num);
    put32(buf + extra + 20, exif->wb_b_den);
    extra += 24;
    e += 12;

    /* 0xC65A: CalibrationIlluminant1 = 21 (D65) */
    ifd_entry(e, 0xC65A, T_SHORT, 1, 21); e += 12;

    /* 0xC68D: ActiveArea (4 LONGs: top, left, bottom, right) */
    ifd_entry(e, 0xC68D, T_LONG, 4, extra);
    put32(buf + extra + 0,  active_y1);
    put32(buf + extra + 4,  active_x1);
    put32(buf + extra + 8,  active_y2);
    put32(buf + extra + 12, active_x2);
    extra += 16;
    e += 12;

    /* Next IFD offset = 0 (end of IFD chain) */
    put32(e, 0);

    /* --- EXIF IFD at exif_ifd_off --- */
    uint8_t* exif_ifd = buf + exif_ifd_off;
    put16(exif_ifd, n_exif);
    uint8_t* ex = exif_ifd + 2;

    /* 0x829A: ExposureTime (RATIONAL) */
    ifd_entry(ex, 0x829A, T_RATIONAL, 1, extra);
    put32(buf + extra + 0, exif->shutter_num);
    put32(buf + extra + 4, exif->shutter_den);
    extra += 8;
    ex += 12;

    /* 0x829D: FNumber (RATIONAL) */
    ifd_entry(ex, 0x829D, T_RATIONAL, 1, extra);
    put32(buf + extra + 0, exif->aperture_num);
    put32(buf + extra + 4, exif->aperture_den);
    extra += 8;
    ex += 12;

    /* 0x8827: ISOSpeedRatings (SHORT, inline) */
    ifd_entry(ex, 0x8827, T_SHORT, 1, exif->iso); ex += 12;

    /* 0x9003: DateTimeOriginal (ASCII, 20 bytes) */
    {
        int len = 20;  /* "YYYY:MM:DD HH:MM:SS\0" = 20 bytes */
        ifd_entry(ex, 0x9003, T_ASCII, len, extra);
        memcpy(buf + extra, exif->datetime, len);
        extra += (len + 1) & ~1;
        ex += 12;
    }

    /* 0x920A: FocalLength (RATIONAL) */
    ifd_entry(ex, 0x920A, T_RATIONAL, 1, extra);
    put32(buf + extra + 0, exif->focal_len);
    put32(buf + extra + 4, 1);
    extra += 8;
    ex += 12;

    /* 0xA434: LensModel (ASCII) */
    {
        int len = strlen(exif->lens_name) + 1;
        if (len < 2) {
            /* no lens name — write generic */
            const char* generic = "Unknown";
            len = 8;
            ifd_entry(ex, 0xA434, T_ASCII, len, extra);
            memcpy(buf + extra, generic, len);
        } else {
            ifd_entry(ex, 0xA434, T_ASCII, len, extra);
            memcpy(buf + extra, exif->lens_name, len);
        }
        extra += (len + 1) & ~1;
        ex += 12;
    }

    /* EXIF IFD: next IFD = 0 */
    put32(ex, 0);

    return extra;  /* total header size */
}

/* =====================================================================
 * Chunked raw writer with optimized byte-swap
 * ===================================================================*/

#define CHUNK_SIZE (256 * 1024)

/**
 * Write raw pixel data with 16-bit byte-swap in chunks.
 * Optimized: processes 4 bytes per iteration using 32-bit word ops.
 */
static int write_raw_chunked(FILE* f, void* src, int size)
{
    uint8_t* chunk = malloc(CHUNK_SIZE);
    if (!chunk) return 0;

    uint8_t* p = (uint8_t*)UNCACHEABLE(src);
    int remaining = size;

    while (remaining > 0)
    {
        int n = remaining < CHUNK_SIZE ? remaining : CHUNK_SIZE;
        memcpy(chunk, p, n);

        /* 16-bit byte-swap: AB CD → BA DC
         * Process 4 bytes at a time using 32-bit word operations */
        int n_aligned = n & ~3;
        uint32_t* w = (uint32_t*)chunk;
        for (int i = 0; i < n_aligned; i += 4)
        {
            uint32_t v = *w;
            *w = ((v & 0xFF00FF00) >> 8) | ((v & 0x00FF00FF) << 8);
            w++;
        }
        /* Handle last 0-2 bytes if size not 4-aligned */
        for (int i = n_aligned; i < n - 1; i += 2)
        {
            uint8_t tmp = chunk[i];
            chunk[i] = chunk[i+1];
            chunk[i+1] = tmp;
        }

        if (FIO_WriteFile(f, chunk, n) != n)
        {
            free(chunk);
            return 0;
        }

        p += n;
        remaining -= n;
    }

    free(chunk);
    return 1;
}

/* =====================================================================
 * DNG save — header + raw data
 * ===================================================================*/

/**
 * Save a complete DNG file with EXIF metadata.
 * Returns 1 on success, 0 on failure.
 */
static int mechdng_save(const char* filename, struct raw_info* ri,
                        struct mechdng_exif* exif)
{
    uint8_t hdr[2048];
    memset(hdr, 0, sizeof(hdr));

    /* First pass: calculate header size with raw_offset=0 */
    uint32_t raw_offset = (build_dng_header(hdr, ri, 0, exif) + 3) & ~3;

    /* Second pass: rebuild with correct raw_offset */
    memset(hdr, 0, sizeof(hdr));
    build_dng_header(hdr, ri, raw_offset, exif);

    FILE* f = FIO_CreateFile(filename);
    if (!f) return 0;

    /* Write header (padded with zeros to raw_offset) */
    if (FIO_WriteFile(f, hdr, raw_offset) != (int)raw_offset)
    {
        FIO_CloseFile(f);
        FIO_RemoveFile(filename);
        return 0;
    }

    /* Write raw data with chunked byte-swap */
    if (!write_raw_chunked(f, ri->buffer, ri->frame_size))
    {
        FIO_CloseFile(f);
        FIO_RemoveFile(filename);
        return 0;
    }

    FIO_CloseFile(f);
    return 1;
}

/* =====================================================================
 * camera.wait — poll until Canon finishes processing
 * ===================================================================*/

/**
 * Wait until camera is idle (not writing/processing).
 * Equivalent to camera.wait() from Lua API.
 * timeout_ms: max wait in milliseconds (0 = no wait)
 */
static void mechdng_wait_idle(int timeout_ms)
{
    for (int i = 0; i < timeout_ms / 20; i++)
    {
        if (lens_info.job_state == 0)
            return;
        msleep(20);
    }
}

/* =====================================================================
 * Module logic
 * ===================================================================*/

static unsigned int mechdng_shoot_task_cbr(unsigned int ctx)
{
    if (!mechdng_enabled)
        return CBR_RET_CONTINUE;

    if (is_saving)
        return CBR_RET_CONTINUE;

    if (!QR_MODE)
        return CBR_RET_CONTINUE;

    int cur_file_number = get_shooting_card()->file_number;
    if (cur_file_number == last_file_number)
        return CBR_RET_CONTINUE;

    if (!can_use_raw_overlays_photo())
        return CBR_RET_CONTINUE;

    if (!raw_update_params())
        return CBR_RET_CONTINUE;

    if (!raw_info.buffer)
        return CBR_RET_CONTINUE;

    is_saving = 1;
    last_file_number = cur_file_number;

    /* Snapshot raw_info — buffer may be reused on next shot */
    struct raw_info local_raw_info = raw_info;

    /* Capture EXIF metadata while lens_info is still valid */
    struct mechdng_exif exif;
    mechdng_capture_exif(&exif);

    /* Wait for Canon to finish writing CR2/JPG (up to 4 seconds).
     * This reduces I/O contention when both write to the same card. */
    mechdng_wait_idle(4000);

    /* Generate filename */
    char filename[100];
    snprintf(filename, sizeof(filename), "%s/MD_%04d.DNG",
             get_dcim_dir(), cur_file_number % 10000);

    NotifyBox(2000, "MechDNG: saving...");

    int ok = mechdng_save(filename, &local_raw_info, &exif);

    if (ok)
        NotifyBox(2000, "MechDNG: MD_%04d.DNG", cur_file_number % 10000);
    else
        NotifyBox(3000, "MechDNG: SAVE FAILED!");

    is_saving = 0;
    return CBR_RET_CONTINUE;
}

static struct menu_entry mechdng_menu[] = {
    {
        .name    = "Mech DNG Save",
        .priv    = &mechdng_enabled,
        .max     = 1,
        .help    = "Save 14-bit DNG with EXIF after each photo.",
        .help2   = "Set Canon quality to RAW+JPEG for JPEG+DNG workflow.",
    },
};

static unsigned int mechdng_init()
{
    menu_add("Shoot", mechdng_menu, COUNT(mechdng_menu));
    return 0;
}

static unsigned int mechdng_deinit()
{
    menu_remove("Shoot", mechdng_menu, COUNT(mechdng_menu));
    return 0;
}

MODULE_INFO_START()
    MODULE_INIT(mechdng_init)
    MODULE_DEINIT(mechdng_deinit)
MODULE_INFO_END()

MODULE_CBRS_START()
    MODULE_CBR(CBR_SHOOT_TASK, mechdng_shoot_task_cbr, 0)
MODULE_CBRS_END()

MODULE_CONFIGS_START()
    MODULE_CONFIG(mechdng_enabled)
MODULE_CONFIGS_END()
