/**
 * mechdng: Mechanical Shutter DNG Interceptor
 *
 * Saves uncompressed 14-bit DNG after mechanical shutter actuation.
 * Uses its own minimal DNG writer with chunked byte-swap (256KB)
 * to avoid modifying the camera's DMA buffer in-place.
 */

#include <module.h>
#include <dryos.h>
#include <property.h>
#include <bmp.h>
#include <menu.h>
#include <config.h>
#include <raw.h>
#include <shoot.h>

static CONFIG_INT("mechdng.enabled", mechdng_enabled, 0);
static int is_saving = 0;
static int last_file_number = -1;

/* =====================================================================
 * Minimal DNG writer — self-contained, no dependency on chdk-dng.c
 * Only needs malloc(256KB) for chunked byte-swap, not 41MB.
 * ===================================================================*/

/* TIFF types */
#define T_SHORT  3
#define T_LONG   4
#define T_RATIONAL 5
#define T_SRATIONAL 10
#define T_BYTE   1

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

/**
 * Build a minimal DNG header for uncompressed 14-bit CFA data.
 * Returns header size, writes header into buf (must be >= 1024 bytes).
 * raw_offset = offset where raw pixel data will start in the file.
 */
static int build_dng_header(uint8_t* buf, struct raw_info* ri, uint32_t raw_offset)
{
    /* We build a single-IFD DNG (no thumbnail, no SubIFDs).
     * IFD entries must be sorted by tag value. */

    int active_y1 = ri->active_area.y1;
    int active_x1 = ri->active_area.x1;
    int active_y2 = ri->active_area.y2;
    int active_x2 = ri->active_area.x2;

    /* --- TIFF header (8 bytes) --- */
    put16(buf+0, 0x4949);  /* little endian */
    put16(buf+2, 42);      /* TIFF magic */
    put32(buf+4, 8);       /* offset to IFD0 */

    /* --- IFD0 starts at offset 8 --- */
    int nentries = 20;
    uint8_t* ifd = buf + 8;
    put16(ifd, nentries);
    uint8_t* e = ifd + 2;  /* first entry */

    /* Extra data area starts after IFD + next-IFD pointer */
    uint32_t extra = 8 + 2 + nentries * 12 + 4;

    /* Tag 0xFE: NewSubFileType = 0 (main image) */
    ifd_entry(e, 0x00FE, T_LONG, 1, 0); e += 12;

    /* Tag 0x100: ImageWidth */
    ifd_entry(e, 0x0100, T_LONG, 1, ri->width); e += 12;

    /* Tag 0x101: ImageLength */
    ifd_entry(e, 0x0101, T_LONG, 1, ri->height); e += 12;

    /* Tag 0x102: BitsPerSample = 14 */
    ifd_entry(e, 0x0102, T_SHORT, 1, ri->bits_per_pixel); e += 12;

    /* Tag 0x103: Compression = 1 (uncompressed) */
    ifd_entry(e, 0x0103, T_SHORT, 1, 1); e += 12;

    /* Tag 0x106: PhotometricInterpretation = 32803 (CFA) */
    ifd_entry(e, 0x0106, T_SHORT, 1, 32803); e += 12;

    /* Tag 0x111: StripOffsets = raw_offset */
    ifd_entry(e, 0x0111, T_LONG, 1, raw_offset); e += 12;

    /* Tag 0x115: SamplesPerPixel = 1 */
    ifd_entry(e, 0x0115, T_SHORT, 1, 1); e += 12;

    /* Tag 0x116: RowsPerStrip = height */
    ifd_entry(e, 0x0116, T_LONG, 1, ri->height); e += 12;

    /* Tag 0x117: StripByteCounts = frame_size */
    ifd_entry(e, 0x0117, T_LONG, 1, ri->frame_size); e += 12;

    /* Tag 0x828D: CFARepeatPatternDim = 2,2 */
    ifd_entry(e, 0x828D, T_SHORT, 2, 0x00020002); e += 12;

    /* Tag 0x828E: CFAPattern = RGGB (4 bytes inline) */
    ifd_entry(e, 0x828E, T_BYTE, 4, ri->cfa_pattern); e += 12;

    /* Tag 0xC612: DNGVersion = 1.3.0.0 */
    ifd_entry(e, 0xC612, T_BYTE, 4, 0x00000301); e += 12;

    /* Tag 0xC612: DNGBackwardVersion = 1.1.0.0 */
    ifd_entry(e, 0xC613, T_BYTE, 4, 0x00000101); e += 12;

    /* Tag 0xC614: UniqueCameraModel (pointer to extra data) */
    {
        const char* model = "Canon EOS 5D Mark III";
        int len = 22; /* including null */
        ifd_entry(e, 0xC614, T_BYTE, len, extra);
        memcpy(buf + extra, model, len);
        extra += (len + 1) & ~1; /* align to even */
        e += 12;
    }

    /* Tag 0xC61A: BlackLevel */
    ifd_entry(e, 0xC61A, T_LONG, 1, ri->black_level); e += 12;

    /* Tag 0xC61D: WhiteLevel */
    ifd_entry(e, 0xC61D, T_LONG, 1, ri->white_level); e += 12;

    /* Tag 0xC621: ColorMatrix1 — 9 SRationals (pointer to extra) */
    ifd_entry(e, 0xC621, T_SRATIONAL, 9, extra);
    memcpy(buf + extra, color_matrix1, 9 * 8);
    extra += 9 * 8;
    e += 12;

    /* Tag 0xC65A: CalibrationIlluminant1 = 21 (D65) */
    ifd_entry(e, 0xC65A, T_SHORT, 1, 21); e += 12;

    /* Tag 0xC68D: ActiveArea (4 LONGs: top, left, bottom, right) */
    ifd_entry(e, 0xC68D, T_LONG, 4, extra);
    put32(buf + extra + 0,  active_y1);
    put32(buf + extra + 4,  active_x1);
    put32(buf + extra + 8,  active_y2);
    put32(buf + extra + 12, active_x2);
    extra += 16;
    e += 12;

    /* Next IFD offset = 0 (no more IFDs) */
    put32(e, 0);

    return extra;  /* total header size */
}

/**
 * Write raw pixel data with 16-bit byte-swap in 256KB chunks.
 * Returns 1 on success, 0 on failure.
 */
#define CHUNK_SIZE (256 * 1024)

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

        /* 16-bit byte-swap (AB CD -> BA DC) */
        for (int i = 0; i < n - 1; i += 2)
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

/**
 * Save a complete DNG file using our minimal writer.
 * Returns 1 on success, 0 on failure.
 */
static int mechdng_save(const char* filename, struct raw_info* ri)
{
    /* Build DNG header — needs ~1KB */
    uint8_t hdr[1024];
    memset(hdr, 0, sizeof(hdr));

    /* Raw data starts right after the header, aligned to 4 bytes */
    uint32_t raw_offset = (build_dng_header(hdr, ri, 0) + 3) & ~3;

    /* Rebuild header with correct raw_offset now that we know it */
    memset(hdr, 0, sizeof(hdr));
    int hdr_size = build_dng_header(hdr, ri, raw_offset);
    (void)hdr_size;

    FILE* f = FIO_CreateFile(filename);
    if (!f) return 0;

    /* Pad header to raw_offset with zeros */
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

    struct raw_info local_raw_info = raw_info;

    char filename[100];
    snprintf(filename, sizeof(filename), "%s/MD_%04d.DNG",
             get_dcim_dir(), cur_file_number % 10000);

    NotifyBox(2000, "MechDNG: saving...");

    int ok = mechdng_save(filename, &local_raw_info);

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
        .help    = "Save uncompressed 14-bit DNG after each photo.",
        .help2   = "Requires Canon quality set to RAW. Saved alongside CR2.",
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
