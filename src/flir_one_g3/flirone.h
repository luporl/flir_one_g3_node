#ifndef SRC_FLIRONE_H
#define SRC_FLIRONE_H

#include <stddef.h>
#include <stdint.h>

/* F1 loop return status */
#define F1L_OK               0
#define F1L_NEW_IMG_FRAME    1
#define F1L_NEW_IR_FRAME     2
#define F1L_BUSY             4
#define F1L_INIT1_ERROR     -1
#define F1L_INIT2_ERROR     -2
#define F1L_USB_ERROR       -3
#define F1L_VFRAME_ERROR    -4

struct f1_cfg
{
    char pro;
    char pal_inv;           /* invert palette colors */
    char pal_colors;        /* show palette colors only */
    const char *pal_path;   /* path to palette file */
};

#ifdef __cplusplus
extern "C" {
#endif

struct f1_frame
{
    unsigned char *jpg_ptr;
    size_t jpg_sz;
    unsigned char *ir_ptr;
    size_t ir_sz;
    uint32_t ir_width;
    uint32_t ir_height;

    double temp_min;
    double temp_med;
    double temp_max;
    uint32_t temp_max_x;
    uint32_t temp_max_y;
};

struct f1_frame *f1_init(struct f1_cfg *cfg);
void f1_exit(void);
int f1_loop(void);

void ros_info(const char *fmt, ...);
void ros_error(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
