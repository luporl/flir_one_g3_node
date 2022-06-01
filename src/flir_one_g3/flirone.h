#ifndef SRC_FLIRONE_H
#define SRC_FLIRONE_H

#include <stddef.h>

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

extern unsigned char *f1_jpg_ptr;
extern size_t f1_jpg_sz;
extern unsigned char *f1_ir_ptr;
extern size_t f1_ir_sz;

int f1_init(struct f1_cfg *cfg);
void f1_exit(void);
int f1_loop(void);

#ifdef __cplusplus
}
#endif

#endif
