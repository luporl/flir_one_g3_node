#ifndef SRC_FLIRONE_H
#define SRC_FLIRONE_H

struct f1_cfg
{
    char pro;
    char pal_inv;           /* invert palette colors */
    char pal_colors;        /* show palette colors only */
    const char *pal_path;   /* path to palette file */
};

int f1_init(struct f1_cfg *cfg);
int f1_loop(void);

#endif
