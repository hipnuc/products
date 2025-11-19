// global_options.h
#ifndef GLOBAL_OPTIONS_H
#define GLOBAL_OPTIONS_H

#define HIHOST_INI_FILE "hihost.ini"

#ifdef HIHOST_SOURCE_DIR
#define HIHOST_INI_ABS_PATH HIHOST_SOURCE_DIR "/" HIHOST_INI_FILE
#else
#define HIHOST_INI_ABS_PATH HIHOST_INI_FILE
#endif

typedef struct {
    char *port_name;
    int baud_rate;
    char *record_raw_file;   // 原始数据记录文件路径
    char *record_json_file;  // JSON数据记录文件路径
} GlobalOptions;

#endif
