// global_options.h
#ifndef GLOBAL_OPTIONS_H
#define GLOBAL_OPTIONS_H

#define TMP_CONFIG_FILE "/tmp/hihost_config.tmp"

typedef struct {
    char *port_name;
    int baud_rate;
    char *record_raw_file;   // 原始数据记录文件路径
    char *record_json_file;  // JSON数据记录文件路径
} GlobalOptions;

#endif
