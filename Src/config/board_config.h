/**
 * @file    board_config.h
 * @brief   板级配置入口 — 纯路由，不含任何硬件定义
 *
 * === 如何切换板子 ===
 *   编译时添加 -DBOARD_XSTAR 即可切换到 X-STAR-S 配置，无需修改任何代码。
 *   CMakeLists.txt 中已提供 -DBOARD_XSTAR 选项（option BOARD_XSTAR）。
 *
 * === 如何新增一个板子 ===
 *   1. 在 Src/config/boards/ 下新建 board_<name>.h
 *   2. 复制 board_vectorfoc.h 为模板，修改所有宏的值（宏名必须保持一致）
 *   3. 在下方添加 #elif defined(BOARD_<NAME>) 分支
 *
 * === 规则 ===
 *   所有源文件只能 #include "board_config.h"，不得直接包含 boards/ 下的文件。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef BOARD_XSTAR
#  include "boards/board_xstar.h"
#else
#  include "boards/board_vectorfoc.h"
#endif

#endif /* BOARD_CONFIG_H */
