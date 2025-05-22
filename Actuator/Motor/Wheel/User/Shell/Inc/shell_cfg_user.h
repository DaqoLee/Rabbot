/**
 * @file shell_cfg_user.h
 * @author Letter (nevermindzzt@gmail.com)
 * @brief shell config
 * @version 3.0.0
 * @date 2019-12-31
 * 
 * @copyright (c) 2019 Letter
 * 
 */

#ifndef __SHELL_CFG_USER_H__
#define __SHELL_CFG_USER_H__

#include "stm32g4xx_hal.h"
/**
 * @brief 是否使用默认shell任务while循环
 *        使能此宏，则`shellTask()`函数会一直循环读取输入，一般使用操作系统建立shell
 *        任务时使能此宏，关闭此宏的情况下，一般适用于无操作系统，在主循环中调用`shellTask()`
 */
#define     SHELL_TASK_WHILE     0
/**
 * @brief 是否使用shell伴生对象
 *        一些扩展的组件(文件系统支持，日志工具等)需要使用伴生对象
 */
#define     SHELL_USING_COMPANION       1

/**
 * @brief 是否使用命令导出方式
 *        使能此宏后，可以使用`SHELL_EXPORT_CMD()`等导出命令
 *        定义shell命令，关闭此宏的情况下，需要使用命令表的方式
 */
#define     SHELL_USING_CMD_EXPORT      1

/**
 * @brief 支持shell尾行模式
 */
#define     SHELL_SUPPORT_END_LINE      1


/**
 * @brief 使用LF作为命令行回车触发
 *        可以和SHELL_ENTER_CR同时开启
 */
#define     SHELL_ENTER_LF              1

/**
 * @brief 使用CR作为命令行回车触发
 *        可以和SHELL_ENTER_LF同时开启
 */
#define     SHELL_ENTER_CR              0

/**
 * @brief 使用CRLF作为命令行回车触发
 *        不可以和SHELL_ENTER_LF或SHELL_ENTER_CR同时开启
 */
#define     SHELL_ENTER_CRLF            1

/**
 * @brief shell格式化输入的缓冲大小
 *        为0时不使用shell格式化输入
 * @note shell格式化输入会阻塞shellTask, 仅适用于在有操作系统的情况下使用
 */
#define     SHELL_SCAN_BUFFER          128

/**
 * @brief 获取系统时间(ms)
 *        定义此宏为获取系统Tick，如`HAL_GetTick()`
 * @note 此宏不定义时无法使用双击tab补全命令help，无法使用shell超时锁定
 */
#define     SHELL_GET_TICK()            HAL_GetTick()


#endif