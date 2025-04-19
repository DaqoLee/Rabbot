/**
 * @file shell_cpp.h
 * @author Letter (nevermindzzt@gmail.com)
 * @brief shell cpp support
 * @version 1.0.0
 * @date 2021-01-09
 * 
 * @copyright (c) 2021 Letter
 * 
 */
#ifndef __SHELL_CPP_H__
#define __SHELL_CPP_H__

#ifdef __cplusplus
extern "C" {

#include "shell.h"

/**
 * @brief shell command cpp ֧�� cmd ����
 */
typedef struct shell_command_cpp_cmd
{
    int attr;                                                   /**< ���� */
    const char *name;                                           /**< ������ */
    int (*function)();                                          /**< ����ִ�к��� */
    const char *desc;                                           /**< �������� */
#if SHELL_USING_FUNC_SIGNATURE == 1
    const char *signature;                                      /**< ����ǩ�� */
#endif
} ShellCommandCppCmd;

/**
 * @brief shell command cpp ֧�� var ����
 */
typedef struct shell_command_cpp_var
{
    int attr;                                                   /**< ���� */
    const char *name;                                           /**< ������ */
    void *value;                                                /**< ����ֵ */
    const char *desc;                                           /**< �������� */
#if SHELL_USING_FUNC_SIGNATURE == 1
    void *unused;                                               /**< δʹ�ó�Ա����Ҫ���ֺ� ShellCommandCppCmd ��Сһ�� */
#endif
} ShellCommandCppVar;

/**
 * @brief shell command cpp ֧�� user ����
 */
typedef struct shell_command_cpp_user
{
    int attr;                                                   /**< ���� */
    const char *name;                                           /**< �û��� */
    const char *password;                                       /**< �û����� */
    const char *desc;                                           /**< �û����� */
#if SHELL_USING_FUNC_SIGNATURE == 1
    void *unused;                                               /**< δʹ�ó�Ա����Ҫ���ֺ� ShellCommandCppCmd ��Сһ�� */
#endif
} ShellCommandCppUser;

/**
 * @brief shell command cpp ֧�� key ����
 */
typedef struct shell_command_cpp_key
{
    int attr;                                                   /**< ���� */
    int value;                                                  /**< ������ֵ */
    void (*function)(Shell *);                                  /**< ����ִ�к��� */
    const char *desc;                                           /**< �������� */
#if SHELL_USING_FUNC_SIGNATURE == 1
    void *unused;                                               /**< δʹ�ó�Ա����Ҫ���ֺ� ShellCommandCppCmd ��Сһ�� */
#endif
} ShellCommandCppKey;

#if SHELL_USING_FUNC_SIGNATURE == 1
typedef struct shell_command_cpp_param_parser
{
    int attr;                                                   /**< ���� */
    const char *type;                                           /**< �������� */
    int (*parser)(char *, void **);;                            /**< �������� */
    int (*cleaner)(void *);                                     /**< ������ */
    void *unsed;                                                /**< δʹ�ó�Ա����Ҫ���ֺ� ShellCommandCppCmd ��Сһ�� */
} ShellCommandCppParamParser;
#endif

#if SHELL_USING_CMD_EXPORT == 1

    #undef SHELL_EXPORT_CMD
    /**
     * @brief shell �����
     * 
     * @param _attr ��������
     * @param _name ������
     * @param _func �����
     * @param _desc ��������
     * @param ... ��������
     */
    #define SHELL_EXPORT_CMD(_attr, _name, _func, _desc, ...) \
            const char shellCmd##_name[] = #_name; \
            const char shellDesc##_name[] = #_desc; \
            extern "C" SHELL_USED const ShellCommandCppCmd \
            shellCommand##_name SHELL_SECTION("shellCommand") =  \
            { \
                _attr, \
                shellCmd##_name, \
                (int (*)())_func, \
                shellDesc##_name, \
                ##__VA_ARGS__ \
            }

    #undef SHELL_EXPORT_VAR
    /**
     * @brief shell ��������
     * 
     * @param _attr ��������
     * @param _name ������
     * @param _value ����ֵ
     * @param _desc ��������
     */
    #define SHELL_EXPORT_VAR(_attr, _name, _value, _desc) \
            const char shellCmd##_name[] = #_name; \
            const char shellDesc##_name[] = #_desc; \
            extern "C" SHELL_USED const ShellCommandCppVar \
            shellVar##_name SHELL_SECTION("shellCommand") =  \
            { \
                _attr, \
                shellCmd##_name, \
                (void *)_value, \
                shellDesc##_name \
            }

    #undef SHELL_EXPORT_USER
    /**
     * @brief shell �û�����
     * 
     * @param _attr �û�����
     * @param _name �û���
     * @param _password �û�����
     * @param _desc �û�����
     */
    #define SHELL_EXPORT_USER(_attr, _name, _password, _desc) \
            const char shellCmd##_name[] = #_name; \
            const char shellPassword##_name[] = #_password; \
            const char shellDesc##_name[] = #_desc; \
            extern "C" SHELL_USED const ShellCommandCppUser \
            shellUser##_name SHELL_SECTION("shellCommand") =  \
            { \
                _attr|SHELL_CMD_TYPE(SHELL_TYPE_USER), \
                shellCmd##_name, \
                shellPassword##_name, \
                shellDesc##_name \
            }

    #undef SHELL_EXPORT_KEY
    /**
     * @brief shell ��������
     * 
     * @param _attr ��������
     * @param _value ������ֵ
     * @param _func ��������
     * @param _desc ��������
     */
    #define SHELL_EXPORT_KEY(_attr, _value, _func, _desc) \
            const char shellDesc##_value[] = #_desc; \
            extern "C" SHELL_USED const ShellCommandCppKey \
            shellKey##_value SHELL_SECTION("shellCommand") =  \
            { \
                _attr|SHELL_CMD_TYPE(SHELL_TYPE_KEY), \
                _value, \
                (void (*)(Shell *))_func, \
                shellDesc##_value \
            }

#if SHELL_USING_FUNC_SIGNATURE == 1
    #undef SHELL_EXPORT_PARAM_PARSER
    /**
     * @brief shell ��������������
     * 
     * @param _attr ��������������
     * @param _type ��������������
     * @param _parser ��������������
     * @param _cleaner ����������
     */
    #define SHELL_EXPORT_PARAM_PARSER(_attr, _type, _parser, _cleaner) \
            const char shellDesc##_parser[] = #_type; \
            extern "C" SHELL_USED const ShellCommandCppParamParser \
            shellCommand##_parser SHELL_SECTION("shellCommand") = \
            { \
                _attr|SHELL_CMD_TYPE(SHELL_TYPE_PARAM_PARSER), \
                shellDesc##_parser, \
                (int (*)(char *, void **))_parser, \
                (int (*)(void *))_cleaner \
            }
#endif
#endif /** SHELL_USING_CMD_EXPORT == 1 */

}
#endif /**< defined __cplusplus */

#endif /**< __SHELL_CPP_H__ */