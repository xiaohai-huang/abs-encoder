/**
 * @file app_entry.h
 * @brief C-compatible entry point between the CubeMX-generated main.c
 *        and the C++ application logic (see app_entry.cpp).
 *
 * This is the only App header main.c may include; everything else in
 * App/ is C++ (classes, references, constexpr) and cannot be consumed
 * from the generated C code.
 */
#ifndef APP_APP_ENTRY_H
#define APP_APP_ENTRY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * One-time setup of the C++ application: HAL backend, gear-geometry
 * validation, I2C slave register map.
 * @return 1 when the gear set is usable, 0 otherwise (the caller should
 *         enter its error handler).
 */
int AppInit(void);

/**
 * Take one sample from each of the four encoders (keeping the last valid
 * reading per role across failed reads) and publish the decoded position
 * to the I2C register map.  Call once per TIM2 sample tick.
 */
void AppProcessSample(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_APP_ENTRY_H */