/**
 * @file uart_guard.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief RAII guard for UART resource synchronization
 *
 * Disables interrupts that may call printf() during construction,
 * and re-enables them during destruction. Provides exception-safe
 * serialization of UART access between main-line and interrupt contexts.
 *
 * @version 0.1
 * @date 2026-04-08
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef UART_GUARD_HPP
#define UART_GUARD_HPP

#include "stm32h5xx_hal.h"

class UartGuard {
public:
  /**
   * @brief Construct guard and disable printf-related interrupts
   *
   * Disables:
   * - EXTI13_IRQn (Button ISR - calls printf)
   * - I2C1_EV_IRQn (I2C event - may call printf in callbacks)
   * - I2C1_ER_IRQn (I2C error - may call printf in callbacks)
   * - FDCAN2_IT0_IRQn (FDCAN RX ISR - may call printf in callbacks)
   */
  UartGuard() {
    HAL_NVIC_DisableIRQ(EXTI13_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  }

  /**
   * @brief Destruct guard and re-enable printf-related interrupts
   */
  ~UartGuard() {
    HAL_NVIC_EnableIRQ(EXTI13_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  }

  // Delete copy/move operations - this is a scoped lock
  UartGuard(const UartGuard &) = delete;
  UartGuard &operator=(const UartGuard &) = delete;
  UartGuard(UartGuard &&) = delete;
  UartGuard &operator=(UartGuard &&) = delete;
};

#endif // UART_GUARD_HPP
