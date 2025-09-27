/**
 * @file GpioData.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

namespace rc_ecat_master {

enum class GpioType {
  GPIO_Input = 0,
  GPIO_Output = 1,
};

struct GpioData {
  GpioType type;
  bool value;
};

}  // namespace rc_ecat_master