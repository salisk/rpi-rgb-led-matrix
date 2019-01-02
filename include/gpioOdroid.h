// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

#ifndef RPI_GPIO_H
#define RPI_GPIO_H

#include <stdint.h>
#include <vector>

#define REGISTER_X_MASK 0xFFFFFC00
#define REGISTER_Y_MASK 0x3FF
#define CLEAR_MASK 0x3

// Putting this in our namespace to not collide with other things called like
// this.
namespace rgb_matrix {
// For now, everything is initialized as output.
class GPIO {
 public:
  // Available bits that actually have pins.
  static const uint32_t kValidBits;

  GPIO();

  // Initialize before use. Returns 'true' if successful, 'false' otherwise
  // (e.g. due to a permission problem).
  bool Init(int
#if RGB_SLOWDOWN_GPIO
            slowdown = RGB_SLOWDOWN_GPIO
#else
            slowdown = 1
#endif
            );

  // Initialize outputs.
  // Returns the bits that were available and could be set for output.
  // (never use the optional adafruit_hack_needed parameter, it is used
  // internally to this library).
  uint32_t InitOutputs(uint32_t outputs, bool adafruit_hack_needed = false);

  // Request given bitmap of GPIO inputs.
  // Returns the bits that were available and could be reserved.
  uint32_t RequestInputs(uint32_t inputs);

  // Set the bits that are '1' in the output. Leave the rest untouched.
  inline void SetBits(uint32_t value) {
    if (!value) return;

    // process bits for the GPIOY register
    if (value & REGISTER_Y_MASK) {
      unsigned value_y = value & ~(~0U << 10);
      value_y &= CLEAR_MASK;
      // get the special bits
      value_y |= (value >> 30);
      value_y |= (value >> 31);
      *gpioy_bits_ = value_y;
    }

    // process bits for the GPIOX register
    *gpiox_bits_ = value & REGISTER_X_MASK;

    // slowdown
    for (int i = 0; i < slowdown_; ++i) {
      *gpiox_bits_ = value & REGISTER_X_MASK;
    }
  }

  // Clear the bits that are '1' in the output. Leave the rest untouched.
  inline void ClearBits(uint32_t value) {
    if (!value) return;
    SetBits(~value);
  }

  // Write all the bits of "value" mentioned in "mask". Leave the rest untouched.
  inline void WriteMaskedBits(uint32_t value, uint32_t mask) {
    // Writing a word is two operations. The IO is actually pretty slow, so
    // this should probably  be unnoticable.
    ClearBits(~value & mask);
    SetBits(value & mask);
  }

  inline void Write(uint32_t value) { WriteMaskedBits(value, output_bits_); }
  inline uint32_t Read() const { return 0; }

 private:
  uint32_t output_bits_;
  uint32_t input_bits_;
  uint32_t reserved_bits_;
  int slowdown_;
  volatile uint32_t *gpiox_bits_;
  volatile uint32_t *gpioy_bits_;
  volatile uint32_t *gpiox_read_bits_;
  volatile uint32_t *gpioy_read_bits_;
};

// A PinPulser is a utility class that pulses a GPIO pin. There can be various
// implementations.
class PinPulser {
public:
  // Factory for a PinPulser. Chooses the right implementation depending
  // on the context (CPU and which pins are affected).
  // "gpio_mask" is the mask that should be output (since we only
  //   need negative pulses, this is what it does)
  // "nano_wait_spec" contains a list of time periods we'd like
  //   invoke later. This can be used to pre-process timings if needed.
  static PinPulser *Create(GPIO *io, uint32_t gpio_mask,
                           bool allow_hardware_pulsing,
                           const std::vector<int> &nano_wait_spec);

  virtual ~PinPulser() {}

  // Send a pulse with a given length (index into nano_wait_spec array).
  virtual void SendPulse(int time_spec_number) = 0;

  // If SendPulse() is asynchronously implemented, wait for pulse to finish.
  virtual void WaitPulseFinished() {}
};

}  // end namespace rgb_matrix

#endif  // RPI_GPIO_H
