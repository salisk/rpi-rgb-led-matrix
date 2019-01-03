#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <sys/mman.h>
#include <sys/utsname.h>

/*
 * nanosleep() takes longer than requested because of OS jitter.
 * In about 99.9% of the cases, this is <= 25 microcseconds on
 * the Raspberry Pi (empirically determined with a Raspbian kernel), so
 * we substract this value whenever we do nanosleep(); the remaining time
 * we then busy wait to get a good accurate result.
 *
 * You can measure the overhead using DEBUG_SLEEP_JITTER below.
 *
 * Note: A higher value here will result in more CPU use because of more busy
 * waiting inching towards the real value (for all the cases that nanosleep()
 * actually was better than this overhead).
 *
 * This might be interesting to tweak in particular if you have a realtime
 * kernel with different characteristics.
 */
#define EMPIRICAL_NANOSLEEP_OVERHEAD_US 25

/*
 * In few cases on a standard kernel, we see that the overhead is actually
 * even longer; these additional 35usec cover up for the 99.999%-ile.
 * So ideally, we always use these additional time and also busy-wait them,
 * right ?
 * However, that would take away a lot of CPU on older, one-core Raspberry Pis
 * or Pi Zeros. They rely for us to sleep when possible for it to do work.
 * So we only enable it, if we have have a newer Pi where we anyway burn
 * away on one core (And are isolated there with isolcpus=3).
 */
#define EMPIRICAL_NANOSLEEP_EXTRA_OVERHEAD_US 35

/*----------------------------------------------------------------------------*/
#define ODROIDC2_GPIO_MASK		(0xFFFFFF00)
#define ODROIDC2_GPIO_BASE		0xC8834000

// #define GPIO_PIN_BASE			136

// #define C2_GPIODV_PIN_START		(GPIO_PIN_BASE + 45)
// #define C2_GPIODV_PIN_END		(GPIO_PIN_BASE + 74)
// #define C2_GPIOY_PIN_START		(GPIO_PIN_BASE + 75)
// #define C2_GPIOY_PIN_END		(GPIO_PIN_BASE + 91)
// #define C2_GPIOX_PIN_START		(GPIO_PIN_BASE + 92)
// #define C2_GPIOX_PIN_END		(GPIO_PIN_BASE + 114)

#define C2_GPIOX_FSEL_REG_OFFSET	0x118
#define C2_GPIOX_OUTP_REG_OFFSET	0x119
#define C2_GPIOX_INP_REG_OFFSET		0x11A
// #define C2_GPIOX_PUPD_REG_OFFSET	0x13E
// #define C2_GPIOX_PUEN_REG_OFFSET	0x14C

#define C2_GPIOY_FSEL_REG_OFFSET	0x10F
#define C2_GPIOY_OUTP_REG_OFFSET	0x110
#define C2_GPIOY_INP_REG_OFFSET		0x111
// #define C2_GPIOY_PUPD_REG_OFFSET	0x13B
// #define C2_GPIOY_PUEN_REG_OFFSET	0x149

#define TIMER_A_OFFSET 0xC1109988

/*----------------------------------------------------------------------------*/
#define	PAGE_SIZE	(4*1024)
#define	BLOCK_SIZE	(4*1024)

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static volatile uint32_t *s_GPIO_registers = NULL;

namespace rgb_matrix {
/*static*/ const uint32_t GPIO::kValidBits
= ((1 <<  0) | (1 <<  1) | (1 <<  2) | (1 <<  3) | (1 <<  4) |
   (1 <<  5) | (1 <<  6) | (1 <<  7) | (1 <<  8) | (1 <<  9) |
   (1 << 10) | (1 << 11) | (1 << 19) | (1 << 21) | (1 << 24) |
   (1 << 28) | (1 << 29) | (1 << 30) | (1 << 31)
);

GPIO::GPIO() : output_bits_(0), slowdown_(1), gpio_port_(NULL) {
}

uint32_t GPIO::InitOutputs(uint32_t outputs,
                           bool adafruit_pwm_transition_hack_needed) {
  if (gpio_port_ == NULL) {
    fprintf(stderr, "Attempt to init outputs but not yet Init()-ialized.\n");
    return 0;
  }

  outputs &= kValidBits;   // Sanitize input.
  output_bits_ = outputs;

  // set GPIOX register gpio to output 
  for (uint32_t b = 0; b <= 21; ++b) {
    if (outputs & (1 << b)) {
		*(gpio_port_ + C2_GPIOX_FSEL_REG_OFFSET) = (*(gpio_port_ + C2_GPIOX_FSEL_REG_OFFSET) & ~(1 << b));
    }
  }

  // set GPIOY register gpio to output
  for (uint32_t b = 0, b < 9; ++b) {
	  if (outputs & (1 << (b + 21))) {
		*(gpio_port_ + C2_GPIOY_FSEL_REG_OFFSET) = (*(gpio_port_ + C2_GPIOY_FSEL_REG_OFFSET) & ~(1 << b));
	  }
  }

  // set two last special cases
  if (outputs & (1 << 30))
	*(gpio_port_ + C2_GPIOY_FSEL_REG_OFFSET) = (*(gpio_port_ + C2_GPIOY_FSEL_REG_OFFSET) & ~(1 << 13));

  if (outputs & (1 << 31))
	*(gpio_port_ + C2_GPIOY_FSEL_REG_OFFSET) = (*(gpio_port_ + C2_GPIOY_FSEL_REG_OFFSET) & ~(1 << 14));
	
  return output_bits_;
}

// /*----------------------------------------------------------------------------*/
// //
// // offset to the GPIO Set regsiter
// //
// /*----------------------------------------------------------------------------*/
// static int gpioToGPSETReg (int pin)
// {
// 	if (pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
// 		return  C2_GPIOX_OUTP_REG_OFFSET;
// 	if (pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
// 		return  C2_GPIOY_OUTP_REG_OFFSET;
// 	if (pin >= C2_GPIODV_PIN_START && pin <= C2_GPIODV_PIN_END)
// 		return  C2_GPIODV_OUTP_REG_OFFSET;
// 	return	-1;
// }

// /*----------------------------------------------------------------------------*/
// //
// // offset to the GPIO bit
// //
// /*----------------------------------------------------------------------------*/
// static int gpioToShiftReg (int pin)
// {
// 	if (pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
// 		return  pin - C2_GPIOX_PIN_START;
// 	if (pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
// 		return  pin - C2_GPIOY_PIN_START;
// 	if (pin >= C2_GPIODV_PIN_START && pin <= C2_GPIODV_PIN_END)
// 		return  pin - C2_GPIODV_PIN_START;
// 	return	-1;
// }

/*----------------------------------------------------------------------------*/
static uint32_t *init_gpio_mmap (off_t register_offset)
{
	int	fd;

	/* GPIO mmap setup */
	if (access("/dev/gpiomem",0) == 0) {
		if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0) {
			perror("setup: Unable to open /dev/gpiomem: %s\n");
      return NULL;
    }
	} else {
		if (geteuid () != 0) {
			perror("setup: Must be root. (Did you forget sudo?)\n");
      return NULL;
    }
		if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0) {
			perror("setup: Unable to open /dev/mem: %s\n");
			return NULL;
    }
	}

	uint32_t *result = 
    (uint32_t*) mmap(0,
                     BLOCK_SIZE, 
                     PROT_READ|PROT_WRITE,
                     MAP_SHARED,
                     fd,
                     register_offset
                     );

	if ((int32_t)gpio == -1) {
		perror("mmap error: ");
    return NULL;
  }

  return result;
}

static bool mmap_all_bcm_registers_once() {
  if (s_GPIO_registers != NULL) return true;  // alrady done.

  // The common GPIO registers.
  s_GPIO_registers = init_gpio_mmap(ODROIDC2_GPIO_BASE);
  if (s_GPIO_registers == NULL) {
    return false;
  }

  // Time measurement.
  uint32_t *timereg = init_gpio_mmap(TIMER_A_OFFSET);
  if (timereg == NULL) {
    return false;
  }
  s_Timer1Mhz = timereg;

//   // Hardware pin-pulser.
//   s_PWM_registers  = mmap_bcm_register(isPI2, GPIO_PWM_BASE_OFFSET);
//   s_CLK_registers  = mmap_bcm_register(isPI2, GPIO_CLK_BASE_OFFSET);
//   if (!s_PWM_registers || !s_CLK_registers) {
//     return false;
//   }

  return true;
}

bool GPIO::Init(int slowdown) {
  slowdown_ = slowdown;

  if (!mmap_all_bcm_registers_once())
    return false;

  gpiox_bits_ = s_GPIO_registers + C2_GPIOX_OUTP_REG_OFFSET;
  gpioy_bits_ = s_GPIO_registers + C2_GPIOY_OUTP_REG_OFFSET;

  gpiox_read_bits_ = s_GPIO_registers + C2_GPIOX_INP_REG_OFFSET;
  gpioy_read_bits_ = s_GPIO_registers + C2_GPIOY_INP_REG_OFFSET;

  return true;
}

// --- PinPulser. Private implementation parts.
namespace {
// Manual timers.
class Timers {
public:
  static bool Init();
  static void sleep_nanos(long t);
};

// Simplest of PinPulsers. Uses somewhat jittery and manual timers
// to get the timing, but not optimal.
class TimerBasedPinPulser : public PinPulser {
public:
  TimerBasedPinPulser(GPIO *io, uint32_t bits,
                      const std::vector<int> &nano_specs)
    : io_(io), bits_(bits), nano_specs_(nano_specs) {}

  virtual void SendPulse(int time_spec_number) {
    io_->ClearBits(bits_);
    Timers::sleep_nanos(nano_specs_[time_spec_number]);
    io_->SetBits(bits_);
  }

private:
  GPIO *const io_;
  const uint32_t bits_;
  const std::vector<int> nano_specs_;
};

static void DisableRealtimeThrottling() {
  const int out = open("/proc/sys/kernel/sched_rt_runtime_us", O_WRONLY);
  if (out < 0) return;
  write(out, "-1", 2);
  close(out);
}

static uint32_t JitterAllowanceMicroseconds() {
  // We can allow to burn a bit more busy-wait
  // CPU cycles to get the timing accurate as we have more CPU to spare.
  static int allowance_us = EMPIRICAL_NANOSLEEP_OVERHEAD_US
    + EMPIRICAL_NANOSLEEP_EXTRA_OVERHEAD_US;
  return allowance_us;
}

bool Timers::Init() {
  if (!mmap_all_bcm_registers_once())
    return false;

  busy_sleep_impl = sleep_nanos;
  DisableRealtimeThrottling();
  
  return true;
}

void Timers::sleep_nanos(long nanos) {
  // For smaller durations, we go straight to busy wait.

  // For larger duration, we use nanosleep() to give the operating system
  // a chance to do something else.
  // However, these timings have a lot of jitter, so we do a two way
  // approach: we use nanosleep(), but for some shorter time period so
  // that we can tolerate some jitter (also, we need at least an offset of
  // EMPIRICAL_NANOSLEEP_OVERHEAD_US as the nanosleep implementations on RPi
  // actually have such offset).
  //
  // We use the global 1Mhz hardware timer to measure the actual time period
  // that has passed, and then inch forward for the remaining time with
  // busy wait.
  static long kJitterAllowanceNanos = JitterAllowanceMicroseconds() * 1000;
  if (nanos > kJitterAllowanceNanos + 5000) {
    const uint32_t before = *s_Timer1Mhz;
    struct timespec sleep_time
      = { 0, nanos - kJitterAllowanceNanos };
    nanosleep(&sleep_time, NULL);
    const uint32_t after = *s_Timer1Mhz;
    const long nanoseconds_passed = 1000 * (uint32_t)(after - before);
    if (nanoseconds_passed > nanos) {
      return;  // darn, missed it.
    } else {
      nanos -= nanoseconds_passed; // remaining time with busy-loop
    }
  }

  busy_sleep_impl(nanos);
}

static void sleep_nanos(long nanos) {
  if (nanos < 20) return;
  // The following loop is determined empirically on a 900Mhz RPi 2
  for (uint32_t i = (nanos - 20) * 100 / 110; i != 0; --i) {
    asm("");
  }
}

// Public PinPulser factory
PinPulser *PinPulser::Create(GPIO *io, uint32_t gpio_mask,
                             bool allow_hardware_pulsing,
                             const std::vector<int> &nano_wait_spec) {
  if (!Timers::Init()) return NULL;
  if (allow_hardware_pulsing && HardwarePinPulser::CanHandle(gpio_mask)) {
    return new HardwarePinPulser(gpio_mask, nano_wait_spec);
  } else {
    return new TimerBasedPinPulser(io, gpio_mask, nano_wait_spec);
  }
}

uint32_t GetMicrosecondCounter() {
  return s_Timer1Mhz ? *s_Timer1Mhz : 0;
}

} // namespace rgb_matrix