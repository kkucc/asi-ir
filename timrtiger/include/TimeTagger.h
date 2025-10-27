/*
This file is part of Time Tagger software defined digital data acquisition.

Copyright (C) 2011-2019 Swabian Instruments
All Rights Reserved

Unauthorized copying of this file is strictly prohibited.
*/

#ifndef TIMETAGGER_H_
#define TIMETAGGER_H_

#ifdef _MSC_VER
#pragma warning(disable : 4251)
#endif

#ifdef LIBTIMETAGGER_EXPORTS
#ifdef _WIN32
#define TT_API __declspec(dllexport)
#else
#define TT_API __attribute__((visibility("default")))
#endif
#else
#if defined(__linux) || defined(SWIG) || defined(NOEXPORT)
#define TT_API
#else
#define TT_API __declspec(dllimport)
#ifdef _DEBUG
#pragma comment(lib, "TimeTaggerD")
#else
#pragma comment(lib, "TimeTagger")
#endif
#endif
#endif

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <stdint.h>
#include <string>
#include <unordered_set>
#include <vector>

/*! \mainpage TimeTagger
 *
 * \brief backend for TimeTagger, an OpalKelly based single photon counting library
 *
 * \author Markus Wick <markus@swabianinstruments.com>
 * \author Helmut Fedder <helmut@swabianinstruments.com>
 * \author Michael Schlagmüller <michael@swabianinstruments.com>
 *
 * TimeTagger provides an easy to use and cost effective hardware solution for
 * time-resolved single photon counting applications.
 *
 * This document describes the C++ native interface to the TimeTagger device.
 */

/**
 * \defgroup TimeTaggers Implementations with a Time Tagger interface
 */

class IteratorBase;
class IteratorBaseListNode;
class TimeTagger;
class TimeTaggerBase;
class TimeTaggerNetwork;
class TimeTaggerRunner;
class TimeTaggerVirtual;

/// The type for all timestamps used in the Time Tagger suite, always in picoseconds
#define timestamp_t long long

/// The type for storing a channel identifier
#define channel_t int

#ifndef SWIG
/// The version of this software suite
#define TIMETAGGER_VERSION "2.17.4"
#endif

/**
 * \brief Get the version of the TimeTagger cxx backend
 */
TT_API std::string getVersion();

/**
 * \brief Constant for unused channel.
 *
 * Magic channel_t value to indicate an unused channel. So the iterators either
 * have to disable this channel, or to choose a default one.
 *
 * This value changed in version 2.1. The old value -1 aliases with falling events.
 * The old value will still be accepted for now if the old numbering scheme is active.
 */
constexpr channel_t CHANNEL_UNUSED = -134217728;
constexpr channel_t CHANNEL_UNUSED_OLD = -1;

/**
 * \brief Allowed values for setTimeTaggerChannelNumberScheme().
 *
 * _ZERO will typically allocate the channel numbers 0 to 7 for the 8 input channels.
 * 8 to 15 will be allocated for the corresponding falling events.
 *
 * _ONE will typically allocate the channel numbers 1 to 8 for the 8 input channels.
 * -1 to -8 will be allocated for the corresponding falling events.
 *
 * _AUTO will choose the scheme based on the hardware revision and so based on the printed label.
 *
 * _DEFAULT will always pick _ONE, but it will yield a warning if _AUTO would have picked _ZERO.
 */
constexpr int TT_CHANNEL_NUMBER_SCHEME_AUTO = 0;
constexpr int TT_CHANNEL_NUMBER_SCHEME_ZERO = 1;
constexpr int TT_CHANNEL_NUMBER_SCHEME_ONE = 2;
constexpr int TT_CHANNEL_NUMBER_SCHEME_DEFAULT = 3;

#ifndef TIMETAGGER_NO_WRAPPER
/**
 * \brief This are the default wrapper functions without any overloadings.
 */
#define GET_DATA_1D(function_name, type, argout, attribute)                                                            \
  attribute void function_name(std::function<type *(size_t)> argout)
#define GET_DATA_1D_OP1(function_name, type, argout, optional_type, optional_name, optional_default, attribute)        \
  attribute void function_name(std::function<type *(size_t)> argout, optional_type optional_name = optional_default)
#define GET_DATA_1D_OP2(function_name, type, argout, optional_type, optional_name, optional_default, optional_type2,   \
                        optional_name2, optional_default2, attribute)                                                  \
  attribute void function_name(std::function<type *(size_t)> argout, optional_type optional_name = optional_default,   \
                               optional_type2 optional_name2 = optional_default2)
#define GET_DATA_2D(function_name, type, argout, attribute)                                                            \
  attribute void function_name(std::function<type *(size_t, size_t)> argout)
#define GET_DATA_2D_OP1(function_name, type, argout, optional_type, optional_name, optional_default, attribute)        \
  attribute void function_name(std::function<type *(size_t, size_t)> argout,                                           \
                               optional_type optional_name = optional_default)
#define GET_DATA_2D_OP2(function_name, type, argout, optional_type, optional_name, optional_default, optional_type2,   \
                        optional_name2, optional_default2, attribute)                                                  \
  attribute void function_name(std::function<type *(size_t, size_t)> argout,                                           \
                               optional_type optional_name = optional_default,                                         \
                               optional_type2 optional_name2 = optional_default2)
#define GET_DATA_3D(function_name, type, argout, attribute)                                                            \
  attribute void function_name(std::function<type *(size_t, size_t, size_t)> argout)
#endif

/**
 * \brief This enum selects the high resolution mode of the Time Tagger series.
 *
 * If any high resolution mode is selected, the hardware will combine 2, 4 or even 8 input channels and average their
 * timestamps. This results in a discretization jitter improvement of factor sqrt(N) for N combined channels. The
 * averaging is implemented before any filter, buffer or USB transmission. So all of those features are available with
 * the averaged timestamps. Because of hardware limitations, only fixed combinations of channels are supported:
 * * HighResA: 1 : [1,2], 3 : [3,4], 5 : [5,6], 7 : [7,8], 10 : [10,11], 12 : [12,13], 14 : [14,15], 16 : [16,17], 9, 18
 * * HighResB: 1 : [1,2,3,4], 5 : [5,6,7,8], 10 : [10,11,12,13], 14 : [14,15,16,17], 9, 18
 * * HighResC: 5 : [1,2,3,4,5,6,7,8], 14 : [10,11,12,13,14,15,16,17], 9, 18
 * The inputs 9 and 18 are always available without averaging.
 * The number of channels available will be limited to the number of channels licensed.
 */
enum class Resolution { Standard = 0, HighResA = 1, HighResB = 2, HighResC = 3 };

/**
 * \brief Enum for filtering the channel list returned by getChannelList.
 */
enum class ChannelEdge : int32_t {
// Bitwise filters, shall not be exported to wrapped languages
#ifndef SWIG
  NoFalling = 1 << 0,
  NoRising = 1 << 1,
  NoStandard = 1 << 2,
  NoHighRes = 1 << 3,
#endif

  All = 0,
  Rising = 1,
  Falling = 2,
  HighResAll = 4,
  HighResRising = 4 | 1,
  HighResFalling = 4 | 2,
  StandardAll = 8,
  StandardRising = 8 | 1,
  StandardFalling = 8 | 2
};
constexpr ChannelEdge TT_CHANNEL_RISING_AND_FALLING_EDGES = ChannelEdge::All;
constexpr ChannelEdge TT_CHANNEL_RISING_EDGES = ChannelEdge::Rising;
constexpr ChannelEdge TT_CHANNEL_FALLING_EDGES = ChannelEdge::Falling;

struct SoftwareClockState {
  // configuration state
  timestamp_t clock_period;
  channel_t input_channel;
  channel_t ideal_clock_channel;
  double averaging_periods;
  bool enabled;

  // runtime information
  bool is_locked;
  uint32_t error_counter;
  timestamp_t last_ideal_clock_event;
  double period_error;           // in picoseconds
  double phase_error_estimation; // in picoseconds, including TDC discretization error
};

/**
 * \brief Enum for selecting the fpga link output interface.
 */
enum class FpgaLinkInterface {
  SFPP_10GE,
  QSFPP_40GE,
};

/**
 * \brief default constructor factory.
 *
 * \param serial serial number of FPGA board to use. if empty, the first board found is used.
 * \param resolution enum for how many channels shall be grouped. \see Resolution for details
 */
TT_API TimeTagger *createTimeTagger(std::string serial = "", Resolution resolution = Resolution::Standard);

/**
 * \brief default constructor factory for the createTimeTaggerVirtual class.
 */
TT_API TimeTaggerVirtual *createTimeTaggerVirtual();

/**
 * \brief default constructor factory for the TimeTaggerNetwork class.
 *
 * \param address IP address of the server. Use hostname:port.
 */
TT_API TimeTaggerNetwork *createTimeTaggerNetwork(std::string address = "localhost:41101");

/**
 * \brief set path and filename of the bitfile to be loaded into the FPGA
 *
 * For debugging/development purposes the firmware loaded into the FPGA can be set manually with this function. To load
 * the default bitfile set bitFileName = ""
 *
 * \param bitFileName custom bitfile to use for the FPGA.
 */
TT_API void setCustomBitFileName(const std::string &bitFileName);

/**
 * \brief free a copy of a TimeTagger reference.
 *
 * \param tagger the TimeTagger reference to free
 */
TT_API bool freeTimeTagger(TimeTaggerBase *tagger);

/**
 * \brief fetches a list of all available TimeTagger serials.
 *
 * This function may return serials blocked by other processes or already disconnected some milliseconds later.
 */
TT_API std::vector<std::string> scanTimeTagger();

/**
 * \brief connect to a Time Tagger server.
 *
 * \param address ip address or domain and port of the server hosting time tagger. Use hostname:port.
 *
 */
TT_API std::string getTimeTaggerServerInfo(std::string address = "localhost:41101");

/**
 * \brief scan the local network for running time tagger servers.
 *
 * \return  a vector of strings of "ip_address:port" for each active server in local network.
 */
TT_API std::vector<std::string> scanTimeTaggerServers();

/*
 * \brief returns the model name of the Time Tagger identified by the serial number.
 *
 * \param serial the Time Tagger serial number to query
 */
TT_API std::string getTimeTaggerModel(const std::string &serial);

/**
 * \brief Configure the numbering scheme for new TimeTagger objects.
 *
 * This function sets the numbering scheme for newly created TimeTagger objects.
 * The default value is _AUTO.
 *
 * Note: TimeTagger objects are cached internally, so the scheme should be set before the first call of
 * createTimeTagger().
 *
 * _ZERO will typically allocate the channel numbers 0 to 7 for the 8 input channels.
 * 8 to 15 will be allocated for the corresponding falling events.
 *
 * _ONE will typically allocate the channel numbers 1 to 8 for the 8 input channels.
 * -1 to -8 will be allocated for the corresponding falling events.
 *
 * _AUTO will choose the scheme based on the hardware revision and so based on the printed label.
 *
 * \param scheme new numbering scheme, must be TT_CHANNEL_NUMBER_SCHEME_AUTO, TT_CHANNEL_NUMBER_SCHEME_ZERO or
 * TT_CHANNEL_NUMBER_SCHEME_ONE
 */
TT_API void setTimeTaggerChannelNumberScheme(int scheme);

/**
 * \brief Fetch the currently configured global numbering scheme.
 *
 * Please see setTimeTaggerChannelNumberScheme() for details.
 * Please use TimeTagger::getChannelNumberScheme() to query the actual used numbering scheme,
 * this function here will just return the scheme a newly created TimeTagger object will use.
 */
TT_API int getTimeTaggerChannelNumberScheme();

/**
 * \brief Check if a license for the TimeTaggerVirtual is available
 */
TT_API bool hasTimeTaggerVirtualLicense();

/**
 * \brief Update the license on the device.
 *
 * Updated license may be fetched by getRemoteLicense.
 * The Time Tagger must not be instantiated while updating the license.
 *
 * \param serial the serial of the device to update the license. Must not be empty
 * \param license the binary license, encoded as a hexadecimal string
 */
TT_API void flashLicense(const std::string &serial, const std::string &license);

/**
 * \brief Converts binary license to JSON.
 *
 * \param license the binary license, encoded as a hexadecimal string
 * \return a JSON string containing the current device license
 */
TT_API std::string extractDeviceLicense(const std::string &license);

// log values are taken from https://docs.python.org/3/library/logging.html
enum LogLevel { LOGGER_ERROR = 40, LOGGER_WARNING = 30, LOGGER_INFO = 10 };
typedef void (*logger_callback)(LogLevel level, std::string msg);

/**
 * \brief Sets the notifier callback which is called for each log message
 *
 * If this function is called with nullptr, the default callback will be used.
 *
 * \returns The old callback
 */
TT_API logger_callback setLogger(logger_callback callback);

/**
 * \brief Raise a new log message. Please use the XXXLog macro instead.
 */
TT_API void LogBase(LogLevel level, const char *file, int line, bool suppressed, const char *fmt, ...)
#ifdef __GNUC__
    __attribute__((format(printf, 5, 6)))
#endif
    ;
#define LogMessage(level, ...) LogBase(level, __FILE__, __LINE__, false, __VA_ARGS__);
#define ErrorLog(...) LogMessage(LOGGER_ERROR, __VA_ARGS__);
#define WarningLog(...) LogMessage(LOGGER_WARNING, __VA_ARGS__);
#define InfoLog(...) LogMessage(LOGGER_INFO, __VA_ARGS__);

// This suppressed methods are used when the log may contain private/confidential data and we
// don't want the usage statistics system to record such data.
#define LogMessageSuppressed(level, ...) LogBase(level, __FILE__, __LINE__, true, __VA_ARGS__);
#define ErrorLogSuppressed(...) LogMessageSuppressed(LOGGER_ERROR, __VA_ARGS__);
#define WarningLogSuppressed(...) LogMessageSuppressed(LOGGER_WARNING, __VA_ARGS__);
#define InfoLogSuppressed(...) LogMessageSuppressed(LOGGER_INFO, __VA_ARGS__);

/// Helper class for setLogger
class TT_API CustomLogger {
public:
  CustomLogger();
  virtual ~CustomLogger();

  void enable();
  void disable();
  virtual void Log(int level, const std::string &msg) = 0;

private:
  static void LogCallback(LogLevel level, std::string msg);
  static CustomLogger *instance;
  static std::mutex instance_mutex;
};

/**
 * \brief Checks the MSVCP and okFrontPanel system library if they match the expected versions
 */
TT_API void checkSystemLibraries();

class ClientNetworkStream;
/**
 * \brief Basis interface for all Time Tagger classes
 *
 * This basis interface represents all common methods to add, remove, and run measurements.
 *
 * \ingroup TimeTaggers
 */
class TT_API TimeTaggerBase {
  friend class IteratorBase;
  friend class TimeTaggerProxy;
  friend class TimeTaggerRunner;
  friend class ClientNetworkStream;

public:
  /**
   * \brief Generate a new fence object, which validates the current configuration and the current time.
   *
   * This fence is uploaded to the earliest pipeline stage of the Time Tagger.
   * Waiting on this fence ensures that all hardware settings such as trigger levels, channel
   * registrations, etc., have propagated to the FPGA and are physically active. Synchronizes
   * the Time Tagger internal memory, so that all tags arriving after the waitForFence call were
   * actually produced after the getFence call. The waitForFence function waits until all tags,
   * which are present at the time of the function call within the internal memory of the Time
   * Tagger, are processed.
   * This call might block to limit the amount of active fences.
   *
   * \param alloc_fence if false, a reference to the most recently created fence will be returned instead
   * \return the allocated fence
   */
  virtual unsigned int getFence(bool alloc_fence = true) = 0;

  /**
   * \brief Wait for a fence in the data stream.
   *
   * See getFence for more details.
   *
   * \param fence fence object, which shall be waited on
   * \param timeout timeout in milliseconds. Negative means no timeout, zero returns immediately.
   * \return true if the fence has passed, false on timeout
   */
  virtual bool waitForFence(unsigned int fence, int64_t timeout = -1) = 0;

  /**
   * \brief Sync the timetagger pipeline, so that all started iterators and their enabled channels are ready
   *
   * This is a shortcut for calling getFence and waitForFence at once. See getFence for more details.
   *
   * \param timeout timeout in milliseconds. Negative means no timeout, zero returns immediately.
   * \return true on success, false on timeout
   */
  virtual bool sync(int64_t timeout = -1) = 0;

  /**
   * \brief get the falling channel id for a rising channel and vice versa
   *
   * If this channel has no inverted channel, UNUSED_CHANNEL is returned.
   * This is the case for most virtual channels.
   *
   * \param channel The channel id to query
   * \return the inverted channel id
   */
  virtual channel_t getInvertedChannel(channel_t channel) = 0;

  /**
   * \brief compares the provided channel with CHANNEL_UNUSED
   *
   * But also keeps care about the channel number scheme and selects
   * either CHANNEL_UNUSED or CHANNEL_UNUSED_OLD
   */
  virtual bool isUnusedChannel(channel_t channel) = 0;

  typedef std::function<void(IteratorBase *)> IteratorCallback;
  typedef std::map<IteratorBase *, IteratorCallback> IteratorCallbackMap;

  /**
   * \brief Run synchronized callbacks for a list of iterators
   *
   * This method has a list of callbacks for a list of iterators.
   * Those callbacks are called for a synchronized data set, but in parallel.
   * They are called from an internal worker thread.
   * As the data set is synchronized, this creates a bottleneck for one
   * worker thread, so only fast and non-blocking callbacks are allowed.
   *
   * \param callbacks Map of callbacks per iterator
   * \param block Shall this method block until all callbacks are finished
   */
  virtual void runSynchronized(const IteratorCallbackMap &callbacks, bool block = true) = 0;

  /**
   * \brief Fetches the overall configuration status of the Time Tagger object.
   *
   * \return a JSON serialized string with all configuration and status flags.
   */
  virtual std::string getConfiguration() = 0;

  /**
   * \brief set time delay on a channel
   *
   * When set, every event on this channel is delayed by the given delay in picoseconds.
   *
   * This method has the best performance with "small delays". The delay is considered
   * "small" when less than 100 events arrive within the time of the largest delay set.
   * For example, if the total event-rate over all channels used is 10 Mevent/s, the
   * signal can be delayed efficiently up to 10 microseconds. For large delays, please
   * use DelayedChannel instead.
   *
   * \param channel  the channel to set
   * \param delay    the delay in picoseconds
   */
  virtual void setInputDelay(channel_t channel, timestamp_t delay) = 0;

  /**
   * \brief set time delay on a channel
   *
   * When set, every event on this physical input channel is delayed by the given delay in picoseconds.
   * This delay is implemented on the hardware before any filter with no performance overhead.
   * The maximum delay on the Time Tagger Ultra series is 2 us.
   * This affects both the rising and the falling event at the same time.
   *
   * \param channel  the channel to set
   * \param delay    the hardware delay in picoseconds
   */
  virtual void setDelayHardware(channel_t channel, timestamp_t delay) = 0;

  /**
   * \brief set time delay on a channel
   *
   * When set, every event on this channel is delayed by the given delay in picoseconds.
   * This happens on the computer and so after the on-device filters.
   * Please use setDelayHardware instead for better performance.
   * This affects either the the rising or the falling event only.
   *
   * This method has the best performance with "small delays". The delay is considered
   * "small" when less than 100 events arrive within the time of the largest delay set.
   * For example, if the total event-rate over all channels used is 10 Mevent/s, the
   * signal can be delayed efficiently up to 10 microseconds. For large delays, please
   * use DelayedChannel instead.
   *
   * \param channel  the channel to set
   * \param delay    the software delay in picoseconds
   */
  virtual void setDelaySoftware(channel_t channel, timestamp_t delay) = 0;

  /**
   * \brief get time delay of a channel
   *
   * see setInputDelay
   *
   * \param channel   the channel
   * \return the software delay in picoseconds
   */
  virtual timestamp_t getInputDelay(channel_t channel) = 0;

  /**
   * \brief get time delay of a channel
   *
   * see setDelaySoftware
   *
   * \param channel   the channel
   * \return the software delay in picoseconds
   */
  virtual timestamp_t getDelaySoftware(channel_t channel) = 0;

  /**
   * \brief get time delay of a channel
   *
   * see setDelayHardware
   *
   * \param channel   the channel
   * \return the hardware delay in picoseconds
   */
  virtual timestamp_t getDelayHardware(channel_t channel) = 0;

  /**
   * \brief set the deadtime between two edges on the same channel.
   *
   * This function sets the user configurable deadtime. The requested time will
   * be rounded to the nearest multiple of the clock time. The deadtime will also
   * be clamped to device specific limitations.
   *
   * As the actual deadtime will be altered, the real value will be returned.
   *
   * \param channel channel to be configured
   * \param deadtime new deadtime in picoseconds
   * \return the real configured deadtime in picoseconds
   */
  virtual timestamp_t setDeadtime(channel_t channel, timestamp_t deadtime) = 0;

  /**
   * \brief get the deadtime between two edges on the same channel.
   *
   * This function gets the user configurable deadtime.
   *
   * \param channel channel to be queried
   * \return the real configured deadtime in picoseconds
   */
  virtual timestamp_t getDeadtime(channel_t channel) = 0;

  /**
   * \brief enable/disable internal test signal on a channel.
   *
   * This will connect or disconnect the channel with the on-chip uncorrelated signal generator.
   *
   * \param channel  the channel
   * \param enabled  enabled / disabled flag
   */
  virtual void setTestSignal(channel_t channel, bool enabled) = 0;

  /**
   * \brief enable/disable internal test signal on multiple channels.
   *
   * This will connect or disconnect the channels with the on-chip uncorrelated signal generator.
   *
   * \param channel  a vector of channels
   * \param enabled  enabled / disabled flag
   */
  virtual void setTestSignal(std::vector<channel_t> channel, bool enabled) = 0;

  /**
   * \brief fetch the status of the test signal generator
   *
   * \param channel   the channel
   */
  virtual bool getTestSignal(channel_t channel) = 0;

  /**
   * \brief enables a software PLL to lock the time to an external clock
   *
   * This feature implements a software PLL on the CPU.
   * This can replace external clocks with no restrictions on correlated data to other inputs.
   * It uses a first-order loop filter to ignore the discretization noise of the input and to provide some kind of
   * cutoff frequency when to apply the extern clock.
   * \note Within the first 100 * averaging_factor * clock_period, a frequency locking approach is applied. The
   *       phase gets locked afterwards.
   *
   * \param input_channel The physical input channel
   * \param input_frequency Frequency of the configured external clock. Slight variations will be canceled out.
   *                        Defaults to 10e6 for 10 MHz
   * \param averaging_periods Times clock_period is the cutoff period for the filter. Shorter periods are evaluated
   *                          with the Time Tagger's internal clock, longer periods are evaluated with the here
   *                          configured external clock
   * \param wait_until_locked Blocks the execution until the software clock is locked. Throws an exception on
   *                          locking errors. All locking log messages are filtered while this call is executed.
   */
  virtual void setSoftwareClock(channel_t input_channel, double input_frequency = 10e6, double averaging_periods = 1000,
                                bool wait_until_locked = true) = 0;

  /**
   * \brief disabled the software PLL
   *
   * See setSoftwareClock for further details.
   */
  virtual void disableSoftwareClock() = 0;

  /**
   * \brief queries all state information of the software clock
   *
   * See setSoftwareClock for further details.
   */
  virtual SoftwareClockState getSoftwareClockState() = 0;

  /**
   * \brief get overflow count
   *
   * Get the number of communication overflows occurred
   *
   */
  virtual long long getOverflows() = 0;

  /**
   * \brief clear overflow counter
   *
   * Sets the overflow counter to zero
   */
  virtual void clearOverflows() = 0;

  /**
   * \brief get and clear overflow counter
   *
   * Get the number of communication overflows occurred and sets them to zero
   */
  virtual long long getOverflowsAndClear() = 0;

protected:
  /**
   * \brief abstract interface class
   */
  TimeTaggerBase() {}

  /**
   * \brief destructor
   */
  virtual ~TimeTaggerBase(){};

  // Non Copyable
  TimeTaggerBase(const TimeTaggerBase &) = delete;
  TimeTaggerBase &operator=(const TimeTaggerBase &) = delete;

  // Used by IteratorBase to add itself
  virtual std::shared_ptr<IteratorBaseListNode> addIterator(IteratorBase *it) = 0;

  // Used by IteratorBase to specify when it's being deleted.
  virtual void freeIterator(IteratorBase *it) = 0;

  // allocate a new virtual output channel
  virtual channel_t getNewVirtualChannel() = 0;

  // free a virtual channel being used.
  virtual void freeVirtualChannel(channel_t channel) = 0;

  /**
   * \brief register a FPGA channel.
   *
   * Only events on previously registered channels will be transferred over
   * the communication channel.
   *
   * \param channel  the channel
   */
  virtual void registerChannel(channel_t channel) = 0;
  virtual void registerChannel(std::set<channel_t> channels) = 0;

  /**
   * \brief release a previously registered channel.
   *
   * \param channel   the channel
   */
  virtual void unregisterChannel(channel_t channel) = 0;
  virtual void unregisterChannel(std::set<channel_t> channels) = 0;

  // Used by proxy time tagger to add itself as a dependent tagger.
  virtual void addChild(TimeTaggerBase *child) = 0;

  // Used by proxy time tagger to remove itself as a dependent tagger.
  virtual void removeChild(TimeTaggerBase *child) = 0;

  // Used by a proxy time tagger to allow its parent to release it and its dependencies.
  virtual void release() = 0;
};

/**
 * \brief virtual TimeTagger based on dump files
 *
 * The TimeTaggerVirtual class represents a virtual Time Tagger.
 * But instead of connecting to Swabian hardware, it replays all tags
 * from a recorded file.
 *
 * \ingroup TimeTaggers
 */
class TT_API TimeTaggerVirtual : virtual public TimeTaggerBase {
public:
  /**
   * \brief replay a given dump file on the disc
   *
   * This method adds the file to the replay queue.
   * If the flag 'queue' is false, the current queue will be flushed and this file will be replayed immediately.
   *
   * \param file the file to be replayed, must be encoded as UTF-8
   * \param begin amount of ps to skip at the begin of the file. A negative time will generate a pause in the replay
   * \param duration time period in ps of the file. -1 replays till the last tag
   * \param queue flag if this file shall be queued
   * \return ID of the queued file
   */
  virtual uint64_t replay(const std::string &file, timestamp_t begin = 0, timestamp_t duration = -1,
                          bool queue = true) = 0;

  /**
   * \brief stops the current and all queued files.
   *
   * This method stops the current file and clears the replay queue.
   */
  virtual void stop() = 0;

  /**
   * \brief stops the all queued files and resets the TimeTaggerVirtual to its default settings
   *
   * This method stops the current file, clears the replay queue and resets the TimeTaggerVirtual to its default
   * settings.
   */
  virtual void reset() = 0;

  /**
   * \brief block the current thread until the replay finish
   *
   * This method blocks the current execution and waits till the given file has finished its replay.
   * If no ID is provided, it waits until all queued files are replayed.
   *
   * This function does not block on a zero timeout. Negative timeouts are interpreted as infinite timeouts.
   *
   * \param ID selects which file to wait for
   * \param timeout timeout in milliseconds
   * \return true if the file is complete, false on timeout
   */
  virtual bool waitForCompletion(uint64_t ID = 0, int64_t timeout = -1) = 0;

  /**
   * \brief configures the speed factor for the virtual tagger.
   *
   * This method configures the speed factor of this virtual Time Tagger.
   * A value of 1.0 will replay in real time.
   * All values < 0.0 will replay the data as fast as possible, but stops at the end of all data. This is the default
   * value.
   *
   * \param speed ratio of the replay speed and the real time
   */
  virtual void setReplaySpeed(double speed) = 0;

  /**
   * \brief fetches the speed factor
   *
   * Please see setReplaySpeed for more details.
   *
   * \return the speed factor
   */
  virtual double getReplaySpeed() = 0;

  /**
   * \brief configures the conditional filter
   *
   * After each event on the trigger channels, one event per filtered channel
   * will pass afterwards. This is handled in a very early stage in the pipeline,
   * so all event limitations but the deadtime are suppressed. But the accuracy
   * of the order of those events is low.
   *
   * Refer the Manual for a description of this function.
   *
   * \param trigger the channels that sets the condition
   * \param filtered  the channels that are filtered by the condition
   */
  virtual void setConditionalFilter(std::vector<channel_t> trigger, std::vector<channel_t> filtered) = 0;

  /**
   * \brief deactivates the conditional filter
   *
   * equivalent to setConditionalFilter({},{})
   *
   */
  virtual void clearConditionalFilter() = 0;

  /**
   * \brief fetches the configuration of the conditional filter
   *
   * see setConditionalFilter
   */
  virtual std::vector<channel_t> getConditionalFilterTrigger() = 0;

  /**
   * \brief fetches the configuration of the conditional filter
   *
   * see setConditionalFilter
   */
  virtual std::vector<channel_t> getConditionalFilterFiltered() = 0;

  /**
   * \brief Fetches channels from the input file.
   *
   * \return a vector of channels from the input file.
   */
  virtual std::vector<channel_t> getChannelList() = 0;
};

enum class AccessMode { Listen = 0, Control = 2, SynchronousControl = 3 };

/**
 * \brief network TimeTagger client.
 *
 * The TimeTaggerNetwork class is a client that implements access to the Time Tagger server.
 * TimeTaggerNetwork receives the time-tag stream from the Time Tagger server over the network
 * and provides an interface for controlling connection and the Time Tagger hardware.
 * Instance of this class can be transparently used to create measurements,
 * virtual channels and other Iterator instances.
 */
class TT_API TimeTaggerNetwork : virtual public TimeTaggerBase {
public:
  /**
   * \brief check if the network time tagger is currently connected to a server
   *
   * \return          returns true if it's currently connected to a server; false, otherwise.
   */
  virtual bool isConnected() = 0;

  /**
   * \brief set the trigger voltage threshold of a channel
   *
   * \param channel   the channel to set
   * \param voltage    voltage level.. [0..1]
   */
  virtual void setTriggerLevel(channel_t channel, double voltage) = 0;

  /**
   * \brief get the trigger voltage threshold of a channel
   *
   * \param channel the channel
   */
  virtual double getTriggerLevel(channel_t channel) = 0;

  /**
   * \brief configures the conditional filter
   *
   * After each event on the trigger channels, one event per filtered channel
   * will pass afterwards. This is handled in a very early stage in the pipeline,
   * so all event limitations but the deadtime are suppressed. But the accuracy
   * of the order of those events is low.
   *
   * Refer the Manual for a description of this function.
   *
   * \param trigger the channels that sets the condition
   * \param filtered  the channels that are filtered by the condition
   * \param hardwareDelayCompensation if false, the physical hardware delay will not be compensated
   */
  virtual void setConditionalFilter(std::vector<channel_t> trigger, std::vector<channel_t> filtered,
                                    bool hardwareDelayCompensation = true) = 0;

  /**
   * \brief deactivates the conditional filter
   *
   * equivalent to setConditionalFilter({},{})
   *
   */
  virtual void clearConditionalFilter() = 0;

  /**
   * \brief fetches the configuration of the conditional filter
   *
   * see setConditionalFilter
   */
  virtual std::vector<channel_t> getConditionalFilterTrigger() = 0;

  /**
   * \brief fetches the configuration of the conditional filter
   *
   * see setConditionalFilter
   */
  virtual std::vector<channel_t> getConditionalFilterFiltered() = 0;

  /**
   * \brief set the divider for the frequency of the test signal
   *
   * The base clock of the test signal oscillator for the Time Tagger Ultra is running at 100.8 MHz sampled down by an
   * factor of 2 to have a similar base clock as the Time Tagger 20 (~50 MHz). The default divider is 63 -> ~800
   * kEvents/s
   *
   * \param divider frequency divisor of the oscillator
   */
  virtual void setTestSignalDivider(int divider) = 0;

  /**
   * \brief get the divider for the frequency of the test signal
   */
  virtual int getTestSignalDivider() = 0;

  /**
   * \brief fetch the status of the test signal generator
   *
   * \param channel   the channel
   */
  virtual bool getTestSignal(channel_t channel) = 0;

  /**
   * \brief set time delay on a channel
   *
   * When set, every event on this channel is delayed by the given delay in picoseconds.
   *
   * This delay is implemented on the client and does not affect the server nor requires the Control flag.
   *
   * \param channel  the channel to set
   * \param time     the delay in picoseconds
   */
  virtual void setDelayClient(channel_t channel, timestamp_t time) = 0;

  /**
   * \brief get the time delay of a channel
   *
   * see setDelayClient
   *
   * \param channel   the channel
   * \return the software delay in picoseconds
   */
  virtual timestamp_t getDelayClient(channel_t channel) = 0;

  /**
   * \brief get hardware delay compensation of a channel
   *
   * The physical input delays are calibrated and compensated.
   * However this compensation is implemented after the conditional filter and so affects its result.
   * This function queries the effective input delay, which compensates the hardware delay.
   *
   * \param channel the channel
   * \return the hardware delay compensation in picoseconds
   */
  virtual timestamp_t getHardwareDelayCompensation(channel_t channel) = 0;

  /**
   * \brief enables or disables the normalization of the distribution.
   *
   * Refer the Manual for a description of this function.
   *
   * \param channels list of channels to modify
   * \param state the new state
   */
  virtual void setNormalization(std::vector<channel_t> channels, bool state) = 0;

  /**
   * \brief returns the the normalization of the distribution.
   *
   * Refer the Manual for a description of this function.
   *
   * \param channel the channel to query
   * \return if the normalization is enabled
   */
  virtual bool getNormalization(channel_t channel) = 0;

  /**
   * \brief sets the maximum USB buffer size
   *
   * This option controls the maximum buffer size of the USB connection.
   * This can be used to balance low input latency vs high (peak) throughput.
   *
   * \param size the maximum buffer size in events
   */
  virtual void setHardwareBufferSize(int size) = 0;

  /**
   * \brief queries the size of the USB queue
   *
   * See setHardwareBufferSize for more information.
   *
   * \return the actual size of the USB queue in events
   */
  virtual int getHardwareBufferSize() = 0;

  /**
   * \brief sets the maximum events and latency for the stream block size
   *
   * This option controls the latency and the block size of the data stream.
   * The default values are max_events = 131072 events and max_latency = 20 ms.
   * Depending on which of the two parameters is exceeded first, the block stream size is adjusted accordingly.
   * The block size will be reduced automatically for blocks when no signal is arriving for 512 ns on the Time Tagger
   * Ultra and 1536 ns for the Time Tagger 20.   *
   *
   * \param max_events  maximum number of events
   * \param max_latency maximum latency in ms
   */
  virtual void setStreamBlockSize(int max_events, int max_latency) = 0;
  virtual int getStreamBlockSizeEvents() = 0;
  virtual int getStreamBlockSizeLatency() = 0;

  /**
   * \brief Divides the amount of transmitted edge per channel
   *
   * This filter decimates the events on a given channel by a specified.
   * factor. So for a divider n, every nth event is transmitted through
   * the filter and n-1 events are skipped between consecutive
   * transmitted events. If a conditional filter is also active, the event
   * divider is applied after the conditional filter, so the conditional
   * is applied to the complete event stream and only events which pass the
   * conditional filter are forwarded to the divider.
   *
   * As it is a hardware filter, it reduces the required USB bandwidth and
   * CPU processing power, but it cannot be configured for virtual channels.
   *
   * \param channel channel to be configured
   * \param divider new divider, must be at least 1 and smaller than 65536
   */
  virtual void setEventDivider(channel_t channel, unsigned int divider) = 0;

  /**
   * \brief Returns the factor of the dividing filter
   *
   * See setEventDivider for further details.
   *
   * \param channel channel to be queried
   * \return the configured divider
   */
  virtual unsigned int getEventDivider(channel_t channel) = 0;

  /**
   * \brief identifies the hardware by serial number
   */
  virtual std::string getSerial() = 0;

  /**
   * \brief identifies the hardware by Time Tagger Model
   */
  virtual std::string getModel() = 0;

  /**
   * \brief Fetch the configured numbering scheme for this TimeTagger object
   *
   * Please see setTimeTaggerChannelNumberScheme() for details.
   */
  virtual int getChannelNumberScheme() = 0;

  /**
   * \brief returns the minimum and the maximum voltage of the DACs as a trigger reference
   */
  virtual std::vector<double> getDACRange() = 0;

  /**
   * \brief fetch a vector of all physical input channel ids
   *
   * The function returns the channel of all rising and falling edges.
   * For example for the Time Tagger 20 (8 input channels)
   * TT_CHANNEL_NUMBER_SCHEME_ZERO: {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}
   * and for
   * TT_CHANNEL_NUMBER_SCHEME_ONE: {-8,-7,-6,-5,-4,-3,-2,-1,1,2,3,4,5,6,7,8}
   *
   * TT_CHANNEL_RISING_EDGES returns only the rising edges
   * SCHEME_ONE: {1,2,3,4,5,6,7,8}
   * and
   * TT_CHANNEL_FALLING_EDGES return only the falling edges
   * SCHEME_ONE: {-1,-2,-3,-4,-5,-6,-7,-8}
   * which are the invertedChannels of the rising edges.
   */
  virtual std::vector<channel_t> getChannelList(ChannelEdge type = ChannelEdge::All) = 0;

  /**
   * \brief fetch the duration of each clock cycle in picoseconds
   */
  virtual timestamp_t getPsPerClock() = 0;

  /**
   * \brief Return the hardware version of the PCB board. Version 0 is everything before mid 2018
   * and with the channel configuration ZERO. version >= 1 is channel configuration ONE
   */
  virtual std::string getPcbVersion() = 0;

  /**
   * \brief Return an unique identifier for the applied firmware.
   *
   * This function returns a comma separated list of the firmware version with
   * - the device identifier: TT-20 or TT-Ultra
   * - the firmware identifier: FW 3
   * - optional the timestamp of the assembling of the firmware
   * - the firmware identifier of the USB chip: OK 1.30
   * eg "TT-Ultra, FW 3, TS 2018-11-13 22:57:32, OK 1.30"
   */
  virtual std::string getFirmwareVersion() = 0;

  /**
   * \brief Show the status of the sensor data from the FPGA and peripherals on the console
   */
  virtual std::string getSensorData() = 0;

  /**
   * \brief Enforce a state to the LEDs
   * 0: led_status[R]      16: led_status[R] - mux
   * 1: led_status[G]      17: led_status[G] - mux
   * 2: led_status[B]      18: led_status[B] - mux
   * 3: led_power[R]       19: led_power[R]  - mux
   * 4: led_power[G]       20: led_power[G]  - mux
   * 5: led_power[B]       21: led_power[B]  - mux
   * 6: led_clock[R]       22: led_clock[R]  - mux
   * 7: led_clock[G]       23: led_clock[G]  - mux
   * 8: led_clock[B]       24: led_clock[B]  - mux
   */
  virtual void setLED(uint32_t bitmask) = 0;

  /**
   * \brief gets the license, installed on this device currently
   * \return a JSON string containing the current device license
   */
  virtual std::string getDeviceLicense() = 0;

  /**
   * \brief Set the Time Taggers internal buzzer to a frequency in Hz (freq_hz==0 to disable)
   * \param freq_hz the generated audio frequency
   */
  virtual void setSoundFrequency(uint32_t freq_hz) = 0;

  /**
   * \brief enable or disable additional compression of the timetag stream as ent over the network.
   *
   * \param active      set if the compressio is active or not.
   */
  virtual void setTimeTaggerNetworkStreamCompression(bool active) = 0;

  virtual long long getOverflowsClient() = 0;
  virtual void clearOverflowsClient() = 0;
  virtual long long getOverflowsAndClearClient() = 0;

  /**
   * \brief enable high impedance termination mode
   *
   * Caution:  This feature is only supported on the Time Tagger X
   *
   * \param channel channel to be configured
   * \param high_impedance set for the high impedance mode or cleared for the 50 Ohm termination mode
   */
  virtual void setInputImpedanceHigh(channel_t channel, bool high_impedance) = 0;

  /**
   * \brief query the state of the high impedance termination mode
   *
   * \param channel channel to be queried
   * \return true for the high impedance mode or false for the 50 Ohm termination mode
   */
  virtual bool getInputImpedanceHigh(channel_t channel) = 0;

  /**
   * \brief configure the hysteresis voltage of the input comparator
   *
   * Caution:  This feature is only supported on the Time Tagger X
   *           The supported hysteresis voltages are 1 mV, 20 mV or 70 mV
   *
   * \param channel channel to be configured
   * \param value the hysteresis voltage in milli Volt
   */
  virtual void setInputHysteresis(channel_t channel, int value) = 0;

  /**
   * \brief query the hysteresis voltage of the input comparator
   *
   * \param channel channel to be queried
   * \return the hysteresis voltage in milli Volt
   */
  virtual int getInputHysteresis(channel_t channel) = 0;
};

/**
 * \brief backend for the TimeTagger.
 *
 * The TimeTagger class connects to the hardware, and handles the communication over the usb.
 * There may be only one instance of the backend per physical device.
 *
 * \ingroup TimeTaggers
 */
class TT_API TimeTagger : virtual public TimeTaggerBase {
public:
  /**
   * \brief reset the TimeTagger object to default settings and detach all iterators
   */
  virtual void reset() = 0;

  virtual bool isChannelRegistered(channel_t chan) = 0;

  /**
   * \brief set the divider for the frequency of the test signal
   *
   * The base clock of the test signal oscillator for the Time Tagger Ultra is running at 100.8 MHz sampled down by an
   * factor of 2 to have a similar base clock as the Time Tagger 20 (~50 MHz). The default divider is 63 -> ~800
   * kEvents/s. The base clock for the TTX is 333.3 MHz. The default divider is tuned to ~800 kEvents/s
   *
   * \param divider frequency divisor of the oscillator
   */
  virtual void setTestSignalDivider(int divider) = 0;

  /**
   * \brief get the divider for the frequency of the test signal
   */
  virtual int getTestSignalDivider() = 0;

  /**
   * \brief set the divider for the frequency of the aux out signal generator and enable aux out
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * The base clock for the TTX is 333.3 MHz.
   *
   * \param channel select Aux Out 1 or 2
   * \param divider frequency divisor of the oscillator
   * \param duty_cycle the duty cycle ratio, will be clamped and rounded to an integer divisor
   */
  virtual void xtra_setAuxOutSignal(int channel, int divider, double duty_cycle = 0.5) = 0;

  /**
   * \brief get the divider for the frequency of the aux out signal generator
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * \param channel select Aux Out 1 or 2
   * \return the configured divider
   */
  virtual int xtra_getAuxOutSignalDivider(int channel) = 0;

  /**
   * \brief get the dutycycle of the aux out signal generator
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * \param channel select Aux Out 1 or 2
   * \return the configured duty cycle
   */
  virtual double xtra_getAuxOutSignalDutyCycle(int channel) = 0;

  /**
   * \brief enable or disable aux out
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * This will enable or disable the signal generator on the aux outputs.
   *
   * \param channel select Aux Out 1 or 2
   * \param enabled enabled / disabled flag
   */
  virtual void xtra_setAuxOut(int channel, bool enabled) = 0;

  /**
   * \brief fetch the status of the aux out signal generator
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * \param channel select Aux Out 1 or 2
   * \return true if the aux out signal generator is enabled
   */
  virtual bool xtra_getAuxOut(int channel) = 0;

  /**
   * \brief configures the FAN speed on TTU HW >= 1.3
   * \param percentage the new speed, 0 means off, 100 means full on, negative means controlled.
   * \note This setting will get reset on USB errors.
   */
  virtual void xtra_setFanSpeed(double percentage = -1) = 0;

  /**
   * \brief set the trigger voltage threshold of a channel
   *
   * \param channel   the channel to set
   * \param voltage    voltage level.. [0..1]
   */
  virtual void setTriggerLevel(channel_t channel, double voltage) = 0;

  /**
   * \brief get the trigger voltage threshold of a channel
   *
   * \param channel the channel
   */
  virtual double getTriggerLevel(channel_t channel) = 0;

  /**
   * \brief measures the electrically applied the trigger voltage threshold of a channel
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * \param channel the channel
   * \return the voltage
   */
  virtual double xtra_measureTriggerLevel(channel_t channel) = 0;

  /**
   * \brief get hardware delay compensation of a channel
   *
   * The physical input delays are calibrated and compensated.
   * However this compensation is implemented after the conditional filter and so affects its result.
   * This function queries the effective input delay, which compensates the hardware delay.
   *
   * \param channel the channel
   * \return the hardware delay compensation in picoseconds
   */
  virtual timestamp_t getHardwareDelayCompensation(channel_t channel) = 0;

  /**
   * \brief configures the input multiplexer
   *
   * Every physical input channel has an input multiplexer with 4 modes:
   * 0: normal input mode
   * 1: use the input from channel -1 (left)
   * 2: use the input from channel +1 (right)
   * 3: use the reference oscillator
   *
   * Mode 1 and 2 cascades, so many inputs can be configured to get the same input events.
   *
   * \param channel the physical channel of the input multiplexer
   * \param mux_mode the configuration mode of the input multiplexer
   */
  virtual void setInputMux(channel_t channel, int mux_mode) = 0;

  /**
   * \brief fetches the configuration of the input multiplexer
   *
   * \param channel the physical channel of the input multiplexer
   * \return the configuration mode of the input multiplexer
   */
  virtual int getInputMux(channel_t channel) = 0;

  /**
   * \brief configures the conditional filter
   *
   * After each event on the trigger channels, one event per filtered channel
   * will pass afterwards. This is handled in a very early stage in the pipeline,
   * so all event limitations but the deadtime are suppressed. But the accuracy
   * of the order of those events is low.
   *
   * Refer the Manual for a description of this function.
   *
   * \param trigger the channels that sets the condition
   * \param filtered  the channels that are filtered by the condition
   * \param hardwareDelayCompensation if false, the physical hardware delay will not be compensated
   */
  virtual void setConditionalFilter(std::vector<channel_t> trigger, std::vector<channel_t> filtered,
                                    bool hardwareDelayCompensation = true) = 0;

  /**
   * \brief deactivates the conditional filter
   *
   * equivalent to setConditionalFilter({},{})
   *
   */
  virtual void clearConditionalFilter() = 0;

  /**
   * \brief fetches the configuration of the conditional filter
   *
   * see setConditionalFilter
   */
  virtual std::vector<channel_t> getConditionalFilterTrigger() = 0;

  /**
   * \brief fetches the configuration of the conditional filter
   *
   * see setConditionalFilter
   */
  virtual std::vector<channel_t> getConditionalFilterFiltered() = 0;

  /**
   * \brief enables or disables the normalization of the distribution.
   *
   * Refer the Manual for a description of this function.
   *
   * \param channels list of channels to modify
   * \param state the new state
   */
  virtual void setNormalization(std::vector<channel_t> channels, bool state) = 0;

  /**
   * \brief returns the the normalization of the distribution.
   *
   * Refer the Manual for a description of this function.
   *
   * \param channel the channel to query
   * \return if the normalization is enabled
   */
  virtual bool getNormalization(channel_t channel) = 0;

  /**
   * \brief sets the maximum USB buffer size
   *
   * This option controls the maximum buffer size of the USB connection.
   * This can be used to balance low input latency vs high (peak) throughput.
   *
   * \param size the maximum buffer size in events
   */
  virtual void setHardwareBufferSize(int size) = 0;

  /**
   * \brief queries the size of the USB queue
   *
   * See setHardwareBufferSize for more information.
   *
   * \return the actual size of the USB queue in events
   */
  virtual int getHardwareBufferSize() = 0;

  /**
   * \brief sets the maximum events and latency for the stream block size
   *
   * This option controls the latency and the block size of the data stream.
   * The default values are max_events = 131072 events and max_latency = 20 ms.
   * Depending on which of the two parameters is exceeded first, the block stream size is adjusted accordingly.
   * The block size will be reduced automatically for blocks when no signal is arriving for 512 ns on the Time Tagger
   * Ultra and 1536 ns for the Time Tagger 20.   *
   *
   * \param max_events  maximum number of events
   * \param max_latency maximum latency in ms
   */
  virtual void setStreamBlockSize(int max_events, int max_latency) = 0;
  virtual int getStreamBlockSizeEvents() = 0;
  virtual int getStreamBlockSizeLatency() = 0;

  /**
   * \brief Divides the amount of transmitted edge per channel
   *
   * This filter decimates the events on a given channel by a specified.
   * factor. So for a divider n, every nth event is transmitted through
   * the filter and n-1 events are skipped between consecutive
   * transmitted events. If a conditional filter is also active, the event
   * divider is applied after the conditional filter, so the conditional
   * is applied to the complete event stream and only events which pass the
   * conditional filter are forwarded to the divider.
   *
   * As it is a hardware filter, it reduces the required USB bandwidth and
   * CPU processing power, but it cannot be configured for virtual channels.
   *
   * \param channel channel to be configured
   * \param divider new divider, must be at least 1 and smaller than 65536
   */
  virtual void setEventDivider(channel_t channel, unsigned int divider) = 0;

  /**
   * \brief Returns the factor of the dividing filter
   *
   * See setEventDivider for further details.
   *
   * \param channel channel to be queried
   * \return the configured divider
   */
  virtual unsigned int getEventDivider(channel_t channel) = 0;

  /**
   * \brief runs a calibrations based on the on-chip uncorrelated signal generator.
   */
  GET_DATA_1D(autoCalibration, double, array_out, virtual) = 0;

  /**
   * \brief identifies the hardware by serial number
   */
  virtual std::string getSerial() = 0;

  /**
   * \brief identifies the hardware by Time Tagger Model
   */
  virtual std::string getModel() = 0;

  /**
   * \brief Fetch the configured numbering scheme for this TimeTagger object
   *
   * Please see setTimeTaggerChannelNumberScheme() for details.
   */
  virtual int getChannelNumberScheme() = 0;

  /**
   * \brief returns the minimum and the maximum voltage of the DACs as a trigger reference
   */
  virtual std::vector<double> getDACRange() = 0;

  /**
   * \brief get internal calibration data
   */
  GET_DATA_2D(getDistributionCount, uint64_t, array_out, virtual) = 0;

  /**
   * \brief get internal calibration data
   */
  GET_DATA_2D(getDistributionPSecs, double, array_out, virtual) = 0;

  /**
   * \brief fetch a vector of all physical input channel ids
   *
   * The function returns the channel of all rising and falling edges.
   * For example for the Time Tagger 20 (8 input channels)
   * TT_CHANNEL_NUMBER_SCHEME_ZERO: {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}
   * and for
   * TT_CHANNEL_NUMBER_SCHEME_ONE: {-8,-7,-6,-5,-4,-3,-2,-1,1,2,3,4,5,6,7,8}
   *
   * TT_CHANNEL_RISING_EDGES returns only the rising edges
   * SCHEME_ONE: {1,2,3,4,5,6,7,8}
   * and
   * TT_CHANNEL_FALLING_EDGES return only the falling edges
   * SCHEME_ONE: {-1,-2,-3,-4,-5,-6,-7,-8}
   * which are the invertedChannels of the rising edges.
   */
  virtual std::vector<channel_t> getChannelList(ChannelEdge type = ChannelEdge::All) = 0;

  /**
   * \brief fetch the duration of each clock cycle in picoseconds
   */
  virtual timestamp_t getPsPerClock() = 0;

  /**
   * \brief Return the hardware version of the PCB board. Version 0 is everything before mid 2018
   * and with the channel configuration ZERO. version >= 1 is channel configuration ONE
   */
  virtual std::string getPcbVersion() = 0;

  /**
   * \brief Return an unique identifier for the applied firmware.
   *
   * This function returns a comma separated list of the firmware version with
   * - the device identifier: TT-20 or TT-Ultra
   * - the firmware identifier: FW 3
   * - optional the timestamp of the assembling of the firmware
   * - the firmware identifier of the USB chip: OK 1.30
   * eg "TT-Ultra, FW 3, TS 2018-11-13 22:57:32, OK 1.30"
   */
  virtual std::string getFirmwareVersion() = 0;

  /**
   * \brief manually overwrite the reference clock source
   *
   * 0: internal clock
   * 1: external 10 MHz
   * 2: external 500 MHz
   *
   * \param source selects the clock source
   */
  virtual void xtra_setClockSource(int source) = 0;

  /**
   * \brief fetch the overwritten reference clock source
   *
   * -1: auto selecting of below options
   * 0: internal clock
   * 1: external 10 MHz
   * 2: external 500 MHz
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * \return selects the clock source
   */
  virtual int xtra_getClockSource() = 0;

  /**
   * \brief activates auto clocking function
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * \param enabled true for auto detection of external clock
   */
  virtual void xtra_setClockAutoSelect(bool enabled) = 0;

  /**
   * \brief queries if the auto clocking function is enabled
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * \return true if the external clock auto detection is enabled
   */
  virtual bool xtra_getClockAutoSelect() = 0;

  /**
   * \brief enables the clock output
   *
   * Caution: this feature is for development purposes only and may not be part of future builds without further notice.
   *
   * \param enabled true for enabling the 10 MHz clock output
   */
  virtual void xtra_setClockOut(bool enabled) = 0;

  /**
   * \brief Show the status of the sensor data from the FPGA and peripherals on the console
   */
  virtual std::string getSensorData() = 0;

  /**
   * \brief Enforce a state to the LEDs
   * 0: led_status[R]      16: led_status[R] - mux
   * 1: led_status[G]      17: led_status[G] - mux
   * 2: led_status[B]      18: led_status[B] - mux
   * 3: led_power[R]       19: led_power[R]  - mux
   * 4: led_power[G]       20: led_power[G]  - mux
   * 5: led_power[B]       21: led_power[B]  - mux
   * 6: led_clock[R]       22: led_clock[R]  - mux
   * 7: led_clock[G]       23: led_clock[G]  - mux
   * 8: led_clock[B]       24: led_clock[B]  - mux
   */
  virtual void setLED(uint32_t bitmask) = 0;

  /**
   * \brief disables the LEDs on the TT
   *
   * Caution: This feature currently lacks support for disabling the power LED on the Time Tagger X.
   *
   * \param disabled true to disable all LEDs on the TT
   */
  virtual void disableLEDs(bool disabled) = 0;

  /**
   * \brief gets the license, installed on this device currently
   * \return a JSON string containing the current device license
   */
  virtual std::string getDeviceLicense() = 0;

  /**
   * \brief Direct read/write access to WireIn/WireOuts in FPGA (mask==0 for readonly)
   *
   * DO NOT USE. Only for internal debug purposes.
   */
  virtual uint32_t factoryAccess(uint32_t pw, uint32_t addr, uint32_t data, uint32_t mask, bool use_wb = false) = 0;

  /**
   * \brief Set the Time Taggers internal buzzer to a frequency in Hz (freq_hz==0 to disable)
   *
   * \param freq_hz the generated audio frequency
   */
  virtual void setSoundFrequency(uint32_t freq_hz) = 0;

  /**
   * \brief Enable the FPGA link of the Time Tagger X
   *
   * \param channels list of channels, which shall be streamed over the FPGA link
   * \param destination_mac Destination MAC, use an empty string for the broadcast address of "FF:FF:FF:FF:FF:FF"
   * \param link_interface selects which interface shall be used, default is FpgaLinkInterface::SFPP_10GE
   * \param exclusive determines if time tags should exclusively be transmitted over Ethernet, increasing Ethernet
   * performance and avoiding USB issues, default is mixed USB & ethernet
   */
  virtual void enableFpgaLink(std::vector<channel_t> channels, std::string destination_mac,
                              FpgaLinkInterface link_interface = FpgaLinkInterface::SFPP_10GE,
                              bool exclusive = false) = 0;

  /**
   * \brief Disable the FPGA link of the Time Tagger X
   */
  virtual void disableFpgaLink() = 0;

  /**
   * \brief starts the Time Tagger server that will stream the time tags to the client.
   *
   * \param access_mode set the type of access a user can have.
   * \param port        port at which this time tagger server will be listening on.
   * \param channels    channels to be streamed, if empty, all the channels will be exposed.
   */
  virtual void startServer(AccessMode access_mode, std::vector<channel_t> channels = std::vector<channel_t>(),
                           uint32_t port = 41101) = 0;

  /**
   * \brief check if the server is still running.
   *
   * \return          returns true if running; false, if not running
   */
  virtual bool isServerRunning() = 0;

  /**
   * \brief stops the time tagger server if currently running, otherwise does nothing.
   *
   */
  virtual void stopServer() = 0;

  /**
   * \brief enable or disable additional compression of the timetag stream as ent over the network.
   *
   * \param active      set if the compression is active or not.
   */
  virtual void setTimeTaggerNetworkStreamCompression(bool active) = 0;

  /**
   * \brief enable high impedance termination mode
   *
   * Caution:  This feature is only supported on the Time Tagger X
   *
   * \param channel channel to be configured
   * \param high_impedance set for the high impedance mode or cleared for the 50 Ohm termination mode
   */
  virtual void setInputImpedanceHigh(channel_t channel, bool high_impedance) = 0;

  /**
   * \brief query the state of the high impedance termination mode
   *
   * \param channel channel to be queried
   * \return true for the high impedance mode or false for the 50 Ohm termination mode
   */
  virtual bool getInputImpedanceHigh(channel_t channel) = 0;

  /**
   * \brief configure the hysteresis voltage of the input comparator
   *
   * Caution:  This feature is only supported on the Time Tagger X
   *           The supported hysteresis voltages are 1 mV, 20 mV or 70 mV
   *
   * \param channel channel to be configured
   * \param value the hysteresis voltage in milli Volt
   */
  virtual void setInputHysteresis(channel_t channel, int value) = 0;

  /**
   * \brief query the hysteresis voltage of the input comparator
   *
   * \param channel channel to be queried
   * \return the hysteresis voltage in milli Volt
   */
  virtual int getInputHysteresis(channel_t channel) = 0;

  /**
   * \brief configures if the rising and falling events shall be averaged
   *
   * This is implemented on the device before any filter like event divider and it
   * does not require to transfer both events.
   *
   * They need to be manually delayed to be within a window of +-500 ps of error, else events might get lost.
   * This method has no side effects on the channel `getInvertedChannel(channel)`, you can still fetch the original
   * events there. However if both are configured to return the averaged result, the timestamps will be identical.
   *
   * \param channel the channel, on which the average value shall be returned
   * \param enable true if this channel shall yield the averaged timestamps
   */
  virtual void xtra_setAvgRisingFalling(channel_t channel, bool enable) = 0;

  /**
   * \brief query if the rising and falling events shall be averaged
   *
   * \param channel channel to be queried
   * \return if the rising and falling events shall be averaged
   */
  virtual bool xtra_getAvgRisingFalling(channel_t channel) = 0;

  /**
   * \brief configures if this channel shall exit overflow regions.
   *
   * If configured, each event of this channel within an overflow region will emit an OverflowEnd marker before the
   * event and an OverflowBegin marker after the event. This can be used to split up regions by a slow trigger, e.g. for
   * CountBetweenMarker usage.
   *
   * \warning Using this option disables the internal safety method for unrecoverable memory overflows. So only use this
   * option on channels with a low datarate, else expect to get Error events, which invalidates the global time.
   *
   * \param channel the channel, which shall be configured as high priority
   * \param enable true if this channel shall have a high priority
   */
  virtual void xtra_setHighPrioChannel(channel_t channel, bool enable) = 0;

  /**
   * \brief if this channel shall exit overflow regions
   *
   * \param channel channel to be queried
   * \return if this channel shall exit overflow regions
   */
  virtual bool xtra_getHighPrioChannel(channel_t channel) = 0;

  /**
   * \brief updates the firmware of the Time Tagger X board management controller
   *
   * \note The firmware is applied on the next power cycle of the device, *not* on pressing the power button.
   *
   * \param firmware filename of the new firmware on disc
   */
  virtual void updateBMCFirmware(const std::string &firmware) = 0;
};

/**
 * \brief a single event on a channel
 *
 * Channel events are passed from the backend to registered iterators
 * by the IteratorBase::next() callback function.
 *
 * A Tag describes a single event on a channel.
 */
struct TT_API Tag {
  /**
   * \brief This enum marks what kind of event this object represents
   *
   * - TimeTag: a normal event from any input channel
   * - Error: an error in the internal data processing, e.g. on plugging the external clock. This invalidates the global
   *          time
   * - OverflowBegin: this marks the begin of an interval with incomplete data because of too high data rates
   * - OverflowEnd: this marks the end of the interval. All events, which were lost in this interval, have been handled
   * - MissedEvents: this virtual event signals the amount of lost events per channel within an overflow interval.
   *                 Repeated usage for higher amounts of events
   */
  enum class Type : unsigned char {
    TimeTag = 0,
    Error = 1,
    OverflowBegin = 2,
    OverflowEnd = 3,
    MissedEvents = 4
  } type{Type::TimeTag};

  /**
   * \brief 8 bit padding
   *
   * Reserved for future use. Set it to zero.
   */
  char reserved{};

  /**
   * \brief Amount of missed events in overflow mode
   *
   * Within overflow intervals, the timing of all events is skipped. However, the total amount of events is still
   * recorded. For events with type = MissedEvents, this indicates that a given amount of tags for this channel have
   * been skipped in the interval. Note: There might be many missed events tags per overflow interval and channel. The
   * accumulated amount represents the total skipped events.
   */
  unsigned short missed_events{};

  /// the channel number
  channel_t channel{};

  /// the timestamp of the event in picoseconds
  timestamp_t time{};

  Tag() {}
  Tag(timestamp_t ts, channel_t ch, Type type = Type::TimeTag) : type{type}, channel{ch}, time{ts} {}
  Tag(Type type, char reserved, unsigned short missed_events, channel_t ch, timestamp_t ts)
      : type{type}, reserved{reserved}, missed_events{missed_events}, channel{ch}, time{ts} {}
};

TT_API bool operator==(Tag const &a, Tag const &b);

/// Helper for implementing parallel measurements
class TT_API OrderedBarrier {
public:
  /// Internal object for serialization
  class TT_API OrderInstance {
  public:
    OrderInstance();
    OrderInstance(OrderedBarrier *parent, uint64_t instance_id);
    ~OrderInstance();
    void sync();
    void release();

  private:
    friend class OrderedBarrier;

    OrderedBarrier *parent{};
    bool obtained{};
    uint64_t instance_id{};
  };

  OrderedBarrier();
  ~OrderedBarrier();

  OrderInstance queue();
  void waitUntilFinished();

private:
  friend class OrderInstance;

  void release(uint64_t index);
  void obtain(uint64_t index);

  uint64_t accumulator{};
  uint64_t current_state{};
  std::mutex inner_mutex;
  std::condition_variable cv;
};

/// Helper for implementing parallel measurements
class TT_API OrderedPipeline {
public:
  OrderedPipeline();
  ~OrderedPipeline();

private:
  friend class IteratorBase;

  bool initialized = false;
  std::list<OrderedBarrier>::iterator stage;
};

/**
 * \brief Base class for all iterators
 */
class TT_API IteratorBase {
  friend class TimeTaggerRunner;
  friend class TimeTaggerProxy;
  friend class SynchronizedMeasurements;
  friend class IteratorTest;

private:
  // Abstract class
  IteratorBase() = delete;

  // Non Copyable
  IteratorBase(const IteratorBase &) = delete;
  IteratorBase &operator=(const IteratorBase &) = delete;
  void clearWithoutLock();

protected:
  /**
   * \brief Standard constructor, which will register with the Time Tagger backend.
   */
  IteratorBase(TimeTaggerBase *tagger, std::string base_type_ = "IteratorBase", std::string extra_info_ = "");

public:
  /**
   * \brief destructor, will unregister from the Time Tagger prior finalization.
   */
  virtual ~IteratorBase();

  /**
   * \brief Starts or continues data acquisition.
   *
   * This method is implicitly called when a measurement object is created.
   */
  void start();

  /**
   * \brief Starts or continues the data acquisition for the given duration.
   *
   * After the duration time, the method stop() is called and isRunning() will return False. Whether the accumulated
   * data is cleared at the beginning of startFor() is controlled with the second parameter clear, which is True by
   * default.
   *
   * \param capture_duration capture duration in picoseconds until the measurement is stopped
   * \param clear resets the data acquired
   */
  void startFor(timestamp_t capture_duration, bool clear = true);

  /**
   * \brief Blocks the execution until the measurement has finished. Can be used with startFor().
   *
   * waitUntilFinished will wait according to the timeout and return true if the iterator finished or false if not.
   * Furthermore, when waitUntilFinished is called on a iterator running indefinitely, it will log an error and
   * return immediately.
   *
   * \param timeout     time in milliseconds to wait for the measurements. If negative, wait until finished.
   * \return True if the measurement has finished, false on timeout
   */
  bool waitUntilFinished(int64_t timeout = -1);

  /**
   * \brief After calling this method, the measurement will stop processing incoming tags.
   *
   * Use start() or startFor() to continue or restart the measurement.
   */
  void stop();

  /**
   * \brief Discards accumulated measurement data, initializes the data buffer with zero values, and resets the state to
   * the initial state.
   */
  void clear();

  /**
   * \brief Immediately aborts the measurement, discarding accumulated measurement data, and resets the state to the
   * initial state.
   *
   * \warning After calling `abort`, the last block of data might become irreversibly corrupted.
   * Please always use `stop` to end a measurement.
   */
  void abort();

  /**
   * \brief Returns True if the measurement is collecting the data.
   *
   * This method will returns False if the measurement was stopped manually by calling stop() or automatically after
   * calling startFor() and the duration has passed.
   *
   * \note All measurements start accumulating data immediately after their creation.
   * \return True if the measurement is still running
   */
  bool isRunning();

  /**
   * \brief Total capture duration since the measurement creation or last call to clear().
   *
   * \return Capture duration in ps
   */
  timestamp_t getCaptureDuration();

  /**
   * \brief Fetches the overall configuration status of the measurement.
   *
   * \return a JSON serialized string with all configuration and status flags.
   */
  std::string getConfiguration();

  /**
   * \brief A custom runtime error thrown by the `abort` call. This can be caught and handled by measurement classes,
   * including `CustomMeasurement`, to perform actions within the abortion process.
   *
   */
  class AbortError : public std::runtime_error {
  public:
    AbortError(const std::string &what_arg) : std::runtime_error(what_arg){};
    ~AbortError(){};
  };

protected:
  /**
   * \brief register a channel
   *
   * Only channels registered by any iterator attached to a backend are delivered over the usb.
   *
   * \param channel  the channel
   */
  void registerChannel(channel_t channel);

  /**
   * \brief unregister a channel
   *
   * \param channel  the channel
   */
  void unregisterChannel(channel_t channel);

  /**
   * \brief allocate a new virtual output channel for this iterator
   */
  channel_t getNewVirtualChannel();

  /**
   * \brief method to call after finishing the initialization of the measurement
   */
  void finishInitialization();

  /**
   * \brief clear Iterator state.
   *
   * Each Iterator should implement the clear_impl() method to reset
   * its internal state.
   * The clear_impl() function is guarded by the update lock.
   */
  virtual void clear_impl() {};

  /**
   * \brief callback when the measurement class is started
   *
   * This function is guarded by the update lock.
   */
  virtual void on_start() {};

  /**
   * \brief callback when the measurement class is stopped
   *
   * This function is guarded by the update lock.
   */
  virtual void on_stop() {};

  /**
   * \brief acquire update lock
   *
   * All mutable operations on a iterator are guarded with an update mutex.
   * Implementers are advised to lock() an iterator, whenever internal state
   * is queried or changed.
   *
   * \deprecated use getLock
   */
  void lock();

  /**
   * \brief release update lock
   *
   * see lock()
   *
   * \deprecated use getLock
   */
  void unlock();

  /**
   * \brief release lock and continue work in parallel
   *
   * The measurement's lock is released, allowing this measurement to continue,
   * while still executing work in parallel.
   *
   * \return a ordered barrier instance that can be synced afterwards.
   */
  OrderedBarrier::OrderInstance parallelize(OrderedPipeline &pipeline);

  /**
   * \brief acquire update lock
   *
   * All mutable operations on a iterator are guarded with an update mutex.
   * Implementers are advised to lock an iterator, whenever internal state
   * is queried or changed.
   *
   * \return a lock object, which releases the lock when this instance is freed
   */
  std::unique_lock<std::mutex> getLock();

  /**
   * \brief update iterator state
   *
   * Each Iterator must implement the next_impl() method.
   * The next_impl() function is guarded by the update lock.
   *
   * The backend delivers each Tag on each registered channel
   * to this callback function.
   *
   * \param incoming_tags block of events
   * \param begin_time earliest event in the block
   * \param end_time begin_time of the next block, not including in this block
   * \return true if the content of this block was modified, false otherwise
   */
  virtual bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) = 0;

  /**
   * \brief Callback for the measurement to stop itself.
   *
   * It shall only be called while the measurement mutex is locked.
   * It will make sure that no new data is passed to this measurement.
   * The caller has to call on_stop themself if needed.
   */
  void finish_running();

  /// list of channels used by the iterator
  std::set<channel_t> channels_registered;

  /// running state of the iterator
  bool running;

  /// Condition if this measurement shall be started by the finishInitialization callback.
  bool autostart;

  /// Pointer to the corresponding Time Tagger object.
  TimeTaggerBase *tagger;

  /// Duration the iterator has already processed data
  timestamp_t capture_duration;
  /// For internal use.
  timestamp_t pre_capture_duration;

  // to abort measurement;
  std::atomic<bool> aborting;
  // This call shall be placed in every next_impl loop to allow single threaded measurements to abort.
  // It will be inlined wherever header is included
  void checkForAbort() {
    if (aborting) {
      on_abort();
    }
  };
  // Overload for multithreaded measurements
  template <typename T> void checkForAbort(T callback) {
    if (aborting) {
      callback();
      on_abort();
    }
  };

private:
  struct TelemetryData {
    uint64_t duration;
    bool is_on;
  };

  void next(std::unique_lock<std::mutex> &lock, std::vector<Tag> &incoming_tags, timestamp_t begin_time,
            timestamp_t end_time, uint32_t fence, TelemetryData &telem_data);

  void pre_stop();
  void on_abort();
  std::shared_ptr<IteratorBaseListNode> iter;
  timestamp_t max_capture_duration; // capture duration at which the .stop() method will be called, <0 for infinity
  std::mutex pre_stop_mutex;
  uint32_t min_fence;
  std::unordered_set<channel_t> virtual_channels;
  const std::string base_type;
  const std::string extra_info;
  uint64_t id{};
  bool initialized{};
  uint64_t clear_tick{};
};
using _Iterator = IteratorBase;

enum class LanguageUsed : std::uint32_t {
  Cpp = 0,
  Python,
  Csharp,
  Matlab,
  Labview,
  Mathematica,
  // Add more languages/Platforms
  Unknown = 255,
};

enum class FrontendType : std::uint32_t {
  Undefined = 0,
  WebApp,
  Firefly,
  Pyro5RPC,
  UserFrontend,
};

/**
 * \brief sets the language being used currently for usage statistics system.
 *
 * \param pw  password for authorization to change the language.
 * \param language  programming language being used.
 * \param version  version of the programming language being used.
 */
TT_API void setLanguageInfo(std::uint32_t pw, LanguageUsed language, std::string version);

/**
 * \brief sets the frontend being used currently for usage statistics system.
 *
 * \param frontend  the frontend currently being used.
 */
TT_API void setFrontend(FrontendType frontend);

enum class UsageStatisticsStatus {
  Disabled,               // User Opted out
  Collecting,             // User enabled it to collect for debug purpose
  CollectingAndUploading, // User gave their consent to collect and upload
};

/**
 * \brief sets the status of the usage statistics system.
 *
 * This functionality allows configuring the usage statistics system.
 *
 * \param new_status  new status of the usage statistics system.
 */
TT_API void setUsageStatisticsStatus(UsageStatisticsStatus new_status);

/**
 * \brief gets the status of the usage statistics system.
 *
 * \return the current status of the usage statistics system.
 */
TT_API UsageStatisticsStatus getUsageStatisticsStatus();

/**
 * \brief gets the current recorded data by the usage statistics system.
 *
 * Use this function to see what data has been collected so far and what will be sent to Swabian Instruments if
 * 'CollectingAndUploading' is enabled. All data is pseudonymous.
 *
 * \note if no data has been collected or due to a system error, the database was corrupted, it will return an error.
 * else it will be a database in json format.
 *
 * \return the current recorded data by the usage statistics system.
 */
TT_API std::string getUsageStatisticsReport();

/**
 * \brief merges several tag streams.
 *
 * The function reads tags from several input streams, adjusts channel numbers and tag time
 * as specified by 'channel_offsets' and 'time_offsets' respectively,
 * and merges them to a single output stream.
 * Throws if merge cannot be done.
 *
 * \param output_filename  output stream file name, splitting is done as in 'FileWriter', with 1GB file size limit.
 * \param input_filenames  file names of input streams.
 * \param channel_offsets  offsets to shift channel numbers for corresponding input streams.
 * \param time_offsets     offsets to shift tag time for corresponding input streams.
 * \param overlap_only     specifies if only events in the time overlapping region of all input streams should be
 *                         merged.
 */
TT_API void mergeStreamFiles(const std::string &output_filename, const std::vector<std::string> &input_filenames,
                             const std::vector<int> &channel_offsets, const std::vector<timestamp_t> &time_offsets,
                             bool overlap_only);

#endif /* TIMETAGGER_H_ */
