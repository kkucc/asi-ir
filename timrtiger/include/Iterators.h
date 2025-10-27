/*
This file is part of Time Tagger software defined digital data acquisition.

Copyright (C) 2011-2019 Swabian Instruments
All Rights Reserved

Unauthorized copying of this file is strictly prohibited.
*/

#ifndef TT_ITERATORS_H_
#define TT_ITERATORS_H_

#include <algorithm>
#include <array>
#include <assert.h>
#include <atomic>
#include <complex>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <set>
#include <stdint.h>
#include <stdio.h>
#include <unordered_map>
#include <vector>

// Include mulh helpers on MSVC
#if !defined(__SIZEOF_INT128__) && (defined(_M_X64) || defined(_M_ARM64))
#include <intrin.h>
#endif

#include "TimeTagger.h"

/**
 * \brief Helper class for fast division with a constant divisor.
 *
 * It chooses the method on initialization time and precompile the evaluation functions for all methods.
 */
class TT_API FastBinning {
public:
  enum class Mode {
    ConstZero,
    Dividend,
    PowerOfTwo,
    FixedPoint_32,
    FixedPoint_64,
    Divide_32,
    Divide_64,
  };

  FastBinning() {}

  FastBinning(uint64_t divisor, uint64_t max_duration_);

  template <Mode mode> uint64_t divide(uint64_t duration) const {
    assert(duration <= max_duration);
    assert(mode == this->mode);
    uint64_t out;
    switch (mode) {
    case Mode::ConstZero:
      out = 0;
      break;
    case Mode::Dividend:
      out = duration;
      break;
    case Mode::PowerOfTwo:
      out = duration >> bits_shift;
      break;
    case Mode::FixedPoint_32:
      out = (duration * factor) >> 32;
      break;
    case Mode::FixedPoint_64:
      out = MulHigh(duration, factor);
      break;
    case Mode::Divide_32:
      out = uint32_t(duration) / uint32_t(divisor);
      break;
    case Mode::Divide_64:
      out = duration / divisor;
      break;
    }
    assert(out == duration / divisor);
    return out;
  }

  Mode getMode() const { return mode; }

private:
  // returns (a*b) >> 64 in a generic but accelerated way
  uint64_t MulHigh(uint64_t a, uint64_t b) const {
#ifdef __SIZEOF_INT128__
    return ((unsigned __int128)a * (unsigned __int128)b) >> 64; // GCC, clang, ...
#elif defined(_M_X64) || defined(_M_ARM64)
    return __umulh(a, b); // MSVC
#else
    // Generic fallback
    uint64_t a_lo = uint32_t(a);
    uint64_t a_hi = a >> 32;
    uint64_t b_lo = uint32_t(b);
    uint64_t b_hi = b >> 32;

    uint64_t a_x_b_hi = a_hi * b_hi;
    uint64_t a_x_b_mid = a_hi * b_lo;
    uint64_t b_x_a_mid = b_hi * a_lo;
    uint64_t a_x_b_lo = a_lo * b_lo;

    uint64_t carry_bit = ((uint64_t)(uint32_t)a_x_b_mid + (uint64_t)(uint32_t)b_x_a_mid + (a_x_b_lo >> 32)) >> 32;

    uint64_t multhi = a_x_b_hi + (a_x_b_mid >> 32) + (b_x_a_mid >> 32) + carry_bit;

    return multhi;
#endif
  }

  uint64_t divisor;
  uint64_t max_duration;
  uint64_t factor;
  int bits_shift;
  Mode mode;
};

/// FastBinning caller helper
#define BINNING_TEMPLATE_HELPER(fun_name, binner, ...)                                                                 \
  switch (binner.getMode()) {                                                                                          \
  case FastBinning::Mode::ConstZero:                                                                                   \
    fun_name<FastBinning::Mode::ConstZero>(__VA_ARGS__);                                                               \
    break;                                                                                                             \
  case FastBinning::Mode::Dividend:                                                                                    \
    fun_name<FastBinning::Mode::Dividend>(__VA_ARGS__);                                                                \
    break;                                                                                                             \
  case FastBinning::Mode::PowerOfTwo:                                                                                  \
    fun_name<FastBinning::Mode::PowerOfTwo>(__VA_ARGS__);                                                              \
    break;                                                                                                             \
  case FastBinning::Mode::FixedPoint_32:                                                                               \
    fun_name<FastBinning::Mode::FixedPoint_32>(__VA_ARGS__);                                                           \
    break;                                                                                                             \
  case FastBinning::Mode::FixedPoint_64:                                                                               \
    fun_name<FastBinning::Mode::FixedPoint_64>(__VA_ARGS__);                                                           \
    break;                                                                                                             \
  case FastBinning::Mode::Divide_32:                                                                                   \
    fun_name<FastBinning::Mode::Divide_32>(__VA_ARGS__);                                                               \
    break;                                                                                                             \
  case FastBinning::Mode::Divide_64:                                                                                   \
    fun_name<FastBinning::Mode::Divide_64>(__VA_ARGS__);                                                               \
    break;                                                                                                             \
  }

/**
 * \defgroup ITERATOR All measurements and virtual channels
 *
 * \brief Base iterators for photon counting applications
 */

/**
 * \defgroup EventCounting Event counting
 * \ingroup ITERATOR
 */

/**
 * \defgroup TimeHistograms Time histograms
 * \ingroup ITERATOR
 *
 * \brief This section describes various measurements that calculate time differences between events and accumulate the
 * results into a histogram.
 */

/**
 * \defgroup FLIM Fluorescence-lifetime imaging (FLIM)
 * \ingroup ITERATOR
 *
 * \brief This section describes the Flim related measurements classes of the Time Tagger API.
 */

/**
 * \defgroup PhaseAndFrequency Phase & frequency analysis
 * \ingroup ITERATOR
 *
 * \brief This section describes measurement classes that expect periodic signals.
 */

/**
 * \defgroup TimeTagStreaming Time-tag-streaming
 * \ingroup ITERATOR
 *
 * \brief Measurement classes described in this section provide direct access to the time tag stream with minimal or no
 * pre-processing.
 */

/**
 * \defgroup HelperClasses Helper classes
 * \ingroup ITERATOR
 */

/**
 * \defgroup VirtualChannels Virtual Channels
 * \ingroup ITERATOR
 *
 * Virtual channels are software-defined channels as compared to the real input channels. Virtual channels can be
 * understood as a stream flow processing units. They have an input through which they receive time-tags from a real or
 * another virtual channel and output to which they send processed time-tags.
 *
 * Virtual channels are used as input channels to the measurement classes the same way as real channels. Since the
 * virtual channels are created during run-time, the corresponding channel number(s) are assigned dynamically and can be
 * retrieved using getChannel() or getChannels() methods of virtual channel object.
 */

class CombinerImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief Combine some channels in a virtual channel which has a tick for each tick in the input channels
 *
 * \image html Combiner.svg
 * \image latex Combiner.pdf
 *
 * This iterator can be used to get aggregation channels, eg if you want to monitor the countrate
 * of the sum of two channels.
 */
class TT_API Combiner : public IteratorBase {
public:
  /**
   * \brief construct a combiner
   *
   * \param tagger    reference to a TimeTagger
   * \param channels    vector of channels to combine
   */
  Combiner(TimeTaggerBase *tagger, std::vector<channel_t> channels);

  ~Combiner();

  /**
   * \brief get sum of counts
   *
   * For reference, this iterators sums up how much ticks are generated because of which input channel.
   * So this functions returns an array with one value per input channel.
   */
  GET_DATA_1D(getChannelCounts, int64_t, array_out, );

  /**
   * \brief get sum of counts
   *
   * deprecated, use getChannelCounts instead.
   */
  GET_DATA_1D(getData, int64_t, array_out, );

  /**
   * \brief the new virtual channel
   *
   * This function returns the new allocated virtual channel.
   * It can be used now in any new iterator.
   *
   */
  channel_t getChannel();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class CombinerImpl;
  std::unique_ptr<CombinerImpl> impl;
};

class CountBetweenMarkersImpl;
/**
 * \ingroup EventCounting
 *
 * \brief a simple counter where external marker signals determine the bins
 *
 * \image html CountBetweenMarkers.svg
 * \image latex CountBetweenMarkers.pdf
 *
 * Counts events on a single channel within the time indicated by a “start” and “stop” signals. The bin edges between
 * which counts are accumulated are determined by one or more hardware triggers. Specifically, the measurement records
 * data into a vector of length n_values (initially filled with zeros). It waits for tags on the begin_channel. When a
 * tag is detected on the begin_channel it starts counting tags on the click_channel. When the next tag is detected on
 * the begin_channel it stores the current counter value as the next entry in the data vector, resets the counter to
 * zero and starts accumulating counts again. If an end_channel is specified, the measurement stores the current counter
 * value and resets the counter when a tag is detected on the end_channel rather than the begin_channel. You can use
 * this, e.g., to accumulate counts within a gate by using rising edges on one channel as the begin_channel and falling
 * edges on the same channel as the end_channel. The accumulation time for each value can be accessed via
 * getBinWidths(). The measurement stops when all entries in the data vector are filled.
 */
class TT_API CountBetweenMarkers : public IteratorBase {
public:
  /**
   * \brief constructor of CountBetweenMarkers
   *
   *
   * @param tagger         reference to a TimeTagger
   * @param click_channel  channel that increases the count
   * @param begin_channel  channel that triggers beginning of counting and stepping to the next value
   * @param end_channel    channel that triggers end of counting
   * @param n_values       the number of counter values to be stored
   */
  CountBetweenMarkers(TimeTaggerBase *tagger, channel_t click_channel, channel_t begin_channel,
                      channel_t end_channel = CHANNEL_UNUSED, int32_t n_values = 1000);

  ~CountBetweenMarkers();

  /**
   * \brief Returns true when the entire array is filled.
   */
  bool ready();

  /**
   * \brief Returns array of size n_values containing the acquired counter values.
   */
  GET_DATA_1D(getData, int32_t, array_out, );

  /**
   * \brief fetches the widths of each bins
   */
  GET_DATA_1D(getBinWidths, timestamp_t, array_out, );

  /**
   * \brief fetches the starting time of each bin
   */
  GET_DATA_1D(getIndex, timestamp_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class CountBetweenMarkersImpl;
  std::unique_ptr<CountBetweenMarkersImpl> impl;
};

class CounterDataState;
class CounterImpl;
class Counter;
/**
 * \brief Helper object as return value for Counter::getDataObject.
 *
 * This object stores the result of up to n_values bins.
 */
class TT_API CounterData {
public:
  ~CounterData();

  /**
   * \brief get the amount of clicks (or 0 if overflow occurs) per bin and per channel
   *
   * Consider using getFrequency for explicit overflows.
   * Alternatively, you may check overflow field and/or call getOverflowMask function.
   */
  GET_DATA_2D(getData, int32_t, array_out, );

  /**
   * \brief get the counts normalized to the specified time scale
   *
   * Bins in overflow mode are marked as `NaN`. The parameter time_scale scales the return value to this time interval.
   * Default is 1 s, so the return value is in Hz. For negative values, the time scale is set to `binwidth`.
   */
  GET_DATA_2D_OP1(getFrequency, double, array_out, timestamp_t, time_scale, 1000000000000, );

  /**
   * \brief get the average rate of clicks per bin and per channel
   */
  GET_DATA_2D(getDataNormalized, double, array_out, );

  /**
   * \brief get the total amount of clicks per channel since the last clear up to the most rececnt bin
   */
  GET_DATA_1D(getDataTotalCounts, uint64_t, array_out, );

  /**
   * \brief get an index which corresponds to the timestamp of these bins
   */
  GET_DATA_1D(getIndex, timestamp_t, array_out, );

  /**
   * \brief get the timestamp of the bins since the last clear
   */
  GET_DATA_1D(getTime, timestamp_t, array_out, );

  /**
   * \brief get if the bins were in overflow
   */
  GET_DATA_1D(getOverflowMask, signed char, array_out, );

  /**
   * \brief get the configured list of channels
   */
  GET_DATA_1D(getChannels, channel_t, array_out, );

  /// number of returned bins
  const uint32_t size;
  /// number of bins which have been dropped because n_bins has been exceeded, usually 0
  const uint32_t dropped_bins;
  /// has anything been in overflow mode
  const bool overflow;

private:
  friend class CounterImpl;
  friend class Counter;

  CounterData(uint32_t size_, uint32_t dropped_bins_, bool overflow_, std::shared_ptr<CounterDataState> data_);

  const std::shared_ptr<CounterDataState> data;
};

/**
 * \ingroup EventCounting
 *
 * \brief a simple counter on one or more channels
 *
 * \image html Counter.svg
 * \image latex Counter.pdf
 *
 * Time trace of the count rate on one or more channels. Specifically, this measurement repeatedly counts tags on one or
 * more channels within a time interval binwidth and stores the results in a two-dimensional array of size ‘number of
 * channels’ by ‘n_values’. The array is treated as a circular buffer, which means all values in the array are shifted
 * by one position when a new value is generated. The last entry in the array is always the most recent value.
 */
class TT_API Counter : public IteratorBase {
public:
  /**
   * \brief construct a counter
   *
   * \param tagger       reference to a TimeTagger
   * \param channels     channels to count on
   * \param binwidth     counts are accumulated for binwidth picoseconds
   * \param n_values     number of counter values stored (for each channel)
   */
  Counter(TimeTaggerBase *tagger, std::vector<channel_t> channels, timestamp_t binwidth = 1000000000,
          int32_t n_values = 1);

  ~Counter();

  /**
   * \brief An array of size ‘number of channels’ by n_values containing the current values of the circular buffer
   * (counts in each bin).
   *
   * \param array_out allocator callback for managed return values
   * \param rolling if true, the returning array starts with the oldest data and goes up to the newest data
   */
  GET_DATA_2D_OP1(getData, int32_t, array_out, bool, rolling, true, );

  /**
   * \brief get countrate in Hz
   *
   * the counts are normalized are copied to a newly allocated allocated memory, an the
   * pointer to this location is returned.
   * Invalid bins are replaced with NaNs.
   *
   * \param array_out allocator callback for managed return values
   * \param rolling if true, the returning array starts with the oldest data and goes up to the newest data
   */
  GET_DATA_2D_OP1(getDataNormalized, double, array_out, bool, rolling, true, );

  /**
   * \brief get the total amount of clicks per channel since the last clear including the currently integrating bin
   */
  GET_DATA_1D(getDataTotalCounts, uint64_t, array_out, );

  /**
   * \brief A vector of size n_values containing the time bins in ps.
   */
  GET_DATA_1D(getIndex, timestamp_t, array_out, );

  /**
   * \brief Fetch the most recent up to n_values bins, which have not been removed before.
   *
   * This method allows atomic polling of bins, so each bin is guaranteed to be returned exactly once.
   *
   * \param remove remove all fetched bins
   * \returns a CounterData object, which contains all data of the fetches bins
   */
  CounterData getDataObject(bool remove = false);

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class CounterImpl;
  std::unique_ptr<CounterImpl> impl;
};

/**
 * \brief type of timestamp for the Coincidence virtual channel (Last, Average, First, ListedFirst)
 */
enum class CoincidenceTimestamp : uint32_t {
  Last = 0,        ///< time of the last event completing the coincidence (fastest option - default)
  Average = 1,     ///< average time of all tags completing the coincidence
  First = 2,       ///< time of the first event received of the coincidence
  ListedFirst = 3, ///< time of the first channel of the list with which the Coincidence was initialized
};

class CoincidencesImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief a coincidence monitor for many channel groups
 *
 * Monitor coincidences for given coincidence groups passed by the constructor.
 * A coincidence is hereby defined as for a given coincidence group
 * a) the incoming is part of this group
 * b) at least tag arrived within the coincidenceWindow [ps] for all other channels of this coincidence group
 * Each coincidence will create a virtual event.
 * The block of event IDs for those coincidence group can be fetched.
 */
class TT_API Coincidences : public IteratorBase {
public:
  /**
   * \brief construct a Coincidences
   *
   * \param tagger               reference to a TimeTagger
   * \param coincidenceGroups    a vector of channels defining the coincidences
   * \param coincidenceWindow    the size of the coincidence window in picoseconds
   * \param timestamp            type of timestamp for virtual channel (Last, Average, First, ListedFirst)
   */
  Coincidences(TimeTaggerBase *tagger, std::vector<std::vector<channel_t>> coincidenceGroups,
               timestamp_t coincidenceWindow, CoincidenceTimestamp timestamp = CoincidenceTimestamp::Last);

  ~Coincidences();

  /**
   * \brief fetches the block of virtual channels for those coincidence groups
   */
  std::vector<channel_t> getChannels();

  void setCoincidenceWindow(timestamp_t coincidenceWindow);

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

private:
  friend class CoincidencesImpl;
  std::unique_ptr<CoincidencesImpl> impl;
};

/**
 * \ingroup VirtualChannels
 *
 * \brief a coincidence monitor for one channel group
 *
 * \image html Coincidence.svg
 * \image latex Coincidence.pdf
 *
 * Monitor coincidences for a given channel groups passed by the constructor.
 * A coincidence is event is detected when all selected channels have a click
 * within the given coincidenceWindow [ps]
 * The coincidence will create a virtual events on a virtual channel with
 * the channel number provided by getChannel().
 * For multiple coincidence channel combinations use the class Coincidences
 * which outperformes multiple instances of Coincidence.
 */
class TT_API Coincidence : public Coincidences {
public:
  /**
   * \brief construct a coincidence
   *
   * \param tagger    reference to a TimeTagger
   * \param channels    vector of channels to match
   * \param coincidenceWindow    max distance between all clicks for a coincidence [ps]
   * \param timestamp  type of timestamp for virtual channel (Last, Average, First, ListedFirst)
   */
  Coincidence(TimeTaggerBase *tagger, std::vector<channel_t> channels, timestamp_t coincidenceWindow = 1000,
              CoincidenceTimestamp timestamp = CoincidenceTimestamp::Last)
      : Coincidences(tagger, {channels}, coincidenceWindow, timestamp) {}

  /**
   * \brief virtual channel which contains the coincidences
   */
  channel_t getChannel() { return getChannels()[0]; }
};

class CountrateImpl;
/**
 * \ingroup EventCounting
 * \brief count rate on one or more channels
 *
 * \image html Countrate.svg
 * \image latex Countrate.pdf
 *
 * Measures the average count rate on one or more channels. Specifically, it counts
 * incoming clicks and determines the time between the initial click and the latest click.
 * The number of clicks divided by the time corresponds to the average countrate
 * since the initial click.
 *
 */
class TT_API Countrate : public IteratorBase {
public:
  /**
   * \brief constructor of Countrate
   *
   * @param tagger    reference to a TimeTagger
   * @param channels    the channels to count on
   */
  Countrate(TimeTaggerBase *tagger, std::vector<channel_t> channels);

  ~Countrate();

  /**
   * \brief get the count rates
   *
   * Returns the average rate of events per second per channel as an array.
   */
  GET_DATA_1D(getData, double, array_out, );

  /**
   * \brief get the total amount of events
   *
   * Returns the total amount of events per channel as an array.
   */
  GET_DATA_1D(getCountsTotal, int64_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class CountrateImpl;
  std::unique_ptr<CountrateImpl> impl;
};

class DelayedChannelImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief a simple delayed queue
 *
 * \image html DelayedChannel.svg
 * \image latex DelayedChannel.pdf
 *
 * A simple first-in first-out queue of delayed event timestamps.
 */
class TT_API DelayedChannel : public IteratorBase {
public:
  /**
   * \brief constructor of a DelayedChannel
   *
   * \param tagger                reference to a TimeTagger
   * \param input_channel         channel which is delayed
   * \param delay                 amount of time to delay
   */
  DelayedChannel(TimeTaggerBase *tagger, channel_t input_channel, timestamp_t delay);

#ifndef SWIG
  /**
   * \brief constructor of a DelayedChannel for delaying many channels at once
   *
   * This function is not exposed to Python/C#/Matlab/Labview
   *
   * \param tagger                reference to a TimeTagger
   * \param input_channels        channels which will be delayed
   * \param delay                 amount of time to delay
   */
  DelayedChannel(TimeTaggerBase *tagger, std::vector<channel_t> input_channels, timestamp_t delay);
#endif

  ~DelayedChannel();

  /**
   * \brief the first new virtual channel
   *
   * This function returns the first of the new allocated virtual channels.
   * It can be used now in any new iterator.
   */
  channel_t getChannel();

#ifndef SWIG
  /**
   * \brief the new virtual channels
   *
   * This function returns the new allocated virtual channels.
   * It can be used now in any new iterator.
   */
  std::vector<channel_t> getChannels();
#endif

  /**
   * \brief set the delay time delay for the cloned tags in the virtual channels. A negative delay will delay all other
   * events.
   *
   * Note: When the delay is the same or greater than the previous value all incoming tags will be visible at virtual
   * channel. By applying a shorter delay time, the tags stored in the local buffer will be flushed and won't be visible
   * in the virtual channel.
   */
  void setDelay(timestamp_t delay);

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void on_start() override;

private:
  friend class DelayedChannelImpl;
  std::unique_ptr<DelayedChannelImpl> impl;
};

class TriggerOnCountrateImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief Inject trigger events when exceeding or falling below a given count rate within a rolling time window.
 *
 * \image html TriggerOnCountrate.svg
 * \image latex TriggerOnCountrate.pdf
 *
 * Measures the count rate inside a rolling time window and emits tags when a given reference_countrate is crossed.
 * A `TriggerOnCountrate` object provides two virtual channels: The `above` channel is triggered when the count rate
 * exceeds the threshold (transition from `below` to `above`). The `below` channel is triggered when the count rate
 * falls below the threshold (transition from `above` to `below`). To avoid the emission of multiple trigger tags in the
 * transition area, the `hysteresis` count rate modifies the threshold with respect to the transition direction: An
 * event in the `above` channel will be triggered when the channel is in the `below` state and rises to
 * `reference_countrate + hysteresis` or above. Vice versa, the `below` channel fires when the channel is in the `above`
 * state and falls to the limit of `reference_countrate - hysteresis` or below.
 *
 * The time-tags are always injected at the end of the integration window.
 * You can use the `DelayedChannel` to adjust the temporal position of the trigger tags with respect to the integration
 * time window.
 *
 * The very first tag of the virtual channel will be emitted `time_window` after the instantiation of the object and
 * will reflect the current state, so either `above` or `below`.
 */
class TT_API TriggerOnCountrate : public IteratorBase {
public:
  /**
   * \brief constructor of a TriggerOnCountrate
   *
   * \param tagger                Reference to a TimeTagger object.
   * \param input_channel         Channel number of the channel whose count rate will control the trigger channels.
   * \param reference_countrate   The reference count rate in Hz that separates the `above` range from the `below`
   * range.
   * \param hysteresis            The threshold count rate in Hz for transitioning to the `above` threshold state
   *                              is `countrate >= reference_countrate + hysteresis`, whereas it is `countrate <=
   *                              reference_countrate - hysteresis` for transitioning to the `below` threshold state.
   *                              The hysteresis avoids the emission of multiple trigger tags upon a single transition.
   * \param time_window           Rolling time window size in ps. The count rate is analyzed within this time window and
   *                              compared to the threshold count rate.
   */
  TriggerOnCountrate(TimeTaggerBase *tagger, channel_t input_channel, double reference_countrate, double hysteresis,
                     timestamp_t time_window);

  ~TriggerOnCountrate();

  /**
   * \brief Get the channel number of the `above` channel.
   */
  channel_t getChannelAbove();

  /**
   * \brief Get the channel number of the `below` channel.
   */
  channel_t getChannelBelow();

  /**
   * \brief Get both virtual channel numbers: [`getChannelAbove()`, `getChannelBelow()`]
   */
  std::vector<channel_t> getChannels();

  /**
   * \brief Returns whether the Virtual Channel is currently in the `above` state.
   */
  bool isAbove();

  /**
   * \brief Returns whether the Virtual Channel is currently in the `below` state.
   */
  bool isBelow();

  /**
   * \brief Get the current count rate averaged within the `time_window`.
   */
  double getCurrentCountrate();

  /**
   * \brief Emit a time-tag into the respective channel according to the current state.
   *
   * Emit a time-tag into the respective channel according to the current state.
   * This is useful if you start a new measurement that requires the information.
   * The function returns whether it was possible to inject the event.
   * The injection is not possible if the Time Tagger is in overflow mode or the time window has not passed yet.
   * The function call is non-blocking.
   */
  bool injectCurrentState();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void on_start() override;
  void clear_impl() override;

private:
  friend class TriggerOnCountrateImpl;
  std::unique_ptr<TriggerOnCountrateImpl> impl;
};

/**
 * \brief Initial state of the gate of a GatedChannel (Closed, Open)
 */
enum class GatedChannelInitial : uint32_t {
  Closed = 0, ///< the gate is closed initially (default)
  Open = 1,   ///< the gate is open initially
};

class GatedChannelImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief An input channel is gated by a gate channel.
 *
 * \image html GatedChannel.svg
 * \image latex GatedChannel.pdf
 *
 * Note: The gate is edge sensitive and not level sensitive. That means that the gate will transfer data only when an
 * appropriate level change is detected on the gate_start_channel.
 */
class TT_API GatedChannel : public IteratorBase {
public:
  /**
   * \brief constructor of a GatedChannel
   *
   * \param tagger                reference to a TimeTagger
   * \param input_channel         channel which is gated
   * \param gate_start_channel    channel on which a signal detected will start the transmission of the input_channel
   * through the gate \param gate_stop_channel     channel on which a signal detected will stop the transmission of the
   * input_channel through the gate
   * \param initial               initial state of the gate
   */
  GatedChannel(TimeTaggerBase *tagger, channel_t input_channel, channel_t gate_start_channel,
               channel_t gate_stop_channel, GatedChannelInitial initial = GatedChannelInitial::Closed);

  ~GatedChannel();

  /**
   * \brief the new virtual channel
   *
   * This function returns the new allocated virtual channel.
   * It can be used now in any new iterator.
   *
   */
  channel_t getChannel();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class GatedChannelImpl;
  std::unique_ptr<GatedChannelImpl> impl;
};

class FrequencyMultiplierImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief The signal of an input channel is scaled up to a higher frequency according to the multiplier passed as a
 * parameter.
 *
 * \image html FrequencyMultiplier.svg
 * \image latex FrequencyMultiplier.pdf
 *
 * The FrequencyMultiplier inserts copies the original input events from the input_channel and adds additional events to
 * match the upscaling factor. The algorithm used assumes a constant frequency and calculates out of the last two
 * incoming events linearly the intermediate timestamps to match the upscaled frequency given by the multiplier
 * parameter.
 *
 * The FrequencyMultiplier can be used to restore the actual frequency applied to an input_channel which was reduces via
 * the EventDivider to lower the effective data rate. For example a 80 MHz laser sync signal can be scaled down via
 * setEventDivider(..., 80) to 1 MHz (hardware side) and an 80 MHz signal can be restored via FrequencyMultiplier(...,
 * 80) on the software side with some loss in precision. The FrequencyMultiplier is an alternative way to reduce the
 * data rate in comparison to the EventFilter, which has a higher precision but can be more difficult to use.
 */
class TT_API FrequencyMultiplier : public IteratorBase {
public:
  /**
   * \brief constructor of a FrequencyMultiplier
   *
   * \param tagger                reference to a TimeTagger
   * \param input_channel         channel on which the upscaling of the frequency is based on
   * \param multiplier            frequency upscaling factor
   */
  FrequencyMultiplier(TimeTaggerBase *tagger, channel_t input_channel, int32_t multiplier);

  ~FrequencyMultiplier();

  channel_t getChannel();
  int32_t getMultiplier();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class FrequencyMultiplierImpl;
  std::unique_ptr<FrequencyMultiplierImpl> impl;
};

class IteratorImpl;
/**
 * \ingroup TimeTagStreaming
 *
 * \brief a deprecated simple event queue
 *
 * A simple Iterator, just keeping a first-in first-out queue of event timestamps.
 *
 * \deprecated use TimeTagStream
 */
class TT_API Iterator : public IteratorBase {
public:
  /**
   * \brief standard constructor
   *
   * @param tagger        the backend
   * @param channel       the channel to get events from
   */
  Iterator(TimeTaggerBase *tagger, channel_t channel);

  ~Iterator();

  /**
   * \brief get next timestamp
   *
   * get the next timestamp from the queue.
   */
  timestamp_t next();

  /**
   * \brief get queue size
   */
  uint64_t size();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class IteratorImpl;
  std::unique_ptr<IteratorImpl> impl;
};

class TimeTagStreamImpl;
class FileReaderImpl;
/// return object for TimeTagStream::getData
class TT_API TimeTagStreamBuffer {
  friend class TimeTagStreamImpl;
  friend class FileReaderImpl;

public:
  ~TimeTagStreamBuffer();

  GET_DATA_1D(getOverflows, unsigned char, array_out, ); // deprecated, please use getEventTypes instead
  GET_DATA_1D(getChannels, channel_t, array_out, );
  GET_DATA_1D(getTimestamps, timestamp_t, array_out, );
  GET_DATA_1D(getMissedEvents, unsigned short, array_out, );
  GET_DATA_1D(getEventTypes, unsigned char, array_out, );

  uint64_t size;
  bool hasOverflows;
  timestamp_t tStart;
  timestamp_t tGetData;

private:
  TimeTagStreamBuffer();

  std::vector<channel_t> tagChannels;
  std::vector<timestamp_t> tagTimestamps;
  std::vector<unsigned short> tagMissedEvents;
  std::vector<Tag::Type> tagTypes;
};

/**
 * \ingroup TimeTagStreaming
 * \brief access the time tag stream
 */
class TT_API TimeTagStream : public IteratorBase {
public:
  /**
   * \brief constructor of a TimeTagStream thread
   *
   * Gives access to the time tag stream
   *
   * @param tagger      reference to a TimeTagger
   * @param n_max_events    maximum number of tags stored
   * @param channels      channels which are dumped to the file
   */
  TimeTagStream(TimeTaggerBase *tagger, uint64_t n_max_events, std::vector<channel_t> channels);
  ~TimeTagStream();

  /**
   * \brief return the number of stored tags
   */
  uint64_t getCounts();

  /**
   * \brief fetches all stored tags and clears the internal state
   */
  TimeTagStreamBuffer getData();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class TimeTagStreamImpl;
  std::unique_ptr<TimeTagStreamImpl> impl;
};

class DumpImpl;
/**
 * \ingroup TimeTagStreaming
 * \brief dump all time tags to a file
 *
 * @deprecated use FileWriter
 */
class TT_API Dump : public IteratorBase {
public:
  /**
   * \brief constructor of a Dump thread
   *
   * @param tagger        reference to a TimeTagger
   * @param filename      name of the file to dump to, must be encoded as UTF-8
   * @param max_tags      stop after this number of tags has been dumped. Negative values will dump forever
   * @param channels      channels which are dumped to the file (when empty or not passed all active channels are
   * dumped)
   */
  Dump(TimeTaggerBase *tagger, std::string filename, int64_t max_tags,
       std::vector<channel_t> channels = std::vector<channel_t>());
  ~Dump();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;
  void on_stop() override;

private:
  friend class DumpImpl;
  std::unique_ptr<DumpImpl> impl;
};

class StartStopImpl;
/**
 * @ingroup TimeHistograms
 *
 * \brief simple start-stop measurement
 *
 * \image html StartStop.svg
 * \image latex StartStop.pdf
 *
 * This class performs a start-stop measurement between two channels
 * and stores the time differences in a histogram. The histogram resolution
 * is specified beforehand (binwidth) but the histogram range is unlimited.
 * It is adapted to the largest time difference that was detected. Thus
 * all pairs of subsequent clicks are registered.
 *
 * Be aware, on long-running measurements this may considerably slow down
 * system performance and even crash the system entirely when attached to an
 * unsuitable signal source.
 *
 */
class TT_API StartStop : public IteratorBase {
public:
  /**
   * \brief constructor of StartStop
   *
   * @param tagger                reference to a TimeTagger
   * @param click_channel         channel for stop clicks
   * @param start_channel         channel for start clicks
   * @param binwidth              width of one histogram bin in ps
   */
  StartStop(TimeTaggerBase *tagger, channel_t click_channel, channel_t start_channel = CHANNEL_UNUSED,
            timestamp_t binwidth = 1000);

  ~StartStop();

  GET_DATA_2D(getData, timestamp_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class StartStopImpl;
  std::unique_ptr<StartStopImpl> impl;
};

template <typename T> class TimeDifferencesImpl;
/**
 * @ingroup TimeHistograms
 * \brief Accumulates the time differences between clicks on two channels in one or more histograms.
 *
 * \image html TimeDifferences.svg
 * \image latex TimeDifferences.pdf
 *
 * A multidimensional histogram measurement with the option up to include three additional channels
 * that control how to step through the indices of the histogram array. This is a very powerful and
 * generic measurement. You can use it to record cross-correlation, lifetime measurements, fluorescence
 * lifetime imaging and many more measurements based on pulsed excitation. Specifically, the measurement
 * waits for a tag on the ‘start_channel’, then measures the time difference between the start tag and
 * all subsequent tags on the ‘click_channel’ and stores them in a histogram. If no ‘start_channel’ is
 * specified, the ‘click_channel’ is used as ‘start_channel’ corresponding to an auto-correlation
 * measurement. The histogram has a number ‘n_bins’ of bins of bin width ‘binwidth’. Clicks that fall
 * outside the histogram range are discarded. Data accumulation is performed independently for all start
 * tags. This type of measurement is frequently referred to as ‘multiple start, multiple stop’
 * measurement and corresponds to a full auto- or cross-correlation measurement.
 *
 * The data obtained from subsequent start tags can be accumulated into the same histogram (one-
 * dimensional measurement) or into different histograms (two-dimensional measurement). In this way, you
 * can perform more general two-dimensional time-difference measurements. The parameter ‘n_histograms’
 * specifies the number of histograms. After each tag on the ‘next_channel’, the histogram index is
 * incremented by one and reset to zero after reaching the last valid index. The measurement starts with
 * the first tag on the ‘next_channel’.
 *
 * You can also provide a synchronization trigger that resets the histogram index by specifying a
 * ‘sync_channel’. The measurement starts when a tag on the ‘sync_channel’ arrives with a subsequent tag
 * on ‘next_channel’. When a rollover occurs, the accumulation is stopped until the next sync and
 * subsequent next signal. A sync signal before a rollover will stop the accumulation, reset the
 * histogram index and a subsequent signal on the ‘next_channel’ starts the accumulation again.
 *
 * Typically, you will run the measurement indefinitely until stopped by the user. However, it is also
 * possible to specify the maximum number of rollovers of the histogram index. In this case the
 * measurement stops when the number of rollovers has reached the specified value. This means that for
 * both a one-dimensional and for a two-dimensional measurement, it will measure until the measurement
 * went through the specified number of rollovers / sync tags.
 */
class TT_API TimeDifferences : public IteratorBase {
public:
  /**
   * \brief constructor of a TimeDifferences measurement
   *
   *
   * \param tagger                reference to a TimeTagger
   * \param click_channel         channel that increments the count in a bin
   * \param start_channel         channel that sets start times relative to which clicks on the click channel are
   * measured \param next_channel          channel that increments the histogram index \param sync_channel channel that
   * resets the histogram index to zero \param binwidth              width of one histogram bin in ps \param n_bins
   * number of bins in each histogram \param n_histograms          number of histograms
   */
  TimeDifferences(TimeTaggerBase *tagger, channel_t click_channel, channel_t start_channel = CHANNEL_UNUSED,
                  channel_t next_channel = CHANNEL_UNUSED, channel_t sync_channel = CHANNEL_UNUSED,
                  timestamp_t binwidth = 1000, int32_t n_bins = 1000, int32_t n_histograms = 1);

  ~TimeDifferences();

  /**
   * \brief returns a two-dimensional array of size ‘n_bins’ by ‘n_histograms’ containing the histograms
   */
  GET_DATA_2D(getData, int32_t, array_out, );

  /**
   * \brief returns a vector of size ‘n_bins’ containing the time bins in ps
   */
  GET_DATA_1D(getIndex, timestamp_t, array_out, );

  /**
   * \brief set the number of rollovers at which the measurement stops integrating
   *
   * \param max_counts             maximum number of sync/next clicks
   */
  void setMaxCounts(uint64_t max_counts);

  /**
   * \brief returns the number of rollovers (histogram index resets)
   */
  uint64_t getCounts();

  /**
   * \brief The index of the currently processed histogram or the waiting state.
   *
   * Possible return values are:
   * `-2`: Waiting for an event on `sync_channel` (only if `sync_channel` is defined)
   * `-1`: Waiting for an event on `next_channel` (only if `sync_channel` is defined)
   * `0` ... `(n_histograms - 1)`: Index of the currently processed histogram
   */
  int32_t getHistogramIndex() const;

  /**
   * \brief returns ‘true’ when the required number of rollovers set by ‘setMaxCounts’ has been reached
   */
  bool ready();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class TimeDifferencesImpl<TimeDifferences>;
  std::unique_ptr<TimeDifferencesImpl<TimeDifferences>> impl;
};

template <typename T> class HistogramNDImpl;
/**
 * @ingroup TimeHistograms
 * \brief A 2-dimensional histogram of time differences. This can be used in measurements similar to 2D NRM
 * spectroscopy.
 *
 * \image html Histogram2D.svg
 * \image latex Histogram2D.pdf
 *
 * This measurement is a 2-dimensional version of the Histogram measurement.
 * The measurement accumulates two-dimensional histogram where stop signals from two
 * separate channels define the bin coordinate. For instance, this kind of measurement
 * is similar to that of typical 2D NMR spectroscopy.
 */
class TT_API Histogram2D : public IteratorBase {
public:
  /**
   * \brief constructor of a Histogram2D measurement
   *
   * \param tagger time tagger object
   * \param start_channel channel on which start clicks are received
   * \param stop_channel_1 channel on which stop clicks for the time axis 1 are received
   * \param stop_channel_2 channel on which stop clicks for the time axis 2 are received
   * \param binwidth_1 bin width in ps for the time axis 1
   * \param binwidth_2 bin width in ps for the time axis 2
   * \param n_bins_1 the number of bins along the time axis 1
   * \param n_bins_2 the number of bins along the time axis 2
   */
  Histogram2D(TimeTaggerBase *tagger, channel_t start_channel, channel_t stop_channel_1, channel_t stop_channel_2,
              timestamp_t binwidth_1, timestamp_t binwidth_2, int32_t n_bins_1, int32_t n_bins_2);
  ~Histogram2D();

  /**
   * Returns a two-dimensional array of size n_bins_1 by n_bins_2 containing the 2D histogram.
   */
  GET_DATA_2D(getData, int32_t, array_out, );

  /**
   * Returns a 3D array containing two coordinate matrices (meshgrid) for time bins in ps for the time axes 1 and 2.
   * For details on meshgrid please take a look at the respective documentation either for Matlab or Python NumPy
   */
  GET_DATA_3D(getIndex, timestamp_t, array_out, );

  /**
   * Returns a vector of size n_bins_1 containing the bin locations in ps for the time axis 1.
   */
  GET_DATA_1D(getIndex_1, timestamp_t, array_out, );

  /**
   * Returns a vector of size `n_bins_2` containing the bin locations in ps for the time axis 2.
   */
  GET_DATA_1D(getIndex_2, timestamp_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class HistogramNDImpl<Histogram2D>;
  std::unique_ptr<HistogramNDImpl<Histogram2D>> impl;
};

/**
 * @ingroup ITERATOR
 * \brief A N-dimensional histogram of time differences. This can be used in measurements similar to 2D NRM
 * spectroscopy.
 *
 * This measurement is a N-dimensional version of the Histogram measurement.
 * The measurement accumulates N-dimensional histogram where stop signals from N
 * separate channels define the bin coordinate. For instance, this kind of measurement
 * is similar to that of typical 2D NMR spectroscopy.
 */
class TT_API HistogramND : public IteratorBase {
public:
  /**
   * \brief constructor of a Histogram2D measurement
   * \param tagger time tagger object
   * \param start_channel channel on which start clicks are received
   * \param stop_channels channels on which stop clicks for each time axis are received
   * \param binwidths bin widths in ps for each time axis
   * \param n_bins the number of bins along each time axis
   */
  HistogramND(TimeTaggerBase *tagger, channel_t start_channel, std::vector<channel_t> stop_channels,
              std::vector<timestamp_t> binwidths, std::vector<int32_t> n_bins);
  ~HistogramND();

  /**
   * Returns a one-dimensional array of size of the product of n_bins containing the N-dimensional histogram.
   * The 1D return value is in row-major ordering like on C, Python, C#. This conflicts with Fortran or Matlab.
   * Please reshape the result to get the N-dimensional array.
   */
  GET_DATA_1D(getData, int32_t, array_out, );

  /**
   * Returns a vector of size n_bins[dim] containing the bin locations in ps for the corresponding time axis.
   */
  GET_DATA_1D_OP1(getIndex, timestamp_t, array_out, int32_t, dim, 0, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class HistogramNDImpl<HistogramND>;
  std::unique_ptr<HistogramNDImpl<HistogramND>> impl;
};

class TimeDifferencesNDImpl;
/**
 * @ingroup TimeHistograms
 * \brief Accumulates the time differences between clicks on two channels in a multi-dimensional histogram.
 *
 * \image html TimeDifferencesND.svg
 * \image latex TimeDifferencesND.pdf
 *
 * This is a multidimensional implementation of the TimeDifferences measurement class. Please read their documentation
 * first.
 *
 * This measurement class extends the TimeDifferences interface for a multidimensional amount of histograms.
 * It captures many multiple start - multiple stop histograms, but with many asynchronous next_channel triggers.
 * After each tag on each next_channel, the histogram index of the associated dimension is
 * incremented by one and reset to zero after reaching the last valid index.  The elements of the parameter n_histograms
 * specifies the number of histograms per dimension. The accumulation starts when next_channel has been triggered on all
 * dimensions.
 *
 * You should provide a synchronization trigger by specifying a sync_channel per dimension.
 * It will stop the accumulation when an associated histogram index rollover occurs.
 * A sync event will also stop the accumulation, reset the histogram index of the associated dimension,
 * and a subsequent event on the corresponding next_channel starts the accumulation again.
 * The synchronization is done asynchronous, so an event on the next_channel increases the histogram index even if the
 * accumulation is stopped. The accumulation starts when a tag on the sync_channel arrives with a subsequent tag on
 * next_channel for all dimensions.
 *
 * Please use setInputDelay to adjust the latency of all channels. In general, the order of the provided triggers
 * including maximum jitter should be: old start trigger -- all sync triggers -- all next triggers -- new start trigger
 */
class TT_API TimeDifferencesND : public IteratorBase {
public:
  /**
   * \brief constructor of a TimeDifferencesND measurement
   *
   *
   * \param tagger                reference to a TimeTagger
   * \param click_channel         channel that increments the count in a bin
   * \param start_channel         channel that sets start times relative to which clicks on the click channel are
   *                              measured
   * \param next_channels         vector of channels that increments the histogram index \param sync_channels
   *                              vector of channels that resets the histogram index to zero
   * \param n_histograms          vector of numbers of histograms per dimension.
   * \param binwidth              width of one histogram bin in ps \param n_bins number of bins
   *                              in each histogram
   */
  TimeDifferencesND(TimeTaggerBase *tagger, channel_t click_channel, channel_t start_channel,
                    std::vector<channel_t> next_channels, std::vector<channel_t> sync_channels,
                    std::vector<int32_t> n_histograms, timestamp_t binwidth, int32_t n_bins);

  ~TimeDifferencesND();

  /**
   * \brief returns a two-dimensional array of size n_bins by all n_histograms containing the histograms
   */
  GET_DATA_2D(getData, int32_t, array_out, );

  /**
   * \brief returns a vector of size n_bins containing the time bins in ps
   */
  GET_DATA_1D(getIndex, timestamp_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class TimeDifferencesNDImpl;
  std::unique_ptr<TimeDifferencesNDImpl> impl;
};

/**
 * @ingroup TimeHistograms
 *
 * \brief Accumulate time differences into a histogram
 *
 * \image html Histogram.svg
 * \image latex Histogram.pdf
 *
 * This is a simple multiple start, multiple stop measurement. This is a special
 * case of the more general 'TimeDifferences' measurement.
 *    Specifically, the thread waits for clicks on a first channel, the 'start channel',
 * then measures the time difference between the last start click and all subsequent
 * clicks on a second channel, the 'click channel', and stores them in a histogram.
 * The histogram range and resolution is specified by the number of bins and the binwidth.
 * Clicks that fall outside the histogram range are ignored.
 * Data accumulation is performed independently for all start clicks. This type of measurement
 * is frequently referred to as 'multiple start, multiple stop' measurement and corresponds to a
 * full auto- or cross-correlation measurement.
 */
class TT_API Histogram : public IteratorBase {
public:
  /**
   * \brief constructor of a Histogram measurement
   *
   *
   * \param tagger                reference to a TimeTagger
   * \param click_channel         channel that increments the count in a bin
   * \param start_channel         channel that sets start times relative to which clicks on the click channel are
   * measured \param binwidth              width of one histogram bin in ps \param n_bins                number of bins
   * in the histogram
   */
  Histogram(TimeTaggerBase *tagger, channel_t click_channel, channel_t start_channel = CHANNEL_UNUSED,
            timestamp_t binwidth = 1000, int32_t n_bins = 1000);

  ~Histogram();

  GET_DATA_1D(getData, int32_t, array_out, );

  GET_DATA_1D(getIndex, timestamp_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class TimeDifferencesImpl<Histogram>;
  std::unique_ptr<TimeDifferencesImpl<Histogram>> impl;
};

struct FrequencyCounterDataImpl;

class TT_API FrequencyCounterData {
public:
  ~FrequencyCounterData();

  /**
   * \brief Returns the index of each sampling point
   */
  GET_DATA_1D(getIndex, timestamp_t, array_out, );

  /**
   * \brief get the timestamp of the bins since the last clear
   */
  GET_DATA_1D(getTime, timestamp_t, array_out, );

  /**
   * \brief Returns an overflow mask with 1 = "has overflow" and 0 = "is valid".
   */
  GET_DATA_2D(getOverflowMask, signed char, array_out, );

  /**
   * \brief Integer part of the absolute phase.
   */
  GET_DATA_2D(getPeriodsCount, timestamp_t, array_out, );

  /**
   * \brief Fraction of the current cycle as a value from [0, 1).
   */
  GET_DATA_2D(getPeriodsFraction, double, array_out, );

  /**
   * \brief Frequency of the previous sampling period calculated from the phase difference between the current and the
   * previous sample.
   */
  GET_DATA_2D_OP1(getFrequency, double, array_out, timestamp_t, time_scale, 1000000000000, );

  /**
   * \brief Instantaneous frequency within the fitting window obtained from the fit slope.
   */
  GET_DATA_2D(getFrequencyInstantaneous, double, array_out, );

  /**
   * \brief Phase with respect to an expected reference frequency.
   */
  GET_DATA_2D_OP1(getPhase, double, array_out, double, reference_frequency, 0, );

  /// \brief Number of sampling points affected by an overflow range since the start of the measurement.
  const timestamp_t overflow_samples;

  /// \brief Number of sampling points represented by the object.
  const unsigned int size;

  /// \brief Indicates if the sampling grid has been aligned to the SoftwareClock.
  const bool align_to_reference;

  /// \brief The sampling interval in picoseconds.
  const timestamp_t sampling_interval;

  /// \brief Index offset of the first index
  const timestamp_t sample_offset;

  /// \brief Channels as last dimension
  const bool channels_last_dim;

private:
  FrequencyCounterData(timestamp_t overflow_samples, unsigned int size, bool align_to_reference,
                       timestamp_t sampling_interval, timestamp_t index_offset, bool channels_last_dim);
  friend class FrequencyCounter;

  std::shared_ptr<FrequencyCounterDataImpl> data;
};

class FrequencyCounterImpl;

/**
 * @ingroup PhaseAndFrequency
 *
 * \brief Calculate the phase of multiple channels at equidistant sampling points.
 *
 * This measurement calculates the phase of a periodic signal at evenly spaced sampling times. Multiple channels
 * can be analyzed in parallel to compare the phase evolution in time. Around every sampling time, the time tags within
 * an adjustable `fitting_window` are used to fit the phase.
 *
 */
class TT_API FrequencyCounter : public IteratorBase {
public:
  FrequencyCounter(TimeTaggerBase *tagger, std::vector<channel_t> channels, timestamp_t sampling_interval,
                   timestamp_t fitting_window, int32_t n_values = 0);
  ~FrequencyCounter();
  FrequencyCounterData getDataObject(uint16_t event_divider = 1, bool remove = false, bool channels_last_dim = false);

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class FrequencyCounterImpl;
  std::unique_ptr<FrequencyCounterImpl> impl;
};

class HistogramLogBinsImpl;

struct HistogramLogBinsDataImpl;

/**
 * \brief Helper object as return value for HistogramLogBins::getDataObject.
 *
 * This object stores counts and normalization of the logarithmic bins.
 */
class TT_API HistogramLogBinsData {
public:
  ~HistogramLogBinsData();

  /**
   * \brief Get the amount of clicks per bin and per channel
   */
  GET_DATA_1D(getCounts, uint64_t, array_out, );

  /**
   * \brief Get the calculated normalization for each bin
   */
  GET_DATA_1D(getG2Normalization, double, array_out, );

  /**
   * \brief Get the normalized histogram
   */
  GET_DATA_1D(getG2, double, array_out, );

  const timestamp_t accumulation_time_start;
  const timestamp_t accumulation_time_click;

private:
  HistogramLogBinsData(timestamp_t accumulation_time_start, timestamp_t accumulation_time_click);
  friend HistogramLogBinsImpl;

  std::shared_ptr<HistogramLogBinsDataImpl> data;
};

struct TT_API ChannelGate {
  ChannelGate(channel_t gate_open_channel, channel_t gate_close_channel,
              GatedChannelInitial initial = GatedChannelInitial::Open)
      : gate_open_channel(gate_open_channel), gate_close_channel(gate_close_channel), initial(initial){};
  const channel_t gate_open_channel;
  const channel_t gate_close_channel;
  const GatedChannelInitial initial;
};

/**
 * @ingroup TimeHistograms
 *
 * \brief Accumulate time differences into a histogram with logarithmic increasing bin sizes
 *
 * \image html HistogramLogBins.svg
 * \image latex HistogramLogBins.pdf
 *
 * This is a multiple start, multiple stop measurement, and works the very same way as the histogram measurement but
 * with logarithmic increasing bin widths. After initializing the measurement (or after an overflow) no data is
 * accumulated in the histogram until the full histogram duration has passed to ensure a balanced count accumulation
 * over the full histogram.
 *
 */
class TT_API HistogramLogBins : public IteratorBase {
public:
  /**
   * \brief constructor of a HistogramLogBins measurement
   *
   * \param tagger                reference to a TimeTagger
   * \param click_channel         channel that increments the count in a bin
   * \param start_channel         channel that sets start times relative to which clicks on the click channel are
   *                              measured
   * \param exp_start             exponent for the lowest time differences in the histogram: 10^exp_start s,
   *                              lowest exp_start: -12 => 1ps
   * \param exp_stop              exponent for the highest time differences in the histogram: 10^exp_stop s
   * \param n_bins                total number of bins in the histogram
   * \param click_gate            ChannelGate object for toggling the click_channel, nullptr if unused
   * \param start_gate            ChannelGate object for toggling the start_channel, nullptr if unused
   */
  HistogramLogBins(TimeTaggerBase *tagger, channel_t click_channel, channel_t start_channel, double exp_start,
                   double exp_stop, int32_t n_bins, const ChannelGate *click_gate = nullptr,
                   const ChannelGate *start_gate = nullptr);
  ~HistogramLogBins();

  HistogramLogBinsData getDataObject();

  /**
   * \brief returns the absolute counts for the bins
   */
  GET_DATA_1D(getData, uint64_t, array_out, );

  /**
   * \brief returns the counts normalized by the binwidth of each bin
   */
  GET_DATA_1D(getDataNormalizedCountsPerPs, double, array_out, );

  /**
   * \brief returns the counts normalized by the binwidth and the average count rate.
   *
   * This matches the implementation of Correlation::getDataNormalized
   */
  GET_DATA_1D(getDataNormalizedG2, double, array_out, );

  /**
   * \brief returns the edges of the bins in ps
   */
  GET_DATA_1D(getBinEdges, timestamp_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class HistogramLogBinsImpl;
  std::unique_ptr<HistogramLogBinsImpl> impl;
};

class CorrelationImpl;
/**
 * @ingroup TimeHistograms
 *
 * \brief Auto- and Cross-correlation measurement.
 *
 * \image html Correlation.svg
 * \image latex Correlation.pdf
 *
 * Accumulates time differences between clicks on two channels into a histogram, where all clicks are considered both as
 * “start” and “stop” clicks and both positive and negative time differences are calculated.
 *
 */
class TT_API Correlation : public IteratorBase {
public:
  /**
   * \brief constructor of a correlation measurement
   *
   * \note When channel_1 is left empty or set to CHANNEL_UNUSED -> an auto-correlation measurement is performed, which
   * is the same as setting channel_1 = channel_2.
   *
   * \param tagger                time tagger object
   * \param channel_1             channel on which (stop) clicks are received
   * \param channel_2             channel on which reference clicks (start) are received
   * \param binwidth              bin width in ps
   * \param n_bins                the number of bins in the resulting histogram
   */
  Correlation(TimeTaggerBase *tagger, channel_t channel_1, channel_t channel_2 = CHANNEL_UNUSED,
              timestamp_t binwidth = 1000, int n_bins = 1000);

  /// destructor of the Correlation measurement
  ~Correlation();

  /**
   * \brief returns a one-dimensional array of size n_bins containing the histogram
   *
   * \param array_out allocator callback for managed return values
   */
  GET_DATA_1D(getData, int32_t, array_out, );

  /**
   * \brief get the g(2) normalized histogram
   *
   * Return the data normalized as:
   * \f$g^{(2)}(\tau) = \frac{\Delta{t}}{binwidth(\tau) \cdot N_1 \cdot N_2} \cdot histogram(\tau)\f$
   *
   * This is normalized in such a way that a perfectly uncorrelated signals would result in a histogram with a mean
   * value of bins equal to one.
   *
   * \param array_out allocator callback for managed return values
   */
  GET_DATA_1D(getDataNormalized, double, array_out, );

  /**
   * \brief returns a vector of size n_bins containing the time bins in ps
   *
   * \param array_out allocator callback for managed return values
   */
  GET_DATA_1D(getIndex, timestamp_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class CorrelationImpl;
  std::unique_ptr<CorrelationImpl> impl;
};

/// Input state in the return object of Scope
enum State {
  UNKNOWN,
  HIGH,
  LOW,
};
/// Object for the return value of Scope::getData
struct Event {
  timestamp_t time;
  State state;
};
class ScopeImpl;
/**
 * @ingroup TimeTagStreaming
 * \brief a scope measurement
 *
 * \image html Scope.svg
 * \image latex Scope.pdf
 *
 * The Scope class allows to visualize time tags for rising and falling edges in a time trace diagram similarly to an
 * ultrafast logic analyzer. The trace recording is synchronized to a trigger signal which can be any physical or
 * virtual channel. However, only physical channels can be specified to the event_channels parameter. Additionally, one
 * has to specify the time window_size which is the timetrace duration to be recorded, the number of traces to be
 * recorded and the maximum number of events to be detected. If n_traces < 1 then retriggering will occur infinitely,
 * which is similar to the “normal” mode of an oscilloscope.
 */
class TT_API Scope : public IteratorBase {
public:
  /**
   * \brief constructor of a Scope measurement
   *
   * \param tagger                reference to a TimeTagger
   * \param event_channels        channels which are captured
   * \param trigger_channel       channel that starts a new trace
   * \param window_size           window time of each trace
   * \param n_traces              amount of traces (n_traces < 1, automatic retrigger)
   * \param n_max_events          maximum number of tags in each trace
   */
  Scope(TimeTaggerBase *tagger, std::vector<channel_t> event_channels, channel_t trigger_channel,
        timestamp_t window_size = 1000000000, int32_t n_traces = 1, int32_t n_max_events = 1000);

  ~Scope();

  bool ready();

  int32_t triggered();

  std::vector<std::vector<Event>> getData();

  timestamp_t getWindowSize();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class ScopeImpl;
  std::unique_ptr<ScopeImpl> impl;
};

class TimeTaggerProxy;
/**
 * \ingroup HelperClasses
 * \brief start, stop and clear several measurements synchronized
 *
 * For the case that several measurements should be started, stopped or cleared at the very same time,
 * a SynchronizedMeasurements object can be create to which all the measurements (also called iterators)
 * can be registered with .registerMeasurement(measurement).
 * Calling .stop(), .start() or .clear() on the SynchronizedMeasurements object will call the respective
 * method on each of the registered measurements at the very same time. That means that all measurements
 * taking part will have processed the very same time tags.
 *
 **/
class TT_API SynchronizedMeasurements {
public:
  /**
   * \brief construct a SynchronizedMeasurements object
   *
   * \param tagger reference to a TimeTagger
   */
  SynchronizedMeasurements(TimeTaggerBase *tagger);

  ~SynchronizedMeasurements();

  /**
   * \brief register a measurement (iterator) to the SynchronizedMeasurements-group.
   *
   * All available methods called on the SynchronizedMeasurements will happen at the very same time for all the
   * registered measurements.
   */
  void registerMeasurement(IteratorBase *measurement);

  /**
   * \brief unregister a measurement (iterator) from the SynchronizedMeasurements-group.
   *
   * Stops synchronizing calls on the selected measurement, if the measurement is not within
   * this synchronized group, the method does nothing.
   */
  void unregisterMeasurement(IteratorBase *measurement);

  /**
   * \brief clear all registered measurements synchronously
   */
  void clear();

  /**
   * \brief start all registered measurements synchronously
   */
  void start();

  /**
   * \brief stop all registered measurements synchronously
   */
  void stop();

  /**
   * \brief start all registered measurements synchronously, and stops them after the capture_duration
   */
  void startFor(timestamp_t capture_duration, bool clear = true);

  /**
   * \brief wait until all registered measurements have finished running.
   *
   * \param timeout     time in milliseconds to wait for the measurements. If negative, wait until finished.
   *
   * waitUntilFinished will wait according to the timeout and return true if all measurements finished or false if not.
   * Furthermore, when waitUntilFinished is called on a set running indefinitely, it will log an error and return
   * immediately.
   */
  bool waitUntilFinished(int64_t timeout = -1);

  /**
   * \brief check if any iterator is running
   */
  bool isRunning();

  /**
   * \brief Returns a proxy tagger object, which shall be used to create immediately registered measurements.
   *
   * Those measurements will not start automatically.
   */
  TimeTaggerBase *getTagger();

protected:
  /**
   * \brief run a callback on all registered measurements synchronously
   *
   * Please keep in mind that the callback is copied for each measurement.
   * So please avoid big captures.
   */
  void runCallback(TimeTaggerBase::IteratorCallback callback, std::unique_lock<std::mutex> &lk, bool block = true);

private:
  friend class TimeTaggerProxy;

  void release();

  std::set<IteratorBase *> registered_measurements;
  std::mutex measurements_mutex;
  TimeTaggerBase *tagger;
  bool has_been_released = false;
  std::unique_ptr<TimeTaggerProxy> proxy;
};

class ConstantFractionDiscriminatorImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief a virtual CFD implementation which returns the mean time between a rising and a falling pair of edges
 *
 * \image html ConstantFractionDiscriminator.svg
 * \image latex ConstantFractionDiscriminator.pdf
 *
 */
class TT_API ConstantFractionDiscriminator : public IteratorBase {
public:
  /**
   * \brief constructor of a ConstantFractionDiscriminator
   *
   * \param tagger                reference to a TimeTagger
   * \param channels              list of channels for the CFD, the formers of the rising+falling pairs must be given
   * \param search_window         interval for the CFD window, must be positive
   */
  ConstantFractionDiscriminator(TimeTaggerBase *tagger, std::vector<channel_t> channels, timestamp_t search_window);

  ~ConstantFractionDiscriminator();

  /**
   * \brief the list of new virtual channels
   *
   * This function returns the list of new allocated virtual channels.
   * It can be used now in any new measurement class.
   */
  std::vector<channel_t> getChannels();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void on_start() override;

private:
  friend class ConstantFractionDiscriminatorImpl;
  std::unique_ptr<ConstantFractionDiscriminatorImpl> impl;
};

class FileWriterImpl;
/**
 * \ingroup TimeTagStreaming
 * \brief compresses and stores all time tags to a file
 */
class TT_API FileWriter : public IteratorBase {
public:
  /**
   * \brief constructor of a FileWriter
   *
   * \param tagger        reference to a TimeTagger
   * \param filename      name of the file to store to, must be encoded as UTF-8
   * \param channels      channels which are stored to the file
   */
  FileWriter(TimeTaggerBase *tagger, const std::string &filename, std::vector<channel_t> channels);
  ~FileWriter();

  /**
   * \brief Close the current file and create a new one
   *
   * \param new_filename filename of the new file. If empty, the old one will be used.
   */
  void split(const std::string &new_filename = "");

  /**
   * \brief Set the maximum file size on disk when the automatic split shall happen.
   *
   * \note This is a rough limit, the actual file might be larger by one block.
   *
   * \param max_file_size new maximum file size in bytes
   */
  void setMaxFileSize(uint64_t max_file_size);

  /**
   * \brief fetches the maximum file size. Please see setMaxFileSize for more details.
   *
   * \return the maximum file size in bytes
   */
  uint64_t getMaxFileSize();

  /**
   * \brief queries the total amount of events stored in all files
   *
   * \return the total amount of events stored
   */
  uint64_t getTotalEvents();

  /**
   * \brief queries the total amount of bytes stored in all files
   *
   * \return the total amount of bytes stored
   */
  uint64_t getTotalSize();

  /**
   * \brief writes a marker in the file. While parsing the file, the last marker can be extracted again.
   *
   * \param marker the marker to write into the file
   */
  void setMarker(const std::string &marker);

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;
  void on_stop() override;

private:
  friend class FileWriterImpl;
  std::unique_ptr<FileWriterImpl> impl;
};

/**
 * \brief Reads tags from the disk files, which has been created by FileWriter.
 *
 * Its usage is compatible with the TimeTagStream.
 *
 * \ingroup TimeTagStreaming
 */
class TT_API FileReader {
public:
  /**
   * \brief Creates a file reader with the given filename.
   *
   * The file reader automatically continues to read split FileWriter Streams
   * In case multiple filenames are given, the files will be read in successively.
   *
   * \param filenames list of files to read, must be encoded as UTF-8
   */
  FileReader(std::vector<std::string> filenames);

  /**
   * \brief Creates a file reader with the given filename.
   *
   * The file reader automatically continues to read split FileWriter Streams
   *
   * \param filename file to read, must be encoded as UTF-8
   */
  FileReader(const std::string &filename);
  ~FileReader();

  /**
   * \brief Checks if there are still events in the FileReader
   *
   * \return false if no more events can be read from this FileReader
   */
  bool hasData();

  /**
   * \brief Fetches and delete the next tags from the internal buffer.
   *
   * Every tag is returned exactly once. If less than n_events
   * are returned, the reader is at the end-of-files.
   * \param n_events maximum amount of elements to fetch
   * \return a TimeTagStreamBuffer with up to n_events events
   */
  TimeTagStreamBuffer getData(uint64_t n_events);

  /**
   * \brief Low level file reading.
   *
   * This function will return the next non-empty buffer in a raw format.
   *
   * \param tag_buffer a buffer, which will be filled with the new events
   * \return true if fetching the data was successfully
   */
  bool getDataRaw(std::vector<Tag> &tag_buffer);

  /**
   * \brief Fetches the overall configuration status of the Time Tagger object,
   * which was serialized in the current file.
   *
   * \return a JSON serialized string with all configuration and status flags.
   */
  std::string getConfiguration();

  /**
   * \brief Fetches channels from the input file.
   *
   * \return a vector of channels from the input file.
   */
  std::vector<channel_t> getChannelList();

  /**
   * \brief return the last processed marker from the file.
   *
   * \return the last marker from the file
   */
  std::string getLastMarker();

private:
  friend class FileReaderImpl;
  std::unique_ptr<FileReaderImpl> impl;
};

class EventGeneratorImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief Generate predefined events in a virtual channel relative to a trigger event.
 *
 * \image html EventGenerator.svg
 * \image latex EventGenerator.pdf
 *
 * This iterator can be used to generate a predefined series of events, the pattern, relative to a trigger event on a
 * defined channel. A trigger_divider can be used to fire the pattern not on every, but on every n'th trigger received.
 * The trigger_offset can be used to select on which of the triggers the pattern will be generated when trigger
 * trigger_divider is greater than 1. To abort the pattern being generated, a stop_channel can be defined. In case it is
 * the very same as the trigger_channel, the subsequent generated patterns will not overlap.
 */
class TT_API EventGenerator : public IteratorBase {
public:
  /**
   * \brief construct a event generator
   *
   * \param tagger          reference to a TimeTagger
   * \param trigger_channel trigger for generating the pattern
   * \param pattern         vector of time stamp generated relative to the trigger event
   * \param trigger_divider establishes every how many trigger events a pattern is generated
   * \param divider_offset  the offset of the divided trigger when the pattern shall be emitted
   * \param stop_channel    channel on which a received event will stop all pending patterns from being generated
   */
  EventGenerator(TimeTaggerBase *tagger, channel_t trigger_channel, std::vector<timestamp_t> pattern,
                 uint64_t trigger_divider = 1, uint64_t divider_offset = 0, channel_t stop_channel = CHANNEL_UNUSED);

  ~EventGenerator();

  /**
   * \brief the new virtual channel
   *
   * This function returns the new allocated virtual channel.
   * It can be used now in any new iterator.
   *
   */
  channel_t getChannel();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class EventGeneratorImpl;
  std::unique_ptr<EventGeneratorImpl> impl;
};

class CombinationsImpl;
/**
 * \ingroup VirtualChannels
 *
 * A combination is defined as one or more events occurring on distinct channels within
 * a given time window, preceded and followed by two guard windows of the same duration without
 * any events on these channels. The guard window starts at the time of the last event during the
 * coincidence window.
 *
 * This iterator emits events on virtual channels whenever such a combination is detected
 * on the monitored channels. Given \f$ N \f$ input channels \f$ {c_1, \ldots, c_N }\f$, there are
 * \f$ 2^N - 1 \f$ possible combination, each having a corresponding virtual channel.
 *
 * The individual virtual channels one is interested in have to be enabled by calling getChannel() before clicks on them
 * are actually generated.
 *
 * Additionally, there are N "sum channels". The n-th sum channel generates a clicks on each n-fold combination
 * (irrespective of the specific contributing input channel).
 *
 * Note that multiple events on the same channel during the coincidence window are counted as one.
 */
class TT_API Combinations : public IteratorBase {
public:
  /**
   * \brief construct a Combinations
   *
   * \param tagger      reference to a TimeTagger
   * \param channels    the set of channels to monitor. Elements must be distinct.
   * \param window_size duration of the coincidence window
   */
  Combinations(TimeTaggerBase *tagger, std::vector<channel_t> const &channels, timestamp_t window_size);

  ~Combinations();

  /**
   * @brief Return the virtual channel ID corresponding to an exclusive coincidence on the given
   * \a input_channels. The channel gets implicitly enabled.
   */
  channel_t getChannel(std::vector<channel_t> const &input_channels) const;

  //! return the ID of the virtual channel corresponding to an n_channel-fold combination of input channels
  channel_t getSumChannel(int n_channels) const;

  //! Return the set of input channels that emit a coincidence event on the given virtual channel \a virtual_channel.
  std::vector<channel_t> getCombination(channel_t virtual_channel) const;

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class CombinationsImpl;
  std::unique_ptr<CombinationsImpl> impl;
};

/**
 * \brief Helper class for custom measurements in Python and C#
 *
 * \ingroup HelperClasses
 */
class TT_API CustomMeasurementBase : public IteratorBase {
protected:
  // Only usable for subclasses.
  CustomMeasurementBase(TimeTaggerBase *tagger);

public:
  ~CustomMeasurementBase() override;

  // Stop all running custom measurements. Use this to avoid races on shutdown the target language.
  static void stop_all_custom_measurements();

  // Forward the public API of the measurement
  void register_channel(channel_t channel);
  void unregister_channel(channel_t channel);
  void finalize_init();
  bool is_running() const;
  void _lock();
  void _unlock();

protected:
  // By default, this calls next_impl_cs
  virtual bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

  // Handler with easier to wrap API. By default, this does nothing
  virtual void next_impl_cs(void *tags_ptr, uint64_t num_tags, timestamp_t begin_time, timestamp_t end_time);

  // Forward the public handlers for swig to detect this virtual methods. By default, they do nothing
  virtual void clear_impl() override;
  virtual void on_start() override;
  virtual void on_stop() override;
};

/// Interface for FLIM measurements, Flim and FlimBase classes inherit from it
class TT_API FlimAbstract : public IteratorBase {
public:
  /**
   * \brief construct a FlimAbstract object, Flim and FlimBase classes inherit from it
   *
   * \param tagger                   reference to a TimeTagger
   * \param start_channel            channel on which start clicks are received for the time differences histogramming
   * \param click_channel            channel on which clicks are received for the time differences histogramming
   * \param pixel_begin_channel      start of a pixel (histogram)
   * \param n_pixels                 number of pixels (histograms) of one frame
   * \param n_bins                   number of histogram bins for each pixel
   * \param binwidth                 bin size in picoseconds
   * \param pixel_end_channel        end marker of a pixel - incoming clicks on the click_channel will be ignored
   *                                 afterwards
   * \param frame_begin_channel      (optional) start the frame, or reset the pixel index
   * \param finish_after_outputframe (optional) sets the number of frames stored within the measurement class. After
   *                                 reaching the number, the measurement will stop. If the number is 0 (default value),
   *                                 one frame is stored and the measurement runs continuously.
   * \param n_frame_average          (optional) average multiple input frames into one output frame, default: 1
   * \param pre_initialize           (optional) initializes the measurement on constructing.
   */
  FlimAbstract(TimeTaggerBase *tagger, channel_t start_channel, channel_t click_channel, channel_t pixel_begin_channel,
               uint32_t n_pixels, uint32_t n_bins, timestamp_t binwidth, channel_t pixel_end_channel = CHANNEL_UNUSED,
               channel_t frame_begin_channel = CHANNEL_UNUSED, uint32_t finish_after_outputframe = 0,
               uint32_t n_frame_average = 1, bool pre_initialize = true);

  ~FlimAbstract();

  /**
   * \brief tells if the data acquisition has finished reaching finish_after_outputframe
   *
   * This function returns a boolean which tells the user if the class is still acquiring data.
   * It can only reach the false state for finish_after_outputframe > 0.
   *
   * \note This can differ from isRunning. The return value of isRunning state depends only on start/startFor/stop.
   */
  bool isAcquiring() const { return acquiring; }

protected:
  template <FastBinning::Mode bin_mode> void process_tags(const std::vector<Tag> &incoming_tags);
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

  virtual void on_frame_end() = 0;

  const channel_t start_channel;
  const channel_t click_channel;
  const channel_t pixel_begin_channel;
  const uint32_t n_pixels;
  const uint32_t n_bins;
  const timestamp_t binwidth;
  const channel_t pixel_end_channel;
  const channel_t frame_begin_channel;
  const uint32_t finish_after_outputframe;
  const uint32_t n_frame_average;

  const timestamp_t time_window;

  timestamp_t current_frame_begin;
  timestamp_t current_frame_end;

  // state
  bool acquiring{};
  bool frame_acquisition{};
  bool pixel_acquisition{};

  uint32_t pixels_processed{};
  uint32_t frames_completed{};
  uint32_t ticks{};
  size_t data_base{};

  std::vector<uint32_t> frame;

  std::vector<timestamp_t> pixel_begins;
  std::vector<timestamp_t> pixel_ends;
  std::deque<timestamp_t> previous_starts;

  FastBinning binner;

  std::recursive_mutex acquisition_lock;
  bool initialized;
};

/**
 * \brief basic measurement, containing a minimal set of features for efficiency purposes
 *
 * The FlimBase provides only the most essential functionality for FLIM tasks. The benefit from the reduced
 * functionality is that it is very memory and CPU efficient. The class provides the frameReady() callback, which must
 * be used to analyze the data.
 *
 * \ingroup FLIM
 */
class TT_API FlimBase : public FlimAbstract {
public:
  /**
   * \brief construct a basic Flim measurement, containing a minimum featureset for efficiency purposes
   *
   * \param tagger                   reference to a TimeTagger
   * \param start_channel            channel on which start clicks are received for the time differences histogramming
   * \param click_channel            channel on which clicks are received for the time differences histogramming
   * \param pixel_begin_channel      start of a pixel (histogram)
   * \param n_pixels                 number of pixels (histograms) of one frame
   * \param n_bins                   number of histogram bins for each pixel
   * \param binwidth                 bin size in picoseconds
   * \param pixel_end_channel        end marker of a pixel - incoming clicks on the click_channel will be ignored
   *                                 afterwards
   * \param frame_begin_channel      (optional) start the frame, or reset the pixel index
   * \param finish_after_outputframe (optional) sets the number of frames stored within the measurement class. After
   *                                 reaching the number, the measurement will stop. If the number is 0 (default value),
   *                                 one frame is stored and the measurement runs continuously.
   * \param n_frame_average          (optional) average multiple input frames into one output frame, default: 1
   * \param pre_initialize           (optional) initializes the measurement on constructing.
   */
  FlimBase(TimeTaggerBase *tagger, channel_t start_channel, channel_t click_channel, channel_t pixel_begin_channel,
           uint32_t n_pixels, uint32_t n_bins, timestamp_t binwidth, channel_t pixel_end_channel = CHANNEL_UNUSED,
           channel_t frame_begin_channel = CHANNEL_UNUSED, uint32_t finish_after_outputframe = 0,
           uint32_t n_frame_average = 1, bool pre_initialize = true);

  ~FlimBase();

  /**
   * \brief initializes and starts measuring this Flim measurement
   *
   * This function initializes the Flim measurement and starts executing it. It does
   * nothing if preinitialized in the constructor is set to true.
   *
   */
  void initialize();

protected:
  void on_frame_end() override final;

  virtual void frameReady(uint32_t frame_number, std::vector<uint32_t> &data,
                          std::vector<timestamp_t> &pixel_begin_times, std::vector<timestamp_t> &pixel_end_times,
                          timestamp_t frame_begin_time, timestamp_t frame_end_time);

  uint32_t total_frames;
};

/// object for storing the state of Flim::getCurrentFrameEx
class TT_API FlimFrameInfo {
  friend class Flim;

public:
  ~FlimFrameInfo();

  /**
   * \brief index of this frame
   *
   * This function returns the frame number, starting from 0 for the very first frame acquired.
   * If the index is -1, it is an invalid frame which is returned on error
   *
   * deprecated, use frame_number instead..
   *
   */
  int32_t getFrameNumber() const { return frame_number; }

  /**
   * \brief tells if this frame is valid
   *
   * This function returns a boolean which tells if this frame is valid or not.
   * Invalid frames are possible on errors, such as asking for the last completed frame when
   * no frame has been completed so far.
   *
   * deprecated, use isValid instead.
   *
   */
  bool isValid() const { return valid; }

  /**
   * \brief number of pixels acquired on this frame
   *
   * This function returns a value which tells how many pixels were processed
   * for this frame.
   *
   */
  uint32_t getPixelPosition() const { return pixel_position; }

  GET_DATA_2D(getHistograms, uint32_t, array_out, );
  GET_DATA_1D(getIntensities, float, array_out, );
  GET_DATA_1D(getSummedCounts, uint64_t, array_out, );
  GET_DATA_1D(getPixelBegins, timestamp_t, array_out, );
  GET_DATA_1D(getPixelEnds, timestamp_t, array_out, );

private:
  FlimFrameInfo();
  std::vector<uint32_t> histograms;
  std::vector<timestamp_t> pixel_begins;
  std::vector<timestamp_t> pixel_ends;

public:
  uint32_t pixels;
  uint32_t bins;
  int32_t frame_number;
  uint32_t pixel_position;
  bool valid;
};

/**
 * @ingroup FLIM
 *
 * \brief Fluorescence lifetime imaging
 *
 * \image html Flim.svg
 * \image latex Flim.pdf
 *
 * Successively acquires n histograms (one for each pixel in the image), where
 * each histogram is determined by the number of bins and the binwidth.
 * Clicks that fall outside the histogram range are ignored.
 *
 * Fluorescence-lifetime imaging microscopy or Flim is an imaging technique for producing an image based on
 * the differences in the exponential decay rate of the fluorescence from a fluorescent sample.
 *
 * Fluorescence lifetimes can be determined in the time domain by using a pulsed source. When a population
 * of fluorophores is excited by an ultrashort or delta pulse of light, the time-resolved fluorescence will
 * decay exponentially.
 *
 */
class TT_API Flim : public FlimAbstract {
public:
  /**
   * \brief construct a Flim measurement with a variety of high-level functionality
   *
   * \param tagger                   reference to a TimeTagger
   * \param start_channel            channel on which start clicks are received for the time differences histogramming
   * \param click_channel            channel on which clicks are received for the time differences histogramming
   * \param pixel_begin_channel      start of a pixel (histogram)
   * \param n_pixels                 number of pixels (histograms) of one frame
   * \param n_bins                   number of histogram bins for each pixel
   * \param binwidth                 bin size in picoseconds
   * \param pixel_end_channel        end marker of a pixel - incoming clicks on the click_channel will be ignored
   *                                 afterwards
   * \param frame_begin_channel      (optional) start the frame, or reset the pixel index
   * \param finish_after_outputframe (optional) sets the number of frames stored within the measurement class. After
   *                                 reaching the number, the measurement will stop. If the number is 0 (default value),
   *                                 one frame is stored and the measurement runs continuously.
   * \param n_frame_average          (optional) average multiple input frames into one output frame, default: 1
   * \param pre_initialize           (optional) initializes the measurement on constructing.
   */
  Flim(TimeTaggerBase *tagger, channel_t start_channel, channel_t click_channel, channel_t pixel_begin_channel,
       uint32_t n_pixels, uint32_t n_bins, timestamp_t binwidth, channel_t pixel_end_channel = CHANNEL_UNUSED,
       channel_t frame_begin_channel = CHANNEL_UNUSED, uint32_t finish_after_outputframe = 0,
       uint32_t n_frame_average = 1, bool pre_initialize = true);

  ~Flim();

  /**
   * \brief initializes and starts measuring this Flim measurement
   *
   * This function initializes the Flim measurement and starts executing it. It does
   * nothing if preinitialized in the constructor is set to true.
   *
   */
  void initialize();

  /**
   * \brief obtain for each pixel the histogram for the given frame index
   *
   * This function returns the histograms for all pixels according to the frame index
   * given. If the index is -1, it will return the last frame, which has been completed.
   * When finish_after_outputframe is 0, the index value must be -1.
   * If index >= finish_after_outputframe, it will throw an error.
   *
   * \param array_out callback for the array output allocation
   * \param index  index of the frame to be obtained. if -1, the last frame which has been completed is returned
   */
  GET_DATA_2D_OP1(getReadyFrame, uint32_t, array_out, int32_t, index, -1, );

  /**
   * \brief obtain an array of the pixel intensity of the given frame index
   *
   * This function returns the intensities according to the frame index
   * given. If the index is -1, it will return the intensity of the last frame, which has been completed.
   * When finish_after_outputframe is 0, the index value must be -1.
   * If index >= finish_after_outputframe, it will throw an error.
   *
   * The pixel intensity is defined by the number of counts acquired within the pixel divided by the respective
   * integration time.
   *
   * \param array_out callback for the array output allocation
   * \param index  index of the frame to be obtained. if -1, the last frame which has been completed is returned
   */
  GET_DATA_1D_OP1(getReadyFrameIntensity, float, array_out, int32_t, index, -1, );

  /**
   * \brief obtain for each pixel the histogram for the frame currently active
   *
   * This function returns the histograms for all pixels of the currently active frame
   */
  GET_DATA_2D(getCurrentFrame, uint32_t, array_out, );

  /**
   * \brief obtain the array of the pixel intensities of the frame currently active
   *
   * This function returns the intensities of all pixels of the currently active frame
   *
   * The pixel intensity is defined by the number of counts acquired within the pixel divided by the respective
   * integration time.
   */
  GET_DATA_1D(getCurrentFrameIntensity, float, array_out, );

  /**
   * \brief obtain for each pixel the histogram from all frames acquired so far
   *
   * This function returns the histograms for all pixels. The counts within the histograms are integrated since the
   * start or the last clear of the measurement.
   *
   * \param array_out callback for the array output allocation
   * \param only_ready_frames  if true, only the finished frames are added. On false, the currently active frame is
   * aggregated.
   * \param clear_summed       if true, the summed frames memory will be cleared.
   */
  GET_DATA_2D_OP2(getSummedFrames, uint32_t, array_out, bool, only_ready_frames, true, bool, clear_summed, false, );

  /**
   * \brief obtain the array of the pixel intensities from all frames acquired so far
   *
   * The pixel intensity is the number of counts within the pixel divided by the integration time.
   *
   * This function returns the intensities of all pixels summed over all acquired frames.
   *
   * \param array_out callback for the array output allocation
   * \param only_ready_frames  if true only the finished frames are added. On false, the currently active frame is
   * aggregated.
   * \param clear_summed       if true, the summed frames memory will be cleared.
   */
  GET_DATA_1D_OP2(getSummedFramesIntensity, float, array_out, bool, only_ready_frames, true, bool, clear_summed,
                  false, );

  /**
   * \brief obtain a frame information object, for the given frame index
   *
   * This function returns a frame information object according to the index
   * given. If the index is -1, it will return the last completed frame. When finish_after_outputframe
   * is 0, index must be -1.
   * If index >= finish_after_outputframe, it will throw an error.
   *
   * \param index  index of the frame to be obtained. if -1, last completed frame will be returned
   */
  FlimFrameInfo getReadyFrameEx(int32_t index = -1);

  /**
   * \brief obtain a frame information object, for the currently active frame
   *
   * This function returns the frame information object for the currently active frame
   */
  FlimFrameInfo getCurrentFrameEx();

  /**
   * \brief obtain a frame information object, that represents the sum of all frames acquired so for.
   *
   * This function returns the frame information object that represents the sum of all acquired frames.
   *
   * \param only_ready_frames  if true only the finished frames are added. On false, the currently active is aggregated.
   * \param clear_summed       if true, the summed frames memory will be reset and all frames stored prior
   * will be unaccounted in the future.
   */
  FlimFrameInfo getSummedFramesEx(bool only_ready_frames = true, bool clear_summed = false);

  /**
   * \brief total number of frames completed so far
   *
   * This function returns the amount of frames that have been completed so far, since the creation / last clear
   * of the object.
   */
  uint32_t getFramesAcquired() const { return total_frames; }

  /**
   * \brief a vector of size n_bins containing the time bins in ps
   *
   * This function returns a vector of size n_bins containing the time bins in ps.
   */
  GET_DATA_1D(getIndex, timestamp_t, array_out, );

protected:
  void on_frame_end() override final;
  void clear_impl() override;

  uint32_t get_ready_index(int32_t index);

  virtual void frameReady(uint32_t frame_number, std::vector<uint32_t> &data,
                          std::vector<timestamp_t> &pixel_begin_times, std::vector<timestamp_t> &pixel_end_times,
                          timestamp_t frame_begin_time, timestamp_t frame_end_time);

  std::vector<std::vector<uint32_t>> back_frames;
  std::vector<std::vector<timestamp_t>> frame_begins;
  std::vector<std::vector<timestamp_t>> frame_ends;
  std::vector<uint32_t> pixels_completed;
  std::vector<uint32_t> summed_frames;
  std::vector<timestamp_t> accum_diffs;
  uint32_t captured_frames;
  uint32_t total_frames;
  int32_t last_frame;

  std::mutex swap_chain_lock;
};

class SamplerImpl;
/**
 * \ingroup TimeTagStreaming
 *
 * \brief a triggered sampling measurement
 *
 * This measurement class will perform a triggered sampling measurement.
 * So for every event on the trigger input, the current state (low : 0, high : 1, unknown : 2)
 * will be written to an internal buffer. Fetching the data of the internal buffer will clear
 * its internal state without any deadtime. So every event will recorded exactly once.
 *
 * The unknown state might happen after an overflow without an event on the input channel.
 * This processing assumes that no event was filtered by the deadtime. Else invalid data will
 * be reported till the next event on this input channel.
 */
class TT_API Sampler : public IteratorBase {
public:
  /**
   * \brief constructor of a Sampler measurement
   *
   * \param tagger        reference to a TimeTagger
   * \param trigger       the channel which shall trigger the measurement
   * \param channels      a list of channels which will be recorded for every trigger
   * \param max_triggers  the maximum amount of triggers without getData* call till this measurement will stop itself
   */
  Sampler(TimeTaggerBase *tagger, channel_t trigger, std::vector<channel_t> channels, size_t max_triggers);
  ~Sampler();

  /**
   * \brief fetches the internal data as 2D array.
   *
   * Its layout is roughly:
   * [
   *   [timestamp of first trigger, state of channel 0, state of channel 1, ...],
   *   [timestamp of second trigger, state of channel 0, state of channel 1, ...],
   *   ...
   * ]
   * Where state means:
   *   0 -- low
   *   1 -- high
   *   2 -- undefined (after overflow)
   */
  GET_DATA_2D(getData, timestamp_t, array_out, );

  /**
   * \brief fetches the internal data as 2D array with a channel mask.
   *
   * Its layout is roughly:
   * [
   *   [timestamp of first trigger, (state of channel 0) << 0 | (state of channel 1) << 1 | ... | undefined << 63],
   *   [timestamp of second trigger, (state of channel 0) << 0 | (state of channel 1) << 1 | ... | undefined << 63],
   *   ...
   * ]
   * Where state means:
   *   0 -- low or undefined (after overflow)
   *   1 -- high
   */
  GET_DATA_2D(getDataAsMask, timestamp_t, array_out, );

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class SamplerImpl;
  std::unique_ptr<SamplerImpl> impl;
};

class SyntheticSingleTagImpl;
/**
 * @ingroup HelperClasses
 *
 * \brief synthetic trigger timetag generator.
 *
 * Creates timetags based on a trigger method. Whenever the user calls the 'trigger'
 * method, a timetag will be added to the base_channel.
 *
 * This synthetic channel can inject timetags into an existing channel or create a new virtual channel.
 *
 */
class TT_API SyntheticSingleTag : public IteratorBase {
public:
  /**
   * \brief Construct a pulse event generator.
   *
   * \param tagger        reference to a TimeTagger
   * \param base_channel  base channel to which this signal will be added. If unused, a new channel will be
   *                      created.
   */
  SyntheticSingleTag(TimeTaggerBase *tagger, channel_t base_channel = CHANNEL_UNUSED);
  ~SyntheticSingleTag();

  /**
   * \brief Generate a timetag for each call of this method.
   */
  void trigger();

  channel_t getChannel() const;

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

private:
  friend class SyntheticSingleTagImpl;
  std::unique_ptr<SyntheticSingleTagImpl> impl;
};

class FrequencyStabilityImpl;
struct FrequencyStabilityDataImpl;
class FrequencyStability;

/**
 * \brief return data object for FrequencyStability::getData.
 */
class TT_API FrequencyStabilityData {
public:
  ~FrequencyStabilityData();

  /**
   * \brief returns the standard derivation of each period pair
   */
  GET_DATA_1D(getSTDD, double, array_out, );

  /**
   * \brief returns the overlapping Allan deviation
   */
  GET_DATA_1D(getADEV, double, array_out, );

  /**
   * \brief returns the modified overlapping Allan deviation
   */
  GET_DATA_1D(getMDEV, double, array_out, );

  /**
   * \brief returns the overlapping time deviation
   *
   * This is the scaled version of the modified overlapping Allan deviation.
   */
  GET_DATA_1D(getTDEV, double, array_out, );

  /**
   * \brief returns the overlapping Hadamard deviation
   */
  GET_DATA_1D(getHDEV, double, array_out, );

  /**
   * \brief returns the scaled version of the overlapping Allan deviation
   */
  GET_DATA_1D(getADEVScaled, double, array_out, );

  /**
   * \brief returns the scaled version of the overlapping Hadamard deviation
   */
  GET_DATA_1D(getHDEVScaled, double, array_out, );

  /**
   * \brief returns the analysis position of all deviations
   */
  GET_DATA_1D(getTau, double, array_out, );

  /**
   * \brief returns a trace of the last phase samples in seconds
   */
  GET_DATA_1D(getTracePhase, double, array_out, );

  /**
   * \brief returns a trace of the last normalized frequency error samples in pp1
   */
  GET_DATA_1D(getTraceFrequency, double, array_out, );

  /**
   * \brief returns a trace of the last absolute frequency samples in Hz
   *
   * \param array_out allocator for return array
   * \param input_frequency reference frequency in Hz
   *
   * \note The precision of the parameter input_frequency and so the mean value of the return values are limited to
   * 15 digits. However the relative errors within the return values have a higher precision.
   */
  GET_DATA_1D_OP1(getTraceFrequencyAbsolute, double, array_out, double, input_frequency, 0.0, );

  /**
   * \brief returns the timestamps of the traces in seconds
   */
  GET_DATA_1D(getTraceIndex, double, array_out, );

private:
  FrequencyStabilityData();
  friend class FrequencyStability;

  std::shared_ptr<FrequencyStabilityDataImpl> data;
};

/**
 * \ingroup PhaseAndFrequency
 *
 * \brief Allan deviation (and related metrics) calculator
 *
 * It shall analyze the stability of a clock by computing deviations of phase[i] - phase[i + n].
 * The list of all n values needs to be declared in the beginning.
 *
 * Reference: https://www.nist.gov/publications/handbook-frequency-stability-analysis
 *
 * It calculates the STDD, ADEV, MDEV and HDEV on the fly:
 * - STDD: Standard derivation of each period pair.
 *         This is not a stable analysis with frequency drifts and only calculated for reference.
 * - ADEV: Overlapping Allan deviation, the most common analysis framework.
 *         Square mean value of the second derivate `phase[i] - 2*phase[i + n] + phase[i + 2*n]`.
 *         In a loglog plot, the slope allows to identify the source of noise:
 *   -        -1: white or flicker phase noise, like discretization or analog noisy delay
 *   -      -0.5: white period noise
 *   -         0: flicker period noise, like electric noisy oscillator
 *   -       0.5: integrated white period noise (random walk period)
 *   -         1: frequency drift, e.g. thermal
 *   .
 *         As this tool is most likely used to analyze timings, a scaled ADEV is implemented.
 *         It adds 1.0 to each slope and normalize the return value to picoseconds for phase noise.
 * - MDEV: Modified overlapping Allan deviation. It averages the second derivate of ADEV before
 *         calculating the MSE. This splits the slope of white and flicker phase noise:
 *   -      -1.5: white phase noise, like discretization
 *   -      -1.0: flicker phase noise, like an electric noisy delay
 *   .
 *         The scaled approach (+1 on each slope yielding picoseconds as return value) is called TDEV and
 *         more commonly used than MDEV.
 * - HDEV: The overlapping Hadamard deviation uses the third derivate of the phase.
 *         This cancels the effect of a constant phase drift.
 */
class TT_API FrequencyStability : public IteratorBase {
public:
  /**
   * \brief constructor of a FrequencyStability measurement
   *
   * \param tagger time tagger object
   * \param channel the clock input channel used for the analysis
   * \param steps a vector or integer tau values for all deviations
   * \param average an averaging down sampler to reduce noise and memory requirements
   * \param trace_len length of the phase and frequency trace capture of the averaged data
   *
   * \note This measurements needs 24 times the largest value in steps bytes of main memory
   */
  FrequencyStability(TimeTaggerBase *tagger, channel_t channel, std::vector<uint64_t> steps, timestamp_t average = 1000,
                     uint64_t trace_len = 1000);
  ~FrequencyStability();

  /**
   * \brief get a return object with all data in a synchronized way
   */
  FrequencyStabilityData getDataObject();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class FrequencyStabilityImpl;
  std::unique_ptr<FrequencyStabilityImpl> impl;
};

class PRBS;

/**
 * \brief Namespace for features, which are still in development and are likely to change.
 */
namespace Experimental {

class PulsePerSecondImpl;
class PulsePerSecondDataState;

/**
 * \brief Helper object as return value for PulsePerSecondMonitor::getDataObject.
 *
 * This object stores the results of all monitored PPS pulses.
 */
class TT_API PulsePerSecondData {
public:
  /**
   * \brief The indices of each reference pulse in the `PulsePerSecondData` object. In case of overflows in the
   * reference channel, this index will be incremented by the number of missed pulses.
   */
  GET_DATA_1D(getIndices, int64_t, array_out, );
  /**
   * \brief A list of offsets of each reference pulse with respective to its predecessor, with the period subtracted.
   */
  GET_DATA_1D(getReferenceOffsets, double, array_out, );
  /**
   * \brief For each reference contained in the `PulsePerSecondData` object a list of offsets for each signal channel is
   * given, in the channel order given by `signal_channels`.
   */
  GET_DATA_2D(getSignalOffsets, double, array_out, );
  /**
   * \brief The number of elapsed seconds from the beginning of the Unix epoch (1st of January 1970) to the time at
   * which each reference pulse is processed, as a floating point number.
   */
  GET_DATA_1D(getUtcSeconds, double, array_out, );
  /**
   * \brief The UTC timestamps for the system time at which each reference pulse is processed, as a string with ISO 8601
   * formatting.
   */
  std::vector<std::string> getUtcDates();
  /**
   * \brief A vector of booleans values describing whether all signals, including from the
   * reference source, were detected.
   */
  GET_DATA_1D(getStatus, bool, array_out, );
  /**
   * \brief Number of reference pulses contained in the `PulsePerSecondData` object.
   */
  const size_t size;
  ~PulsePerSecondData();

private:
  PulsePerSecondData(std::shared_ptr<PulsePerSecondDataState> data_ptr, const std::vector<channel_t> channel_list,
                     size_t size);
  std::shared_ptr<PulsePerSecondDataState> data;
  const std::vector<channel_t> channel_list;
  friend PulsePerSecondDataState;
  friend PulsePerSecondImpl;
};

/**
 * \ingroup PhaseAndFrequency
 *
 * \brief Monitors the synchronicity of 1 pulse per second (PPS) signals
 *
 * This measurement allows the user to monitor the synchronicity of different sources of 1 pulse per second (PPS)
 * signals with respect to a reference source. For each signal from the reference PPS source, comparative offsets are
 * calculated for the other signal channels. Upon processing, a UTC timestamp from the system time is associated with
 * each reference pulse.
 *
 * The monitoring starts on the first signal from the reference source and will run uninterrupted until the measurement
 * is stopped. If a signal from a channel is not detected within one and a half periods, its respective offset will not
 * be calculated but the measurement will continue nonetheless.
 *
 * By specifying an output file name, the monitoring data can be continuously written to a comma-separated value file
 * (.csv).
 */
class TT_API PulsePerSecondMonitor : public IteratorBase {
public:
  /**
   * \brief constructor of a PulsePerSecondMonitor measurement
   *
   * \param tagger a TimeTagger object.
   * \param reference_channel the channel whose signal will be the standard against which other signals are compared.
   * \param signal_channels a list of channel numbers with PPS signals to be compared to the reference.
   * \param filename the name of the .csv file to store measurement data. By default, no data is written to file.
   * \param period the assumed period of the reference source, typically one second, in picoseconds.
   *
   */
  PulsePerSecondMonitor(TimeTaggerBase *tagger, channel_t reference_channel, std::vector<channel_t> signal_channels,
                        std::string filename = "", timestamp_t period = 1E12);
  ~PulsePerSecondMonitor();
  /**
   * \brief Fetches the results of all measured PPS pulses
   *
   * \returns a `PulsePerSecondData` object, which contains all data of the monitored PPS pulses. To remove the data
   * from the internal memory after each call, set `remove` to `true`.
   */
  PulsePerSecondData getDataObject(bool remove = false);

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;
  void on_start() override;

private:
  friend class PulsePerSecondImpl;
  std::unique_ptr<PulsePerSecondImpl> impl;
};

class SignalGeneratorBaseImpl;

class TT_API SignalGeneratorBase : public IteratorBase {
public:
  SignalGeneratorBase(TimeTaggerBase *tagger, channel_t base_channel = CHANNEL_UNUSED);
  ~SignalGeneratorBase();

  /**
   * \brief the new virtual channel
   *
   * This function returns the new allocated virtual channel.
   * It can be used now in any new iterator.
   *
   */
  channel_t getChannel();

  // void registerReactor(std::string property, channel_t trigger_channel, std::vector<float> values, bool repeat);

protected:
  virtual void initialize(timestamp_t initial_time) = 0;
  virtual timestamp_t get_next() = 0;

  // void addReactable(std::string property, std::function<void, float> &&callback);

  virtual void on_restart(timestamp_t restart_time);

  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void on_stop() override;

  // callbacks
  bool isProcessingFinished();
  void set_processing_finished(bool is_finished);

  friend class SignalGeneratorBaseImpl;
  std::unique_ptr<SignalGeneratorBaseImpl> impl;
};

class TT_API PhotonGenerator : public SignalGeneratorBase {
public:
  /**
  * \brief A generator for TimeTags arising from a laser driven process. `PhotonGenerator` should be used as the base
  class of a virtual class with a dedicated `get_intensity` function which models the relevant physical processes.
  *
  * \param tagger                reference to a TimeTagger.
  * \param countrate             rate (in Hz) of Time Tags to be generated.
  * \param base_channel        base channel to which this signal will be added. If unused, a new channel will be
  * created.
    \param seed                  Seed number for the Pseudo-random number generator. Use -1 to use the
  * current time as seed.
  */
  PhotonGenerator(TimeTaggerBase *tagger, double countrate, channel_t base_channel, int32_t seed = -1);
  ~PhotonGenerator();
  void finalize_init();
  void set_T_PERIOD(timestamp_t new_T);
  timestamp_t get_T_PERIOD();

protected:
  void initialize(timestamp_t initial_time) override;
  void on_restart(timestamp_t restart_time) override;
  timestamp_t get_next() override;
  virtual double get_intensity() = 0;
  timestamp_t T_PERIOD;

private:
  timestamp_t get_new_stamp();
  std::minstd_rand0 generator;
  std::exponential_distribution<double> exp_distribution;
  timestamp_t accumulated;
  timestamp_t base_time;
  timestamp_t t_evolution;
  double current_intensity;
};

class DlsSignalGeneratorImpl;

class TT_API DlsSignalGenerator : public PhotonGenerator {
public:
  /**
   * \brief Construct a DLS event channel.
   *
   * \param tagger                reference to a TimeTagger.
   * \param decay_time            characteristic decay times (in seconds) for the g2 curve.
   * \param countrate             rate (in Hz) of Time Tags to be generated.
   * \param output_channel        base channel to which this signal will be added. If unused, a new channel will be
   *                              created.
   * \param seed                  Seed number for the Pseudo-random number generator. Use -1 to use the
   *                              current time as seed.
   */
  DlsSignalGenerator(TimeTaggerBase *tagger, double decay_time, double countrate,
                     channel_t output_channel = CHANNEL_UNUSED, int32_t seed = -1);
  DlsSignalGenerator(TimeTaggerBase *tagger, std::vector<double> decay_times, double countrate,
                     channel_t output_channel = CHANNEL_UNUSED, int32_t seed = -1);
  ~DlsSignalGenerator();
  unsigned int get_N();

protected:
  double get_intensity() override;

private:
  friend class DlsSignalGeneratorImpl;
  std::unique_ptr<DlsSignalGeneratorImpl> impl;
};

class FcsSignalGeneratorImpl;

class TT_API FcsSignalGenerator : public PhotonGenerator {
public:
  /**
   * \brief Construct an FCS event channel.
   *
   * \param tagger                reference to a TimeTagger.
   * \param correlation_time      characteristic correlation time in the exponential g2 curve.
   * \param countrate             rate (in Hz) of Time Tags to be generated.
   * \param N_focus               the average number of particles in the laser focus.
   * \param output_channel        base channel to which this signal will be added. If unused, a new channel will be
   * created.
     \param seed                  Seed number for the Pseudo-random number generator. Use -1 to use the
   * current time as seed.
   */
  FcsSignalGenerator(TimeTaggerBase *tagger, double correlation_time, double N_focus, double countrate,
                     channel_t output_channel = CHANNEL_UNUSED, int32_t seed = -1);
  ~FcsSignalGenerator();
  unsigned int get_N();
  void set_boundary_limit(double new_boundary);

protected:
  double get_intensity() override;

private:
  friend class FcsSignalGeneratorImpl;
  std::unique_ptr<FcsSignalGeneratorImpl> impl;
};

class TT_API UniformSignalGenerator : public SignalGeneratorBase {
public:
  /**
   * \brief Construct a random uniform event channel.
   *
   * \param tagger              reference to a TimeTagger
   * \param upper_bound         Max possible offset of event generated compared to latest.
   * \param lower_bound         Min possible offset of event generated, must be higher than 0.
   * \param base_channel        base channel to which this signal will be added. If unused, a new channel will be
   *                            created.
   * \param seed                Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                            seed.
   */
  UniformSignalGenerator(TimeTaggerBase *tagger, timestamp_t upper_bound, timestamp_t lower_bound = 1,
                         channel_t base_channel = CHANNEL_UNUSED, int32_t seed = -1);
  ~UniformSignalGenerator();

protected:
  void initialize(timestamp_t initial_time) override;
  timestamp_t get_next() override;

  void on_restart(timestamp_t restart_time) override;

private:
  std::unique_ptr<PRBS> generator;
  timestamp_t lower_bound;
  timestamp_t period;
  timestamp_t accumulated;
  timestamp_t base_time;
};

class TT_API GaussianSignalGenerator : public SignalGeneratorBase {
public:
  /**
   * \brief Construct a gaussian event channel.
   *
   * \param tagger              reference to a TimeTagger
   * \param mean                mean time each event is generated.
   * \param standard_deviation  standard deviation of the normal distribution.
   * \param base_channel        base channel to which this signal will be added. If unused, a new channel will be
   *                            created.
   * \param seed                Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                            seed.
   */
  GaussianSignalGenerator(TimeTaggerBase *tagger, double mean, double standard_deviation,
                          channel_t base_channel = CHANNEL_UNUSED, int32_t seed = -1);
  ~GaussianSignalGenerator();

protected:
  void initialize(timestamp_t initial_time) override;
  timestamp_t get_next() override;

  void on_restart(timestamp_t restart_time) override;

private:
  std::minstd_rand0 generator;
  std::normal_distribution<double> distr;
  timestamp_t accumulated;
  timestamp_t base_time;
};

class FlickerDistributionVossMcCartney;

class TT_API OscillatorSimulation : public SignalGeneratorBase {
public:
  /**
   * \brief Construct a simulated oscillator event channel.
   *
   * \param tagger              reference to a TimeTagger
   * \param nominal_frequency   Normal frequency of the oscillator in Hz
   * \param coeff_phase_white    RMS value of the white phase noise in seconds.
   * \param coeff_phase_flicker   RMS value of the flicker phase noise in seconds per octave.
   * \param coeff_freq_white     Scaling parameter for the white frequency modulated noise in sqrt(s),
   *                            use `10e-12 * sqrt(1e-3)` for 10 ppt RMS error at 1 kHz cutoff frequency.
   * \param coeff_freq_flicker    Scaling parameter for the relative flicker frequency modulated noise,
   *                            use `10e-12` for 10 ppt error per octave.
   * \param coeff_random_drift   Scaling parameter for the random walk drift in sqrt(Hz),
   *                            use `10e-9 / sqrt(60*60*24)` for 10 ppb / sqrt(day).
   * \param coeff_linear_drift   Scaling parameter for the relative linear frequency drift in Hz,
   *                            use `1e-6 / (60*60*24*365)` for 1 ppm / year.
   * \param base_channel        base channel to which this signal will be added. If unused, a new channel will be
   *                            created.
   * \param seed                Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                            seed.
   */
  OscillatorSimulation(TimeTaggerBase *tagger, double nominal_frequency, double coeff_phase_white = 0.0,
                       double coeff_phase_flicker = 0.0, double coeff_freq_white = 0.0, double coeff_freq_flicker = 0.0,
                       double coeff_random_drift = 0.0, double coeff_linear_drift = 0.0,
                       channel_t base_channel = CHANNEL_UNUSED, int32_t seed = -1);

  ~OscillatorSimulation();

protected:
  void initialize(timestamp_t initial_time) override;
  timestamp_t get_next() override;

  void on_restart(timestamp_t restart_time) override;

private:
  double const coeff_phase_white, coeff_phase_flicker, coeff_freq_white, coeff_freq_flicker, coeff_random_drift,
      coeff_linear_drift;

  timestamp_t const nominal_period_int;   //!< integer (ps) part of period
  double const nominal_period_fractional; //!< remaining (sub-ps) part of period

  std::unique_ptr<FlickerDistributionVossMcCartney> flicker_phase, flicker_freq;
  std::normal_distribution<double> white;
  std::mt19937_64 generator;
  double freq_random_walk_acc, //!< accumulated phase drift from freq. flicker noise
      fractional_ps_acc;       //!< accumulate sub-ps noise
  timestamp_t last_time;
  uint64_t num_periods_passed{}; //!< number of passed nominal periods
};

class TT_API TwoStateExponentialSignalGenerator : public SignalGeneratorBase {
public:
  /**
   * \brief Construct a two-state exponential event channel.
   *
   * \param tagger          reference to a TimeTagger
   * \param excitation_time excitation time in seconds.
   * \param life_time       life time of the excited state in seconds
   * \param base_channel    base channel to which this signal will be added. If unused, a new channel will be
   *                        created.
   * \param seed            Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                        seed.
   */
  TwoStateExponentialSignalGenerator(TimeTaggerBase *tagger, double excitation_time, double life_time,
                                     channel_t base_channel = CHANNEL_UNUSED, int32_t seed = -1);
  ~TwoStateExponentialSignalGenerator();

protected:
  void initialize(timestamp_t initial_time) override;
  timestamp_t get_next() override;

  void on_restart(timestamp_t restart_time) override;

private:
  std::minstd_rand0 generator;
  std::exponential_distribution<double> excitation_time_distr;
  std::exponential_distribution<double> life_time_distr;
  timestamp_t accumulated;
  timestamp_t base_time;
};

class MarkovProcessGeneratorImpl;
class TT_API MarkovProcessGenerator : public IteratorBase {
public:
  /**
   * \brief Construct a continuous-time Markov chain process.
   *
   * https://en.wikipedia.org/wiki/Continuous-time_Markov_chain
   *
   * \param tagger          reference to a TimeTagger
   * \param num_states      Number of exponential states.
   * \param frequencies     frequencies of each state transition, it's size is num_states * num_states.
   * \param ref_channels    tells the net channel to look at on a state transition. its size is num_states * num_states.
   * \param base_channels   channels in which to generate or add the new timetags if CHANNEL_UNUSED or empty, generate
   *                        a new virtual channel
   * \param seed            Seed number for the Pseudo-random number generator. Use -1 to use the
   *                        current time as seed.
   */
  MarkovProcessGenerator(TimeTaggerBase *tagger, uint64_t num_states, std::vector<double> frequencies,
                         std::vector<channel_t> ref_channels,
                         std::vector<channel_t> base_channels = std::vector<channel_t>(), int32_t seed = -1);
  ~MarkovProcessGenerator();

  channel_t getChannel();
  std::vector<channel_t> getChannels();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void on_stop() override;

private:
  friend class MarkovProcessGeneratorImpl;
  std::unique_ptr<MarkovProcessGeneratorImpl> impl;
};

class TT_API ExponentialSignalGenerator : public SignalGeneratorBase {
public:
  /**
   * \brief Construct a exponential event channel.
   *
   * \param tagger              reference to a TimeTagger
   * \param rate                event rate in herz
   * \param base_channel        base channel to which this signal will be added. If unused, a new channel will be
   *                            created.
   * \param seed                Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                            seed.
   */
  ExponentialSignalGenerator(TimeTaggerBase *tagger, double rate, channel_t base_channel = CHANNEL_UNUSED,
                             int32_t seed = -1);
  ~ExponentialSignalGenerator();

protected:
  void initialize(timestamp_t initial_time) override;
  timestamp_t get_next() override;

  void on_restart(timestamp_t restart_time) override;

private:
  std::minstd_rand0 generator;
  std::exponential_distribution<double> distr;
  timestamp_t accumulated;
  timestamp_t base_time;
};

class TT_API GammaSignalGenerator : public SignalGeneratorBase {
public:
  /**
   * \brief Construct a gamma event channel.
   *
   * \param tagger              reference to a TimeTagger
   * \param alpha               alpha value of the gamma distribution
   * \param beta                beta value of the gamma distribution
   * \param base_channel        base channel to which this signal will be added. If unused, a new channel will be
   *                            created.
   * \param seed                Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                            seed.
   */
  GammaSignalGenerator(TimeTaggerBase *tagger, double alpha, double beta, channel_t base_channel = CHANNEL_UNUSED,
                       int32_t seed = -1);
  ~GammaSignalGenerator();

protected:
  void initialize(timestamp_t initial_time) override;
  timestamp_t get_next() override;

  void on_restart(timestamp_t restart_time) override;

private:
  std::minstd_rand0 generator;
  std::gamma_distribution<double> distr;
  timestamp_t accumulated;
  timestamp_t base_time;
};

class TT_API PatternSignalGenerator : public SignalGeneratorBase {
public:
  /**
   * \brief Construct a pattern event generator.
   *
   * \param tagger        reference to a TimeTagger
   * \param sequence      sequence of offsets pattern to be used continuously.
   * \param repeat        tells if to repeat the pattern or only generate it once.
   * \param start_delay   initial delay before the first pattern is applied.
   * \param spacing       delay between pattern repetitions.
   * \param base_channel  base channel to which this signal will be added. If unused, a new channel will be
   *                      created.
   */
  PatternSignalGenerator(TimeTaggerBase *tagger, std::vector<timestamp_t> sequence, bool repeat = false,
                         timestamp_t start_delay = 0, timestamp_t spacing = 0, channel_t base_channel = CHANNEL_UNUSED);
  ~PatternSignalGenerator();

protected:
  void initialize(timestamp_t initial_time) override;
  timestamp_t get_next() override;

  void on_restart(timestamp_t restart_time) override;

private:
  std::vector<timestamp_t> sequence;
  bool repeat;
  int64_t index;
  timestamp_t base_time;
  timestamp_t accumulated;
  timestamp_t start_delay;
  timestamp_t spacing;
};

class SimSignalSplitterImpl;
class TT_API SimSignalSplitter : public IteratorBase {
public:
  /**
   * \brief Construct a signal splitter which will split events from an input channel
   * into a left and a right virtual channels.
   *
   * \param tagger        reference to a TimeTagger
   * \param input_channel channel to be split.
   * \param ratio         bias towards right or left channel.
   * \param seed          Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                      seed.
   */
  SimSignalSplitter(TimeTaggerBase *tagger, channel_t input_channel, double ratio = 0.5, int32_t seed = -1);

  ~SimSignalSplitter();

  std::vector<channel_t> getChannels();
  channel_t getLeftChannel();
  channel_t getRightChannel();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

private:
  friend class SimSignalSplitterImpl;
  std::unique_ptr<SimSignalSplitterImpl> impl;
};

class TT_API TransformEfficiency : public IteratorBase {
public:
  /**
   * \brief Construct a transformation that will apply an efficiency filter to an
   * specified channel. An efficiency filter will drop events based on an efficiency
   * value. A perfect effcincy of 1.0 won't drop any events, an efficiency of 0.5
   * will drop half the events.
   *
   * @note this measurement is a transformation, it will modify the input channel
   * unless its copy parameter is set to to true, in that case the modifications will
   * be reflected on a virtual channel.
   *
   * \param tagger        reference to a TimeTagger
   * \param input_channel channel to be filtered.
   * \param efficiency    efficiency of the transformation. a 0.5 efficiency will drop half the events.
   *                      A 1.0 won't drop any.
   * \param copy          tells if this transformation modifies the input or creates a new virtual channel
   *                      with the transformation.
   * \param seed          Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                      seed.
   */
  TransformEfficiency(TimeTaggerBase *tagger, channel_t input_channel, double efficiency, bool copy = false,
                      int32_t seed = -1);

  ~TransformEfficiency();

  channel_t getChannel();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

private:
  std::vector<Tag> mirror;

  const channel_t input_channel;
  const channel_t output_channel;

  const uint32_t limit;
  std::unique_ptr<PRBS> generator;
};

class TT_API TransformGaussianBroadening : public IteratorBase {
public:
  /**
   * \brief Construct a transformation that will apply gaussian brodening to each event in an
   * specified channel.
   *
   * @note this measurement is a transformation, it will modify the input channel
   * unless its copy parameter is set to to true, in that case the modifications will
   * be reflected on a virtual channel.
   *
   * @note-2 broadening will be limited to 5 times the standard deviation.
   *
   * \param tagger              reference to a TimeTagger
   * \param input_channel       channel to be transformed.
   * \param standard_deviation  gaussian standard deviation which will affect the broadening
   * \param copy                tells if this transformation modifies the input or creates a new virtual channel
   *                            with the transformation.
   * \param seed                Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                            seed.
   */
  TransformGaussianBroadening(TimeTaggerBase *tagger, channel_t input_channel, double standard_deviation,
                              bool copy = false, int32_t seed = -1);

  ~TransformGaussianBroadening();

  channel_t getChannel();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

private:
  std::vector<Tag> mirror;

  const channel_t input_channel;
  const channel_t output_channel;

  std::minstd_rand0 generator;
  std::normal_distribution<double> distr;

  bool overflow_state_on{};

  std::vector<Tag> accumulated_tags;
  timestamp_t delay{};
  std::deque<Tag> delayed_tags;
};

class TT_API TransformDeadtime : public IteratorBase {
public:
  /**
   * \brief Construct a transformation that will apply deadtime every event, filtering any events
   * within the deadtime period.
   *
   * @note this measurement is a transformation, it will modify the input channel
   * unless its copy parameter is set to to true, in that case the modifications will
   * be reflected on a virtual channel.
   *
   *
   * \param tagger              reference to a TimeTagger
   * \param input_channel       channel to transform.
   * \param deadtime            deadtime in seconds.
   * \param copy                tells if this transformation modifies the input or creates a new virtual channel
   *                            with the transformation.
   */
  TransformDeadtime(TimeTaggerBase *tagger, channel_t input_channel, double deadtime, bool copy = false);

  ~TransformDeadtime();

  channel_t getChannel();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

private:
  std::vector<Tag> mirror;

  const channel_t input_channel;
  const channel_t output_channel;

  timestamp_t deadtime{};
  timestamp_t last_gen_event{};
};

class TT_API TransformCrosstalk : public IteratorBase {
public:
  /**
   * \brief Construct a transformation that will apply crosstalk effect between an input
   * channel and a relay channel.
   *
   * @note this measurement is a transformation, it will modify the input channel
   * unless its copy parameter is set to to true, in that case the modifications will
   * be reflected on a virtual channel.
   *
   * \param tagger              reference to a TimeTagger
   * \param input_channel       channel to transform.
   * \param relay_input_channel channel that causes the delays
   * \param delay               amount of delay triggered by relay channel.
   * \param tau                 the decay after which an event of relay input channel has no effect anymore.
   * \param copy                tells if this transformation modifies the input or creates a new virtual channel
   *                            with the transformation.
   */
  TransformCrosstalk(TimeTaggerBase *tagger, channel_t input_channel, channel_t relay_input_channel, double delay,
                     double tau, bool copy = false);

  ~TransformCrosstalk();

  channel_t getChannel();

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

private:
  std::vector<Tag> mirror;

  const channel_t input_channel;
  const channel_t relay_input_channel;
  const channel_t output_channel;

  double delay{};
  double tau{};
  double accumulated_delay{};
  timestamp_t last_timestamp{};
  std::deque<Tag> delayed_tags;
  bool overflow_state_on{};
};

class TT_API SimDetector {
public:
  /**
   * \brief Construct a simulation of a physical detector for a given channel/signal.
   *
   * \param tagger              reference to a TimeTagger
   * \param input_channel       channel with the signal passing through the virtual detector
   * \param efficiency          rate of acceptance for inputs.
   * \param darkcount_rate      rate of noise in Herz.
   * \param jitter              standard deviation of the gaussian broadening, in seconds.
   * \param deadtime            deadtime, in seconds.
   * \param seed                Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                            seed.
   */
  SimDetector(TimeTaggerBase *tagger, channel_t input_channel, double efficiency = 1.0, double darkcount_rate = 0.0,
              double jitter = 0, double deadtime = 0.0, int32_t seed = -1);

  ~SimDetector();

  channel_t getChannel();

private:
  channel_t output_channel;
  std::unique_ptr<TransformEfficiency> efficiency_meas;
  std::unique_ptr<ExponentialSignalGenerator> added_noise_meas;
  std::unique_ptr<TransformGaussianBroadening> jitter_meas;
  std::unique_ptr<TransformDeadtime> deadtime_meas;
};

class TT_API SimLifetime : public IteratorBase {
public:
  /**
   * \brief Construct a simulation of a physical exaltation.
   *
   * \param tagger              reference to a TimeTagger
   * \param input_channel       channel which triggers the exaltation.
   * \param lifetime            lifetime of the exaltation.
   * \param emission_rate       poissonian emission rate for each input event.
   * \param seed                Seed number for the Pseudo-random number generator. Use -1 to use the current time as
   *                            seed.
   */
  SimLifetime(TimeTaggerBase *tagger, channel_t input_channel, double lifetime, double emission_rate = 0.1,
              int32_t seed = -1);

  ~SimLifetime();

  channel_t getChannel();

  void registerLifetimeReactor(channel_t trigger_channel, std::vector<double> lifetimes, bool repeat);

  void registerEmissionReactor(channel_t trigger_channel, std::vector<double> emissions, bool repeat);

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;

private:
  std::vector<Tag> mirror;

  const channel_t input_channel;
  const channel_t output_channel;

  std::minstd_rand0 generator;
  std::exponential_distribution<double> lifetime_distr;
  std::poisson_distribution<uint32_t> emission_distr;
  std::vector<Tag> accumulated_tags;
  bool overflow_state_on{};

  // Reactors.
  bool has_reactor{};

  std::vector<double> reactor_lifetimes;
  channel_t reactor_trigger_lifetimes;
  bool repeat_lifetimes;
  size_t current_index_lifetimes;

  std::vector<double> reactor_emissions;
  channel_t reactor_trigger_emissions;
  bool repeat_emissions;
  size_t current_index_emissions;
};

class PhotonNumberImpl;
/**
 * \ingroup VirtualChannels
 *
 * \brief Photon number resolution
 */
class TT_API PhotonNumber : public IteratorBase {
public:
  /**
   * \brief construct a PhotonNumber
   *
   * \param tagger      reference to a TimeTagger
   * \param trigger_ch  trigger channel
   * \param signal_start_ch   start-of-signal channel (likely rising edge)
   * \param signal_stop_ch   end-of-signal channel (likely falling edge)
   * \param slope       common slope of decision boundary lines
   * \param x_intercepts x-intercepts of decision boundary lines. Has to be in descending order
   * \param dead_time   the dead time of the detector
   */
  PhotonNumber(TimeTaggerBase *tagger, channel_t trigger_ch, channel_t signal_start_ch, channel_t signal_stop_ch,
               double slope, std::vector<double> x_intercepts, timestamp_t dead_time);

  ~PhotonNumber();

  /**
   * \brief the new virtual channels
   *
   * This function returns the IDs of the allocated virtual channels, corresponding to photon counts of \f$ 1, \ldots,
   * N, \geq N+1\f$ for given \f$N\f$ decision boundary lines.
   *
   *
   */
  std::vector<channel_t> const &getChannels() const;

protected:
  bool next_impl(std::vector<Tag> &incoming_tags, timestamp_t begin_time, timestamp_t end_time) override;
  void clear_impl() override;

private:
  friend class PhotonNumberImpl;
  std::unique_ptr<PhotonNumberImpl> impl;
};

} // namespace Experimental

#endif /* TT_ITERATORS_H_ */
