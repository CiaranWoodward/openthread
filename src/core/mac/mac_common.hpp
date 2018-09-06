/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file includes definitions for the IEEE 802.15.4 MAC.
 */

#ifndef MAC_COMMON_HPP_
#define MAC_COMMON_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/radio-phy.h>
#include <openthread/platform/time.h>

#include "common/locator.hpp"
#include "common/tasklet.hpp"
#include "common/timer.hpp"
#include "mac/mac_filter.hpp"
#include "mac/mac_frame.hpp"
#include "thread/key_manager.hpp"
#include "thread/link_quality.hpp"
#include "thread/network_diagnostic_tlvs.hpp"
#include "thread/topology.hpp"

namespace ot {

/**
 * @addtogroup core-mac
 *
 * @brief
 *   This module includes definitions for the IEEE 802.15.4 MAC
 *
 * @{
 *
 */

namespace Mac {

/**
 * Protocol parameters and constants.
 *
 */
enum
{
    kMinBE             = 3,  ///< macMinBE (IEEE 802.15.4-2006).
    kMaxBE             = 5,  ///< macMaxBE (IEEE 802.15.4-2006).
    kUnitBackoffPeriod = 20, ///< Number of symbols (IEEE 802.15.4-2006).

    kMinBackoff = 1, ///< Minimum backoff (milliseconds).

    kAckTimeout      = 16,  ///< Timeout for waiting on an ACK (milliseconds).
    kDataPollTimeout = 100, ///< Timeout for receiving Data Frame (milliseconds).
    kSleepDelay      = 300, ///< Max sleep delay when frame is pending (milliseconds).
    kNonceSize       = 13,  ///< Size of IEEE 802.15.4 Nonce (bytes).

    kScanChannelsAll     = OT_CHANNEL_ALL, ///< All channels.
    kScanDurationDefault = 300,            ///< Default interval between channels (milliseconds).

    kMaxCsmaBackoffsDirect =
        OPENTHREAD_CONFIG_MAC_MAX_CSMA_BACKOFFS_DIRECT, ///< macMaxCsmaBackoffs for direct transmissions
    kMaxCsmaBackoffsIndirect =
        OPENTHREAD_CONFIG_MAC_MAX_CSMA_BACKOFFS_INDIRECT, ///< macMaxCsmaBackoffs for indirect transmissions

    kMaxFrameRetriesDirect =
        OPENTHREAD_CONFIG_MAC_MAX_FRAME_RETRIES_DIRECT, ///< macMaxFrameRetries for direct transmissions
    kMaxFrameRetriesIndirect =
        OPENTHREAD_CONFIG_MAC_MAX_FRAME_RETRIES_INDIRECT, ///< macMaxFrameRetries for indirect transmissions

    kTxNumBcast = OPENTHREAD_CONFIG_TX_NUM_BCAST ///< Number of times each broadcast frame is transmitted
};

/**
 * This class defines a channel mask.
 *
 * It is a wrapper class around a `uint32_t` bit vector representing a set of channels.
 *
 */
class ChannelMask
{
public:
    enum
    {
        kChannelIteratorFirst = 0xff, ///< Value to pass in `GetNextChannel()` to get the first channel in the mask.
        kInfoStringSize       = 45,   ///< Recommended buffer size to use with `ToString()`.
    };

    /**
     * This type defines the fixed-length `String` object returned from `ToString()`.
     *
     */
    typedef String<kInfoStringSize> InfoString;

    /**
     * This constructor initializes a `ChannelMask` instance.
     *
     */
    ChannelMask(void)
        : mMask(0)
    {
    }

    /**
     * This constructor initializes a `ChannelMask` instance with a given mask.
     *
     * @param[in]  aMask   A channel mask (as a `uint32_t` bit-vector mask with bit 0 (lsb) -> channel 0, and so on).
     *
     */
    ChannelMask(uint32_t aMask)
        : mMask(aMask)
    {
    }

    /**
     * This method clears the channel mask.
     *
     */
    void Clear(void) { mMask = 0; }

    /**
     * This method gets the channel mask (as a `uint32_t` bit-vector mask with bit 0 (lsb) -> channel 0, and so on).
     *
     * @returns The channel mask.
     *
     */
    uint32_t GetMask(void) const { return mMask; }

    /**
     * This method sets the channel mask.
     *
     * @param[in]  aMask   A channel mask (as a `uint32_t` bit-vector mask with bit 0 (lsb) -> channel 0, and so on).
     *
     */
    void SetMask(uint32_t aMask) { mMask = aMask; }

    /**
     * This method indicates if the mask is empty.
     *
     * @returns TRUE if the mask is empty, FALSE otherwise.
     *
     */
    bool IsEmpty(void) const { return (mMask == 0); }

    /**
     * This method indicates if the mask contains only a single channel.
     *
     * @returns TRUE if channel mask contains a single channel, FALSE otherwise
     *
     */
    bool IsSingleChannel(void) const { return ((mMask != 0) && ((mMask & (mMask - 1)) == 0)); }

    /**
     * This method indicates if the mask contains a given channel.
     *
     * @param[in]  aChannel  A channel.
     *
     * @returns TRUE if the channel @p aChannel is included in the mask, FALSE otherwise.
     *
     */
    bool ContainsChannel(uint8_t aChannel) const { return ((1U << aChannel) & mMask) != 0; }

    /**
     * This method adds a channel to the channel mask.
     *
     * @param[in]  aChannel  A channel
     *
     */
    void AddChannel(uint8_t aChannel) { mMask |= (1U << aChannel); }

    /**
     * This method removes a channel from the channel mask.
     *
     * @param[in]  aChannel  A channel
     *
     */
    void RemoveChannel(uint8_t aChannel) { mMask &= ~(1U << aChannel); }

    /**
     * This method updates the channel mask by intersecting it with another mask.
     *
     * @param[in]  aOtherMask  Another channel mask.
     *
     */
    void Intersect(const ChannelMask &aOtherMask) { mMask &= aOtherMask.mMask; }

    /**
     * This method returns the number of channels in the mask.
     *
     * @returns Number of channels in the mask.
     *
     */
    uint8_t GetNumberOfChannels(void) const;

    /**
     * This method gets the next channel in the channel mask.
     *
     * This method can be used to iterate over all channels in the channel mask. To get the first channel (channel with
     * lowest number) in the mask the @p aChannel should be set to `kChannelIteratorFirst`.
     *
     * @param[inout] aChannel        A reference to a `uint8_t`.
     *                               On entry it should contain the previous channel or `kChannelIteratorFirst`.
     *                               On exit it contains the next channel.
     *
     * @retval  OT_ERROR_NONE        Got the next channel, @p aChannel updated successfully.
     * @retval  OT_ERROR_NOT_FOUND   No next channel in the channel mask (note: @p aChannel may be changed).
     *
     */
    otError GetNextChannel(uint8_t &aChannel) const;

    /**
     * This method overloads `==` operator to indicate whether two masks are equal.
     *
     * @param[in] aAnother   A reference to another mask to compare with the current one.
     *
     * @returns TRUE if the two masks are equal, FALSE otherwise.
     *
     */
    bool operator==(const ChannelMask &aAnother) const { return (mMask == aAnother.mMask); }

    /**
     * This method overloads `!=` operator to indicate whether two masks are different.
     *
     * @param[in] aAnother     A reference to another mask to compare with the current one.
     *
     * @returns TRUE if the two masks are different, FALSE otherwise.
     *
     */
    bool operator!=(const ChannelMask &aAnother) const { return (mMask != aAnother.mMask); }

    /**
     * This method converts the channel mask into a human-readable string.
     *
     * Examples of possible output:
     *  -  empty mask      ->  "{ }"
     *  -  all channels    ->  "{ 11-26 }"
     *  -  single channel  ->  "{ 20 }"
     *  -  multiple ranges ->  "{ 11, 14-17, 20-22, 24, 25 }"
     *  -  no range        ->  "{ 14, 21, 26 }"
     *
     * @returns  An `InfoString` object representing the channel mask.
     *
     */
    InfoString ToString(void) const;

private:
#if (OT_RADIO_CHANNEL_MIN >= 32) || (OT_RADIO_CHANNEL_MAX >= 32)
#error `OT_RADIO_CHANNEL_MAX` or `OT_RADIO_CHANNEL_MIN` are larger than 32. `ChannelMask` uses 32 bit mask.
#endif

    uint32_t mMask;
};

/**
 * @}
 *
 */

} // namespace Mac
} // namespace ot

#endif // MAC_COMMON_HPP_
