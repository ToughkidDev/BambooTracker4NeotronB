/*
 * Copyright (C) 2018-2020 BambooTracker contributors
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <unordered_map>

namespace chip
{
class AbstractRegisterWriteLogger
{
public:
	explicit AbstractRegisterWriteLogger(int target);
	virtual ~AbstractRegisterWriteLogger() = default;
	virtual void recordRegisterChange(uint32_t offset, uint8_t value) = 0;
	void elapse(size_t count) noexcept;
	bool empty() const noexcept;
	void clear() noexcept;
	std::vector<uint8_t> getData();
	size_t getSampleLength() const noexcept;
	size_t setLoopPoint();
	size_t forceMoveLoopPoint() noexcept;

protected:
	int target_;
	std::vector<uint8_t> buf_;
	uint64_t lastWait_;
	bool isSetLoop_;
	uint32_t loopPoint_;

	virtual void setWait() = 0;

private:
	uint64_t totalSampCnt_;
};

class VgmLogger final : public AbstractRegisterWriteLogger
{
public:
	VgmLogger(int target, uint32_t intrRate);
	void recordRegisterChange(uint32_t offset, uint8_t value) override;

	/**
	 * @brief Embed a ROM/RAM data block (VGM command 0x67 0x66) into the stream.
	 * @param data Raw ROM bytes.
	 * @param blockType VGM data block type ID (e.g. 0x81 = YM2608 DELTA-T ROM,
	 *		  0x82 = YM2610 ADPCM ROM, 0x83 = YM2610 DELTA-T ROM).
	 */
	void setDataBlock(std::vector<uint8_t> data, uint8_t blockType = 0x81);

	/**
	 * @brief Set up the YM2610/YM2610B ADPCM-A ("Rhythm") unit so it reproduces
	 *		  BambooTracker's built-in OPNA rhythm samples.
	 *
	 * Unlike the YM2608, which plays back 6 fixed samples from its internal ROM
	 * using only a handful of control registers, the YM2610/YM2610B ADPCM-A unit
	 * has no built-in samples: each of its 6 channels has a fully programmable
	 * start/end address into an external sample ROM that must be supplied by the
	 * VGM file itself. This method embeds BambooTracker's own OPNA rhythm ROM as
	 * a 0x82 data block and programs each channel's start/end address registers
	 * to point at the same 6 sample regions used by the YM2608 rhythm unit, so
	 * that keyon/volume/pan writes recorded through the normal YM2608-native
	 * Rhythm register path (0x10-0x1f) keep working after being remapped.
	 *
	 * No-op unless the export target is YM2610B.
	 *
	 * @param rhythmRom Raw bytes of BambooTracker's built-in OPNA rhythm ROM
	 *		  (chip::mame's `YM2608_ADPCM_ROM`, 0x2000 bytes).
	 */
	void setRhythmAdpcmAData(std::vector<uint8_t> rhythmRom);

	/**
	 * @brief Provide an exact address translation table for YM2610B Delta-T
	 *		  start/stop register values, used instead of a blind bit-shift.
	 *
	 * BambooTracker packs its own (YM2608-native) ADPCM sample RAM tightly at
	 * 32-byte granularity, so consecutive samples' start offsets generally
	 * are NOT aligned to YM2610/YM2610B's coarser 256-byte address-register
	 * granularity. Naively rescaling a YM2608-native register value by >>3
	 * (32/8 = 8x) rounds it down to the nearest 256-byte boundary in place --
	 * with no gap between samples, that can land inside the *previous*
	 * sample's data instead of the intended one. The export path builds a
	 * separate YM2610B ROM image with every sample re-aligned to start on
	 * its own 256-byte boundary (mirroring setRhythmAdpcmAData()) and passes
	 * the resulting exact (YM2608-native register value -> YM2610B-native
	 * register value) mapping here. recordRegisterChange() looks up each
	 * observed start/stop value in this table; a value with no entry (e.g.
	 * the whole-DRAM limit register, which isn't tied to one sample) still
	 * falls back to the >>3 approximation.
	 *
	 * No-op unless the export target is YM2610B.
	 *
	 * @param addrMap Map from YM2608-native (32-byte-unit) register value to
	 *		  the exact YM2610B-native (256-byte-unit) register value.
	 */
	void setDeltaTAddressMap(std::unordered_map<uint16_t, uint16_t> addrMap);

	/**
	 * @brief Set the linear amplitude ratio applied to SSG channel volume
	 *		  register writes (offsets 0x08-0x0a: channel A/B/C amplitude).
	 *
	 * The VGM spec's "Chip Volume" extra header would be the tidier way to
	 * carry this, but it's an optional VGM 1.70+ feature that most real
	 * players (e.g. Game_Music_Emu, which several popular players including
	 * Cog are built on) never implemented -- they just skip it, so a gain
	 * set that way is silently inaudible on them. Baking the ratio directly
	 * into the SSG amplitude register values that get written to the VGM
	 * command stream instead works on every player, since it's just an
	 * ordinary register write like any other.
	 *
	 * Only affects fixed-level channels (envelope bit, 0x10, clear); a
	 * channel using the hardware envelope generator for its volume can't be
	 * rescaled by editing this register alone, so those are left untouched.
	 *
	 * @param ratio Linear amplitude multiplier (1.0 = unchanged, e.g. from
	 *		  10^(dB/20)). Pass 1.0 (the default) to disable scaling.
	 */
	void setSsgGain(double ratio) noexcept;

private:
	uint32_t intrRate_;

	// Shadow copy of BambooTracker's YM2608-native Delta-T register bank
	// (offsets 0x00-0x0f on port 1), used only when the export target is
	// YM2610B. Needed to reconstruct the 16-bit start/stop/limit address
	// register pairs (low byte + high byte are written as separate VGM
	// commands) so they can be rescaled before being remapped; see the
	// comment above the YM2610B Delta-T branch in recordRegisterChange().
	uint8_t deltaTRegShadow_[0x10] = {};

	// Exact YM2608-native -> YM2610B-native Delta-T address register value
	// map; see setDeltaTAddressMap().
	std::unordered_map<uint16_t, uint16_t> deltaTAddrMap_;

	double ssgGainRatio_ = 1.0;
	uint8_t scaleSsgVolumeReg(uint8_t regValue) const;

	void setWait() override;
};

class S98Logger final : public AbstractRegisterWriteLogger
{
public:
	explicit S98Logger(int target);
	void recordRegisterChange(uint32_t offset, uint8_t value) override;

private:
	void setWait() override;
};
}
