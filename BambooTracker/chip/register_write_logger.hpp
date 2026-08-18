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

private:
	uint32_t intrRate_;

	// Shadow copy of BambooTracker's YM2608-native Delta-T register bank
	// (offsets 0x00-0x0f on port 1), used only when the export target is
	// YM2610B. Needed to reconstruct the 16-bit start/stop/limit address
	// register pairs (low byte + high byte are written as separate VGM
	// commands) so they can be rescaled before being remapped; see the
	// comment above the YM2610B Delta-T branch in recordRegisterChange().
	uint8_t deltaTRegShadow_[0x10] = {};

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
