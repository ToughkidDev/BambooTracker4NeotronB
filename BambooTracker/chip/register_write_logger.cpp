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

#include "register_write_logger.hpp"
#include "io/export_io.hpp"
#include <algorithm>
#include <iterator>

namespace chip
{
AbstractRegisterWriteLogger::AbstractRegisterWriteLogger(int target)
	: target_(target),
	  lastWait_(0),
	  isSetLoop_(false),
	  loopPoint_(0),
	  totalSampCnt_(0)
{
}

void AbstractRegisterWriteLogger::elapse(size_t count) noexcept
{
	lastWait_ += count;
	totalSampCnt_ += count;
}

bool AbstractRegisterWriteLogger::empty() const noexcept
{
	return (buf_.empty() || lastWait_ != 0);
}

void AbstractRegisterWriteLogger::clear() noexcept
{
	buf_.clear();
	lastWait_ = 0;
	totalSampCnt_ = 0;
	isSetLoop_ = false;
	loopPoint_ = 0;
}

std::vector<uint8_t> AbstractRegisterWriteLogger::getData()
{
	if (lastWait_) setWait();
	return buf_;
}

size_t AbstractRegisterWriteLogger::getSampleLength() const noexcept
{
	return static_cast<size_t>(totalSampCnt_);
}

size_t AbstractRegisterWriteLogger::setLoopPoint()
{
	if (lastWait_) setWait();
	isSetLoop_ = true;
	return loopPoint_;
}

size_t AbstractRegisterWriteLogger::forceMoveLoopPoint() noexcept
{
	loopPoint_ = buf_.size();
	return loopPoint_;
}

//******************************//
VgmLogger::VgmLogger(int target, uint32_t intrRate)
	: AbstractRegisterWriteLogger(target), intrRate_(intrRate) {}

void VgmLogger::recordRegisterChange(uint32_t offset, uint8_t value)
{
	if (lastWait_) setWait();

	const int fm = target_ & io::Export_FmMask;
	const int ssg = target_ & io::Export_SsgMask;

	const uint8_t cmdSsg =
			(ssg != io::Export_InternalSsg) ? 0xa0
											: (fm == io::Export_YM2608) ? 0x56
																		: (fm == io::Export_YM2203) ? 0x55
																									: (fm == io::Export_YM2610B) ? 0x58
																																 : 0x00;
	const uint8_t cmdFmPortA =
			(fm == io::Export_YM2608) ? 0x56
									  : (fm == io::Export_YM2612) ? 0x52
																  : (fm == io::Export_YM2203) ? 0x55
																							  : (fm == io::Export_YM2610B) ? 0x58
																														   : 0x00;
	const uint8_t cmdFmPortB =
			(fm == io::Export_YM2608) ? 0x57
									  : (fm == io::Export_YM2612) ? 0x53
																  : (fm == io::Export_YM2610B) ? 0x59
																							   : 0x00;

	if (cmdSsg && offset < 0x10) {
		buf_.push_back(cmdSsg);
		buf_.push_back(static_cast<uint8_t>(offset));
		buf_.push_back(value);
	}
	else if (fm == io::Export_YM2610B && (offset & 0x100) == 0 && (offset & 0xf0) == 0x10) {
		// BambooTracker always drives its Rhythm channels using YM2608-native
		// register addresses (0x10-0x1f on port 0: keyon/off at 0x10, total
		// volume at 0x11, per-channel pan/level at 0x18-0x1d). On real
		// YM2610/YM2610B hardware the equivalent unit (ADPCM-A) is mapped to
		// port 1 instead, at the same relative register numbers (0x00-0x0d).
		// Remap port and address; see setRhythmAdpcmAData() for the matching
		// per-channel start/end address setup.
		buf_.push_back(cmdFmPortB); // 0x59: port 1
		buf_.push_back(static_cast<uint8_t>(offset - 0x10));
		buf_.push_back(value);
	}
	else if (fm == io::Export_YM2610B && (offset & 0x100) != 0 && (offset & 0xff) < 0x10) {
		// Conversely, BambooTracker's melodic ADPCM (Delta-T) writes use the
		// YM2608-native port 1 addresses 0x100-0x10f. On YM2610/YM2610B the
		// same Delta-T unit is mapped to port 0 instead, at addresses
		// 0x10-0x1f. Only the port and base address differ; the per-register
		// meaning (control/start/stop/delta-N/volume) is identical.
		buf_.push_back(cmdFmPortA); // 0x58: port 0
		buf_.push_back(static_cast<uint8_t>(0x10 + (offset & 0xff)));
		buf_.push_back(value);
	}
	else if (cmdFmPortA && (offset & 0x100) == 0) {
		bool compatible = true;

		if (offset == 0x28) { // Key register
			if (fm == io::Export_YM2203 && (value & 7) >= 3)
				compatible = false;
		}
		else if (offset == 0x29) // Mode register
			compatible = fm == io::Export_YM2608;
		else if ((offset & 0xf0) == 0x10) // Rhythm section
			compatible = fm == io::Export_YM2608;

		if (compatible) {
			buf_.push_back(cmdFmPortA);
			buf_.push_back(offset & 0xff);
			buf_.push_back(value);
		}
	}
	else if (cmdFmPortB && (offset & 0x100) != 0) {
		bool compatible = true;

		if ((offset & 0xff) < 0x10) // ADPCM section
			compatible = fm == io::Export_YM2608;

		if (compatible) {
			buf_.push_back(cmdFmPortB);
			buf_.push_back(offset & 0xff);
			buf_.push_back(value);
		}
	}
}

void VgmLogger::setDataBlock(std::vector<uint8_t> data, uint8_t blockType)
{
	buf_.push_back(0x67);
	buf_.push_back(0x66);
	buf_.push_back(blockType);
	size_t blockSize = data.size() + 8;
	buf_.push_back(blockSize & 0xff);
	buf_.push_back((blockSize >> 8) & 0xff);
	buf_.push_back((blockSize >> 16) & 0xff);
	buf_.push_back(blockSize >> 24);
	buf_.push_back(data.size() & 0xff);
	buf_.push_back((data.size() >> 8) & 0xff);
	buf_.push_back((data.size() >> 16) & 0xff);
	buf_.push_back(data.size() >> 24);
	buf_.resize(buf_.size() + 4);	// Start address is 0
	std::copy(data.begin(), data.end(), std::back_inserter(buf_));
}

void VgmLogger::setRhythmAdpcmAData(std::vector<uint8_t> rhythmRom)
{
	if ((target_ & io::Export_FmMask) != io::Export_YM2610B) return;

	// Byte ranges of the 6 rhythm samples within BambooTracker's built-in OPNA
	// rhythm ROM (chip/mame/fmopn.c's YM2608_ADPCM_ROM_addr): bass drum, snare
	// drum, top cymbal, high hat, tom tom, rim shot, in that channel order.
	static constexpr uint32_t sampleRanges[6][2] = {
		{ 0x0000, 0x01bf },
		{ 0x01c0, 0x043f },
		{ 0x0440, 0x1b7f },
		{ 0x1b80, 0x1cff },
		{ 0x1d00, 0x1f7f },
		{ 0x1f80, 0x1fff },
	};

	setDataBlock(std::move(rhythmRom), 0x82);	// YM2610 ADPCM ROM data

	// YM2610/YM2610B ADPCM-A address registers use 256-byte (1 << 8) units:
	// real byte address = register value << 8.
	constexpr uint32_t addressShift = 8;

	for (uint8_t ch = 0; ch < 6; ++ch) {
		uint32_t startReg = sampleRanges[ch][0] >> addressShift;
		uint32_t endReg = sampleRanges[ch][1] >> addressShift;

		auto writePort1 = [this](uint8_t reg, uint8_t value) {
			buf_.push_back(0x59);
			buf_.push_back(reg);
			buf_.push_back(value);
		};

		writePort1(static_cast<uint8_t>(0x10 + ch), static_cast<uint8_t>(startReg & 0xff));
		writePort1(static_cast<uint8_t>(0x18 + ch), static_cast<uint8_t>((startReg >> 8) & 0xff));
		writePort1(static_cast<uint8_t>(0x20 + ch), static_cast<uint8_t>(endReg & 0xff));
		writePort1(static_cast<uint8_t>(0x28 + ch), static_cast<uint8_t>((endReg >> 8) & 0xff));
	}
}

void VgmLogger::setWait()
{
	while (lastWait_) {
		uint32_t sub;

		if (intrRate_ == 50) {
			if (lastWait_ > 65535) {
				uint32_t tmp = static_cast<uint32_t>(lastWait_ - 65535);
				if (tmp <= 882) {
					//65535 - (882 - tmp)
					sub = 64653 + tmp;
				}
				else if (tmp <= 1764) {
					//65535 - (1764 - tmp)
					sub = 63771 + tmp;
				}
				else if (tmp <= 2646) {
					//65535 - (2646 - tmp)
					sub = 62889 + tmp;
				}
				else {
					sub = 65535;
				}
				buf_.push_back(0x61);
				buf_.push_back(sub & 0x00ff);
				buf_.push_back(static_cast<uint8_t>(sub >> 8));
			}
			else {
				if (lastWait_ <= 16) {
					buf_.push_back(static_cast<uint8_t>(0x70 | (lastWait_ - 1)));
				}
				else if (lastWait_ > 2646) {
					buf_.push_back(0x61);
					buf_.push_back(lastWait_ & 0x00ff);
					buf_.push_back(static_cast<uint8_t>(lastWait_ >> 8));
				}
				else if (lastWait_ == 2646) {
					buf_.push_back(0x63);
					buf_.push_back(0x63);
					buf_.push_back(0x63);
				}
				else if (1764 <= lastWait_ && lastWait_ <= 1780) {
					uint32_t tmp = static_cast<uint32_t>(lastWait_ - 1764);
					buf_.push_back(0x63);
					buf_.push_back(0x63);
					if (tmp) buf_.push_back(0x70 | (tmp - 1));
				}
				else if (882 <= lastWait_ && lastWait_ <= 898) {
					uint32_t tmp = static_cast<uint32_t>(lastWait_ - 882);
					buf_.push_back(0x63);
					if (tmp) buf_.push_back(0x70 | (tmp - 1));
				}
				else {
					buf_.push_back(0x61);
					buf_.push_back(lastWait_ & 0x00ff);
					buf_.push_back(static_cast<uint8_t>(lastWait_ >> 8));
				}
				sub = static_cast<uint32_t>(lastWait_);
			}
		}
		else if (intrRate_ == 60) {
			if (lastWait_ > 65535) {
				uint32_t tmp = static_cast<uint32_t>(lastWait_ - 65535);
				if (tmp <= 735) {
					//65535 - (735 - tmp)
					sub = 64800 + tmp;
				}
				else if (tmp <= 1470) {
					//65535 - (1470 - tmp)
					sub = 64065 + tmp;
				}
				else if (tmp <= 2205) {
					//65535 - (2205 - tmp)
					sub = 63330 + tmp;
				}
				else {
					sub = 65535;
				}
				buf_.push_back(0x61);
				buf_.push_back(sub & 0x00ff);
				buf_.push_back(sub >> 8);
			}
			else {
				if (lastWait_ <= 16) {
					buf_.push_back(static_cast<uint8_t>(0x70 | (lastWait_ - 1)));
				}
				else if (lastWait_ > 2205) {
					buf_.push_back(0x61);
					buf_.push_back(lastWait_ & 0x00ff);
					buf_.push_back(static_cast<uint8_t>(lastWait_ >> 8));
				}
				else if (lastWait_ == 2205) {
					buf_.push_back(0x62);
					buf_.push_back(0x62);
					buf_.push_back(0x62);
				}
				else if (1470 <= lastWait_ && lastWait_ <= 1486) {
					uint32_t tmp = static_cast<uint32_t>(lastWait_ - 1470);
					buf_.push_back(0x62);
					buf_.push_back(0x62);
					if (tmp) buf_.push_back(0x70 | (tmp - 1));
				}
				else if (735 <= lastWait_ && lastWait_ <= 751) {
					uint32_t tmp = static_cast<uint32_t>(lastWait_ - 735);
					buf_.push_back(0x62);
					if (tmp) buf_.push_back(0x70 | (tmp - 1));
				}
				else {
					buf_.push_back(0x61);
					buf_.push_back(lastWait_ & 0x00ff);
					buf_.push_back(static_cast<uint8_t>(lastWait_ >> 8));
				}
				sub = static_cast<uint32_t>(lastWait_);
			}
		}
		else {
			if (lastWait_ > 65535) {
				sub = 65535;
				buf_.push_back(0x61);
				buf_.push_back(sub & 0x00ff);
				buf_.push_back(sub >> 8);
			}
			else {
				buf_.push_back(0x61);
				buf_.push_back(lastWait_ & 0x00ff);
				buf_.push_back(static_cast<uint8_t>(lastWait_ >> 8));
			}
			sub = static_cast<uint32_t>(lastWait_);
		}

		lastWait_ -= sub;
	}

	if (!isSetLoop_) loopPoint_ = buf_.size();
}

//******************************//
S98Logger::S98Logger(int target) : AbstractRegisterWriteLogger(target) {}

void S98Logger::recordRegisterChange(uint32_t offset, uint8_t value)
{
	if (lastWait_) setWait();

	const int fm = target_ & io::Export_FmMask;
	const int ssg = target_ & io::Export_SsgMask;

	const uint8_t cmdSsg =
			(ssg != io::Export_InternalSsg) ? (fm == io::Export_NoneFm) ? 0x01 : 0x02
																		: (fm == io::Export_YM2608) ? 0x00
																									: (fm == io::Export_YM2203) ? 0x00
																																: 0xff;
	const uint8_t cmdFmPortA =
			(fm != io::Export_NoneFm) ? 0x00 : 0xff;
	const uint8_t cmdFmPortB =
			(fm == io::Export_YM2608 || fm == io::Export_YM2612) ? 0x01 : 0xff;

	if (cmdSsg != 0xff && offset < 0x10) {
		buf_.push_back(cmdSsg);
		buf_.push_back(offset);
		buf_.push_back(value);
	}
	else if (cmdFmPortA != 0xff && (offset & 0x100) == 0) {
		bool compatible = true;

		if (offset == 0x28) { // Key register
			if (fm == io::Export_YM2203 && (value & 7) >= 3)
				compatible = false;
		}
		else if (offset == 0x29) // Mode register
			compatible = fm == io::Export_YM2608;
		else if ((offset & 0xf0) == 0x10) // Rhythm section
			compatible = fm == io::Export_YM2608;

		if (compatible) {
			buf_.push_back(cmdFmPortA);
			buf_.push_back(offset & 0xff);
			buf_.push_back(value);
		}
	}
	else if (cmdFmPortB != 0xff && (offset & 0x100) != 0) {
		bool compatible = true;

		if (offset < 0x10) // ADPCM section
			compatible = fm == io::Export_YM2608;

		if (compatible) {
			buf_.push_back(cmdFmPortB);
			buf_.push_back(offset & 0xff);
			buf_.push_back(value);
		}
	}
}

void S98Logger::setWait()
{
	if (lastWait_ == 1) {
		buf_.push_back(0xff);
	}
	else {
		buf_.push_back(0xfe);
		lastWait_ -= 2;
		do {
			uint8_t b = lastWait_ & 0x7f;
			lastWait_ >>= 7;
			if (lastWait_ > 0) b |= 0x80;
			buf_.push_back(b);
		} while (lastWait_ > 0);
	}
	if (!isSetLoop_) loopPoint_ = buf_.size();
	lastWait_ = 0;
}
}
