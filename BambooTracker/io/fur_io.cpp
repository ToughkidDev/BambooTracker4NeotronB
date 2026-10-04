/*
 * Copyright (C) 2026 BambooTracker contributors
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

#include "fur_io.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include "binary_container.hpp"
#include "module.hpp"
#include "instruments_manager.hpp"
#include "instrument.hpp"
#include "effect.hpp"
#include "note.hpp"
#include "bamboo_tracker_defs.hpp"
#include "chip/register_write_logger.hpp"

/*
 * Furnace module writer.
 *
 * The layout below mirrors DivEngine::saveFur() (and the putData() helpers it
 * calls) of Furnace 0.6.8.3, which writes format version 232: the "INFO"
 * song header followed by "FLAG", "ADIR", "INS2", "SMP2" and "PATN" blocks.
 * Version 232 is used instead of Furnace's newer "INF2" layout (>= 240),
 * which only development builds can read.
 */

namespace io
{
namespace
{
constexpr uint16_t FUR_VERSION = 232;
constexpr int FUR_MAX_CHIPS = 32;
constexpr int FUR_MAX_MACRO_LEN = 255;
constexpr int FUR_MAX_EFFECT_COLS = 8;
constexpr int FUR_MAX_PATTERNS = 256;
constexpr int FUR_MAX_INSTRUMENTS = 256;
constexpr int FUR_MAX_SAMPLES = 256;
constexpr int FUR_NOTE_MAP_SIZE = 120;		// Instrument sample map entries (C-0 to B-9)

// Pattern note values.
constexpr int FUR_NOTE_C0 = 60;				// 0 is C-(-5), 60 is C-0
constexpr int FUR_NOTE_OFF = 180;
constexpr int FUR_NOTE_RELEASE = 181;		// Key off and macro release

// System IDs (.fur file values).
constexpr uint8_t FUR_SYS_YM2608 = 0x8e;
constexpr uint8_t FUR_SYS_YM2608_EXT = 0xb7;
constexpr uint8_t FUR_SYS_YM2610B = 0x9e;
constexpr uint8_t FUR_SYS_YM2610B_EXT = 0xde;

// Instrument types.
constexpr uint8_t FUR_INS_FM = 1;
constexpr uint8_t FUR_INS_AY = 6;
constexpr uint8_t FUR_INS_ADPCMA = 37;
constexpr uint8_t FUR_INS_ADPCMB = 38;

// Sample depths.
constexpr uint8_t FUR_SAMPLE_ADPCM_A = 5;
constexpr uint8_t FUR_SAMPLE_ADPCM_B = 6;

// Macro codes.
namespace MacroCode
{
enum : uint8_t
{
	Vol = 0, Arp = 1, Duty = 2, Wave = 3, Pitch = 4, Ex1 = 5, Ex2 = 6, Ex3 = 7,
	Alg = 8, Fb = 9, Fms = 10, Ams = 11, PanL = 12, PanR = 13, PhaseReset = 14,
	Ex4 = 15, Ex5 = 16
};
}

// Operator macro codes.
namespace OpMacroCode
{
enum : uint8_t
{
	AM = 0, AR = 1, DR = 2, MULT = 3, RR = 4, SL = 5, TL = 6, DT2 = 7, RS = 8, DT = 9,
	D2R = 10, SSG = 11
};
}

// Furnace stores OPN operators in register order (1, 3, 2, 4).
// Furnace operator slot -> BambooTracker operator index.
constexpr int BT_OP_OF_FUR_OP[4] = { 0, 2, 1, 3 };
// BambooTracker operator index -> Furnace operator slot.
constexpr int FUR_OP_OF_BT_OP[4] = { 0, 2, 1, 3 };

// BambooTracker stores the raw DT register value; Furnace uses 0-7 with 3 as center.
constexpr int FUR_DT_OF_REG_DT[8] = { 3, 4, 5, 6, 3, 2, 1, 0 };

// YM2608 ADPCM (Delta-T) sample rate for DELTA-N = 65536: master clock / 144.
constexpr double ADPCMB_FULL_RATE = 3993600.0 * 2 / 144;
// YM2610/YM2610B ADPCM-A playback rate (8 MHz / 432).
constexpr int ADPCMA_RATE = 18518;

// BambooTracker computes SSG tone periods for a clock one octave above the
// note names (A4 plays at 880 Hz), while Furnace plays notes at their names.
constexpr int SSG_TRANSPOSE = 12;

// Furnace's SSG output is this much louder relative to FM than BambooTracker's
// at a flat mixer setting (measured with Furnace 0.6.8.3, YM2608 and YM2610B).
constexpr double FUR_SSG_LEVEL_OFFSET_DB = 3.4;
constexpr int FUR_SSG_VOL_UNITY = 128;	// Default "ssgVol" chip setting

// BambooTracker's tick counter derives step lengths from "10 * tickRate / 4 / tempo".
constexpr double BT_TEMPO_TICK_FACTOR = 2.5;

const char* const RHYTHM_NAMES[6] = { "Bass drum", "Snare drum", "Top cymbal", "Hi-hat", "Tom", "Rim shot" };

// Byte ranges (inclusive) of the 6 rhythm samples in the built-in OPNA rhythm ROM,
// in BambooTracker's rhythm track order (see chip::VgmLogger::setRhythmAdpcmAData()).
constexpr uint32_t RHYTHM_ROM_RANGES[6][2] = {
	{ 0x0000, 0x01bf },
	{ 0x01c0, 0x043f },
	{ 0x0440, 0x1b7f },
	{ 0x1b80, 0x1cff },
	{ 0x1d00, 0x1f7f },
	{ 0x1f80, 0x1fff }
};

template<typename T>
inline T clampValue(T v, T lo, T hi)
{
	return std::max(lo, std::min(v, hi));
}

/********** Binary helpers **********/
void appendFloat(BinaryContainer& ctr, float v)
{
	uint32_t u;
	std::memcpy(&u, &v, sizeof(u));
	ctr.appendUint32(u);
}

void appendCString(BinaryContainer& ctr, const std::string& str)
{
	if (!str.empty()) ctr.appendString(str);
	ctr.appendUint8(0);
}

/// Write a 4-character block ID and a dummy size, returning the size offset.
size_t beginBlock(BinaryContainer& ctr, const char* id)
{
	ctr.appendString(std::string(id, 4));
	size_t ofs = ctr.size();
	ctr.appendUint32(0);
	return ofs;
}

void endBlock(BinaryContainer& ctr, size_t sizeOfs)
{
	ctr.writeUint32(sizeOfs, static_cast<uint32_t>(ctr.size() - sizeOfs - 4));
}

/// Write a 2-character instrument feature code and a dummy size, returning the size offset.
size_t beginFeature(BinaryContainer& ctr, const char* code)
{
	ctr.appendString(std::string(code, 2));
	size_t ofs = ctr.size();
	ctr.appendUint16(0);
	return ofs;
}

void endFeature(BinaryContainer& ctr, size_t sizeOfs)
{
	ctr.writeUint16(sizeOfs, static_cast<uint16_t>(ctr.size() - sizeOfs - 2));
}

/********** Data model of the written module **********/
struct FurMacro
{
	uint8_t code = 0;
	std::vector<int> val;
	int loop = -1;
	int rel = -1;
	uint8_t mode = 0;
	uint8_t delay = 0;
	bool instantRelease = false;
};

struct FurOperator
{
	bool enable = true;
	int ar = 31, dr = 0, d2r = 0, rr = 15, sl = 0, tl = 0, rs = 0, mult = 1, dt = 3, ssg = 0;
	bool am = false;
};

struct FurInstrument
{
	std::string name;
	uint8_t type = 0;

	bool hasFM = false;
	int alg = 0, fb = 0, fms = 0, ams = 0;
	std::array<FurOperator, 4> op;

	bool hasSampleMap = false;
	int initSample = -1;
	bool useNoteMap = false;
	std::array<std::pair<int, int>, FUR_NOTE_MAP_SIZE> noteMap;	// (note to play, sample)

	std::vector<FurMacro> macros;
	std::array<std::vector<FurMacro>, 4> opMacros;
};

struct FurSample
{
	std::string name;
	uint8_t depth = 0;
	std::vector<uint8_t> data;
	int rate = 0;
	int loopStart = -1;
	int loopEnd = -1;
};

struct FurRow
{
	int note = -1;
	int ins = -1;
	int vol = -1;
	std::vector<std::pair<int, int>> fx;
};

using FurPattern = std::vector<FurRow>;

/********** Instrument sequences to macros **********/
/**
 * A BambooTracker instrument sequence played through its iterator: the values
 * output each tick before key off ("pre") and after it ("rel"), each with an
 * optional loop start index. Nested and counted loops are flattened.
 */
template<class T>
struct Unrolled
{
	std::vector<T> pre, rel;
	int preLoop = -1, relLoop = -1;
};

constexpr size_t TRACE_LENGTH = 4096;

/// Run an iterator and record its output, detecting the period of an infinite loop.
template<class IterPtr, class T>
void traceIterator(IterPtr& it, std::vector<T>& vals, int& loop)
{
	std::vector<int> pos;
	vals.clear();
	loop = -1;
	while (!it->hasEnded() && pos.size() < TRACE_LENGTH) {
		pos.push_back(it->pos());
		vals.push_back(it->data());
		it->next();
	}
	if (it->hasEnded()) return;

	// The trace is eventually periodic. Compute the Z-function of the reversed
	// position trace; the smallest shift that matches over at least half the
	// trace is the period of the tail, and the match length gives its start.
	const size_t n = pos.size();
	std::vector<int> r(pos.rbegin(), pos.rend());
	std::vector<size_t> z(n, 0);
	for (size_t i = 1, lft = 0, rgt = 0; i < n; ++i) {
		if (i < rgt) z[i] = std::min(rgt - i, z[i - lft]);
		while (i + z[i] < n && r[z[i]] == r[i + z[i]]) ++z[i];
		if (i + z[i] > rgt) {
			lft = i;
			rgt = i + z[i];
		}
	}
	for (size_t p = 1; p < n; ++p) {
		if (p + z[p] >= n / 2) {
			size_t start = n - (p + z[p]);
			vals.resize(start + p);
			loop = static_cast<int>(start);
			return;
		}
	}
	// No period detected within the trace; keep it as a one-shot sequence.
}

template<class IterPtr, class Factory>
auto unrollSequence(Factory makeIter) -> Unrolled<decltype(makeIter()->data())>
{
	Unrolled<decltype(makeIter()->data())> u;
	IterPtr it = makeIter();
	it->front();
	traceIterator(it, u.pre, u.preLoop);

	IterPtr itRel = makeIter();
	itRel->front();
	itRel->release();
	if (!itRel->hasEnded()) traceIterator(itRel, u.rel, u.relLoop);
	return u;
}

struct PhaseLayout
{
	size_t length;
	int loop;	// -1: no loop
};

/**
 * Lay several sequences side by side as they would play simultaneously.
 * Returns, for each combined tick, the index into each sequence (-1 if the
 * sequence is empty), and sets the loop start of the combined sequence.
 */
std::vector<std::vector<int>> combinePhases(const std::vector<PhaseLayout>& seqs, int& loop)
{
	size_t start = 0;
	size_t period = 1;
	size_t maxLen = 0;
	bool hasLoop = false;
	for (const auto& s : seqs) {
		if (!s.length) continue;
		maxLen = std::max(maxLen, s.length);
		if (s.loop >= 0) {
			hasLoop = true;
			start = std::max(start, static_cast<size_t>(s.loop));
			size_t p = s.length - static_cast<size_t>(s.loop);
			period = period / std::gcd(period, p) * p;
			if (period > FUR_MAX_MACRO_LEN) period = FUR_MAX_MACRO_LEN;
		}
		else {
			start = std::max(start, s.length - 1);
		}
	}

	size_t total;
	if (hasLoop) {
		total = start + period;
		loop = static_cast<int>(start);
	}
	else {
		total = maxLen;
		loop = -1;
	}
	if (total > FUR_MAX_MACRO_LEN) {
		total = FUR_MAX_MACRO_LEN;
		if (loop >= FUR_MAX_MACRO_LEN) loop = -1;
	}

	std::vector<std::vector<int>> idx(total, std::vector<int>(seqs.size(), -1));
	for (size_t t = 0; t < total; ++t) {
		for (size_t i = 0; i < seqs.size(); ++i) {
			const auto& s = seqs[i];
			if (!s.length) continue;
			size_t k;
			if (t < s.length) k = t;
			else if (s.loop >= 0) k = s.loop + (t - s.loop) % (s.length - s.loop);
			else k = s.length - 1;
			idx[t][i] = static_cast<int>(k);
		}
	}
	return idx;
}

/**
 * Build a Furnace macro from the combined pre- and post-key-off values.
 *
 * Furnace sustains a macro up to and including its release index, looping
 * back to the loop index while held, and jumps to the release index on key
 * off when "instant release" is set. Placing the release index on the last
 * pre-key-off value therefore reproduces BambooTracker's behavior of jumping
 * to the release part of every sequence at key off.
 */
FurMacro buildMacro(uint8_t code, std::vector<int> pre, int preLoop, const std::vector<int>& rel, int relLoop)
{
	FurMacro m;
	m.code = code;
	if (pre.empty()) {
		if (rel.empty()) return m;
		pre.push_back(rel.front());
		preLoop = -1;
	}
	const int a = static_cast<int>(pre.size());
	m.val = std::move(pre);
	if (!rel.empty()) {
		m.val.insert(m.val.end(), rel.begin(), rel.end());
		m.rel = a - 1;
		m.instantRelease = true;
		m.loop = (preLoop >= 0) ? preLoop : ((relLoop >= 0) ? a + relLoop : -1);
	}
	else if (preLoop >= 0) {
		// BambooTracker stops a sequence without release part at key off.
		m.loop = preLoop;
		if (preLoop < a - 1) {
			m.rel = a - 1;
			m.instantRelease = true;
		}
	}

	if (m.val.size() > FUR_MAX_MACRO_LEN) m.val.resize(FUR_MAX_MACRO_LEN);
	const int len = static_cast<int>(m.val.size());
	if (m.loop >= len) m.loop = -1;
	if (m.rel >= len) {
		m.rel = -1;
		m.instantRelease = false;
	}
	return m;
}

/// Build a macro from a single unrolled sequence, mapping each value with conv.
template<class T, class Conv>
FurMacro macroFromSequence(uint8_t code, const Unrolled<T>& u, Conv conv)
{
	std::vector<int> pre, rel;
	for (const auto& v : u.pre) pre.push_back(conv(v));
	for (const auto& v : u.rel) rel.push_back(conv(v));
	return buildMacro(code, std::move(pre), u.preLoop, rel, u.relLoop);
}

/// Arpeggio sequences: offsets in semitones, fixed notes flagged by bit 30.
FurMacro arpeggioMacro(const Unrolled<InstrumentSequenceBaseUnit>& u, SequenceType type, int fixedTranspose = 0)
{
	auto convPhase = [type, fixedTranspose](const std::vector<InstrumentSequenceBaseUnit>& src, int loop, int carry) {
		std::vector<int> out;
		int sum = carry;
		for (const auto& unit : src) {
			int d = unit.data;
			switch (type) {
			case SequenceType::FixedSequence:
				out.push_back(0x40000000 | std::max(0, d + fixedTranspose));
				break;
			case SequenceType::RelativeSequence:
				// Accumulates in BambooTracker; approximated within one pass.
				sum += d - Note::DEFAULT_NOTE_NUM;
				out.push_back(sum);
				break;
			default:
				out.push_back(d - Note::DEFAULT_NOTE_NUM);
				break;
			}
		}
		// A fixed sequence returns to the played note when it ends.
		if (type == SequenceType::FixedSequence && loop < 0 && !out.empty()) out.push_back(0);
		return out;
	};
	std::vector<int> pre = convPhase(u.pre, u.preLoop, 0);
	std::vector<int> rel = convPhase(u.rel, u.relLoop, (type == SequenceType::RelativeSequence && !pre.empty())
											  ? pre.back() : 0);
	if (type == SequenceType::FixedSequence && !u.rel.empty() && u.preLoop < 0 && !pre.empty()) {
		pre.pop_back();	// The release part follows directly.
	}
	return buildMacro(MacroCode::Arp, std::move(pre), u.preLoop, rel, u.relLoop);
}

/// Pitch sequences: BambooTracker uses 1/32 semitone units, Furnace (linear pitch) 1/128.
FurMacro pitchMacro(const Unrolled<InstrumentSequenceBaseUnit>& u, SequenceType type)
{
	FurMacro m = macroFromSequence(MacroCode::Pitch, u, [](const InstrumentSequenceBaseUnit& unit) {
		return (unit.data - SEQ_PITCH_CENTER) * 4;
	});
	if (type == SequenceType::RelativeSequence) m.mode = 1;
	return m;
}

void addMacro(std::vector<FurMacro>& list, FurMacro&& m)
{
	if (!m.val.empty()) list.push_back(std::move(m));
}

/********** Writers **********/
void writeMacro(BinaryContainer& ctr, const FurMacro& m)
{
	if (m.val.empty()) return;
	const size_t len = std::min(m.val.size(), static_cast<size_t>(FUR_MAX_MACRO_LEN));
	int vmin = *std::min_element(m.val.begin(), m.val.begin() + static_cast<long>(len));
	int vmax = *std::max_element(m.val.begin(), m.val.begin() + static_cast<long>(len));
	uint8_t wordSize;
	if (vmin >= 0 && vmax <= 255) wordSize = 0;
	else if (vmin >= -128 && vmax <= 127) wordSize = 64;
	else if (vmin >= -32768 && vmax <= 32767) wordSize = 128;
	else wordSize = 192;

	ctr.appendUint8(m.code & 31);
	ctr.appendUint8(static_cast<uint8_t>(len));
	ctr.appendUint8((m.loop < 0) ? 255 : static_cast<uint8_t>(m.loop));
	ctr.appendUint8((m.rel < 0) ? 255 : static_cast<uint8_t>(m.rel));
	ctr.appendUint8(m.mode);
	ctr.appendUint8(static_cast<uint8_t>((m.instantRelease ? 8 : 0) | 1 | wordSize));	// Open in editor
	ctr.appendUint8(m.delay);
	ctr.appendUint8(1);	// Speed
	for (size_t i = 0; i < len; ++i) {
		int v = m.val[i];
		switch (wordSize) {
		case 0:		ctr.appendUint8(static_cast<uint8_t>(v));	break;
		case 64:	ctr.appendInt8(static_cast<int8_t>(v));		break;
		case 128:	ctr.appendInt16(static_cast<int16_t>(v));	break;
		default:	ctr.appendInt32(static_cast<int32_t>(v));	break;
		}
	}
}

void writeMacroList(BinaryContainer& ctr, const char* code, const std::vector<FurMacro>& macros)
{
	size_t ofs = beginFeature(ctr, code);
	ctr.appendUint16(8);	// Length of macro header
	for (const auto& m : macros) writeMacro(ctr, m);
	ctr.appendUint8(0xff);	// End of macro list
	endFeature(ctr, ofs);
}

void writeInstrument(BinaryContainer& ctr, const FurInstrument& ins)
{
	size_t blockOfs = beginBlock(ctr, "INS2");
	ctr.appendUint16(FUR_VERSION);
	ctr.appendUint8(ins.type);
	ctr.appendUint8(0);

	if (!ins.name.empty()) {
		size_t ofs = beginFeature(ctr, "NA");
		appendCString(ctr, ins.name);
		endFeature(ctr, ofs);
	}

	if (ins.hasFM) {
		size_t ofs = beginFeature(ctr, "FM");
		uint8_t flags = 4;	// Operator count
		for (int i = 0; i < 4; ++i) {
			if (ins.op[i].enable) flags |= static_cast<uint8_t>(16 << i);
		}
		ctr.appendUint8(flags);
		ctr.appendUint8(static_cast<uint8_t>(((ins.alg & 7) << 4) | (ins.fb & 7)));
		ctr.appendUint8(static_cast<uint8_t>(((ins.ams & 3) << 3) | (ins.fms & 7)));
		ctr.appendUint8(32);	// 4-op
		ctr.appendUint8(0);		// Block
		for (const FurOperator& op : ins.op) {
			ctr.appendUint8(static_cast<uint8_t>(((op.dt & 7) << 4) | (op.mult & 15)));
			ctr.appendUint8(static_cast<uint8_t>(op.tl & 127));
			ctr.appendUint8(static_cast<uint8_t>(((op.rs & 3) << 6) | (op.ar & 31)));
			ctr.appendUint8(static_cast<uint8_t>((op.am ? 128 : 0) | (op.dr & 31)));
			ctr.appendUint8(static_cast<uint8_t>((2 << 5) | (op.d2r & 31)));	// KVS: automatic
			ctr.appendUint8(static_cast<uint8_t>(((op.sl & 15) << 4) | (op.rr & 15)));
			ctr.appendUint8(static_cast<uint8_t>(op.ssg & 15));
			ctr.appendUint8(0);
		}
		endFeature(ctr, ofs);
	}

	if (!ins.macros.empty()) writeMacroList(ctr, "MA", ins.macros);

	if (ins.hasSampleMap) {
		size_t ofs = beginFeature(ctr, "SM");
		ctr.appendInt16(static_cast<int16_t>(ins.initSample));
		ctr.appendUint8(static_cast<uint8_t>(2 | (ins.useNoteMap ? 1 : 0)));	// Use sample (+ sample map)
		ctr.appendUint8(0);	// Waveform length
		if (ins.useNoteMap) {
			for (const auto& entry : ins.noteMap) {
				ctr.appendInt16(static_cast<int16_t>(entry.first));
				ctr.appendInt16(static_cast<int16_t>(entry.second));
			}
		}
		endFeature(ctr, ofs);
	}

	static const char* const OP_CODES[4] = { "O1", "O2", "O3", "O4" };
	for (int i = 0; i < 4; ++i) {
		if (!ins.opMacros[i].empty()) writeMacroList(ctr, OP_CODES[i], ins.opMacros[i]);
	}

	endBlock(ctr, blockOfs);
}

void writeSample(BinaryContainer& ctr, const FurSample& smp)
{
	size_t blockOfs = beginBlock(ctr, "SMP2");
	appendCString(ctr, smp.name);
	ctr.appendUint32(static_cast<uint32_t>(smp.data.size() * 2));	// 4-bit ADPCM: 2 samples per byte
	ctr.appendUint32(static_cast<uint32_t>(smp.rate));	// Compatibility rate
	ctr.appendUint32(static_cast<uint32_t>(smp.rate));	// C-4 rate
	ctr.appendUint8(smp.depth);
	ctr.appendUint8(0);	// Loop direction: forward
	ctr.appendUint8(0);	// Flags
	ctr.appendUint8(0);	// Flags 2
	bool loop = (smp.loopStart >= 0 && smp.loopEnd >= 0);
	ctr.appendInt32(loop ? smp.loopStart : -1);
	ctr.appendInt32(loop ? smp.loopEnd : -1);
	for (int i = 0; i < 4; ++i) ctr.appendUint32(0xffffffff);	// Present in the memory of every chip
	ctr.appendVector(smp.data);
	endBlock(ctr, blockOfs);
}

void writePattern(BinaryContainer& ctr, int chan, int index, const FurPattern& pat, size_t effectCols)
{
	size_t blockOfs = beginBlock(ctr, "PATN");
	ctr.appendUint8(0);	// Sub-song
	ctr.appendUint8(static_cast<uint8_t>(chan));
	ctr.appendUint16(static_cast<uint16_t>(index));
	appendCString(ctr, "");	// Name

	int emptyRows = 0;
	for (const FurRow& row : pat) {
		uint8_t mask = 0;
		uint16_t effectMask = 0;
		if (row.note >= 0) mask |= 1;
		if (row.ins >= 0) mask |= 2;
		if (row.vol >= 0) mask |= 4;
		size_t nfx = std::min(row.fx.size(), effectCols);
		for (size_t k = 0; k < nfx; ++k) {
			effectMask |= static_cast<uint16_t>(3 << (k * 2));
			if (k == 0) mask |= 8 | 16;
			else if (k < 4) mask |= 32;
			else mask |= 64;
		}

		if (!mask) {
			if (++emptyRows > 127) {
				ctr.appendUint8(static_cast<uint8_t>(128 | (emptyRows - 2)));
				emptyRows = 0;
			}
			continue;
		}
		if (emptyRows > 1) ctr.appendUint8(static_cast<uint8_t>(128 | (emptyRows - 2)));
		else if (emptyRows) ctr.appendUint8(0);
		emptyRows = 0;

		ctr.appendUint8(mask);
		if (mask & 32) ctr.appendUint8(static_cast<uint8_t>(effectMask & 0xff));
		if (mask & 64) ctr.appendUint8(static_cast<uint8_t>(effectMask >> 8));
		if (mask & 1) ctr.appendUint8(static_cast<uint8_t>(row.note));
		if (mask & 2) ctr.appendUint8(static_cast<uint8_t>(row.ins));
		if (mask & 4) ctr.appendUint8(static_cast<uint8_t>(row.vol));
		for (size_t k = 0; k < nfx; ++k) {
			ctr.appendUint8(static_cast<uint8_t>(row.fx[k].first));
			ctr.appendUint8(static_cast<uint8_t>(row.fx[k].second));
		}
	}
	ctr.appendUint8(0xff);	// End of pattern

	endBlock(ctr, blockOfs);
}

/********** Converter **********/
enum class ChannelKind { FM, SSG, Rhythm, ADPCM };

struct ChannelInfo
{
	ChannelKind kind;
	SoundSource src;
	int chInSrc;
	FMOperatorType opType;	// FM channels only
};

struct ChannelState
{
	int btInst = -1;
	std::deque<int> echo;	// Latest note first
	int pan = 3;
	bool hardReset = false;
	bool usesLfo = false;
};

class FurConverter
{
public:
	FurConverter(std::shared_ptr<Module> mod, std::shared_ptr<InstrumentsManager> instMan,
				 int songNum, FurExportTarget target, const std::vector<uint8_t>& rhythmRom, double ssgMixDb)
		: mod_(mod), instMan_(instMan), song_(mod->getSong(songNum)), target_(target), rhythmRom_(rhythmRom),
		  ssgMixDb_(ssgMixDb)
	{
	}

	std::vector<std::string> write(BinaryContainer& ctr);

private:
	std::shared_ptr<Module> mod_;
	std::shared_ptr<InstrumentsManager> instMan_;
	Song& song_;
	FurExportTarget target_;
	const std::vector<uint8_t>& rhythmRom_;
	double ssgMixDb_;

	std::vector<ChannelInfo> chans_;
	std::vector<FurInstrument> insts_;
	std::map<std::pair<int, int>, int> instMap_;	// (BT instrument, variant) -> Furnace instrument
	std::vector<FurSample> samples_;
	std::map<int, int> sampleMap_;		// BT ADPCM sample -> Furnace sample
	std::array<int, 6> rhythmInsts_ = { { -1, -1, -1, -1, -1, -1 } };
	std::set<std::string> warnings_;

	// Per instrument facts needed while converting patterns.
	struct InstInfo
	{
		bool releaseOnKeyOff = false;	// Key off should release macros instead of cutting
		bool envReset = false;
		bool lfoEnabled = false;
		int lfoFreq = 0;
	};
	std::map<std::pair<int, int>, InstInfo> instInfo_;

	// Playback state followed in order while converting.
	std::vector<ChannelState> chState_;
	int lfoState_ = -1;
	bool tempoMode_ = true;
	int tempo_ = 150;
	int tempoDenom_ = 150;

	void setupChannels();
	int furnaceSample(int btSamp);
	int furnaceInstrument(int btInst, FMOperatorType opType);
	void buildFM(FurInstrument& ins, InstInfo& info, const std::shared_ptr<InstrumentFM>& inst, FMOperatorType opType);
	void buildSSG(FurInstrument& ins, InstInfo& info, const std::shared_ptr<InstrumentSSG>& inst);
	void buildADPCM(FurInstrument& ins, InstInfo& info, const std::shared_ptr<InstrumentADPCM>& inst);
	void buildDrumkit(FurInstrument& ins, const std::shared_ptr<InstrumentDrumkit>& inst);
	void createRhythmInstruments();
	FurRow convertStep(size_t c, Step& step);
	void convertEffect(const ChannelInfo& ch, const Effect& eff, const std::string& id, std::vector<std::pair<int, int>>& out);
	const InstInfo& infoOf(size_t c) const;
};

void FurConverter::setupChannels()
{
	SongStyle style = song_.getStyle();
	for (const TrackAttribute& attrib : style.trackAttribs) {
		ChannelInfo info;
		info.src = attrib.source;
		info.chInSrc = attrib.channelInSource;
		info.opType = FMOperatorType::All;
		switch (attrib.source) {
		case SoundSource::FM:
			info.kind = ChannelKind::FM;
			if (style.type == SongType::FM3chExpanded) {
				switch (attrib.channelInSource) {
				case 2:	info.opType = FMOperatorType::Op1;	break;
				case 6:	info.opType = FMOperatorType::Op2;	break;
				case 7:	info.opType = FMOperatorType::Op3;	break;
				case 8:	info.opType = FMOperatorType::Op4;	break;
				default:	break;
				}
			}
			break;
		case SoundSource::SSG:		info.kind = ChannelKind::SSG;		break;
		case SoundSource::RHYTHM:	info.kind = ChannelKind::Rhythm;	break;
		case SoundSource::ADPCM:	info.kind = ChannelKind::ADPCM;		break;
		default:	throw std::out_of_range("Unknown sound source");
		}
		chans_.push_back(info);
	}
	chState_.assign(chans_.size(), ChannelState());
}

int FurConverter::furnaceSample(int btSamp)
{
	auto it = sampleMap_.find(btSamp);
	if (it != sampleMap_.end()) return it->second;
	if (samples_.size() >= FUR_MAX_SAMPLES) {
		warnings_.insert("Too many samples; some ADPCM samples were left out.");
		return -1;
	}

	FurSample smp;
	smp.name = "ADPCM sample " + std::to_string(btSamp);
	smp.depth = FUR_SAMPLE_ADPCM_B;
	smp.data = instMan_->getSampleADPCMRawSample(btSamp);
	// Furnace lays samples out on 256-byte boundaries and plays each one to the
	// end of its last block; pad with codes that leave the decoder silent.
	size_t padded = (smp.data.size() + 0xff) & ~static_cast<size_t>(0xff);
	if (padded > smp.data.size()) {
		std::vector<uint8_t> pad = chip::makeAdpcmBPadding(smp.data.data(), smp.data.size(), padded - smp.data.size());
		smp.data.insert(smp.data.end(), pad.begin(), pad.end());
	}

	// Rate when played at C-4, from the DELTA-N used at the root key.
	int rootKey = instMan_->getSampleADPCMRootKeyNumber(btSamp);
	double rootRate = instMan_->getSampleADPCMRootDeltaN(btSamp) * ADPCMB_FULL_RATE / 65536.0;
	smp.rate = static_cast<int>(std::round(rootRate * std::pow(2.0, (Note::DEFAULT_NOTE_NUM - rootKey) / 12.0)));

	if (instMan_->isSampleADPCMRepeatable(btSamp)) {
		// Repeat addresses are in 32-byte units (64 samples).
		SampleRepeatRange range = instMan_->getSampleADPCMRepeatRange(btSamp);
		int total = static_cast<int>(smp.data.size() * 2);
		smp.loopStart = std::min(static_cast<int>(range.first() * 64), total);
		smp.loopEnd = std::min(static_cast<int>((range.last() + 1) * 64), total);
		if (smp.loopEnd <= smp.loopStart) smp.loopStart = smp.loopEnd = -1;
	}

	int idx = static_cast<int>(samples_.size());
	samples_.push_back(std::move(smp));
	sampleMap_[btSamp] = idx;
	return idx;
}

void FurConverter::createRhythmInstruments()
{
	if (target_ != FurExportTarget::YM2610B) return;
	if (rhythmRom_.size() < 0x2000) {
		warnings_.insert("Rhythm samples are unavailable; rhythm tracks will be silent.");
		return;
	}
	for (int i = 0; i < 6; ++i) {
		if (samples_.size() >= FUR_MAX_SAMPLES || insts_.size() >= FUR_MAX_INSTRUMENTS) break;
		uint32_t begin = RHYTHM_ROM_RANGES[i][0];
		uint32_t len = RHYTHM_ROM_RANGES[i][1] - begin + 1;
		FurSample smp;
		smp.name = std::string("Rhythm ") + RHYTHM_NAMES[i];
		smp.depth = FUR_SAMPLE_ADPCM_A;
		smp.data.assign(rhythmRom_.begin() + begin, rhythmRom_.begin() + begin + len);
		size_t padded = (len + 0xff) & ~static_cast<size_t>(0xff);
		std::vector<uint8_t> pad = chip::makeAdpcmAPadding(&rhythmRom_[begin], len, padded - len);
		smp.data.insert(smp.data.end(), pad.begin(), pad.end());
		smp.rate = ADPCMA_RATE;
		int smpIdx = static_cast<int>(samples_.size());
		samples_.push_back(std::move(smp));

		FurInstrument ins;
		ins.name = std::string("Rhythm ") + RHYTHM_NAMES[i];
		ins.type = FUR_INS_ADPCMA;
		ins.hasSampleMap = true;
		ins.initSample = smpIdx;
		rhythmInsts_[i] = static_cast<int>(insts_.size());
		insts_.push_back(std::move(ins));
	}
}

int FurConverter::furnaceInstrument(int btInst, FMOperatorType opType)
{
	std::pair<int, int> key(btInst, static_cast<int>(opType));
	auto it = instMap_.find(key);
	if (it != instMap_.end()) return it->second;

	std::shared_ptr<AbstractInstrument> inst = instMan_->getInstrumentSharedPtr(btInst);
	if (!inst) return -1;
	if (insts_.size() >= FUR_MAX_INSTRUMENTS) {
		warnings_.insert("Too many instruments; some instruments were left out.");
		return -1;
	}

	FurInstrument ins;
	InstInfo info;
	ins.name = inst->getName();
	switch (inst->getType()) {
	case InstrumentType::FM:
		buildFM(ins, info, std::dynamic_pointer_cast<InstrumentFM>(inst), opType);
		if (opType != FMOperatorType::All) {
			static const char* const OP_NAMES[5] = { "", " (OP1)", " (OP2)", " (OP3)", " (OP4)" };
			ins.name += OP_NAMES[static_cast<int>(opType)];
		}
		break;
	case InstrumentType::SSG:
		buildSSG(ins, info, std::dynamic_pointer_cast<InstrumentSSG>(inst));
		break;
	case InstrumentType::ADPCM:
		buildADPCM(ins, info, std::dynamic_pointer_cast<InstrumentADPCM>(inst));
		break;
	case InstrumentType::Drumkit:
		buildDrumkit(ins, std::dynamic_pointer_cast<InstrumentDrumkit>(inst));
		break;
	}

	int idx = static_cast<int>(insts_.size());
	insts_.push_back(std::move(ins));
	instMap_[key] = idx;
	instInfo_[key] = info;
	return idx;
}

void FurConverter::buildFM(FurInstrument& ins, InstInfo& info, const std::shared_ptr<InstrumentFM>& inst,
						   FMOperatorType opType)
{
	static const FMEnvelopeParameter PARAMS[4][9] = {
		{ FMEnvelopeParameter::AR1, FMEnvelopeParameter::DR1, FMEnvelopeParameter::SR1, FMEnvelopeParameter::RR1,
		  FMEnvelopeParameter::SL1, FMEnvelopeParameter::TL1, FMEnvelopeParameter::KS1, FMEnvelopeParameter::ML1,
		  FMEnvelopeParameter::DT1 },
		{ FMEnvelopeParameter::AR2, FMEnvelopeParameter::DR2, FMEnvelopeParameter::SR2, FMEnvelopeParameter::RR2,
		  FMEnvelopeParameter::SL2, FMEnvelopeParameter::TL2, FMEnvelopeParameter::KS2, FMEnvelopeParameter::ML2,
		  FMEnvelopeParameter::DT2 },
		{ FMEnvelopeParameter::AR3, FMEnvelopeParameter::DR3, FMEnvelopeParameter::SR3, FMEnvelopeParameter::RR3,
		  FMEnvelopeParameter::SL3, FMEnvelopeParameter::TL3, FMEnvelopeParameter::KS3, FMEnvelopeParameter::ML3,
		  FMEnvelopeParameter::DT3 },
		{ FMEnvelopeParameter::AR4, FMEnvelopeParameter::DR4, FMEnvelopeParameter::SR4, FMEnvelopeParameter::RR4,
		  FMEnvelopeParameter::SL4, FMEnvelopeParameter::TL4, FMEnvelopeParameter::KS4, FMEnvelopeParameter::ML4,
		  FMEnvelopeParameter::DT4 }
	};
	static const uint8_t OP_MACRO_OF_PARAM[9] = {
		OpMacroCode::AR, OpMacroCode::DR, OpMacroCode::D2R, OpMacroCode::RR, OpMacroCode::SL,
		OpMacroCode::TL, OpMacroCode::RS, OpMacroCode::MULT, OpMacroCode::DT
	};
	static const FMEnvelopeParameter SSGEG[4] = {
		FMEnvelopeParameter::SSGEG1, FMEnvelopeParameter::SSGEG2, FMEnvelopeParameter::SSGEG3, FMEnvelopeParameter::SSGEG4
	};
	static const FMLFOParameter AM[4] = { FMLFOParameter::AM1, FMLFOParameter::AM2, FMLFOParameter::AM3, FMLFOParameter::AM4 };

	ins.type = FUR_INS_FM;
	ins.hasFM = true;
	ins.alg = inst->getEnvelopeParameter(FMEnvelopeParameter::AL);
	ins.fb = inst->getEnvelopeParameter(FMEnvelopeParameter::FB);
	for (int fop = 0; fop < 4; ++fop) {
		int bop = BT_OP_OF_FUR_OP[fop];
		FurOperator& op = ins.op[fop];
		const auto& p = PARAMS[bop];
		op.enable = inst->getOperatorEnabled(bop);
		op.ar = inst->getEnvelopeParameter(p[0]);
		op.dr = inst->getEnvelopeParameter(p[1]);
		op.d2r = inst->getEnvelopeParameter(p[2]);
		op.rr = inst->getEnvelopeParameter(p[3]);
		op.sl = inst->getEnvelopeParameter(p[4]);
		op.tl = inst->getEnvelopeParameter(p[5]);
		op.rs = inst->getEnvelopeParameter(p[6]);
		op.mult = inst->getEnvelopeParameter(p[7]);
		op.dt = FUR_DT_OF_REG_DT[inst->getEnvelopeParameter(p[8]) & 7];
		int ssgeg = inst->getEnvelopeParameter(SSGEG[bop]);
		op.ssg = (ssgeg < 0) ? 0 : (8 | (ssgeg & 7));
	}

	// LFO: the frequency is global on the chip and is set with effect 10xy in patterns.
	if (inst->getLFOEnabled()) {
		info.lfoEnabled = true;
		info.lfoFreq = inst->getLFOParameter(FMLFOParameter::FREQ) & 7;
		int pms = inst->getLFOParameter(FMLFOParameter::PMS);
		int ams = inst->getLFOParameter(FMLFOParameter::AMS);
		int delay = clampValue(inst->getLFOParameter(FMLFOParameter::Count), 0, 255);
		if (delay == 0) {
			ins.fms = pms;
			ins.ams = ams;
			for (int bop = 0; bop < 4; ++bop) ins.op[FUR_OP_OF_BT_OP[bop]].am = inst->getLFOParameter(AM[bop]);
		}
		else {
			// Start the LFO depth after the delay count, as BambooTracker does.
			auto delayed = [delay](uint8_t code, int v) {
				FurMacro m;
				m.code = code;
				m.val = { v };
				m.delay = static_cast<uint8_t>(delay);
				return m;
			};
			if (pms) ins.macros.push_back(delayed(MacroCode::Fms, pms));
			if (ams) ins.macros.push_back(delayed(MacroCode::Ams, ams));
			for (int bop = 0; bop < 4; ++bop) {
				if (inst->getLFOParameter(AM[bop]))
					ins.opMacros[FUR_OP_OF_BT_OP[bop]].push_back(delayed(OpMacroCode::AM, 1));
			}
		}
	}

	// Envelope parameter sequences.
	auto opSeq = [&](FMEnvelopeParameter param) {
		return unrollSequence<FMOperatorSequenceIter>([&] { return inst->getOperatorSequenceSequenceIterator(param); });
	};
	auto plain = [](const InstrumentSequenceBaseUnit& u) { return u.data; };
	if (inst->getOperatorSequenceEnabled(FMEnvelopeParameter::AL))
		addMacro(ins.macros, macroFromSequence(MacroCode::Alg, opSeq(FMEnvelopeParameter::AL), plain));
	if (inst->getOperatorSequenceEnabled(FMEnvelopeParameter::FB))
		addMacro(ins.macros, macroFromSequence(MacroCode::Fb, opSeq(FMEnvelopeParameter::FB), plain));
	for (int bop = 0; bop < 4; ++bop) {
		for (int i = 0; i < 9; ++i) {
			FMEnvelopeParameter param = PARAMS[bop][i];
			if (!inst->getOperatorSequenceEnabled(param)) continue;
			auto u = opSeq(param);
			FurMacro m = (OP_MACRO_OF_PARAM[i] == OpMacroCode::DT)
						 ? macroFromSequence(OP_MACRO_OF_PARAM[i], u, [](const InstrumentSequenceBaseUnit& unit) {
							   return FUR_DT_OF_REG_DT[unit.data & 7];
						   })
						 : macroFromSequence(OP_MACRO_OF_PARAM[i], u, plain);
			addMacro(ins.opMacros[FUR_OP_OF_BT_OP[bop]], std::move(m));
		}
	}

	if (inst->getArpeggioEnabled(opType)) {
		auto u = unrollSequence<ArpeggioIter>([&] { return inst->getArpeggioSequenceIterator(opType); });
		addMacro(ins.macros, arpeggioMacro(u, inst->getArpeggioType(opType)));
	}
	if (inst->getPitchEnabled(opType)) {
		auto u = unrollSequence<PitchIter>([&] { return inst->getPitchSequenceIterator(opType); });
		addMacro(ins.macros, pitchMacro(u, inst->getPitchType(opType)));
	}
	if (inst->getPanEnabled()) {
		auto u = unrollSequence<PanIter>([&] { return inst->getPanSequenceIterator(); });
		addMacro(ins.macros, macroFromSequence(MacroCode::PanL, u, plain));
	}

	info.releaseOnKeyOff = true;	// FM key off always enters the release phase
	info.envReset = inst->getEnvelopeResetEnabled(opType);
}

void FurConverter::buildSSG(FurInstrument& ins, InstInfo& info, const std::shared_ptr<InstrumentSSG>& inst)
{
	ins.type = FUR_INS_AY;

	Unrolled<SSGWaveformUnit> wf;
	Unrolled<SSGToneNoiseUnit> tn;
	Unrolled<SSGEnvelopeUnit> env;
	bool hasWf = inst->getWaveformEnabled();
	bool hasTn = inst->getToneNoiseEnabled();
	bool hasEnv = inst->getEnvelopeEnabled();
	if (hasWf) wf = unrollSequence<SSGWaveformIter>([&] { return inst->getWaveformSequenceIterator(); });
	if (hasTn) tn = unrollSequence<SSGToneNoiseIter>([&] { return inst->getToneNoiseSequenceIterator(); });
	if (hasEnv) env = unrollSequence<SSGEnvelopeIter>([&] { return inst->getEnvelopeSequenceIterator(); });

	// Per-tick register state worked out from the waveform, tone/noise and
	// envelope sequences, which BambooTracker evaluates together.
	struct TickState
	{
		int wave;		// bit 0: tone, bit 1: noise, bit 2: hardware envelope
		int noise;		// Furnace noise value (31 - period), -1 if unused
		int shape;		// Hardware envelope shape, -1 if unused
		int num, den;	// Auto-envelope ratio (0: disabled)
		int fixedTone;	// Raw tone period, 0 if following the note
		int envPeriod;	// Raw hardware envelope period, -1 if unused
		int vol;
	};
	auto evaluate = [&](const SSGWaveformUnit* w, const SSGToneNoiseUnit* t, const SSGEnvelopeUnit* e) {
		TickState s { 1, -1, -1, 0, 0, 0, -1, 15 };
		int wt = w ? w->data : SSGWaveformType::SQUARE;
		bool buzzer = (wt == SSGWaveformType::TRIANGLE || wt == SSGWaveformType::SAW || wt == SSGWaveformType::INVSAW);
		bool masked = (wt == SSGWaveformType::SQM_TRIANGLE || wt == SSGWaveformType::SQM_SAW
					   || wt == SSGWaveformType::SQM_INVSAW);

		// Tone and noise mixer
		bool tone = true, noise = false;
		if (t) {
			int type = t->data;
			if (type == 65) tone = false;
			else if (type > 32) {
				noise = true;
				s.noise = 31 - (64 - type);
			}
			else if (type > 0) {
				tone = false;
				noise = true;
				s.noise = 31 - (32 - type);
			}
		}
		if (buzzer) tone = false;
		s.wave = (tone ? 1 : 0) | (noise ? 2 : 0);

		// Waveforms built on the hardware envelope
		if (buzzer || masked) {
			bool tri = (wt == SSGWaveformType::TRIANGLE || wt == SSGWaveformType::SQM_TRIANGLE);
			bool saw = (wt == SSGWaveformType::SAW || wt == SSGWaveformType::SQM_SAW);
			s.wave |= 4;
			s.shape = tri ? 0x0e : saw ? 0x0c : 0x08;
			int mul = tri ? 2 : 1;	// Triangle period is twice the saw period
			if (masked && w->type == SSGWaveformUnit::RawSubdata) {
				int raw;
				w->getSubdataAsRaw(raw);
				s.fixedTone = std::max(1, raw);
				s.num = mul;
				s.den = 1;
			}
			else if (masked && w->type == SSGWaveformUnit::RatioSubdata) {
				int r1, r2;
				w->getSubdataAsRatio(r1, r2);
				s.num = mul * std::max(1, r1);
				s.den = std::max(1, r2);
			}
			else {
				s.num = mul;
				s.den = 1;
			}
			return s;
		}

		if (e) {
			if (e->data < 16) {
				s.vol = std::max(0, e->data);
			}
			else {	// Hardware envelope
				s.wave |= 4;
				s.shape = e->data - 16 + 8;
				int div = (e->data == 18 || e->data == 22) ? 2 : 1;	// Triangle shapes
				switch (e->type) {
				case SSGEnvelopeUnit::RawSubdata:
				{
					int raw;
					e->getSubdataAsRaw(raw);
					s.envPeriod = raw;
					break;
				}
				case SSGEnvelopeUnit::RatioSubdata:
				{
					int r1, r2;
					e->getSubdataAsRatio(r1, r2);
					s.num = div * std::max(1, r2);
					s.den = std::max(1, r1);
					break;
				}
				case SSGEnvelopeUnit::ShiftSubdata:
				{
					int rshift;
					e->getSubdataAsShift(rshift);
					int rs = rshift - 4 + (div - 1);
					if (rs >= 0) {
						s.num = 1 << std::min(rs, 7);
						s.den = 1;
					}
					else {
						s.num = 1;
						s.den = 1 << std::min(-rs, 7);
					}
					break;
				}
				default:
					break;
				}
			}
		}
		return s;
	};

	// Lay the three sequences side by side for both phases.
	auto states = [&](bool release, int& loop) {
		std::vector<PhaseLayout> layout;
		auto add = [&](bool enabled, size_t preLen, int preLoop, size_t relLen, int relLoop) {
			if (!enabled) layout.push_back({ 0, -1 });
			else if (release) layout.push_back({ relLen, relLoop });
			else layout.push_back({ preLen, preLoop });
		};
		add(hasWf, wf.pre.size(), wf.preLoop, wf.rel.size(), wf.relLoop);
		add(hasTn, tn.pre.size(), tn.preLoop, tn.rel.size(), tn.relLoop);
		add(hasEnv, env.pre.size(), env.preLoop, env.rel.size(), env.relLoop);
		auto idx = combinePhases(layout, loop);
		std::vector<TickState> out;
		for (const auto& k : idx) {
			// A sequence without release part keeps its last value after key off.
			const SSGWaveformUnit* w = !hasWf ? nullptr
										: (k[0] >= 0) ? &(release ? wf.rel : wf.pre)[k[0]]
										: (!wf.pre.empty() ? &wf.pre.back() : nullptr);
			const SSGToneNoiseUnit* t = !hasTn ? nullptr
										: (k[1] >= 0) ? &(release ? tn.rel : tn.pre)[k[1]]
										: (!tn.pre.empty() ? &tn.pre.back() : nullptr);
			const SSGEnvelopeUnit* e = !hasEnv ? nullptr
									   : (k[2] >= 0) ? &(release ? env.rel : env.pre)[k[2]]
									   : (!env.pre.empty() ? &env.pre.back() : nullptr);
			out.push_back(evaluate(w, t, e));
		}
		return out;
	};
	int preLoop, relLoop;
	std::vector<TickState> pre = states(false, preLoop);
	bool anyRelease = (hasWf && !wf.rel.empty()) || (hasTn && !tn.rel.empty()) || (hasEnv && !env.rel.empty());
	std::vector<TickState> rel;
	if (anyRelease) rel = states(true, relLoop);
	else relLoop = -1;

	bool useWave = false, useNoise = false, useShape = false, useAuto = false, useFixed = false;
	bool usePeriod = false, useVol = hasEnv;
	for (const auto* list : { &pre, &rel }) {
		for (const TickState& s : *list) {
			useWave |= (s.wave != 1);
			useNoise |= (s.noise >= 0);
			useShape |= (s.shape >= 0);
			useAuto |= (s.num > 0);
			useFixed |= (s.fixedTone > 0);
			usePeriod |= (s.envPeriod >= 0);
			useVol |= (s.wave & 4) != 0;
		}
	}

	auto emit = [&](uint8_t code, auto get) {
		std::vector<int> a, b;
		for (const TickState& s : pre) a.push_back(get(s));
		for (const TickState& s : rel) b.push_back(get(s));
		addMacro(ins.macros, buildMacro(code, std::move(a), preLoop, b, relLoop));
	};
	// Fill unused entries of a value with the nearest used one so that
	// Furnace does not rewrite the register with a meaningless value.
	auto filled = [&](auto get) {
		int last = -1;
		for (const auto* list : { &pre, &rel }) {
			for (const TickState& s : *list) {
				if (get(s) >= 0) {
					if (last < 0) last = get(s);
				}
			}
		}
		return std::max(0, last);
	};

	if (useVol) emit(MacroCode::Vol, [](const TickState& s) { return (s.wave & 4) ? 15 : s.vol; });
	if (useNoise) {
		int def = filled([](const TickState& s) { return s.noise; });
		emit(MacroCode::Duty, [def](const TickState& s) { return (s.noise >= 0) ? s.noise : def; });
	}
	if (useWave) emit(MacroCode::Wave, [](const TickState& s) { return s.wave; });
	if (useShape) {
		int def = filled([](const TickState& s) { return s.shape; });
		emit(MacroCode::Ex2, [def](const TickState& s) { return (s.shape >= 0) ? s.shape : def; });
	}
	if (useAuto) {
		emit(MacroCode::Ex3, [](const TickState& s) { return clampValue(s.num, 0, 255); });
		emit(MacroCode::Alg, [](const TickState& s) { return clampValue(s.den, 0, 255); });
	}
	if (useFixed) emit(MacroCode::Ex4, [](const TickState& s) { return clampValue(s.fixedTone, 0, 4095); });
	if (usePeriod) {
		int def = filled([](const TickState& s) { return s.envPeriod; });
		emit(MacroCode::Ex5, [def](const TickState& s) { return (s.envPeriod >= 0) ? s.envPeriod : def; });
	}

	if (inst->getArpeggioEnabled()) {
		auto u = unrollSequence<ArpeggioIter>([&] { return inst->getArpeggioSequenceIterator(); });
		addMacro(ins.macros, arpeggioMacro(u, inst->getArpeggioType(), SSG_TRANSPOSE));
	}
	if (inst->getPitchEnabled()) {
		auto u = unrollSequence<PitchIter>([&] { return inst->getPitchSequenceIterator(); });
		addMacro(ins.macros, pitchMacro(u, inst->getPitchType()));
	}

	// Without an envelope release part, BambooTracker silences the channel at key off.
	info.releaseOnKeyOff = hasEnv && !env.rel.empty();
}

void FurConverter::buildADPCM(FurInstrument& ins, InstInfo& info, const std::shared_ptr<InstrumentADPCM>& inst)
{
	ins.type = FUR_INS_ADPCMB;
	ins.hasSampleMap = true;
	ins.initSample = furnaceSample(inst->getSampleNumber());

	auto plain = [](const InstrumentSequenceBaseUnit& u) { return u.data; };
	bool hasEnv = inst->getEnvelopeEnabled();
	bool envRelease = false;
	if (hasEnv) {
		auto u = unrollSequence<ADPCMEnvelopeIter>([&] { return inst->getEnvelopeSequenceIterator(); });
		envRelease = !u.rel.empty();
		addMacro(ins.macros, macroFromSequence(MacroCode::Vol, u, plain));
	}
	if (inst->getArpeggioEnabled()) {
		auto u = unrollSequence<ArpeggioIter>([&] { return inst->getArpeggioSequenceIterator(); });
		addMacro(ins.macros, arpeggioMacro(u, inst->getArpeggioType()));
	}
	if (inst->getPitchEnabled()) {
		auto u = unrollSequence<PitchIter>([&] { return inst->getPitchSequenceIterator(); });
		addMacro(ins.macros, pitchMacro(u, inst->getPitchType()));
	}
	if (inst->getPanEnabled()) {
		auto u = unrollSequence<PanIter>([&] { return inst->getPanSequenceIterator(); });
		addMacro(ins.macros, macroFromSequence(MacroCode::PanL, u, plain));
	}

	info.releaseOnKeyOff = envRelease;
}

void FurConverter::buildDrumkit(FurInstrument& ins, const std::shared_ptr<InstrumentDrumkit>& inst)
{
	ins.type = FUR_INS_ADPCMB;
	ins.hasSampleMap = true;
	ins.useNoteMap = true;
	for (int n = 0; n < FUR_NOTE_MAP_SIZE; ++n) ins.noteMap[n] = { n, -1 };
	for (int key : inst->getAssignedKeys()) {
		if (key < 0 || key >= FUR_NOTE_MAP_SIZE || !inst->getSampleEnabled(key)) continue;
		int smp = furnaceSample(inst->getSampleNumber(key));
		if (smp < 0) continue;
		if (ins.initSample < 0) ins.initSample = smp;
		// Furnace plays a sample at its C-4 rate on C-4, so the sample's root
		// key shifted by the key's pitch reproduces the root DELTA-N.
		int root = inst->getSampleRootKeyNumber(key);
		ins.noteMap[key] = { clampValue(root + inst->getPitch(key), 0, FUR_NOTE_MAP_SIZE - 1), smp };
	}
}

const FurConverter::InstInfo& FurConverter::infoOf(size_t c) const
{
	static const InstInfo none;
	const ChannelState& st = chState_[c];
	if (st.btInst < 0) return none;
	auto it = instInfo_.find({ st.btInst, static_cast<int>(chans_[c].opType) });
	return (it == instInfo_.end()) ? none : it->second;
}

void FurConverter::convertEffect(const ChannelInfo& ch, const Effect& eff, const std::string& id,
								 std::vector<std::pair<int, int>>& out)
{
	const int v = eff.value;
	auto add = [&](int code, int val) { out.emplace_back(code, clampValue(val, 0, 255)); };
	auto unsupported = [&](const char* what) {
		warnings_.insert("Effect " + id + " (" + what + ") has no Furnace equivalent and was dropped.");
	};

	switch (eff.type) {
	case EffectType::Arpeggio:			add(0x00, v);	break;
	case EffectType::PortamentoUp:		add(0x01, v);	break;	// Same unit: 1/32 semitone per tick
	case EffectType::PortamentoDown:	add(0x02, v);	break;
	case EffectType::TonePortamento:	add(0x03, v);	break;
	case EffectType::Vibrato:
	case EffectType::Tremolo:
	{
		// BambooTracker: triangle wave, x = ticks per quarter cycle, peak = x * y.
		// Furnace: 64-step cycle advanced by x per tick, peak depth y/16 semitone.
		int x = v >> 4, y = v & 0x0f;
		int code = (eff.type == EffectType::Vibrato) ? 0x04 : 0x07;
		if (!x || !y) {
			add(code, 0);
		}
		else {
			int rate = clampValue(static_cast<int>(std::lround(16.0 / x)), 1, 15);
			int depth = (eff.type == EffectType::Vibrato) ? clampValue((x * y + 1) / 2, 1, 15) : y;
			add(code, (rate << 4) | depth);
		}
		break;
	}
	case EffectType::Pan:
	{
		static const int PAN[4] = { 0x00, 0x0f, 0xf0, 0xff };	// None, right, left, center
		if (0 <= v && v < 4) add(0x08, PAN[v]);
		break;
	}
	case EffectType::VolumeSlide:
	{
		// Furnace slides 4 times slower per unit than BambooTracker.
		int hi = v >> 4, lo = v & 0x0f;
		if (hi && !lo) add(0x0a, std::min(15, hi * 4) << 4);
		else if (!hi) add(0x0a, std::min(15, lo * 4));
		break;
	}
	case EffectType::PositionJump:	add(0x0b, v);	break;
	case EffectType::SongEnd:		add(0xff, 0);	break;
	case EffectType::PatternBreak:	add(0x0d, v);	break;
	case EffectType::SpeedTempoChange:
		if (v < 0x20) {
			if (!v) break;
			add(0x0f, v);
			if (!tempoMode_) add(0xfd, tempo_);	// Speed changes return to tempo mode
			tempoMode_ = true;
		}
		else {
			add(0xfd, v);	// Virtual tempo numerator
			tempo_ = v;
			tempoMode_ = true;
		}
		break;
	case EffectType::Groove:
		if (v < static_cast<int>(mod_->getGrooveCount())) {
			add(0x09, v);
			if (tempoMode_) add(0xfd, tempoDenom_);	// Grooves are played without tempo scaling
			tempoMode_ = false;
		}
		break;
	case EffectType::NoteDelay:		add(0xed, v);	break;
	case EffectType::Detune:		add(0xe5, 0x80 + (v - 0x80) * 4);	break;
	case EffectType::NoteSlideUp:
	case EffectType::NoteSlideDown:
	{
		// BambooTracker slides y semitones over x ticks; Furnace moves x/8 semitone per tick.
		int x = v >> 4, y = v & 0x0f;
		int speed = x ? clampValue(static_cast<int>(std::lround(8.0 * y / x)), 1, 15) : 15;
		add((eff.type == EffectType::NoteSlideUp) ? 0xe1 : 0xe2, y ? ((speed << 4) | y) : 0);
		break;
	}
	case EffectType::NoteRelease:	add(0xfc, v);	break;
	case EffectType::TransposeDelay:
	{
		int cnt = (v & 0x70) >> 4, semi = v & 0x0f;
		add((v & 0x80) ? 0xe9 : 0xe8, (cnt << 4) | semi);
		break;
	}
	case EffectType::NoteCut:		add(0xec, v);	break;
	case EffectType::Retrigger:
		if (v & 0x0f) add(0x0c, v & 0x0f);
		if (v & 0x70) warnings_.insert("Volume changes of retrigger effect 0K were dropped.");
		break;
	case EffectType::MasterVolume:	add(0x1f, v);	break;	// Rhythm / ADPCM-A total level
	case EffectType::ToneNoiseMix:
	{
		static const int MODE[4] = { 0x07, 0x00, 0x01, 0x02 };	// None, tone, noise, tone & noise
		if (0 <= v && v < 4) add(0x20, MODE[v]);
		break;
	}
	case EffectType::NoisePitch:		add(0x21, v);	break;
	case EffectType::HardEnvHighPeriod:	add(0x24, v);	break;
	case EffectType::HardEnvLowPeriod:	add(0x23, v);	break;
	case EffectType::AutoEnvelope:
	{
		int shape = v & 0x0f;
		int shift = (v >> 4) - 8;
		if (!shape) {
			add(0x22, 0x00);
			add(0x29, 0x00);
			break;
		}
		add(0x22, (shape << 4) | 1);
		if (shift == -8) {	// Raw period set by 0I/0J
			add(0x29, 0x00);
			break;
		}
		static const int AUTO_ENV_SHAPE_TYPE[15] = { 17, 17, 17, 21, 21, 21, 21, 16, 17, 18, 19, 20, 21, 22, 23 };
		int d = AUTO_ENV_SHAPE_TYPE[shape - 1];
		int rs = shift - 4 + ((d == 18 || d == 22) ? 1 : 0);
		int num = (rs >= 0) ? (1 << std::min(rs, 3)) : 1;
		int den = (rs >= 0) ? 1 : std::min(15, 1 << std::min(-rs, 4));
		add(0x29, (std::min(num, 15) << 4) | den);
		break;
	}
	case EffectType::FBControl:		add(0x11, v);	break;
	case EffectType::TLControl:
	{
		int op = v >> 8;
		if (0 < op && op < 5) add(0x12 + op - 1, v & 0xff);
		break;
	}
	case EffectType::MLControl:		add(0x16, v);	break;
	case EffectType::ARControl:
	{
		int op = v >> 8;
		if (0 < op && op < 5) add(0x1a + op - 1, v & 0xff);
		break;
	}
	case EffectType::DRControl:
	{
		int op = v >> 8;
		if (0 < op && op < 5) add(0x57 + op - 1, v & 0xff);
		break;
	}
	case EffectType::RRControl:		add(0x52, v);	break;
	case EffectType::Brightness:	unsupported("brightness");			break;
	case EffectType::FineDetune:	unsupported("fine detune");			break;
	case EffectType::VolumeDelay:	unsupported("volume delay");		break;
	case EffectType::XVolumeSlide:	unsupported("extended volume slide");	break;
	case EffectType::RegisterAddress0:
	case EffectType::RegisterAddress1:
	case EffectType::RegisterValue:	unsupported("register write");		break;
	default:	break;
	}
	(void)ch;
}

FurRow FurConverter::convertStep(size_t c, Step& step)
{
	FurRow row;
	const ChannelInfo& ch = chans_[c];
	ChannelState& st = chState_[c];

	// Instrument
	if (step.hasInstrument() && ch.kind != ChannelKind::Rhythm) {
		int n = step.getInstrumentNumber();
		if (auto inst = instMan_->getInstrumentSharedPtr(n)) {
			InstrumentType t = inst->getType();
			bool ok = (ch.kind == ChannelKind::FM && t == InstrumentType::FM)
					  || (ch.kind == ChannelKind::SSG && t == InstrumentType::SSG)
					  || (ch.kind == ChannelKind::ADPCM && (t == InstrumentType::ADPCM || t == InstrumentType::Drumkit));
			if (ok) {
				int fi = furnaceInstrument(n, ch.opType);
				if (fi >= 0) {
					row.ins = fi;
					st.btInst = n;
				}
			}
		}
	}
	const InstInfo& info = infoOf(c);

	// Volume
	if (step.hasVolume()) {
		int v = step.getVolume();
		switch (ch.kind) {
		case ChannelKind::FM:		if (v < bt_defs::NSTEP_FM_VOLUME) row.vol = bt_defs::NSTEP_FM_VOLUME - 1 - v;	break;
		case ChannelKind::SSG:		if (v < bt_defs::NSTEP_SSG_VOLUME) row.vol = v;		break;
		case ChannelKind::Rhythm:	if (v < bt_defs::NSTEP_RHYTHM_VOLUME) row.vol = v;	break;
		case ChannelKind::ADPCM:	if (v < bt_defs::NSTEP_ADPCM_VOLUME) row.vol = v;	break;
		}
	}

	// Effects
	for (int i = 0; i < Step::N_EFFECT; ++i) {
		std::string id = step.getEffectId(i);
		Effect eff = effect_utils::validateEffect(ch.src, id, step.getEffectValue(i));
		if (eff.type != EffectType::NoEffect) convertEffect(ch, eff, id, row.fx);
	}

	// Note
	int note = step.getNoteNumber();
	int keyOn = -1;	// BambooTracker note number of a key on
	switch (note) {
	case Step::NOTE_NONE:
		break;
	case Step::NOTE_KEY_OFF:
		row.note = (ch.kind != ChannelKind::Rhythm && info.releaseOnKeyOff) ? FUR_NOTE_RELEASE : FUR_NOTE_OFF;
		break;
	case Step::NOTE_KEY_CUT:
		row.note = FUR_NOTE_OFF;
		break;
	case Step::NOTE_ECHO0:
	case Step::NOTE_ECHO1:
	case Step::NOTE_ECHO2:
	case Step::NOTE_ECHO3:
	{
		size_t n = static_cast<size_t>(Step::NOTE_ECHO0 - note);
		if (n < st.echo.size()) keyOn = st.echo[n];
		break;
	}
	default:
		keyOn = note;
		break;
	}

	if (keyOn >= 0) {
		st.echo.push_front(keyOn);
		if (st.echo.size() > 4) st.echo.pop_back();

		switch (ch.kind) {
		case ChannelKind::Rhythm:
			row.note = FUR_NOTE_C0 + Note::DEFAULT_NOTE_NUM;
			if (target_ == FurExportTarget::YM2610B && rhythmInsts_[ch.chInSrc] >= 0)
				row.ins = rhythmInsts_[ch.chInSrc];
			break;
		case ChannelKind::SSG:
			row.note = std::min(FUR_NOTE_C0 + keyOn + SSG_TRANSPOSE, FUR_NOTE_OFF - 1);
			break;
		default:
			row.note = FUR_NOTE_C0 + keyOn;
			break;
		}

		if (ch.kind == ChannelKind::FM) {
			// Hardware envelope reset before key on
			if (info.envReset != st.hardReset) {
				row.fx.emplace_back(0x30, info.envReset ? 1 : 0);
				st.hardReset = info.envReset;
			}
			// Chip-wide LFO frequency, written by BambooTracker on key on
			st.usesLfo = info.lfoEnabled;
			int want = info.lfoEnabled ? info.lfoFreq : -1;
			if (want < 0) {
				bool other = false;
				for (size_t i = 0; i < chState_.size(); ++i)
					if (i != c && chans_[i].kind == ChannelKind::FM && chState_[i].usesLfo) other = true;
				if (other) want = lfoState_;
			}
			if (want != lfoState_) {
				row.fx.emplace_back(0x10, (want < 0) ? 0 : (0x10 | want));
				lfoState_ = want;
			}
		}
		else if (ch.kind == ChannelKind::ADPCM && st.btInst >= 0) {
			// Drumkit keys carry their own panning.
			auto kit = std::dynamic_pointer_cast<InstrumentDrumkit>(instMan_->getInstrumentSharedPtr(st.btInst));
			if (kit && kit->getSampleEnabled(keyOn)) {
				int pan = kit->getPan(keyOn);
				if (pan != st.pan && 0 <= pan && pan < 4) {
					static const int PAN[4] = { 0x00, 0x0f, 0xf0, 0xff };
					row.fx.emplace_back(0x08, PAN[pan]);
				}
			}
		}
	}

	// Track panning set by effects
	for (const auto& fx : row.fx) {
		if (fx.first == 0x08) {
			st.pan = ((fx.second & 0xf0) ? 2 : 0) | ((fx.second & 0x0f) ? 1 : 0);
		}
	}
	if (row.fx.size() > FUR_MAX_EFFECT_COLS) {
		row.fx.resize(FUR_MAX_EFFECT_COLS);
		warnings_.insert("Some rows needed more than 8 effect columns; extra effects were dropped.");
	}
	return row;
}

std::vector<std::string> FurConverter::write(BinaryContainer& ctr)
{
	setupChannels();
	const bool ext = (song_.getStyle().type == SongType::FM3chExpanded);
	uint8_t sysId;
	size_t expectedChans;
	if (target_ == FurExportTarget::YM2608) sysId = ext ? FUR_SYS_YM2608_EXT : FUR_SYS_YM2608;
	else sysId = ext ? FUR_SYS_YM2610B_EXT : FUR_SYS_YM2610B;
	expectedChans = ext ? 19 : 16;
	if (chans_.size() != expectedChans) throw std::runtime_error("Unexpected track layout");

	createRhythmInstruments();

	// Timing: BambooTracker's step length is 2.5 * tickRate * speed / tempo ticks,
	// which is Furnace's virtual tempo with numerator tempo and denominator 2.5 * tickRate.
	const unsigned int tickRate = mod_->getTickFrequency();
	tempoDenom_ = clampValue(static_cast<int>(std::lround(BT_TEMPO_TICK_FACTOR * tickRate)), 1, 255);
	tempo_ = song_.getTempo();
	tempoMode_ = song_.isUsedTempo();

	std::vector<std::vector<int>> grooves;
	for (size_t i = 0; i < mod_->getGrooveCount() && i < 256; ++i) {
		std::vector<int> g = mod_->getGroove(static_cast<int>(i));
		if (g.size() > 16) {
			g.resize(16);
			warnings_.insert("Grooves longer than 16 steps were truncated.");
		}
		for (int& s : g) s = clampValue(s, 1, 255);
		if (g.empty()) g.push_back(1);
		grooves.push_back(std::move(g));
	}
	bool usesGroove = !tempoMode_;
	for (size_t t = 0; t < chans_.size() && !usesGroove; ++t) {
		Track& track = song_.getTrack(static_cast<int>(t));
		for (size_t o = 0; o < track.getOrderSize() && !usesGroove; ++o) {
			Pattern& pat = track.getPatternFromOrderNumber(static_cast<int>(o));
			for (size_t r = 0; r < pat.getSize() && !usesGroove; ++r) {
				Step& step = pat.getStep(static_cast<int>(r));
				for (int i = 0; i < Step::N_EFFECT; ++i) {
					if (effect_utils::validateEffect(chans_[t].src, step.getEffectId(i), step.getEffectValue(i)).type
							== EffectType::Groove) usesGroove = true;
				}
			}
		}
	}

	std::vector<int> speeds;
	int vTempoN;
	if (tempoMode_) {
		speeds = { clampValue(song_.getSpeed(), 1, 255) };
		vTempoN = clampValue(tempo_, 1, 255);
	}
	else {
		int g = song_.getGroove();
		speeds = (0 <= g && g < static_cast<int>(grooves.size())) ? grooves[static_cast<size_t>(g)] : std::vector<int>{ 6 };
		vTempoN = tempoDenom_;
	}

	// Patterns: convert every order in sequence, keeping per-channel state as
	// in playback, and share identical results between orders.
	const size_t patLen = std::min(song_.getDefaultPatternSize(), static_cast<size_t>(256));
	const size_t ordLen = std::min(song_.getTrack(0).getOrderSize(), static_cast<size_t>(FUR_MAX_PATTERNS));
	std::vector<std::vector<FurPattern>> pats(chans_.size());
	std::vector<std::map<std::vector<int>, int>> patIndex(chans_.size());
	std::vector<std::vector<int>> orders(chans_.size(), std::vector<int>(ordLen, 0));
	for (size_t o = 0; o < ordLen; ++o) {
		std::vector<FurPattern> cur(chans_.size(), FurPattern(patLen));
		for (size_t r = 0; r < patLen; ++r) {
			for (size_t c = 0; c < chans_.size(); ++c) {
				Pattern& pat = song_.getTrack(static_cast<int>(c)).getPatternFromOrderNumber(static_cast<int>(o));
				if (r < pat.getSize()) cur[c][r] = convertStep(c, pat.getStep(static_cast<int>(r)));
			}
		}
		for (size_t c = 0; c < chans_.size(); ++c) {
			std::vector<int> key;
			for (const FurRow& row : cur[c]) {
				key.insert(key.end(), { row.note, row.ins, row.vol, static_cast<int>(row.fx.size()) });
				for (const auto& fx : row.fx) key.insert(key.end(), { fx.first, fx.second });
			}
			auto it = patIndex[c].find(key);
			if (it != patIndex[c].end()) {
				orders[c][o] = it->second;
			}
			else if (pats[c].size() < FUR_MAX_PATTERNS) {
				int idx = static_cast<int>(pats[c].size());
				patIndex[c][key] = idx;
				pats[c].push_back(std::move(cur[c]));
				orders[c][o] = idx;
			}
			else {
				orders[c][o] = FUR_MAX_PATTERNS - 1;
				warnings_.insert("A channel needed more than 256 distinct patterns; some orders reuse another pattern.");
			}
		}
	}
	std::vector<size_t> effectCols(chans_.size(), 1);
	for (size_t c = 0; c < chans_.size(); ++c) {
		for (const auto& p : pats[c])
			for (const auto& row : p)
				effectCols[c] = std::max(effectCols[c], std::min(row.fx.size(), static_cast<size_t>(FUR_MAX_EFFECT_COLS)));
	}
	size_t numPats = 0;
	for (const auto& p : pats) numPats += p.size();

	/***** Header *****/
	ctr.appendString("-Furnace module-");
	ctr.appendUint16(FUR_VERSION);
	ctr.appendUint16(0);
	ctr.appendUint32(32);	// Song info pointer
	ctr.appendUint32(0);
	ctr.appendUint32(0);

	/***** Song info *****/
	size_t infoOfs = beginBlock(ctr, "INFO");
	ctr.appendUint8(0);	// Time base
	ctr.appendUint8(static_cast<uint8_t>(speeds[0]));
	ctr.appendUint8(static_cast<uint8_t>((speeds.size() >= 2) ? speeds[1] : speeds[0]));
	ctr.appendUint8(1);	// Arpeggio speed
	appendFloat(ctr, static_cast<float>(tickRate));
	ctr.appendUint16(static_cast<uint16_t>(patLen));
	ctr.appendUint16(static_cast<uint16_t>(ordLen));
	ctr.appendUint8(static_cast<uint8_t>(std::min<size_t>(mod_->getStepHighlight1Distance(), 255)));
	ctr.appendUint8(static_cast<uint8_t>(std::min<size_t>(mod_->getStepHighlight2Distance(), 255)));
	ctr.appendUint16(static_cast<uint16_t>(insts_.size()));
	ctr.appendUint16(0);	// Wavetables
	ctr.appendUint16(static_cast<uint16_t>(samples_.size()));
	ctr.appendUint32(static_cast<uint32_t>(numPats));
	for (int i = 0; i < FUR_MAX_CHIPS; ++i) ctr.appendUint8(i ? 0 : sysId);
	for (int i = 0; i < FUR_MAX_CHIPS; ++i) ctr.appendUint8(64);	// Chip volume 1.0
	for (int i = 0; i < FUR_MAX_CHIPS; ++i) ctr.appendUint8(0);		// Chip panning
	size_t flagPtrOfs = ctr.size();
	for (int i = 0; i < FUR_MAX_CHIPS; ++i) ctr.appendUint32(0);	// Chip flag pointers
	appendCString(ctr, mod_->getTitle());
	appendCString(ctr, mod_->getAuthor());
	appendFloat(ctr, 440.0f);	// A-4 tuning
	// Compatibility flags (Furnace defaults for new songs).
	static const uint8_t COMPAT1[20] = { 0, 2, 2, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 };
	for (uint8_t b : COMPAT1) ctr.appendUint8(b);

	size_t ptrOfs = ctr.size();
	for (size_t i = 0; i < insts_.size() + samples_.size() + numPats; ++i) ctr.appendUint32(0);

	for (size_t c = 0; c < chans_.size(); ++c)
		for (size_t o = 0; o < ordLen; ++o) ctr.appendUint8(static_cast<uint8_t>(orders[c][o]));
	for (size_t c = 0; c < chans_.size(); ++c) ctr.appendUint8(static_cast<uint8_t>(effectCols[c]));
	for (size_t c = 0; c < chans_.size(); ++c) ctr.appendUint8(3);	// Shown in pattern and oscilloscope
	for (size_t c = 0; c < chans_.size(); ++c) ctr.appendUint8(0);	// Not collapsed
	for (size_t c = 0; c < chans_.size(); ++c) appendCString(ctr, "");	// Channel names
	for (size_t c = 0; c < chans_.size(); ++c) appendCString(ctr, "");	// Channel short names
	appendCString(ctr, mod_->getComment());
	appendFloat(ctr, 1.0f);	// Master volume
	static const uint8_t COMPAT2[28] = { 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 4, 0, 0, 1, 1, 0, 0, 0, 0, 2, 0, 1, 0, 0, 0 };
	for (uint8_t b : COMPAT2) ctr.appendUint8(b);
	ctr.appendUint16(static_cast<uint16_t>(vTempoN));
	ctr.appendUint16(static_cast<uint16_t>(tempoDenom_));
	appendCString(ctr, song_.getTitle());	// Sub-song name
	appendCString(ctr, "");	// Sub-song comment
	ctr.appendUint8(0);	// Additional sub-songs
	ctr.appendUint8(0);
	ctr.appendUint8(0);
	ctr.appendUint8(0);
	appendCString(ctr, "");	// System name
	appendCString(ctr, "");	// Album/category
	appendCString(ctr, "");	// Song name (Japanese)
	appendCString(ctr, "");	// Author (Japanese)
	appendCString(ctr, "");	// System name (Japanese)
	appendCString(ctr, "");	// Album/category (Japanese)
	appendFloat(ctr, 1.0f);	// Chip volume
	appendFloat(ctr, 0.0f);	// Chip panning
	appendFloat(ctr, 0.0f);	// Chip front/rear balance
	ctr.appendUint32(0);	// Patchbay connections
	ctr.appendUint8(1);		// Automatic patchbay
	for (int i = 0; i < 8; ++i) ctr.appendUint8(0);	// More compatibility flags
	ctr.appendUint8(static_cast<uint8_t>(std::min<size_t>(speeds.size(), 16)));
	for (size_t i = 0; i < 16; ++i) ctr.appendUint8(static_cast<uint8_t>((i < speeds.size()) ? speeds[i] : 0));
	if (usesGroove) {
		ctr.appendUint8(static_cast<uint8_t>(grooves.size()));
		for (const auto& g : grooves) {
			ctr.appendUint8(static_cast<uint8_t>(g.size()));
			for (size_t i = 0; i < 16; ++i) ctr.appendUint8(static_cast<uint8_t>((i < g.size()) ? g[i] : 0));
		}
	}
	else {
		ctr.appendUint8(0);
	}
	size_t assetDirOfs = ctr.size();
	for (int i = 0; i < 3; ++i) ctr.appendUint32(0);
	endBlock(ctr, infoOfs);

	/***** Chip flags *****/
	{
		int ssgVol = static_cast<int>(std::lround(FUR_SSG_VOL_UNITY
												  * std::pow(10.0, (ssgMixDb_ - FUR_SSG_LEVEL_OFFSET_DB) / 20.0)));
		ctr.writeUint32(flagPtrOfs, static_cast<uint32_t>(ctr.size()));
		size_t ofs = beginBlock(ctr, "FLAG");
		appendCString(ctr, "ssgVol=" + std::to_string(clampValue(ssgVol, 0, 1024)) + "\n");
		endBlock(ctr, ofs);
	}

	/***** Asset directories (empty: everything uncategorized) *****/
	for (int i = 0; i < 3; ++i) {
		ctr.writeUint32(assetDirOfs + static_cast<size_t>(i) * 4, static_cast<uint32_t>(ctr.size()));
		size_t ofs = beginBlock(ctr, "ADIR");
		ctr.appendUint32(0);
		endBlock(ctr, ofs);
	}

	/***** Instruments, samples and patterns *****/
	size_t ptr = ptrOfs;
	for (const FurInstrument& ins : insts_) {
		ctr.writeUint32(ptr, static_cast<uint32_t>(ctr.size()));
		ptr += 4;
		writeInstrument(ctr, ins);
	}
	for (const FurSample& smp : samples_) {
		ctr.writeUint32(ptr, static_cast<uint32_t>(ctr.size()));
		ptr += 4;
		writeSample(ctr, smp);
	}
	for (size_t c = 0; c < chans_.size(); ++c) {
		for (size_t i = 0; i < pats[c].size(); ++i) {
			ctr.writeUint32(ptr, static_cast<uint32_t>(ctr.size()));
			ptr += 4;
			writePattern(ctr, static_cast<int>(c), static_cast<int>(i), pats[c][i], effectCols[c]);
		}
	}

	return std::vector<std::string>(warnings_.begin(), warnings_.end());
}
}

std::vector<std::string> writeFur(BinaryContainer& ctr, std::weak_ptr<Module> mod,
								  std::weak_ptr<InstrumentsManager> instMan, int songNum,
								  FurExportTarget target, const std::vector<uint8_t>& rhythmRom,
								  double ssgMixDb)
{
	FurConverter conv(mod.lock(), instMan.lock(), songNum, target, rhythmRom, ssgMixDb);
	return conv.write(ctr);
}
}
