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

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <cstdint>

class Module;
class InstrumentsManager;

namespace io
{
class BinaryContainer;

/// Chip the exported Furnace module is built for.
enum class FurExportTarget
{
	YM2608,		///< Yamaha YM2608 (OPNA): channel layout identical to BambooTracker's
	YM2610B		///< Yamaha YM2610B (OPNB2): rhythm tracks become ADPCM-A channels
};

/**
 * @brief Convert one song of a module to a Furnace module (.fur).
 *
 * The file is written in the format of Furnace 0.6.8.3 (format version 232),
 * the latest stable release at the time of writing, so that both stable and
 * development builds of Furnace can open it. It is stored uncompressed, which
 * Furnace accepts as well.
 *
 * Furnace plays modules with its own engine, so the result is meant to be a
 * faithful starting point for further editing in Furnace rather than a
 * sample-exact rendition: instrument sequences become Furnace macros, effects
 * are translated to their closest Furnace equivalents and the few
 * BambooTracker features without an equivalent are dropped (listed in the
 * returned warnings).
 *
 * @param ctr Output container.
 * @param mod Module to export.
 * @param instMan Instrument manager of the module.
 * @param songNum Number of the song to export.
 * @param target Chip to build the Furnace module for.
 * @param rhythmRom BambooTracker's built-in OPNA rhythm ROM (0x2000 bytes).
 *		  Its samples are embedded as ADPCM-A samples for the YM2610B target,
 *		  which has no rhythm ROM of its own; ignored for YM2608.
 * @param ssgMixDb Level of SSG relative to FM in the mixer (dB), reproduced
 *		  with Furnace's SSG volume chip setting.
 * @return Human-readable notes about BambooTracker features that could not be
 *		   converted. Empty if everything was converted.
 */
std::vector<std::string> writeFur(BinaryContainer& ctr, std::weak_ptr<Module> mod,
								  std::weak_ptr<InstrumentsManager> instMan, int songNum,
								  FurExportTarget target, const std::vector<uint8_t>& rhythmRom,
								  double ssgMixDb);
}
