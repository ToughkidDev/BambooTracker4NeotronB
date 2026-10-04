# File I/O

## Module

The tracker enables to open and save to .btm (BambooTracker module file).

## Instrument

The tracker can load instrument from the following files.

- .bti (BambooTracker instrument file)
- .dmp (DefleMask preset file)
- .tfi (TFM Music Maker instrument file)
- .vgi (VGM Music Maker instrument file)
- .opni (WOPN instrument file)
- .y12 (Gens KMod dump file)
- .ins (MVSTracker instrument file)
- .spb (Raw ADPCM sample file)

It also supports loading FM envelopes in plain text formats.  
ADPCM waveform editor supports .wav import (16-bit mono 2k-55.5kHz).

An instrument is saved as a .bti file.

## Bank

The tracker can load bank from the following files.

- .btb (BambooTracker bank file)
- .wopn (WOPN bank file)
- .ff (PMD FF file)
- .ppc (PMD PPC file)
- .pvi (FMP PVI file)
- .dat (MUCOM88 voice file)
- .pzi (FMP PZI file)
- .p86 (PMD P86 file)
- .pps (PMD PPS file)
- .pmb (FM Towns PMB file)

A bank is saved as a .btb file.

## Export

The tracker can export a song to the following files:

- .wav (WAVE file)
- .vgm (VGM file)
- .s98 (S98 file)
- .fur (Furnace module, for [Furnace](https://github.com/tildearrow/furnace) 0.6.8.3 or later)

### Furnace module export

The current song is converted to a Furnace module for either the YM2608 (OPNA)
or the YM2610B (OPNB2) chip, chosen in the export dialog. Songs in FM3ch
expanded mode use Furnace's "Extended Channel 3" variant of the chip.

- YM2608: tracks map one-to-one to Furnace's YM2608 channels.
- YM2610B: rhythm tracks become ADPCM-A channels playing the built-in rhythm
  samples, which are embedded in the module; the ADPCM track becomes the
  ADPCM-B channel.

Instrument sequences are converted to Furnace macros and effects to their
closest Furnace equivalents. SSG notes are transposed up one octave and the SSG
volume chip setting is adjusted to the module mixer so that the result sounds
like it does in BambooTracker. Furnace plays the module with its own engine,
so the result is a close starting point for further editing rather than a
sample-exact copy. The following have no Furnace equivalent and are dropped
(the export reports them): brightness (`B0xx`), fine detune (`FPxx`), volume
delay (`Mxyy`), extended volume slide (`EAxy`), register writes (`0Xxx`,
`0Yxx`, `0Zxx`) and the volume change of retrigger (`0Kxy`).
