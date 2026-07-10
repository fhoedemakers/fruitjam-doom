//
// Copyright(C) 1993-1996 Id Software, Inc.
// Copyright(C) 2005-2014 Simon Howard
// Copyright(C) 2008 David Flater
// Copyright(C) 2021-2022 Graham Sanderson
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// DESCRIPTION:
//	System interface for sound.

#ifndef __I_PICO_SOUND__
#define __I_PICO_SOUND__

#include "pico.h"
typedef struct audio_buffer audio_buffer_t;

// Allow the board-level cflags to force a specific sample rate (fruitjam_cflags.h
// pins us to 44100 so pico_hdmi's ACR table has a matching entry — the emu8950
// native 49716 Hz falls through to the 48 kHz default N/CTS and the sink hears
// static from the constant FIFO overrun).
#ifndef PICO_SOUND_SAMPLE_FREQ
#if USE_EMU8950_OPL
#define PICO_SOUND_SAMPLE_FREQ 49716
#else
#define PICO_SOUND_SAMPLE_FREQ 44100
#endif
#endif

#ifndef NUM_SOUND_CHANNELS
// this is the defaul tin game not 16
#define NUM_SOUND_CHANNELS 8
#endif

void I_PicoSoundSetMusicGenerator(void (*generator)(audio_buffer_t *buffer));
bool I_PicoSoundIsInitialized(void);
void I_PicoSoundFade(bool in);
bool I_PicoSoundFading(void);

// Counting lock over the mixer/OPL state. While held, the hardware-timer
// audio pump (i_picosound.c) skips its mix, so main-loop code can safely
// mutate channel or OPL state. Nesting-safe; main-loop (thread) context
// only — the pump ISR itself never blocks on it, it just yields.
// opl/opl_pico.c wires OPL_Lock()/OPL_Unlock() to these.
void I_PicoSoundLock(void);
void I_PicoSoundUnlock(void);
#endif
