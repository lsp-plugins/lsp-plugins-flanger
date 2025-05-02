/*
 * Copyright (C) 2020 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2020 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-flanger
 * Created on: 25 нояб. 2020 г.
 *
 * lsp-plugins-flanger is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * lsp-plugins-flanger is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lsp-plugins-flanger. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef PRIVATE_META_FLANGER_H_
#define PRIVATE_META_FLANGER_H_

#include <lsp-plug.in/plug-fw/meta/types.h>
#include <lsp-plug.in/plug-fw/const.h>

namespace lsp
{
    //-------------------------------------------------------------------------
    // Plugin metadata
    namespace meta
    {
        typedef struct flanger
        {
            static constexpr float  DEPTH_MIN_MIN       = 0.01f;
            static constexpr float  DEPTH_MIN_MAX       = 10.0f;
            static constexpr float  DEPTH_MIN_DFL       = 0.25f;
            static constexpr float  DEPTH_MIN_STEP      = 0.003f;

            static constexpr float  DEPTH_MIN           = 0.1f;
            static constexpr float  DEPTH_MAX           = 20.0f;
            static constexpr float  DEPTH_DFL           = 2.0f;
            static constexpr float  DEPTH_STEP          = 0.005f;

            static constexpr float  SHIFT_MIN           = 0.0f;
            static constexpr float  SHIFT_MAX           = 1.0f;
            static constexpr float  SHIFT_DFL           = 0.0f;
            static constexpr float  SHIFT_STEP          = 0.04f;

            static constexpr float  PHASE_MIN           = 0.0f;
            static constexpr float  PHASE_MAX           = 360.0f;
            static constexpr float  PHASE_DFL           = 0.0f;
            static constexpr float  PHASE_STEP          = 0.04f;

            static constexpr float  RATE_MIN            = 0.01f;
            static constexpr float  RATE_MAX            = 20.0f;
            static constexpr float  RATE_DFL            = 0.25f;
            static constexpr float  RATE_STEP           = 0.005f;

            static constexpr float  CROSSFADE_MIN       = 0.0f;
            static constexpr float  CROSSFADE_MAX       = 50.0f;
            static constexpr float  CROSSFADE_DFL       = 0.0f;
            static constexpr float  CROSSFADE_STEP      = 0.015f;

            static constexpr float  FEEDBACK_GAIN_MIN   = 0.0f;
            static constexpr float  FEEDBACK_GAIN_MAX   = 0.891250938134f; // -1 dB
            static constexpr float  FEEDBACK_GAIN_DFL   = GAIN_AMP_M_6_DB;
            static constexpr float  FEEDBACK_GAIN_STEP  = 0.015f;

            static constexpr float  FEEDBACK_DELAY_MIN  = 0.0f;
            static constexpr float  FEEDBACK_DELAY_MAX  = 5.0f;
            static constexpr float  FEEDBACK_DELAY_DFL  = 0.0f;
            static constexpr float  FEEDBACK_DELAY_STEP = 0.001f;

            static constexpr float  FEEDBACK_DRIVE_MIN  = 0.0f;
            static constexpr float  FEEDBACK_DRIVE_MAX  = 1.0f;
            static constexpr float  FEEDBACK_DRIVE_DFL  = 0.0f;
            static constexpr float  FEEDBACK_DRIVE_STEP = 0.025f;

            static constexpr float  TEMPO_MIN           = 20.0f;
            static constexpr float  TEMPO_MAX           = 360.0f;
            static constexpr float  TEMPO_STEP          = 0.1f;
            static constexpr float  TEMPO_DFL           = 120.0f;

            static constexpr float  FRACTION_MIN        = 1.0f / 64.0f;
            static constexpr float  FRACTION_MAX        = 8.0f;
            static constexpr float  FRACTION_STEP       = 1.0f / 64.0f;
            static constexpr float  FRACTION_DFL        = 1.0f;

            static constexpr float  DENOMINATOR_MIN     = 1.0f;
            static constexpr float  DENOMINATOR_MAX     = 64.0f;
            static constexpr float  DENOMINATOR_STEP    = 1.0f;
            static constexpr float  DENOMINATOR_DFL     = 4.0f;

            static constexpr size_t LFO_MESH_SIZE       = 361;
            static constexpr size_t OVERSAMPLING_MAX    = 8;

            static constexpr float  DELAY_OUT_MAX_TIME  = 10000.0f;

            enum osc_period_t
            {
                OSC_FULL,
                OSC_FIRST,
                OSC_LAST
            };
        } flanger;

        // Plugin type metadata
        extern const plugin_t flanger_mono;
        extern const plugin_t flanger_stereo;

    } /* namespace meta */
} /* namespace lsp */

#endif /* PRIVATE_META_FLANGER_H_ */
