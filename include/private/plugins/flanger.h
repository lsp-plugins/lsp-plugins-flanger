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

#ifndef PRIVATE_PLUGINS_FLANGER_H_
#define PRIVATE_PLUGINS_FLANGER_H_

#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/dsp-units/util/RingBuffer.h>
#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/plug-fw/plug.h>
#include <private/meta/flanger.h>

namespace lsp
{
    namespace plugins
    {
        /**
         * Base class for the latency compensation delay
         */
        class flanger: public plug::Module
        {
            private:
                flanger & operator = (const flanger &);
                flanger (const flanger &);

            protected:
                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Bypass        sBypass;            // Bypass
                    dspu::RingBuffer    sRing;              // Ring buffer for flanger effect processing

                    // Parameters
                    uint32_t            nInitPhase;         // Initial phase value
                    float               fFeedback;          // Feedback sample
                    float              *vBuffer;            // Processed signal
                    float              *vIn;                // Input buffer
                    float              *vOut;               // Output buffer

                    // Input ports
                    plug::IPort        *pIn;                // Input port
                    plug::IPort        *pOut;               // Output port

                    // Output ports
                    plug::IPort        *pInLevel;           // Input signal level
                    plug::IPort        *pOutLevel;          // Output signal level
                } channel_t;

                typedef float (*lfo_func_t)(float phase);

            protected:
                static lfo_func_t   all_lfo_functions[];

            protected:
                size_t              nChannels;          // Number of channels
                channel_t          *vChannels;          // Delay channels
                float              *vBuffer;            // Temporary buffer for audio processing

                size_t              nDepthMin;          // Minimum depth value in samples
                size_t              nDepth;             // Depth value in samples
                uint32_t            nPhase;             // Current phase value
                uint32_t            nPhaseStep;         // Phase increment
                lfo_func_t          pLfoFunc;           // LFO function
                float               fAmount;            // The overall amount
                float               fFeedGain;          // Feed-back gain
                float               fDryGain;           // Dry gain (unprocessed signal)
                float               fWetGain;           // Wet gain (processed signal)

                plug::IPort        *pBypass;            // Bypass
                plug::IPort        *pDepthMin;          // Minimal depth
                plug::IPort        *pDepth;             // Depth
                plug::IPort        *pRate;              // Rate
                plug::IPort        *pFunc;              // Oscillator function
                plug::IPort        *pAmount;            // Amount
                plug::IPort        *pInitPhase;         // Initial Phase
                plug::IPort        *pFeedGain;          // Feedback gain
                plug::IPort        *pFeedPhase;         // Feedback phase
                plug::IPort        *pDry;               // Dry gain
                plug::IPort        *pWet;               // Wet gain
                plug::IPort        *pOutGain;           // Output gain

                uint8_t            *pData;              // Allocated data

            protected:
                static float        lfo_triangular(float phase);
                static float        lfo_sine(float phase);
                static float        lfo_cubic(float phase);
                static float        lfo_parabolic(float phase);
                static float        lfo_rev_parabolic(float phase);
                static float        lfo_logarithmic(float phase);
                static float        lfo_rev_logarithmic(float phase);
                static float        lfo_sqrt(float phase);
                static float        lfo_rev_sqrt(float phase);

            public:
                explicit flanger(const meta::plugin_t *meta);
                virtual ~flanger();

                virtual void        init(plug::IWrapper *wrapper, plug::IPort **ports);
                void                destroy();

            public:
                virtual void        update_sample_rate(long sr);
                virtual void        update_settings();
                virtual void        process(size_t samples);
                virtual void        dump(dspu::IStateDumper *v) const;
        };

    } /* namespace plugins */
} /* namespace lsp */


#endif /* PRIVATE_PLUGINS_FLANGER_H_ */

