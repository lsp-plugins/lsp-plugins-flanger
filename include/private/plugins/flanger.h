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
                    uint32_t            nPhase;             // Current phase value

                    // Input ports
                    plug::IPort        *pIn;                // Input port
                    plug::IPort        *pOut;               // Output port

                    // Output ports
                    plug::IPort        *pInLevel;           // Input signal level
                    plug::IPort        *pOutLevel;          // Output signal level
                } channel_t;

            protected:
                size_t              nChannels;          // Number of channels
                channel_t          *vChannels;          // Delay channels
                float              *vBuffer;            // Temporary buffer for audio processing

                float               fDepthMin;          // Minimum depth value
                float               fDepth;             // Depth value
                uint32_t            nPhaseStep;         // Phase increment
                float               fDryGain;           // Dry gain (unprocessed signal)
                float               fWetGain;           // Wet gain (processed signal)

                plug::IPort        *pBypass;            // Bypass
                plug::IPort        *pDepthMin;          // Minimal depth
                plug::IPort        *pDepth;             // Depth
                plug::IPort        *pRate;              // Rate
                plug::IPort        *pAmount;            // Amount
                plug::IPort        *pInitPhase;         // Initial Phase
                plug::IPort        *pDry;               // Dry gain
                plug::IPort        *pWet;               // Wet gain
                plug::IPort        *pOutGain;           // Output gain

                uint8_t            *pData;              // Allocated data

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

