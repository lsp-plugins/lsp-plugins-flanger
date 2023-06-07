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

#include <lsp-plug.in/dsp-units/util/RingBuffer.h>
#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/dsp-units/ctl/Toggle.h>
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
                typedef float (*lfo_func_t)(float phase);

                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Bypass        sBypass;            // Bypass
                    dspu::RingBuffer    sRing;              // Ring buffer for flanger effect processing
                    dspu::RingBuffer    sFeedback;          // Feedback delay buffer

                    // Parameters
                    uint32_t            nOldPhaseShift;     // Old phase shift
                    uint32_t            nPhaseShift;        // Phase shift
                    size_t              nLfoType;           // Type of LFO
                    lfo_func_t          pLfoFunc;           // LFO function
                    bool                bSyncLfo;           // Synchronize LFO graph

                    float              *vIn;                // Input buffer
                    float              *vOut;               // Output buffer
                    float              *vBuffer;            // Processed signal
                    float              *vLfoMesh;           // LFO mesh amplitude data

                    // Input ports
                    plug::IPort        *pIn;                // Input port
                    plug::IPort        *pOut;               // Output port

                    // Output ports
                    plug::IPort        *pPhase;             // Current phase
                    plug::IPort        *pLfoType;           // Oscillator type
                    plug::IPort        *pLfoShift;          // LFO shift
                    plug::IPort        *pLfoMesh;           // LFO mesh
                    plug::IPort        *pInLevel;           // Input signal level
                    plug::IPort        *pOutLevel;          // Output signal level
                } channel_t;

            protected:
                static lfo_func_t   all_lfo_functions[];

            protected:
                dspu::Toggle        sReset;             // Reset toggle

                size_t              nChannels;          // Number of channels
                channel_t          *vChannels;          // Delay channels
                float              *vBuffer;            // Temporary buffer for audio processing
                float              *vLfoPhase;          // LFO mesh phase data

                uint32_t            nOldDepthMin;       // Old minimum depth
                uint32_t            nDepthMin;          // Minimum depth value in samples
                uint32_t            nOldDepth;          // Old depth value in samples
                uint32_t            nDepth;             // Depth value in samples
                uint32_t            nInitPhase;         // Initial phase
                uint32_t            nPhase;             // Current phase value
                uint32_t            nOldPhaseStep;      // Old phase increment
                uint32_t            nPhaseStep;         // Phase increment
                float               fOldAmount;         // Old overal amount
                float               fAmount;            // The overall amount
                float               fOldFeedGain;       // Old feedback gain
                float               fFeedGain;          // Feed-back gain
                size_t              nOldFeedDelay;      // Old feedback delay
                size_t              nFeedDelay;         // Feed-back delay
                float               fOldInGain;         // Old input gain
                float               fInGain;            // Input gain
                float               fOldDryGain;        // Old dry gain
                float               fDryGain;           // Dry gain (unprocessed signal)
                float               fOldWetGain;        // Old wet gain
                float               fWetGain;           // Wet gain (processed signal)
                bool                bMidSide;           // Mid/Side mode

                plug::IPort        *pBypass;            // Bypass
                plug::IPort        *pRate;              // Rate
                plug::IPort        *pInitPhase;         // Initial Phase
                plug::IPort        *pPhaseDiff;         // Phase difference between left and right
                plug::IPort        *pReset;             // Reset phase to default

                plug::IPort        *pMsSwitch;          // Mid/Side switch
                plug::IPort        *pDepthMin;          // Minimal depth
                plug::IPort        *pDepth;             // Depth
                plug::IPort        *pSignalPhase;       // Signal phase
                plug::IPort        *pAmount;            // Amount
                plug::IPort        *pFeedGain;          // Feedback gain
                plug::IPort        *pFeedDelay;         // Feedback delay
                plug::IPort        *pFeedPhase;         // Feedback phase
                plug::IPort        *pInGain;            // Input gain
                plug::IPort        *pDry;               // Dry gain
                plug::IPort        *pWet;               // Wet gain
                plug::IPort        *pOutGain;           // Output gain

                uint8_t            *pData;              // Allocated data

            protected:
                static float        lfo_triangular(float phase);
                static float        lfo_sine(float phase);
                static float        lfo_step_sine(float phase);
                static float        lfo_cubic(float phase);
                static float        lfo_step_cubic(float phase);
                static float        lfo_parabolic(float phase);
                static float        lfo_rev_parabolic(float phase);
                static float        lfo_logarithmic(float phase);
                static float        lfo_rev_logarithmic(float phase);
                static float        lfo_sqrt(float phase);
                static float        lfo_rev_sqrt(float phase);
                static float        lfo_circular(float phase);
                static float        lfo_rev_circular(float phase);

                static inline uint32_t  phase_to_int(float phase);
                static inline float     lerp(float o_value, float n_value, float k);
                static inline int32_t   ilerp(int32_t o_value, int32_t n_value, float k);

            public:
                explicit flanger(const meta::plugin_t *meta);
                virtual ~flanger() override;

                virtual void        init(plug::IWrapper *wrapper, plug::IPort **ports) override;
                virtual void        destroy() override;

            public:
                virtual void        update_sample_rate(long sr) override;
                virtual void        update_settings() override;
                virtual void        process(size_t samples) override;
                virtual void        ui_activated() override;
                virtual bool        inline_display(plug::ICanvas *cv, size_t width, size_t height) override;
                virtual void        dump(dspu::IStateDumper *v) const override;
        };

    } /* namespace plugins */
} /* namespace lsp */


#endif /* PRIVATE_PLUGINS_FLANGER_H_ */

