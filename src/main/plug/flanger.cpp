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

#include <lsp-plug.in/common/alloc.h>
#include <lsp-plug.in/common/debug.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/plug-fw/meta/func.h>

#include <private/plugins/flanger.h>

/* The size of temporary buffer for audio processing */
static constexpr size_t BUFFER_SIZE             = 512;
static constexpr float  PHASE_COEFF             = 1.0f / float(0x100000000LL);
static constexpr float  REV_LN100               = 0.5f / M_LN10;

namespace lsp
{
    static plug::IPort *TRACE_PORT(plug::IPort *p)
    {
        lsp_trace("  port id=%s", (p)->metadata()->id);
        return p;
    }

    namespace plugins
    {
        //---------------------------------------------------------------------
        // Plugin factory
        static const meta::plugin_t *plugins[] =
        {
            &meta::flanger_mono,
            &meta::flanger_stereo
        };

        static plug::Module *plugin_factory(const meta::plugin_t *meta)
        {
            return new flanger(meta);
        }

        static plug::Factory factory(plugin_factory, plugins, 2);

        //---------------------------------------------------------------------
        // Implementation
        flanger::lfo_func_t flanger::all_lfo_functions[] =
        {
            flanger::lfo_triangular,
            flanger::lfo_sine,
            flanger::lfo_step_sine,
            flanger::lfo_cubic,
            flanger::lfo_step_cubic,
            flanger::lfo_parabolic,
            flanger::lfo_rev_parabolic,
            flanger::lfo_logarithmic,
            flanger::lfo_rev_logarithmic,
            flanger::lfo_sqrt,
            flanger::lfo_rev_sqrt,
            flanger::lfo_circular,
            flanger::lfo_rev_circular,
        };

        flanger::flanger(const meta::plugin_t *meta):
            Module(meta)
        {
            // Compute the number of audio channels by the number of inputs
            nChannels       = 0;
            for (const meta::port_t *p = meta->ports; p->id != NULL; ++p)
                if (meta::is_audio_in_port(p))
                    ++nChannels;

            // Initialize other parameters
            vChannels       = NULL;
            vBuffer         = NULL;
            vLfoPhase       = NULL;
            vLfoMesh        = NULL;

            nOldDepthMin    = meta::flanger::DEPTH_MIN_DFL;
            nDepthMin       = meta::flanger::DEPTH_MIN_DFL;
            nOldDepth       = meta::flanger::DEPTH_DFL;
            nDepth          = meta::flanger::DEPTH_DFL;
            nInitPhase      = 0;
            nPhase          = meta::flanger::PHASE_DFL;
            nOldPhaseStep   = 0;
            nPhaseStep      = 0;
            nLfoType        = -1;
            pLfoFunc        = lfo_triangular;
            fOldAmount      = 0.0f;
            fAmount         = 0.0f;
            fOldFeedGain    = 0.0f;
            fFeedGain       = 0.0f;
            nOldFeedDelay   = 0;
            nFeedDelay      = 0;
            fOldDryGain     = 0.0f;
            fDryGain        = 0.0f;
            fOldWetGain     = 0.0f;
            fWetGain        = 0.0f;
            bSyncLfo        = true;

            pBypass         = NULL;
            pRate           = NULL;
            pLfoType        = NULL;
            pInitPhase      = NULL;
            pPhaseDiff      = NULL;
            pReset          = NULL;
            pLfoMesh        = NULL;

            pDepthMin       = NULL;
            pDepth          = NULL;
            pSignalPhase    = NULL;
            pAmount         = NULL;
            pFeedGain       = NULL;
            pFeedDelay      = NULL;
            pFeedPhase      = NULL;
            pDry            = NULL;
            pWet            = NULL;
            pOutGain        = NULL;

            pData           = NULL;
        }

        flanger::~flanger()
        {
            destroy();
        }

        void flanger::init(plug::IWrapper *wrapper, plug::IPort **ports)
        {
            // Call parent class for initialization
            Module::init(wrapper, ports);

            // Estimate the number of bytes to allocate
            size_t szof_channels    = align_size(sizeof(channel_t) * nChannels, OPTIMAL_ALIGN);
            size_t mesh_buf_sz      = align_size(meta::flanger::LFO_MESH_SIZE * sizeof(float), OPTIMAL_ALIGN);
            size_t buf_sz           = BUFFER_SIZE * sizeof(float);
            size_t alloc            =
                szof_channels +         // vChannels
                buf_sz +                // vBuffer
                mesh_buf_sz +           // vLfoPhase
                mesh_buf_sz +           // vLfoMesh
                buf_sz * nChannels;     // channel_t::vBuffer

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;

            // Initialize pointers to channels and temporary buffer
            vChannels               = reinterpret_cast<channel_t *>(ptr);
            ptr                    += szof_channels;
            vBuffer                 = reinterpret_cast<float *>(ptr);
            ptr                    += buf_sz;
            vLfoPhase               = reinterpret_cast<float *>(ptr);
            ptr                    += mesh_buf_sz;
            vLfoMesh                = reinterpret_cast<float *>(ptr);
            ptr                    += mesh_buf_sz;

            for (size_t i=0; i < nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Construct in-place DSP processors
                c->sBypass.construct();
                c->sRing.construct();
                c->sFeedback.construct();

                c->nOldPhaseShift       = 0;
                c->nPhaseShift          = 0;
                c->vBuffer              = reinterpret_cast<float *>(ptr);
                ptr                    += buf_sz;
                c->vIn                  = NULL;
                c->vOut                 = NULL;

                c->pIn                  = NULL;
                c->pOut                 = NULL;

                c->pPhase               = NULL;
                c->pLfoShift            = NULL;
                c->pInLevel             = NULL;
                c->pOutLevel            = NULL;
            }

            // Bind ports
            lsp_trace("Binding input ports");
            size_t port_id      = 0;

            // Bind input audio ports
            for (size_t i=0; i<nChannels; ++i)
                vChannels[i].pIn    = TRACE_PORT(ports[port_id++]);

            // Bind output audio ports
            for (size_t i=0; i<nChannels; ++i)
                vChannels[i].pOut   = TRACE_PORT(ports[port_id++]);

            // Bind bypass
            lsp_trace("Binding common ports");
            pBypass             = TRACE_PORT(ports[port_id++]);
            pRate               = TRACE_PORT(ports[port_id++]);
            pLfoType            = TRACE_PORT(ports[port_id++]);
            pInitPhase          = TRACE_PORT(ports[port_id++]);
            if (nChannels > 1)
                pPhaseDiff          = TRACE_PORT(ports[port_id++]);
            pReset              = TRACE_PORT(ports[port_id++]);
            pLfoMesh            = TRACE_PORT(ports[port_id++]);

            pDepthMin           = TRACE_PORT(ports[port_id++]);
            pDepth              = TRACE_PORT(ports[port_id++]);
            pSignalPhase        = TRACE_PORT(ports[port_id++]);
            pAmount             = TRACE_PORT(ports[port_id++]);
            pFeedGain           = TRACE_PORT(ports[port_id++]);
            pFeedDelay          = TRACE_PORT(ports[port_id++]);
            pFeedPhase          = TRACE_PORT(ports[port_id++]);
            pDry                = TRACE_PORT(ports[port_id++]);
            pWet                = TRACE_PORT(ports[port_id++]);
            pOutGain            = TRACE_PORT(ports[port_id++]);

            // Bind output meters
            lsp_trace("Binding channel ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                c->pPhase               = TRACE_PORT(ports[port_id++]);
                c->pLfoShift            = TRACE_PORT(ports[port_id++]);
                c->pInLevel             = TRACE_PORT(ports[port_id++]);
                c->pOutLevel            = TRACE_PORT(ports[port_id++]);
            }
        }

        void flanger::destroy()
        {
            Module::destroy();

            // Destroy channels
            if (vChannels != NULL)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];
                    c->sBypass.destroy();
                    c->sRing.destroy();
                    c->sFeedback.destroy();
                }
                vChannels   = NULL;
            }

            vBuffer     = NULL;

            // Free previously allocated data chunk
            if (pData != NULL)
            {
                free_aligned(pData);
                pData       = NULL;
            }
        }

        void flanger::update_sample_rate(long sr)
        {
            // Update sample rate for the bypass processors
            size_t max_delay = dspu::millis_to_samples(sr, meta::flanger::DEPTH_MIN_MAX + meta::flanger::DEPTH_MAX);
            size_t max_feedback = max_delay + dspu::millis_to_samples(sr, meta::flanger::FEEDBACK_DELAY_MAX);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];
                c->sRing.init(max_delay + BUFFER_SIZE);
                c->sFeedback.init(max_feedback + BUFFER_SIZE);
                c->sBypass.init(sr);
            }
        }

        inline uint32_t flanger::phase_to_int(float phase)
        {
            return double(0x100000000LL) * (phase / 360.0f);
        }

        void flanger::update_settings()
        {
            float out_gain          = pOutGain->value();
            bool bypass             = pBypass->value() >= 0.5f;
            float rate              = pRate->value() / fSampleRate;
            float feed_gain         = pFeedGain->value();
            float amount_gain       = pAmount->value();
            size_t lfo_type         = size_t(pLfoType->value());

            sReset.submit(pReset->value());

            nOldDepthMin            = nDepthMin;
            nDepthMin               = dspu::millis_to_samples(fSampleRate, pDepthMin->value());
            nOldDepth               = nDepth;
            nDepth                  = dspu::millis_to_samples(fSampleRate, pDepth->value());
            nOldPhaseStep           = nPhaseStep;
            nPhaseStep              = double(0x100000000LL) * rate;
            nInitPhase              = phase_to_int(pInitPhase->value());
            nOldFeedDelay           = nFeedDelay;
            nFeedDelay              = dspu::millis_to_samples(fSampleRate, pFeedDelay->value());
            fOldFeedGain            = fFeedGain;
            fFeedGain               = (pFeedPhase->value() >= 0.5f) ? -feed_gain : feed_gain;
            fOldDryGain             = fDryGain;
            fDryGain                = pDry->value() * out_gain;
            fOldWetGain             = fWetGain;
            fWetGain                = pWet->value() * out_gain;
            fAmount                 = (pSignalPhase->value() >= 0.5f) ? -amount_gain : amount_gain;

            // Update the LFO information
            if (lfo_type != nLfoType)
            {
                nLfoType                = lfo_type;
                pLfoFunc                = all_lfo_functions[lfo_type];
                bSyncLfo                = true;

                // Update the mesh contents
                float k                 = 1.0f / (meta::flanger::LFO_MESH_SIZE - 1);
                for (size_t i=0; i<meta::flanger::LFO_MESH_SIZE; ++i)
                {
                    float phase             = i * k;
                    vLfoPhase[i]            = phase * 360.0f;
                    vLfoMesh[i]             = pLfoFunc(phase);
                }
            }

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Store the parameters for each processor
                c->nOldPhaseShift       = c->nPhaseShift;
                c->nPhaseShift          = (i > 0) ? phase_to_int(pPhaseDiff->value()) : 0;

                // Update processors
                c->sBypass.set_bypass(bypass);
            }
        }

        float flanger::lfo_triangular(float phase)
        {
            return (phase < 0.5f) ? phase * 2.0f : (1.0f - phase) * 2.0f;
        }

        float flanger::lfo_sine(float phase)
        {
            return 0.5f - 0.5f * cosf(2.0f * M_PI * phase);
        }

        float flanger::lfo_step_sine(float phase)
        {
            if ((phase >= 0.25f) && (phase < 0.75f))
            {
                phase -= 0.25f;
                return 0.75f - 0.25f * cosf(4.0f * M_PI * phase);
            }

            return 0.25f - 0.25f * cosf(4.0f * M_PI * phase);
        }

        float flanger::lfo_cubic(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;

            return phase * phase * (12.0f - 16.0f * phase);
        }

        float flanger::lfo_step_cubic(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;

            phase      -= 0.25f;
            return 0.5f + 32.0f * phase * phase * phase;
        }

        float flanger::lfo_parabolic(float phase)
        {
            phase -= 0.5f;
            return 1.0f - 4.0f * phase * phase;
        }

        float flanger::lfo_rev_parabolic(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;

            return 4.0f * phase * phase;
        }

        float flanger::lfo_logarithmic(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;
            return logf(1.0f + 198.0f *phase) * REV_LN100;
        }

        float flanger::lfo_rev_logarithmic(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;
            return 1.0f - logf(100.0f - 198.0f * phase) * REV_LN100;
        }

        float flanger::lfo_sqrt(float phase)
        {
            phase      -= 0.5f;
            return sqrtf(1.0f - 4.0f * phase * phase);
        }

        float flanger::lfo_rev_sqrt(float phase)
        {
            if (phase >= 0.5f)
                phase          -= 1.0f;
            return 1.0f - sqrtf(1.0f - 4.0f * phase * phase);
        }

        float flanger::lfo_circular(float phase)
        {
            if (phase < 0.25f)
                return 0.5f - sqrtf(0.25f - 4.0f * phase * phase);

            if (phase > 0.75f)
            {
                phase          -= 1.0f;
                return 0.5f - sqrtf(0.25f - 4.0f * phase * phase);
            }

            phase      -= 0.5f;
            return 0.5f + sqrtf(0.25f - 4.0f * phase * phase);
        }

        float flanger::lfo_rev_circular(float phase)
        {
            if (phase >= 0.5f)
                phase   = 1.0f - phase;

            phase -= 0.25f;
            return (phase < 0.0f) ?
                sqrtf(0.25f - 4.0f * phase * phase) :
                1.0f - sqrtf(0.25f - 4.0f * phase * phase);
        }

        inline float flanger::lerp(float o_value, float n_value, float k)
        {
            return o_value + (n_value - o_value) * k;
        }

        inline int32_t flanger::ilerp(int32_t o_value, int32_t n_value, float k)
        {
            return o_value + (n_value - o_value) * k;
        }

        void flanger::process(size_t samples)
        {
            // Reset phase if phase request is pending
            if (sReset.pending())
            {
                nPhase                  = nInitPhase;
                sReset.commit();
            }

            // Perform the routing
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                c->vIn                  = c->pIn->buffer<float>();
                c->vOut                 = c->pOut->buffer<float>();

                // Measure the input level
                c->pInLevel->set_value(dsp::abs_max(c->vIn, samples));
            }

            for (size_t offset=0; offset<samples; )
            {
                uint32_t to_do          = lsp_min(samples - offset, BUFFER_SIZE);
                float k_to_do           = 1.0f / float(to_do);
                uint32_t phase          = nPhase;

                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];
                    phase                   = nPhase;

                    // Apply the flanging effect
                    for (size_t i=0; i<to_do; ++i)
                    {
                        float s                 = i * k_to_do;
                        float c_phase           = (phase + ilerp(c->nOldPhaseShift, c->nPhaseShift, s)) * PHASE_COEFF;
                        size_t c_shift          =
                            ilerp(nOldDepthMin, nDepthMin, s) +
                            ilerp(nOldDepth, nDepth, s) * pLfoFunc(c_phase);
                        size_t c_fbshift        =
                            c_shift +
                            ilerp(nOldFeedDelay, nFeedDelay, s);

                        float c_sample          = c->vIn[i];
                        c->sRing.append(c_sample);

                        float c_dsample         = c->sRing.get(c_shift);
                        float c_fbsample        = c->sFeedback.get(c_fbshift);
                        float c_rsample         = c_dsample + c_fbsample * lerp(fOldFeedGain, fFeedGain, s);

                        c->vBuffer[i]           =
                            c_sample +
                            c_rsample * lerp(fOldAmount, fAmount, s);

                        c->sFeedback.append(c_rsample);

                        phase                  += ilerp(nOldPhaseStep, nPhaseStep, s);
                    }

                    // Apply Dry/Wet and measure output level
                    dsp::lramp1(c->vBuffer, fOldWetGain, fWetGain, to_do);
                    dsp::lramp_add2(c->vBuffer, c->vIn, fOldDryGain, fDryGain, to_do);
                    c->pOutLevel->set_value(dsp::abs_max(c->vBuffer, to_do));

                    // Apply bypass
                    c->sBypass.process(c->vOut, c->vIn, c->vBuffer, to_do);

                    // Move pointers
                    c->vIn                 += to_do;
                    c->vOut                += to_do;
                }

                // Commit values
                nPhase              = phase;
                nOldDepthMin        = nDepthMin;
                nOldDepth           = nDepth;
                nOldPhaseStep       = nPhaseStep;
                fOldAmount          = fAmount;
                fOldFeedGain        = fFeedGain;
                nOldFeedDelay       = nFeedDelay;
                fOldDryGain         = fDryGain;
                fOldWetGain         = fWetGain;

                offset                 += to_do;
            }

            // Output information about phase for each channel
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                float phase             = (nPhase + c->nPhaseShift) * PHASE_COEFF;

                c->pPhase->set_value(phase * 360.0f);
                c->pLfoShift->set_value(pLfoFunc(phase));
            }

            // Need to synchronize LFO mesh?
            if (bSyncLfo)
            {
                plug::mesh_t *lfo       = (pLfoMesh != NULL) ? pLfoMesh->buffer<plug::mesh_t>() : NULL;
                if ((lfo != NULL) && (lfo->isEmpty()))
                {
                    dsp::copy(lfo->pvData[0], vLfoPhase, meta::flanger::LFO_MESH_SIZE);
                    dsp::copy(lfo->pvData[1], vLfoMesh, meta::flanger::LFO_MESH_SIZE);
                    lfo->data(2, meta::flanger::LFO_MESH_SIZE);

                    bSyncLfo = false;
                }
            }

            // Request the inline display for redraw
            if (pWrapper != NULL)
                pWrapper->query_display_draw();
        }

        void flanger::ui_activated()
        {
            bSyncLfo        = true;
        }

        bool flanger::inline_display(plug::ICanvas *cv, size_t width, size_t height)
        {
            // TODO
            return false;
        }

        void flanger::dump(dspu::IStateDumper *v) const
        {
            // TODO
        }

    } /* namespace plugins */
} /* namespace lsp */


