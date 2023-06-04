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
#define BUFFER_SIZE         0x1000U

static constexpr float PHASE_COEFF          = 1.0f / float(0x100000000LL);
static constexpr float REV_LN10             = 1.0f / M_LN10;

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
            flanger::lfo_cubic,
            flanger::lfo_parabolic,
            flanger::lfo_rev_parabolic,
            flanger::lfo_logarithmic,
            flanger::lfo_rev_logarithmic,
            flanger::lfo_sqrt,
            flanger::lfo_rev_sqrt
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

            nDepthMin       = 0;
            nDepth          = 0;
            nInitPhase      = 0;
            nPhase          = 0;
            nPhaseStep      = 0;
            pLfoFunc        = lfo_triangular;
            fAmount         = 0.0f;
            fFeedGain       = 0.0f;
            fDryGain        = 0.0f;
            fWetGain        = 0.0f;

            pBypass         = NULL;
            pRate           = NULL;
            pFunc           = NULL;
            pInitPhase      = NULL;
            pPhaseDiff      = NULL;
            pReset          = NULL;

            pDepthMin       = NULL;
            pDepth          = NULL;
            pAmount         = NULL;
            pFeedGain       = NULL;
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
            size_t buf_sz           = BUFFER_SIZE * sizeof(float);
            size_t alloc            = szof_channels + buf_sz + buf_sz * nChannels;

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;

            // Initialize pointers to channels and temporary buffer
            vChannels               = reinterpret_cast<channel_t *>(ptr);
            ptr                    += szof_channels;
            vBuffer                 = reinterpret_cast<float *>(ptr);
            ptr                    += buf_sz;

            for (size_t i=0; i < nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Construct in-place DSP processors
                c->sBypass.construct();
                c->sRing.construct();

                c->nPhaseShift          = 0;
                c->vBuffer              = reinterpret_cast<float *>(ptr);
                ptr                    += buf_sz;
                c->vIn                  = NULL;
                c->vOut                 = NULL;

                c->pIn                  = NULL;
                c->pOut                 = NULL;
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
            pFunc               = TRACE_PORT(ports[port_id++]);
            pInitPhase          = TRACE_PORT(ports[port_id++]);
            if (nChannels > 1)
                pPhaseDiff          = TRACE_PORT(ports[port_id++]);
            pReset              = TRACE_PORT(ports[port_id++]);
            pDepthMin           = TRACE_PORT(ports[port_id++]);
            pDepth              = TRACE_PORT(ports[port_id++]);
            pAmount             = TRACE_PORT(ports[port_id++]);
            pFeedGain           = TRACE_PORT(ports[port_id++]);
            pFeedPhase          = TRACE_PORT(ports[port_id++]);
            pDry                = TRACE_PORT(ports[port_id++]);
            pWet                = TRACE_PORT(ports[port_id++]);
            pOutGain            = TRACE_PORT(ports[port_id++]);

            // Bind ports for audio processing channels
//            lsp_trace("Binding channel ports");
//            for (size_t i=0; i<nChannels; ++i)
//            {
//                channel_t *c            = &vChannels[i];
//            }


            // Bind output meters
            lsp_trace("Binding output meters");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
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

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];
                c->sRing.init(max_delay + BUFFER_SIZE);
                c->sBypass.init(sr);
            }
        }

        void flanger::update_settings()
        {
            float out_gain          = pOutGain->value();
            bool bypass             = pBypass->value() >= 0.5f;
            float rate              = pRate->value() / fSampleRate;
            float feed_gain         = pFeedGain->value();

            sReset.submit(pReset->value());

            nDepthMin               = dspu::millis_to_samples(fSampleRate, pDepthMin->value());
            nDepth                  = dspu::millis_to_samples(fSampleRate, pDepth->value());
            nPhaseStep              = double(0x100000000LL) * rate;
            nInitPhase              = uint32_t(nPhaseStep * double(pInitPhase->value() / 360.0f));
            fFeedGain               = (pFeedPhase->value() >= 0.5f) ? -feed_gain : feed_gain;
            fDryGain                = pDry->value() * out_gain;
            fWetGain                = pWet->value() * out_gain;
            fAmount                 = pAmount->value();
            pLfoFunc                = all_lfo_functions[size_t(pFunc->value())];

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Store the parameters for each processor
                c->nPhaseShift          = (i > 0) ? nPhaseStep * (pPhaseDiff->value() / 360.0f) : 0;

                // Update processors
                c->sBypass.set_bypass(bypass);
            }
        }

        float flanger::lfo_triangular(float phase)
        {
            return (phase < 0.5f) ? phase * 2.0f : 1.0f - phase * 2.0f;
        }

        float flanger::lfo_sine(float phase)
        {
            return sinf(2.0f * M_PI * phase);
        }

        float flanger::lfo_cubic(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;

            return phase * phase * (12.0f - 16.0f * phase);
        }

        float flanger::lfo_parabolic(float phase)
        {
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
            return logf(1.0f + 18.0f *phase) * REV_LN10;
        }

        float flanger::lfo_rev_logarithmic(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;
            return 1.0f - logf(1.0f + 18.0f *phase) * REV_LN10;
        }

        float flanger::lfo_sqrt(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;
            return sqrtf(phase * 2.0f);
        }

        float flanger::lfo_rev_sqrt(float phase)
        {
            if (phase >= 0.5f)
                phase      = 1.0f - phase;
            return 1.0f - sqrtf(phase * 2.0f);
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
                size_t to_do            = lsp_min(samples - offset, BUFFER_SIZE);

                if (nChannels > 1)
                {
                    channel_t *l            = &vChannels[0];
                    channel_t *r            = &vChannels[1];

                    // Apply the flanging effect
                    for (size_t i=0; i<to_do; ++i)
                    {
                        float l_sample          = l->vIn[i];
                        float r_sample          = r->vIn[i];

                        l->sRing.append(l_sample);
                        r->sRing.append(r_sample);

                        float l_phase           = uint32_t(nPhase + l->nPhaseShift) * PHASE_COEFF;
                        float r_phase           = uint32_t(nPhase + r->nPhaseShift) * PHASE_COEFF;

                        size_t l_shift          = nDepthMin + pLfoFunc(l_phase) * nDepth;
                        size_t r_shift          = nDepthMin + pLfoFunc(r_phase) * nDepth;

                        float l_dsample         = l->sRing.get(l_shift);
                        float r_dsample         = r->sRing.get(r_shift);

                        l->vBuffer[i]           = l_sample + (l_dsample + l->fFeedback * fFeedGain) * fAmount;
                        r->vBuffer[i]           = r_sample + (r_dsample + r->fFeedback * fFeedGain) * fAmount;
                        l->fFeedback            = l_dsample;
                        r->fFeedback            = r_dsample;

                        nPhase                 += nPhaseStep;
                    }
                }
                else
                {
                    channel_t *c            = &vChannels[0];

                    // Apply the flanging effect
                    for (size_t i=0; i<to_do; ++i)
                    {
                        float c_sample          = c->vIn[i];

                        c->sRing.append(c_sample);

                        float c_phase           = uint32_t(nPhase + c->nPhaseShift) * PHASE_COEFF;

                        size_t c_shift          = nDepthMin + pLfoFunc(c_phase) * nDepth;

                        float c_dsample         = c->sRing.get(c_shift);
                        c->vBuffer[i]           = c_sample + (c_dsample + c->fFeedback * fFeedGain) * fAmount;
                        c->fFeedback            = c_dsample;

                        nPhase                 += nPhaseStep;
                    }
                }

                // Post-process flanger data
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c            = &vChannels[i];

                    c->pOutLevel->set_value(dsp::abs_max(c->vBuffer, to_do));
                    c->sBypass.process(c->vOut, c->vIn, c->vBuffer, to_do);

                    c->vIn                 += to_do;
                    c->vOut                += to_do;
                }

                offset                 += to_do;
            }
        }

        void flanger::dump(dspu::IStateDumper *v) const
        {
            // TODO
        }

    } /* namespace plugins */
} /* namespace lsp */


