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
#include <lsp-plug.in/shared/id_colors.h>

#include <private/plugins/flanger.h>

/* The size of temporary buffer for audio processing */
static constexpr size_t     BUFFER_SIZE             = 0x600;
static constexpr uint32_t   PHASE_MAX               = 0x80000000;
static constexpr uint32_t   PHASE_MASK              = PHASE_MAX - 1;
static constexpr float      PHASE_COEFF             = 1.0f / float(PHASE_MAX);
static constexpr float      REV_LN100               = 0.5f / M_LN10;

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
        dspu::lfo::function_t flanger::all_lfo_functions[] =
        {
            dspu::lfo::triangular,
            dspu::lfo::sine,
            dspu::lfo::step_sine,
            dspu::lfo::cubic,
            dspu::lfo::step_cubic,
            dspu::lfo::parabolic,
            dspu::lfo::rev_parabolic,
            dspu::lfo::logarithmic,
            dspu::lfo::rev_logarithmic,
            dspu::lfo::sqrt,
            dspu::lfo::rev_sqrt,
            dspu::lfo::circular,
            dspu::lfo::rev_circular,
            NULL
        };

        dspu::over_mode_t flanger::all_oversampling_modes[] =
        {
            dspu::over_mode_t::OM_NONE,
            dspu::over_mode_t::OM_LANCZOS_2X16BIT,
            dspu::over_mode_t::OM_LANCZOS_2X24BIT,
            dspu::over_mode_t::OM_LANCZOS_3X16BIT,
            dspu::over_mode_t::OM_LANCZOS_3X24BIT,
            dspu::over_mode_t::OM_LANCZOS_4X16BIT,
            dspu::over_mode_t::OM_LANCZOS_4X24BIT,
            dspu::over_mode_t::OM_LANCZOS_6X16BIT,
            dspu::over_mode_t::OM_LANCZOS_6X24BIT,
            dspu::over_mode_t::OM_LANCZOS_8X16BIT,
            dspu::over_mode_t::OM_LANCZOS_8X24BIT
        };

        inline float flanger::lerp(float o_value, float n_value, float k)
        {
            return o_value + (n_value - o_value) * k;
        }

        inline float flanger::qlerp(float o_value, float n_value, float k)
        {
            return o_value * sqrtf(1.0f - k) + n_value * sqrtf(k);
        }

        inline int32_t flanger::ilerp(int32_t o_value, int32_t n_value, float k)
        {
            return o_value + (n_value - o_value) * k;
        }

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

            nOldDepthMin    = meta::flanger::DEPTH_MIN_DFL;
            nDepthMin       = meta::flanger::DEPTH_MIN_DFL;
            nOldDepth       = meta::flanger::DEPTH_DFL;
            nDepth          = meta::flanger::DEPTH_DFL;
            nInitPhase      = 0;
            nPhase          = meta::flanger::PHASE_DFL;
            nOldPhaseStep   = 0;
            nPhaseStep      = 0;
            nCrossfade      = 0;
            fCrossfade      = PHASE_COEFF;
            pCrossfadeFunc  = qlerp;
            fOldAmount      = 0.0f;
            fAmount         = 0.0f;
            fOldFeedGain    = 0.0f;
            fFeedGain       = 0.0f;
            nOldFeedDelay   = 0;
            nFeedDelay      = 0;
            fOldInGain      = 0.0f;
            fInGain         = 0.0f;
            fOldDryGain     = 0.0f;
            fDryGain        = 0.0f;
            fOldWetGain     = 0.0f;
            fWetGain        = 0.0f;
            bMidSide        = false;
            bCustomLfo      = false;
            bMono           = false;

            pBypass         = NULL;
            pMono           = NULL;
            pRate           = NULL;
            pFraction       = NULL;
            pTempo          = NULL;
            pTempoSync      = NULL;
            pTimeMode       = NULL;
            pCrossfade      = NULL;
            pCrossfadeType  = NULL;
            pInitPhase      = NULL;
            pPhaseDiff      = NULL;
            pReset          = NULL;

            pMsSwitch       = NULL;
            pDepthMin       = NULL;
            pDepth          = NULL;
            pSignalPhase    = NULL;
            pAmount         = NULL;
            pOversampling   = NULL;
            pFeedOn         = NULL;
            pFeedGain       = NULL;
            pFeedDelay      = NULL;
            pFeedPhase      = NULL;
            pInGain         = NULL;
            pDry            = NULL;
            pWet            = NULL;
            pOutGain        = NULL;

            pIDisplay       = NULL;
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
                (
                    buf_sz +                // channel_t::vBuffer
                    mesh_buf_sz             // channel_t::vLfoMesh
                ) * nChannels;

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

            for (size_t i=0; i < nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Construct in-place DSP processors
                c->sBypass.construct();
                c->sDelay.construct();
                c->sRing.construct();
                c->sFeedback.construct();
                c->sOversampler.construct();
                c->sOversampler.init();

                c->nOldPhaseShift       = 0;
                c->nPhaseShift          = 0;
                c->nLfoType             = -1;
                c->nLfoPeriod           = -1;
                c->fLfoArg[0]           = 1.0f;
                c->fLfoArg[1]           = 0.0f;
                c->pLfoFunc             = NULL;
                c->fOutPhase            = 0.0f;
                c->fOutShift            = 0.0f;
                c->bSyncLfo             = true;

                c->vIn                  = NULL;
                c->vOut                 = NULL;
                c->vBuffer              = reinterpret_cast<float *>(ptr);
                ptr                    += buf_sz;
                c->vLfoMesh             = reinterpret_cast<float *>(ptr);
                ptr                    += mesh_buf_sz;

                c->pIn                  = NULL;
                c->pOut                 = NULL;

                c->pPhase               = NULL;
                c->pLfoType             = NULL;
                c->pLfoPeriod           = NULL;
                c->pLfoShift            = NULL;
                c->pLfoMesh             = NULL;
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
            if (nChannels > 1)
                pMono               = TRACE_PORT(ports[port_id++]);
            pRate               = TRACE_PORT(ports[port_id++]);
            pFraction           = TRACE_PORT(ports[port_id++]);
            TRACE_PORT(ports[port_id++]);   // Skip denominator
            pTempo              = TRACE_PORT(ports[port_id++]);
            pTempoSync          = TRACE_PORT(ports[port_id++]);
            pTimeMode           = TRACE_PORT(ports[port_id++]);
            pCrossfade          = TRACE_PORT(ports[port_id++]);
            pCrossfadeType      = TRACE_PORT(ports[port_id++]);
            vChannels[0].pLfoType   = TRACE_PORT(ports[port_id++]);
            vChannels[0].pLfoPeriod = TRACE_PORT(ports[port_id++]);
            if (nChannels > 1)
            {
                vChannels[1].pLfoType   = TRACE_PORT(ports[port_id++]);
                vChannels[1].pLfoPeriod = TRACE_PORT(ports[port_id++]);
            }
            pInitPhase          = TRACE_PORT(ports[port_id++]);
            if (nChannels > 1)
                pPhaseDiff          = TRACE_PORT(ports[port_id++]);
            pReset              = TRACE_PORT(ports[port_id++]);
            vChannels[0].pLfoMesh   = TRACE_PORT(ports[port_id++]);
            if (nChannels > 1)
                vChannels[1].pLfoMesh   = TRACE_PORT(ports[port_id++]);

            if (nChannels > 1)
                pMsSwitch           = TRACE_PORT(ports[port_id++]);
            pDepthMin           = TRACE_PORT(ports[port_id++]);
            pDepth              = TRACE_PORT(ports[port_id++]);
            pSignalPhase        = TRACE_PORT(ports[port_id++]);
            pAmount             = TRACE_PORT(ports[port_id++]);
            pOversampling       = TRACE_PORT(ports[port_id++]);
            pFeedOn             = TRACE_PORT(ports[port_id++]);
            pFeedGain           = TRACE_PORT(ports[port_id++]);
            pFeedDelay          = TRACE_PORT(ports[port_id++]);
            pFeedPhase          = TRACE_PORT(ports[port_id++]);
            pInGain             = TRACE_PORT(ports[port_id++]);
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

            // Fill LFO phase data
            float phase_k           = 360.0f / (meta::flanger::LFO_MESH_SIZE - 1);
            for (size_t i=0; i<meta::flanger::LFO_MESH_SIZE; ++i)
                vLfoPhase[i]            = i * phase_k;
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
                    c->sDelay.destroy();
                    c->sRing.destroy();
                    c->sFeedback.destroy();
                    c->sOversampler.destroy();
                }
                vChannels   = NULL;
            }

            vBuffer     = NULL;

            // Destroy inline display buffer
            if (pIDisplay != NULL)
            {
                pIDisplay->destroy();
                pIDisplay   = NULL;
            }

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
                c->sBypass.init(sr);
                c->sDelay.init(BUFFER_SIZE*2);
                c->sRing.init(max_delay * meta::flanger::OVERSAMPLING_MAX + BUFFER_SIZE*2);
                c->sFeedback.init(max_feedback * meta::flanger::OVERSAMPLING_MAX + BUFFER_SIZE*2);
                c->sOversampler.set_sample_rate(sr);
            }
        }

        inline uint32_t flanger::phase_to_int(float phase)
        {
            return float(PHASE_MAX) * (phase / 360.0f);
        }

        void flanger::update_settings()
        {
            // Update oversampling settings
            dspu::over_mode_t omode = all_oversampling_modes[size_t(pOversampling->value())];
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                if (c->sOversampler.mode() != omode)
                {
                    c->sOversampler.set_mode(omode);
                    c->sOversampler.set_filtering(false);
                    c->sOversampler.update_settings();

                    c->sDelay.set_delay(c->sOversampler.latency());
                    c->sDelay.clear();
                    c->sDelay.clear();
                    c->sRing.clear();
                    c->sFeedback.clear();
                }
            }
            size_t oversampling     = vChannels[0].sOversampler.get_oversampling();
            size_t latency          = vChannels[0].sOversampler.latency();

            // Update state of the 'reset' trigger
            sReset.submit(pReset->value());

            // Pre-compute several attributes
            float in_gain           = pInGain->value();
            float out_gain          = pOutGain->value();
            bool bypass             = pBypass->value() >= 0.5f;
            size_t srate            = fSampleRate * oversampling;
            bool fb_on              = pFeedOn->value() >= 0.5f;
            float feed_gain         = (fb_on) ? pFeedGain->value() : 0.0f;
            float amount_gain       = pAmount->value();
            bool mid_side           = (pMsSwitch != NULL) ? pMsSwitch->value() >= 0.5f : false;
            float crossfade         = pCrossfade->value() * 0.01f;

            // Compute LFO rate
            float rate              = pRate->value();
            if (pTimeMode->value() >= 1.0f)
            {
                // Use tempo instead of rate
                float tempo             = (pTempoSync->value() >= 0.5f) ? pWrapper->position()->beatsPerMinute : pTempo->value();
                rate                    =
                    lsp_limit(
                        dspu::time_signature_to_frequency(pFraction->value(), tempo),
                        meta::flanger::RATE_MIN,
                        meta::flanger::RATE_MAX);
            }
            rate                   /= srate;

            // Update common parameters
            nOldDepthMin            = nDepthMin;
            nDepthMin               = dspu::millis_to_samples(srate, pDepthMin->value());
            nOldDepth               = nDepth;
            nDepth                  = dspu::millis_to_samples(srate, pDepth->value());
            nOldPhaseStep           = nPhaseStep;
            nPhaseStep              = float(PHASE_MAX) * rate;
            nInitPhase              = (phase_to_int(pInitPhase->value()) - nPhaseStep * latency) & PHASE_MASK;
            nOldFeedDelay           = nFeedDelay;
            nFeedDelay              = dspu::millis_to_samples(srate, pFeedDelay->value());
            nCrossfade              = float(PHASE_MAX) * crossfade * 2;
            fCrossfade              = PHASE_COEFF * (1.0f - crossfade);
            pCrossfadeFunc          = (int(pCrossfadeType->value()) == 0) ? lerp : qlerp;
            fOldFeedGain            = fFeedGain;
            fFeedGain               = (pFeedPhase->value() >= 0.5f) ? -feed_gain : feed_gain;
            fOldInGain              = fInGain;
            fInGain                 = in_gain;
            fOldDryGain             = fDryGain;
            fDryGain                = pDry->value() * out_gain;
            fOldWetGain             = fWetGain;
            fWetGain                = pWet->value() * out_gain;
            fAmount                 = (pSignalPhase->value() >= 0.5f) ? -amount_gain : amount_gain;

            bool custom_lfo         = false;

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Update LFO preferences
                size_t lfo_type         = size_t(c->pLfoType->value());
                size_t lfo_period       = size_t(c->pLfoPeriod->value());
                if (i > 0)
                {
                    custom_lfo              = lfo_type > 0;
                    if (!custom_lfo)
                        lfo_period              = vChannels[0].nLfoPeriod;
                    lfo_type                = (custom_lfo) ? lfo_type - 1 : vChannels[0].nLfoType;
                }

                if ((lfo_type != c->nLfoType) || (lfo_period != c->nLfoPeriod))
                {
                    c->nLfoType             = lfo_type;
                    c->nLfoPeriod           = lfo_period;
                    c->pLfoFunc             = all_lfo_functions[lfo_type];
                    c->bSyncLfo             = true;

                    // Select the function coefficients
                    switch (lfo_period)
                    {
                        case meta::flanger::OSC_FIRST:
                            c->fLfoArg[0]       = 0.5f;
                            c->fLfoArg[1]       = 0.0f;
                            break;
                        case meta::flanger::OSC_LAST:
                            c->fLfoArg[0]       = 0.5f;
                            c->fLfoArg[1]       = 0.5f;
                            break;
                        case meta::flanger::OSC_FULL:
                        default:
                            c->fLfoArg[0]       = 1.0f;
                            c->fLfoArg[1]       = 0.0f;
                            break;
                    }

                    // Update LFO image
                    float k                 = c->fLfoArg[0] / (meta::flanger::LFO_MESH_SIZE - 1);
                    if (c->pLfoFunc != NULL)
                    {
                        for (size_t i=0; i<meta::flanger::LFO_MESH_SIZE; ++i)
                            c->vLfoMesh[i]          = c->pLfoFunc(i * k + c->fLfoArg[1]);
                    }
                    else
                        for (size_t i=0; i<meta::flanger::LFO_MESH_SIZE; ++i)
                            c->vLfoMesh[i]          = 0.0f;
                }

                // For Mid/Side switch change, clear the buffers
                if (mid_side != bMidSide)
                {
                    c->sRing.clear();
                    c->sFeedback.clear();
                }

                // Store the parameters for each processor
                c->nOldPhaseShift       = c->nPhaseShift;
                c->nPhaseShift          = (i > 0) ? phase_to_int(pPhaseDiff->value()) : 0;

                // Update processors
                c->sBypass.set_bypass(bypass);
            }

            bMidSide                = mid_side;
            bCustomLfo              = custom_lfo;
            bMono                   = (pMono != NULL) ? pMono->value() >= 0.5f : false;

            // Update latency
            set_latency(latency);
        }

        bool flanger::set_position(const plug::position_t *pos)
        {
            return pos->beatsPerMinute != pWrapper->position()->beatsPerMinute;
        }

        void flanger::process(size_t samples)
        {
            // Reset phase if phase request is pending
            if (sReset.pending())
            {
                nPhase                  = nInitPhase;
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c            = &vChannels[i];
                    c->sRing.clear();
                    c->sFeedback.clear();
                }
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

            size_t oversampling     = vChannels[0].sOversampler.get_oversampling();
            size_t max_buf_samples  = BUFFER_SIZE / oversampling;

            for (size_t offset=0; offset<samples; )
            {
                uint32_t to_do          = lsp_min(samples - offset, max_buf_samples);
                uint32_t phase          = nPhase;

                // Convert to Mid/Side if needed
                if ((bMidSide) && (nChannels > 1))
                {
                    dsp::lr_to_ms(
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        vChannels[0].vIn,
                        vChannels[1].vIn,
                        to_do
                    );

                    dsp::lramp2(vChannels[0].vBuffer, vChannels[0].vBuffer, fOldInGain, fInGain, to_do);
                    dsp::lramp2(vChannels[1].vBuffer, vChannels[1].vBuffer, fOldInGain, fInGain, to_do);
                }
                else
                {
                    dsp::lramp2(vChannels[0].vBuffer, vChannels[0].vIn, fOldInGain, fInGain, to_do);
                    if (nChannels > 1)
                        dsp::lramp2(vChannels[1].vBuffer, vChannels[1].vIn, fOldInGain, fInGain, to_do);
                }

                // Do audio processing
                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];
                    phase                   = nPhase;

                    // Apply oversampling and delay
                    uint32_t up_to_do       = to_do * oversampling;
                    float k_up_to_do        = 1.0f / float(up_to_do);
                    c->sOversampler.upsample(c->vBuffer, c->vBuffer, to_do);

                    // Apply the flanging effect
                    if (c->pLfoFunc != NULL)
                    {
                        for (size_t i=0; i<up_to_do; ++i)
                        {
                            float s                 = i * k_up_to_do;
                            uint32_t i_phase        = (phase + ilerp(c->nOldPhaseShift, c->nPhaseShift, s)) & PHASE_MASK;
                            float o_phase           = i_phase * fCrossfade;
                            float c_phase           = o_phase * c->fLfoArg[0] + c->fLfoArg[1];
                            float c_func            = c->pLfoFunc(c_phase);

                            float c_sample          = c->vBuffer[i];
                            c->sRing.append(c_sample);
                            c->fOutPhase            = o_phase;
                            c->fOutShift            = c_func;

                            size_t c_shift          =
                                ilerp(nOldDepthMin, nDepthMin, s) +
                                ilerp(nOldDepth, nDepth, s) * c_func;
                            size_t c_fbshift        =
                                c_shift +
                                ilerp(nOldFeedDelay, nFeedDelay, s);
                            float c_dsample         = c->sRing.get(c_shift);
                            float c_fbsample        = c->sFeedback.get(c_fbshift);

                            // Perform cross-fade if required
                            if (i_phase < nCrossfade)
                            {
                                float mix               = float(i_phase) / float(nCrossfade);
                                i_phase                 = i_phase + PHASE_MAX;
                                c_phase                 = i_phase * fCrossfade * c->fLfoArg[0] + c->fLfoArg[1];
                                c_shift                 =
                                    ilerp(nOldDepthMin, nDepthMin, s) +
                                    ilerp(nOldDepth, nDepth, s) * c->pLfoFunc(c_phase);
                                c_fbshift               =
                                    c_shift +
                                    ilerp(nOldFeedDelay, nFeedDelay, s);
                                c_dsample               = pCrossfadeFunc(c->sRing.get(c_shift), c_dsample, mix);
                                c_fbsample              = pCrossfadeFunc(c->sFeedback.get(c_fbshift), c_fbsample, mix);
                            }

                            // Do the final processing
                            float c_rsample         = c_dsample + c_fbsample * lerp(fOldFeedGain, fFeedGain, s);
                            c->vBuffer[i]           =
                                c_sample +
                                c_rsample * lerp(fOldAmount, fAmount, s);
                            c->sFeedback.append(c_rsample);

                            // Update the phase
                            phase                   = (phase + ilerp(nOldPhaseStep, nPhaseStep, s)) & PHASE_MASK;
                        }
                    }
                    else
                    {
                        // Do nothing, just update phase
                        for (size_t i=0; i<to_do; ++i)
                        {
                            float s                 = i * k_up_to_do;
                            uint32_t i_phase        = (phase + ilerp(c->nOldPhaseShift, c->nPhaseShift, s)) & PHASE_MASK;
                            float o_phase           = i_phase * fCrossfade;
                            c->fOutPhase            = o_phase;
                            phase                   = (phase + ilerp(nOldPhaseStep, nPhaseStep, s)) & PHASE_MASK;
                        }

                        c->fOutShift            = 0.0f;
                    }

                    // Perform downsampling
                    c->sOversampler.downsample(c->vBuffer, c->vBuffer, to_do);

                    // Update channel's phase shift
                    c->nOldPhaseShift       = c->nPhaseShift;
                }

                // Convert back to left-right if needed
                if ((bMidSide) && (nChannels > 1))
                {
                    dsp::ms_to_lr(
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        vChannels[0].vBuffer,
                        vChannels[1].vBuffer,
                        to_do
                    );
                }

                // Apply Dry/Wet and measure output level
                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];

                    // Apply latency compensation
                    c->sDelay.process(vBuffer, c->vIn, to_do);

                    // Mix dry/wet
                    dsp::lramp1(c->vBuffer, fOldWetGain, fWetGain, to_do);
                    dsp::lramp_add2(c->vBuffer, vBuffer, fOldDryGain, fDryGain, to_do);
                    c->pOutLevel->set_value(dsp::abs_max(c->vBuffer, to_do));
                }

                // Apply mono compatibility switch
                if ((nChannels > 1) && (bMono))
                {
                    dsp::lr_to_mid(vChannels[0].vBuffer, vChannels[0].vBuffer, vChannels[1].vBuffer, to_do);
                    dsp::copy(vChannels[1].vBuffer, vChannels[0].vBuffer, to_do);
                }

                // Apply bypass and update buffer pointers
                for (size_t nc=0; nc<nChannels; ++nc)
                {
                    channel_t *c            = &vChannels[nc];

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
                fOldInGain          = fInGain;
                fOldDryGain         = fDryGain;
                fOldWetGain         = fWetGain;

                offset             += to_do;
            }

            // Output information about phase for each channel
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->pPhase->set_value(c->fOutPhase * 360.0f);
                c->pLfoShift->set_value(c->fOutShift);

                // Need to synchronize LFO mesh?
                if (c->bSyncLfo)
                {
                    plug::mesh_t *lfo       = (c->pLfoMesh != NULL) ? c->pLfoMesh->buffer<plug::mesh_t>() : NULL;
                    if ((lfo != NULL) && (lfo->isEmpty()))
                    {
                        dsp::copy(lfo->pvData[0], vLfoPhase, meta::flanger::LFO_MESH_SIZE);
                        dsp::copy(lfo->pvData[1], c->vLfoMesh, meta::flanger::LFO_MESH_SIZE);
                        lfo->data(2, meta::flanger::LFO_MESH_SIZE);

                        c->bSyncLfo         = false;
                    }
                }
            }

            // Request the inline display for redraw
            if (pWrapper != NULL)
                pWrapper->query_display_draw();
        }

        void flanger::ui_activated()
        {
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];
                c->bSyncLfo             = true;
            }
        }

        bool flanger::inline_display(plug::ICanvas *cv, size_t width, size_t height)
        {
            // Check proportions
            if (height > width)
                height  = width;

            // Init canvas
            if (!cv->init(width, height))
                return false;
            width   = cv->width();
            height  = cv->height();

            // Clear background
            bool bypassing = vChannels[0].sBypass.bypassing();
            cv->set_color_rgb((bypassing) ? CV_DISABLED : CV_BACKGROUND);
            cv->paint();

            // Draw horizontal and vertical lines
            cv->set_line_width(1.0);
            cv->set_color_rgb((bypassing) ? CV_SILVER: CV_YELLOW, 0.5f);
            for (float i=1; i < 8; ++i)
            {
                float y = i * 45.0f;
                float x = i * 0.0125f;
                cv->line(0, y, width, y);
                cv->line(x, 0, x, height);
            }

            // Reuse display
            size_t count        = lsp_max(width, height);
            pIDisplay           = core::IDBuffer::reuse(pIDisplay, 2, count);
            core::IDBuffer *b   = pIDisplay;
            if (b == NULL)
                return false;

            static const uint32_t c_colors[] = {
                CV_MIDDLE_CHANNEL,
                CV_LEFT_CHANNEL, CV_RIGHT_CHANNEL,
                CV_MIDDLE_CHANNEL, CV_SIDE_CHANNEL
            };

            const uint32_t *colors  = &c_colors[0];
            size_t lines            = 1;
            if ((nChannels > 1) && (bCustomLfo))
            {
                colors  = (bMidSide) ? &c_colors[3] : &c_colors[1];
                lines   = 2;
            }

            bool aa = cv->set_anti_aliasing(true);
            lsp_finally { cv->set_anti_aliasing(aa); };

            cv->set_line_width(2);

            dsp::lramp_set1(b->v[1], 0.0f, height-1, count);

            for (size_t i=0; i<lines; ++i)
            {
                channel_t *c    = &vChannels[i];

                for (size_t j=0; j<count; ++j)
                {
                    size_t k        = (j*meta::flanger::LFO_MESH_SIZE)/count;
                    b->v[0][j]      = c->vLfoMesh[k] * width;
                }

                // Draw mesh
                uint32_t color = (bypassing || !(active())) ? CV_SILVER : colors[i];
                cv->set_color_rgb(color);
                cv->draw_lines(b->v[0], b->v[1], count);
            }

            // Draw dots with lines
            if (active())
            {
                colors  = (nChannels <= 1) ? &c_colors[0] :
                          (bMidSide) ? &c_colors[3] : &c_colors[1];
                cv->set_line_width(1);

                // Draw lines first
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];
                    cv->set_color_rgb(colors[i]);
                    float x = c->fOutShift * width;
                    cv->line(x, 0, x, height);
                }

                // Draw dots next
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];

                    uint32_t color = (bypassing) ? CV_SILVER : colors[i];
                    Color c1(color), c2(color);
                    c2.alpha(0.9);

                    float x = c->fOutShift * width;
                    float y = c->fOutPhase * height;

                    cv->radial_gradient(x, y, c1, c2, 12);
                    cv->set_color_rgb(0);
                    cv->circle(x, y, 4);
                    cv->set_color_rgb(color);
                    cv->circle(x, y, 3);
                }
            }

            return true;
        }

        void flanger::dump(dspu::IStateDumper *v) const
        {
            // TODO
        }

    } /* namespace plugins */
} /* namespace lsp */


