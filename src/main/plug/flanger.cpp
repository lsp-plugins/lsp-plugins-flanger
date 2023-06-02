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

            fDepthMin       = 0.0f;
            fDepth          = 0.0f;
            nPhaseStep      = 0;
            fDryGain        = 0.0f;
            fWetGain        = 0.0f;

            pBypass         = NULL;
            pDepthMin       = NULL;
            pDepth          = NULL;
            pRate           = NULL;
            pAmount         = NULL;
            pInitPhase      = NULL;
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
            size_t alloc            = szof_channels + buf_sz;

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

                c->nInitPhase           = 0;
                c->nPhase               = 0;

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
            pBypass              = TRACE_PORT(ports[port_id++]);

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

            fDepthMin               = dspu::millis_to_samples(fSampleRate, pDepthMin->value());
            fDepth                  = dspu::millis_to_samples(fSampleRate, pDepth->value());
            nPhaseStep              = double(0x100000000LL) * rate;
            fDryGain                = pDry->value() * out_gain;
            fWetGain                = pWet->value() * out_gain;

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Store the parameters for each processor
                c->nInitPhase           = nPhaseStep * (pInitPhase->value() / 360.0f);

                // Update processors
                c->sBypass.set_bypass(bypass);
            }
        }

        void flanger::process(size_t samples)
        {
        }

        void flanger::dump(dspu::IStateDumper *v) const
        {
            // TODO
        }

    } /* namespace plugins */
} /* namespace lsp */


