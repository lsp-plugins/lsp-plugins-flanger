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

#include <lsp-plug.in/plug-fw/meta/ports.h>
#include <lsp-plug.in/shared/meta/developers.h>
#include <private/meta/flanger.h>

#define LSP_PLUGINS_FLANGER_VERSION_MAJOR       1
#define LSP_PLUGINS_FLANGER_VERSION_MINOR       0
#define LSP_PLUGINS_FLANGER_VERSION_MICRO       0

#define LSP_PLUGINS_FLANGER_VERSION  \
    LSP_MODULE_VERSION( \
        LSP_PLUGINS_FLANGER_VERSION_MAJOR, \
        LSP_PLUGINS_FLANGER_VERSION_MINOR, \
        LSP_PLUGINS_FLANGER_VERSION_MICRO  \
    )

namespace lsp
{
    namespace meta
    {
        //-------------------------------------------------------------------------
        // Plugin metadata

        static const port_t flanger_mono_ports[] =
        {
            // Input and output audio ports
            PORTS_MONO_PLUGIN,

            BYPASS,
            CONTROL("dmin", "Min Depth", U_MSEC, flanger::DEPTH_MIN),
            CONTROL("depth", "Depth", U_MSEC, flanger::DEPTH),
            CONTROL("rate", "Rate", U_HZ, flanger::RATE),
            AMP_GAIN10("amount", "The overall amount of the effect", 0.5f),
            CYC_CONTROL("iphase", "Initial Phase", U_DEG, flanger::PHASE),
            DRY_GAIN(0.0f),
            WET_GAIN(1.0f),
            OUT_GAIN,

            METER_GAIN("min", "Input gain", GAIN_AMP_P_48_DB),
            METER_GAIN("mout", "Output gain", GAIN_AMP_P_48_DB),

            PORTS_END
        };

        static const port_t flanger_stereo_ports[] =
        {
            PORTS_STEREO_PLUGIN,

            BYPASS,
            CONTROL("dmin", "Min Depth", U_MSEC, flanger::DEPTH_MIN),
            CONTROL("depth", "Depth", U_MSEC, flanger::DEPTH),
            CONTROL("rate", "Rate", U_HZ, flanger::RATE),
            AMP_GAIN10("amount", "The overall amount of the effect", 0.5f),
            CYC_CONTROL("iphase", "Initial Phase", U_DEG, flanger::PHASE),
            DRY_GAIN(0.0f),
            WET_GAIN(1.0f),
            OUT_GAIN,

            METER_GAIN("min_l", "Input gain left",  GAIN_AMP_P_48_DB),
            METER_GAIN("mout_l", "Output gain left",  GAIN_AMP_P_48_DB),
            METER_GAIN("min_r", "Input gain right",  GAIN_AMP_P_48_DB),
            METER_GAIN("mout_r", "Output gain right", GAIN_AMP_P_48_DB),

            PORTS_END
        };

        static const int plugin_classes[]       = { C_FLANGER, -1 };
        static const int clap_features_mono[]   = { CF_AUDIO_EFFECT, CF_FLANGER, CF_MONO, -1 };
        static const int clap_features_stereo[] = { CF_AUDIO_EFFECT, CF_FLANGER, CF_STEREO, -1 };

        const meta::bundle_t flanger_bundle =
        {
            "flanger",
            "Plugin Template",
            B_UTILITIES,
            "", // TODO: provide ID of the video on YouTube
            "" // TODO: write plugin description, should be the same to the english version in 'bundles.json'
        };

        const plugin_t flanger_mono =
        {
            "Flanger Mono",
            "Flanger Mono",
            "F1M",
            &developers::v_sadovnikov,
            "flanger_mono",
            LSP_LV2_URI("flanger_mono"),
            LSP_LV2UI_URI("flanger_mono"),
            "lf1m",
            LSP_LADSPA_FLANGER_BASE + 0,
            LSP_LADSPA_URI("flanger_mono"),
            LSP_CLAP_URI("flanger_mono"),
            LSP_PLUGINS_FLANGER_VERSION,
            plugin_classes,
            clap_features_mono,
            E_DUMP_STATE,
            flanger_mono_ports,
            "effects/flanger.xml",
            NULL,
            mono_plugin_port_groups,
            &flanger_bundle
        };

        const plugin_t flanger_stereo =
        {
            "Flanger Stereo",
            "Flanger Stereo",
            "F1S",
            &developers::v_sadovnikov,
            "flanger_stereo",
            LSP_LV2_URI("flanger_stereo"),
            LSP_LV2UI_URI("flanger_stereo"),
            "lf1s",
            LSP_LADSPA_FLANGER_BASE + 1,
            LSP_LADSPA_URI("flanger_stereo"),
            LSP_CLAP_URI("flanger_stereo"),
            LSP_PLUGINS_FLANGER_VERSION,
            plugin_classes,
            clap_features_stereo,
            E_DUMP_STATE,
            flanger_stereo_ports,
            "effects/flanger.xml",
            NULL,
            stereo_plugin_port_groups,
            &flanger_bundle
        };
    } /* namespace meta */
} /* namespace lsp */



