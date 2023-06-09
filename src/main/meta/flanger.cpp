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
        #define LFO_LIST \
            { "Triangular",             "flanger.osc.triangular"            }, \
            { "Sine",                   "flanger.osc.sine"                  }, \
            { "Stepped Sine",           "flanger.osc.stepped_sine"          }, \
            { "Cubic",                  "flanger.osc.cubic"                 }, \
            { "Stepped Cubic",          "flanger.osc.stepped_cubic"         }, \
            { "Parabolic",              "flanger.osc.parabolic"             }, \
            { "Reverse Parabolic",      "flanger.osc.reverse_parabolic"     }, \
            { "Logarithmic",            "flanger.osc.logarithmic"           }, \
            { "Reverse Logarithmic",    "flanger.osc.reverse_logarithmic"   }, \
            { "Square Root",            "flanger.osc.square_root"           }, \
            { "Reverse Square Root",    "flanger.osc.reverse_square_root"   }, \
            { "Circular",               "flanger.osc.circular"              }, \
            { "Reverse Circular",       "flanger.osc.reverse_circular"      }, \
            { "Off",                    "flanger.osc.off"                   }, \
            { NULL, NULL }

        static const port_item_t oscillator_functions[] =
        {
            LFO_LIST
        };

        static const port_item_t additional_oscillator_functions[] =
        {
            { "Same",                   "flanger.osc.same"                  },
            LFO_LIST
        };

        static const port_item_t oscillator_periods[] =
        {
            { "Full",                   "flanger.period.full"               }, \
            { "First",                  "flanger.period.first"              }, \
            { "Last",                   "flanger.period.last"               }, \
            { NULL, NULL }
        };

        static const port_item_t crossfade_type[] =
        {
            { "Linear",                 "fade.linear"      },
            { "Const Power",            "fade.const_power" },
            { NULL, NULL }
        };

        static const port_item_t rate_type[] =
        {
            { "Rate",                   "flanger.rate.rate"                 },
            { "Tempo",                  "flanger.rate.tempo"                },
            { NULL, NULL }
        };

        static const port_item_t oversampling_mode[] =
        {
            { "None",                   "flanger.oversampler.none"          },
            { "2X Medium",              "flanger.oversampler.2x_medium"     },
            { "2X High",                "flanger.oversampler.2x_high"       },
            { "3X Medium",              "flanger.oversampler.3x_medium"     },
            { "3X High",                "flanger.oversampler.3x_high"       },
            { "4X Medium",              "flanger.oversampler.4x_medium"     },
            { "4X High",                "flanger.oversampler.4x_high"       },
            { "6X Medium",              "flanger.oversampler.6x_medium"     },
            { "6X High",                "flanger.oversampler.6x_high"       },
            { "8X Medium",              "flanger.oversampler.8x_medium"     },
            { "8X High",                "flanger.oversampler.8x_high"       },
            { NULL,                     NULL}
        };

        //-------------------------------------------------------------------------
        // Plugin metadata
        static const port_t flanger_mono_ports[] =
        {
            // Input and output audio ports
            PORTS_MONO_PLUGIN,

            BYPASS,

            CONTROL("rate", "Rate", U_HZ, flanger::RATE),
            CONTROL("frac", "Time fraction", U_BAR, flanger::FRACTION),
            CONTROL("denom", "Time fraction denominator", U_BAR, flanger::DENOMINATOR),
            CONTROL("tempo", "Tempo", U_BPM, flanger::TEMPO),
            SWITCH("sync", "Tempo sync", 0.0f),
            COMBO("time", "Time computing method", 0, rate_type),
            CONTROL("xfade", "Crossfade", U_PERCENT, flanger::CROSSFADE),
            COMBO("xtype", "Crossfade Type", 1, crossfade_type),
            COMBO("type", "LFO type", 0, oscillator_functions),
            COMBO("period", "LFO period", 0, oscillator_periods),
            CYC_CONTROL("iphase", "Initial phase", U_DEG, flanger::PHASE),
            TRIGGER("reset", "Reset phase to initial"),
            MESH("lfo", "LFO graph", 2, flanger::LFO_MESH_SIZE),

            CONTROL("dmin", "Min depth", U_MSEC, flanger::DEPTH_MIN),
            CONTROL("depth", "Depth", U_MSEC, flanger::DEPTH),
            SWITCH("sphase", "Signal phase switch", 0.0f),
            AMP_GAIN10("amount", "The overall amount of the effect", GAIN_AMP_M_6_DB),
            COMBO("ovs", "Oversampling", 0, oversampling_mode),
            SWITCH("fb_on", "Feedback on", 0),
            CONTROL("fgain", "Feedback gain", U_GAIN_AMP, flanger::FEEDBACK_GAIN),
            CONTROL("fdelay", "Feedback delay", U_MSEC, flanger::FEEDBACK_DELAY),
            SWITCH("fphase", "Feedback phase switch", 0.0f),

            IN_GAIN,
            DRY_GAIN(0.0f),
            WET_GAIN(1.0f),
            OUT_GAIN,

            METER("clph", "Current LFO phase", U_DEG, flanger::PHASE),
            METER("clsh", "Current LFO shift", U_NONE, flanger::SHIFT),

            METER_GAIN("min", "Input gain", GAIN_AMP_P_48_DB),
            METER_GAIN("mout", "Output gain", GAIN_AMP_P_48_DB),

            PORTS_END
        };

        static const port_t flanger_stereo_ports[] =
        {
            PORTS_STEREO_PLUGIN,

            BYPASS,

            SWITCH("mono", "Test for mono compatibility", 0),
            CONTROL("rate", "Rate", U_HZ, flanger::RATE),
            CONTROL("frac", "Time fraction", U_BAR, flanger::FRACTION),
            CONTROL("denom", "Time fraction denominator", U_BAR, flanger::DENOMINATOR),
            CONTROL("tempo", "Tempo", U_BPM, flanger::TEMPO),
            SWITCH("sync", "Tempo sync", 0.0f),
            COMBO("time", "Time computing method", 0, rate_type),
            CONTROL("xfade", "Crossfade", U_PERCENT, flanger::CROSSFADE),
            COMBO("xtype", "Crossfade Type", 1, crossfade_type),
            COMBO("type", "LFO type", 0, oscillator_functions),
            COMBO("period", "LFO period", 0, oscillator_periods),
            COMBO("atype", "Additional LFO type", 0, additional_oscillator_functions),
            COMBO("aperiod", "Additional LFO period", 0, oscillator_periods),
            CYC_CONTROL("iphase", "Initial phase", U_DEG, flanger::PHASE),
            CYC_CONTROL("dphase", "Phase difference between left and right", U_DEG, flanger::PHASE),
            TRIGGER("reset", "Reset phase to initial"),
            MESH("lfo", "LFO graph", 2, flanger::LFO_MESH_SIZE),
            MESH("alfo", "Additional LFO graph", 2, flanger::LFO_MESH_SIZE),

            SWITCH("ms", "Mid/Side mode switch", 0.0f),
            CONTROL("dmin", "Min depth", U_MSEC, flanger::DEPTH_MIN),
            CONTROL("depth", "Depth", U_MSEC, flanger::DEPTH),
            SWITCH("sphase", "Signal phase switch", 0.0f),
            AMP_GAIN10("amount", "The overall amount of the effect", GAIN_AMP_0_DB),
            COMBO("ovs", "Oversampling", 0, oversampling_mode),
            SWITCH("fb_on", "Feedback on", 0),
            CONTROL("fgain", "Feedback gain", U_GAIN_AMP, flanger::FEEDBACK_GAIN),
            CONTROL("fdelay", "Feedback delay", U_MSEC, flanger::FEEDBACK_DELAY),
            SWITCH("fphase", "Feedback phase switch", 0.0f),

            IN_GAIN,
            DRY_GAIN(0.0f),
            WET_GAIN(1.0f),
            OUT_GAIN,

            METER("clph_l", "Current LFO phase left", U_DEG, flanger::PHASE),
            METER("clsh_l", "Current LFO shift left", U_NONE, flanger::SHIFT),
            METER_GAIN("min_l", "Input gain left",  GAIN_AMP_P_48_DB),
            METER_GAIN("mout_l", "Output gain left",  GAIN_AMP_P_48_DB),

            METER("clph_r", "Current LFO phase right", U_DEG, flanger::PHASE),
            METER("clsh_r", "Current LFO shift right", U_NONE, flanger::SHIFT),
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
            E_DUMP_STATE | E_INLINE_DISPLAY,
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
            E_DUMP_STATE | E_INLINE_DISPLAY,
            flanger_stereo_ports,
            "effects/flanger.xml",
            NULL,
            stereo_plugin_port_groups,
            &flanger_bundle
        };
    } /* namespace meta */
} /* namespace lsp */



