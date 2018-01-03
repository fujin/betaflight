/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_rc.h"

controlRateConfig_t *currentControlRateProfile;


PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 0);

void pgResetFn_controlRateProfiles(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &controlRateConfig[i],
            .rcRate8 = 100,
            .rcYawRate8 = 100,
            .rcExpo8 = 0,
            .thrMid8 = 50,
            .thrExpo8 = 0,
            .dynThrPID = 10,
            .rcYawExpo8 = 0,
            .tpa_breakpoint = 1650,
            .rates[FD_ROLL] = 70,
            .rates[FD_PITCH] = 70,
            .rates[FD_YAW] = 70,
            .rfRatesEnabled = 0,
            .rfRate[FD_ROLL] = 400,
            .rfAcro[FD_ROLL] = 140,
            .rfExpo[FD_ROLL] = 50,
            .rfRate[FD_PITCH] = 400,
            .rfAcro[FD_PITCH] = 140,
            .rfExpo[FD_PITCH] = 50,
            .rfRate[FD_YAW] = 400,
            .rfAcro[FD_YAW] = 140,
            .rfExpo[FD_YAW] = 50
        );
    }
}

void setControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = controlRateProfileIndex;
        currentControlRateProfile = controlRateProfilesMutable(controlRateProfileIndex);
    }
}

void changeControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex >= CONTROL_RATE_PROFILE_COUNT) {
        controlRateProfileIndex = CONTROL_RATE_PROFILE_COUNT - 1;
    }
    setControlRateProfile(controlRateProfileIndex);
    generateThrottleCurve();
}

void copyControlRateProfile(const uint8_t dstControlRateProfileIndex, const uint8_t srcControlRateProfileIndex) {
    if ((dstControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT-1 && srcControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT-1)
        && dstControlRateProfileIndex != srcControlRateProfileIndex
    ) {
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex), controlRateProfilesMutable(srcControlRateProfileIndex), sizeof(controlRateConfig_t));
    }
}
