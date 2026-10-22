#ifndef PRIVATE_CONFIG_H
#define PRIVATE_CONFIG_H

#include <stdint.h>

// Copy to private_config.h and replace with the team's local values.
// Do not commit private_config.h.
static const uint8_t PRE_SHARED_KEY[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define COMMUNITY_ACCESS_CREDENTIAL "REPLACE_WITH_LOCAL_COMMUNITY_PASSCODE"
#define ADMIN_DASHBOARD_USERNAME "REPLACE_WITH_ADMIN_USERNAME"
#define ADMIN_DASHBOARD_PASSWORD "REPLACE_WITH_LONG_RANDOM_ADMIN_PASSWORD"

#error "Replace example values in private_config.h before building."

#endif
