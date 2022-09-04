#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

#ifndef AP_EFI_SCRIPTING_ENABLED
#define AP_EFI_SCRIPTING_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_EFI_SCRIPTING_ENABLED

class AP_EFI_Scripting : public AP_EFI_Backend {
public:
    using AP_EFI_Backend::AP_EFI_Backend;

    void update() override;

    void handle_scripting(const EFI_State &efi_state) override;
};
#endif // AP_EFI_SCRIPTING_ENABLED
