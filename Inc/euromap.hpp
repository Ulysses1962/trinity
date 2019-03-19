/**
  @author Sergey Rotanov aka Ulysses
  @date   March 2019
  @brief  EUROMAP API definitions
*/
#ifndef __EUROMAP_API__
#define __EUROMAP_API__

#include "main.h"
#include "vector"

using namespace std;

using SPI       = SPI_HandleTypeDef*;
using LINE_PORT = GPIO_TypeDef*;
using STATUS    = HAL_StatusTypeDef;

using EUROMAP_LINE = struct {
    LINE_PORT port;
    uint16_t line_number;
};

enum EMAP_STATE {INIT, DEVICE_INACTIVE, DEVICE_ALARM, MOULD_CLOSING, MOULD_AREA_IN_USE, EJECTOR_BACK, EJECTOR_FORWARD, CP_POS2, CP_POS1, MOULD_FULL_OPEN};
enum EMAP_COMMAND {MOULD_CLOSURE_ENABLED, MOULD_AREA_IS_FREE, MOULD_AREA_IS_IN_USE, DEVICE_EMGS, DEVICE_ENABLED, DEVICE_DISABLED, DEACTIVATE_EJECTOR, ACTIVATE_EJECTOR,
        CORE_PULLERS_ENABLED, CORE_PULLERS_DISABLED, MOVE_CORE_PULLERS_TO_POS2, MOVE_CORE_PULLERS_TO_POS1, ENABLE_MOULD_FULL_OPEN};

class Euromap
{
    uint8_t euromap_version;

    SPI out_port;
    vector<EUROMAP_LINE> lines {};

    uint16_t machine_state; 
    uint16_t control_word;
    EMAP_STATE state {INIT};

    static auto const EMAP_COMMAND_ACK              {0xf0f0};
    static auto const EMAP_COMMAND_NACK             {0x0f0f}; 

    // EUROMAP command fields masks
    static auto const FULL_MOLD_OPEN_ENA_MASK       {0x0001};
    static auto const CPP1_MOV_ENA_MASK 	        {0x0002};
    static auto const CPP2_MOV_ENA_MASK 	        {0x0004};
    static auto const EJECT_FORWARD_ENA_MASK        {0x0008};
    static auto const EJECT_BACK_ENA_MASK 	        {0x0010};
    static auto const DEVICE_OP_MODE_MASK 	        {0x0020};
    static auto const DEVICE_EMGS_MASK 		        {0x0040};
    static auto const MOLD_AREA_FREE_MASK 	        {0x0080};
    static auto const MOLD_CLOSE_ENA_MASK 	        {0x0100};

    // EUROMAP moulding machine state fields masks
    static auto const MACHINE_EMERGENCY_STOP_MASK 	{0x0001};
    static auto const MOULD_IN_OPEN_POS_MASK 		{0x0002};
    static auto const MACHINE_SAFETY_MASK 			{0x0004};
    static auto const EJECTOR_IN_BACK_POS_MASK 		{0x0008};
    static auto const EJECTOR_IN_FORWARD_POS_MASK 	{0x0010};
    static auto const MOULD_INTER_OPEN_POS_MASK 	{0x0020};
    static auto const MOULD_IS_CLOSED_MASK 			{0x0040};
    static auto const DEVICE_OPERATION_ENABLED_MASK {0x0080};
    static auto const MOULD_REJECT_MASK 			{0x0100};
    static auto const CORE_PULLERS_IN_POS2_MASK 	{0x0200};
    static auto const CORE_PULLERS_IN_POS1_MASK 	{0x0400};

public:
    Euromap(uint8_t version, SPI port, vector<EUROMAP_LINE>& lines);

    auto update_state() -> void;
    auto process_command() -> void;
    auto process_state() -> void;
};

extern "C" void EMAPServiceTask(void const* param);

#endif
