#include "app.h"

#include <memory>

#include "tim.h"
#include "i2c.h"
#include "fdcan.h"

#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/scalar/Integer32_1_0.h>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/motors/stepper/stepper.hpp>

EEPROM eeprom(&hi2c4);

class CustomSimpleStepper: public StepperMotorSimple {
public:
    using StepperMotorSimple::StepperMotorSimple; // inherit constructors

    HAL_StatusTypeDef init() override {
        // Override this method to customize config pins
        config.sd_mode.set();
        config.spi_mode.reset();

        config.cfg0.reset();
        config.cfg1.reset();

        config.cfg2.reset();
        config.cfg3.reset();
        config.cfg4.reset();

        config.cfg5.set();
        config.cfg6.set();

        return start_pwm();
    }
};

std::unique_ptr<CustomSimpleStepper> motor;

[[noreturn]] void app() {
    start_timers();
    start_cyphal();

    while (!eeprom.is_connected()) {
        eeprom.delay();
    }
    eeprom.delay();

    motor = std::make_unique<CustomSimpleStepper>(
        StepperSimpleConfig {
            .enable = GpioPin(DRV_EN_GPIO_Port, DRV_EN_Pin),
            .direction = GpioPin(REFL_DIR_GPIO_Port, REFL_DIR_Pin),
            .sd_mode = GpioPin(SD_MODE_GPIO_Port, SD_MODE_Pin),
            .spi_mode = GpioPin(SPI_MODE_GPIO_Port, SPI_MODE_Pin),
            .cfg0 = GpioPin(CFG0_GPIO_Port, CFG0_Pin),
            .cfg1 = GpioPin(CFG1_GPIO_Port, CFG1_Pin),
            .cfg2 = GpioPin(CFG2_GPIO_Port, CFG2_Pin),
            .cfg3 = GpioPin(CFG3_GPIO_Port, CFG3_Pin),
            .cfg4 = GpioPin(CFG4_GPIO_Port, CFG4_Pin),
            .cfg5 = GpioPin(CFG5_GPIO_Port, CFG5_Pin),
            .cfg6 = GpioPin(CFG6_GPIO_Port, CFG6_Pin),
            .step_channel = TIM_CHANNEL_1,
            .timer = &htim1
        }
    );
    motor->init();
    HAL_Delay(150);
    motor->set_pulse_freq(250, true);
    motor->start();

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    while(true) {
        cyphal_loop();
    }
}

TYPE_ALIAS(Integer32, uavcan_primitive_scalar_Integer32_1_0)
static constexpr CanardPortID FREQ_PORT = 5800;

void in_loop_reporting(millis current_t) {
}

class FreqSub: public AbstractSubscription<Integer32> {
public:
    FreqSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Integer32>(interface, port_id) {};
    void handler(const Integer32::Type& msg, CanardRxTransfer* _) override {
        motor->set_pulse_freq(abs(msg.value), msg.value > 0);
    }
};

ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<1>> registers_handler;
ReservedObject<FreqSub> freq_sub;

void setup_subscriptions() {
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    auto cyphal_interface = get_interface();
    const auto node_id = get_node_id();

    freq_sub.create(cyphal_interface, FREQ_PORT + node_id);
    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.stepper",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );
    registers_handler.create(
        std::array<RegisterDefinition, 1>{{
            {
                "motor.is_on",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse::Type& response
                ){
                    static bool value = false;
                    if (v_in._tag_ == 3) {
                        value = v_in.bit.value.bitpacked[0] == 1;
                    }
                    else {
                        // TODO: report error
                    }

                    motor->set_state(value);

                    response.persistent = true;
                    response._mutable = true;
                    v_out._tag_ = 3;
                    v_out.bit.value.bitpacked[0] = motor->is_on();
                    v_out.bit.value.count = 1;
                }
            }
        }},
        cyphal_interface
    );

    static FDCAN_FilterTypeDef sFilterConfig;
    uint32_t filter_index = 0;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        node_info_reader->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        registers_handler->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        freq_sub->make_filter(node_id)
    ))
}
