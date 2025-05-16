#include "Protocol.h"
// #include "MotorControl.h"
#include "ServoController.h"
#include "SensorPolling.h"

Protocol& Protocol::getInstance() {
    static Protocol _instance;
    return _instance;
}

void Protocol::on_new_request() {
    uint32_t task_notify = TASK_NOTIFY_RX_DATA_SIGNAL;
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(task_handle, task_notify, eSetBits, &task_woken);
}

void servo_control_request_handle(
    const uint8_t* request_data, uint32_t request_data_size, uint8_t* response_data,
    uint32_t& response_data_size) {
    if (request_data_size != sizeof(protocol_request_servo)) {
        int32_t status = 0;
        protocol_request_servo request = *(protocol_request_servo*)request_data;
        ServoController& SC = ServoController::getInstance();
        status |= SC.set_angle(request.left_servo_angle, request.right_servo_angle);
        protocol_response_servo response = {.status = status};
        memcpy(response_data, &response, sizeof(protocol_response_servo));
        response_data_size = sizeof(protocol_response_servo);
    }
}

void Protocol::init() {
    BaseType_t status = xTaskCreate(startTaskImpl, "protocol_task", 512, this, 3, &task_handle);
    if (status == pdTRUE) {
        bind(SERVO_TARGET_ANGLES_MSG_TAG, servo_control_request_handle);

        communication_unit.set_rx_complete_callback(Protocol::on_new_request);
        communication_unit.init();

        vTaskResume(task_handle);
    } else {
        printf("Error сreating protocol task\n");
    }
}

void Protocol::task() {
    uint32_t task_notify = 0;
    while (true) {
        if (xTaskNotifyWait(0x00, TASK_NOTIFY_RX_DATA_SIGNAL, &task_notify, portMAX_DELAY)
            == pdTRUE) {
            if (task_notify == TASK_NOTIFY_RX_DATA_SIGNAL) {
                uint32_t id = 0;
                uint8_t request_data[64];
                uint8_t response_data[64];
                uint32_t request_data_size = 0;
                uint32_t response_data_size = 0;
                communication_unit.get_message(id, request_data, request_data_size);
                int32_t tag = (int32_t)id - REQUEST_BASE_ID_OFFSET;
                if ((tag >= 0) && (tag < CALLBACKS_MAX_NUM) && (request_callback[tag]) != nullptr) {
                    request_callback[tag](
                        request_data, request_data_size, response_data, response_data_size);
                }
            }
            task_notify = 0;
        }
    }
}

void Protocol::bind(uint32_t tag, request_callback_t callback) {
    if (tag < CALLBACKS_MAX_NUM) {
        request_callback[tag + REQUEST_BASE_ID_OFFSET] = callback;
    }
}

void Protocol::transmit_encoders_speed(float left_speed, float right_speed) {
    uint8_t msg_buff[8] = {0};
    memcpy(&(msg_buff[0]), &left_speed, sizeof(left_speed));
    memcpy(&(msg_buff[4]), &right_speed, sizeof(right_speed));
    communication_unit.transmit(ENCODERS_SPEED_MSG_TAG, msg_buff, 8);
}

void Protocol::transmit_servo_angles(float left_angle, float right_angle) {
    uint8_t msg_buff[8] = {0};
    memcpy(&(msg_buff[0]), &left_angle, sizeof(left_angle));
    memcpy(&(msg_buff[4]), &right_angle, sizeof(right_angle));
    communication_unit.transmit(SERVO_MEASURED_ANGLES_MSG_TAG, msg_buff, 8);
}

void Protocol::transmit_magn_data(uint16_t x_axis, uint16_t y_axis, uint16_t z_axis) {
    uint8_t msg_buff[6] = {0};
    memcpy(&(msg_buff[0]), &x_axis, sizeof(x_axis));
    memcpy(&(msg_buff[2]), &y_axis, sizeof(y_axis));
    memcpy(&(msg_buff[4]), &z_axis, sizeof(z_axis));
    communication_unit.transmit(MAGN_Z_MSG_TAG, msg_buff, 6);
}