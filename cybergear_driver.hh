#ifndef CYBER_GEAR_DRIVER_H
#define CYBER_GEAR_DRIVER_H

#include "cybergear_can_interface.hh"
#include "cybergear_driver_defs.hh"

// Define the maximum number of motor parameters
#define MAX_MOTOR_PARAMETERS 20

/**
 * @brief Motor status structure
 */
struct MotorStatus {
    unsigned long stamp_usec;  //< timestamp
    uint8_t motor_id;          //!< motor ID
    float position;            //!< motor position (-4pi to 4pi)
    float velocity;            //!< motor velocity (-30rad/s to 30rad/s)
    float effort;              //!< motor effort (-12Nm to 12Nm)
    uint16_t raw_position;     //!< raw position for sync data
    uint16_t raw_velocity;     //!< raw velocity for sync data
    uint16_t raw_effort;       //!< raw effort for sync data
};

/**
 * @brief Cybergear driver class
 */
class CybergearDriver {
public:
    CybergearDriver();
    CybergearDriver(uint8_t master_can_id, uint8_t target_can_id);
    ~CybergearDriver();

    void init(CybergearCanInterface *can, uint16_t wait_response_time_usec = 0);
    void init_motor(uint8_t run_mode);
    void enable_motor();
    void reset_motor();
    void set_run_mode(uint8_t run_mode);
    void motor_control(float position, float speed, float torque, float kp, float kd);
    void set_limit_speed(float speed);
    void set_limit_current(float current);
    void set_limit_torque(float torque);
    void set_position_kp(float kp);
    void set_velocity_kp(float kp);
    void set_velocity_ki(float ki);
    void get_mech_position();
    void get_mech_velocity();
    void get_vbus();
    void get_rotation();
    void set_position_ref(float position);
    void set_speed_ref(float speed);
    void set_current_ref(float current);
    void dump_motor_param();
    bool process_packet();
    bool update_motor_status(unsigned long id, const uint8_t *data, unsigned long len);

private:
    CybergearCanInterface *can_;
    uint8_t master_can_id_;
    uint8_t target_can_id_;
    uint16_t wait_response_time_usec_;
    uint8_t run_mode_;
    unsigned long send_count_;
    uint8_t receive_buffer_[64];
    MotorStatus motor_status_;

    void send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t *data);
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(uint16_t x, float x_min, float x_max);
};

#endif  // CYBER_GEAR_DRIVER_H
