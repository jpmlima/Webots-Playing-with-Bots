#include <webots/motor.h>

#include <webots/robot.h>

#include <webots/distance_sensor.h>

#include <stdio.h>

#include <stdlib.h>

#include <webots/keyboard.h>

#include <webots/gps.h>

#define MAX_SPEED 47.6

#define NUMBER_OF_ULTRASONIC_SENSORS 3
static const char * ultrasonic_sensors_names[NUMBER_OF_ULTRASONIC_SENSORS] = {
    "front left ultrasonic sensor",
    "front ultrasonic sensor",
    "front right ultrasonic sensor"
};

#define NUMBER_OF_INFRARED_SENSORS 3
static const char * infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor"
};

#define NUMBER_OF_INFRARED_GROUND_SENSORS 4
static const char * infrared_sensors_ground_names[NUMBER_OF_INFRARED_GROUND_SENSORS] = {
    "ground left infrared sensor",
    "ground front left infrared sensor",
    "ground front right infrared sensor",
    "ground right infrared sensor"
};

int main(int argc, char ** argv) {
    wb_robot_init();

    int time_step = (int) wb_robot_get_basic_time_step();
    int i;

    //Activar GPS
    WbDeviceTag gps = wb_robot_get_device("global");
    wb_gps_enable(gps, time_step);

    // Activar teclado
    wb_keyboard_enable(time_step);
    // Activar infravermelhos de baixo
    WbDeviceTag infrared_sensors_ground[4];
    for (i = 0; i < 4; i++) {

        infrared_sensors_ground[i] = wb_robot_get_device(infrared_sensors_ground_names[i]);
        wb_distance_sensor_enable(infrared_sensors_ground[i], time_step);
    }

    // Activar ultrasons
    WbDeviceTag ultrasonic_sensors[3];

    for (i = 0; i < 3; ++i) {
        ultrasonic_sensors[i] = wb_robot_get_device(ultrasonic_sensors_names[i]);
        wb_distance_sensor_enable(ultrasonic_sensors[i], time_step);
    }

    // Activar infravermelhos
    WbDeviceTag infrared_sensors[3];
    for (i = 0; i < 3; ++i) {
        infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
        wb_distance_sensor_enable(infrared_sensors[i], time_step);
    }

    // Ativar motores e definir posição
    WbDeviceTag left_motor, right_motor;
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

  
    int last_display_second = 0;
    int count = 0;
    
    // Ciclo principal
    while (wb_robot_step(time_step) != -1) {

        int display_second = (int) wb_robot_get_time();
        const double X = wb_gps_get_values(gps)[0];
        const double Y = wb_gps_get_values(gps)[1];
        const double Z = wb_gps_get_values(gps)[2];

        if (X > 0 && Y > -0.000007) {

            if (display_second != last_display_second) {
                last_display_second = display_second;

                printf("time = %d [s]\n", display_second);

                printf("---------------\n");
                for (i = 0; i < 3; ++i)
                    printf("| \033[1;31m ultrasonic | %s = %f [m] \033[0m\n", ultrasonic_sensors_names[i], wb_distance_sensor_get_value(ultrasonic_sensors[i]));
                for (i = 0; i < 3; ++i)
                    printf("| \033[1;33m  infrared  | %s = %f [m]  \033[0m\n", infrared_sensors_names[i], wb_distance_sensor_get_value(infrared_sensors[i]));
                for (i = 0; i < 1; ++i) {
                    printf("| \033[0;32m    GPS     | X=%f[m] Y=%f[m] Z=%f[m] \033[0m\n", X, Y, Z);
                    printf("---------------\n");
                }

                if (display_second % 7 == 0) {
                    wb_motor_set_velocity(left_motor, -0.085 * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, 0.085 * MAX_SPEED);
                    ++count;
                } else {
                    wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);

                }
                if (count == 4) {
                    wb_motor_set_velocity(left_motor, 0);
                    wb_motor_set_velocity(right_motor, 0);
                }

            }

        }
        if (X < 0 && Y > -0.000007) {
            
            int key = wb_keyboard_get_key();
            if (key < 0) {

                if (display_second != last_display_second) {
                    last_display_second = display_second;

                    printf("time = %d [s]\n", display_second);

                    printf("---------------\n");
                    for (i = 0; i < 3; ++i)
                        printf("| \033[1;31m ultrasonic | %s = %f [m] \033[0m\n", ultrasonic_sensors_names[i], wb_distance_sensor_get_value(ultrasonic_sensors[i]));
                    for (i = 0; i < 3; ++i)
                        printf("| \033[1;33m  infrared  | %s = %f [m]  \033[0m\n", infrared_sensors_names[i], wb_distance_sensor_get_value(infrared_sensors[i]));
                    for (i = 0; i < 1; ++i) {
                        printf("| \033[0;32m    GPS     | X=%f[m] Y=%f[m] Z=%f[m] \033[0m\n", X, Y, Z);
                        printf("---------------\n");
                    }

                }

                double speed_offset = 0.2 * (MAX_SPEED - 0.09 * wb_distance_sensor_get_value(infrared_sensors[1]));
                double speed_delta = 0.04 * wb_distance_sensor_get_value(infrared_sensors[0]) - 0.04 * wb_distance_sensor_get_value(infrared_sensors[2]);
                wb_motor_set_velocity(left_motor, speed_offset + speed_delta);
                wb_motor_set_velocity(right_motor, speed_offset - speed_delta);

                if (wb_distance_sensor_get_value(ultrasonic_sensors[1]) != 0 || wb_distance_sensor_get_value(ultrasonic_sensors[0]) != 0 || wb_distance_sensor_get_value(ultrasonic_sensors[2]) != 0) {

                    wb_motor_set_velocity(left_motor, speed_offset + speed_delta);
                    wb_motor_set_velocity(right_motor, speed_offset - speed_delta);
                }
            } else {
                //Controlo de teclado
                switch (key) {

                case WB_KEYBOARD_UP:
                    printf("UP");
                    wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);

                    break;

                case WB_KEYBOARD_DOWN:
                    printf("DOWN");
                    wb_motor_set_velocity(left_motor, -0.1 * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, -0.1 * MAX_SPEED);

                    break;

                case WB_KEYBOARD_RIGHT:
                    printf("RIGHT");
                    wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, -0.1 * MAX_SPEED);

                    break;

                case WB_KEYBOARD_LEFT:
                    printf("LEFT");
                    wb_motor_set_velocity(left_motor, -0.1 * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);

                    break;

                case WB_KEYBOARD_HOME:

                    printf("STOP");
                    wb_motor_set_velocity(left_motor, 0);
                    wb_motor_set_velocity(right_motor, 0);
                }
            }
        }

        //Seguir a linha
        if (Y < -0.000007) {

            if (wb_distance_sensor_get_value(infrared_sensors_ground[1]) > 270 && wb_distance_sensor_get_value(infrared_sensors_ground[2]) > 270) {
                if (wb_distance_sensor_get_value(infrared_sensors_ground[0]) < 270 || wb_distance_sensor_get_value(infrared_sensors_ground[1]) < 400) {
                    wb_motor_set_velocity(left_motor, -0.1 * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);

                }
                if (wb_distance_sensor_get_value(infrared_sensors_ground[3]) < 270 || wb_distance_sensor_get_value(infrared_sensors_ground[2]) < 400) {
                    wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, -0.1 * MAX_SPEED);
                }
            } else {
                wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
                wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);

            }

        }

    }

    wb_robot_cleanup();
    return EXIT_SUCCESS;

}