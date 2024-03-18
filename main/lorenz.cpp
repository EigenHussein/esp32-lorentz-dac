#include <stdio.h>
#include <math.h>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/dac_oneshot.h>

const double SIGMA = 10.0;
const double RHO = 28.0;
const double BETA = 8.0 / 3.0;
const double dt = 0.001;

void update_state(double *x, double *y, double *z)
{
    double dx = SIGMA * (*y - *x) * dt;
    double dy = (*x * (RHO - *z) - *y) * dt;
    double dz = (*x * *y - BETA * *z) * dt;

    *x += dx;
    *y += dy;
    *z += dz;
}

struct Maximas
{
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;

    Maximas()
    {
        x_min = 1000.0;
        x_max = -1000.0;
        y_min = 1000.0;
        y_max = -1000.0;
        z_min = 1000.0;
        z_max = -1000.0;
    }
};

Maximas get_region(double x0, double y0, double z0, int steps)
{
    Maximas Max = Maximas();

    for (int i = 0; i < steps; i++)
    {
        update_state(&x0, &y0, &z0);
        if (x0 > Max.x_max)
            Max.x_max = x0;
        if (x0 < Max.x_min)
            Max.x_min = x0;

        if (y0 > Max.y_max)
            Max.y_max = y0;
        if (y0 < Max.y_min)
            Max.y_min = y0;

        if (z0 > Max.z_max)
            Max.z_max = z0;
        if (z0 < Max.z_min)
            Max.z_min = z0;

        // printf("New state: {x: %f;y: %f;z: %f}\n", x0, y0, z0);
    }
    return Max;
}

extern "C" void app_main(void)
{
    dac_oneshot_handle_t dac1_handler, dac2_handler = {};

    dac_oneshot_config_t dac1{.chan_id = DAC_CHAN_0};
    dac_oneshot_config_t dac2{.chan_id = DAC_CHAN_1};

    dac_oneshot_new_channel(&dac1, &dac1_handler);
    dac_oneshot_new_channel(&dac2, &dac2_handler);

    Maximas region = get_region(1, 1, 1, 20000);
    printf("Found range\nX:(%f,%f)\nX:(%f,%f)\nX:(%f,%f)\n", region.x_min, region.x_max, region.y_min, region.y_max, region.z_min, region.z_max);

    double x_range = region.x_max - region.x_min;
    double y_range = region.y_max - region.y_min;
    double z_range = region.z_max - region.z_min;

    double x = 1.0, y = 1.0, z = 1.0;

    while (true)
    {
        update_state(&x, &y, &z);
        double normalized_x = (x - region.x_min) / x_range;
        double normalized_y = (y - region.y_min) / y_range;
        double normalized_z = (z - region.z_min) / z_range;
        dac_oneshot_output_voltage(dac1_handler, 254 * normalized_x);
        dac_oneshot_output_voltage(dac2_handler, 254 * normalized_z);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

