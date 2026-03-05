#include "config.h"
#include "nvs.h"
#include "esp_log.h"

#define NVS_NAMESPACE "stewart"
#define NVS_KEY       "cfg"

#define DEFAULT_KP      1.0f
#define DEFAULT_KI      0.0f
#define DEFAULT_KD      0.5f
#define DEFAULT_NEUTRAL 95.0f

static const char *TAG = "config";

static void config_set_defaults(platform_config_t *cfg)
{
    cfg->kp = DEFAULT_KP;
    cfg->ki = DEFAULT_KI;
    cfg->kd = DEFAULT_KD;
    for (int i = 0; i < SERVO_COUNT; i++)
        cfg->neutral_angles[i] = DEFAULT_NEUTRAL;
}

esp_err_t config_load(platform_config_t *cfg)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err != ESP_OK) {
        config_set_defaults(cfg);
        ESP_LOGI(TAG, "No saved config, using defaults");
        goto log;
    }

    size_t size = sizeof(platform_config_t);
    err = nvs_get_blob(handle, NVS_KEY, cfg, &size);
    nvs_close(handle);

    if (err != ESP_OK || size != sizeof(platform_config_t)) {
        config_set_defaults(cfg);
        ESP_LOGI(TAG, "Invalid saved config, using defaults");
    } else {
        ESP_LOGI(TAG, "Config loaded from NVS");
    }

log:
    ESP_LOGI(TAG, "Kp=%.2f Ki=%.2f Kd=%.2f Neutral=[%.1f, %.1f, %.1f]",
            cfg->kp, cfg->ki, cfg->kd,
            cfg->neutral_angles[0], cfg->neutral_angles[1], cfg->neutral_angles[2]);
    return ESP_OK;
}

esp_err_t config_save(const platform_config_t *cfg)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, NVS_KEY, cfg, sizeof(platform_config_t));
    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);

    if (err == ESP_OK)
        ESP_LOGI(TAG, "Config saved to NVS");
    else
        ESP_LOGE(TAG, "Failed to save config: %s", esp_err_to_name(err));

    return err;
}
